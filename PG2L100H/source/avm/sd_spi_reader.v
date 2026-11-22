//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 单时钟 SD v1/v2 SPI 初始化及 CMD17 单块读取器，按 CCS 选择块/字节寻址，输出可反压的 512 字节数据流。
// Module Name: sd_spi_reader
// File Name: source/avm/sd_spi_reader.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 单时钟 SD v1/v2 SPI 初始化及 CMD17 单块读取器，按 CCS 选择块/字节寻址，输出可反压的 512 字节数据流。
// i_start 仅在 o_ready 时接受；初始化低速分频与读取 DIV4 分频分离，token 使用墙钟超时并独立监控输出背压进度；同步低有效复位清致命错误。
//
// Dependencies: spi_byte_master。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-14
// Design Name: PG2L100H SD SPI read-only engine
// Module Name: sd_spi_reader
// Project Name: pg2l100h_sd_lut
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Single-clock SD v1/v2 SPI init and CMD17, one-byte elastic output.
//              i_start is accepted only with o_ready. Reset clears fatal errors.
//              C_*DIV >= 1, C_TIMEOUT_BYTES >= 1, C_INIT_RETRIES >= 1.
// Dependencies: spi_byte_master.v
// Revision: 0.03 - DIV4，token 墙钟超时及独立背压进度监控
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module sd_spi_reader
#(
	parameter C_INIT_DIV = 54,                  // 初始化阶段 SPI 半周期分频值
	parameter C_READ_DIV = 4,                  // 读取阶段 SPI 半周期分频值
	parameter C_POWER_WAIT = 270000,            // 上电等待的系统时钟周期数
	parameter C_TIMEOUT_BYTES = 65535,          // 仅 R1 响应等待的最大字节数（兼容旧参数名）
	parameter C_TOKEN_TIMEOUT_CYCLES = 28620000, // token 墙钟窗口：27 MHz 下 1.06 秒
	parameter C_BACKPRESSURE_TIMEOUT_CYCLES = 27000000, // 独立下游停顿窗口：1 秒
	parameter C_INIT_RETRIES = 4096             // ACMD41 初始化的最大重试次数
)
(
	input i_clk,                                // 单一系统时钟
	input i_rstn,                               // 低有效同步复位
	input i_start,                              // 仅空闲时接受的一拍启动请求
	input [31:0] i_lba,                         // 待读取的 512 字节逻辑扇区号
	output o_ready,                             // 空闲状态下可接受读取请求
	output o_init_done,                         // 初始化成功标志，复位前保持
	output [7:0] o_error,                       // 协议/超时错误码，0 表示无错误
	output [7:0] o_data,                        // 当前弹性输出字节
	output o_valid,                             // 输出字节有效，背压时保持
	input i_ready,                              // 下游接受当前输出字节
	output o_done,                              // 单块读取完成单拍脉冲
	output o_sd_clk,                            // SD SPI 时钟，由字节引擎生成
	output o_sd_mosi,                           // SD CMD/MOSI 输出
	input i_sd_miso,                            // SD D0/MISO 输入
	output o_sd_cs,                             // SD D3/CS 输出，低有效
	output [7:0] o_last_r1,                     // 最近有效的 R1 响应
	output [7:0] o_token,                       // 最近的非 FF 数据 token
	output o_ccs                                // OCR CCS，1 为 SDHC/SDXC 块寻址
);
	//-------------状态参数区域-------------//
	localparam [4:0] ST_POWER = 5'd0;           // 上电等待，SPI 保持空闲
	localparam [4:0] ST_CLOCK = 5'd1;           // CS 高电平下发出启动时钟字节
	localparam [4:0] ST_CLOCK_DONE = 5'd2;      // 统计已完成的启动时钟字节
	localparam [4:0] ST_PREP = 5'd3;            // 准备命令参数及命令前空闲字节
	localparam [4:0] ST_GAP_DONE = 5'd4;        // 拉低 CS，开始发送命令
	localparam [4:0] ST_CMD = 5'd5;             // 发送六字节 SPI 命令包
	localparam [4:0] ST_CMD_DONE = 5'd6;        // 推进命令包字节索引
	localparam [4:0] ST_R1 = 5'd7;              // 轮询首字节响应
	localparam [4:0] ST_R1_DONE = 5'd8;         // 检查响应有效位与等待超时
	localparam [4:0] ST_DISPATCH = 5'd9;        // 按命令检查 R1 并选择后续流程
	localparam [4:0] ST_EXTRA = 5'd10;          // 接收 R7 或 OCR 扩展响应字节
	localparam [4:0] ST_EXTRA_DONE = 5'd11;     // 拼接四字节扩展响应
	localparam [4:0] ST_EXTRA_CHECK = 5'd12;    // 检查 CMD8 回显及 OCR 电压/CCS
	localparam [4:0] ST_INIT_FINISH = 5'd13;    // 释放 CS 并完成初始化
	localparam [4:0] ST_IDLE = 5'd14;           // 等待可接受的单块读取请求
	localparam [4:0] ST_TOKEN = 5'd15;          // 轮询 CMD17 数据 token
	localparam [4:0] ST_TOKEN_DONE = 5'd16;     // 检查 FE token 与异常/超时
	localparam [4:0] ST_DATA = 5'd17;           // 接收一个数据字节
	localparam [4:0] ST_DATA_DONE = 5'd18;      // 把收到的字节写入弹性输出寄存器
	localparam [4:0] ST_DELIVER = 5'd19;        // 等待下游 ready/valid 握手
	localparam [4:0] ST_CRC = 5'd20;            // 接收并消耗块尾 CRC 字节
	localparam [4:0] ST_CRC_DONE = 5'd21;       // 统计两字节块尾 CRC
	localparam [4:0] ST_FINISH = 5'd22;         // 释放 CS 并产生块间空闲时钟
	localparam [4:0] ST_FINISH_DONE = 5'd23;    // 产生单拍读取完成信号
	localparam [4:0] ST_XFER = 5'd24;           // 发起一次 SPI 字节传输
	localparam [4:0] ST_WAIT = 5'd25;           // 等待 SPI 字节传输完成
	localparam [4:0] ST_ERROR = 5'd26;          // 保持错误状态，等待复位恢复

	//--------------状态机信号--------------//
	reg [4:0] state_current;
	reg [4:0] state_next;
	reg [4:0] state_return;

	//---------------计数信号---------------//
	reg [31:0] cnt_power;
	reg [3:0] cnt_power_byte;
	reg [2:0] cnt_command;
	reg [1:0] cnt_extra;
	reg [31:0] cnt_timeout;
	reg [31:0] cnt_token_cycles;
	reg [31:0] cnt_backpressure;
	wire flag_token_wait;
	reg [31:0] cnt_init_retry;
	reg [8:0] cnt_data;
	reg cnt_crc;

	//--------------寄存器信号--------------//
	reg [5:0] reg_command;
	reg [31:0] reg_argument;
	reg [31:0] reg_extra;
	reg [31:0] reg_lba;
	reg flag_v2;
	reg [7:0] reg_tx;

	//------------模块实例化信号------------//
	wire [7:0] spi_rx;
	wire spi_done;
	wire spi_busy;
	wire spi_start;
	wire [15:0] spi_div;

	//---------------输出信号---------------//
	reg init_done_o;
	reg [7:0] error_o;
	reg [7:0] data_o;
	reg valid_o;
	reg done_o;
	reg cs_o;
	reg [7:0] last_r1_o;
	reg [7:0] token_o;
	reg ccs_o;

	//---------------译码信号---------------//
	reg [7:0] dec_error;

	//-------------输出信号连线-------------//
	assign o_ready = (state_current == ST_IDLE);
	assign o_init_done = init_done_o;
	assign o_error = error_o;
	assign o_data = data_o;
	assign o_valid = valid_o;
	assign o_done = done_o;
	assign o_sd_cs = cs_o;
	assign o_last_r1 = last_r1_o;
	assign o_token = token_o;
	assign o_ccs = ccs_o;

	//-------------其他信号连线-------------//
	assign flag_token_wait = (state_current == ST_TOKEN) || (state_current == ST_TOKEN_DONE) ||
		(((state_current == ST_XFER) || (state_current == ST_WAIT)) && (state_return == ST_TOKEN_DONE));
	assign spi_start = (state_current == ST_XFER);
	assign spi_div = init_done_o ? C_READ_DIV : C_INIT_DIV;

	//--------------状态机区域--------------//
	// 三段式状态机：字节发起与等待完全分离，done 不会造成重复请求。
	always@(posedge i_clk)begin
		if(!i_rstn)begin
			state_current <= ST_POWER;
		end else begin
			state_current <= state_next;
		end
	end

	// 根据当前状态、SPI 响应与背压条件产生下一状态及错误码。
	always@(*)begin
		state_next = state_current;
		dec_error = 8'd0;
		case(state_current)
			ST_POWER:begin
				if(cnt_power >= C_POWER_WAIT)begin
					state_next = ST_CLOCK;
				end
			end
			ST_CLOCK, ST_PREP, ST_CMD, ST_R1, ST_EXTRA, ST_INIT_FINISH,
			ST_TOKEN, ST_DATA, ST_CRC, ST_FINISH:begin
				state_next = ST_XFER;
			end
			ST_XFER:begin
				state_next = ST_WAIT;
			end
			ST_WAIT:begin
				if(spi_done)begin
					state_next = state_return;
				end
			end
			ST_CLOCK_DONE:begin
				if(cnt_power_byte == 4'd9)begin
					state_next = ST_PREP;
				end else begin
					state_next = ST_CLOCK;
				end
			end
			ST_GAP_DONE:begin
				state_next = ST_CMD;
			end
			ST_CMD_DONE:begin
				if(cnt_command == 3'd5)begin
					state_next = ST_R1;
				end else begin
					state_next = ST_CMD;
				end
			end
			ST_R1_DONE:begin
				if(!spi_rx[7])begin
					state_next = ST_DISPATCH;
				end else if(cnt_timeout >= C_TIMEOUT_BYTES - 1)begin
					dec_error = 8'h01;
				end else begin
					state_next = ST_R1;
				end
			end
			ST_DISPATCH:begin
				case(reg_command)
					6'd0:begin
						if(last_r1_o == 8'h01)begin
							state_next = ST_PREP;
						end else begin
							dec_error = 8'h02;
						end
					end
					6'd8:begin
						if(last_r1_o == 8'h01)begin
							state_next = ST_EXTRA;
						end else if(last_r1_o == 8'h05)begin
							state_next = ST_PREP;
						end else begin
							dec_error = 8'h03;
						end
					end
					6'd55:begin
						if(last_r1_o == 8'h01 || last_r1_o == 8'h00)begin
							state_next = ST_PREP;
						end else begin
							dec_error = 8'h05;
						end
					end
					6'd41:begin
						if(last_r1_o == 8'h00)begin
							state_next = ST_PREP;
						end else if(last_r1_o != 8'h01)begin
							dec_error = 8'h06;
						end else if(cnt_init_retry >= C_INIT_RETRIES - 1)begin
							dec_error = 8'h07;
						end else begin
							state_next = ST_PREP;
						end
					end
					6'd58:begin
						if(last_r1_o == 8'h00)begin
							state_next = ST_EXTRA;
						end else begin
							dec_error = 8'h08;
						end
					end
					6'd16:begin
						if(last_r1_o == 8'h00)begin
							state_next = ST_INIT_FINISH;
						end else begin
							dec_error = 8'h0a;
						end
					end
					6'd17:begin
						if(last_r1_o == 8'h00)begin
							state_next = ST_TOKEN;
						end else begin
							dec_error = 8'h0b;
						end
					end
					default:begin
						dec_error = 8'hff;
					end
				endcase
			end
			ST_EXTRA_DONE:begin
				if(cnt_extra == 2'd3)begin
					state_next = ST_EXTRA_CHECK;
				end else begin
					state_next = ST_EXTRA;
				end
			end
			ST_EXTRA_CHECK:begin
				if(reg_command == 6'd8)begin
					if(reg_extra != 32'h000001aa)begin
						dec_error = 8'h04;
					end else begin
						state_next = ST_PREP;
					end
				end else begin
					// OCR power-up ready，电压窗覆盖 3.3V；旧版卡不得报告 CCS。
					if(!reg_extra[31] || !(|reg_extra[23:20]) || (!flag_v2 && reg_extra[30]))begin
						dec_error = 8'h09;
					end else if(reg_extra[30])begin
						state_next = ST_INIT_FINISH;
					end else begin
						state_next = ST_PREP;
					end
				end
			end
			ST_IDLE:begin
				if(i_start)begin
					if(!ccs_o && (|i_lba[31:23]))begin
						dec_error = 8'h0e;
					end else begin
						state_next = ST_PREP;
					end
				end
			end
			ST_TOKEN_DONE:begin
				if(spi_rx == 8'hfe)begin
					state_next = ST_DATA;
				end else if(spi_rx != 8'hff)begin
					dec_error = 8'h0d;
				end else if(cnt_token_cycles >= C_TOKEN_TIMEOUT_CYCLES - 1)begin
					dec_error = 8'h0c;
				end else begin
					state_next = ST_TOKEN;
				end
			end
			ST_DATA_DONE:begin
				state_next = ST_DELIVER;
			end
			ST_DELIVER:begin
				if(valid_o && i_ready)begin
					if(cnt_data == 9'd511)begin
						state_next = ST_CRC;
					end else begin
						state_next = ST_DATA;
					end
				end
			end
			ST_CRC_DONE:begin
				if(cnt_crc)begin
					state_next = ST_FINISH;
				end else begin
					state_next = ST_CRC;
				end
			end
			ST_FINISH_DONE:begin
				state_next = ST_IDLE;
			end
			ST_ERROR:begin
				state_next = ST_ERROR;
			end
			default:begin
				dec_error = 8'hff;
			end
		endcase
		// token 窗口包含所有 SPI 字节等待周期，与读取分频无关。
		if(flag_token_wait && (cnt_token_cycles >= C_TOKEN_TIMEOUT_CYCLES - 1))begin
			dec_error = 8'h0c;
		end
		if((state_current == ST_DELIVER) && valid_o && !i_ready &&
			(cnt_backpressure >= C_BACKPRESSURE_TIMEOUT_CYCLES - 1))begin
			dec_error = 8'h0f;
		end
		if(dec_error != 8'd0)begin
			state_next = ST_ERROR;
		end
	end

	// token 与下游背压独立计时；退出各自等待状态即清零。
	always@(posedge i_clk)begin
		if(!i_rstn || !flag_token_wait)begin
			cnt_token_cycles <= 32'd0;
		end else if(cnt_token_cycles < C_TOKEN_TIMEOUT_CYCLES)begin
			cnt_token_cycles <= cnt_token_cycles + 32'd1;
		end
	end

	always@(posedge i_clk)begin
		if(!i_rstn || (state_current != ST_DELIVER) || !valid_o || i_ready)begin
			cnt_backpressure <= 32'd0;
		end else if(cnt_backpressure < C_BACKPRESSURE_TIMEOUT_CYCLES)begin
			cnt_backpressure <= cnt_backpressure + 32'd1;
		end
	end

	//-----------状态任务处理区域-----------//
	// 数据通路、命令参数与观测寄存器。
	always@(posedge i_clk)begin
		if(!i_rstn)begin
			state_return <= ST_ERROR;
			cnt_power <= 32'd0;
			cnt_power_byte <= 4'd0;
			cnt_command <= 3'd0;
			cnt_extra <= 2'd0;
			cnt_timeout <= 32'd0;
			cnt_init_retry <= 32'd0;
			cnt_data <= 9'd0;
			cnt_crc <= 1'b0;
			reg_command <= 6'd0;
			reg_argument <= 32'd0;
			reg_extra <= 32'd0;
			reg_lba <= 32'd0;
			flag_v2 <= 1'b0;
			reg_tx <= 8'hff;
			init_done_o <= 1'b0;
			error_o <= 8'd0;
			data_o <= 8'd0;
			valid_o <= 1'b0;
			done_o <= 1'b0;
			cs_o <= 1'b1;
			last_r1_o <= 8'hff;
			token_o <= 8'hff;
			ccs_o <= 1'b0;
		end else begin
			done_o <= 1'b0;
			if(dec_error != 8'd0)begin
				error_o <= dec_error;
			end
			case(state_current)
				ST_POWER:begin
					cnt_power <= cnt_power + 32'd1;
				end
				ST_CLOCK:begin
					reg_tx <= 8'hff;
					state_return <= ST_CLOCK_DONE;
				end
				ST_CLOCK_DONE:begin
					cnt_power_byte <= cnt_power_byte + 4'd1;
				end
				ST_PREP:begin
					cs_o <= 1'b1;
					reg_tx <= 8'hff;
					state_return <= ST_GAP_DONE;
					cnt_command <= 3'd0;
					cnt_timeout <= 32'd0;
					cnt_extra <= 2'd0;
					reg_extra <= 32'd0;
					case(reg_command)
						6'd8:begin
							reg_argument <= 32'h000001aa;
						end
						6'd41:begin
							reg_argument <= flag_v2 ? 32'h40000000 : 32'd0;
						end
						6'd16:begin
							reg_argument <= 32'd512;
						end
						6'd17:begin
							reg_argument <= ccs_o ? reg_lba : {reg_lba[22:0], 9'd0};
						end
						default:begin
							reg_argument <= 32'd0;
						end
					endcase
				end
				ST_GAP_DONE:begin
					cs_o <= 1'b0;
				end
				ST_CMD:begin
					state_return <= ST_CMD_DONE;
					case(cnt_command)
						3'd0:begin
							reg_tx <= {2'b01, reg_command};
						end
						3'd1:begin
							reg_tx <= reg_argument[31:24];
						end
						3'd2:begin
							reg_tx <= reg_argument[23:16];
						end
						3'd3:begin
							reg_tx <= reg_argument[15:8];
						end
						3'd4:begin
							reg_tx <= reg_argument[7:0];
						end
						default:begin
							if(reg_command == 6'd0)begin
								reg_tx <= 8'h95;
							end else if(reg_command == 6'd8)begin
								reg_tx <= 8'h87;
							end else begin
								reg_tx <= 8'hff;
							end
						end
					endcase
				end
				ST_CMD_DONE:begin
					cnt_command <= cnt_command + 3'd1;
				end
				ST_R1:begin
					reg_tx <= 8'hff;
					state_return <= ST_R1_DONE;
				end
				ST_R1_DONE:begin
					cnt_timeout <= cnt_timeout + 32'd1;
					if(!spi_rx[7])begin
						last_r1_o <= spi_rx;
					end
				end
				ST_DISPATCH:begin
					cnt_timeout <= 32'd0;
					case(reg_command)
						6'd0:begin
							reg_command <= 6'd8;
						end
						6'd8:begin
							if(last_r1_o == 8'h05)begin
								flag_v2 <= 1'b0;
								reg_command <= 6'd55;
							end
						end
						6'd55:begin
							reg_command <= 6'd41;
						end
						6'd41:begin
							cnt_init_retry <= cnt_init_retry + 32'd1;
							reg_command <= (last_r1_o == 8'd0) ? 6'd58 : 6'd55;
						end
						default:begin
						end
					endcase
				end
				ST_EXTRA:begin
					reg_tx <= 8'hff;
					state_return <= ST_EXTRA_DONE;
				end
				ST_EXTRA_DONE:begin
					// 扩展响应字节包括 ff，绝不把 ff 当作等待填充跳过。
					reg_extra <= {reg_extra[23:0], spi_rx};
					cnt_extra <= cnt_extra + 2'd1;
				end
				ST_EXTRA_CHECK:begin
					if(reg_command == 6'd8)begin
						flag_v2 <= 1'b1;
						reg_command <= 6'd55;
					end else begin
						ccs_o <= reg_extra[30];
						reg_command <= 6'd16;
					end
				end
				ST_INIT_FINISH:begin
					cs_o <= 1'b1;
					reg_tx <= 8'hff;
					state_return <= ST_IDLE;
					init_done_o <= 1'b1;
				end
				ST_IDLE:begin
					if(i_start)begin
						reg_lba <= i_lba;
						reg_command <= 6'd17;
						cnt_data <= 9'd0;
						cnt_crc <= 1'b0;
						token_o <= 8'hff;
					end
				end
				ST_TOKEN:begin
					reg_tx <= 8'hff;
					state_return <= ST_TOKEN_DONE;
				end
				ST_TOKEN_DONE:begin
					cnt_timeout <= cnt_timeout + 32'd1;
					if(spi_rx != 8'hff)begin
						token_o <= spi_rx;
					end
				end
				ST_DATA:begin
					reg_tx <= 8'hff;
					state_return <= ST_DATA_DONE;
				end
				ST_DATA_DONE:begin
					data_o <= spi_rx;
					valid_o <= 1'b1;
				end
				ST_DELIVER:begin
					if(valid_o && i_ready)begin
						valid_o <= 1'b0;
						cnt_data <= cnt_data + 9'd1;
					end
				end
				ST_CRC:begin
					reg_tx <= 8'hff;
					state_return <= ST_CRC_DONE;
				end
				ST_CRC_DONE:begin
					cnt_crc <= 1'b1;
				end
				ST_FINISH:begin
					cs_o <= 1'b1;
					reg_tx <= 8'hff;
					state_return <= ST_FINISH_DONE;
				end
				ST_FINISH_DONE:begin
					done_o <= 1'b1;
				end
				ST_ERROR:begin
					cs_o <= 1'b1;
					valid_o <= 1'b0;
				end
				default:begin
				end
			endcase
		end
	end

	//------------模块实例化区域------------//
	// 字节引擎沿用同一系统时钟，通过分频计数产生外部 SPI 时钟。
	spi_byte_master
	#(
		.C_DIV(C_INIT_DIV)
	)
	u_master
	(
		.i_clk(i_clk),
		.i_rstn(i_rstn),
		.i_start(spi_start),
		.i_div(spi_div),
		.i_tx(reg_tx),
		.o_rx(spi_rx),
		.o_busy(spi_busy),
		.o_done(spi_done),
		.o_sclk(o_sd_clk),
		.o_mosi(o_sd_mosi),
		.i_miso(i_sd_miso)
	);
endmodule
