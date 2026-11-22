//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 7 位从机地址、16 位寄存器地址及 8 位数据转换为一次带 ACK 检测的 I2C 写事务，输出开漏 SDA 控制、busy、done 和 ack_error。
// Module Name: i2c_master_write
// File Name: source/hdmi/i2c_master_write.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 7 位从机地址、16 位寄存器地址及 8 位数据转换为一次带 ACK 检测的 I2C 写事务，输出开漏 SDA 控制、busy、done 和 ack_error。
// 默认寄存器低字节先发，参数可切换为 OV5640 使用的高字节先发；状态机依次处理 START、数据、ACK 与 STOP。
//
// Dependencies: 无子模块；上层提供时钟、SDA 采样及开漏三态连接。
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
//
// Create Date: 2026-09-10
// Design Name: I2C Register Write Master
// Module Name: i2c_master_write
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 以开漏方式完成一次 7-bit 从机、16-bit 寄存器和 8-bit 数据写事务。
//
// Dependencies: None
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments: 地址按 MS7210 同板参考的低字节、后高字节顺序发送。
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module i2c_master_write #(
	parameter integer C_HALF_PERIOD = 34,
	// HDMI bridge tables in this project use low-byte-first register order;
	// OV5640 SCCB uses the conventional high-byte-first order.
	parameter integer C_REG_ADDR_HIGH_FIRST = 0
)(
	input i_clk,
	input i_rst_n,
	input i_start,
	input [6:0] i_device_addr,
	input [15:0] i_reg_addr,
	input [7:0] i_data,
	input i_sda,
	output o_busy,
	output o_done,
	output o_ack_error,
	output o_scl,
	output o_sda_drive_low
);

	localparam [3:0] ST_IDLE = 4'd0;
	localparam [3:0] ST_START = 4'd1;
	localparam [3:0] ST_START_LOW = 4'd2;
	localparam [3:0] ST_DATA_SETUP = 4'd3;
	localparam [3:0] ST_DATA_HIGH = 4'd4;
	localparam [3:0] ST_DATA_HOLD = 4'd5;
	localparam [3:0] ST_ACK_HIGH = 4'd6;
	localparam [3:0] ST_ACK_SAMPLE = 4'd7;
	localparam [3:0] ST_ACK_LOW = 4'd8;
	localparam [3:0] ST_STOP_LOW = 4'd9;
	localparam [3:0] ST_STOP_HIGH = 4'd10;
	localparam [3:0] ST_STOP_RELEASE = 4'd11;

	reg [3:0] state_current;
	reg [3:0] state_next;
	reg [15:0] cnt_half_period;
	reg [1:0] cnt_byte;
	reg [2:0] cnt_bit;
	reg [6:0] reg_device_addr;
	reg [15:0] reg_reg_addr;
	reg [7:0] reg_data;
	reg [7:0] current_byte;
	reg busy_o;
	reg done_o;
	reg ack_error_o;
	reg flag_ack_failed;
	reg scl_o;
	reg sda_drive_low_o;

	wire flag_half_tick;
	assign flag_half_tick = (cnt_half_period == C_HALF_PERIOD - 1);

	assign o_busy = busy_o;
	assign o_done = done_o;
	assign o_ack_error = ack_error_o;
	assign o_scl = scl_o;
	assign o_sda_drive_low = sda_drive_low_o;

	always@(*)begin
		case(cnt_byte)
			2'd0:current_byte = {reg_device_addr, 1'b0};
			2'd1:current_byte = (C_REG_ADDR_HIGH_FIRST != 0) ? reg_reg_addr[15:8] : reg_reg_addr[7:0];
			2'd2:current_byte = (C_REG_ADDR_HIGH_FIRST != 0) ? reg_reg_addr[7:0] : reg_reg_addr[15:8];
			default:current_byte = reg_data;
		endcase
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			cnt_half_period <= 16'd0;
		end else if(state_current == ST_IDLE)begin
			cnt_half_period <= 16'd0;
		end else if(flag_half_tick == 1'b1)begin
			cnt_half_period <= 16'd0;
		end else begin
			cnt_half_period <= cnt_half_period + 1'b1;
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			state_current <= ST_IDLE;
		end else begin
			state_current <= state_next;
		end
	end

	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_IDLE:if(i_start == 1'b1)state_next = ST_START;
			ST_START:if(flag_half_tick == 1'b1)state_next = ST_START_LOW;
			ST_START_LOW:if(flag_half_tick == 1'b1)state_next = ST_DATA_SETUP;
			ST_DATA_SETUP:if(flag_half_tick == 1'b1)state_next = ST_DATA_HIGH;
			ST_DATA_HIGH:if(flag_half_tick == 1'b1)state_next = ST_DATA_HOLD;
			ST_DATA_HOLD:if(flag_half_tick == 1'b1)begin
				if(cnt_bit == 3'd0)state_next = ST_ACK_HIGH;
				else state_next = ST_DATA_SETUP;
			end
			ST_ACK_HIGH:if(flag_half_tick == 1'b1)state_next = ST_ACK_SAMPLE;
			ST_ACK_SAMPLE:if(flag_half_tick == 1'b1)state_next = ST_ACK_LOW;
			ST_ACK_LOW:if(flag_half_tick == 1'b1)begin
				if(cnt_byte == 2'd3)state_next = ST_STOP_LOW;
				else state_next = ST_DATA_SETUP;
			end
			ST_STOP_LOW:if(flag_half_tick == 1'b1)state_next = ST_STOP_HIGH;
			ST_STOP_HIGH:if(flag_half_tick == 1'b1)state_next = ST_STOP_RELEASE;
			ST_STOP_RELEASE:if(flag_half_tick == 1'b1)state_next = ST_IDLE;
			default:state_next = ST_IDLE;
		endcase
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			cnt_byte <= 2'd0;
			cnt_bit <= 3'd7;
		end else if(state_current == ST_IDLE && i_start == 1'b1)begin
			cnt_byte <= 2'd0;
			cnt_bit <= 3'd7;
		end else if(state_current == ST_DATA_HOLD && flag_half_tick == 1'b1)begin
			if(cnt_bit != 3'd0)cnt_bit <= cnt_bit - 1'b1;
		end else if(state_current == ST_ACK_LOW && flag_half_tick == 1'b1 && cnt_byte != 2'd3)begin
			cnt_byte <= cnt_byte + 1'b1;
			cnt_bit <= 3'd7;
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			reg_device_addr <= 7'd0;
			reg_reg_addr <= 16'd0;
			reg_data <= 8'd0;
			busy_o <= 1'b0;
			done_o <= 1'b0;
			ack_error_o <= 1'b0;
			flag_ack_failed <= 1'b0;
		end else begin
			done_o <= 1'b0;
			if(state_current == ST_IDLE && i_start == 1'b1)begin
				reg_device_addr <= i_device_addr;
				reg_reg_addr <= i_reg_addr;
				reg_data <= i_data;
				busy_o <= 1'b1;
				ack_error_o <= 1'b0;
				flag_ack_failed <= 1'b0;
			end else if(state_current == ST_ACK_SAMPLE && flag_half_tick == 1'b1 && i_sda == 1'b1)begin
				flag_ack_failed <= 1'b1;
			end else if(state_current == ST_STOP_RELEASE && flag_half_tick == 1'b1)begin
				busy_o <= 1'b0;
				done_o <= 1'b1;
				ack_error_o <= flag_ack_failed;
			end
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(i_rst_n == 1'b0)begin
			scl_o <= 1'b1;
			sda_drive_low_o <= 1'b0;
		end else begin
			case(state_current)
				ST_IDLE:begin
					scl_o <= 1'b1;
					sda_drive_low_o <= 1'b0;
				end
				ST_START:if(flag_half_tick == 1'b1)begin
					scl_o <= 1'b1;
					sda_drive_low_o <= 1'b1;
				end
				ST_START_LOW:if(flag_half_tick == 1'b1)scl_o <= 1'b0;
				ST_DATA_SETUP:if(flag_half_tick == 1'b1)sda_drive_low_o <= ~current_byte[cnt_bit];
				ST_DATA_HIGH:if(flag_half_tick == 1'b1)scl_o <= 1'b1;
				ST_DATA_HOLD:if(flag_half_tick == 1'b1)begin
					scl_o <= 1'b0;
					if(cnt_bit == 3'd0)sda_drive_low_o <= 1'b0;
				end
				ST_ACK_HIGH:if(flag_half_tick == 1'b1)scl_o <= 1'b1;
				ST_ACK_LOW:if(flag_half_tick == 1'b1)scl_o <= 1'b0;
				ST_STOP_LOW:if(flag_half_tick == 1'b1)begin
					scl_o <= 1'b0;
					sda_drive_low_o <= 1'b1;
				end
				ST_STOP_HIGH:if(flag_half_tick == 1'b1)scl_o <= 1'b1;
				ST_STOP_RELEASE:if(flag_half_tick == 1'b1)sda_drive_low_o <= 1'b0;
				default:begin
					scl_o <= scl_o;
					sda_drive_low_o <= sda_drive_low_o;
				end
			endcase
		end
	end

endmodule
