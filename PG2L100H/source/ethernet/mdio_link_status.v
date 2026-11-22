//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-11-01
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: RTL8211F Clause-22 初始化与链路轮询控制器，执行 PHY 硬复位、PHYID 探测、page D08/reg11 bit8 的内部 TX 2ns 延迟读改写与读回确认。
// Module Name: mdio_link_status
// File Name: source/ethernet/mdio_link_status.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// RTL8211F Clause-22 初始化与链路轮询控制器，执行 PHY 硬复位、PHYID 探测、page D08/reg11 bit8 的内部 TX 2ns 延迟读改写与读回确认。
// 首探参数 PHY 地址失败后扫描地址 1..7，禁止广播写；保存并恢复原 page，完全不访问 RX reg15，配置、PHYID 和 page 均读回确认才置 config_done。
// 27MHz 单时钟分频产生 MDC，在高半周期末采样 MDIO；错误延时重试，o_speed 保持 00 unknown，避免误用 BMSR 解释速率。
//
// Dependencies: 27MHz 系统时钟、RTL8211F（PHYID=001cc916）、板级 MDIO 上拉及 PHY 复位管脚。
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
// Create Date: 2026-11-01
// Design Name: RTL8211F Clause-22 initialization and link polling
// Module Name: mdio_link_status
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 (project reference: PDS 2022.2-SP6.4)
// Description: PHY复位、真实PHYID发现、TX内部2ns延迟配置及链路轮询。
// Dependencies: 27MHz时钟；RTL8211F PHYID=001cc916；板级MDIO上拉。
// Revision: 0.01 - 替换原只读BMSR实现；单owner读改写并校验及恢复页。
// Additional Comments:
//   同板demo使用PHY地址1；参数地址首探失败后扫描1..7，禁止广播写。
//   默认复位11ms、释放等待31ms，满足原厂要求至少10ms/30ms。
//   TX延迟位依据Linux v6.12 realtek.c：page D08 reg11 bit8。
//   RX reg15完全不访问；所有其他TX位及进入时page均保留。
//   C_*周期参数均须>=1；C_MDC_DIV默认16，读采样在MDC高半周期末。
//   o_config_done仅在PHYID/配置/原page全部读回确认后为1；错误延时重试。
//   o_speed保持00(unknown)，不将BMSR位误解释为速率。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module mdio_link_status #(
	parameter [4:0] C_PHY_ADDR = 5'd1,
	parameter [4:0] C_REG_ADDR = 5'd1,
	parameter integer C_RESET_CYCLES = 297000,
	parameter integer C_SETTLE_CYCLES = 837000,
	parameter integer C_MDC_DIV = 16,
	parameter integer C_POLL_CYCLES = 27000,
	parameter integer C_RETRY_CYCLES = 270000
)(
	input i_clk,
	input i_rst_n,
	inout io_mdio,
	output o_mdc,
	output o_link,
	output [1:0] o_speed,
	output o_phy_reset_n,
	output o_config_done,
	output o_config_error
);
	//-------------状态参数区域-------------//
	localparam [4:0] ST_RESET = 5'd0;
	localparam [4:0] ST_SETTLE = 5'd1;
	localparam [4:0] ST_ID1 = 5'd2;
	localparam [4:0] ST_ID2 = 5'd3;
	localparam [4:0] ST_NEXT_ADDR = 5'd4;
	localparam [4:0] ST_SAVE_PAGE = 5'd5;
	localparam [4:0] ST_SELECT_PAGE = 5'd6;
	localparam [4:0] ST_VERIFY_SELECT = 5'd7;
	localparam [4:0] ST_READ_TX = 5'd8;
	localparam [4:0] ST_WRITE_TX = 5'd9;
	localparam [4:0] ST_VERIFY_TX = 5'd10;
	localparam [4:0] ST_RESTORE_PAGE = 5'd11;
	localparam [4:0] ST_VERIFY_PAGE = 5'd12;
	localparam [4:0] ST_READY = 5'd13;
	localparam [4:0] ST_LINK = 5'd14;
	localparam [4:0] ST_RETRY = 5'd15;

	//---------------状态及事务信号---------------//
	reg [1:0] reset_sync;
	wire reset_n;
	reg [4:0] state_current;
	reg [4:0] state_next;
	reg [31:0] cnt_wait;
	reg [4:0] reg_phy_addr;
	reg [3:0] cnt_scan;
	reg [15:0] reg_saved_page;
	reg [15:0] reg_desired_tx;
	reg flag_saved_page;
	reg flag_init_failed;
	reg flag_transaction_wait;
	reg flag_command_state;
	reg reg_command_write;
	reg [4:0] reg_command_addr;
	reg [15:0] reg_command_data;
	wire flag_transaction_start;
	wire flag_transaction_finished;
	reg flag_transfer_busy;
	reg flag_transfer_done;
	reg flag_transfer_ack;
	reg [15:0] reg_transfer_data;
	reg [31:0] cnt_mdc;
	reg [5:0] cnt_bit;
	reg [63:0] reg_frame_shift;
	reg [15:0] reg_read_shift;
	reg flag_transfer_write;
	reg mdio_oe;
	reg mdio_o;
	reg mdc_o;
	reg link_o;
	reg phy_reset_n_o;
	reg config_done_o;
	reg config_error_o;

	//-------------输出信号连线-------------//
	assign reset_n = reset_sync[1];
	assign o_mdc = mdc_o;
	assign io_mdio = mdio_oe ? mdio_o : 1'bz;
	assign o_link = link_o;
	assign o_speed = 2'b00;
	assign o_phy_reset_n = phy_reset_n_o;
	assign o_config_done = config_done_o;
	assign o_config_error = config_error_o;
	assign flag_transaction_start = flag_command_state && !flag_transaction_wait && !flag_transfer_busy;
	assign flag_transaction_finished = flag_transaction_wait && flag_transfer_done;

	// 异步拉低复位、27MHz域两拍同步释放；事务中按钮复位立即释放MDIO。
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reset_sync <= 2'b00;
		end else begin
			reset_sync <= {reset_sync[0], 1'b1};
		end
	end

	// 单一事务序列覆盖初始化、失败恢复和BMSR轮询。
	// 外部硬复位用独立寄存器，避免多位FSM译码在状态切换时产生窄脉冲。
	always@(posedge i_clk or negedge reset_n)begin
		if(!reset_n)begin
			phy_reset_n_o <= 1'b0;
		end else if(state_next == ST_RESET)begin
			phy_reset_n_o <= 1'b0;
		end else if(state_current == ST_RESET && state_next == ST_SETTLE)begin
			phy_reset_n_o <= 1'b1;
		end
	end

	always@(posedge i_clk or negedge reset_n)begin
		if(!reset_n)begin
			state_current <= ST_RESET;
		end else begin
			state_current <= state_next;
		end
	end

	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_RESET:begin
				if(cnt_wait >= C_RESET_CYCLES - 1)begin
					state_next = ST_SETTLE;
				end
			end
			ST_SETTLE:begin
				if(cnt_wait >= C_SETTLE_CYCLES - 1)begin
					state_next = ST_ID1;
				end
			end
			ST_ID1:begin
				if(flag_transaction_finished)begin
					if(flag_transfer_ack && reg_transfer_data == 16'h001c)begin
						state_next = ST_ID2;
					end else begin
						state_next = ST_NEXT_ADDR;
					end
				end
			end
			ST_ID2:begin
				if(flag_transaction_finished)begin
					if(flag_transfer_ack && reg_transfer_data == 16'hc916)begin
						state_next = ST_SAVE_PAGE;
					end else begin
						state_next = ST_NEXT_ADDR;
					end
				end
			end
			ST_NEXT_ADDR:begin
				if(cnt_scan < 4'd7)begin
					state_next = ST_ID1;
				end else begin
					state_next = ST_RETRY;
				end
			end
			ST_SAVE_PAGE:begin
				if(flag_transaction_finished)begin
					if(flag_transfer_ack)begin
						state_next = ST_SELECT_PAGE;
					end else begin
						state_next = ST_RETRY;
					end
				end
			end
			ST_SELECT_PAGE:begin
				if(flag_transaction_finished)begin
					state_next = ST_VERIFY_SELECT;
				end
			end
			ST_VERIFY_SELECT:begin
				if(flag_transaction_finished)begin
					if(flag_transfer_ack && reg_transfer_data == 16'h0d08)begin
						state_next = ST_READ_TX;
					end else begin
						state_next = ST_RESTORE_PAGE;
					end
				end
			end
			ST_READ_TX:begin
				if(flag_transaction_finished)begin
					if(flag_transfer_ack)begin
						state_next = ST_WRITE_TX;
					end else begin
						state_next = ST_RESTORE_PAGE;
					end
				end
			end
			ST_WRITE_TX:begin
				if(flag_transaction_finished)begin
					state_next = ST_VERIFY_TX;
				end
			end
			ST_VERIFY_TX:begin
				if(flag_transaction_finished)begin
					state_next = ST_RESTORE_PAGE;
				end
			end
			ST_RESTORE_PAGE:begin
				if(flag_transaction_finished)begin
					state_next = ST_VERIFY_PAGE;
				end
			end
			ST_VERIFY_PAGE:begin
				if(flag_transaction_finished)begin
					if(flag_transfer_ack && reg_transfer_data == reg_saved_page && !flag_init_failed)begin
						state_next = ST_READY;
					end else begin
						state_next = ST_RETRY;
					end
				end
			end
			ST_READY:begin
				if(cnt_wait >= C_POLL_CYCLES - 1)begin
					state_next = ST_LINK;
				end
			end
			ST_LINK:begin
				if(flag_transaction_finished)begin
					if(flag_transfer_ack)begin
						state_next = ST_READY;
					end else begin
						state_next = ST_RETRY;
					end
				end
			end
			ST_RETRY:begin
				if(cnt_wait >= C_RETRY_CYCLES - 1)begin
					if(flag_saved_page)begin
						state_next = ST_RESTORE_PAGE;
					end else begin
						state_next = ST_ID1;
					end
				end
			end
			default:begin
				state_next = ST_RESET;
			end
		endcase
	end

	// 各阶段只提交一个Clause-22事务；引擎固定64bit结束，不等待外部无限应答。
	always@(*)begin
		flag_command_state = 1'b1;
		reg_command_write = 1'b0;
		reg_command_addr = 5'd31;
		reg_command_data = 16'd0;
		case(state_current)
			ST_ID1:begin
				reg_command_addr = 5'd2;
			end
			ST_ID2:begin
				reg_command_addr = 5'd3;
			end
			ST_SAVE_PAGE, ST_VERIFY_SELECT, ST_VERIFY_PAGE:begin
				reg_command_addr = 5'd31;
			end
			ST_SELECT_PAGE:begin
				reg_command_write = 1'b1;
				reg_command_data = 16'h0d08;
			end
			ST_READ_TX, ST_VERIFY_TX:begin
				reg_command_addr = 5'h11;
			end
			ST_WRITE_TX:begin
				reg_command_write = 1'b1;
				reg_command_addr = 5'h11;
				reg_command_data = reg_desired_tx;
			end
			ST_RESTORE_PAGE:begin
				reg_command_write = 1'b1;
				reg_command_data = reg_saved_page;
			end
			ST_LINK:begin
				reg_command_addr = C_REG_ADDR;
			end
			default:begin
				flag_command_state = 1'b0;
			end
		endcase
	end

	always@(posedge i_clk or negedge reset_n)begin
		if(!reset_n)begin
			cnt_wait <= 32'd0;
		end else if(state_current != state_next)begin
			cnt_wait <= 32'd0;
		end else if(state_current == ST_RESET || state_current == ST_SETTLE ||
			state_current == ST_READY || state_current == ST_RETRY)begin
			cnt_wait <= cnt_wait + 1'b1;
		end
	end

	always@(posedge i_clk or negedge reset_n)begin
		if(!reset_n)begin
			flag_transaction_wait <= 1'b0;
		end else if(flag_transaction_finished)begin
			flag_transaction_wait <= 1'b0;
		end else if(flag_transaction_start)begin
			flag_transaction_wait <= 1'b1;
		end
	end

	// 已保存原page后即使读回失败，也先恢复该page，禁止把D08误当下一次原page。
	always@(posedge i_clk or negedge reset_n)begin
		if(!reset_n)begin
			reg_phy_addr <= (C_PHY_ADDR == 5'd0) ? 5'd1 : C_PHY_ADDR;
			cnt_scan <= 4'd0;
			reg_saved_page <= 16'd0;
			reg_desired_tx <= 16'd0;
			flag_saved_page <= 1'b0;
			flag_init_failed <= 1'b0;
			link_o <= 1'b0;
			config_done_o <= 1'b0;
			config_error_o <= 1'b0;
		end else begin
			if(state_current == ST_NEXT_ADDR && cnt_scan < 4'd7)begin
				reg_phy_addr <= cnt_scan + 5'd1;
				cnt_scan <= cnt_scan + 1'b1;
			end
			if(state_current == ST_RETRY)begin
				config_done_o <= 1'b0;
				config_error_o <= 1'b1;
				link_o <= 1'b0;
				if(state_next == ST_ID1)begin
					reg_phy_addr <= (C_PHY_ADDR == 5'd0) ? 5'd1 : C_PHY_ADDR;
					cnt_scan <= 4'd0;
					flag_init_failed <= 1'b0;
				end
			end
			if(flag_transaction_finished)begin
				case(state_current)
					ST_SAVE_PAGE:begin
						if(flag_transfer_ack)begin
							reg_saved_page <= reg_transfer_data;
							flag_saved_page <= 1'b1;
							flag_init_failed <= 1'b0;
						end
					end
					ST_VERIFY_SELECT:begin
						if(flag_transfer_ack && reg_transfer_data == 16'h0d08)begin
							// 精确匹配才继续；仿真X/Z同样进入失败分支。
						end else begin
							flag_init_failed <= 1'b1;
							config_error_o <= 1'b1;
						end
					end
					ST_READ_TX:begin
						if(flag_transfer_ack)begin
							reg_desired_tx <= reg_transfer_data | 16'h0100;
						end else begin
							flag_init_failed <= 1'b1;
							config_error_o <= 1'b1;
						end
					end
					ST_VERIFY_TX:begin
						if(flag_transfer_ack && reg_transfer_data == reg_desired_tx)begin
							// 保留整个原始寄存器，仅bit8允许变化。
						end else begin
							flag_init_failed <= 1'b1;
							config_error_o <= 1'b1;
						end
					end
					ST_VERIFY_PAGE:begin
						if(flag_transfer_ack && reg_transfer_data == reg_saved_page)begin
							flag_saved_page <= 1'b0;
							if(!flag_init_failed)begin
								config_done_o <= 1'b1;
								config_error_o <= 1'b0;
							end
						end else begin
							flag_init_failed <= 1'b1;
							config_error_o <= 1'b1;
						end
					end
					ST_LINK:begin
						if(flag_transfer_ack && reg_transfer_data[2] == 1'b1)begin
							link_o <= 1'b1;
						end else begin
							link_o <= 1'b0;
						end
					end
					default:begin
					end
				endcase
			end
		end
	end

	// MDC只作为输出信号，内部始终使用27MHz时钟及分频使能。
	// PHY允许在上升沿后300ns给出数据，故在高半周期结束前采样，而非上升沿。
	always@(posedge i_clk or negedge reset_n)begin
		if(!reset_n)begin
			cnt_mdc <= 32'd0;
			cnt_bit <= 6'd0;
			reg_frame_shift <= 64'd0;
			reg_read_shift <= 16'd0;
			reg_transfer_data <= 16'd0;
			flag_transfer_write <= 1'b0;
			flag_transfer_busy <= 1'b0;
			flag_transfer_done <= 1'b0;
			flag_transfer_ack <= 1'b0;
			mdio_oe <= 1'b0;
			mdio_o <= 1'b1;
			mdc_o <= 1'b0;
		end else begin
			flag_transfer_done <= 1'b0;
			if(flag_transaction_start)begin
				cnt_mdc <= 32'd0;
				cnt_bit <= 6'd0;
				reg_frame_shift <= {32'hffff_ffff, 2'b01,
					(reg_command_write ? 2'b01 : 2'b10), reg_phy_addr,
					reg_command_addr, 2'b10, reg_command_data};
				reg_read_shift <= 16'd0;
				flag_transfer_write <= reg_command_write;
				flag_transfer_busy <= 1'b1;
				flag_transfer_ack <= 1'b1;
				mdio_oe <= 1'b1;
				mdio_o <= 1'b1;
				mdc_o <= 1'b0;
			end else if(flag_transfer_busy)begin
				if(cnt_mdc >= C_MDC_DIV - 1)begin
					cnt_mdc <= 32'd0;
					mdc_o <= ~mdc_o;
					if(mdc_o)begin
						if(!flag_transfer_write && cnt_bit == 6'd47)begin
							if(io_mdio == 1'b0)begin
								flag_transfer_ack <= 1'b1;
							end else begin
								flag_transfer_ack <= 1'b0;
							end
						end
						if(!flag_transfer_write && cnt_bit >= 6'd48)begin
							reg_read_shift <= {reg_read_shift[14:0], io_mdio};
						end
						if(cnt_bit == 6'd63)begin
							flag_transfer_busy <= 1'b0;
							flag_transfer_done <= 1'b1;
							reg_transfer_data <= {reg_read_shift[14:0], io_mdio};
							mdio_oe <= 1'b0;
						end else begin
							cnt_bit <= cnt_bit + 1'b1;
							reg_frame_shift <= {reg_frame_shift[62:0], 1'b0};
							mdio_o <= reg_frame_shift[62];
							mdio_oe <= flag_transfer_write || cnt_bit < 6'd45;
						end
					end
				end else begin
					cnt_mdc <= cnt_mdc + 1'b1;
				end
			end else begin
				mdc_o <= 1'b0;
				mdio_oe <= 1'b0;
			end
		end
	end
endmodule
