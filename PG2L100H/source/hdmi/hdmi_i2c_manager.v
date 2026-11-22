//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 共享 MS7200 接收器与 MS7210 发送器 I2C 总线的唯一开漏控制器，依次完成 RX 表、连接设置与 TX 的 49 条寄存器初始化。
// Module Name: hdmi_i2c_manager
// File Name: source/hdmi/hdmi_i2c_manager.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 共享 MS7200 接收器与 MS7210 发送器 I2C 总线的唯一开漏控制器，依次完成 RX 表、连接设置与 TX 的 49 条寄存器初始化。
// NACK 事务完整发出 STOP 后延时重试原条目，全部配置及稳定等待结束后发布初始化完成状态。
//
// Dependencies: i2c_master_write、ms7200_rx_table.vh；板级 SDA 开漏与上拉连接。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// Single open-drain owner for the shared MS7200/MS7210 I2C bus.
// RX uses the board HDMI-loop demo table; TX preserves the existing 720p
// sequence.  Only one i2c_master_write instance drives K22/K23.
// Revision 2026-10-25: NACK完整STOP后等待并重试原条目，恢复后继续初始化。
module hdmi_i2c_manager #(
	parameter integer C_I2C_HALF_PERIOD = 34,
	parameter integer C_RESET_HOLD_CYCLES = 27000,
	parameter integer C_TX_SETTLE_CYCLES = 8640000,
	parameter integer C_ENABLE_RX = 1,
	// 27MHz系统域10ms；参数契约为>=1，不影响无NACK时的命令拍数。
	parameter integer C_RETRY_DELAY_CYCLES = 270000
)(
	input i_clk,
	input i_rst_n,
	input i_sda,
	output o_scl,
	output o_sda_drive_low,
	output o_rx_init_done,
	output o_tx_init_done,
	output o_init_done,
	output o_init_error
);
	localparam integer C_RX_TABLE_COUNT = 311;
	localparam integer C_RX_TOTAL_COUNT = C_RX_TABLE_COUNT + 1;
	localparam integer C_TX_COUNT = 49;
	localparam [3:0] ST_RESET_HOLD = 4'd0;
	localparam [3:0] ST_RX_LAUNCH = 4'd1;
	localparam [3:0] ST_RX_WAIT = 4'd2;
	localparam [3:0] ST_TX_LAUNCH = 4'd3;
	localparam [3:0] ST_TX_WAIT = 4'd4;
	localparam [3:0] ST_TX_SETTLE = 4'd5;
	localparam [3:0] ST_DONE = 4'd6;
	localparam [3:0] ST_ERROR = 4'd7;
	reg [3:0] state_current;
	reg [31:0] delay_count;
	reg [9:0] rx_index;
	reg [6:0] tx_index;
	reg rx_done_reg, tx_done_reg, error_reg, master_start;
	reg retry_tx;
	reg [15:0] command_addr;
	reg [7:0] command_data;
	reg [6:0] command_device;
	reg [15:0] rx_addr;
	reg [7:0] rx_data;
	wire master_busy;
	wire master_done;
	wire master_ack_error;

	assign o_rx_init_done = rx_done_reg;
	assign o_tx_init_done = tx_done_reg;
	assign o_init_done = rx_done_reg && tx_done_reg;
	assign o_init_error = error_reg;

	// The demo performs a connection write before its 311-entry RX table.  We
	// retain that write and treat its ACK as the transport-level check.
	always @(*) begin
		rx_addr = 16'h0000;
		rx_data = 8'h00;
		case (rx_index - 10'd1)
`include "ms7200_rx_table.vh"
			default: begin rx_addr = 16'h0000; rx_data = 8'h00; end
		endcase
		command_device = 7'h2b;
		command_addr = 16'h0000;
		command_data = 8'h00;
		if (state_current == ST_RX_LAUNCH || state_current == ST_RX_WAIT) begin
			if (rx_index == 10'd0) begin command_addr=16'h0003; command_data=8'h5a; end
			else begin command_addr=rx_addr; command_data=rx_data; end
		end else if (state_current == ST_TX_LAUNCH || state_current == ST_TX_WAIT ||
			state_current == ST_TX_SETTLE) begin
			command_device = 7'h59;
			case (tx_index)
				7'd0: begin command_addr=16'h1281; command_data=8'h04; end
				7'd1: begin command_addr=16'h0016; command_data=8'h04; end
				7'd2: begin command_addr=16'h0009; command_data=8'h01; end
				7'd3: begin command_addr=16'h0007; command_data=8'h09; end
				7'd4: begin command_addr=16'h0008; command_data=8'hf0; end
				7'd5: begin command_addr=16'h000a; command_data=8'hf0; end
				7'd6: begin command_addr=16'h0006; command_data=8'h11; end
				7'd7: begin command_addr=16'h0531; command_data=8'h84; end
				7'd8: begin command_addr=16'h0900; command_data=8'h20; end
				7'd9: begin command_addr=16'h0901; command_data=8'h47; end
				7'd10: begin command_addr=16'h0904; command_data=8'h09; end
				7'd11: begin command_addr=16'h0923; command_data=8'h07; end
				7'd12: begin command_addr=16'h0924; command_data=8'h44; end
				7'd13: begin command_addr=16'h0925; command_data=8'h44; end
				7'd14: begin command_addr=16'h090f; command_data=8'h80; end
				7'd15: begin command_addr=16'h091f; command_data=8'h07; end
				7'd16: begin command_addr=16'h0920; command_data=8'h1e; end
				7'd17: begin command_addr=16'h0018; command_data=8'h20; end
				7'd18: begin command_addr=16'h05c0; command_data=8'hfe; end
				7'd19: begin command_addr=16'h000b; command_data=8'h00; end
				7'd20: begin command_addr=16'h0507; command_data=8'h06; end
				7'd21: begin command_addr=16'h0906; command_data=8'h04; end
				7'd22: begin command_addr=16'h0920; command_data=8'h5e; end
				7'd23: begin command_addr=16'h0926; command_data=8'hdd; end
				7'd24: begin command_addr=16'h0927; command_data=8'h0d; end
				7'd25: begin command_addr=16'h0928; command_data=8'h88; end
				7'd26: begin command_addr=16'h0929; command_data=8'h08; end
				7'd27: begin command_addr=16'h0910; command_data=8'h01; end
				7'd28: begin command_addr=16'h000b; command_data=8'h11; end
				7'd29: begin command_addr=16'h050e; command_data=8'h00; end
				7'd30: begin command_addr=16'h050a; command_data=8'h82; end
				7'd31: begin command_addr=16'h0509; command_data=8'h02; end
				7'd32: begin command_addr=16'h050b; command_data=8'h0d; end
				7'd33: begin command_addr=16'h050d; command_data=8'h06; end
				7'd34: begin command_addr=16'h050d; command_data=8'h11; end
				7'd35: begin command_addr=16'h050d; command_data=8'h58; end
				7'd36: begin command_addr=16'h050d; command_data=8'h00; end
				7'd37: begin command_addr=16'h050d; command_data=8'h00; end
				7'd38: begin command_addr=16'h050d; command_data=8'h00; end
				7'd39: begin command_addr=16'h050d; command_data=8'h00; end
				7'd40: begin command_addr=16'h050d; command_data=8'h00; end
				7'd41: begin command_addr=16'h050d; command_data=8'h00; end
				7'd42: begin command_addr=16'h050d; command_data=8'h00; end
				7'd43: begin command_addr=16'h050d; command_data=8'h00; end
				7'd44: begin command_addr=16'h050d; command_data=8'h00; end
				7'd45: begin command_addr=16'h050d; command_data=8'h00; end
				7'd46: begin command_addr=16'h050d; command_data=8'h00; end
				7'd47: begin command_addr=16'h050e; command_data=8'h40; end
				default: begin command_addr=16'h0507; command_data=8'h00; end
			endcase
		end
	end

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			state_current<=ST_RESET_HOLD; delay_count<=0; rx_index<=0; tx_index<=0;
			rx_done_reg<=0; tx_done_reg<=0; error_reg<=0; master_start<=0;
			retry_tx<=1'b0;
		end else begin
			master_start <= 1'b0;
			case (state_current)
				ST_RESET_HOLD: if (delay_count >= C_RESET_HOLD_CYCLES-1) begin
					delay_count<=0; state_current <= (C_ENABLE_RX != 0) ? ST_RX_LAUNCH : ST_TX_LAUNCH;
				end else delay_count<=delay_count+1'b1;
				ST_RX_LAUNCH: begin master_start<=1'b1; state_current<=ST_RX_WAIT; end
				ST_RX_WAIT: if (master_done) begin
					// master_done已包含完整STOP；保留index，不能跳过失败寄存器。
					if (master_ack_error) begin error_reg<=1'b1; retry_tx<=1'b0; delay_count<=0; state_current<=ST_ERROR; end
					else if (rx_index == C_RX_TOTAL_COUNT-1) begin
						error_reg<=1'b0; rx_done_reg<=1'b1; tx_index<=0; state_current<=ST_TX_LAUNCH;
					end else begin error_reg<=1'b0; rx_index<=rx_index+1'b1; state_current<=ST_RX_LAUNCH; end
				end
				ST_TX_LAUNCH: begin master_start<=1'b1; state_current<=ST_TX_WAIT; end
				ST_TX_WAIT: if (master_done) begin
					if (master_ack_error) begin error_reg<=1'b1; retry_tx<=1'b1; delay_count<=0; state_current<=ST_ERROR; end
					else if (tx_index == C_TX_COUNT-1) begin error_reg<=1'b0; tx_done_reg<=1'b1; state_current<=ST_DONE; end
					else if (tx_index == 7'd18 && C_TX_SETTLE_CYCLES != 0) begin error_reg<=1'b0; delay_count<=0; state_current<=ST_TX_SETTLE; end
					else begin error_reg<=1'b0; tx_index<=tx_index+1'b1; state_current<=ST_TX_LAUNCH; end
				end
				ST_TX_SETTLE: if (delay_count >= C_TX_SETTLE_CYCLES-1) begin
					delay_count<=0; tx_index<=tx_index+1'b1; state_current<=ST_TX_LAUNCH;
				end else delay_count<=delay_count+1'b1;
				ST_DONE: state_current<=ST_DONE;
				// 外设晚就绪或短暂NACK时自行恢复；永久NACK保持有界频率重试，
				// 不拉高done、不更改原命令，也不重启已成功完成的另一芯片。
				ST_ERROR: if (delay_count >= C_RETRY_DELAY_CYCLES-1) begin
					delay_count<=0; state_current<=retry_tx ? ST_TX_LAUNCH : ST_RX_LAUNCH;
				end else delay_count<=delay_count+1'b1;
				default: state_current<=ST_RESET_HOLD;
			endcase
		end
	end

	i2c_master_write #(.C_HALF_PERIOD(C_I2C_HALF_PERIOD)) u_i2c_master_write(
		.i_clk(i_clk), .i_rst_n(i_rst_n), .i_start(master_start),
		.i_device_addr(command_device), .i_reg_addr(command_addr), .i_data(command_data),
		.i_sda(i_sda), .o_busy(master_busy), .o_done(master_done),
		.o_ack_error(master_ack_error), .o_scl(o_scl), .o_sda_drive_low(o_sda_drive_low)
	);
endmodule
