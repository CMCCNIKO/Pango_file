//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: OV5640 的离线 SCCB 初始化序列器，延时启动后向 7'h3c 写入寄存器表。
// Module Name: ov5640_sccb_init
// File Name: source/hdmi/ov5640_sccb_init.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// OV5640 的离线 SCCB 初始化序列器，延时启动后向 7'h3c 写入寄存器表。
// 软件复位 3008=82 后等待传感器稳定，再继续常规配置；遇 NACK 进入错误终态，全部完成后保持 done。
//
// Dependencies: i2c_master_write（C_REG_ADDR_HIGH_FIRST=1）、ov5640_reg_table.vh。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// Offline OV5640 CMOS1 SCCB sequencer.
// The donor table is kept as a ROM include and is sent through the generic
// open-drain I2C writer with high-byte-first register addressing.
module ov5640_sccb_init #(
	parameter integer C_HALF_PERIOD = 34,
	parameter integer C_START_DELAY = 567000,
	// OV5640 software reset (3008=82) needs a settle interval before
	// normal-run (3008=42). 10 ms at the 27 MHz system clock preserves the
	// donor design's dummy-write delay without sending invalid registers.
	parameter integer C_RESET_SETTLE_CYCLES = 270000,
	parameter integer C_TABLE_COUNT = 257
)(
	input i_clk,
	input i_rst_n,
	input i_sda,
	output o_scl,
	output o_sda_drive_low,
	output o_busy,
	output o_done,
	output o_error
);
	localparam [2:0] ST_DELAY = 3'd0;
	localparam [2:0] ST_LAUNCH = 3'd1;
	localparam [2:0] ST_WAIT = 3'd2;
	localparam [2:0] ST_DONE = 3'd3;
	localparam [2:0] ST_ERROR = 3'd4;
	localparam [2:0] ST_RESET_WAIT = 3'd5;
	reg [2:0] state_current;
	reg [31:0] delay_count;
	reg [8:0] reg_index;
	reg master_start;
	reg [15:0] reg_addr;
	reg [7:0] reg_data;
	wire master_busy;
	wire master_done;
	wire master_ack_error;

	always @(*) begin
		reg_addr = 16'h0000;
		reg_data = 8'h00;
		case (reg_index)
`include "ov5640_reg_table.vh"
			default: begin reg_addr = 16'h0000; reg_data = 8'h00; end
		endcase
	end

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			state_current <= ST_DELAY;
			delay_count <= 0;
			reg_index <= 0;
			master_start <= 1'b0;
		end else begin
			master_start <= 1'b0;
			case (state_current)
				ST_DELAY: if (delay_count >= C_START_DELAY-1) begin
					delay_count <= 0;
					state_current <= ST_LAUNCH;
				end else delay_count <= delay_count + 1'b1;
				ST_LAUNCH: begin
					master_start <= 1'b1;
					state_current <= ST_WAIT;
				end
				ST_WAIT: if (master_done) begin
					if (master_ack_error) state_current <= ST_ERROR;
					// 3008=82 is table index 1; wait for the sensor's software
					// reset to settle before launching index 2 (3008=42).
					else if (reg_index == 1 && C_RESET_SETTLE_CYCLES != 0) begin
						delay_count <= 0;
						state_current <= ST_RESET_WAIT;
					end
					else if (reg_index == C_TABLE_COUNT-1) state_current <= ST_DONE;
					else begin reg_index <= reg_index + 1'b1; state_current <= ST_LAUNCH; end
				end
				ST_RESET_WAIT: if (delay_count >= C_RESET_SETTLE_CYCLES-1) begin
					delay_count <= 0;
					reg_index <= reg_index + 1'b1;
					state_current <= ST_LAUNCH;
				end else delay_count <= delay_count + 1'b1;
				ST_DONE: state_current <= ST_DONE;
				ST_ERROR: state_current <= ST_ERROR;
				default: state_current <= ST_DELAY;
			endcase
		end
	end

	assign o_busy = (state_current != ST_DONE) && (state_current != ST_ERROR);
	assign o_done = (state_current == ST_DONE);
	assign o_error = (state_current == ST_ERROR);

	i2c_master_write #(
		.C_HALF_PERIOD(C_HALF_PERIOD),
		.C_REG_ADDR_HIGH_FIRST(1)
	) u_ov5640_i2c_master(
		.i_clk(i_clk), .i_rst_n(i_rst_n), .i_start(master_start),
		.i_device_addr(7'h3c), .i_reg_addr(reg_addr), .i_data(reg_data),
		.i_sda(i_sda), .o_busy(master_busy), .o_done(master_done),
		.o_ack_error(master_ack_error), .o_scl(o_scl),
		.o_sda_drive_low(o_sda_drive_low)
	);
endmodule
