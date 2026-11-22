//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-11
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 四宫格 DDR 到 HDMI 官方异步 FIFO 包装与 RGB565 serializer，每个 256-bit 字低像素在前输出 16 个像素。
// Module Name: quad_ddr_to_hdmi_fifo_wrapper
// File Name: source/memory/quad_ddr_to_hdmi_fifo_wrapper.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 四宫格 DDR 到 HDMI 官方异步 FIFO 包装与 RGB565 serializer，每个 256-bit 字低像素在前输出 16 个像素。
// display_switch 触发双域有序 FIFO flush，并清除像素 serializer 的旧槽尾部，再接受新帧。
//
// Dependencies: c5_ddr_to_hdmi_fifo 官方 FIFO IP。
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
// Create Date: 2026-09-11
// Module Name: quad_ddr_to_hdmi_fifo_wrapper
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: C7 DDR 到 HDMI 官方 FIFO 封装。每次 display_switch 在写侧
//              清空 FIFO，读侧在确认写复位后同步清空 serializer，确保像素
//              FIFO 绝不把旧 DISPLAYING 槽的尾数据混进新槽。
//
// Dependencies: ipcore/c5_ddr_to_hdmi_fifo/c5_ddr_to_hdmi_fifo.idf
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module quad_ddr_to_hdmi_fifo_wrapper(
	input i_ddr_clk,
	input i_ddr_rst_n,
	input i_frame_switch,
	output o_flush_busy,
	output o_flush_done,
	input i_word_push,
	input [255:0] i_word_data,
	output o_word_full,
	input i_pixel_clk,
	input i_pixel_rst_n,
	input i_measure_enable,
	input i_pixel_pop,
	output [15:0] o_pixel_rgb565,
	output o_pixel_valid,
	output reg [31:0] o_underflow_count,
	output reg o_underflow_event,
	output [15:0] o_wr_water_level,
	output [15:0] o_rd_water_level
);
	reg [3:0] reg_pixel_index;
	reg reg_measure_enable_meta /* synthesis syn_preserve = 1 */;
	reg reg_measure_enable_sync /* synthesis syn_preserve = 1 */;
	localparam [1:0] ST_RESET_ASSERT = 2'd0;
	localparam [1:0] ST_RESET_RELEASE = 2'd1;
	localparam [1:0] ST_RESET_IDLE = 2'd2;
	reg [1:0] reset_state;
	reg reg_pixel_reset_ack_meta /* synthesis syn_preserve = 1 */;
	reg reg_pixel_reset_ack_sync /* synthesis syn_preserve = 1 */;
	reg flush_done_o;
	reg flag_frame_poisoned;
	reg reg_fifo_rd_rst_meta /* synthesis syn_preserve = 1 */;
	reg reg_fifo_rd_rst_sync /* synthesis syn_preserve = 1 */;
	reg reg_fifo_rd_pending;
	reg [1:0] reg_word_count;
	reg [255:0] reg_current_word;
	reg [255:0] reg_next_word;
	wire fifo_empty;
	wire [255:0] fifo_data;
	wire fifo_rd_en;
	wire fifo_wr_rst;
	wire fifo_rd_rst;
	wire pixel_take;
	wire word_last;
	wire [9:0] fifo_wr_water_level;
	wire [9:0] fifo_rd_water_level;
	wire fifo_full;

	assign fifo_wr_rst = (reset_state == ST_RESET_ASSERT);
	assign fifo_rd_rst = reg_fifo_rd_rst_sync;
	assign o_flush_busy = (reset_state != ST_RESET_IDLE);
	assign o_flush_done = flush_done_o;
	assign o_word_full = fifo_full || o_flush_busy;
	assign fifo_rd_en = !fifo_rd_rst && !flag_frame_poisoned && !fifo_empty && !reg_fifo_rd_pending &&
						(reg_word_count != 2'd2);
	assign pixel_take = i_pixel_pop && o_pixel_valid;
	assign word_last = pixel_take && (reg_pixel_index == 4'd15);
	assign o_pixel_valid = !fifo_rd_rst && !flag_frame_poisoned && (reg_word_count != 2'd0);
	assign o_pixel_rgb565 = !o_pixel_valid ? 16'h0000 :
						reg_current_word[reg_pixel_index * 16 +: 16];
	assign o_wr_water_level = {6'd0, fifo_wr_water_level};
	assign o_rd_water_level = {6'd0, fifo_rd_water_level};

	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if (!i_ddr_rst_n) begin
			reset_state <= ST_RESET_ASSERT;
			reg_pixel_reset_ack_meta <= 1'b0;
			reg_pixel_reset_ack_sync <= 1'b0;
			flush_done_o <= 1'b0;
		end else begin
			reg_pixel_reset_ack_meta <= fifo_rd_rst;
			reg_pixel_reset_ack_sync <= reg_pixel_reset_ack_meta;
			flush_done_o <= 1'b0;
			case (reset_state)
				ST_RESET_ASSERT: begin
					if (reg_pixel_reset_ack_sync) begin reset_state <= ST_RESET_RELEASE; end
				end
				ST_RESET_RELEASE: begin
					if (!reg_pixel_reset_ack_sync) begin
						reset_state <= ST_RESET_IDLE;
						flush_done_o <= 1'b1;
					end
				end
				ST_RESET_IDLE: begin
					if (i_frame_switch) begin reset_state <= ST_RESET_ASSERT; end
				end
				default: begin reset_state <= ST_RESET_ASSERT; end
			endcase
		end
	end

	// Payload ownership is cleared by reg_word_count/pending on reset/flush.
	// The existing valid mask hides old data until a new FIFO return owns it.
	always @(posedge i_pixel_clk) begin
		if(i_pixel_rst_n && !fifo_rd_rst)begin
			if(reg_fifo_rd_pending && word_last)begin
				reg_current_word <= fifo_data;
			end else if(reg_fifo_rd_pending)begin
				if(reg_word_count == 2'd0)reg_current_word <= fifo_data;
				else reg_next_word <= fifo_data;
			end else if(word_last && reg_word_count == 2'd2)begin
				reg_current_word <= reg_next_word;
			end
		end
	end

	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if (!i_pixel_rst_n) begin
			reg_fifo_rd_rst_meta <= 1'b1;
			reg_fifo_rd_rst_sync <= 1'b1;
		end else begin
			reg_fifo_rd_rst_meta <= fifo_wr_rst;
			reg_fifo_rd_rst_sync <= reg_fifo_rd_rst_meta;
		end
	end

	// 一次欠流后本输出帧保持黑色，不把后到数据错位显示到余下像素。
	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if (!i_pixel_rst_n) begin flag_frame_poisoned <= 1'b0; end
		else if (fifo_rd_rst) begin flag_frame_poisoned <= 1'b0; end
		else if (i_pixel_pop && (reg_word_count == 0) && reg_measure_enable_sync) begin
			flag_frame_poisoned <= 1'b1;
		end
	end

	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if (!i_pixel_rst_n) begin
			reg_pixel_index <= 4'd0;
			reg_measure_enable_meta <= 1'b0;
			reg_measure_enable_sync <= 1'b0;
			o_underflow_count <= 32'd0;
			o_underflow_event <= 1'b0;
			reg_fifo_rd_pending <= 1'b0;
			reg_word_count <= 2'd0;
		end else if (fifo_rd_rst) begin
			reg_pixel_index <= 4'd0;
			reg_measure_enable_meta <= 1'b0;
			reg_measure_enable_sync <= 1'b0;
			reg_fifo_rd_pending <= 1'b0;
			reg_word_count <= 2'd0;
			o_underflow_event <= 1'b0;
		end else begin
			reg_measure_enable_meta <= i_measure_enable;
			reg_measure_enable_sync <= reg_measure_enable_meta;
			o_underflow_event <= 1'b0;
			reg_fifo_rd_pending <= fifo_rd_en;
			if (i_pixel_pop && (reg_word_count == 2'd0) && reg_measure_enable_sync && !flag_frame_poisoned) begin
				o_underflow_count <= o_underflow_count + 1'b1;
				o_underflow_event <= 1'b1;
			end
			if (reg_fifo_rd_pending && word_last) begin
				reg_word_count <= 2'd1;
				reg_pixel_index <= 4'd0;
			end else if (reg_fifo_rd_pending) begin
				reg_word_count <= reg_word_count + 1'b1;
				if (pixel_take) reg_pixel_index <= reg_pixel_index + 1'b1;
			end else if (word_last) begin
				reg_pixel_index <= 4'd0;
				reg_word_count <= reg_word_count - 1'b1;
			end else if (pixel_take) begin
				reg_pixel_index <= reg_pixel_index + 1'b1;
			end
		end
	end

	c5_ddr_to_hdmi_fifo u_c5_ddr_to_hdmi_fifo(
		.wr_clk(i_ddr_clk), .wr_rst(fifo_wr_rst), .wr_en(i_word_push && !o_word_full),
		.wr_data(i_word_data), .wr_full(fifo_full), .wr_water_level(fifo_wr_water_level), .almost_full(),
		.rd_clk(i_pixel_clk), .rd_rst(fifo_rd_rst), .rd_en(fifo_rd_en),
		.rd_data(fifo_data), .rd_empty(fifo_empty), .rd_water_level(fifo_rd_water_level), .almost_empty()
	);
endmodule
