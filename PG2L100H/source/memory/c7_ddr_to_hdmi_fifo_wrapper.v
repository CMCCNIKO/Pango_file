//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-11
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 旧 C7 DDR 到 HDMI 异步 FIFO 及 RGB565 串行化封装，每个 256-bit word 按低位优先输出 16 个像素。
// Module Name: c7_ddr_to_hdmi_fifo_wrapper
// File Name: source/memory/c7_ddr_to_hdmi_fifo_wrapper.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 旧 C7 DDR 到 HDMI 异步 FIFO 及 RGB565 串行化封装，每个 256-bit word 按低位优先输出 16 个像素。
// 切换显示槽时写侧清 FIFO，读侧确认写复位后同步清 FIFO/serializer，避免跨帧残留。
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
// Module Name: c7_ddr_to_hdmi_fifo_wrapper
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: C7 DDR 到 HDMI 官方 FIFO 封装。每次 display_switch 在写侧
//              清空 FIFO，读侧在确认写复位后同步清空 serializer，确保像素
//              FIFO 绝不把旧 DISPLAYING 槽的尾数据混进新槽。
//
// Dependencies: ipcore/c5_ddr_to_hdmi_fifo/c5_ddr_to_hdmi_fifo.idf
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module c7_ddr_to_hdmi_fifo_wrapper(
	input i_ddr_clk,
	input i_ddr_rst_n,
	input i_frame_switch,
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
	reg [1:0] reg_measure_enable_sync;
	reg [1:0] reg_fifo_wr_rst_sync;
	reg [1:0] reg_fifo_wr_release_sync;
	reg [1:0] reg_fifo_rd_rst_sync;
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

	assign fifo_wr_rst = reg_fifo_wr_rst_sync[1];
	assign fifo_rd_rst = reg_fifo_rd_rst_sync[1];
	assign fifo_rd_en = !fifo_rd_rst && !fifo_empty && !reg_fifo_rd_pending &&
						(reg_word_count != 2'd2);
	assign pixel_take = i_pixel_pop && (reg_word_count != 2'd0);
	assign word_last = pixel_take && (reg_pixel_index == 4'd15);
	assign o_pixel_valid = (reg_word_count != 2'd0);
	assign o_pixel_rgb565 = (reg_word_count == 2'd0) ? 16'h0000 :
						reg_current_word[reg_pixel_index * 16 +: 16];
	assign o_wr_water_level = {6'd0, fifo_wr_water_level};
	assign o_rd_water_level = {6'd0, fifo_rd_water_level};

	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if (!i_ddr_rst_n) begin
			reg_fifo_wr_rst_sync <= 2'b11;
		end else if (i_frame_switch) begin
			reg_fifo_wr_rst_sync <= 2'b11;
		end else begin
			reg_fifo_wr_rst_sync <= {reg_fifo_wr_rst_sync[0], 1'b0};
		end
	end

	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if (!i_pixel_rst_n) begin
			reg_fifo_wr_release_sync <= 2'b00;
			reg_fifo_rd_rst_sync <= 2'b11;
		end else begin
			reg_fifo_wr_release_sync <= {reg_fifo_wr_release_sync[0], !fifo_wr_rst};
			if (!reg_fifo_wr_release_sync[1]) begin
				reg_fifo_rd_rst_sync <= 2'b11;
			end else begin
				reg_fifo_rd_rst_sync <= {reg_fifo_rd_rst_sync[0], 1'b0};
			end
		end
	end

	always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
		if (!i_pixel_rst_n) begin
			reg_pixel_index <= 4'd0;
			reg_measure_enable_sync <= 2'b00;
			o_underflow_count <= 32'd0;
			o_underflow_event <= 1'b0;
			reg_fifo_rd_pending <= 1'b0;
			reg_word_count <= 2'd0;
			reg_current_word <= 256'd0;
			reg_next_word <= 256'd0;
		end else if (fifo_rd_rst) begin
			reg_pixel_index <= 4'd0;
			reg_measure_enable_sync <= 2'b00;
			reg_fifo_rd_pending <= 1'b0;
			reg_word_count <= 2'd0;
			reg_current_word <= 256'd0;
			reg_next_word <= 256'd0;
			o_underflow_event <= 1'b0;
		end else begin
			reg_measure_enable_sync <= {reg_measure_enable_sync[0], i_measure_enable};
			o_underflow_event <= 1'b0;
			reg_fifo_rd_pending <= fifo_rd_en;
			if (i_pixel_pop && (reg_word_count == 2'd0) && reg_measure_enable_sync[1]) begin
				o_underflow_count <= o_underflow_count + 1'b1;
				o_underflow_event <= 1'b1;
			end
			if (reg_fifo_rd_pending && word_last) begin
				reg_current_word <= fifo_data;
				reg_word_count <= 2'd1;
				reg_pixel_index <= 4'd0;
			end else if (reg_fifo_rd_pending) begin
				if (reg_word_count == 2'd0) reg_current_word <= fifo_data;
				else reg_next_word <= fifo_data;
				reg_word_count <= reg_word_count + 1'b1;
				if (pixel_take) reg_pixel_index <= reg_pixel_index + 1'b1;
			end else if (word_last) begin
				reg_pixel_index <= 4'd0;
				if (reg_word_count == 2'd2) reg_current_word <= reg_next_word;
				reg_word_count <= reg_word_count - 1'b1;
			end else if (pixel_take) begin
				reg_pixel_index <= reg_pixel_index + 1'b1;
			end
		end
	end

	c5_ddr_to_hdmi_fifo u_c5_ddr_to_hdmi_fifo(
		.wr_clk(i_ddr_clk), .wr_rst(fifo_wr_rst), .wr_en(i_word_push),
		.wr_data(i_word_data), .wr_full(o_word_full), .wr_water_level(fifo_wr_water_level), .almost_full(),
		.rd_clk(i_pixel_clk), .rd_rst(fifo_rd_rst), .rd_en(fifo_rd_en),
		.rd_data(fifo_data), .rd_empty(fifo_empty), .rd_water_level(fifo_rd_water_level), .almost_empty()
	);
endmodule
