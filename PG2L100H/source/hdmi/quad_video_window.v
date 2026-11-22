//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 生成独立 1280×720 HDMI 栅格，中央上下两排各放两个 640×320 tile，顶部及底部各留 40 行黑条。
// Module Name: quad_video_window
// File Name: source/hdmi/quad_video_window.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 生成独立 1280×720 HDMI 栅格，中央上下两排各放两个 640×320 tile，顶部及底部各留 40 行黑条。
// tile 按左上、右上、左下、右下编号，可选择本地图案或 RGB565 输入流；DE 覆盖整个有效栅格，RGB/HS/VS/DE 同沿寄存。
//
// Dependencies: local_tile_pattern。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Module Name: quad_video_window
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: Q1 standalone 1280x720 timing and four local 640x320 tiles.
//              Active video remains DE=1 for the full 1280x720 raster; y=0..39
//              and y=680..719 are black bars. Tile pixels assert o_pixel_pop and
//              select source_id 0..3 in TL/TR/BL/BR order. Set i_stream_mode or
//              C_USE_STREAM_DEFAULT to display the supplied RGB565 stream instead.
//////////////////////////////////////////////////////////////////////////////////
module quad_video_window #(
	parameter integer C_H_ACTIVE = 1280,
	parameter integer C_H_FP = 110,
	parameter integer C_H_SYNC = 40,
	parameter integer C_H_BP = 220,
	parameter integer C_V_ACTIVE = 720,
	parameter integer C_V_FP = 5,
	parameter integer C_V_SYNC = 5,
	parameter integer C_V_BP = 20,
	parameter integer C_TILE_WIDTH = 640,
	parameter integer C_TILE_HEIGHT = 320,
	parameter integer C_TOP_BAR = 40,
	parameter integer C_USE_STREAM_DEFAULT = 0,
	parameter integer C_STREAM_ALL_TILES = 0,
	parameter integer C_STREAM_TILE_ID = 0
)(
	input i_pixel_clk,
	input i_rst_n,
	output o_pclk,
	output reg o_vs,
	output reg o_hs,
	output reg o_de,
	output reg [23:0] o_rgb,
	output o_pixel_pop,
	output o_stream_pop,
	output o_frame_boundary,
	output [11:0] o_x,
	output [11:0] o_y,
	output [1:0] o_tile_id,
	input i_stream_mode,
	input [15:0] i_stream_rgb565,
	input i_stream_valid
);
	localparam integer C_H_TOTAL = C_H_ACTIVE + C_H_FP + C_H_SYNC + C_H_BP;
	localparam integer C_V_TOTAL = C_V_ACTIVE + C_V_FP + C_V_SYNC + C_V_BP;
	reg [11:0] x;
	reg [11:0] y;
	wire in_active;
	wire in_tile;
	wire [1:0] tile_id;
	wire [9:0] tile_x;
	wire [8:0] tile_y;
	wire [23:0] tile_rgb;
	wire [23:0] stream_rgb;
	wire use_stream;

	assign o_pclk = i_pixel_clk;
	assign o_x = x;
	assign o_y = y;
	assign o_frame_boundary = i_rst_n && (x == 12'd0) && (y == 12'd0);
	assign in_active = i_rst_n && (x < C_H_ACTIVE) && (y < C_V_ACTIVE);
	assign in_tile = in_active && (y >= C_TOP_BAR) &&
		(y < C_TOP_BAR + 2*C_TILE_HEIGHT);
	assign tile_id = (y < C_TOP_BAR + C_TILE_HEIGHT) ?
		((x < C_TILE_WIDTH) ? 2'd0 : 2'd1) :
		((x < C_TILE_WIDTH) ? 2'd2 : 2'd3);
	assign tile_x = (x < C_TILE_WIDTH) ? x[9:0] : (x - C_TILE_WIDTH);
	assign tile_y = (y < C_TOP_BAR + C_TILE_HEIGHT) ?
		(y - C_TOP_BAR) : (y - C_TOP_BAR - C_TILE_HEIGHT);
	assign o_pixel_pop = in_tile;
	assign o_stream_pop = in_tile && use_stream;
	assign o_tile_id = in_tile ? tile_id : 2'd0;
	assign use_stream = in_tile && ((C_STREAM_ALL_TILES != 0) || (tile_id == C_STREAM_TILE_ID[1:0])) &&
		(((C_USE_STREAM_DEFAULT != 0) ? 1'b1 : i_stream_mode));
	// Match video_window.v: replicate high bits when expanding RGB565.
	assign stream_rgb = {i_stream_rgb565[15:11], i_stream_rgb565[15:13],
		 i_stream_rgb565[10:5], i_stream_rgb565[10:9],
		 i_stream_rgb565[4:0], i_stream_rgb565[4:2]};

	local_tile_pattern u_local_tile_pattern(
		.i_tile_id(tile_id),
		.i_x(tile_x),
		.i_y(tile_y),
		.o_rgb(tile_rgb)
	);


	// Match the official MS7210 loopback interface: register every parallel
	// video signal on the outgoing pixel-clock edge. This removes the long
	// FIFO-index/color-expand/window combinational path from the package pins
	// and gives RGB/control the same clock-to-output phase.
	always @(posedge i_pixel_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			o_vs <= 1'b0;
			o_hs <= 1'b0;
			o_de <= 1'b0;
			o_rgb <= 24'h000000;
		end else begin
			o_de <= in_active;
			o_hs <= (x >= C_H_ACTIVE + C_H_FP) &&
				(x < C_H_ACTIVE + C_H_FP + C_H_SYNC);
			o_vs <= (y >= C_V_ACTIVE + C_V_FP) &&
				(y < C_V_ACTIVE + C_V_FP + C_V_SYNC);
			if (in_tile && use_stream) begin
				if (i_stream_valid) o_rgb <= stream_rgb;
				else o_rgb <= 24'h000000;
			end else if (in_tile) begin
				o_rgb <= tile_rgb;
			end else begin
				o_rgb <= 24'h000000;
			end
		end
	end

	always @(posedge i_pixel_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			x <= 12'd0;
			y <= 12'd0;
		end else begin
			if (x == C_H_TOTAL - 1) begin
				x <= 12'd0;
				if (y == C_V_TOTAL - 1) y <= 12'd0;
				else y <= y + 1'b1;
			end else begin
				x <= x + 1'b1;
			end
		end
	end
endmodule
