//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 旧版单窗口 720p60 时序发生器，在 x=320..959、y=200..519 显示 640×320 RGB565 图像，其余有效区填黑。
// Module Name: video_window
// File Name: source/hdmi/video_window.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 旧版单窗口 720p60 时序发生器，在 x=320..959、y=200..519 显示 640×320 RGB565 图像，其余有效区填黑。
// 同步帧就绪状态并在帧起点使能显示，输出取像素请求、帧边界及寄存的 RGB/同步信号。
//
// Dependencies: 无子模块；上层提供已缓存帧的 RGB565 像素及 valid。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// 720p60 timing generator with a native-size 640x320 RGB565 window at the
// C5 contract position: 320 <= x < 960, 200 <= y < 520.
module video_window #(
	parameter integer C_H_ACTIVE = 1280,
	parameter integer C_H_FP = 110,
	parameter integer C_H_SYNC = 40,
	parameter integer C_H_BP = 220,
	parameter integer C_V_ACTIVE = 720,
	parameter integer C_V_FP = 5,
	parameter integer C_V_SYNC = 5,
	parameter integer C_V_BP = 20,
	parameter integer C_X_START = 320,
	parameter integer C_Y_START = 200,
	parameter integer C_WINDOW_WIDTH = 640,
	parameter integer C_WINDOW_HEIGHT = 320
)(
	input i_pixel_clk,
	input i_rst_n,
	input i_frame_ready,
	input i_pixel_valid,
	input [15:0] i_pixel_rgb565,
	output o_pixel_pop,
	output o_frame_boundary,
	output o_pclk,
	output reg o_vs,
	output reg o_hs,
	output reg o_de,
	output reg [23:0] o_rgb
);
	localparam integer C_H_TOTAL = C_H_ACTIVE + C_H_FP + C_H_SYNC + C_H_BP;
	localparam integer C_V_TOTAL = C_V_ACTIVE + C_V_FP + C_V_SYNC + C_V_BP;
	reg [11:0] x;
	reg [11:0] y;
	reg [1:0] frame_ready_sync;
	reg display_enable;
	wire in_window;
	wire [7:0] red8;
	wire [7:0] green8;
	wire [7:0] blue8;

	assign o_pclk = i_pixel_clk;
	// x/y 为当前正在输出的像素坐标；该脉冲位于完整垂直消隐后的首拍，
	// 到 y=200 的源图窗口仍有约 4.4 ms，足够完成 DDR 读槽切换与预取。
	assign o_frame_boundary = (x == 12'd0) && (y == 12'd0);
	assign in_window = (x >= C_X_START) && (x < C_X_START + C_WINDOW_WIDTH) &&
				   (y >= C_Y_START) && (y < C_Y_START + C_WINDOW_HEIGHT);
	assign o_pixel_pop = in_window && display_enable;
	assign red8 = {i_pixel_rgb565[15:11], i_pixel_rgb565[15:13]};
	assign green8 = {i_pixel_rgb565[10:5], i_pixel_rgb565[10:9]};
	assign blue8 = {i_pixel_rgb565[4:0], i_pixel_rgb565[4:2]};

	always @(posedge i_pixel_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			x <= 12'd0;
			y <= 12'd0;
			frame_ready_sync <= 2'b00;
			display_enable <= 1'b0;
			o_vs <= 1'b0;
			o_hs <= 1'b0;
			o_de <= 1'b0;
			o_rgb <= 24'd0;
		end else begin
			frame_ready_sync <= {frame_ready_sync[0], i_frame_ready};
			if ((x == 12'd0) && (y == 12'd0) && frame_ready_sync[1]) display_enable <= 1'b1;
			o_de <= (x < C_H_ACTIVE) && (y < C_V_ACTIVE);
			o_hs <= (x >= C_H_ACTIVE + C_H_FP) && (x < C_H_ACTIVE + C_H_FP + C_H_SYNC);
			o_vs <= (y >= C_V_ACTIVE + C_V_FP) && (y < C_V_ACTIVE + C_V_FP + C_V_SYNC);
			if (in_window && display_enable && i_pixel_valid) o_rgb <= {red8, green8, blue8};
			else o_rgb <= 24'd0;
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
