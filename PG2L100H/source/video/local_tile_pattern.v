//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 产生四宫格本地诊断图案，各 source_id 使用不同固定可辨颜色。
// Module Name: local_tile_pattern
// File Name: source/video/local_tile_pattern.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 产生四宫格本地诊断图案，各 source_id 使用不同固定可辨颜色。
// tile 内坐标生成网格和棋盘格，帮助检查画面边界、位置及源路由。
//
// Dependencies: 无子模块；quad_video_window 提供 tile 编号和坐标。
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
// Module Name: local_tile_pattern
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: Q1 四宫格独立验证图案。四路 source_id 使用固定可辨颜色；
//              tile 内坐标用于网格/棋盘格，便于边界与源选择自检。
//////////////////////////////////////////////////////////////////////////////////
module local_tile_pattern(
	input [1:0] i_tile_id,
	input [9:0] i_x,
	input [8:0] i_y,
	output reg [23:0] o_rgb
);
	reg [23:0] base_rgb;
	reg [7:0] grad_x;
	reg [7:0] grad_y;
	always @(*) begin
		// The high nibble is the source identity; low bits vary in both axes.
		grad_x = {2'b00, i_x[5:0]};
		grad_y = {2'b00, i_y[5:0]};
		case (i_tile_id)
			2'd0: base_rgb = {8'hc0 + grad_y, grad_x, 8'h10 + i_x[5:4]};
			2'd1: base_rgb = {grad_y, 8'hc0 + grad_x, 8'h10 + i_y[5:4]};
			2'd2: base_rgb = {grad_x, 8'h10 + i_y[5:4], 8'hc0 + grad_y};
			default: base_rgb = ((i_x[5] ^ i_y[5]) == 1'b0) ?
				{8'h40 + grad_x[5:0], 8'h40 + grad_y[5:0], 8'h40 + grad_x[5:0]} :
				{8'ha0 - grad_x[5:0], 8'ha0 - grad_y[5:0], 8'ha0 - grad_x[5:0]};
		endcase
		// White markers at all four corners and sparse axes make orientation visible.
		if (((i_x < 10'd16) || (i_x >= 10'd624)) &&
			((i_y < 9'd16) || (i_y >= 9'd304))) o_rgb = 24'hffffff;
		else if ((i_x[5:0] == 6'd8) || (i_y[5:0] == 6'd8)) o_rgb = 24'hffffff;
		else o_rgb = base_rgb;
	end
endmodule
