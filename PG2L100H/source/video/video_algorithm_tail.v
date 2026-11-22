//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将高斯、灰度中值和 Sobel 三个阶段真实串联，各阶段可独立选择与处理分支等延迟的旁路。
// Module Name: video_algorithm_tail
// File Name: source/video/video_algorithm_tail.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将高斯、灰度中值和 Sobel 三个阶段真实串联，各阶段可独立选择与处理分支等延迟的旁路。
// 三段各 12 拍合计固定 36 拍，完整 tag 与 RGB 共同输出，接入 HDMI 与 PCIe 共用视频路径。
//
// Dependencies: video_stage_tagged；上游已完成的去雾像素及 tag。
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
// Create Date: 2026-10-10
// Design Name: HDMI Serial Algorithm Tail
// Module Name: video_algorithm_tail
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 高斯→灰度中值→Sobel真串联，独立同延迟旁路；总36拍，无适配器。
// Dependencies: video_stage_tagged；输入为已完成的 Pango 去雾输出。
// Revision: 1.0 - 已由 video_algorithm_chain 接入 HDMI 与 PCIe 共用旁路。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module video_algorithm_tail
#(
	parameter integer C_FRAME_WIDTH = 1280,
	parameter integer C_FRAME_HEIGHT = 720
)
(
	input i_clk,
	input i_rst_n,
	input [23:0] i_rgb,
	input [159:0] i_tag,
	output [23:0] o_rgb,
	output [159:0] o_tag
);
	wire [23:0] gaussian_rgb;
	wire [23:0] median_rgb;
	wire [159:0] gaussian_tag;
	wire [159:0] median_tag;
	video_stage_tagged #(
		.C_KIND(0),
		.C_FRAME_WIDTH(C_FRAME_WIDTH),
		.C_FRAME_HEIGHT(C_FRAME_HEIGHT)
	) video_stage_tagged_Inst_gaussian(
		.i_clk(i_clk),
		.i_rst_n(i_rst_n),
		.i_rgb(i_rgb),
		.i_tag(i_tag),
		.o_rgb(gaussian_rgb),
		.o_tag(gaussian_tag)
	);
	video_stage_tagged #(
		.C_KIND(1),
		.C_FRAME_WIDTH(C_FRAME_WIDTH),
		.C_FRAME_HEIGHT(C_FRAME_HEIGHT)
	) video_stage_tagged_Inst_median(
		.i_clk(i_clk),
		.i_rst_n(i_rst_n),
		.i_rgb(gaussian_rgb),
		.i_tag(gaussian_tag),
		.o_rgb(median_rgb),
		.o_tag(median_tag)
	);
	video_stage_tagged #(
		.C_KIND(2),
		.C_FRAME_WIDTH(C_FRAME_WIDTH),
		.C_FRAME_HEIGHT(C_FRAME_HEIGHT)
	) video_stage_tagged_Inst_sobel(
		.i_clk(i_clk),
		.i_rst_n(i_rst_n),
		.i_rgb(median_rgb),
		.i_tag(median_tag),
		.o_rgb(o_rgb),
		.o_tag(o_tag)
	);
endmodule
