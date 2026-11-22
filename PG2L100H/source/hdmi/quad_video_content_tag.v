//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 封装四宫格视频时序，将显示坐标、tile 编号、tile 内坐标和内容有效标记与已寄存 RGB 同沿对齐。
// Module Name: quad_video_content_tag
// File Name: source/hdmi/quad_video_content_tag.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 封装四宫格视频时序，将显示坐标、tile 编号、tile 内坐标和内容有效标记与已寄存 RGB 同沿对齐。
// 公共时序及取流接口由 quad_video_window 透传；内容有效同时要求真实流、流有效与对应源存在。
//
// Dependencies: quad_video_window；其内部依赖 local_tile_pattern。
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
// Design Name: HDMI 四宫格内容侧带
// Module Name: quad_video_content_tag
// Project Name: PG2L100H_RK3566_Algorithm_for_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 原模块公共输出直接传递；侧带与原 RGB 输出在同一沿注册。
// Dependencies: 原 quad_video_window、local_tile_pattern；无新增 IP。
// Revision: 0.01 - File Created；未修改原模块、顶层或 PCIe。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module quad_video_content_tag
#(
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
	output o_vs,
	output o_hs,
	output o_de,
	output [23:0] o_rgb,
	output o_pixel_pop,
	output o_stream_pop,
	output o_frame_boundary,
	output [11:0] o_x,
	output [11:0] o_y,
	output [1:0] o_tile_id,
	input i_stream_mode,
	input [15:0] i_stream_rgb565,
	input i_stream_valid,
	input [3:0] i_source_present,
	input i_source_present_valid,
	output o_content_valid,
	output [11:0] o_display_x,
	output [11:0] o_display_y,
	output [1:0] o_display_tile_id,
	output [9:0] o_tile_x,
	output [8:0] o_tile_y
);
	reg content_valid_o;
	reg [11:0] display_x_o;
	reg [11:0] display_y_o;
	reg [1:0] display_tile_id_o;
	reg [9:0] tile_x_o;
	reg [8:0] tile_y_o;
	wire [11:0] tile_x_full;
	wire [11:0] tile_y_full;
	assign o_content_valid = content_valid_o;
	assign o_display_x = display_x_o;
	assign o_display_y = display_y_o;
	assign o_display_tile_id = display_tile_id_o;
	assign o_tile_x = tile_x_o;
	assign o_tile_y = tile_y_o;
	assign tile_x_full = (o_x < C_TILE_WIDTH) ? o_x : (o_x - C_TILE_WIDTH);
	assign tile_y_full = (o_y < C_TOP_BAR + C_TILE_HEIGHT) ?
		(o_y - C_TOP_BAR) : (o_y - C_TOP_BAR - C_TILE_HEIGHT);

	// 所有原公共输出保持原模块连接、数值和拍数；o_x/y/tile_id 仍为原未注册接口。
	quad_video_window
	#(
		.C_H_ACTIVE(C_H_ACTIVE),
		.C_H_FP(C_H_FP),
		.C_H_SYNC(C_H_SYNC),
		.C_H_BP(C_H_BP),
		.C_V_ACTIVE(C_V_ACTIVE),
		.C_V_FP(C_V_FP),
		.C_V_SYNC(C_V_SYNC),
		.C_V_BP(C_V_BP),
		.C_TILE_WIDTH(C_TILE_WIDTH),
		.C_TILE_HEIGHT(C_TILE_HEIGHT),
		.C_TOP_BAR(C_TOP_BAR),
		.C_USE_STREAM_DEFAULT(C_USE_STREAM_DEFAULT),
		.C_STREAM_ALL_TILES(C_STREAM_ALL_TILES),
		.C_STREAM_TILE_ID(C_STREAM_TILE_ID)
	) quad_video_window_Inst_baseline(
		.i_pixel_clk(i_pixel_clk),
		.i_rst_n(i_rst_n),
		.o_pclk(o_pclk),
		.o_vs(o_vs),
		.o_hs(o_hs),
		.o_de(o_de),
		.o_rgb(o_rgb),
		.o_pixel_pop(o_pixel_pop),
		.o_stream_pop(o_stream_pop),
		.o_frame_boundary(o_frame_boundary),
		.o_x(o_x),
		.o_y(o_y),
		.o_tile_id(o_tile_id),
		.i_stream_mode(i_stream_mode),
		.i_stream_rgb565(i_stream_rgb565),
		.i_stream_valid(i_stream_valid)
	);

	// 采样的是本沿之前的坐标/pop，正好与原模块本沿注册的 RGB 相配。
	// stream_pop 已包含真实 stream 模式/tile 选择；RGB=0 不表示来源无效。
	always@(posedge i_pixel_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			content_valid_o <= 1'b0;
			display_x_o <= 12'd0;
			display_y_o <= 12'd0;
			display_tile_id_o <= 2'd0;
			tile_x_o <= 10'd0;
			tile_y_o <= 9'd0;
		end else begin
			content_valid_o <= o_stream_pop && i_stream_valid && i_source_present_valid && i_source_present[o_tile_id];
			display_x_o <= o_x;
			display_y_o <= o_y;
			display_tile_id_o <= o_tile_id;
			if(o_pixel_pop)begin
				tile_x_o <= tile_x_full[9:0];
				tile_y_o <= tile_y_full[8:0];
			end else begin
				tile_x_o <= 10'd0;
				tile_y_o <= 9'd0;
			end
		end
	end
endmodule
