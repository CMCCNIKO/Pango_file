//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-11-16（v028 连续自适应去雾）
// Design Name: 在共享去雾之前串联通道增益、共享亮度空间处理、色调 LUT 与饱和度处理，连续吞吐 II=1。
// Module Name: isp_pipeline
// File Name: source/video/isp/isp_pipeline.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在共享去雾之前串联通道增益、共享亮度空间处理、色调 LUT 与饱和度处理，连续吞吐 II=1。
// 含消隐的完整像素及侧带按固定延迟推进，复位配置为恒等变换，支持增益后独立下游配置。
//
// Dependencies: isp_gain_stage、isp_spatial、isp_tone_sat_stage 及其官方 IP。
//
// Revision:
// 2026-11-16 / v028 - 连续自动更新原因与有效epoch同拍传递，保持原像素延迟。
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-19
// Design Name: Streaming color ISP before shared dehaze
// Module Name: isp_pipeline
// Project Name: PG2L100H_AVM_ISP
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Gain -> shared spatial luma -> tone/saturation, II=1.
//              Every sideband follows the same fixed delay, including blanking.
// Dependencies: isp_gain_stage, isp_spatial, isp_tone_sat_stage and generated IP.
// Revision: 0.01 - Frame-stable configurations, identity on reset.
//           0.02 - Optional downstream-only configuration after gain.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module isp_pipeline
#(
	parameter integer C_H_TOTAL = 1650,
	parameter integer C_H_ACTIVE = 1280,
	parameter integer C_V_ACTIVE = 720,
	parameter integer C_TILE_WIDTH = 640,
	parameter integer C_TILE_HEIGHT = 320,
	parameter integer C_TOP_BAR = 40,
	parameter integer C_COMPACT_CONFIG = 0
)
(
	input i_clk,
	input i_rstn,
	input [23:0] i_rgb,
	input [159:0] i_tag,
	input i_frame_boundary,
	input [63:0] i_algo_config,
	input [519:0] i_isp_config,
	input [63:0] i_effective_epoch,
	input i_isp_continuous_update,
	input [3:0] i_source_alive,
	input i_source_alive_valid,
	output [23:0] o_rgb,
	output [159:0] o_tag,
	output o_frame_boundary,
	output [63:0] o_algo_config,
	output [519:0] o_isp_config,
	output [63:0] o_effective_epoch,
	output o_isp_continuous_update,
	output [3:0] o_source_alive,
	output o_source_alive_valid
);
	localparam [127:0] C_DOWNSTREAM_TILE_MASK =
		{1'b0,{7{1'b1}},54'd0,{46{1'b1}},20'd0};
	localparam [519:0] C_DOWNSTREAM_CONFIG_MASK =
		{8'h1f,{4{C_DOWNSTREAM_TILE_MASK}}};
	wire [23:0] rgb_bus [0:3];
	wire [159:0] tag_bus [0:3];
	wire boundary_bus [0:3];
	wire [63:0] algo_config_bus [0:3];
	wire [519:0] isp_config_bus [0:3];
	wire [519:0] spatial_isp_config;
	wire [63:0] epoch_bus [0:3];
	wire continuous_bus [0:3];
	wire [3:0] alive_bus [0:3];
	wire alive_valid_bus [0:3];
	assign rgb_bus[0] = i_rgb;
	assign tag_bus[0] = i_tag;
	assign boundary_bus[0] = i_frame_boundary;
	assign algo_config_bus[0] = i_algo_config;
	assign isp_config_bus[0] = i_isp_config;
	assign epoch_bus[0] = i_effective_epoch;
	assign continuous_bus[0] = i_isp_continuous_update;
	assign alive_bus[0] = i_source_alive;
	assign alive_valid_bus[0] = i_source_alive_valid;
	// Gain parameters are consumed before spatial; six enables and the v025
	// natural-tone flag(bit126) must survive compact-config forwarding.
	// Full mode retains the complete configuration port contract by default.
	assign spatial_isp_config = C_COMPACT_CONFIG != 0 ?
		(isp_config_bus[1] & C_DOWNSTREAM_CONFIG_MASK) : isp_config_bus[1];
	assign o_rgb = rgb_bus[3];
	assign o_tag = tag_bus[3];
	assign o_frame_boundary = boundary_bus[3];
	assign o_algo_config = algo_config_bus[3];
	assign o_isp_config = isp_config_bus[3];
	assign o_effective_epoch = epoch_bus[3];
	assign o_isp_continuous_update = continuous_bus[3];
	assign o_source_alive = alive_bus[3];
	assign o_source_alive_valid = alive_valid_bus[3];
	isp_gain_stage isp_gain_stage_Inst_gain(
		.i_clk(i_clk),.i_rstn(i_rstn),.i_rgb(rgb_bus[0]),.i_tag(tag_bus[0]),
		.i_frame_boundary(boundary_bus[0]),.i_algo_config(algo_config_bus[0]),
		.i_isp_config(isp_config_bus[0]),.i_effective_epoch(epoch_bus[0]),
		.i_isp_continuous_update(continuous_bus[0]),
		.i_source_alive(alive_bus[0]),.i_source_alive_valid(alive_valid_bus[0]),
		.o_rgb(rgb_bus[1]),.o_tag(tag_bus[1]),.o_frame_boundary(boundary_bus[1]),
		.o_algo_config(algo_config_bus[1]),.o_isp_config(isp_config_bus[1]),
		.o_effective_epoch(epoch_bus[1]),
		.o_isp_continuous_update(continuous_bus[1]),.o_source_alive(alive_bus[1]),
		.o_source_alive_valid(alive_valid_bus[1]));
	isp_spatial #(.C_H_TOTAL(C_H_TOTAL),.C_H_ACTIVE(C_H_ACTIVE),
		.C_V_ACTIVE(C_V_ACTIVE),.C_TILE_WIDTH(C_TILE_WIDTH),
		.C_TILE_HEIGHT(C_TILE_HEIGHT),.C_TOP_BAR(C_TOP_BAR)) isp_spatial_Inst_spatial(
		.i_clk(i_clk),.i_rstn(i_rstn),.i_rgb(rgb_bus[1]),.i_tag(tag_bus[1]),
		.i_frame_boundary(boundary_bus[1]),.i_algo_config(algo_config_bus[1]),
		.i_isp_config(spatial_isp_config),.i_effective_epoch(epoch_bus[1]),
		.i_isp_continuous_update(continuous_bus[1]),
		.i_source_alive(alive_bus[1]),.i_source_alive_valid(alive_valid_bus[1]),
		.o_rgb(rgb_bus[2]),.o_tag(tag_bus[2]),.o_frame_boundary(boundary_bus[2]),
		.o_algo_config(algo_config_bus[2]),.o_isp_config(isp_config_bus[2]),
		.o_effective_epoch(epoch_bus[2]),
		.o_isp_continuous_update(continuous_bus[2]),.o_source_alive(alive_bus[2]),
		.o_source_alive_valid(alive_valid_bus[2]));
	isp_tone_sat_stage isp_tone_sat_stage_Inst_tone_sat(
		.i_clk(i_clk),.i_rstn(i_rstn),.i_rgb(rgb_bus[2]),.i_tag(tag_bus[2]),
		.i_frame_boundary(boundary_bus[2]),.i_algo_config(algo_config_bus[2]),
		.i_isp_config(isp_config_bus[2]),.i_effective_epoch(epoch_bus[2]),
		.i_isp_continuous_update(continuous_bus[2]),
		.i_source_alive(alive_bus[2]),.i_source_alive_valid(alive_valid_bus[2]),
		.o_rgb(rgb_bus[3]),.o_tag(tag_bus[3]),.o_frame_boundary(boundary_bus[3]),
		.o_algo_config(algo_config_bus[3]),.o_isp_config(isp_config_bus[3]),
		.o_effective_epoch(epoch_bus[3]),
		.o_isp_continuous_update(continuous_bus[3]),.o_source_alive(alive_bus[3]),
		.o_source_alive_valid(alive_valid_bus[3]));
endmodule
