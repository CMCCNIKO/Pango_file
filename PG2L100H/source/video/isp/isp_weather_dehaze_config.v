//////////////////////////////////////////////////////////////////////////////////
// Company: HRBUST_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-11-16
// Design Name: 分源天气自动去雾参数覆盖
// Module Name: isp_weather_dehaze_config
// Project Name: PG2L100H_PRJ / v026
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Description:
//   像素域组合覆盖，无新增流水。天气控制器仅在帧边界更新参数。
//   输入位于暗通道2拍之后，同时覆盖像素tag与A估计器配置，避免边界差拍。
//   无雾明确清tile使能走原流水旁路；退出自动模式原手动配置直接恢复。
// Dependencies: isp_weather_control、algo_dehaze_pango
// Revision: 2026-11-16 v026.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module isp_weather_dehaze_config
(
	input i_enable,
	input [3:0] i_valid,
	input [3:0] i_fog_request,
	input [31:0] i_omega,
	input [31:0] i_tmin,
	input [159:0] i_tag,
	input [63:0] i_config,
	output [159:0] o_tag,
	output [63:0] o_config
);
	reg [159:0] tag_o;
	reg [63:0] config_o;
	wire [3:0] selected_mask;
	wire [1:0] tile;
	assign selected_mask = i_valid & i_fog_request;
	assign tile = i_tag[133:132];
	assign o_tag = tag_o;
	assign o_config = config_o;
	always@(*)begin
		tag_o = i_tag;
		config_o = i_config;
		if(i_enable)begin
			tag_o[56] = |selected_mask;
			tag_o[51:48] = selected_mask;
			tag_o[47:40] = i_omega[tile * 8 +: 8];
			tag_o[39:32] = i_tmin[tile * 8 +: 8];
			tag_o[16] = 1'b1;
			config_o[56] = |selected_mask;
			config_o[51:48] = selected_mask;
			config_o[16] = 1'b1;
		end
	end
endmodule
