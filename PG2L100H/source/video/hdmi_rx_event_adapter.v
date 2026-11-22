//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 MS7200 的并行 RGB888、VS/HS/DE 栅格转换为帧、行和有效像素事件，全部保持在 HDMI RX PCLK 域。
// Module Name: hdmi_rx_event_adapter
// File Name: source/video/hdmi_rx_event_adapter.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 MS7200 的并行 RGB888、VS/HS/DE 栅格转换为帧、行和有效像素事件，全部保持在 HDMI RX PCLK 域。
// DE 是有效视频及行边界的依据；HS 只采样保留用于调试，不用来补造像素。
//
// Dependencies: 无子模块；后接 hdmi_rx_tile_normalizer。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// Convert the MS7200 parallel raster into explicit events for the tile
// normalizer.  All outputs stay in the HDMI RX PCLK domain.
//
// The board demo exposes VS/HS/DE as separate signals.  DE is the authoritative
// active-video qualifier for pixel_valid and line boundaries; HS is sampled and
// retained for debug/forward compatibility but is not used to invent pixels.
module hdmi_rx_event_adapter #(
	parameter integer C_VS_ACTIVE_HIGH = 1,
	parameter integer C_DE_ACTIVE_HIGH = 1
)(
	input i_pclk,
	input i_rst_n,
	input i_vs,
	input i_hs,
	input i_de,
	input [23:0] i_rgb888,
	output reg o_frame_start,
	output reg o_frame_end,
	output reg o_line_start,
	output reg o_line_end,
	output reg o_pixel_valid,
	output reg [23:0] o_rgb888,
	output reg o_vs_active,
	output reg o_hs_active,
	output reg o_de_active
);
	reg vs_active_d;
	reg hs_active_d;
	reg de_active_d;
	wire vs_active = C_VS_ACTIVE_HIGH ? i_vs : ~i_vs;
	wire hs_active = C_VS_ACTIVE_HIGH ? i_hs : ~i_hs;
	wire de_active = C_DE_ACTIVE_HIGH ? i_de : ~i_de;

	always @(posedge i_pclk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			vs_active_d <= 1'b0;
			hs_active_d <= 1'b0;
			de_active_d <= 1'b0;
			o_frame_start <= 1'b0;
			o_frame_end <= 1'b0;
			o_line_start <= 1'b0;
			o_line_end <= 1'b0;
			o_pixel_valid <= 1'b0;
			o_rgb888 <= 24'd0;
			o_vs_active <= 1'b0;
			o_hs_active <= 1'b0;
			o_de_active <= 1'b0;
		end else begin
			// Register the event and its pixel together.  The normalizer therefore
			// never sees a pixel from one RX cycle with an event from another.
			o_frame_start <= vs_active && !vs_active_d;
			o_frame_end <= !vs_active && vs_active_d;
			o_line_start <= de_active && !de_active_d;
			o_line_end <= !de_active && de_active_d;
			o_pixel_valid <= de_active;
			o_rgb888 <= i_rgb888;
			o_vs_active <= vs_active;
			o_hs_active <= hs_active;
			o_de_active <= de_active;
			vs_active_d <= vs_active;
			hs_active_d <= hs_active;
			de_active_d <= de_active;
		end
	end
endmodule
