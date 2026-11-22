//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将原生 HDMI RX RGB888 事件流按水平/垂直抽取与裁剪转换为 640×320 RGB565 tile，默认保留偶数列和裁剪区偶数行。
// Module Name: hdmi_rx_tile_normalizer
// File Name: source/video/hdmi_rx_tile_normalizer.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将原生 HDMI RX RGB888 事件流按水平/垂直抽取与裁剪转换为 640×320 RGB565 tile，默认保留偶数列和裁剪区偶数行。
// 有效像素计满后只产生一次正常结束；crop 截止仍不足时只产生一次错误终结并关闭当前帧，禁止尾部补采样。
//
// Dependencies: 上游 hdmi_rx_event_adapter 在同一 RX PCLK 域提供帧/行事件。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// Normalize a standard HDMI RX RGB888 raster to a 640x320 RGB565 tile.
// The RX stream is expected to be converted to explicit frame/line/pixel
// events in its native PCLK domain before this module.  Even source x pixels
// are retained, and even rows in the crop window are retained.
// Revision 2026-10-10: crop结束仍不足时单次异常终结，正常像素/早完成拍数保持。
module hdmi_rx_tile_normalizer #(
	parameter integer C_SOURCE_WIDTH = 1280,
	parameter integer C_SOURCE_HEIGHT = 720,
	parameter integer C_CROP_TOP = 20,
	parameter integer C_TILE_WIDTH = 640,
	parameter integer C_TILE_HEIGHT = 320,
	parameter integer C_H_DECIM = 2,
	parameter integer C_V_DECIM = 2
)(
	input i_pclk,
	input i_rst_n,
	input i_frame_start,
	input i_frame_end,
	input i_line_start,
	input i_pixel_valid,
	input [23:0] i_pixel_rgb888,
	output reg o_pixel_valid,
	output reg [15:0] o_pixel_rgb565,
	output reg o_frame_start,
	output reg o_frame_end,
	output reg o_frame_error
);
	reg [11:0] source_x;
	reg [10:0] source_y;
	reg [11:0] x_decim_phase;
	reg [10:0] y_decim_phase;
	reg line_seen;
	reg [18:0] output_count;
	reg frame_complete_pending;
	// 正常完成或crop截止异常后关闭本帧，禁止尾部补采样及重复终态。
	reg frame_closed;
	wire [11:0] pixel_x = i_line_start ? 12'd0 : source_x;
	wire [10:0] pixel_y = i_frame_start ? 11'd0 :
		((i_line_start && line_seen) ? (source_y + 1'b1) : source_y);
	wire [11:0] pixel_x_phase = i_line_start ? 12'd0 : x_decim_phase;
	wire [10:0] advanced_y_phase = (y_decim_phase >= C_V_DECIM-1) ?
		11'd0 : (y_decim_phase + 1'b1);
	wire [10:0] pixel_y_phase = (pixel_y <= C_CROP_TOP) ? 11'd0 :
		((i_line_start && line_seen && !i_frame_start) ?
		 advanced_y_phase : y_decim_phase);
	wire keep_row = (pixel_y >= C_CROP_TOP) &&
		(pixel_y < C_CROP_TOP + C_V_DECIM*C_TILE_HEIGHT) &&
		(pixel_y_phase == 0);
	wire keep_column = (pixel_x_phase == 0);
	wire [15:0] rgb565 = {i_pixel_rgb888[23:19], i_pixel_rgb888[15:10],
		i_pixel_rgb888[7:3]};

	always @(posedge i_pclk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			source_x <= 12'd0;
			source_y <= 11'd0;
			x_decim_phase <= 12'd0;
			y_decim_phase <= 11'd0;
			line_seen <= 1'b0;
			output_count <= 19'd0;
			o_pixel_valid <= 1'b0;
			o_pixel_rgb565 <= 16'd0;
			o_frame_start <= 1'b0;
			o_frame_end <= 1'b0;
			o_frame_error <= 1'b0;
			frame_complete_pending <= 1'b0;
			frame_closed <= 1'b1;
		end else begin
			o_pixel_valid <= 1'b0;
			o_frame_start <= i_frame_start;
			o_frame_end <= frame_complete_pending;
			frame_complete_pending <= 1'b0;
			if (i_frame_start) begin
				source_x <= 12'd0;
				source_y <= 11'd0;
				x_decim_phase <= 12'd0;
				y_decim_phase <= 11'd0;
				line_seen <= 1'b0;
				output_count <= 19'd0;
				frame_complete_pending <= 1'b0;
				frame_closed <= 1'b0;
				o_frame_error <= 1'b0;
			end
			if (i_line_start) begin
				source_x <= 12'd0;
				x_decim_phase <= 12'd0;
				y_decim_phase <= pixel_y_phase;
				if (i_frame_start)
					source_y <= 11'd0;
				else if (line_seen && (source_y < C_SOURCE_HEIGHT-1))
					source_y <= source_y + 1'b1;
				line_seen <= 1'b1;
			end
			if (i_pixel_valid) begin
				if ((!frame_closed || i_frame_start) && keep_column && keep_row && (output_count < C_TILE_WIDTH*C_TILE_HEIGHT)) begin
					o_pixel_valid <= 1'b1;
					o_pixel_rgb565 <= rgb565;
					output_count <= output_count + 1'b1;
					if (output_count == C_TILE_WIDTH*C_TILE_HEIGHT-1) begin
						frame_complete_pending <= 1'b1;
						frame_closed <= 1'b1;
					end
				end
				source_x <= pixel_x + 1'b1;
				if (pixel_x_phase >= C_H_DECIM-1)
					x_decim_phase <= 12'd0;
				else
					x_decim_phase <= pixel_x_phase + 1'b1;
			end
			// i_frame_end来自VS下降，位于有效图像之前，不能用于短帧判定。
			// 在crop已结束的下一行起点终结不足帧，为下一VS前drain/授权留时间。
			if (!i_frame_start && !frame_closed && i_line_start &&
				(pixel_y >= C_CROP_TOP + C_V_DECIM*C_TILE_HEIGHT) &&
				(output_count < C_TILE_WIDTH*C_TILE_HEIGHT)) begin
				o_frame_end <= 1'b1;
				o_frame_error <= 1'b1;
				frame_complete_pending <= 1'b0;
				frame_closed <= 1'b1;
			end
		end
	end
endmodule
