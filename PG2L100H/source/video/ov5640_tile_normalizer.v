//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将默认 1280×720 RGB565 摄像头图像按偶数列及源行 20..658 的偶数行抽取为 640×320 tile。
// Module Name: ov5640_tile_normalizer
// File Name: source/video/ov5640_tile_normalizer.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将默认 1280×720 RGB565 摄像头图像按偶数列及源行 20..658 的偶数行抽取为 640×320 tile。
// 首像素与 line_start 同拍时使用当前行有效坐标，像素计满后输出一次帧完成，数据始终在摄像头 PCLK 域。
//
// Dependencies: 上游 ov5640_dvp_unpack 的同域帧/行/像素事件。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// 1280x720 RGB565 camera stream -> 640x320 tile:
// keep even x pixels, keep even source rows 20..658, and emit rows 20..339
// after 2x decimation. The output remains in the camera PCLK domain.
module ov5640_tile_normalizer #(
	parameter integer C_SOURCE_WIDTH = 1280,
	parameter integer C_SOURCE_HEIGHT = 720,
	parameter integer C_CROP_TOP = 20,
	parameter integer C_TILE_WIDTH = 640,
	parameter integer C_TILE_HEIGHT = 320
)(
	input i_pclk,
	input i_rst_n,
	input i_frame_start,
	input i_frame_end,
	input i_line_start,
	input i_line_end,
	input i_pixel_valid,
	input [15:0] i_pixel_rgb565,
	output reg o_pixel_valid,
	output reg [15:0] o_pixel_rgb565,
	output reg o_frame_start,
	output reg o_frame_end,
	output reg o_frame_error
);
	reg [11:0] source_x;
	reg [10:0] source_y;
	reg [18:0] output_count;
	reg line_seen;
	reg frame_complete_pending;
	// The unpacker asserts line_start together with the first pixel pair.  Use
	// effective coordinates for that cycle so the first pixel is not evaluated
	// with the previous line's state.
	wire [11:0] pixel_x = i_line_start ? 12'd0 : source_x;
	wire [10:0] pixel_y = i_frame_start ? 11'd0 :
		((i_line_start && line_seen) ? (source_y + 1'b1) : source_y);
	wire keep_row = (pixel_y >= C_CROP_TOP) &&
		(pixel_y < C_CROP_TOP + 2*C_TILE_HEIGHT) &&
		(((pixel_y - C_CROP_TOP) & 1) == 0);

	always @(posedge i_pclk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			source_x <= 12'd0;
			source_y <= 11'd0;
			output_count <= 19'd0;
			line_seen <= 1'b0;
			o_pixel_valid <= 1'b0;
			o_pixel_rgb565 <= 16'd0;
			o_frame_start <= 1'b0;
			o_frame_end <= 1'b0;
			o_frame_error <= 1'b0;
			frame_complete_pending <= 1'b0;
		end else begin
			o_pixel_valid <= 1'b0;
			o_frame_start <= i_frame_start;
			o_frame_end <= frame_complete_pending;
			frame_complete_pending <= 1'b0;
			if (i_frame_start) begin
				source_x <= 12'd0;
				source_y <= 11'd0;
				output_count <= 19'd0;
				line_seen <= 1'b0;
				frame_complete_pending <= 1'b0;
				o_frame_error <= 1'b0;
			end
			if (i_line_start) begin
				source_x <= 12'd0;
				if (i_frame_start)
					source_y <= 11'd0;
				else if (line_seen && (source_y < C_SOURCE_HEIGHT-1))
					source_y <= source_y + 1'b1;
				line_seen <= 1'b1;
			end
			if (i_pixel_valid) begin
				if (!pixel_x[0] && keep_row && (output_count < C_TILE_WIDTH*C_TILE_HEIGHT)) begin
					o_pixel_valid <= 1'b1;
					o_pixel_rgb565 <= i_pixel_rgb565;
					output_count <= output_count + 1'b1;
					if (output_count == C_TILE_WIDTH*C_TILE_HEIGHT-1)
						frame_complete_pending <= 1'b1;
				end
				// Consume the first pixel when line_start and pixel_valid coincide.
				source_x <= pixel_x + 1'b1;
			end
		end
	end
endmodule
