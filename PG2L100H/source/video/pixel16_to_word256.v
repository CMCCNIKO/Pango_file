//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 每 16 个 RGB565 像素组成一个 256 位 HMIC 数据字，首像素占最低 16 位。
// Module Name: pixel16_to_word256
// File Name: source/video/pixel16_to_word256.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 每 16 个 RGB565 像素组成一个 256 位 HMIC 数据字，首像素占最低 16 位。
// valid/ready 完成前保持整字，满字再次到达时报告 overflow 并作废帧；帧结束检查像素总数与尾部对齐。
//
// Dependencies: 下游 256 位 FIFO ready；上游同域像素及帧事件。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// Pack sixteen RGB565 pixels into the existing 256-bit HMIC word order.
module pixel16_to_word256 #(
	parameter integer C_FRAME_PIXELS = 204800
)(
	input i_clk,
	input i_rst_n,
	input i_frame_start,
	input i_frame_end,
	input i_pixel_valid,
	input [15:0] i_pixel_rgb565,
	input i_word_ready,
	output reg o_word_valid,
	output reg [255:0] o_word_data,
	output reg o_frame_end,
	output reg o_frame_error,
	output reg o_overflow
);
	reg [255:0] pack_data;
	reg [4:0] pixel_count;
	reg [18:0] frame_pixel_count;

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			pack_data <= 256'd0;
			pixel_count <= 5'd0;
			frame_pixel_count <= 19'd0;
			o_word_valid <= 1'b0;
			o_word_data <= 256'd0;
			o_frame_end <= 1'b0;
			o_frame_error <= 1'b0;
			o_overflow <= 1'b0;
		end else begin
			// valid/ready 握手前保持完整的 256-bit 数据字。
			o_word_valid <= o_word_valid && !i_word_ready;
			o_frame_end <= i_frame_end;
			o_overflow <= 1'b0;
			if (i_frame_start) begin
				o_word_valid <= 1'b0;
				pack_data <= 256'd0;
				pixel_count <= 5'd0;
				frame_pixel_count <= 19'd0;
				o_frame_error <= 1'b0;
				// Permit a producer to assert frame_start with its first pixel.
				if (i_pixel_valid) begin
					pack_data[15:0] <= i_pixel_rgb565;
					pixel_count <= 5'd1;
					frame_pixel_count <= 19'd1;
				end
			end else if (i_pixel_valid) begin
				if (pixel_count == 5'd15) begin
					if (!o_word_valid || i_word_ready) begin
						o_word_valid <= 1'b1;
						// pack_data holds pixels 0..14 in bits [239:0]; the
						// completing pixel occupies the most-significant slice.
						o_word_data <= {i_pixel_rgb565, pack_data[239:0]};
						pixel_count <= 5'd0;
					end else begin
						o_overflow <= 1'b1;
						o_frame_error <= 1'b1;
						pixel_count <= 5'd0;
					end
				end else begin
					pack_data[pixel_count*16 +: 16] <= i_pixel_rgb565;
					pixel_count <= pixel_count + 1'b1;
				end
				frame_pixel_count <= frame_pixel_count + 1'b1;
			end
			if (i_frame_end) begin
				if ((pixel_count != 5'd0) || (frame_pixel_count != C_FRAME_PIXELS))
					o_frame_error <= 1'b1;
			end
		end
	end
endmodule
