//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 OV5640 DVP 的 8 位字节流按两字节配对还原 RGB565，首字节作为像素高 8 位。
// Module Name: ov5640_dvp_unpack
// File Name: source/video/ov5640_dvp_unpack.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 OV5640 DVP 的 8 位字节流按两字节配对还原 RGB565，首字节作为像素高 8 位。
// 由 VS/HREF 导出帧、行和有效像素事件，全部保留在摄像头 PCLK 域，后续须通过异步 FIFO 跨域。
//
// Dependencies: 无子模块；摄像头必须已配置为 RGB565。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// OV5640 DVP 8-bit byte-pair unpacker. The camera is configured for RGB565;
// the first byte of each pair becomes RGB565[15:8]. All events stay in the
// camera PCLK domain and must cross through an async FIFO afterwards.
module ov5640_dvp_unpack(
	input i_pclk,
	input i_rst_n,
	input i_vsync,
	input i_href,
	input [7:0] i_data,
	output reg o_pixel_valid,
	output reg [15:0] o_pixel_rgb565,
	output reg o_frame_start,
	output reg o_frame_end,
	output reg o_line_start,
	output reg o_line_end
);
	reg vsync_d;
	reg href_d;
	reg byte_phase;
	reg [7:0] first_byte;
	reg line_start_pending;
	reg line_end_pending;

	always @(posedge i_pclk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			vsync_d <= 1'b0;
			href_d <= 1'b0;
			byte_phase <= 1'b0;
			first_byte <= 8'd0;
			line_start_pending <= 1'b0;
			line_end_pending <= 1'b0;
			o_pixel_valid <= 1'b0;
			o_pixel_rgb565 <= 16'd0;
			o_frame_start <= 1'b0;
			o_frame_end <= 1'b0;
			o_line_start <= 1'b0;
			o_line_end <= 1'b0;
		end else begin
			vsync_d <= i_vsync;
			href_d <= i_href;
			o_pixel_valid <= 1'b0;
			o_frame_start <= !vsync_d && i_vsync;
			o_frame_end <= vsync_d && !i_vsync;
			o_line_start <= 1'b0;
			o_line_end <= 1'b0;
			if (!href_d && i_href) line_start_pending <= 1'b1;
			if (href_d && !i_href) line_end_pending <= 1'b1;
			if (!i_href) begin
				byte_phase <= 1'b0;
				if (line_end_pending) begin
					o_line_end <= 1'b1;
					line_end_pending <= 1'b0;
				end
			end else if (!byte_phase) begin
				first_byte <= i_data;
				byte_phase <= 1'b1;
			end else begin
				o_pixel_rgb565 <= {first_byte, i_data};
				o_pixel_valid <= 1'b1;
				o_line_start <= line_start_pending;
				line_start_pending <= 1'b0;
				byte_phase <= 1'b0;
			end
		end
	end
endmodule
