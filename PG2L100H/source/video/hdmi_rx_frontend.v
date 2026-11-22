//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-11-01
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 MS7200 的 RGB888 与 VS/HS/DE 输入统一前置采样，再转换事件并归一化为 640×320 RGB565 tile。
// Module Name: hdmi_rx_frontend
// File Name: source/video/hdmi_rx_frontend.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 MS7200 的 RGB888 与 VS/HS/DE 输入统一前置采样，再转换事件并归一化为 640×320 RGB565 tile。
// 请求输入寄存器映射到 IOL，稳态到原事件层增加一拍；复位释放后等待采样稳定再开放输入处理。
//
// Dependencies: hdmi_rx_event_adapter、hdmi_rx_tile_normalizer。
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
// Create Date: 2026-11-01
// Module Name: hdmi_rx_frontend
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2
// Revision: 0.02 - HDMI输入统一前置采样，请求直接输入寄存器打包到IOL。
//                 稳态输入至原事件层整体增加一拍；复位释放后等待采样稳定。
//////////////////////////////////////////////////////////////////////////////////
// MS7200 RGB888 raster adapter followed by the common 640x320 RGB565
// normalizer.  The complete module remains in the RX PCLK domain; its output
// is intended to feed a pixel packer and an asynchronous source FIFO.
module hdmi_rx_frontend #(
	parameter integer C_SOURCE_WIDTH = 1920,
	parameter integer C_SOURCE_HEIGHT = 1080,
	parameter integer C_CROP_TOP = 60,
	parameter integer C_TILE_WIDTH = 640,
	parameter integer C_TILE_HEIGHT = 320,
	parameter integer C_H_DECIM = 3,
	parameter integer C_V_DECIM = 3,
	parameter integer C_VS_ACTIVE_HIGH = 1,
	parameter integer C_DE_ACTIVE_HIGH = 1
)(
	input i_pclk,
	input i_rst_n,
	input i_vs,
	input i_hs,
	input i_de,
	input [23:0] i_rgb888,
	output o_pixel_valid,
	output [15:0] o_pixel_rgb565,
	output o_frame_start,
	output o_frame_end,
	output o_frame_error,
	output o_rx_vs_active,
	output o_rx_hs_active,
	output o_rx_de_active
);
	wire frame_start;
	wire frame_end;
	wire line_start;
	wire line_end;
	wire pixel_valid;
	wire [23:0] rgb888;
	// 原始引脚只驱动这一组同拍寄存器，避免DE/VS在多个逻辑锥分别采样。
	// 不给数据采样FF添加复位或使能，以保留直接端口到IFF的打包条件。
	// syn_useioff是官方打包请求，须以本次DeviceMap日志/物理结果核实。
	reg [23:0] rx_rgb888_sample /* synthesis syn_useioff = 1 */;
	reg rx_vs_sample /* synthesis syn_useioff = 1 */;
	reg rx_hs_sample /* synthesis syn_useioff = 1 */;
	reg rx_de_sample /* synthesis syn_useioff = 1 */;
	reg [1:0] rx_sample_ready;
	wire event_rst_n;

	always @(posedge i_pclk) begin
		rx_rgb888_sample <= i_rgb888;
		rx_vs_sample <= i_vs;
		rx_hs_sample <= i_hs;
		rx_de_sample <= i_de;
	end

	// PCLK晚于按键复位恢复时，事件层也必须先等原始采样有效。
	always @(posedge i_pclk or negedge i_rst_n) begin
		if (!i_rst_n)
			rx_sample_ready <= 2'b00;
		else
			rx_sample_ready <= {rx_sample_ready[0], 1'b1};
	end
	assign event_rst_n = i_rst_n && rx_sample_ready[1];

	hdmi_rx_event_adapter #(
		.C_VS_ACTIVE_HIGH(C_VS_ACTIVE_HIGH),
		.C_DE_ACTIVE_HIGH(C_DE_ACTIVE_HIGH)
	) u_event_adapter(
		.i_pclk(i_pclk),
		.i_rst_n(event_rst_n),
		.i_vs(rx_vs_sample),
		.i_hs(rx_hs_sample),
		.i_de(rx_de_sample),
		.i_rgb888(rx_rgb888_sample),
		.o_frame_start(frame_start),
		.o_frame_end(frame_end),
		.o_line_start(line_start),
		.o_line_end(line_end),
		.o_pixel_valid(pixel_valid),
		.o_rgb888(rgb888),
		.o_vs_active(o_rx_vs_active),
		.o_hs_active(o_rx_hs_active),
		.o_de_active(o_rx_de_active)
	);

	hdmi_rx_tile_normalizer #(
		.C_SOURCE_WIDTH(C_SOURCE_WIDTH),
		.C_SOURCE_HEIGHT(C_SOURCE_HEIGHT),
		.C_CROP_TOP(C_CROP_TOP),
		.C_TILE_WIDTH(C_TILE_WIDTH),
		.C_TILE_HEIGHT(C_TILE_HEIGHT),
		.C_H_DECIM(C_H_DECIM),
		.C_V_DECIM(C_V_DECIM)
	) u_tile_normalizer(
		.i_pclk(i_pclk),
		.i_rst_n(i_rst_n),
		.i_frame_start(frame_start),
		.i_frame_end(frame_end),
		.i_line_start(line_start),
		.i_pixel_valid(pixel_valid),
		.i_pixel_rgb888(rgb888),
		.o_pixel_valid(o_pixel_valid),
		.o_pixel_rgb565(o_pixel_rgb565),
		.o_frame_start(o_frame_start),
		.o_frame_end(o_frame_end),
		.o_frame_error(o_frame_error)
	);
endmodule
