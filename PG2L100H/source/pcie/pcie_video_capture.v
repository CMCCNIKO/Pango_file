//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-07
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 对最终 HDMI 输出建立只读采集分支，组合像素 tap、官方异步 FIFO 和单帧授权/元数据握手。
// Module Name: pcie_video_capture
// File Name: source/pcie/pcie_video_capture.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 对最终 HDMI 输出建立只读采集分支，组合像素 tap、官方异步 FIFO 和单帧授权/元数据握手。
// 控制、元数据及 128 位输出流位于 PCIe 域，像素源无反压；成功元数据确认前需读完全部字，错误确认前消费者必须停止并接受 flush。
//
// Dependencies: pcie_video_tap、pcie_video_async_fifo_wrapper、pcie_video_frame_ctrl。
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
// Create Date: 2026-10-07
// Module Name: pcie_video_capture
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4 / ModelSim 10.6e
// Description: T03最终HDMI输出的只读取流分支，组合tap、真实FIFO和帧握手。
// Dependencies: pcie_video_tap, pcie_video_async_fifo_wrapper, pcie_video_frame_ctrl.
// Contract: control/metadata/stream在PCIe域；HDMI仅作输入，无反压端口。
// 成功meta ack前消费者必须读完全部words；错误meta ack前消费者必须停稳。
// 任一输入reset清两域；消费者须使用o_pcie_reset_n并在flush时取消旧事务。
// 输出为裸128-bit像素流，T04负责TLP封装，不可直连PCIe slave2。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module pcie_video_capture #(
	parameter integer C_H_ACTIVE = 1280,
	parameter integer C_V_ACTIVE = 720
)(
	input i_pixel_clk,
	input i_pixel_rst_n,
	input i_hdmi_vs,
	input i_hdmi_hs,
	input i_hdmi_de,
	input [23:0] i_hdmi_rgb,
	input i_pcie_clk,
	input i_pcie_rst_n,
	input i_link_up,
	input i_enable,
	input i_stop,
	input i_rate_30,
	input i_host_buffer_ready,
	input i_stream_ready,
	input i_meta_consume_ack_toggle,
	output o_stream_valid,
	output [127:0] o_stream_data,
	output o_meta_valid,
	output [31:0] o_meta_frame_id,
	output [31:0] o_meta_byte_count,
	output [17:0] o_meta_word_count,
	output [31:0] o_meta_crc32,
	output o_meta_drop,
	output o_meta_error,
	output o_meta_ack_toggle,
	output o_busy,
	output o_flush_busy,
	output o_flush_done,
	output o_pcie_reset_n,
	output o_capture_enabled,
	output o_capture_active,
	output [12:0] o_wr_water_level,
	output [12:0] o_rd_water_level
);
	wire common_rst_n;
	reg [1:0] pixel_reset_sync;
	reg [1:0] pcie_reset_sync;
	wire pixel_rst_n, pcie_rst_n;
	wire tap_flush, flush_req, fifo_ack, fifo_busy, tap_ack;
	wire wr_en, fifo_full, fifo_almost_full, rd_valid;
	wire [127:0] wr_data;
	wire tap_start, tap_accept, tap_end, tap_drop, tap_error;
	wire [31:0] tap_id, tap_bytes, tap_crc;
	wire [17:0] tap_words;
	wire tap_meta_toggle, tap_boundary;
	assign common_rst_n = i_pixel_rst_n && i_pcie_rst_n;
	always @(posedge i_pixel_clk or negedge common_rst_n)begin
		if(!common_rst_n)begin pixel_reset_sync<=2'b00; end
		else begin pixel_reset_sync<={pixel_reset_sync[0],1'b1}; end
	end
	always @(posedge i_pcie_clk or negedge common_rst_n)begin
		if(!common_rst_n)begin pcie_reset_sync<=2'b00; end
		else begin pcie_reset_sync<={pcie_reset_sync[0],1'b1}; end
	end
	assign pixel_rst_n=pixel_reset_sync[1];
	assign pcie_rst_n=pcie_reset_sync[1];
	assign o_pcie_reset_n=pcie_rst_n;
	assign o_flush_busy=fifo_busy || (flush_req != fifo_ack) || !i_link_up || !i_enable || i_stop;
	assign o_stream_valid=rd_valid && !o_flush_busy && pcie_rst_n;
	pcie_video_tap #(.C_H_ACTIVE(C_H_ACTIVE),.C_V_ACTIVE(C_V_ACTIVE),
		.C_FRAME_PIXELS(C_H_ACTIVE*C_V_ACTIVE),.C_WORD_COUNT(C_H_ACTIVE*C_V_ACTIVE/8)) u_tap(
		.i_pixel_clk(i_pixel_clk),.i_pixel_rst_n(pixel_rst_n),
		.i_hdmi_vs(i_hdmi_vs),.i_hdmi_hs(i_hdmi_hs),.i_hdmi_de(i_hdmi_de),.i_hdmi_rgb(i_hdmi_rgb),
		.i_capture_enable(o_capture_enabled),.i_flush(tap_flush),
		.i_fifo_full(fifo_full),.i_fifo_almost_full(fifo_almost_full),
		.o_fifo_wr_en(wr_en),.o_fifo_wr_data(wr_data),
		.o_tap_frame_start(tap_start),.o_tap_frame_accept(tap_accept),.o_tap_frame_end(tap_end),
		.o_tap_frame_drop(tap_drop),.o_tap_frame_error(tap_error),.o_tap_frame_active(o_capture_active),
		.o_tap_frame_id(tap_id),.o_tap_byte_count(tap_bytes),.o_tap_word_count(tap_words),
		.o_tap_crc32(tap_crc),.o_tap_meta_toggle(tap_meta_toggle),
		.o_tap_boundary_toggle(tap_boundary),.o_flush_ack_toggle(tap_ack)
	);
	pcie_video_async_fifo_wrapper u_fifo(
		.i_wr_clk(i_pixel_clk),.i_wr_rst_n(pixel_rst_n),.i_wr_en(wr_en),.i_wr_data(wr_data),
		.o_wr_full(fifo_full),.o_wr_almost_full(fifo_almost_full),.o_wr_water_level(o_wr_water_level),
		.i_rd_clk(i_pcie_clk),.i_rd_rst_n(pcie_rst_n),.i_rd_ready(i_stream_ready && !o_flush_busy),
		.o_rd_valid(rd_valid),.o_rd_data(o_stream_data),.o_rd_empty(),.o_rd_almost_empty(),
		.o_rd_water_level(o_rd_water_level),.i_flush_req_toggle(flush_req),
		.o_flush_ack_toggle(fifo_ack),.o_flush_busy(fifo_busy)
	);
	pcie_video_frame_ctrl u_ctrl(
		.i_hdmi_clk(i_pixel_clk),.i_hdmi_rst_n(pixel_rst_n),.i_pcie_clk(i_pcie_clk),.i_pcie_rst_n(pcie_rst_n),
		.i_link_up(i_link_up),.i_enable(i_enable),.i_stop(i_stop),.i_rate_30(i_rate_30),
		.i_host_buffer_ready(i_host_buffer_ready),.i_tap_frame_start(tap_start),.i_tap_frame_accept(tap_accept),
		.i_tap_frame_end(tap_end),.i_tap_frame_drop(tap_drop),.i_tap_frame_error(tap_error),
		.i_tap_frame_id(tap_id),.i_tap_byte_count(tap_bytes),.i_tap_word_count(tap_words),.i_tap_crc32(tap_crc),
		.i_tap_meta_toggle(tap_meta_toggle),.i_tap_boundary_toggle(tap_boundary),
		.i_tap_flush_ack_toggle(tap_ack),.i_fifo_flush_ack_toggle(fifo_ack),
		.i_meta_consume_ack_toggle(i_meta_consume_ack_toggle),
		.o_hdmi_capture_enable(o_capture_enabled),.o_hdmi_flush(tap_flush),.o_flush_req_toggle(flush_req),
		.o_flush_done(o_flush_done),.o_pcie_busy(o_busy),.o_frame_grant_toggle(),
		.o_meta_valid(o_meta_valid),.o_meta_frame_id(o_meta_frame_id),.o_meta_byte_count(o_meta_byte_count),
		.o_meta_word_count(o_meta_word_count),.o_meta_crc32(o_meta_crc32),
		.o_meta_drop(o_meta_drop),.o_meta_error(o_meta_error),.o_meta_ack_toggle(o_meta_ack_toggle)
	);
endmodule
