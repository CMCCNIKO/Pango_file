//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 HDMI RX 的 640×320 RGB565 像素打包为 256 位字，经官方 512×256 双时钟 FIFO 送往 DDR 域。
// Module Name: hdmi_source3_word_cdc
// File Name: source/video/hdmi_source3_word_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 HDMI RX 的 640×320 RGB565 像素打包为 256 位字，经官方 512×256 双时钟 FIFO 送往 DDR 域。
// 传递帧事件并在坏帧时清 FIFO，最后暂存字保持到握手；RX 帧授权需稳定，整帧最终提交由 DDR 侧确认 12800 个 HMIC beat。
//
// Dependencies: pixel16_to_word256、c6_net_to_ddr_fifo_wrapper、c6_event_sync。
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
//
// Create Date: 2026-09-14
// Design Name: Quad Video Mosaic
// Module Name: hdmi_source3_word_cdc
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: HDMI RX 的 640x320 RGB565 流打包为 256-bit 字，复用本工程
//              已生成的官方 512x256 双时钟 FIFO 跨入 DDR 域。坏帧清空 FIFO。
// Dependencies: pixel16_to_word256、c6_net_to_ddr_fifo_wrapper、c6_event_sync；
//               i_frame_accept 须是 RX 域稳定的帧授权，DDR 读者只消费已授权帧。
// Revision:
// Revision 0.01 - File Created
// Revision 0.02 - Retain a stalled final word until its FIFO handshake.
// Additional Comments: 完整帧提交仍须由 DDR 侧确认 12800 个 HMIC 写 beat。
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module hdmi_source3_word_cdc #(
	parameter integer C_FRAME_PIXELS = 204800,
	parameter C_USE_GRANT_TOKEN = 1'b0
)(
	input i_rx_clk,
	input i_rx_rst_n,
	input i_frame_accept,
	input i_grant_toggle,
	input i_frame_start,
	input i_frame_end,
	input i_frame_error,
	input i_pixel_valid,
	input [15:0] i_pixel_rgb565,
	input i_ddr_clk,
	input i_ddr_rst_n,
	input i_word_pop,
	output [255:0] o_word_data,
	output o_word_empty,
	output [15:0] o_wr_water_level,
	output [15:0] o_rd_water_level,
	output o_frame_prepare_ddr,
	output o_frame_complete_ddr,
	output o_frame_abort_ddr
);
	//---------------寄存器信号---------------//
	reg capture_active;
	reg frame_terminal_pending;
	reg prepare_rx;
	reg complete_rx;
	reg abort_rx;
	reg frontend_error_seen;
	reg grant_sync1 /* synthesis syn_preserve = 1 */;
	reg grant_sync2 /* synthesis syn_preserve = 1 */;
	reg grant_seen;

	//---------------其他信号---------------//
	wire packer_word_valid;
	wire [255:0] packer_word_data;
	wire packer_frame_end;
	wire packer_frame_error;
	wire packer_overflow;
	wire fifo_word_ready;
	wire packer_word_ready;
	wire fifo_word_valid;
	wire grant_pending;

	//-------------其他信号连线-------------//
	assign packer_word_ready = fifo_word_ready && capture_active;
	assign fifo_word_valid = packer_word_valid && capture_active;
	assign grant_pending = (grant_sync2 != grant_seen);

	// DDR 域持有 toggle 至下次授权；两个 RX 寄存器同步后只消费一次。
	// i_frame_accept 为 RX 域使能，拉低时保留尚未消费的 token。
	// DDR writer 必须等旧帧排空/中止处理结束后才翻转下一次授权。
	always @(posedge i_rx_clk or negedge i_rx_rst_n) begin
		if (!i_rx_rst_n) begin
			grant_sync1 <= 1'b0;
			grant_sync2 <= 1'b0;
			grant_seen <= 1'b0;
		end else begin
			grant_sync1 <= i_grant_toggle;
			grant_sync2 <= grant_sync1;
			if (C_USE_GRANT_TOKEN && i_frame_start && !capture_active &&
				i_frame_accept && grant_pending)
				grant_seen <= grant_sync2;
		end
	end

	// 前端错误可仅在 frame_end 拍出现，跨过打包器的一拍延迟后再判帧。
	always @(posedge i_rx_clk or negedge i_rx_rst_n) begin
		if (!i_rx_rst_n) begin
			frontend_error_seen <= 1'b0;
		end else if (i_frame_start) begin
			frontend_error_seen <= i_frame_error;
		end else if (i_frame_error) begin
			frontend_error_seen <= 1'b1;
		end
	end

	// 帧授权在 RX 域生效；新帧若撞上未终结帧，废弃旧帧并跳过本帧。
	// packer_frame_end 已比输入帧结束晚一拍，足以观察完整像素计数错误。
	always @(posedge i_rx_clk or negedge i_rx_rst_n) begin
		if (!i_rx_rst_n) begin
			capture_active <= 1'b0;
			frame_terminal_pending <= 1'b0;
			prepare_rx <= 1'b0;
			complete_rx <= 1'b0;
			abort_rx <= 1'b0;
		end else begin
			prepare_rx <= 1'b0;
			complete_rx <= 1'b0;
			abort_rx <= 1'b0;
			if (i_frame_start) begin
				frame_terminal_pending <= 1'b0;
				if (capture_active) begin
					capture_active <= 1'b0;
					abort_rx <= 1'b1;
				end else if (i_frame_accept && (!C_USE_GRANT_TOKEN || grant_pending)) begin
					capture_active <= 1'b1;
					prepare_rx <= 1'b1;
				end
			end else if (capture_active && packer_overflow) begin
				capture_active <= 1'b0;
				frame_terminal_pending <= 1'b0;
				abort_rx <= 1'b1;
			end else if (capture_active && (packer_frame_end || frame_terminal_pending)) begin
				if (packer_frame_error || frontend_error_seen || i_frame_error) begin
					capture_active <= 1'b0;
					frame_terminal_pending <= 1'b0;
					abort_rx <= 1'b1;
				end else if (!packer_word_valid || packer_word_ready) begin
					// The final word may be accepted on this same edge.
					capture_active <= 1'b0;
					frame_terminal_pending <= 1'b0;
					complete_rx <= 1'b1;
				end else begin
					frame_terminal_pending <= 1'b1;
				end
			end
		end
	end

	// 打包器输出 valid/ready；连续像素不能背压，积压一整字时标记 overflow。
	pixel16_to_word256 #(
		.C_FRAME_PIXELS(C_FRAME_PIXELS)
	) u_pixel16_to_word256(
		.i_clk(i_rx_clk),
		.i_rst_n(i_rx_rst_n),
		.i_frame_start(i_frame_start),
		.i_frame_end(i_frame_end),
		.i_pixel_valid(i_pixel_valid),
		.i_pixel_rgb565(i_pixel_rgb565),
		.i_word_ready(packer_word_ready),
		.o_word_valid(packer_word_valid),
		.o_word_data(packer_word_data),
		.o_frame_end(packer_frame_end),
		.o_frame_error(packer_frame_error),
		.o_overflow(packer_overflow)
	);

	// 复用当前 PG2L100H 工程已登记的官方 FIFO 配置，不复制 donor IP。
	c6_net_to_ddr_fifo_wrapper u_source3_fifo(
		.i_net_clk(i_rx_clk),
		.i_net_rst_n(i_rx_rst_n),
		.i_frame_prepare(prepare_rx),
		.i_frame_abort(abort_rx),
		.i_word_valid(fifo_word_valid),
		.i_word_data(packer_word_data),
		.o_word_ready(fifo_word_ready),
		.i_ddr_clk(i_ddr_clk),
		.i_ddr_rst_n(i_ddr_rst_n),
		.i_word_pop(i_word_pop),
		.o_word_data(o_word_data),
		.o_word_empty(o_word_empty),
		.o_wr_water_level(o_wr_water_level),
		.o_rd_water_level(o_rd_water_level)
	);

	// 单帧事件远慢于 DDR 时钟，已有 toggle CDC 可完整传递脉冲。
	c6_event_sync u_prepare_sync(
		.i_src_clk(i_rx_clk), .i_src_rst_n(i_rx_rst_n),
		.i_src_event(prepare_rx),
		.i_dst_clk(i_ddr_clk), .i_dst_rst_n(i_ddr_rst_n),
		.o_dst_event(o_frame_prepare_ddr)
	);
	c6_event_sync u_complete_sync(
		.i_src_clk(i_rx_clk), .i_src_rst_n(i_rx_rst_n),
		.i_src_event(complete_rx),
		.i_dst_clk(i_ddr_clk), .i_dst_rst_n(i_ddr_rst_n),
		.o_dst_event(o_frame_complete_ddr)
	);
	c6_event_sync u_abort_sync(
		.i_src_clk(i_rx_clk), .i_src_rst_n(i_rx_rst_n),
		.i_src_event(abort_rx),
		.i_dst_clk(i_ddr_clk), .i_dst_rst_n(i_ddr_rst_n),
		.o_dst_event(o_frame_abort_ddr)
	);
endmodule
