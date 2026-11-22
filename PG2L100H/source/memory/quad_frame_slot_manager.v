//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 四路视频独立三槽管理外壳，为每个 source 实例化 frame_slot_manager，分别输出 grant、槽基址、有效性和 epoch。
// Module Name: quad_frame_slot_manager
// File Name: source/memory/quad_frame_slot_manager.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 四路视频独立三槽管理外壳，为每个 source 实例化 frame_slot_manager，分别输出 grant、槽基址、有效性和 epoch。
// HDMI 帧边界及读退役事件广播至各源，各源失败或缺帧不会直接改动其他源的槽所有权。
//
// Dependencies: frame_slot_manager、quad_frame_address_map.vh。
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
// Create Date: 2026-09-14
// Design Name: Quad Video Mosaic
// Module Name: quad_frame_slot_manager
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 四路独立三槽 frame ownership 外壳。每一路使用独立的
//              frame_slot_manager，HDMI 帧边界和读侧 retire 事件分别广播到四路。
// Dependencies: frame_slot_manager
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module quad_frame_slot_manager #(
	parameter integer C_FRAME_WORDS = 12800,
	parameter integer C_EXTERNAL_LEASE_ENABLE = 0
)(
	input i_clk,
	input i_rst_n,
	input [3:0] i_frame_prepare,
	output [3:0] o_grant_valid,
	output [7:0] o_grant_slot,
	output [3:0] o_grant_toggle,
	input [3:0] i_hmic_write_beat,
	input [3:0] i_frame_end,
	input [3:0] i_write_stream_drained,
	input [3:0] i_overflow,
	input [3:0] i_error,
	input i_hdmi_frame_boundary,
	input [3:0] i_retire_done,
	input [11:0] i_external_slot_held,
	output [63:0] o_display_epoch,
	output [3:0] o_frame_ready,
	output [3:0] o_frame_aborted,
	output [7:0] o_display_slot,
	output [3:0] o_display_valid,
	output [3:0] o_display_switch,
	output [127:0] o_valid_frame_count,
	output [127:0] o_dropped_frame_count,
	output [127:0] o_display_commit_count,
	output [35:0] o_slot_state,
	output [7:0] o_receiving_slot,
	output [63:0] o_written_words
);
	genvar source_index;

	generate
		for (source_index = 0; source_index < 4; source_index = source_index + 1) begin: gen_source_slot_manager
			frame_slot_manager #(
				.C_FRAME_WORDS(C_FRAME_WORDS), .C_SLOT_COUNT(3), .C_EXTERNAL_LEASE_ENABLE(C_EXTERNAL_LEASE_ENABLE)
			) u_frame_slot_manager(
				.i_clk(i_clk), .i_rst_n(i_rst_n),
				.i_frame_prepare(i_frame_prepare[source_index]),
				.o_grant_valid(o_grant_valid[source_index]),
				.o_grant_slot(o_grant_slot[source_index * 2 +: 2]),
				.o_grant_toggle(o_grant_toggle[source_index]),
				.i_hmic_write_beat(i_hmic_write_beat[source_index]),
				.i_frame_end(i_frame_end[source_index]),
				.i_write_stream_drained(i_write_stream_drained[source_index]),
				.i_overflow(i_overflow[source_index]), .i_error(i_error[source_index]),
				.i_hdmi_frame_boundary(i_hdmi_frame_boundary),
				.i_retire_done(i_retire_done[source_index]),
				.i_external_slot_held(i_external_slot_held[source_index * 3 +: 3]),
				.o_display_epoch(o_display_epoch[source_index * 16 +: 16]),
				.o_frame_ready(o_frame_ready[source_index]),
				.o_frame_aborted(o_frame_aborted[source_index]),
				.o_display_slot(o_display_slot[source_index * 2 +: 2]),
				.o_display_valid(o_display_valid[source_index]),
				.o_display_switch(o_display_switch[source_index]),
				.o_valid_frame_count(o_valid_frame_count[source_index * 32 +: 32]),
				.o_dropped_frame_count(o_dropped_frame_count[source_index * 32 +: 32]),
				.o_display_commit_count(o_display_commit_count[source_index * 32 +: 32]),
				.o_slot0_state(o_slot_state[source_index * 9 +: 3]),
				.o_slot1_state(o_slot_state[source_index * 9 + 3 +: 3]),
				.o_slot2_state(o_slot_state[source_index * 9 + 6 +: 3]),
				.o_receiving_slot(o_receiving_slot[source_index * 2 +: 2]),
				.o_written_words(o_written_words[source_index * 16 +: 16])
			);
		end
	endgenerate
endmodule
