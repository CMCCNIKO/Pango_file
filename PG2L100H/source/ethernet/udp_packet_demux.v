//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-09
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 依据接收头阶段锁存的稳定路由将 UDP start/data/valid/end/error 分发到视频或控制支路。
// Module Name: udp_packet_demux
// File Name: source/ethernet/udp_packet_demux.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 依据接收头阶段锁存的稳定路由将 UDP start/data/valid/end/error 分发到视频或控制支路。
// 纯组合门控不增加首末字节延迟，length 在 end 有效；允许错误 end 没有此前 start 的异常结束契约。
//
// Dependencies: udp_rx 或 udp_ip_mac_top 提供稳定包路由。
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
// Create Date: 2026-10-09
// Design Name: HDMI algorithm control
// Module Name: udp_packet_demux
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Stable RX route gates all packet events without adding latency.
//              Length is valid at end. Error end need not have a preceding start.
// Dependencies: udp_rx
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module udp_packet_demux
(
	input i_is_control,
	input i_start,
	input i_valid,
	input [7:0] i_data,
	input i_end,
	input [15:0] i_length,
	input i_error,
	output o_video_start,
	output o_video_valid,
	output [7:0] o_video_data,
	output o_video_end,
	output [15:0] o_video_length,
	output o_video_error,
	output o_ctrl_start,
	output o_ctrl_valid,
	output [7:0] o_ctrl_data,
	output o_ctrl_end,
	output [15:0] o_ctrl_length,
	output o_ctrl_error
);
	// Combinational routing preserves first-byte and last-byte timing.
	assign o_video_start = i_start && !i_is_control;
	assign o_video_valid = i_valid && !i_is_control;
	assign o_video_data = i_is_control ? 8'd0 : i_data;
	assign o_video_end = i_end && !i_is_control;
	assign o_video_length = i_is_control ? 16'd0 : i_length;
	assign o_video_error = i_error && !i_is_control;
	assign o_ctrl_start = i_start && i_is_control;
	assign o_ctrl_valid = i_valid && i_is_control;
	assign o_ctrl_data = i_is_control ? i_data : 8'd0;
	assign o_ctrl_end = i_end && i_is_control;
	assign o_ctrl_length = i_is_control ? i_length : 16'd0;
	assign o_ctrl_error = i_error && i_is_control;
endmodule
