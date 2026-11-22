//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 跨时钟域事件同步器：源域单拍脉冲翻转 toggle，目标域经两级同步并与历史值异或，恢复成目标域单周期事件。
// Module Name: c6_event_sync
// File Name: source/memory/c6_event_sync.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 跨时钟域事件同步器：源域单拍脉冲翻转 toggle，目标域经两级同步并与历史值异或，恢复成目标域单周期事件。
// 用于 prepare/commit/abort/vblank 等稀疏事件；源事件间隔须大于目标同步窗口，两个时钟域复位须协调。
//
// Dependencies: 无 IP 依赖；纯控制寄存器。
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
// Create Date: 2026-09-10
// Design Name: Ethernet_HDMI
// Module Name: c6_event_sync
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: C6 单帧事务事件的跨时钟域 toggle 同步器。源脉冲转换为
//              目标时钟域的单周期脉冲，适用于帧 commit 和 abort。
//
// Dependencies: 无；源事件间隔必须大于目标域两拍同步窗口。
//
// Revision:
// Revision 0.01 - File Created
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module c6_event_sync(
	input i_src_clk,
	input i_src_rst_n,
	input i_src_event,
	input i_dst_clk,
	input i_dst_rst_n,
	output o_dst_event
);
	//---------------寄存器信号---------------//
	reg reg_event_toggle;
	reg reg_dst_sync0;
	reg reg_dst_sync1;
	reg reg_dst_sync1_d;

	//-------------输出信号连线-------------//
	assign o_dst_event = reg_dst_sync1 ^ reg_dst_sync1_d;

	// 源域只在事件发生时翻转一次；C6 分片之间的间隔远大于同步窗口。
	always @(posedge i_src_clk or negedge i_src_rst_n) begin
		if (!i_src_rst_n) begin
			reg_event_toggle <= 1'b0;
		end else if (i_src_event) begin
			reg_event_toggle <= ~reg_event_toggle;
		end
	end

	// 目标域用两级同步器采样 toggle，并同拍保存前值产生一个时钟周期的事件。
	always @(posedge i_dst_clk or negedge i_dst_rst_n) begin
		if (!i_dst_rst_n) begin
			reg_dst_sync0 <= 1'b0;
			reg_dst_sync1 <= 1'b0;
			reg_dst_sync1_d <= 1'b0;
		end else begin
			reg_dst_sync0 <= reg_event_toggle;
			reg_dst_sync1 <= reg_dst_sync0;
			reg_dst_sync1_d <= reg_dst_sync1;
		end
	end
endmodule
