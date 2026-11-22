//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-11
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 DDR 帧槽 grant 的稳定描述符接收到网络域：两级同步 grant toggle 后锁存配套 slot_id，产生单周期 grant。
// Module Name: c7_slot_grant_sync
// File Name: source/memory/c7_slot_grant_sync.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 DDR 帧槽 grant 的稳定描述符接收到网络域：两级同步 grant toggle 后锁存配套 slot_id，产生单周期 grant。
// slot_id 须在请求 toggle 之前稳定并保持到下一次 grant，不逐位独立同步多位描述符。
//
// Dependencies: 无 IP 依赖；上游提供稳定 payload 与 toggle。
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
// Create Date: 2026-09-11
// Module Name: c7_slot_grant_sync
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: DDR 帧槽管理器到网络域的稳定描述符握手接收端。slot_id 在
//              grant toggle 前已经稳定，并保持到下一次 grant；接收端在
//              toggle 的两级同步完成后锁存它，输出单周期 grant 脉冲。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module c7_slot_grant_sync(
	input i_clk,
	input i_rst_n,
	input i_grant_toggle,
	input [1:0] i_grant_slot,
	output reg o_grant,
	output reg [1:0] o_slot
);
	reg reg_toggle_sync0;
	reg reg_toggle_sync1;
	reg reg_toggle_seen;
	// slot 描述符只在 grant toggle 前后保持不变；这两条独立同步链只在
	// toggle 已确认后的下一拍一起采样，不能当作任意多位实时 CDC 使用。
	reg reg_slot_sync0_bit0 /* synthesis syn_preserve = 1 */;
	reg reg_slot_sync0_bit1 /* synthesis syn_preserve = 1 */;
	reg reg_slot_sync1_bit0 /* synthesis syn_preserve = 1 */;
	reg reg_slot_sync1_bit1 /* synthesis syn_preserve = 1 */;

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			reg_toggle_sync0 <= 1'b0;
			reg_toggle_sync1 <= 1'b0;
			reg_toggle_seen <= 1'b0;
			reg_slot_sync0_bit0 <= 1'b0;
			reg_slot_sync0_bit1 <= 1'b0;
			reg_slot_sync1_bit0 <= 1'b0;
			reg_slot_sync1_bit1 <= 1'b0;
			o_grant <= 1'b0;
			o_slot <= 2'd0;
		end else begin
			reg_toggle_sync0 <= i_grant_toggle;
			reg_toggle_sync1 <= reg_toggle_sync0;
			reg_slot_sync0_bit0 <= i_grant_slot[0];
			reg_slot_sync0_bit1 <= i_grant_slot[1];
			reg_slot_sync1_bit0 <= reg_slot_sync0_bit0;
			reg_slot_sync1_bit1 <= reg_slot_sync0_bit1;
			o_grant <= 1'b0;
			if (reg_toggle_sync1 != reg_toggle_seen) begin
				reg_toggle_seen <= reg_toggle_sync1;
				o_slot <= {reg_slot_sync1_bit1, reg_slot_sync1_bit0};
				o_grant <= 1'b1;
			end
		end
	end
endmodule
