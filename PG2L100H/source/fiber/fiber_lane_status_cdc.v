//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-21
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 把独立系统域的车道状态原子传入 PCIe 只读状态窗口，数据与有效状态保持一致。
// Module Name: fiber_lane_status_cdc
// File Name: source/fiber/fiber_lane_status_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 把独立系统域的车道状态原子传入 PCIe 只读状态窗口，数据与有效状态保持一致。
// PCIe 复位不清系统域源快照，重连后自动读取当前值；marker 保持复位后全零窗口契约。
//
// Dependencies: vehicle_behavior_action_cdc。
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
// Create Date: 2026-10-21
// Design Name: Fiber lane metadata readback
// Module Name: fiber_lane_status_cdc
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 独立sys域快照到PCIe只读窗口的原子mailbox。
//              PCIe复位不清源快照，重新连接后自动读取当前值。
// Dependencies: vehicle_behavior_action_cdc
// Revision: 0.01 - File Created
// Revision: 0.02 - 2026-10-21 仅跨域可变状态，marker 保持复位后全零窗口。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module fiber_lane_status_cdc(
	input i_sys_clk,
	input i_sys_rst_n,
	input [63:0] i_snapshot,
	input i_valid,
	input i_stale,
	input i_pcie_clk,
	input i_pcie_rst_n,
	output [127:0] o_rd_window
);
	wire [31:0] visible_payload;
	wire [66:0] status_sys;
	wire [66:0] status_pcie;
	wire [127:0] rd_window_o;
	// +0 seq，+4规范化payload，+8 bit0有效/bit1过期，+c协议标识。
	// 超时或断链时有效位不再对软件保留；seq仅用于诊断。
	assign visible_payload = i_valid ? i_snapshot[31:0] : 32'd0;
	// 固定 ID/保留零位在读侧重建，避免两端保留 62 位常量寄存器。
	// marker 与整包同拍跨域；首包到达前 ID 为零，与原 128-bit mailbox 一致。
	assign status_sys = {1'b1,i_stale,i_valid,
		visible_payload,i_snapshot[63:32]};
	assign rd_window_o = {status_pcie[66] ? 32'h4c4d0001 : 32'd0,
		30'd0,status_pcie[65:0]};
	assign o_rd_window = rd_window_o;
	vehicle_behavior_action_cdc #(
		.C_DATA_WIDTH(67),
		.C_CHANGE_ONLY(1)
	) u_status_mailbox(
		.i_src_clk(i_sys_clk),
		.i_src_rst_n(i_sys_rst_n),
		.i_src_valid(1'b1),
		.i_src_payload(status_sys),
		.o_src_ready(),
		.o_src_online(),
		.i_dst_clk(i_pcie_clk),
		.i_dst_rst_n(i_pcie_rst_n),
		.i_dst_online(1'b1),
		.o_dst_valid(),
		.o_dst_payload(status_pcie)
	);
endmodule
