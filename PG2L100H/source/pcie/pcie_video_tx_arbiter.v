//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-07
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 在参考 DMA 与视频 DMA 的两路 128 位 TLP 流之间执行包级仲裁。
// Module Name: pcie_video_tx_arbiter
// File Name: source/pcie/pcie_video_tx_arbiter.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 在参考 DMA 与视频 DMA 的两路 128 位 TLP 流之间执行包级仲裁。
// 首个 valid 拍锁定发送方，持续到 last 成功握手后才允许切换；链路断开或复位时禁止发送并清除所有权。
//
// Dependencies: 两路 TLP valid/data/last 与官方 PCIe slave2 ready。
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
// Design Name: T04 video PCIe transport
// Module Name: pcie_video_tx_arbiter
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 从首拍 valid 锁定发送源到末拍握手；模式只在包间切换。
// Dependencies: None.
// Revision 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module pcie_video_tx_arbiter (
	input i_clk,
	input i_rst_n,
	input i_link_up,
	input i_video_mode,
	input i_legacy_valid,
	input [127:0] i_legacy_data,
	input i_legacy_last,
	input i_legacy_user,
	output o_legacy_ready,
	input i_video_valid,
	input [127:0] i_video_data,
	input i_video_last,
	input i_video_user,
	output o_video_ready,
	output o_axis_valid,
	output [127:0] o_axis_data,
	output o_axis_last,
	output o_axis_user,
	input i_axis_ready,
	output o_packet_busy
);
	reg locked;
	reg owner_video;
	wire select_video = locked ? owner_video : i_video_mode;
	wire selected_valid = select_video ? i_video_valid : i_legacy_valid;
	wire selected_last = select_video ? i_video_last : i_legacy_last;
	wire active = i_rst_n && i_link_up;
	assign o_axis_valid = active && selected_valid;
	assign o_axis_data = select_video ? i_video_data : i_legacy_data;
	assign o_axis_last = selected_last;
	assign o_axis_user = select_video ? i_video_user : i_legacy_user;
	assign o_legacy_ready = active && !select_video && i_axis_ready;
	assign o_video_ready = active && select_video && i_axis_ready;
	assign o_packet_busy = locked || o_axis_valid;
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			locked <= 1'b0;
			owner_video <= 1'b0;
		end else if (!i_link_up) begin
			locked <= 1'b0;
			owner_video <= 1'b0;
		end else if (selected_valid) begin
			if (i_axis_ready && selected_last) begin
				locked <= 1'b0;
			end else begin
				locked <= 1'b1;
				owner_video <= select_video;
			end
		end
	end
endmodule

