//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 使用两个官方 128×512 SDP RAM 保存 tile 最终 128 位输出，按 slot 选择一拍读回数据。
// Module Name: f14_backbone_output_slots
// File Name: model/rtl/f14_backbone_output_slots.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 使用两个官方 128×512 SDP RAM 保存 tile 最终 128 位输出，按 slot 选择一拍读回数据。
// 检测读写地址越界与同槽并发读写冲突，输出 sticky error，为计算和写回重叠提供双输出槽。
//
// Dependencies: f14_ram_128x512×2
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 同步低有效复位清除控制/有效/错误；存储内容和读写冲突语义沿用官方 IP。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: 最终tile输出双槽；官方128x512 SDP IP，共4个DRM。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`default_nettype none
module f14_backbone_output_slots (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_wr_en,
	input wire i_wr_slot,
	input wire [11:0] i_wr_addr,
	input wire [127:0] i_wr_data,
	input wire i_rd_en,
	input wire i_rd_slot,
	input wire [11:0] i_rd_addr,
	output wire o_rd_valid,
	output wire [127:0] o_rd_data,
	output wire o_error
);
	wire [127:0] data0, data1;
	reg valid_q, select_q, error_q;
	assign o_rd_valid=valid_q;
	assign o_rd_data=select_q ? data1 : data0;
	assign o_error=error_q;
	always @(posedge i_clk) begin
		if (!i_rst_n) begin valid_q<=0;select_q<=0;error_q<=0;end
		else begin
			valid_q<=i_rd_en;
			if(i_rd_en) select_q<=i_rd_slot;
			if((i_wr_en && i_wr_addr>=512) || (i_rd_en && i_rd_addr>=512)
				|| (i_wr_en && i_rd_en && i_wr_slot==i_rd_slot)) error_q<=1;
		end
	end
	f14_ram_128x512 u_slot0 (
		.wr_clk(i_clk),.wr_rst(!i_rst_n),.wr_en(i_wr_en&&!i_wr_slot),.wr_addr(i_wr_addr[8:0]),.wr_data(i_wr_data),
		.rd_clk(i_clk),.rd_rst(!i_rst_n),.rd_clk_en(i_rd_en&&!i_rd_slot),.rd_addr(i_rd_addr[8:0]),.rd_data(data0));
	f14_ram_128x512 u_slot1 (
		.wr_clk(i_clk),.wr_rst(!i_rst_n),.wr_en(i_wr_en&&i_wr_slot),.wr_addr(i_wr_addr[8:0]),.wr_data(i_wr_data),
		.rd_clk(i_clk),.rd_rst(!i_rst_n),.rd_clk_en(i_rd_en&&i_rd_slot),.rd_addr(i_rd_addr[8:0]),.rd_data(data1));
endmodule
`default_nettype wire
