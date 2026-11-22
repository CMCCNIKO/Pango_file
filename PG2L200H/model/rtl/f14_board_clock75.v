//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将板载 50MHz 经 PLL_user_75 产生 75MHz 推理时钟，并生成系统、八路分区及像素域的复位。
// Module Name: f14_board_clock75
// File Name: model/rtl/f14_board_clock75.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将板载 50MHz 经 PLL_user_75 产生 75MHz 推理时钟，并生成系统、八路分区及像素域的复位。
// 外部复位或 PLL 失锁立即断言复位，在各目标时钟域经过三级链同步释放；分区末级与原第三级并行采样第二级，保持释放拍数。
//
// Dependencies: PLL_user_75；50MHz 输入时钟和像素时钟
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 分区寄存器/网带 syn_preserve/syn_keep；capture/curve 两域共同复位以防 toggle 所有权失配。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: PG2L200H-6FBB484; Tool: PDS 2022.2-SP6.4
// 50MHz晶振经实板验证的PLL_user_75产生75MHz；失锁/外部复位立即断言，三拍同步释放。
`timescale 1ns/1ps
`default_nettype none
module f14_board_clock75 (
	input wire i_clk50,
	input wire i_pixel_clk,
	input wire i_rst_n,
	output wire o_clk75,
	output wire o_rst_n,
	output wire [7:0] o_reset_partition_n,
	output wire o_pixel_rst_n,
	output wire o_locked
);
	wire async_reset_n;
	(* async_reg = "true" *) reg [2:0] reset_q;
	PLL_user_75 u_pll (
		.clkin1(i_clk50),
		.clkout0(o_clk75),
		.lock(o_locked)
	);
	assign async_reset_n = i_rst_n && o_locked;
	always @(posedge o_clk75 or negedge async_reset_n) begin
		if (!async_reset_n) begin
			reset_q <= 3'b000;
		end else begin
			reset_q <= {reset_q[1:0], 1'b1};
		end
	end
	assign o_rst_n = reset_q[2];
	// 分区末级与原第三级同时采样第二级，不增加释放拍数。
	// ADS 手册 syn_preserve/syn_keep 保留独立寄存器及输出网。
	reg [7:0] reset_partition_q /* synthesis syn_preserve=1 */;
	wire [7:0] reset_partition_n /* synthesis syn_keep=1 */;
	always @(posedge o_clk75 or negedge async_reset_n) begin
		if (!async_reset_n) begin
			reset_partition_q <= 8'h00;
		end else begin
			reset_partition_q <= {8{reset_q[1]}};
		end
	end
	assign reset_partition_n = reset_partition_q;
	assign o_reset_partition_n = reset_partition_n;
    // capture/curve两个时钟域共同复位，避免失锁后toggle所有权失配。
    (* async_reg = "true" *) reg [2:0] pixel_reset_q;
    always @(posedge i_pixel_clk or negedge async_reset_n) begin
        if (!async_reset_n) pixel_reset_q <= 0;
        else pixel_reset_q <= {pixel_reset_q[1:0],1'b1};
    end
    assign o_pixel_rst_n = pixel_reset_q[2];
endmodule
`default_nettype wire
