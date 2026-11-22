//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 保存 F14 骨干 S8 特征的 24×21×28 CHW、14112 字节快照，使用四个官方 8×4096 RAM；读口一拍返回数据与有效标志。
// Module Name: host_f14_s8_snapshot
// File Name: host/rtl/video/host_f14_s8_snapshot.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 保存 F14 骨干 S8 特征的 24×21×28 CHW、14112 字节快照，使用四个官方 8×4096 RAM；读口一拍返回数据与有效标志。
// 每次接收新帧清除完成状态，累计完整写入后置 complete，供车道线型采样使用。
//
// Dependencies: f14_ram_8x4096；单系统时钟 i_clk/i_rst_n；仅控制/有效状态复位，RAM 数据不复位。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// S8 CHW 24x21x28快照，4个官方8x4096 DRM。一拍读返回；帧接收清有效。
`timescale 1ns/1ps
`default_nettype none
module host_f14_s8_snapshot (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_frame_accept,
	input wire i_wr_en,
	input wire [13:0] i_wr_addr,
	input wire [7:0] i_wr_data,
	input wire i_rd_en,
	input wire [13:0] i_rd_addr,
	output wire [7:0] o_rd_data,
	output reg o_rd_valid,
	output reg o_complete
);
	reg [13:0] count_q;
	reg [1:0] read_bank_q;
	wire [7:0] data [0:3];
	genvar b;
	generate for (b=0;b<4;b=b+1) begin : g_s8
		f14_ram_8x4096 u_ram (
			.wr_data(i_wr_data), .wr_addr(i_wr_addr[11:0]),
			.wr_en(i_wr_en && i_wr_addr<14112 && i_wr_addr[13:12]==b),
			.wr_clk(i_clk), .wr_rst(1'b0), .rd_data(data[b]),
			.rd_addr(i_rd_addr[11:0]), .rd_clk(i_clk),
			.rd_clk_en(i_rd_en && i_rd_addr<14112 && i_rd_addr[13:12]==b), .rd_rst(1'b0)
		);
	end endgenerate
	assign o_rd_data=data[read_bank_q];
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			count_q<=0; read_bank_q<=0; o_rd_valid<=0; o_complete<=0;
		end else begin
			o_rd_valid<=i_rd_en && i_rd_addr<14112;
			if (i_rd_en) begin read_bank_q<=i_rd_addr[13:12]; end
			if (i_frame_accept) begin count_q<=0; o_complete<=0; end
			else if (i_wr_en && i_wr_addr<14112) begin
				count_q<=count_q+1'b1;
				if (count_q==14111) begin o_complete<=1; end
			end
		end
	end
endmodule
`default_nettype wire
