//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 用37个官方字节 DRM 保存150528字节特征，前64KiB按低4位交错，兼容线性字节读写与128位整字读。
// Module Name: f14_global_feature_bank_wide
// File Name: model/rtl/f14_global_feature_bank_wide.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 用37个官方字节 DRM 保存150528字节特征，前64KiB按低4位交错，兼容线性字节读写与128位整字读。
// 宽读一次并行读取前16个bank，供 head 消费 packed 特征；检测非法地址及互斥读接口冲突。
//
// Dependencies: f14_ram_8x4096×37
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 字节地址与宽字编号单位不同；两读接口互斥、固定一拍返回，复位不清存储，调用方避免同址读写。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-19
// Target Devices: PG2L200H-6FBB484; Tool: PDS 2022.2-SP6.4
// 前64KiB按低4位交错，字节写/读和128bit整字读共享原有37个官方DRM。
// 字节端口地址仍为线性字节地址；宽端口地址为16字节word编号。
// 两个读接口互斥，固定一拍返回；复位不清存储，调用方避免同址读写。
`timescale 1ns/1ps
`default_nettype none
module f14_global_feature_bank_wide (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_wr_en,
	input wire [17:0] i_wr_addr,
	input wire [7:0] i_wr_data,
	input wire i_rd_en,
	input wire [17:0] i_rd_addr,
	output wire [7:0] o_rd_data,
	output reg o_rd_valid,
	input wire i_wide_en,
	input wire [11:0] i_wide_addr,
	output wire [127:0] o_wide_data,
	output reg o_wide_valid,
	output reg o_access_error
);
	wire wr_ok = i_rst_n && i_wr_en && (i_wr_addr < 18'd150528);
	wire rd_ok = i_rst_n && i_rd_en && !i_wide_en && (i_rd_addr < 18'd150528);
	wire wide_ok = i_rst_n && i_wide_en && !i_rd_en;
	wire [295:0] data_bus;
	wire [5:0] wr_bank = (i_wr_addr < 18'd65536) ? {2'd0,i_wr_addr[3:0]} : i_wr_addr[17:12];
	wire [5:0] rd_bank = (i_rd_addr < 18'd65536) ? {2'd0,i_rd_addr[3:0]} : i_rd_addr[17:12];
	reg [5:0] rd_bank_q;
	assign o_rd_data = data_bus[rd_bank_q*8 +: 8];
	assign o_wide_data = data_bus[127:0];
	genvar b;
	generate
		for(b=0;b<37;b=b+1)begin: gen_drm
			wire [11:0] wr_address = (b<16) ? i_wr_addr[15:4] : i_wr_addr[11:0];
			wire [11:0] byte_address = (b<16) ? i_rd_addr[15:4] : i_rd_addr[11:0];
			f14_ram_8x4096 u_generated (
				.wr_data(i_wr_data), .wr_addr(wr_address), .wr_en(wr_ok && wr_bank==b),
				.wr_clk(i_clk), .wr_rst(1'b0), .rd_data(data_bus[b*8 +: 8]),
				.rd_addr((b<16 && wide_ok) ? i_wide_addr : byte_address),
				.rd_clk(i_clk), .rd_clk_en((rd_ok && rd_bank==b) || (b<16 && wide_ok)), .rd_rst(1'b0)
			);
		end
	endgenerate
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			o_rd_valid<=0; o_wide_valid<=0; o_access_error<=0;
		end else begin
			o_rd_valid<=rd_ok; o_wide_valid<=wide_ok;
			o_access_error<=(i_wr_en && !wr_ok) || (i_rd_en && !rd_ok) || (i_wide_en && !wide_ok);
		end
	end
	always@(posedge i_clk)begin
		if(rd_ok)begin rd_bank_q<=rd_bank; end
	end
endmodule
`default_nettype wire
