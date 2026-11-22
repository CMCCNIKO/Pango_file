//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-17
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 为输入、特征及中间存储提供统一的参数化 PG2 DRM 边界：字宽、深度、地址位宽及初始化文件由参数决定。
// Module Name: f14_explicit_drm_ram
// File Name: model/rtl/f14_explicit_drm_ram.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 为输入、特征及中间存储提供统一的参数化 PG2 DRM 边界：字宽、深度、地址位宽及初始化文件由参数决定。
// 实现分支实例化 f14_feature_ram_ip，F14_SIMULATION 分支使用行为存储并可 readmemh 初始化；均保留同步单拍读接口。
//
// Dependencies: f14_feature_ram_ip；可选 C_INIT_FILE
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 无复位端口，调用方负责存储有效性与读写冲突约束；打包不改仿真宏或存储实现选择。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-17
// Design Name: F14 single-frame bring-up
// Module Name: f14_explicit_drm_ram
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: Explicit DRM boundary for input and feature-bank storage.
// Dependencies: PG2L200H IP Compiler generated f14_feature_ram_ip
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps

module f14_explicit_drm_ram #(
	parameter integer C_ADDR_WIDTH = 16,
	parameter integer C_DATA_WIDTH = 128,
	parameter integer C_DEPTH = 65536,
	parameter C_INIT_FILE = "NONE"
) (
	input wire i_clk,
	input wire i_wr_en,
	input wire [C_ADDR_WIDTH - 1:0] i_wr_addr,
	input wire [C_DATA_WIDTH - 1:0] i_wr_data,
	input wire i_rd_en,
	input wire [C_ADDR_WIDTH - 1:0] i_rd_addr,
	output wire [C_DATA_WIDTH - 1:0] o_rd_data
);

`ifndef F14_SIMULATION
	f14_feature_ram_ip #(
		.ADDR_WIDTH(C_ADDR_WIDTH),
		.DATA_WIDTH(C_DATA_WIDTH),
		.DEPTH(C_DEPTH),
		.INIT_FILE(C_INIT_FILE)
	) u_ip (
		.clk(i_clk),
		.wr_en(i_wr_en),
		.wr_addr(i_wr_addr),
		.wr_data(i_wr_data),
		.rd_en(i_rd_en),
		.rd_addr(i_rd_addr),
		.rd_data(o_rd_data)
	);
`else
	reg [C_DATA_WIDTH - 1:0] rd_data_q;
	assign o_rd_data = rd_data_q;
	reg [C_DATA_WIDTH - 1:0] mem [0:C_DEPTH - 1];
	integer index;
	initial begin
		for (index = 0; index < C_DEPTH; index = index + 1)
			mem[index] = {C_DATA_WIDTH{1'b0}};
		if (C_INIT_FILE != "NONE")
			$readmemh(C_INIT_FILE, mem);
	end
	always @(posedge i_clk) begin
		if (i_wr_en)
			mem[i_wr_addr] <= i_wr_data;
		if (i_rd_en)
			rd_data_q <= mem[i_rd_addr];
	end
`endif
endmodule
