//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 用多个官方 8×4096 DRM 组成字节寻址全局特征 bank，提供同步一拍主读和访问范围检查。
// Module Name: f14_global_feature_bank
// File Name: model/rtl/f14_global_feature_bank_v2.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 用多个官方 8×4096 DRM 组成字节寻址全局特征 bank，提供同步一拍主读和访问范围检查。
// 可选奇偶交错布局在不改变线性字节写语义的情况下增加另一奇偶字节读取；输出 bank 选择与返回数据对齐。
//
// Dependencies: f14_ram_8x4096
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 复位只清输出 valid/错误，不清存储内容；调用方避免同址读写，pair 请求须满足地址/奇偶条件。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 dense byte-addressed global feature bank
// Module Name: f14_global_feature_bank
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: Official 8x4096 DRM banks, synchronous one-cycle read.
// Dependencies: IP Compiler generated f14_ram_8x4096
// Revision: 0.02 - Hold read-data bank selection through reset
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
`default_nettype none

// 同址读写遵循官方 IP 的未定义冲突契约，调用方必须避免该操作。
// 复位仅清除输出有效标志，不复位存储内容。读禁止或越界时保持数据。
module f14_global_feature_bank #(
	parameter integer C_DEPTH = 225792,
	parameter integer C_ADDR_WIDTH = 18,
	parameter integer C_PAIR_ENABLE = 0
) (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_wr_en,
	input wire [C_ADDR_WIDTH - 1:0] i_wr_addr,
	input wire [7:0] i_wr_data,
	input wire i_rd_en,
	input wire [C_ADDR_WIDTH - 1:0] i_rd_addr,
	output wire [7:0] o_rd_data,
	output wire o_rd_valid,
	output wire o_access_error,
	input wire i_pair_en,
	input wire [C_ADDR_WIDTH - 1:0] i_pair_addr,
	output wire [7:0] o_pair_data
);

	localparam integer BANK_COUNT = (C_PAIR_ENABLE != 0)
		? ((C_DEPTH + 8191) / 8192) * 2 : (C_DEPTH + 4095) / 4096;
	localparam integer BANK_SEL_WIDTH = (C_ADDR_WIDTH > 12) ? C_ADDR_WIDTH - 12 : 1;
	wire [BANK_COUNT * 8 - 1:0] bank_data;
	wire write_fire;
	wire read_fire;
	wire [BANK_SEL_WIDTH - 1:0] write_bank;
	wire [BANK_SEL_WIDTH - 1:0] read_bank;
	reg [BANK_SEL_WIDTH - 1:0] read_bank_q;
	reg [BANK_SEL_WIDTH - 1:0] pair_bank_q;
	wire [BANK_SEL_WIDTH - 1:0] pair_bank = ((i_pair_addr >> 13) << 1) | i_pair_addr[0];
	wire pair_requested = (C_PAIR_ENABLE != 0) && i_pair_en;
	wire pair_fire = pair_requested && read_fire && (i_pair_addr < C_DEPTH)
		&& (i_pair_addr[0] != i_rd_addr[0]);
	reg rd_valid_o;
	reg access_error_o;
	genvar bank;

	assign write_fire = i_rst_n && i_wr_en && (i_wr_addr < C_DEPTH);
	assign read_fire = i_rst_n && i_rd_en && (i_rd_addr < C_DEPTH);
	// 奇偶交错：沿用同数量官方8x4096 IP，字节地址和写语义不变。
	assign write_bank = (C_PAIR_ENABLE != 0) ? (((i_wr_addr >> 13) << 1) | i_wr_addr[0]) : i_wr_addr >> 12;
	assign read_bank = (C_PAIR_ENABLE != 0) ? (((i_rd_addr >> 13) << 1) | i_rd_addr[0]) : i_rd_addr >> 12;
	assign o_rd_data = bank_data[read_bank_q * 8 +: 8];
	assign o_pair_data = bank_data[pair_bank_q * 8 +: 8];
	assign o_rd_valid = rd_valid_o;
	assign o_access_error = access_error_o;

	// 参数契约：C_ADDR_WIDTH >= 13，1 <= C_DEPTH <= 2**C_ADDR_WIDTH。
	// 双读模式以8192字节为一组，奇偶各一块IP；单读模式保持原布局。
	// 调用方的构建预检核对参数，普通 RTL 不放仿真专用 initial/$fatal。
	generate
		for(bank = 0; bank < BANK_COUNT; bank = bank + 1)begin:gen_drm
			wire [11:0] read_addr = (C_PAIR_ENABLE != 0)
				? (((bank % 2) == i_rd_addr[0]) ? i_rd_addr[12:1] : i_pair_addr[12:1])
				: i_rd_addr[11:0];
			f14_ram_8x4096 u_generated (
				.wr_data(i_wr_data),
				.wr_addr((C_PAIR_ENABLE != 0) ? i_wr_addr[12:1] : i_wr_addr[11:0]),
				.wr_en(write_fire && (write_bank == bank)),
				.wr_clk(i_clk),
				.wr_rst(1'b0),
				.rd_data(bank_data[bank * 8 +: 8]),
				.rd_addr(read_addr),
				.rd_clk(i_clk),
				.rd_clk_en((read_fire && (read_bank == bank)) || (pair_fire && (pair_bank == bank))),
				.rd_rst(1'b0)
			);
		end
	endgenerate

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			rd_valid_o <= 1'b0;
			access_error_o <= 1'b0;
		end else begin
			rd_valid_o <= read_fire;
			access_error_o <= (i_wr_en && !write_fire) || (i_rd_en && !read_fire)
				|| (pair_requested && !pair_fire);
		end
	end

	// 存储内容不复位；选择标签同样保持，防止复位异步改变输出数据。
	// 下一次合法读在valid置位的同一拍装入新标签，不改变一拍读延迟。
	always@(posedge i_clk)begin
		if(read_fire)begin read_bank_q <= read_bank; end
		if(pair_fire)begin pair_bank_q <= pair_bank; end
	end
endmodule

`default_nettype wire
