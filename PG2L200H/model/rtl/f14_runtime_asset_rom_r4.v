//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 封装已初始化的131072×32官方ROM，提供只读32位资产/控制记录接口和同步一拍valid。
// Module Name: f14_runtime_asset_rom
// File Name: model/rtl/f14_runtime_asset_rom_r4.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 封装已初始化的131072×32官方ROM，提供只读32位资产/控制记录接口和同步一拍valid。
// 写口恒禁止，仅在复位解除且rd_en有效时读取，数据内容由r4官方IP及初始化HEX保留。
//
// Dependencies: f14_rom_32x131072_r4；assets/runtime/rom_words32.hex
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 复位只清valid，不复位ROM；不得以旧版同名资产替换r4内容。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: Actual initialized ROM, one-cycle read; no write port exposed.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

module f14_runtime_asset_rom (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_rd_en,
	input wire [16:0] i_rd_addr,
	output wire [31:0] o_rd_data,
	output wire o_rd_valid
);
	reg rd_valid_o;
	wire read_fire;
	assign read_fire = i_rst_n && i_rd_en;
	assign o_rd_valid = rd_valid_o;

	// 官方 IP 保存真实初始化内容；复位只清除 valid，不复位 ROM。
	f14_rom_32x131072_r4 u_generated (
		.wr_data(32'd0), .wr_addr(17'd0), .wr_en(1'b0),
		.wr_clk(i_clk), .wr_rst(1'b0),
		.rd_data(o_rd_data), .rd_addr(i_rd_addr),
		.rd_clk(i_clk), .rd_clk_en(read_fire), .rd_rst(1'b0)
	);
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			rd_valid_o <= 1'b0;
		end else begin
			rd_valid_o <= read_fire;
		end
	end
endmodule
`default_nettype wire
