//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-09
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 通过可综合 RTL 推断简单双口行 RAM，为邻域算法提供一拍同步读取的行历史数据。
// Module Name: pango_line_ram_1280x8
// File Name: source/video/pango_line_ram_1280x8.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 通过可综合 RTL 推断简单双口行 RAM，为邻域算法提供一拍同步读取的行历史数据。
// 无存储器复位；调用方需避开同址读写并屏蔽未写行，深度支持 3..2048、宽度至少 1。
//
// Dependencies: PDS 存储器推断；实际资源映射需在 PDS 报告确认。
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
// Create Date: 2026-10-09
// Design Name: Algorithm Line Storage
// Module Name: pango_line_ram_1280x8
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Inferred simple dual-port line RAM, one-clock synchronous read.
//              Caller avoids same-address read/write and masks unwritten lines.
//              No memory reset or Xilinx primitive; depth 3..2048, width >= 1.
// Dependencies: PDS memory inference (resource mapping must be checked in PDS).
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module pango_line_ram_1280x8
#(
	parameter integer C_DATA_WIDTH = 8,
	parameter integer C_DEPTH = 1280
)
(
	input i_clk,
	input i_wr_en,
	input [10:0] i_wr_addr,
	input [C_DATA_WIDTH - 1:0] i_wr_data,
	input i_rd_en,
	input [10:0] i_rd_addr,
	output [C_DATA_WIDTH - 1:0] o_rd_data
);
	reg [C_DATA_WIDTH - 1:0] reg_memory [0:C_DEPTH - 1];
	reg [C_DATA_WIDTH - 1:0] rd_data_o;
	assign o_rd_data = rd_data_o;

	// Independent synchronous ports match a simple dual-port memory template.
	always@(posedge i_clk)begin
		if(i_wr_en)begin
			reg_memory[i_wr_addr] <= i_wr_data;
		end
	end
	always@(posedge i_clk)begin
		if(i_rd_en)begin
			rd_data_o <= reg_memory[i_rd_addr];
		end
	end
endmodule
