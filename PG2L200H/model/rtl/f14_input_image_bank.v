//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-19
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 用十个官方128×512 DRM保存224×168的RGB565输入图像，每8个连续像素组装为一个128位字。
// Module Name: f14_input_image_bank
// File Name: model/rtl/f14_input_image_bank.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 用十个官方128×512 DRM保存224×168的RGB565输入图像，每8个连续像素组装为一个128位字。
// 外部保持16位像素寻址和单拍读回；内部对齐bank与lane选择，F14_SIMULATION可选行为分支。
//
// Dependencies: f14_ram_128x512；F14_SIMULATION/F14_OFFICIAL_IMAGE 宏
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 写入必须遵循原8像素顺序打包契约；无复位端口，调用方管理已写完整图像及读写所有权。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-19
// Design Name: F14 input image bank
// Module Name: f14_input_image_bank
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: 16-bit RGB565 image storage using ten 128x512 DRM banks.
//              Eight sequential pixels are packed into one 128-bit word;
//              the external pixel address and one-cycle read contract stay
//              unchanged while the previous low/high-byte banking is removed.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module f14_input_image_bank #(
	parameter integer C_DEPTH = 37632
) (
	input wire i_clk,
	input wire i_wr_en,
	input wire [15:0] i_wr_addr,
	input wire [15:0] i_wr_data,
	input wire i_rd_en,
	input wire [15:0] i_rd_addr,
	output wire [15:0] o_rd_data
);
	localparam integer BANK_COUNT = 10;
	localparam integer BANK_DEPTH = 512;

`ifndef F14_SIMULATION
`define F14_IMAGE_IMPL
`elsif F14_OFFICIAL_IMAGE
`define F14_IMAGE_IMPL
`endif
`ifdef F14_IMAGE_IMPL
	wire [127:0] bank_data [0:BANK_COUNT-1];
	reg [3:0] rd_bank_q;
	reg [2:0] rd_lane_q;
	reg [127:0] wr_pack_q;
	wire [12:0] wr_word_addr = i_wr_addr >> 3;
	wire [12:0] rd_word_addr = i_rd_addr >> 3;
	wire [3:0] wr_bank = wr_word_addr[12:9];
	wire [3:0] rd_bank = rd_word_addr[12:9];
	wire [8:0] wr_local = wr_word_addr[8:0];
	wire [8:0] rd_local = rd_word_addr[8:0];
	wire wr_valid = i_wr_en && (i_wr_addr < C_DEPTH);
	wire rd_valid = i_rd_en && (i_rd_addr < C_DEPTH);
	wire wr_word_en = wr_valid && (i_wr_addr[2:0] == 3'd7);
	wire [127:0] wr_word_data = {i_wr_data, wr_pack_q[111:0]};
	genvar b;
	generate
		for (b = 0; b < BANK_COUNT; b = b + 1) begin : g_bank
			f14_ram_128x512 u_image (
				.wr_data(wr_word_data), .wr_addr(wr_local),
				.wr_en(wr_word_en && (wr_bank == b)), .wr_clk(i_clk), .wr_rst(1'b0),
				.rd_data(bank_data[b]), .rd_addr(rd_local), .rd_clk(i_clk),
				.rd_clk_en(rd_valid && (rd_bank == b)), .rd_rst(1'b0)
			);
		end
	endgenerate
	always @(posedge i_clk) begin
		if (wr_valid) begin
			case (i_wr_addr[2:0])
				3'd0: wr_pack_q[15:0] <= i_wr_data;
				3'd1: wr_pack_q[31:16] <= i_wr_data;
				3'd2: wr_pack_q[47:32] <= i_wr_data;
				3'd3: wr_pack_q[63:48] <= i_wr_data;
				3'd4: wr_pack_q[79:64] <= i_wr_data;
				3'd5: wr_pack_q[95:80] <= i_wr_data;
				3'd6: wr_pack_q[111:96] <= i_wr_data;
				3'd7: wr_pack_q[127:112] <= i_wr_data;
				default: begin end
			endcase
		end
		if (rd_valid && (rd_bank < BANK_COUNT)) begin
			rd_bank_q <= rd_bank;
			rd_lane_q <= i_rd_addr[2:0];
		end
	end
	// 显式组合选择：所有 RAM 输出均在敏感列表内，避免函数隐藏依赖。
	reg [127:0] selected_word_data;
	always @(*) begin
		case (rd_bank_q)
			4'd0: selected_word_data = bank_data[0];
			4'd1: selected_word_data = bank_data[1];
			4'd2: selected_word_data = bank_data[2];
			4'd3: selected_word_data = bank_data[3];
			4'd4: selected_word_data = bank_data[4];
			4'd5: selected_word_data = bank_data[5];
			4'd6: selected_word_data = bank_data[6];
			4'd7: selected_word_data = bank_data[7];
			4'd8: selected_word_data = bank_data[8];
			4'd9: selected_word_data = bank_data[9];
			default: selected_word_data = 128'd0;
		endcase
	end
	assign o_rd_data = selected_word_data[rd_lane_q*16 +: 16];
`else
	reg [15:0] mem [0:C_DEPTH-1];
	reg [15:0] rd_data_q;
	integer k;
	initial begin
		for (k = 0; k < C_DEPTH; k = k + 1)
			mem[k] = 16'd0;
	end
	assign o_rd_data = rd_data_q;
	always @(posedge i_clk) begin
		if (i_wr_en && (i_wr_addr < C_DEPTH))
			mem[i_wr_addr] <= i_wr_data;
		if (i_rd_en && (i_rd_addr < C_DEPTH))
			rd_data_q <= mem[i_rd_addr];
	end
`endif
`ifdef F14_IMAGE_IMPL
`undef F14_IMAGE_IMPL
`endif
endmodule
