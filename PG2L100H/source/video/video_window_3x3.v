//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-09
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 使用两份同步行 RAM 构建以当前输入(x,y)为结尾的因果 3×3 像素窗口，固定两拍且无需整帧缓存。
// Module Name: video_window_3x3
// File Name: source/video/video_window_3x3.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 使用两份同步行 RAM 构建以当前输入(x,y)为结尾的因果 3×3 像素窗口，固定两拍且无需整帧缓存。
// 边界掩码覆盖头两行/列及末行/列，有效中心为(x-1,y-1)；支持宽 3..2048、高 3..1024 的连续逐行 DE。
//
// Dependencies: pango_line_ram_1280x8（两份同步行存储）。
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
// Design Name: Causal 3x3 Pixel Window
// Module Name: video_window_3x3
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Two-clock causal window ending at input(x,y), no frame buffering.
//              o_border masks x<2/y<2 and last column/row; useful center is (x-1,y-1).
//              Dimensions width 3..2048, height 3..1024; full sequential DE rows.
// Dependencies: pango_line_ram_1280x8 (two synchronous line memories)
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module video_window_3x3
#(
	parameter integer C_DATA_WIDTH = 8,
	parameter integer C_FRAME_WIDTH = 1280,
	parameter integer C_FRAME_HEIGHT = 720
)
(
	input i_clk,
	input i_rst_n,
	input i_vs,
	input i_hs,
	input i_de,
	input i_sof,
	input i_eol,
	input [C_DATA_WIDTH - 1:0] i_data,
	input [10:0] i_x,
	input [9:0] i_y,
	output o_vs,
	output o_hs,
	output o_de,
	output o_sof,
	output o_eol,
	output o_border,
	output [C_DATA_WIDTH - 1:0] o_p11,
	output [C_DATA_WIDTH - 1:0] o_p12,
	output [C_DATA_WIDTH - 1:0] o_p13,
	output [C_DATA_WIDTH - 1:0] o_p21,
	output [C_DATA_WIDTH - 1:0] o_p22,
	output [C_DATA_WIDTH - 1:0] o_p23,
	output [C_DATA_WIDTH - 1:0] o_p31,
	output [C_DATA_WIDTH - 1:0] o_p32,
	output [C_DATA_WIDTH - 1:0] o_p33
);
	localparam [10:0] LAST_X = C_FRAME_WIDTH - 1;
	localparam [9:0] LAST_Y = C_FRAME_HEIGHT - 1;
	wire [C_DATA_WIDTH - 1:0] line0_data;
	wire [C_DATA_WIDTH - 1:0] line1_data;
	reg [C_DATA_WIDTH - 1:0] reg_data;
	reg [10:0] reg_x;
	reg [9:0] reg_y;
	reg [4:0] reg_timing;
	reg [4:0] timing_o;
	reg border_o;
	reg [C_DATA_WIDTH - 1:0] reg_row1_d1;
	reg [C_DATA_WIDTH - 1:0] reg_row1_d2;
	reg [C_DATA_WIDTH - 1:0] reg_row2_d1;
	reg [C_DATA_WIDTH - 1:0] reg_row2_d2;
	reg [C_DATA_WIDTH - 1:0] reg_row3_d1;
	reg [C_DATA_WIDTH - 1:0] reg_row3_d2;
	reg [9 * C_DATA_WIDTH - 1:0] pixels_o;
	wire flag_write;

	assign flag_write = i_rst_n && reg_timing[2];
	assign {o_vs, o_hs, o_de, o_sof, o_eol} = timing_o;
	assign o_border = border_o;
	assign {o_p11, o_p12, o_p13, o_p21, o_p22, o_p23, o_p31, o_p32, o_p33} = pixels_o;

	// Stage one retains the pixel/address while synchronous RAM reads complete.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_data <= {C_DATA_WIDTH{1'b0}};
			reg_x <= 11'd0;
			reg_y <= 10'd0;
			reg_timing <= 5'd0;
		end else begin
			reg_data <= i_data;
			reg_x <= i_x;
			reg_y <= i_y;
			reg_timing <= {i_vs, i_hs, i_de, i_sof, i_eol};
		end
	end

	// Horizontal history uses only valid samples; first columns are explicitly masked.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_row1_d1 <= {C_DATA_WIDTH{1'b0}};
			reg_row1_d2 <= {C_DATA_WIDTH{1'b0}};
			reg_row2_d1 <= {C_DATA_WIDTH{1'b0}};
			reg_row2_d2 <= {C_DATA_WIDTH{1'b0}};
			reg_row3_d1 <= {C_DATA_WIDTH{1'b0}};
			reg_row3_d2 <= {C_DATA_WIDTH{1'b0}};
		end else if(reg_timing[2])begin
			reg_row1_d1 <= line1_data;
			reg_row1_d2 <= reg_row1_d1;
			reg_row2_d1 <= line0_data;
			reg_row2_d2 <= reg_row2_d1;
			reg_row3_d1 <= reg_data;
			reg_row3_d2 <= reg_row3_d1;
		end
	end

	// Stage two emits the complete causal window and its unmodified timing.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			pixels_o <= {(9 * C_DATA_WIDTH){1'b0}};
			timing_o <= 5'd0;
			border_o <= 1'b1;
		end else begin
			pixels_o <= {reg_row1_d2, reg_row1_d1, line1_data,
				reg_row2_d2, reg_row2_d1, line0_data, reg_row3_d2, reg_row3_d1, reg_data};
			timing_o <= reg_timing;
			border_o <= (reg_x < 11'd2) || (reg_y < 10'd2) || (reg_x == LAST_X) || (reg_y == LAST_Y);
		end
	end

	// Read current x only on DE; write previous x one clock later, including at line end.
	// Disabling reads during blanking avoids an accidental read/write at the last x.
	pango_line_ram_1280x8 #(.C_DATA_WIDTH(C_DATA_WIDTH), .C_DEPTH(C_FRAME_WIDTH))
	pango_line_ram_1280x8_Inst_previous_line(
		.i_clk(i_clk), .i_wr_en(flag_write), .i_wr_addr(reg_x), .i_wr_data(reg_data),
		.i_rd_en(i_rst_n && i_de), .i_rd_addr(i_x), .o_rd_data(line0_data)
	);
	pango_line_ram_1280x8 #(.C_DATA_WIDTH(C_DATA_WIDTH), .C_DEPTH(C_FRAME_WIDTH))
	pango_line_ram_1280x8_Inst_second_line(
		.i_clk(i_clk), .i_wr_en(flag_write), .i_wr_addr(reg_x), .i_wr_data(line0_data),
		.i_rd_en(i_rst_n && i_de), .i_rd_addr(i_x), .o_rd_data(line1_data)
	);
endmodule
