//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-09
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 按 Y=(77R+150G+29B)>>8 转换灰度，对因果 3×3 邻域排序取中值，并将结果复制到 RGB 三通道。
// Module Name: algo_gray_median_pango
// File Name: source/video/algo_gray_median_pango.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 按 Y=(77R+150G+29B)>>8 转换灰度，对因果 3×3 邻域排序取中值，并将结果复制到 RGB 三通道。
// 灰度计算、窗口和奇偶排序合计固定 12 拍，边界像素按约定置黑。
//
// Dependencies: video_window_3x3、其内部 pango_line_ram_1280x8。
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
// Design Name: Gray Median HDMI Algorithm
// Module Name: algo_gray_median_pango
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Y=(77R+150G+29B)>>8, causal 3x3 median, RGB gray output.
//              Fixed 12 clocks: registered Y(1), window(2), odd-even sorting(9).
//              Black first two rows/columns and final row/column; center offset +1,+1.
// Dependencies: video_window_3x3, pango_line_ram_1280x8
// Revision: 0.01 - Pango-oriented RTL; no Xilinx generated IP imported.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module algo_gray_median_pango
#(
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
	input [23:0] i_rgb,
	input [10:0] i_x,
	input [9:0] i_y,
	output o_vs,
	output o_hs,
	output o_de,
	output o_sof,
	output o_eol,
	output [23:0] o_rgb
);
	wire [15:0] gray_sum;
	reg [7:0] reg_gray;
	reg [4:0] reg_timing;
	reg [10:0] reg_x;
	reg [9:0] reg_y;
	wire [4:0] window_timing;
	wire window_border;
	wire [71:0] window_pixels;
	wire [71:0] sorted_pixels [0:9];
	wire [5:0] sorted_timing [0:9];
	wire [7:0] median_value;
	genvar stage_index;

	// Maximum weighted sum is 65280; sixteen unsigned bits preserve all terms.
	assign gray_sum = i_rgb[23:16] * 16'd77 + i_rgb[15:8] * 16'd150 + i_rgb[7:0] * 16'd29;
	assign sorted_pixels[0] = window_pixels;
	assign sorted_timing[0] = {window_border, window_timing};
	assign {o_vs, o_hs, o_de, o_sof, o_eol} = sorted_timing[9][4:0];
	assign median_value = sorted_pixels[9][39:32];
	assign o_rgb = (sorted_timing[9][5] || !o_de) ? 24'd0 : {median_value, median_value, median_value};

	// One registered color conversion stage travels with all canonical metadata.
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_gray <= 8'd0;
			reg_timing <= 5'd0;
			reg_x <= 11'd0;
			reg_y <= 10'd0;
		end else begin
			reg_gray <= gray_sum[15:8];
			reg_timing <= {i_vs, i_hs, i_de, i_sof, i_eol};
			reg_x <= i_x;
			reg_y <= i_y;
		end
	end

	// Nine odd/even adjacent-comparison phases completely sort nine unsigned values.
	generate
		for(stage_index = 0; stage_index < 9; stage_index = stage_index + 1)begin:gen_sort_stage
			reg [71:0] reg_sorted;
			reg [71:0] dec_sorted;
			reg [5:0] reg_sort_timing;
			integer cnt_pair;
			always@(*)begin
				dec_sorted = sorted_pixels[stage_index];
				for(cnt_pair = stage_index % 2; cnt_pair < 8; cnt_pair = cnt_pair + 2)begin
					if(sorted_pixels[stage_index][cnt_pair * 8 +: 8] > sorted_pixels[stage_index][(cnt_pair + 1) * 8 +: 8])begin
						dec_sorted[cnt_pair * 8 +: 8] = sorted_pixels[stage_index][(cnt_pair + 1) * 8 +: 8];
						dec_sorted[(cnt_pair + 1) * 8 +: 8] = sorted_pixels[stage_index][cnt_pair * 8 +: 8];
					end
				end
			end
			always@(posedge i_clk or negedge i_rst_n)begin
				if(!i_rst_n)begin
					reg_sorted <= 72'd0;
					reg_sort_timing <= 6'd0;
				end else begin
					reg_sorted <= dec_sorted;
					reg_sort_timing <= sorted_timing[stage_index];
				end
			end
			assign sorted_pixels[stage_index + 1] = reg_sorted;
			assign sorted_timing[stage_index + 1] = reg_sort_timing;
		end
	endgenerate

	video_window_3x3 #(
		.C_DATA_WIDTH(8), .C_FRAME_WIDTH(C_FRAME_WIDTH), .C_FRAME_HEIGHT(C_FRAME_HEIGHT)
	) video_window_3x3_Inst_gray(
		.i_clk(i_clk), .i_rst_n(i_rst_n),
		.i_vs(reg_timing[4]), .i_hs(reg_timing[3]), .i_de(reg_timing[2]),
		.i_sof(reg_timing[1]), .i_eol(reg_timing[0]), .i_data(reg_gray), .i_x(reg_x), .i_y(reg_y),
		.o_vs(window_timing[4]), .o_hs(window_timing[3]), .o_de(window_timing[2]),
		.o_sof(window_timing[1]), .o_eol(window_timing[0]), .o_border(window_border),
		.o_p11(window_pixels[71:64]), .o_p12(window_pixels[63:56]), .o_p13(window_pixels[55:48]),
		.o_p21(window_pixels[47:40]), .o_p22(window_pixels[39:32]), .o_p23(window_pixels[31:24]),
		.o_p31(window_pixels[23:16]), .o_p32(window_pixels[15:8]), .o_p33(window_pixels[7:0])
	);
endmodule
