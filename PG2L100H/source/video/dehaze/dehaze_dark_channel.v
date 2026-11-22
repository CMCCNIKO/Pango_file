//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 利用四个独立行 RAM bank 构建 tile 内因果 5×5 窗口，求取去雾暗通道。
// Module Name: dehaze_dark_channel
// File Name: source/video/dehaze/dehaze_dark_channel.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 利用四个独立行 RAM bank 构建 tile 内因果 5×5 窗口，求取去雾暗通道。
// 像素、边界及完整 160 位 tag 同步推进，固定 2 拍、II=1，避免不同 tile 之间窗口串扰。
//
// Dependencies: 官方 dehaze_line0_ram（2048×8），实例化四次。
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
// Create Date: 2026-10-10
// Design Name: tile 因果暗通道
// Module Name: dehaze_dark_channel
// Project Name: PG2L100H_RK3566_Algorithm_for_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 真实四bank RAM，tile内因果5x5，完整160位tag等拍推进，II=1。
// Dependencies: 用户官方生成 dehaze_line0_ram，2048x8配置独立实例化四次。
// Revision: 0.01 - File Created
//           0.02 - 2026-10-11，参考demist_filter的5x5窗口，保留因果锚点与2拍契约。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module dehaze_dark_channel
(
	input i_clk,
	input i_rst_n,
	input [23:0] i_rgb,
	input [159:0] i_tag,
	output [23:0] o_rgb,
	output [159:0] o_tag,
	output [7:0] o_dark
);
	reg [23:0] reg_rgb_read;
	reg [159:0] reg_tag_read;
	reg [7:0] reg_min_read;
	reg [7:0] reg_column_last;
	reg [7:0] reg_column_last2;
	reg [7:0] reg_column_last3;
	reg [7:0] reg_column_last4;
	reg [23:0] rgb_o;
	reg [159:0] tag_o;
	reg [7:0] dark_o;
	wire [7:0] minimum_rg;
	wire [7:0] minimum_rgb;
	wire [7:0] input_minimum;
	wire [7:0] ram_data [0:3];
	wire [1:0] previous_bank;
	wire [1:0] previous_bank2;
	wire [1:0] previous_bank3;
	wire [7:0] previous_row;
	wire [7:0] previous_row2;
	wire [7:0] previous_row3;
	wire [7:0] previous_row4;
	wire [7:0] vertical_min1;
	wire [7:0] vertical_min2;
	wire [7:0] vertical_min3;
	wire [7:0] vertical_min;
	wire [7:0] horizontal_last;
	wire [7:0] horizontal_last2;
	wire [7:0] horizontal_last3;
	wire [7:0] horizontal_last4;
	wire [7:0] horizontal_min1;
	wire [7:0] horizontal_min2;
	wire [7:0] horizontal_min;
	genvar bank;
	assign o_rgb = rgb_o;
	assign o_tag = tag_o;
	assign o_dark = dark_o;
	assign minimum_rg = (i_rgb[23:16] <= i_rgb[15:8]) ? i_rgb[23:16] : i_rgb[15:8];
	assign minimum_rgb = (minimum_rg <= i_rgb[7:0]) ? minimum_rg : i_rgb[7:0];
	assign input_minimum = (i_tag[157] && i_tag[112]) ? minimum_rgb : 8'd255;
	// 全局y模4仅选择存储bank；tile_y屏蔽旧帧与上方tile的行。
	// 当前bank读出四行前数据，再在下一拍写入当前行，避免同址碰撞依赖。
	assign previous_bank = reg_tag_read[135:134] - 2'd1;
	assign previous_bank2 = reg_tag_read[135:134] - 2'd2;
	assign previous_bank3 = reg_tag_read[135:134] - 2'd3;
	assign previous_row = (reg_tag_read[121:113] >= 9'd1) ? ram_data[previous_bank] : 8'd255;
	assign previous_row2 = (reg_tag_read[121:113] >= 9'd2) ? ram_data[previous_bank2] : 8'd255;
	assign previous_row3 = (reg_tag_read[121:113] >= 9'd3) ? ram_data[previous_bank3] : 8'd255;
	assign previous_row4 = (reg_tag_read[121:113] >= 9'd4) ? ram_data[reg_tag_read[135:134]] : 8'd255;
	assign vertical_min1 = (previous_row <= previous_row2) ? previous_row : previous_row2;
	assign vertical_min2 = (previous_row3 <= previous_row4) ? previous_row3 : previous_row4;
	assign vertical_min3 = (vertical_min1 <= vertical_min2) ? vertical_min1 : vertical_min2;
	assign vertical_min = (reg_min_read <= vertical_min3) ? reg_min_read : vertical_min3;
	assign horizontal_last = (reg_tag_read[131:122] >= 10'd1) ? reg_column_last : 8'd255;
	assign horizontal_last2 = (reg_tag_read[131:122] >= 10'd2) ? reg_column_last2 : 8'd255;
	assign horizontal_last3 = (reg_tag_read[131:122] >= 10'd3) ? reg_column_last3 : 8'd255;
	assign horizontal_last4 = (reg_tag_read[131:122] >= 10'd4) ? reg_column_last4 : 8'd255;
	assign horizontal_min1 = (horizontal_last <= horizontal_last2) ? horizontal_last : horizontal_last2;
	assign horizontal_min2 = (horizontal_last3 <= horizontal_last4) ? horizontal_last3 : horizontal_last4;
	assign horizontal_min = (horizontal_min1 <= horizontal_min2) ? horizontal_min1 : horizontal_min2;

	// 本拍读当前x，下拍写上一x，避免依赖厂商同址读写碰撞模式。
	generate
		for(bank = 0; bank < 4; bank = bank + 1)begin:gen_line_bank
			dehaze_line0_ram dehaze_line0_ram_Inst_bank(
				.wr_data(reg_min_read),
				.wr_addr(reg_tag_read[154:144]),
				.wr_en(reg_tag_read[157] && (reg_tag_read[135:134] == bank)),
				.wr_clk(i_clk),
				.wr_rst(!i_rst_n),
				.rd_data(ram_data[bank]),
				.rd_addr(i_tag[154:144]),
				.rd_clk(i_clk),
				.rd_clk_en(i_tag[157]),
				.rd_rst(!i_rst_n)
			);
		end
	endgenerate

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			reg_rgb_read <= 24'd0;
			reg_tag_read <= 160'd0;
			reg_min_read <= 8'd255;
		end else begin
			reg_rgb_read <= i_rgb;
			reg_tag_read <= i_tag;
			reg_min_read <= input_minimum;
		end
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			rgb_o <= 24'd0;
			tag_o <= 160'd0;
			dark_o <= 8'd255;
			reg_column_last <= 8'd255;
			reg_column_last2 <= 8'd255;
			reg_column_last3 <= 8'd255;
			reg_column_last4 <= 8'd255;
		end else begin
			rgb_o <= reg_rgb_read;
			tag_o <= reg_tag_read;
			if(reg_tag_read[157])begin
				dark_o <= (vertical_min <= horizontal_min) ? vertical_min : horizontal_min;
				reg_column_last <= vertical_min;
				reg_column_last2 <= reg_column_last;
				reg_column_last3 <= reg_column_last2;
				reg_column_last4 <= reg_column_last3;
			end else begin
				dark_o <= 8'd255;
				reg_column_last <= 8'd255;
				reg_column_last2 <= 8'd255;
				reg_column_last3 <= 8'd255;
				reg_column_last4 <= 8'd255;
			end
		end
	end
endmodule
