//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-11
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 以 tile 内因果 5×5 的导向图 G 与透射率 T 统计计算局部 muT+a×(G-muG)，实现透射率细化。
// Module Name: dehaze_guided_filter
// File Name: source/video/dehaze/dehaze_guided_filter.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 以 tile 内因果 5×5 的导向图 G 与透射率 T 统计计算局部 muT+a×(G-muG)，实现透射率细化。
// 使用 floor 均值与符号幅值 Q8、可配置正则项，固定 41 拍、II=1；本实现不含第二次 a/b box filter。
//
// Dependencies: 官方 dehaze_guided_line_ram、dehaze_guided_div24x16、dehaze_guided_mul24x8、dehaze_mul8x8。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 数学结构参考 demist_filter/guided_core5.v，同步存储与标签为重新实现。
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-11
// Module Name: dehaze_guided_filter
// Project Name: PG2L100H_RK3566_Algorithm_for_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: tile因果5x5局部导向细化；floor均值，符号幅值Q8，II=1。
// Dependencies: 本次官方49位SDPRAM、24/16 Divider、24x8/8x8 APM。
// Source: demist_filter/guided_core5.v 数学结构；重新实现同步存储与标签。
// Revision: 1.0 - 局部muT+a*(G-muG)，不含第二次a/b box filter。
// Revision: 1.1 - G2/GT均值修正分散到已有流水级，保持像素消费拍数。
// Revision: 1.2 - 正则项参数化，17位相加后饱和，保留41拍II=1。
// Revision: 1.4 - 13位G/T均值使用精确倒数单乘法，原流水拍数和8位截断保持。
// Revision: 1.3 - 除法标签数据取消复位，复用26拍有效链恢复完整零气泡。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module dehaze_guided_filter
#(
	parameter [15:0] C_GUIDED_EPSILON = 16'd256
)
(
	input i_clk,
	input i_rst_n,
	input [7:0] i_t,
	input [23:0] i_rgb,
	input [192:0] i_meta,
	output [7:0] o_t,
	output [192:0] o_meta
);
	// meta={apply,A,RGB,tag}; tag坐标、frame/config每个时钟均推进。
	wire [9:0] gray_sum;
	wire [7:0] gray;
	wire [15:0] gray_square;
	wire [15:0] gray_t;
	reg [192:0] reg_moment_meta [0:2];
	reg [7:0] reg_moment_t [0:2];
	reg [7:0] reg_moment_g [0:2];
	wire [48:0] moment_record;
	wire [48:0] ram_data [0:3];
	reg [48:0] reg_read_record;
	reg [192:0] reg_read_meta;
	wire [48:0] row_record [0:4];
	wire [1:0] read_bank;
	reg [10:0] column_g;
	reg [10:0] column_t;
	reg [18:0] column_g2;
	reg [18:0] column_gt;
	reg column_valid;
	reg [192:0] reg_column_meta;
	reg [7:0] reg_column_t;
	reg [7:0] reg_column_g;
	reg [10:0] reg_last_g [0:3];
	reg [10:0] reg_last_t [0:3];
	reg [18:0] reg_last_g2 [0:3];
	reg [18:0] reg_last_gt [0:3];
	reg [3:0] reg_last_valid;
	reg [12:0] reg_sum_g;
	reg [12:0] reg_sum_t;
	reg [20:0] reg_sum_g2;
	reg [20:0] reg_sum_gt;
	reg reg_sum_valid;
	reg [192:0] reg_sum_meta;
	reg [7:0] reg_sum_raw_t;
	reg [7:0] reg_sum_guide;
	reg [7:0] reg_mean_g;
	reg [7:0] reg_mean_t;
	wire [33:0] mean_g2_recip_product;
	wire [33:0] mean_gt_recip_product;
	reg [16:0] reg_mean_g2_q0;
	reg [16:0] reg_mean_gt_q0;
	reg [20:0] reg_mean_sum_g2;
	reg [20:0] reg_mean_sum_gt;
	reg [16:0] reg_mean_g2_q1;
	reg [16:0] reg_mean_gt_q1;
	reg [20:0] reg_mean_sum_g2_pipe;
	reg [20:0] reg_mean_sum_gt_pipe;
	reg [16:0] reg_mean_g2_q2;
	reg [16:0] reg_mean_gt_q2;
	wire [21:0] mean_g2_check0;
	wire [21:0] mean_gt_check0;
	wire [21:0] mean_g2_check1;
	wire [21:0] mean_gt_check1;
	reg reg_mean_valid;
	reg [192:0] reg_mean_meta;
	reg [7:0] reg_mean_raw_t;
	reg [7:0] reg_mean_guide;
	wire [15:0] mean_g_square;
	wire [15:0] mean_g_t;
	wire [15:0] variance;
	wire [16:0] variance_epsilon_sum;
	wire [15:0] epsilon_safe;
	// carry={window_valid,G2/GT占位,meanT,meanG,guide,rawT,meta} 258位；G2/GT在pipe[2]补入。
	wire [257:0] mean_carry;
	reg [257:0] reg_mean_pipe [0:2];
	reg [15:0] reg_cov_abs;
	reg [15:0] reg_variance_epsilon;
	reg reg_cov_negative;
	reg [7:0] reg_delta_abs;
	reg reg_delta_negative;
	reg [7:0] reg_center_t;
	reg [7:0] reg_raw_t;
	reg reg_window_valid;
	reg [192:0] reg_cov_meta;
	wire [39:0] divider_data;
	wire divider_valid;
	wire divider_zero;
	// 无复位的真实Divider：reset会清空有效/标签流水，旧商绝不被选入新像素。
	// carry={window_valid,negative,delta8,meanT8,rawT8,meta193}=219。
	wire [218:0] division_carry;
	wire [218:0] division_carry_filled;
	reg [218:0] reg_div_pipe [0:25];
	reg [25:0] reg_div_valid;
	wire [31:0] correction;
	reg [218:0] reg_product_pipe [0:2];
	wire [23:0] correction_integer;
	wire [7:0] center_t;
	reg [7:0] t_o;
	reg [192:0] meta_o;
	integer k;
	integer div_index;
	genvar bank;
	// reg_sum_g/t只有13位。5243/2^17的正误差在0..8191内小于1/25，
	// 所以右移结果与floor(value/25)完全一致，无需两次余数修正。
	// 返回8位，保持原均值寄存器的截断语义；本轮穷举全部8192输入。
	function [7:0] mean_div25_13;
		input [12:0] value;
		reg [25:0] product_work;
		begin
			product_work = value * 13'd5243;
			mean_div25_13 = product_work[24:17];
		end
	endfunction
	assign o_t = t_o;
	assign o_meta = meta_o;
	assign gray_sum = {2'b0,i_rgb[23:16]} + {1'b0,i_rgb[15:8],1'b0} + {2'b0,i_rgb[7:0]};
	assign gray = gray_sum[9:2];
	dehaze_mul8x8 dehaze_mul8x8_Inst_square(
		.a(gray), .b(gray), .p(gray_square), .clk(i_clk), .rst(!i_rst_n), .ce(1'b1)
	);
	dehaze_mul8x8 dehaze_mul8x8_Inst_cross(
		.a(gray), .b(i_t), .p(gray_t), .clk(i_clk), .rst(!i_rst_n), .ce(1'b1)
	);
	assign moment_record = {reg_moment_meta[2][157] && reg_moment_meta[2][112],
		reg_moment_t[2],reg_moment_g[2],gray_square,gray_t};
	assign read_bank = reg_read_meta[135:134];
	assign row_record[0] = reg_read_record;
	generate
		for(bank = 0; bank < 4; bank = bank + 1)begin:gen_bank
			// 读当前x，下拍写上一x。连续源DE之间无同址读写碰撞。
			dehaze_guided_line_ram dehaze_guided_line_ram_Inst_row(
				.wr_data(reg_read_record),
				.wr_addr(reg_read_meta[154:144]),
				.wr_en(reg_read_meta[157] && read_bank == bank),
				.wr_clk(i_clk),
				.wr_rst(!i_rst_n),
				.rd_addr(reg_moment_meta[2][154:144]),
				.rd_data(ram_data[bank]),
				.rd_clk(i_clk),
				.rd_rst(!i_rst_n)
			);
			assign row_record[bank + 1] = (reg_read_meta[121:113] >= bank + 1) ?
				ram_data[(read_bank + 3 - bank) & 3] : 49'd0;
		end
	endgenerate
	// G2/GT在pipe[2]边界补入；低位均值、像素和meta沿原有三拍前进。
	assign mean_carry = {reg_mean_valid,32'd0,reg_mean_t,reg_mean_g,
		reg_mean_guide,reg_mean_raw_t,reg_mean_meta};
	assign epsilon_safe = C_GUIDED_EPSILON == 16'd0 ? 16'd1 : C_GUIDED_EPSILON;
	assign variance = reg_mean_pipe[2][256:241] >= mean_g_square ?
		reg_mean_pipe[2][256:241] - mean_g_square : 16'd0;
	assign variance_epsilon_sum = {1'b0,variance} + {1'b0,epsilon_safe};
	assign mean_g2_recip_product = {13'd0,reg_sum_g2} * 13'd5243;
	assign mean_gt_recip_product = {13'd0,reg_sum_gt} * 13'd5243;
	// q0最大83887，需要17位；乘25的全精度为22位，不在修正前截断。
	assign mean_g2_check0 = {1'b0,reg_mean_g2_q0,4'd0} + {2'd0,reg_mean_g2_q0,3'd0} + {5'd0,reg_mean_g2_q0};
	assign mean_gt_check0 = {1'b0,reg_mean_gt_q0,4'd0} + {2'd0,reg_mean_gt_q0,3'd0} + {5'd0,reg_mean_gt_q0};
	assign mean_g2_check1 = {1'b0,reg_mean_g2_q1,4'd0} + {2'd0,reg_mean_g2_q1,3'd0} + {5'd0,reg_mean_g2_q1};
	assign mean_gt_check1 = {1'b0,reg_mean_gt_q1,4'd0} + {2'd0,reg_mean_gt_q1,3'd0} + {5'd0,reg_mean_gt_q1};
	dehaze_mul8x8 dehaze_mul8x8_Inst_mean_square(
		.a(reg_mean_g), .b(reg_mean_g), .p(mean_g_square), .clk(i_clk), .rst(!i_rst_n), .ce(1'b1)
	);
	dehaze_mul8x8 dehaze_mul8x8_Inst_mean_cross(
		.a(reg_mean_g), .b(reg_mean_t), .p(mean_g_t), .clk(i_clk), .rst(!i_rst_n), .ce(1'b1)
	);
	assign division_carry = {reg_window_valid,reg_cov_negative ^ reg_delta_negative,
		reg_delta_abs,reg_center_t,reg_raw_t,reg_cov_meta};
	// 完整屏蔽未填满的carry，包含rawT/meta与乘法器输入；保留原零气泡。
	assign division_carry_filled = (i_rst_n && reg_div_valid[25]) ? reg_div_pipe[25] : 219'd0;
	dehaze_guided_div24x16 dehaze_guided_div24x16_Inst_ratio(
		.i_aclk(i_clk),
		.i_axi4s_dividend_tdata({reg_cov_abs,8'd0}),
		.i_axi4s_dividend_tvalid(i_rst_n),
		.i_axi4s_divisor_tdata(reg_variance_epsilon),
		.i_axi4s_divisor_tvalid(i_rst_n),
		.o_axi4s_dout_tvalid(divider_valid),
		.o_axi4s_dout_tdata(divider_data),
		.o_axi4s_dout_tuser(divider_zero)
	);
	dehaze_guided_mul24x8 dehaze_guided_mul24x8_Inst_correction(
		.a(divider_data[39:16]),
		.b(division_carry_filled[216:209]),
		.p(correction),
		.clk(i_clk),
		.rst(!i_rst_n),
		.ce(1'b1)
	);
	assign correction_integer = correction[31:8];
	assign center_t = reg_product_pipe[2][208:201];
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			for(k = 0; k < 3; k = k + 1)begin
				reg_moment_meta[k] <= 0;
				reg_moment_t[k] <= 0;
				reg_moment_g[k] <= 0;
				reg_mean_pipe[k] <= 0;
				reg_product_pipe[k] <= 0;
			end
			reg_read_record <= 0;
			reg_read_meta <= 0;
			column_g <= 0;
			column_t <= 0;
			column_g2 <= 0;
			column_gt <= 0;
			column_valid <= 0;
			reg_column_meta <= 0;
			reg_column_t <= 0;
			reg_column_g <= 0;
			for(k = 0; k < 4; k = k + 1)begin
				reg_last_g[k] <= 0;
				reg_last_t[k] <= 0;
				reg_last_g2[k] <= 0;
				reg_last_gt[k] <= 0;
			end
			reg_last_valid <= 0;
			reg_sum_g <= 0;
			reg_sum_t <= 0;
			reg_sum_g2 <= 0;
			reg_sum_gt <= 0;
			reg_sum_valid <= 0;
			reg_sum_meta <= 0;
			reg_sum_raw_t <= 0;
			reg_sum_guide <= 0;
			reg_mean_g <= 0;
			reg_mean_t <= 0;
			reg_mean_g2_q0 <= 0;
			reg_mean_gt_q0 <= 0;
			reg_mean_sum_g2 <= 0;
			reg_mean_sum_gt <= 0;
			reg_mean_g2_q1 <= 0;
			reg_mean_gt_q1 <= 0;
			reg_mean_sum_g2_pipe <= 0;
			reg_mean_sum_gt_pipe <= 0;
			reg_mean_g2_q2 <= 0;
			reg_mean_gt_q2 <= 0;
			reg_mean_valid <= 0;
			reg_mean_meta <= 0;
			reg_mean_raw_t <= 0;
			reg_mean_guide <= 0;
			reg_cov_abs <= 0;
			reg_variance_epsilon <= 1;
			reg_cov_negative <= 0;
			reg_delta_abs <= 0;
			reg_delta_negative <= 0;
			reg_center_t <= 0;
			reg_raw_t <= 0;
			reg_window_valid <= 0;
			reg_cov_meta <= 0;
			reg_div_valid <= 0;
			t_o <= 0;
			meta_o <= 0;
		end else begin
			reg_moment_meta[0] <= i_meta;
			reg_moment_t[0] <= i_t;
			reg_moment_g[0] <= gray;
			for(k = 1; k < 3; k = k + 1)begin
				reg_moment_meta[k] <= reg_moment_meta[k - 1];
				reg_moment_t[k] <= reg_moment_t[k - 1];
				reg_moment_g[k] <= reg_moment_g[k - 1];
			end
			reg_read_record <= moment_record;
			reg_read_meta <= reg_moment_meta[2];
			column_g <= {3'd0,row_record[0][39:32]} + {3'd0,row_record[1][39:32]} +
				{3'd0,row_record[2][39:32]} + {3'd0,row_record[3][39:32]} + {3'd0,row_record[4][39:32]};
			column_t <= {3'd0,row_record[0][47:40]} + {3'd0,row_record[1][47:40]} +
				{3'd0,row_record[2][47:40]} + {3'd0,row_record[3][47:40]} + {3'd0,row_record[4][47:40]};
			column_g2 <= {3'd0,row_record[0][31:16]} + {3'd0,row_record[1][31:16]} +
				{3'd0,row_record[2][31:16]} + {3'd0,row_record[3][31:16]} + {3'd0,row_record[4][31:16]};
			column_gt <= {3'd0,row_record[0][15:0]} + {3'd0,row_record[1][15:0]} +
				{3'd0,row_record[2][15:0]} + {3'd0,row_record[3][15:0]} + {3'd0,row_record[4][15:0]};
			column_valid <= row_record[0][48] && row_record[1][48] && row_record[2][48] && row_record[3][48] && row_record[4][48];
			reg_column_meta <= reg_read_meta;
			reg_column_t <= reg_read_record[47:40];
			reg_column_g <= reg_read_record[39:32];
			reg_sum_meta <= reg_column_meta;
			reg_sum_raw_t <= reg_column_t;
			reg_sum_guide <= reg_column_g;
			reg_sum_g <= {2'd0,column_g} + {2'd0,reg_last_g[0]} + {2'd0,reg_last_g[1]} + {2'd0,reg_last_g[2]} + {2'd0,reg_last_g[3]};
			reg_sum_t <= {2'd0,column_t} + {2'd0,reg_last_t[0]} + {2'd0,reg_last_t[1]} + {2'd0,reg_last_t[2]} + {2'd0,reg_last_t[3]};
			reg_sum_g2 <= {2'd0,column_g2} + {2'd0,reg_last_g2[0]} + {2'd0,reg_last_g2[1]} + {2'd0,reg_last_g2[2]} + {2'd0,reg_last_g2[3]};
			reg_sum_gt <= {2'd0,column_gt} + {2'd0,reg_last_gt[0]} + {2'd0,reg_last_gt[1]} + {2'd0,reg_last_gt[2]} + {2'd0,reg_last_gt[3]};
			reg_sum_valid <= reg_column_meta[157] && reg_column_meta[112] && reg_column_meta[131:122] >= 4 && column_valid && (&reg_last_valid);
			if(reg_column_meta[157])begin
				reg_last_g[0] <= column_g;
				reg_last_t[0] <= column_t;
				reg_last_g2[0] <= column_g2;
				reg_last_gt[0] <= column_gt;
				for(k = 1; k < 4; k = k + 1)begin
					reg_last_g[k] <= reg_last_g[k - 1];
					reg_last_t[k] <= reg_last_t[k - 1];
					reg_last_g2[k] <= reg_last_g2[k - 1];
					reg_last_gt[k] <= reg_last_gt[k - 1];
				end
				reg_last_valid <= {reg_last_valid[2:0],column_valid};
			end else begin
				reg_last_valid <= 0;
			end
			// 固定25除法为常数算术；可变方差除法使用真实官方流水IP。
			reg_mean_g <= mean_div25_13(reg_sum_g);
			reg_mean_t <= mean_div25_13(reg_sum_t);
			reg_mean_g2_q0 <= mean_g2_recip_product[33:17];
			reg_mean_gt_q0 <= mean_gt_recip_product[33:17];
			reg_mean_sum_g2 <= reg_sum_g2;
			reg_mean_sum_gt <= reg_sum_gt;
			reg_mean_g2_q1 <= mean_g2_check0 > {1'b0,reg_mean_sum_g2} ? reg_mean_g2_q0 - 17'd1 : reg_mean_g2_q0;
			reg_mean_gt_q1 <= mean_gt_check0 > {1'b0,reg_mean_sum_gt} ? reg_mean_gt_q0 - 17'd1 : reg_mean_gt_q0;
			reg_mean_sum_g2_pipe <= reg_mean_sum_g2;
			reg_mean_sum_gt_pipe <= reg_mean_sum_gt;
			reg_mean_g2_q2 <= mean_g2_check1 > {1'b0,reg_mean_sum_g2_pipe} ? reg_mean_g2_q1 - 17'd1 : reg_mean_g2_q1;
			reg_mean_gt_q2 <= mean_gt_check1 > {1'b0,reg_mean_sum_gt_pipe} ? reg_mean_gt_q1 - 17'd1 : reg_mean_gt_q1;
			reg_mean_valid <= reg_sum_valid;
			reg_mean_meta <= reg_sum_meta;
			reg_mean_raw_t <= reg_sum_raw_t;
			reg_mean_guide <= reg_sum_guide;
			reg_mean_pipe[0] <= mean_carry;
			reg_mean_pipe[1] <= reg_mean_pipe[0];
			reg_mean_pipe[2] <= {reg_mean_pipe[1][257],reg_mean_g2_q2[15:0],reg_mean_gt_q2[15:0],reg_mean_pipe[1][224:0]};
			reg_cov_abs <= reg_mean_pipe[2][240:225] >= mean_g_t ?
				reg_mean_pipe[2][240:225] - mean_g_t : mean_g_t - reg_mean_pipe[2][240:225];
			reg_cov_negative <= reg_mean_pipe[2][240:225] < mean_g_t;
			reg_variance_epsilon <= variance_epsilon_sum[16] ? 16'hffff : variance_epsilon_sum[15:0];
			reg_delta_abs <= reg_mean_pipe[2][208:201] >= reg_mean_pipe[2][216:209] ?
				reg_mean_pipe[2][208:201] - reg_mean_pipe[2][216:209] : reg_mean_pipe[2][216:209] - reg_mean_pipe[2][208:201];
			reg_delta_negative <= reg_mean_pipe[2][208:201] < reg_mean_pipe[2][216:209];
			reg_center_t <= reg_mean_pipe[2][224:217];
			reg_raw_t <= reg_mean_pipe[2][200:193];
			reg_window_valid <= reg_mean_pipe[2][257];
			reg_cov_meta <= reg_mean_pipe[2][192:0];
			reg_div_valid <= {reg_div_valid[24:0],1'b1};
			// 官方ZERO_DETECT相对商延后一拍；合法分母由本模块保证，不用该输出作像素门控。
			reg_product_pipe[0] <= {division_carry_filled[218] && reg_div_valid[25] && divider_valid,division_carry_filled[217:0]};
			for(k = 1; k < 3; k = k + 1)begin
				reg_product_pipe[k] <= reg_product_pipe[k - 1];
			end
			meta_o <= reg_product_pipe[2][192:0];
			if(!reg_product_pipe[2][218])begin
				t_o <= reg_product_pipe[2][200:193];
			end else if(reg_product_pipe[2][217])begin
				t_o <= correction_integer >= center_t ? 8'd0 : center_t - correction_integer[7:0];
			end else begin
				t_o <= correction_integer >= 255 - {16'd0,center_t} ? 8'd255 : center_t + correction_integer[7:0];
			end
		end
	end
	// 纯数据链每拍推进，复位只清空既有有效链；运算IP与41拍延迟保持原样。
	always@(posedge i_clk)begin
		reg_div_pipe[0] <= division_carry;
		for(div_index = 1; div_index < 26; div_index = div_index + 1)begin
			reg_div_pipe[div_index] <= reg_div_pipe[div_index - 1];
		end
	end
endmodule
