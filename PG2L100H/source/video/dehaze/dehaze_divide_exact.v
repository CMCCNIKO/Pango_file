//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 以 ROM 倒数乘法和一次商修正计算无符号 16/8 精确整数商，固定 8 拍、II=1。
// Module Name: dehaze_divide_exact
// File Name: source/video/dehaze/dehaze_divide_exact.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 以 ROM 倒数乘法和一次商修正计算无符号 16/8 精确整数商，固定 8 拍、II=1。
// 除数为零时按 1 防护，数据和标签使用相同流水，有效链在复位填充期恢复零气泡。
//
// Dependencies: 官方 dehaze_recip_rom、dehaze_mul16x17、dehaze_mul17x8。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-10
// Module: dehaze_divide_exact
// Project: PG2L100H HDMI dehaze; PDS 2022.2-SP6.4
// Device: PG2L100H-6FBG676
// Description: 无符号16/8精确整数商；ROM倒数+一次修正，8拍，II=1。
// Dependencies: 本次官方 dehaze_recip_rom/mul16x17/mul17x8。
// Revision: 1.0 初始实现。d=0防护为1，调用方仍须提供合法参数。
// Revision: 1.1 标签取消复位，8拍填充有效链恢复原零气泡，数值流水保持原样。
//////////////////////////////////////////////////////////////////////////////
module dehaze_divide_exact #(
	parameter C_TAG_WIDTH = 160
) (
	input wire i_clk,
	input wire i_rst_n,
	input wire [15:0] i_numerator,
	input wire [7:0] i_denominator,
	input wire [C_TAG_WIDTH-1:0] i_tag,
	output wire [16:0] o_quotient,
	output wire [C_TAG_WIDTH-1:0] o_tag
);
	wire [7:0] denominator_safe = (i_denominator == 0) ? 8'd1 : i_denominator;
	wire [16:0] reciprocal;
	wire [32:0] estimate_product;
	wire [16:0] estimate = estimate_product[32:16];
	wire [24:0] correction_product;
	reg [15:0] numerator_pipe [0:6];
	reg [7:0] denominator_pipe [0:6];
	reg [16:0] estimate_pipe [0:2];
	reg [C_TAG_WIDTH-1:0] tag_pipe [0:7];
	reg [7:0] reg_tag_filled;
	reg [16:0] quotient_o;
	integer k;
	integer tag_index;
	dehaze_recip_rom u_reciprocal (
		.addr({1'b0,denominator_safe}), .rd_data(reciprocal), .clk(i_clk), .rst(~i_rst_n)
	);
	dehaze_mul16x17 u_estimate (
		.a(numerator_pipe[0]), .b(reciprocal), .p(estimate_product),
		.clk(i_clk), .rst(~i_rst_n), .ce(1'b1)
	);
	dehaze_mul17x8 u_correction (
		.a(estimate), .b(denominator_pipe[3]), .p(correction_product),
		.clk(i_clk), .rst(~i_rst_n), .ce(1'b1)
	);
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			quotient_o <= 0;
			for (k=0;k<7;k=k+1) begin numerator_pipe[k]<=0; denominator_pipe[k]<=1; end
			for (k=0;k<3;k=k+1) estimate_pipe[k]<=0;
			reg_tag_filled <= 8'd0;
		end else begin
			numerator_pipe[0] <= i_numerator;
			denominator_pipe[0] <= denominator_safe;
			for (k=1;k<7;k=k+1) begin
				numerator_pipe[k] <= numerator_pipe[k-1];
				denominator_pipe[k] <= denominator_pipe[k-1];
			end
			estimate_pipe[0] <= estimate;
			for(k=1;k<3;k=k+1) estimate_pipe[k] <= estimate_pipe[k-1];
			reg_tag_filled <= {reg_tag_filled[6:0],1'b1};
			// 扩展比较与减法，估商从不超过真商；只需要向上修正一次。
			if ({9'd0,numerator_pipe[6]} >= correction_product + {17'd0,denominator_pipe[6]})
				quotient_o <= estimate_pipe[2] + 17'd1;
			else quotient_o <= estimate_pipe[2];
		end
	end
	// 不复位纯标签数据，完整标签在填满前输出零；吞吐与固定8拍不变。
	always@(posedge i_clk)begin
		tag_pipe[0] <= i_tag;
		for(tag_index = 1; tag_index < 8; tag_index = tag_index + 1)begin
			tag_pipe[tag_index] <= tag_pipe[tag_index - 1];
		end
	end
	assign o_quotient = quotient_o;
	assign o_tag = (i_rst_n && reg_tag_filled[7]) ? tag_pipe[7] : {C_TAG_WIDTH{1'b0}};
endmodule
