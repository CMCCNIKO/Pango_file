//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-26
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 由 640 列 RGB565 流计算灰度和低饱和度条件，再与 61 列居中背景比较，输出带坐标和源帧号的二值斑马线掩码。
// Module Name: host_f14_crosswalk_mask
// File Name: host/rtl/video/host_f14_crosswalk_mask.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 由 640 列 RGB565 流计算灰度和低饱和度条件，再与 61 列居中背景比较，输出带坐标和源帧号的二值斑马线掩码。
// 边界采用 REFLECT_101；64 点环形缓存支持行尾右侧 30 点排空与下一行前 30 点输入重叠，不反压输入像素。
//
// Dependencies: 像素域 i_clk/i_rst_n；有序行列坐标及 i_abort；内部小型 LUT RAM 多读口可能由综合工具复制。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-26
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// 640列RGB565流：精确灰度/低饱和度/61列居中背景，REFLECT_101边界。
// 输入不背压，可连续相邻行；右边30点排空与下一行前30点同时执行。
// 64点小环形缓存有3个读地址，工具可能复制小LUT RAM，须查综合。
`timescale 1ns/1ps
`default_nettype none
module host_f14_crosswalk_mask (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_abort,
	input wire i_valid,
	input wire [15:0] i_rgb565,
	input wire [8:0] i_row,
	input wire [9:0] i_col,
	input wire [15:0] i_frame_id,
	output wire o_valid,
	output wire o_mask,
	output wire [8:0] o_row,
	output wire [9:0] o_col,
	output wire [15:0] o_frame_id
);
	wire [7:0] red,green,blue,maximum,minimum,delta;
	wire [16:0] saturation_lhs,saturation_rhs;
	wire [22:0] gray_acc;
	reg valid_q,white_q;
	reg [7:0] gray_q;
	reg [8:0] row_q;
	reg [9:0] col_q;
	reg [15:0] frame_q;
	reg [8:0] pixels [0:63] /* synthesis syn_ramstyle="lut_ram" */;
	reg [13:0] sum_q,prefix_q;
	reg tail_q;
	reg [4:0] tail_index_q;
	reg [8:0] tail_row_q;
	reg [15:0] tail_frame_q;
	reg out_valid_q,out_mask_q;
	reg [8:0] out_row_q;
	reg [9:0] out_col_q;
	reg [15:0] out_frame_q;
	wire [9:0] center_col,leave_col,reflect_col;
	wire [5:0] center_addr,leave_addr,reflect_addr;
	wire [8:0] center_data,leave_data,reflect_data;
	wire [14:0] updated_sum,center_times61;
	wire emitting;
	assign red={i_rgb565[15:11],i_rgb565[15:13]};
	assign green={i_rgb565[10:5],i_rgb565[10:9]};
	assign blue={i_rgb565[4:0],i_rgb565[4:2]};
	assign maximum=red>=green ? (red>=blue ? red : blue) : (green>=blue ? green : blue);
	assign minimum=red<=green ? (red<=blue ? red : blue) : (green<=blue ? green : blue);
	assign delta=maximum-minimum;
	// 65536种RGB565逐一验证，等价于OpenCV S<=100；无需HSV除法器。
	assign saturation_lhs=({9'd0,delta}<<9)-({9'd0,delta}<<1);
	assign saturation_rhs=({9'd0,maximum}<<7)+({9'd0,maximum}<<6)+
		({9'd0,maximum}<<3)+{9'd0,maximum};
	// OpenCV的8bit灰度使用15位系数；14位近似会有270种颜色相差1。
	assign gray_acc=red*23'd9798+green*23'd19235+blue*23'd3735+23'd16384;
	assign center_col=tail_q ? 10'd610+{5'd0,tail_index_q} : col_q-10'd30;
	assign leave_col=tail_q ? 10'd579+{5'd0,tail_index_q} :
		col_q<=60 ? 10'd61-col_q : col_q-10'd61;
	assign reflect_col=10'd638-{5'd0,tail_index_q};
	assign center_addr=center_col[5:0];
	assign leave_addr=leave_col[5:0];
	assign reflect_addr=reflect_col[5:0];
	assign center_data=pixels[center_addr];
	assign leave_data=pixels[leave_addr];
	assign reflect_data=pixels[reflect_addr];
	assign updated_sum=tail_q ? {1'b0,sum_q}+{7'd0,reflect_data[7:0]}-{7'd0,leave_data[7:0]} :
		col_q==30 ? {1'b0,prefix_q}+({7'd0,gray_q}<<1) :
		{1'b0,sum_q}+{7'd0,gray_q}-{7'd0,leave_data[7:0]};
	assign center_times61=({7'd0,center_data[7:0]}<<6)-
		({7'd0,center_data[7:0]}<<1)-{7'd0,center_data[7:0]};
	assign emitting=tail_q || (valid_q && col_q>=30);
	assign o_valid=out_valid_q;
	assign o_mask=out_mask_q;
	assign o_row=out_row_q;
	assign o_col=out_col_q;
	assign o_frame_id=out_frame_q;
	always @(posedge i_clk) begin
		if(valid_q && !i_abort) begin
			pixels[col_q[5:0]]<={white_q,gray_q};
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			valid_q<=0; gray_q<=0; white_q<=0; row_q<=0; col_q<=0; frame_q<=0;
		end else begin
			valid_q<=i_valid && !i_abort;
			if(i_valid) begin
				gray_q<=gray_acc[22:15];
				white_q<=maximum==0 || saturation_lhs<saturation_rhs;
				row_q<=i_row; col_q<=i_col; frame_q<=i_frame_id;
			end
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if(!i_rst_n) begin
			sum_q<=0; prefix_q<=0; tail_q<=0; tail_index_q<=0;
			tail_row_q<=0; tail_frame_q<=0;
			out_valid_q<=0; out_mask_q<=0; out_row_q<=0; out_col_q<=0; out_frame_q<=0;
		end else if(i_abort) begin
			tail_q<=0; out_valid_q<=0; sum_q<=0; prefix_q<=0;
		end else begin
			out_valid_q<=emitting;
			if(emitting) begin
				out_mask_q<=center_data[8] && center_data[7:0]>=160 &&
					center_times61>=updated_sum+15'd1220;
				out_col_q<=center_col;
				out_row_q<=tail_q ? tail_row_q : row_q;
				out_frame_q<=tail_q ? tail_frame_q : frame_q;
				sum_q<=updated_sum[13:0];
			end
			if(tail_q) begin
				if(tail_index_q==29) begin tail_q<=0; end
				else begin tail_index_q<=tail_index_q+1'b1; end
			end
			if(valid_q) begin
				if(col_q==0) begin prefix_q<={6'd0,gray_q}; end
				else if(col_q<30) begin prefix_q<=prefix_q+({6'd0,gray_q}<<1); end
				if(col_q==639) begin
					tail_q<=1; tail_index_q<=0; tail_row_q<=row_q; tail_frame_q<=frame_q;
				end
			end
		end
	end
endmodule
`default_nettype wire
