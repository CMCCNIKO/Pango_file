//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 当前 TinySequence 线型头封装：沿左右内侧车道构造整数等弧长 21 点，以 Q8 双线性采样 S8 特征，顺序复用一个串行分类核心。
// Module Name: host_f14_linetype_tiny_head
// File Name: host/rtl/video/host_f14_linetype_tiny_head.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 当前 TinySequence 线型头封装：沿左右内侧车道构造整数等弧长 21 点，以 Q8 双线性采样 S8 特征，顺序复用一个串行分类核心。
// 输出实线/虚线类别、支持度和置信度，并提供取样、logit、拍数及调试观测；多周期数学与特征读返回严格按状态机配合。
//
// Dependencies: host_f14_uint_math、host_f14_tiny_serial、host_f14_old_coordinates.vh；输入表与 S8 快照所有权保持至 done；i_clk/i_rst_n。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 保留 2026-10-29 修订：累计弧长寄存、端点 RAM 单读口分拍锁存、小 RAM 数据不复位且先写后读。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// 复用冻结整数取样器；左右顺序共用TinySequence串行核心。
// 输入表在start至done保持稳定；S8快照由调用方保持所有权。只分类S1/S2。
// 多周期任务用单时序FSM保持除法返回与数据寄存器同步；无组合动态除法。
// 2026-10-29 修订：累计弧长用寄存器；端点RAM每个仅一个读端口，分拍锁存。
// 小RAM独立同步写、无数据复位；每条线写完有效区后才读取。
`timescale 1ns/1ps
`default_nettype none
module host_f14_linetype_tiny_head #(
	parameter integer C_CUTOFF_Y = 320
) (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_start,
	input wire i_error,
	input wire [839:0] i_x_table,
	input wire [83:0] i_valid_table,
	input wire [7:0] i_geometry_types,
	output reg o_done,
	output reg [7:0] o_types,
	output reg o_feature_req,
	output reg [13:0] o_feature_addr,
	input wire i_feature_valid,
	input wire [7:0] i_feature_data,
	output wire o_busy,
	output reg o_error,
	output reg [1:0] o_supported,
	output reg [1:0] o_high,
	output reg [1:0] o_medium,
	output wire o_sample_valid,
	output wire [1:0] o_slot,
	output wire [8:0] o_sample_addr,
	output wire [15:0] o_sample_data,
	output wire o_logit_valid,
	output wire signed [31:0] o_logit_solid,
	output wire signed [31:0] o_logit_dashed,
	output wire [31:0] o_cycles,
	output wire [6:0] o_debug_state,
	output wire [7:0] o_debug_math,
	output wire [31:0] o_debug_position
);
	localparam [6:0] ST_IDLE=0, ST_LANE=1, ST_SCAN=2, ST_CHECK=3,
		ST_LENGTH=4, ST_LENGTH_SQUARE=5, ST_SQRT=6, ST_SQRT_WAIT=7,
		ST_TARGET=8, ST_TARGET_WAIT=9, ST_SEEK=10, ST_DELTA=11,
		ST_XDIV=12, ST_XWAIT=13, ST_YDIV=14, ST_YWAIT=15,
		ST_UMAP=16, ST_UWAIT=17, ST_VMAP=18, ST_VWAIT=19,
		ST_POOL_INIT=20, ST_PIXEL_ADDR=21, ST_PIXEL_REQ=22, ST_PIXEL_WAIT=23,
		ST_PIXEL_MUL=24, ST_PIXEL_ADD=25, ST_SAMPLE_ADD=26,
		ST_CHANNEL_DONE=27, ST_CORE_START=28, ST_CORE_WAIT=29, ST_NEXT_LANE=30, ST_DONE=31,
		ST_SEEK_CHECK=32, ST_LENGTH_END=33;
	reg [6:0] state_q;
	reg [1:0] slot_q, corner_q;
	reg [4:0] row_q, count_q, index_q, segment_q, point_q, channel_q;
	reg [9:0] px [0:20] /* synthesis syn_ramstyle="lut_ram" */;
	reg [8:0] py [0:20] /* synthesis syn_ramstyle="lut_ram" */;
	reg [17:0] lengths [0:19] /* synthesis syn_ramstyle="lut_ram" */;
	reg [12:0] us [0:20] /* synthesis syn_ramstyle="lut_ram" */;
	reg [12:0] vs [0:20] /* synthesis syn_ramstyle="lut_ram" */;
	reg [21:0] total_q, target_q, fraction_q, segment_arc_q;
	reg [17:0] segment_length_q;
	reg [9:0] base_x_q;
	reg [8:0] base_y_q;
	reg [4:0] point_read_addr;
	wire [9:0] point_read_x = px[point_read_addr];
	wire [8:0] point_read_y = py[point_read_addr];
	wire [21:0] segment_end = segment_arc_q + {4'd0,segment_length_q};
	reg [9:0] dx_q;
	reg [8:0] dy_q;
	reg negative_x_q;
	reg [19:0] dx_square_q,dy_square_q;
	reg [17:0] xq_q;
	reg [16:0] yq_q;
	reg [13:0] channel_base_q;
	reg [25:0] sample_acc_q, pixel_product_q;
	reg [17:0] pixel_weight_q;
	reg [7:0] pixel_q;
	reg [31:0] cycles_q;
	wire core_done,core_error,core_ready;
	wire signed [31:0] core_solid,core_dashed,core_margin;
	wire [31:0] absolute_margin=core_margin[31] ? -core_margin : core_margin;
	wire [25:0] rounded_sample=(sample_acc_q+26'd128)>>8;
	reg math_start_q,math_sqrt_q;
	reg [39:0] math_num_q,math_den_q;
	wire math_done;
	wire [39:0] math_value;
	wire [6:0] scan_index = (slot_q==1 ? 7'd21 : 7'd42)+{2'd0,row_q};
	wire [9:0] scan_x = i_x_table[scan_index*10 +: 10];
	// 常数21除法只用于21项锚点高度映射，不产生动态除法器。
	`include "video/host_f14_old_coordinates.vh"
	wire [8:0] scan_y = old_anchor_y(row_q);
	wire [39:0] old_u_num = {22'd0,xq_q} * 40'd784;
	wire [39:0] old_v_num = {23'd0,yq_q} * 40'd2646;
	wire [4:0] ux = us[point_q][12:8];
	wire [4:0] vy = vs[point_q][12:8];
	wire [4:0] pixel_x = corner_q[0] && ux<27 ? ux+1'b1 : ux;
	wire [4:0] pixel_y = corner_q[1] && vy<20 ? vy+1'b1 : vy;
	wire [8:0] wx = corner_q[0] ? {1'b0,us[point_q][7:0]} : 9'd256-{1'b0,us[point_q][7:0]};
	wire [8:0] wy = corner_q[1] ? {1'b0,vs[point_q][7:0]} : 9'd256-{1'b0,vs[point_q][7:0]};
	assign o_busy=state_q!=ST_IDLE;
	assign o_cycles=cycles_q;
	assign o_debug_state=state_q;
	assign o_debug_position={3'd0,count_q,index_q,segment_q,row_q,point_q,slot_q,corner_q};
	assign o_slot=slot_q;
	assign o_sample_valid=state_q==ST_SAMPLE_ADD;
	assign o_sample_addr=({4'd0,channel_q}<<4)+({4'd0,channel_q}<<2)+{4'd0,channel_q}+{4'd0,point_q};
	assign o_sample_data=rounded_sample[15:0];
	assign o_logit_valid=state_q==ST_CORE_WAIT && core_done && !core_error;
	assign o_logit_solid=core_solid;
	assign o_logit_dashed=core_dashed;
	host_f14_tiny_serial u_tiny (
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_sample_valid(o_sample_valid),
		.i_sample_addr(o_sample_addr),.i_sample_data(o_sample_data),.o_sample_ready(core_ready),
		.i_start(state_q==ST_CORE_START),.o_busy(),.o_done(core_done),.o_error(core_error),
		.o_logit_solid(core_solid),.o_logit_dashed(core_dashed),.o_margin(core_margin),.o_cycles()
	);

	host_f14_uint_math u_math (
		.i_clk(i_clk),.i_rst_n(i_rst_n),.i_start(math_start_q),.i_sqrt(math_sqrt_q),
		.i_numerator(math_num_q),.i_denominator(math_den_q),.o_done(math_done),.o_result(math_value),.o_debug_state(o_debug_math)
	);
	// 端点读地址按状态复用；两端分别用两拍锁存，避免四路动态RAM选择器。
	always @(*) begin
		point_read_addr = 5'd0;
		case (state_q)
			ST_LENGTH: begin point_read_addr = index_q - 5'd1; end
			ST_LENGTH_END: begin point_read_addr = index_q; end
			ST_SEEK: begin point_read_addr = segment_q; end
			ST_DELTA: begin point_read_addr = segment_q + 5'd1; end
			default: begin point_read_addr = 5'd0; end
		endcase
	end
	// 写口均无复位；控制复位屏蔽写使能，数据有效性由当前事务计数保证。
	always @(posedge i_clk) begin
		if (i_rst_n && state_q == ST_SCAN && i_valid_table[scan_index] && scan_x < 640 && scan_y < C_CUTOFF_Y) begin
			px[count_q] <= scan_x;
			py[count_q] <= scan_y;
		end
	end
	always @(posedge i_clk) begin
		if (i_rst_n && state_q == ST_SQRT_WAIT && math_done) begin
			lengths[index_q - 5'd1] <= math_value[17:0];
		end
	end
	always @(posedge i_clk) begin
		if (i_rst_n && state_q == ST_UWAIT && math_done) begin
			us[point_q] <= math_value > 6912 ? 13'd6912 : math_value[12:0];
		end
	end
	always @(posedge i_clk) begin
		if (i_rst_n && state_q == ST_VWAIT && math_done) begin
			vs[point_q] <= math_value > 5120 ? 13'd5120 : math_value[12:0];
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			state_q<=ST_IDLE; o_done<=0; o_types<=0; o_feature_req<=0; o_feature_addr<=0;
			math_start_q<=0; math_sqrt_q<=0; math_num_q<=0; math_den_q<=1;
			slot_q<=1; o_error<=0; o_supported<=0; o_high<=0; o_medium<=0; cycles_q<=0;
			corner_q<=0; row_q<=0; count_q<=0; index_q<=0; segment_q<=0; point_q<=0; channel_q<=0;
			total_q<=0; target_q<=0; fraction_q<=0; segment_arc_q<=0; segment_length_q<=0; base_x_q<=0; base_y_q<=0; dx_q<=0; dy_q<=0; negative_x_q<=0;
			dx_square_q<=0; dy_square_q<=0; xq_q<=0; yq_q<=0; channel_base_q<=0;
			sample_acc_q<=0; pixel_product_q<=0; pixel_weight_q<=0; pixel_q<=0;

		end else begin
			o_done<=0; math_start_q<=0; o_feature_req<=0;
			if(state_q!=ST_IDLE) cycles_q<=cycles_q+1'b1;
			if(o_sample_valid && !core_ready) o_error<=1;
			case (state_q)
				ST_IDLE: begin
					if (i_start) begin
						o_types<={i_geometry_types[7:6],4'b0000,i_geometry_types[1:0]};
						slot_q<=1; o_error<=i_error; o_supported<=0; o_high<=0; o_medium<=0; cycles_q<=0; state_q<=i_error ? ST_DONE : ST_LANE;
					end
				end
				ST_LANE: begin count_q<=0; row_q<=0; state_q<=ST_SCAN; end
				ST_SCAN: begin
					if (i_valid_table[scan_index] && scan_x<640 && scan_y<C_CUTOFF_Y) begin
						count_q<=count_q+1'b1;
					end
					if (row_q==20) begin state_q<=ST_CHECK; end
					else begin row_q<=row_q+1'b1; end
				end
				ST_CHECK: begin
					if (count_q<3) begin state_q<=ST_NEXT_LANE; end
					else begin o_supported[slot_q-1'b1]<=1; index_q<=1; total_q<=0; state_q<=ST_LENGTH; end
				end
				ST_LENGTH: begin
					base_x_q<=point_read_x; base_y_q<=point_read_y; state_q<=ST_LENGTH_END;
				end
				ST_LENGTH_END: begin
					dx_q<=point_read_x>=base_x_q ? point_read_x-base_x_q : base_x_q-point_read_x;
					dy_q<=point_read_y-base_y_q; state_q<=ST_LENGTH_SQUARE;
				end
				ST_LENGTH_SQUARE: begin dx_square_q<=dx_q*dx_q; dy_square_q<=dy_q*dy_q; state_q<=ST_SQRT; end
				ST_SQRT: begin
					math_num_q<=({20'd0,dx_square_q}+{20'd0,dy_square_q})<<16;
					math_sqrt_q<=1; math_start_q<=1; state_q<=ST_SQRT_WAIT;
				end
				ST_SQRT_WAIT: begin
					if (math_done) begin
						total_q<=total_q+math_value[21:0];
						if (index_q==count_q-1) begin
							point_q<=0; segment_q<=0; segment_arc_q<=0; state_q<=ST_TARGET;
						end else begin index_q<=index_q+1'b1; state_q<=ST_LENGTH; end
					end
				end
				ST_TARGET: begin
					math_num_q<=({18'd0,total_q}*point_q)+40'd10; math_den_q<=40'd20;
					math_sqrt_q<=0; math_start_q<=1; state_q<=ST_TARGET_WAIT;
				end
				ST_TARGET_WAIT: begin if (math_done) begin target_q<=math_value[21:0]; state_q<=ST_SEEK; end end
				ST_SEEK: begin
					segment_length_q<=lengths[segment_q];
					base_x_q<=point_read_x; base_y_q<=point_read_y; state_q<=ST_SEEK_CHECK;
				end
				ST_SEEK_CHECK: begin
					if (segment_q<count_q-2 && segment_end<=target_q) begin
						segment_arc_q<=segment_end; segment_q<=segment_q+1'b1; state_q<=ST_SEEK;
					end else begin state_q<=ST_DELTA; end
				end
				ST_DELTA: begin
					dx_q<=point_read_x>=base_x_q ? point_read_x-base_x_q : base_x_q-point_read_x;
					negative_x_q<=point_read_x<base_x_q; dy_q<=point_read_y-base_y_q;
					fraction_q<=target_q-segment_arc_q; state_q<=ST_XDIV;
				end
				ST_XDIV: begin
					math_num_q<=(({30'd0,dx_q}*fraction_q)<<8)+{23'd0,segment_length_q[17:1]};
					math_den_q<={22'd0,segment_length_q}; math_start_q<=1; state_q<=ST_XWAIT;
				end
				ST_XWAIT: begin
					if (math_done) begin
						xq_q<=negative_x_q ? {base_x_q,8'd0}-math_value[17:0] : {base_x_q,8'd0}+math_value[17:0];
						state_q<=ST_YDIV;
					end
				end
				ST_YDIV: begin
					math_num_q<=(({31'd0,dy_q}*fraction_q)<<8)+{23'd0,segment_length_q[17:1]};
					math_start_q<=1; state_q<=ST_YWAIT;
				end
				ST_YWAIT: begin if (math_done) begin yq_q<={base_y_q,8'd0}+math_value[16:0]; state_q<=ST_UMAP; end end
				ST_UMAP: begin
					// 保留旧 grid_sample 的 mixed-align-corners 公式；边界先钳位。
					math_num_q<=old_u_num<=40'd4505600 ? 40'd0 : old_u_num-40'd4505600+40'd8640;
					math_den_q<=17280; math_start_q<=1; state_q<=ST_UWAIT;
				end
				ST_UWAIT: begin
					if (math_done) begin
						state_q<=ST_VMAP;
					end
				end
				ST_VMAP: begin
					math_num_q<=old_v_num<=40'd58959104 ? 40'd0 : old_v_num-40'd58959104+40'd13820;
					math_den_q<=27640; math_start_q<=1; state_q<=ST_VWAIT;
				end
				ST_VWAIT: begin
					if (math_done) begin
						if (point_q==20) begin state_q<=ST_POOL_INIT; end
						else begin point_q<=point_q+1'b1; state_q<=ST_TARGET; end
					end
				end
				ST_POOL_INIT: begin
					channel_q<=0; channel_base_q<=0; point_q<=0; corner_q<=0; sample_acc_q<=0;
					state_q<=ST_PIXEL_ADDR;
				end
				ST_PIXEL_ADDR: begin
					o_feature_addr<=channel_base_q+({9'd0,pixel_y}*14'd28)+{9'd0,pixel_x};
					pixel_weight_q<=wx*wy; state_q<=ST_PIXEL_REQ;
				end
				ST_PIXEL_REQ: begin o_feature_req<=1; state_q<=ST_PIXEL_WAIT; end
				ST_PIXEL_WAIT: begin if (i_feature_valid) begin pixel_q<=i_feature_data; state_q<=ST_PIXEL_MUL; end end
				ST_PIXEL_MUL: begin pixel_product_q<=pixel_q*pixel_weight_q; state_q<=ST_PIXEL_ADD; end
				ST_PIXEL_ADD: begin
					sample_acc_q<=sample_acc_q+pixel_product_q;
					if (corner_q==3) begin state_q<=ST_SAMPLE_ADD; end
					else begin corner_q<=corner_q+1'b1; state_q<=ST_PIXEL_ADDR; end
				end
				ST_SAMPLE_ADD: begin
					sample_acc_q<=0; corner_q<=0;
					if (point_q==20) begin state_q<=ST_CHANNEL_DONE; end
					else begin point_q<=point_q+1'b1; state_q<=ST_PIXEL_ADDR; end
				end
				ST_CHANNEL_DONE: begin
					if(channel_q==23) state_q<=ST_CORE_START;
					else begin channel_q<=channel_q+1'b1;channel_base_q<=channel_base_q+14'd588;
						point_q<=0;state_q<=ST_PIXEL_ADDR;end
				end
				ST_CORE_START: state_q<=ST_CORE_WAIT;
				ST_CORE_WAIT: if(core_done) begin
					if(core_error) o_error<=1;
					else begin
						o_types[slot_q*2 +: 2]<=core_margin>0 ? 2'd2 : 2'd1;
						o_high[slot_q-1'b1]<=absolute_margin>=3471;
						o_medium[slot_q-1'b1]<=absolute_margin>=822;
					end
					state_q<=ST_NEXT_LANE;
				end
				ST_NEXT_LANE: begin
					if (slot_q==2) begin state_q<=ST_DONE; end
					else begin slot_q<=2; state_q<=ST_LANE; end
				end
				ST_DONE: begin o_done<=1; state_q<=ST_IDLE; end
				default: begin state_q<=ST_IDLE; o_types<=0; end
			endcase
		end
	end
endmodule
`default_nettype wire
