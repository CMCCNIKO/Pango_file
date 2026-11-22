//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 按 Stem/bottleneck/adapter 类型实时枚举 tile 坐标、有效宽高、边界偏移和输入原点，不存储整张描述符表。
// Module Name: f14_backbone_tile_geometry
// File Name: model/rtl/f14_backbone_tile_geometry.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 按 Stem/bottleneck/adapter 类型实时枚举 tile 坐标、有效宽高、边界偏移和输入原点，不存储整张描述符表。
// 复用 stride1 步长7、stride2 步长4、pointwise 步长8 的分块规则，处理末块裁边和负 halo 原点。
//
// Dependencies: F14 15 个骨干/adapter 段的几何配置
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// job 在 valid&&ready 时锁存；tile 背压时载荷保持，调用方按现有所有权契约确认 tile_ready。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 on-demand backbone tile geometry
// Module Name: f14_backbone_tile_geometry
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: SOP7/4/8-step geometry without storing the full descriptor table.
// Dependencies: Geometry from npu_s8_feature_map_scheduler; no arithmetic array.
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

// kind:0 stem,1 bottleneck,2 adapter；只支持15个backbone/adapter段。
// job配置在valid&&ready锁存。tile载荷在背压下保持，tile握手即允许推进。
// 调用方仅在该tile计算和写回完成后给tile_ready，避免覆盖正在用的坐标。
// 复用SOP stride1步长7/stride2步长4/pointwise步长8；无全图寄存器表。
module f14_backbone_tile_geometry (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_job_valid,
	output wire o_job_ready,
	input wire [1:0] i_kind,
	input wire i_stride2,
	input wire [7:0] i_input_h,
	input wire [7:0] i_input_w,
	input wire [6:0] i_output_h,
	input wire [6:0] i_output_w,
	output wire o_tile_valid,
	input wire i_tile_ready,
	output wire [6:0] o_tile_y,
	output wire [6:0] o_tile_x,
	output wire [3:0] o_valid_h,
	output wire [3:0] o_valid_w,
	output wire o_shift_y,
	output wire o_shift_x,
	output wire o_far_y,
	output wire o_far_x,
	output wire signed [9:0] o_input_y,
	output wire signed [9:0] o_input_x,
	output wire o_last,
	output wire o_job_done,
	output wire o_job_error,
	output wire o_busy
);
	localparam [1:0] ST_IDLE = 2'd0;
	localparam [1:0] ST_GEOMETRY = 2'd1;
	localparam [1:0] ST_EMIT = 2'd2;
	localparam [1:0] ST_DONE = 2'd3;
	reg [1:0] state_q;
	reg [1:0] state_d;
	reg active_q;
	reg [1:0] kind_q;
	reg stride2_q;
	reg [7:0] input_h_q;
	reg [7:0] input_w_q;
	reg [6:0] output_h_q;
	reg [6:0] output_w_q;
	reg [6:0] tile_y_q;
	reg [6:0] tile_x_q;
	reg [3:0] valid_h_q;
	reg [3:0] valid_w_q;
	reg shift_y_q;
	reg shift_x_q;
	reg far_y_q;
	reg far_x_q;
	reg signed [9:0] origin_y_q;
	reg signed [9:0] origin_x_q;
	reg last_y_q;
	reg last_x_q;
	reg error_q;
	wire legal;
	wire job_fire;
	wire [3:0] step;
	wire [7:0] end_y;
	wire [7:0] end_x;
	wire [3:0] valid_h;
	wire [3:0] valid_w;
	wire shift_y;
	wire shift_x;
	wire signed [9:0] source_y;
	wire signed [9:0] source_x;
	wire downsample;
	wire [7:0] valid_end_y;
	wire [7:0] valid_end_x;
	assign downsample = (i_kind == 2'd0) || ((i_kind == 2'd1) && i_stride2);
	assign legal = (i_kind <= 2'd2) && ((i_kind == 2'd1) || !i_stride2)
		&& (i_input_h != 0) && (i_input_w != 0)
		&& (i_output_h != 0) && (i_output_h <= 7'd84)
		&& (i_output_w != 0) && (i_output_w <= 7'd112)
		&& ({1'b0,i_output_h} == (downsample ? (({1'b0,i_input_h}+9'd1) >> 1) : {1'b0,i_input_h}))
		&& ({1'b0,i_output_w} == (downsample ? (({1'b0,i_input_w}+9'd1) >> 1) : {1'b0,i_input_w}));
	assign job_fire = i_job_valid && o_job_ready;
	assign o_job_ready = active_q && (state_q == ST_IDLE);
	assign o_tile_valid = active_q && (state_q == ST_EMIT);
	assign o_job_done = active_q && (state_q == ST_DONE);
	assign o_job_error = error_q;
	assign o_busy = active_q && (state_q != ST_IDLE);
	assign o_tile_y = tile_y_q;
	assign o_tile_x = tile_x_q;
	assign o_valid_h = valid_h_q;
	assign o_valid_w = valid_w_q;
	assign o_shift_y = shift_y_q;
	assign o_shift_x = shift_x_q;
	assign o_far_y = far_y_q;
	assign o_far_x = far_x_q;
	assign o_input_y = origin_y_q;
	assign o_input_x = origin_x_q;
	assign o_last = last_y_q && last_x_q;
	assign step = (kind_q != 2'd1) ? 4'd8 : (stride2_q ? 4'd4 : 4'd7);
	assign end_y = {1'b0,tile_y_q} + {4'd0,step};
	assign end_x = {1'b0,tile_x_q} + {4'd0,step};
	assign valid_h = (end_y >= {1'b0,output_h_q}) ? output_h_q-tile_y_q : step;
	assign valid_w = (end_x >= {1'b0,output_w_q}) ? output_w_q-tile_x_q : step;
	assign valid_end_y = {1'b0,tile_y_q} + {4'd0,valid_h};
	assign valid_end_x = {1'b0,tile_x_q} + {4'd0,valid_w};
	assign shift_y = (kind_q == 2'd1) && (tile_y_q != 0);
	assign shift_x = (kind_q == 2'd1) && (tile_x_q != 0);
	assign source_y = stride2_q ? ($signed({2'd0,tile_y_q,1'b0}) - 10'sd1)
		: ($signed({3'd0,tile_y_q}) - $signed({9'd0,shift_y}));
	assign source_x = stride2_q ? ($signed({2'd0,tile_x_q,1'b0}) - 10'sd1)
		: ($signed({3'd0,tile_x_q}) - $signed({9'd0,shift_x}));

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin state_q <= ST_IDLE; active_q <= 1'b0; end
		else begin state_q <= state_d; active_q <= 1'b1; end
	end
	always@(*)begin
		state_d = state_q;
		case(state_q)
			ST_IDLE:begin if(job_fire)begin state_d = legal ? ST_GEOMETRY : ST_DONE; end end
			ST_GEOMETRY:begin state_d = ST_EMIT; end
			ST_EMIT:begin if(i_tile_ready)begin state_d = o_last ? ST_DONE : ST_GEOMETRY; end end
			ST_DONE:begin state_d = ST_IDLE; end
			default:begin state_d = ST_IDLE; end
		endcase
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			kind_q <= 2'd0; stride2_q <= 1'b0;
			input_h_q <= 8'd0; input_w_q <= 8'd0; output_h_q <= 7'd0; output_w_q <= 7'd0;
			tile_y_q <= 7'd0; tile_x_q <= 7'd0; valid_h_q <= 4'd0; valid_w_q <= 4'd0;
			shift_y_q <= 1'b0; shift_x_q <= 1'b0; far_y_q <= 1'b0; far_x_q <= 1'b0;
			origin_y_q <= 10'sd0; origin_x_q <= 10'sd0; last_y_q <= 1'b0; last_x_q <= 1'b0; error_q <= 1'b0;
		end else begin
			if(job_fire)begin
				kind_q <= i_kind; stride2_q <= i_stride2;
				input_h_q <= i_input_h; input_w_q <= i_input_w; output_h_q <= i_output_h; output_w_q <= i_output_w;
				tile_y_q <= 7'd0; tile_x_q <= 7'd0; error_q <= !legal;
			end
			if(state_q == ST_GEOMETRY)begin
				valid_h_q <= valid_h; valid_w_q <= valid_w;
				shift_y_q <= shift_y; shift_x_q <= shift_x;
				origin_y_q <= source_y; origin_x_q <= source_x;
				last_y_q <= (end_y >= {1'b0,output_h_q}); last_x_q <= (end_x >= {1'b0,output_w_q});
				// far标志描述有效输出的卷积足迹，不是固定9x9缓存是否越界。
				// 偶数尺寸末尾的部分stride2 tile仍需保留最后一行真实输入。
				far_y_q <= (kind_q == 2'd1) && (stride2_q
					? ({valid_end_y,1'b0} > {1'b0,input_h_q}) : (valid_end_y >= input_h_q));
				far_x_q <= (kind_q == 2'd1) && (stride2_q
					? ({valid_end_x,1'b0} > {1'b0,input_w_q}) : (valid_end_x >= input_w_q));
			end
			if((state_q == ST_EMIT) && i_tile_ready)begin
				if(last_x_q)begin tile_x_q <= 7'd0; tile_y_q <= tile_y_q + {3'd0,valid_h_q}; end
				else begin tile_x_q <= tile_x_q + {3'd0,valid_w_q}; end
			end
		end
	end
endmodule
`default_nettype wire
