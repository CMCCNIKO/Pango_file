//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 从224×168 RGB565输入按3×3/stride2/pad1采样首层8×8输出tile，展开RGB并按紧凑CHW tap顺序打包C8/P2加载字。
// Module Name: f14_stem_rgb565_gather
// File Name: model/rtl/f14_stem_rgb565_gather.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 从224×168 RGB565输入按3×3/stride2/pad1采样首层8×8输出tile，展开RGB并按紧凑CHW tap顺序打包C8/P2加载字。
// 每次仅保留一对输出像素的4个C8字共512位，越界填充，不缓存整图/权重，不执行卷积乘加。
//
// Dependencies: npu_rgb565_expand；外部RGB565帧读口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 每次只发一个读请求并等待valid；复位需与外部读流水共同清在途响应，输出背压保持加载字。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 compact RGB565 stem gather
// Module Name: f14_stem_rgb565_gather
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: Isolated SOP compact CHW tap scatter, no convolution arithmetic.
// Dependencies: Original SOP npu_rgb565_expand, unchanged.
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

// 固定输入224x168 RGB565；输出3x3/s2/p1的8x8 tile，C8/P2地址group*41+pair。
// 复用原SOP compact gather的18像素/4字结构和CHW tap顺序。
// 每次只发一个读请求，等待valid后再推进；读端必须在复位时清空在途响应。
// 512位暂存仅保存一对输出像素的4个C8字，不保存整图/权重，不增加MAC。
module f14_stem_rgb565_gather (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_job_valid,
	output wire o_job_ready,
	input wire [6:0] i_tile_y,
	input wire [6:0] i_tile_x,
	input wire [3:0] i_tile_valid_h,
	input wire [3:0] i_tile_valid_w,
	output wire o_job_done,
	output wire o_job_error,
	output wire o_busy,
	output wire o_frame_rd_en,
	output wire [15:0] o_frame_rd_addr,
	input wire i_frame_rd_valid,
	input wire [15:0] i_frame_rd_data,
	output wire o_word_valid,
	input wire i_word_ready,
	output wire [9:0] o_word_addr,
	output wire [127:0] o_word_data
);
	localparam [3:0] ST_IDLE = 4'd0;
	localparam [3:0] ST_CLEAR = 4'd1;
	localparam [3:0] ST_ADDR = 4'd2;
	localparam [3:0] ST_READ = 4'd3;
	localparam [3:0] ST_WAIT = 4'd4;
	localparam [3:0] ST_ADVANCE = 4'd5;
	localparam [3:0] ST_SEND = 4'd6;
	localparam [3:0] ST_DONE = 4'd7;
	reg [3:0] state_q;
	reg [3:0] state_d;
	reg [6:0] tile_y_q;
	reg [6:0] tile_x_q;
	reg [3:0] valid_h_q;
	reg [3:0] valid_w_q;
	reg [4:0] pair_q;
	reg pixel_q;
	reg [3:0] tap_q;
	reg [1:0] ky_q;
	reg [1:0] kx_q;
	reg [1:0] group_q;
	reg [15:0] frame_addr_q;
	reg error_q;
	reg active_q;
	wire [511:0] words;
	wire [23:0] rgb888;
	wire job_fire;
	wire job_legal;
	wire spatial_legal;
	wire commit_pixel;
	wire advance_pixel = ((state_q == ST_ADDR) && !spatial_legal)
		|| ((state_q == ST_WAIT) && i_frame_rd_valid);
	wire [3:0] local_y;
	wire [3:0] local_x;
	wire signed [9:0] source_y;
	wire signed [9:0] source_x;
	wire [17:0] source_y_ext;
	wire [17:0] linear_addr;
	genvar byte_index;

	assign o_job_ready = active_q && (state_q == ST_IDLE);
	assign job_fire = i_job_valid && o_job_ready;
	assign job_legal = (i_tile_valid_h >= 4'd1) && (i_tile_valid_h <= 4'd8)
		&& (i_tile_valid_w >= 4'd1) && (i_tile_valid_w <= 4'd8)
		&& ({1'b0, i_tile_y} + {4'd0, i_tile_valid_h} <= 8'd84)
		&& ({1'b0, i_tile_x} + {4'd0, i_tile_valid_w} <= 8'd112);
	assign o_job_done = active_q && (state_q == ST_DONE);
	assign o_job_error = error_q;
	assign o_busy = active_q && (state_q != ST_IDLE);
	assign local_y = {1'b0, pair_q[4:2]};
	assign local_x = {pair_q[1:0], pixel_q};
	assign source_y = $signed({2'b00, tile_y_q, 1'b0})
		+ $signed({5'd0, local_y, 1'b0}) + $signed({8'd0, ky_q}) - 10'sd1;
	assign source_x = $signed({2'b00, tile_x_q, 1'b0})
		+ $signed({5'd0, local_x, 1'b0}) + $signed({8'd0, kx_q}) - 10'sd1;
	assign spatial_legal = (local_y < valid_h_q) && (local_x < valid_w_q)
		&& (source_y >= 10'sd0) && (source_y < 10'sd168)
		&& (source_x >= 10'sd0) && (source_x < 10'sd224);
	// 原SOP的224倍移位相减，不产生地址乘法器；先锁存地址再发请求。
	assign source_y_ext = {8'd0, source_y};
	assign linear_addr = (source_y_ext << 8) - (source_y_ext << 5) + {8'd0, source_x};
	assign o_frame_rd_en = active_q && (state_q == ST_READ);
	assign o_frame_rd_addr = frame_addr_q;
	assign commit_pixel = active_q && (state_q == ST_WAIT) && i_frame_rd_valid;
	assign o_word_valid = active_q && (state_q == ST_SEND);
	assign o_word_addr = {8'd0, group_q} * 10'd41 + {5'd0, pair_q};
	assign o_word_data = words[group_q * 128 +: 128];

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			state_q <= ST_IDLE;
			active_q <= 1'b0;
		end else begin
			state_q <= state_d;
			active_q <= 1'b1;
		end
	end

	always@(*)begin
		state_d = state_q;
		case(state_q)
			ST_IDLE:begin
				if(job_fire)begin
					if(job_legal)begin state_d = ST_CLEAR; end
					else begin state_d = ST_DONE; end
				end
			end
			ST_CLEAR:begin state_d = ST_ADDR; end
			ST_ADDR:begin
				if(spatial_legal)begin state_d = ST_READ; end
				else begin state_d = (pixel_q && tap_q == 4'd8) ? ST_SEND : ST_ADDR; end
			end
			ST_READ:begin state_d = ST_WAIT; end
			ST_WAIT:begin
				if(i_frame_rd_valid)begin state_d = (pixel_q && tap_q == 4'd8) ? ST_SEND : ST_ADDR; end
			end
			ST_ADVANCE:begin
				if(pixel_q && (tap_q == 4'd8))begin state_d = ST_SEND; end
				else begin state_d = ST_ADDR; end
			end
			ST_SEND:begin
				if(i_word_ready && (group_q == 2'd3))begin
					if(pair_q == 5'd31)begin state_d = ST_DONE; end
					else begin state_d = ST_CLEAR; end
				end
			end
			ST_DONE:begin state_d = ST_IDLE; end
			default:begin state_d = ST_IDLE; end
		endcase
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			tile_y_q <= 7'd0;
			tile_x_q <= 7'd0;
			valid_h_q <= 4'd0;
			valid_w_q <= 4'd0;
			pair_q <= 5'd0;
			pixel_q <= 1'b0;
			tap_q <= 4'd0;
			ky_q <= 2'd0;
			kx_q <= 2'd0;
			group_q <= 2'd0;
			frame_addr_q <= 16'd0;
			error_q <= 1'b0;
		end else begin
			if(job_fire)begin
				tile_y_q <= i_tile_y;
				tile_x_q <= i_tile_x;
				valid_h_q <= i_tile_valid_h;
				valid_w_q <= i_tile_valid_w;
				pair_q <= 5'd0;
				error_q <= !job_legal;
			end
			if(state_q == ST_CLEAR)begin
				pixel_q <= 1'b0;
				tap_q <= 4'd0;
				ky_q <= 2'd0;
				kx_q <= 2'd0;
				group_q <= 2'd0;
			end
			if(state_q == ST_ADDR)begin frame_addr_q <= linear_addr[15:0]; end
			// 返回的数据使用旧tap写入；同拍推进下一tap，省去独立ADVANCE拍。
			if(advance_pixel)begin
				if(tap_q == 4'd8)begin
					pixel_q <= 1'b1;
					tap_q <= 4'd0;
					ky_q <= 2'd0;
					kx_q <= 2'd0;
				end else begin
					tap_q <= tap_q + 4'd1;
					if(kx_q == 2'd2)begin
						kx_q <= 2'd0;
						ky_q <= ky_q + 2'd1;
					end else begin kx_q <= kx_q + 2'd1; end
				end
			end
			if((state_q == ST_SEND) && i_word_ready)begin
				group_q <= group_q + 2'd1;
				if(group_q == 2'd3)begin pair_q <= pair_q + 5'd1; end
			end
		end
	end

	// 静态逐字节scatter避免512位可变写端口；CHW term=c*9+tap。
	generate
		for(byte_index = 0; byte_index < 64; byte_index = byte_index + 1)begin:gen_scatter
			localparam integer TERM = (byte_index / 16) * 8 + byte_index % 8;
			localparam integer PIXEL = (byte_index % 16) / 8;
			if(TERM < 27)begin:gen_term
				reg [7:0] data_q;
				wire [7:0] channel_data;
				assign channel_data = rgb888[23 - (TERM / 9) * 8 -: 8];
				always@(posedge i_clk)begin
					// 原输入归一化对应 affine U8 zero=128；图外 padding 为实数零。
					if(state_q == ST_CLEAR)begin data_q <= 8'd128; end
					else if(commit_pixel && (pixel_q == PIXEL) && (tap_q == TERM % 9))begin
						data_q <= channel_data;
					end
				end
				assign words[byte_index * 8 +: 8] = data_q;
			end else begin:gen_padding
				assign words[byte_index * 8 +: 8] = 8'd0;
			end
		end
	endgenerate

	npu_rgb565_expand u_rgb565_expand (
		.rgb565(i_frame_rd_data),
		.rgb888(rgb888)
	);
endmodule
`default_nettype wire
