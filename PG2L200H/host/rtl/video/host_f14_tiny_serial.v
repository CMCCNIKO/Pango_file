//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-26
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 单条车道的 TinySequence 整数串行分类核心：依次执行通道压缩、深度卷积、三段求和池化与全连接，输出 signed Q12 实线/虚线 logit、差值和拍数。
// Module Name: host_f14_tiny_serial
// File Name: host/rtl/video/host_f14_tiny_serial.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 单条车道的 TinySequence 整数串行分类核心：依次执行通道压缩、深度卷积、三段求和池化与全连接，输出 signed Q12 实线/虚线 logit、差值和拍数。
// 空闲期按 0..503 顺序装载 24×21 的 Q8 样本，start 消费完整事务；忙时不接收样本，装载计数及错误状态保护未初始化 RAM。
//
// Dependencies: host_f14_tiny_params；i_clk/i_rst_n；共享一处 24×16 乘法资源，内部样本/中间 RAM 数据不复位。
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
// Module Name: host_f14_tiny_serial
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// 单条线24x21 Q8取样，串行reduce/DW/三段求和/FC；仅一处24x16乘法。
// 空闲时按0..503顺序装载，start消费完整事务；忙时不接收新样本。
// RAM数据不复位；装载计数保护未初始化数据。logit为signed Q12。
`timescale 1ns/1ps
`default_nettype none
module host_f14_tiny_serial (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_sample_valid,
	input wire [8:0] i_sample_addr,
	input wire [15:0] i_sample_data,
	output wire o_sample_ready,
	input wire i_start,
	output wire o_busy,
	output wire o_done,
	output wire o_error,
	output wire signed [31:0] o_logit_solid,
	output wire signed [31:0] o_logit_dashed,
	output wire signed [31:0] o_margin,
	output wire [31:0] o_cycles
);
	localparam [3:0] ST_IDLE = 4'd0, ST_BIAS = 4'd1, ST_BIAS_WAIT = 4'd2,
		ST_READ = 4'd3, ST_OPERAND = 4'd4, ST_MUL = 4'd5,
		ST_ACC = 4'd6, ST_STORE = 4'd7, ST_DONE = 4'd8;
	reg [3:0] state_q, state_next;
	reg [1:0] phase_q;
	reg [3:0] channel_q;
	reg [4:0] point_q, term_q;
	reg [1:0] segment_q;
	reg [8:0] loaded_q, sample_addr_q;
	reg [7:0] z_base_q, z_addr_q;
	reg [8:0] weight_base_q;
	reg signed [23:0] operand_a_q;
	reg signed [15:0] operand_b_q;
	reg signed [39:0] product_q;
	reg signed [47:0] accumulator_q;
	reg [18:0] pool_sum_q;
	reg error_q, load_error_q, done_q;
	reg signed [31:0] solid_q, dashed_q;
	reg [31:0] cycles_q;
	reg signed [15:0] samples [0:503] /* synthesis syn_ramstyle="lut_ram" */;
	reg [15:0] reduced [0:167] /* synthesis syn_ramstyle="lut_ram" */;
	reg [18:0] pools [0:23] /* synthesis syn_ramstyle="lut_ram" */;
	reg signed [15:0] sample_data_q;
	reg [15:0] z_data_q;
	reg [18:0] pool_data_q;
	wire [8:0] parameter_addr;
	wire signed [47:0] parameter_data;
	wire last_term;
	wire padding;
	wire signed [47:0] rounded;
	wire [15:0] activation;
	wire [19:0] pool_total;
	wire [4:0] pool_addr;
	assign o_sample_ready = state_q == ST_IDLE && !i_start && loaded_q < 9'd504;
	assign o_busy = state_q != ST_IDLE;
	assign o_done = done_q;
	assign o_error = error_q;
	assign o_logit_solid = solid_q;
	assign o_logit_dashed = dashed_q;
	assign o_margin = dashed_q - solid_q;
	assign o_cycles = cycles_q;
	assign parameter_addr = state_q == ST_BIAS ?
		(phase_q == 0 ? 9'd264 + {5'd0, channel_q} :
		phase_q == 1 ? 9'd272 + {5'd0, channel_q} : 9'd280 + {5'd0, channel_q}) :
		weight_base_q + {4'd0, term_q};
	assign last_term = phase_q == 1 ? term_q == 5'd2 : term_q == 5'd23;
	assign padding = phase_q == 1 && ((point_q == 0 && term_q == 0) ||
		(point_q == 20 && term_q == 2));
	assign rounded = phase_q == 0 ? (accumulator_q + 48'sd65536) >>> 17 :
		phase_q == 1 ? (accumulator_q + 48'sd16384) >>> 15 :
		(accumulator_q + 48'sd32768) >>> 16;
	assign activation = rounded < 0 ? 16'd0 : rounded > 65535 ? 16'hffff : rounded[15:0];
	assign pool_total = {1'b0, pool_sum_q} + {4'd0, activation};
	assign pool_addr = {segment_q, 3'd0} + {1'b0, channel_q};
	host_f14_tiny_params u_params (
		.i_clk(i_clk), .i_en(state_q == ST_BIAS || state_q == ST_READ),
		.i_addr(parameter_addr), .o_data(parameter_data)
	);
	// 三个小RAM各只有一个读地址；写入和计算阶段互斥。
	always @(posedge i_clk) begin
		if (i_sample_valid && o_sample_ready && i_sample_addr == loaded_q) begin
			samples[i_sample_addr] <= i_sample_data ^ 16'h8000;
		end
		if (state_q == ST_READ && phase_q == 0) begin
			sample_data_q <= samples[sample_addr_q];
		end
		if (state_q == ST_READ && phase_q == 1) begin
			z_data_q <= reduced[z_addr_q];
		end
		if (state_q == ST_STORE && phase_q == 0) begin
			reduced[z_base_q + {3'd0, point_q}] <= activation;
		end
		if (state_q == ST_STORE && phase_q == 1 &&
			(point_q == 6 || point_q == 13 || point_q == 20)) begin
			pools[pool_addr] <= pool_total[18:0];
		end
		if (state_q == ST_READ && phase_q == 2) begin
			pool_data_q <= pools[term_q];
		end
	end
	// 共享乘法只有这一处；操作数和乘积分别寄存。
	always @(posedge i_clk) begin
		if (state_q == ST_OPERAND) begin
			operand_a_q <= phase_q == 0 ? {{8{sample_data_q[15]}}, sample_data_q} :
				phase_q == 1 ? (padding ? 24'sd0 : {8'd0, z_data_q}) :
				{5'd0, pool_data_q};
			operand_b_q <= parameter_data[15:0];
		end
		if (state_q == ST_MUL) begin
			product_q <= operand_a_q * operand_b_q;
		end
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			state_q <= ST_IDLE;
		end else begin
			state_q <= state_next;
		end
	end
	always @(*) begin
		state_next = state_q;
		case (state_q)
			ST_IDLE: begin
				if (i_start) begin
					state_next = loaded_q == 9'd504 && !load_error_q ? ST_BIAS : ST_DONE;
				end
			end
			ST_BIAS: state_next = ST_BIAS_WAIT;
			ST_BIAS_WAIT: state_next = ST_READ;
			ST_READ: state_next = ST_OPERAND;
			ST_OPERAND: state_next = ST_MUL;
			ST_MUL: state_next = ST_ACC;
			ST_ACC: state_next = last_term ? ST_STORE : ST_READ;
			ST_STORE: begin
				state_next = phase_q == 2 && channel_q == 1 ? ST_DONE : ST_BIAS;
			end
			ST_DONE: state_next = ST_IDLE;
			default: state_next = ST_IDLE;
		endcase
	end
	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			phase_q <= 0; channel_q <= 0; point_q <= 0; term_q <= 0;
			segment_q <= 0; loaded_q <= 0; sample_addr_q <= 0;
			z_base_q <= 0; z_addr_q <= 0; weight_base_q <= 0;
			accumulator_q <= 0; pool_sum_q <= 0;
			error_q <= 0; load_error_q <= 0; done_q <= 0;
			solid_q <= 0; dashed_q <= 0; cycles_q <= 0;
		end else begin
			done_q <= 1'b0;
			if (state_q != ST_IDLE && state_q != ST_DONE) begin
				cycles_q <= cycles_q + 1'b1;
			end
			case (state_q)
				ST_IDLE: begin
					if (i_sample_valid && o_sample_ready) begin
						if (i_sample_addr == loaded_q && i_sample_data <= 16'd65280) begin
							loaded_q <= loaded_q + 1'b1;
						end else begin
							load_error_q <= 1'b1;
						end
					end
					if (i_start) begin
						error_q <= loaded_q != 9'd504 || load_error_q;
						loaded_q <= 0; load_error_q <= 0;
						phase_q <= 0; channel_q <= 0; point_q <= 0;
						weight_base_q <= 0; z_base_q <= 0;
						cycles_q <= 0; solid_q <= 0; dashed_q <= 0;
					end
				end
				ST_BIAS: begin
					term_q <= 0;
					if (phase_q == 0) begin
						sample_addr_q <= {4'd0, point_q};
					end
					if (phase_q == 1) begin
						z_addr_q <= point_q == 0 ? z_base_q : z_base_q + {3'd0, point_q} - 1'b1;
					end
				end
				ST_BIAS_WAIT: accumulator_q <= parameter_data;
				ST_ACC: begin
					accumulator_q <= accumulator_q + {{8{product_q[39]}}, product_q};
					if (!last_term) begin
						term_q <= term_q + 1'b1;
						if (phase_q == 0) begin
							sample_addr_q <= sample_addr_q + 9'd21;
						end
						if (phase_q == 1) begin
							z_addr_q <= point_q == 20 && term_q == 1 ? z_base_q + 8'd20 :
								z_base_q + {3'd0, point_q} + {3'd0, term_q};
						end
					end
				end
				ST_STORE: begin
					if (phase_q != 2 && rounded > 65535) begin
						error_q <= 1'b1;
					end
					if (phase_q == 0) begin
						if (point_q != 20) begin
							point_q <= point_q + 1'b1;
						end else if (channel_q != 7) begin
							point_q <= 0; channel_q <= channel_q + 1'b1;
							z_base_q <= z_base_q + 8'd21; weight_base_q <= weight_base_q + 9'd24;
						end else begin
							phase_q <= 1; point_q <= 0; channel_q <= 0; segment_q <= 0;
							z_base_q <= 0; weight_base_q <= 9'd192; pool_sum_q <= 0;
						end
					end else if (phase_q == 1) begin
						pool_sum_q <= pool_total[18:0];
						if (point_q == 6 || point_q == 13 || point_q == 20) begin
							pool_sum_q <= 0; segment_q <= segment_q + 1'b1;
						end
						if (point_q != 20) begin
							point_q <= point_q + 1'b1;
						end else if (channel_q != 7) begin
							point_q <= 0; channel_q <= channel_q + 1'b1; segment_q <= 0;
							z_base_q <= z_base_q + 8'd21; weight_base_q <= weight_base_q + 9'd3;
						end else begin
							phase_q <= 2; channel_q <= 0; weight_base_q <= 9'd216;
						end
					end else begin
						if (rounded > 48'sd2147483647 || rounded < -48'sd2147483648) begin
							error_q <= 1'b1;
						end
						if (channel_q == 0) begin
							solid_q <= rounded[31:0]; channel_q <= 1; weight_base_q <= 9'd240;
						end else begin
							dashed_q <= rounded[31:0];
						end
					end
				end
				ST_DONE: done_q <= 1'b1;
				default: begin end
			endcase
		end
	end
endmodule
`default_nettype wire
