//////////////////////////////////////////////////////////////////////////////////
// Company: HRBUST_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-11-15
// Module Name: host_vendor_video_fifo
// Project Name: PG2L200H HDMI / OV5640 source switch V006
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: 复用当前工程已生成的 32x512 异步 FIFO；两实例锁步承载视频。
// Dependencies: host_descriptor_fifo 及其原始官方生成依赖，参数保持不变。
// Revision: 2026-11-15 - 标准同步读接口转换为稳定的 ready/valid 输出。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`default_nettype none

module host_vendor_video_fifo #(
	parameter integer C_DATA_WIDTH = 56
) (
	input wire i_wr_clk,
	input wire i_wr_rst_n,
	input wire i_wr_valid,
	input wire [C_DATA_WIDTH-1:0] i_wr_data,
	output wire o_wr_full,
	output wire o_wr_overflow,
	input wire i_rd_clk,
	input wire i_rd_rst_n,
	output wire o_rd_valid,
	output wire [C_DATA_WIDTH-1:0] o_rd_data,
	output wire o_rd_drained,
	input wire i_rd_ready
);
	wire common_rst_n = i_wr_rst_n && i_rd_rst_n;
	(* async_reg = "true" *) reg [1:0] wr_reset_q;
	(* async_reg = "true" *) reg [1:0] rd_reset_q;
	wire [63:0] packed_wr_data = {{(64-C_DATA_WIDTH){1'b0}}, i_wr_data};
	wire [63:0] packed_rd_data;
	wire [1:0] wr_full;
	wire [1:0] rd_empty;
	wire wr_fire = wr_reset_q[1] && i_wr_valid && !o_wr_full;
	reg wr_overflow_q;

	// 任一侧复位时共同清空 IP；各时钟域独立同步释放。
	always @(posedge i_wr_clk or negedge common_rst_n) begin
		if (!common_rst_n) wr_reset_q <= 2'b00;
		else wr_reset_q <= {wr_reset_q[0], 1'b1};
	end
	always @(posedge i_rd_clk or negedge common_rst_n) begin
		if (!common_rst_n) rd_reset_q <= 2'b00;
		else rd_reset_q <= {rd_reset_q[0], 1'b1};
	end
	always @(posedge i_wr_clk or negedge common_rst_n) begin
		if (!common_rst_n) wr_overflow_q <= 1'b0;
		else wr_overflow_q <= wr_reset_q[1] && i_wr_valid && o_wr_full;
	end
	assign o_wr_full = |wr_full;
	assign o_wr_overflow = wr_overflow_q;

	// OUTPUT_REG=0 的官方 FIFO 为一拍同步读。两个预取寄存器包括一拍
	// 在途事务的容量，背压时保持输出不变，连续读取时每拍输出一个像素。
	reg [63:0] head_q;
	reg [63:0] tail_q;
	reg [1:0] count_q;
	reg rd_pending_q;
	wire pop = (count_q != 0) && i_rd_ready;
	wire [2:0] reserved_count = {1'b0, count_q} + rd_pending_q - pop;
	wire rd_fire = rd_reset_q[1] && !(|rd_empty) && (reserved_count < 2);
	assign o_rd_valid = (count_q != 0) && rd_reset_q[1];
	assign o_rd_data = head_q[C_DATA_WIDTH-1:0];
	// Both physical lanes, the in-flight synchronous read and both prefetch
	// registers must be empty before the stream wrapper acknowledges a flush.
	assign o_rd_drained = rd_reset_q[1] && (&rd_empty) &&
	                      (count_q == 0) && !rd_pending_q;

	always @(posedge i_rd_clk or negedge common_rst_n) begin
		if (!common_rst_n) begin
			head_q <= 0;
			tail_q <= 0;
			count_q <= 0;
			rd_pending_q <= 0;
		end else if (!rd_reset_q[1]) begin
			head_q <= 0;
			tail_q <= 0;
			count_q <= 0;
			rd_pending_q <= 0;
		end else begin
			rd_pending_q <= rd_fire;
			case ({rd_pending_q, pop})
				2'b10: begin
					if (count_q == 0) head_q <= packed_rd_data;
					else tail_q <= packed_rd_data;
					count_q <= count_q + 1'b1;
				end
				2'b01: begin
					head_q <= tail_q;
					count_q <= count_q - 1'b1;
				end
				2'b11: begin
					if (count_q == 1) head_q <= packed_rd_data;
					else begin
						head_q <= tail_q;
						tail_q <= packed_rd_data;
					end
				end
				default: begin end
			endcase
		end
	end

	genvar lane;
	generate
		for (lane = 0; lane < 2; lane = lane + 1) begin : g_fifo
			host_descriptor_fifo u_fifo (
				.wr_clk(i_wr_clk), .wr_rst(!wr_reset_q[1]),
				.wr_en(wr_fire), .wr_data(packed_wr_data[lane*32 +: 32]),
				.wr_full(wr_full[lane]), .wr_water_level(), .almost_full(),
				.rd_clk(i_rd_clk), .rd_rst(!rd_reset_q[1]),
				.rd_en(rd_fire), .rd_data(packed_rd_data[lane*32 +: 32]),
				.rd_empty(rd_empty[lane]), .rd_water_level(), .almost_empty()
			);
		end
	endgenerate
endmodule
`default_nettype wire
