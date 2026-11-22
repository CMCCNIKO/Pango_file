//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-11
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 旧 C7 单帧写调度器，根据稳定 grant slot_id 确定基址，将 256-bit FIFO 字写入 HMIC。
// Module Name: c7_ddr_write_scheduler
// File Name: source/memory/c7_ddr_write_scheduler.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 旧 C7 单帧写调度器，根据稳定 grant slot_id 确定基址，将 256-bit FIFO 字写入 HMIC。
// 仅在收到 commit 且实际完成 12800 个字后发布 written；abort/写错误使本帧失败，保持一次只接收一帧。
//
// Dependencies: 外部网络到 DDR FIFO、c7_frame_slot_manager、HMIC 写口。
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
//
// Create Date: 2026-09-11
// Module Name: c7_ddr_write_scheduler
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: C7 单槽接收写调度器。仅同时接收一帧；稳定 grant 的 slot_id
//              决定写基地址。完成 12800 个字且收到 commit 才发出 written。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module c7_ddr_write_scheduler #(
	parameter integer C_FRAME_WORDS = 12800,
	parameter [27:0] C_SLOT0_BASE = 28'h0000000,
	parameter [27:0] C_SLOT1_BASE = 28'h0040000,
	parameter [27:0] C_SLOT2_BASE = 28'h0080000,
	parameter [27:0] C_WORD_ADDR_STRIDE = 28'd8
)(
	input i_clk,
	input i_rst_n,
	input i_ddr_init_done,
	input i_frame_prepare,
	input [1:0] i_write_slot,
	input i_frame_commit,
	input i_frame_abort,
	input [255:0] i_word_data,
	input i_word_empty,
	output reg o_word_pop,
	output reg [27:0] o_axi_awaddr,
	output o_axi_awuser_ap,
	output [3:0] o_axi_awuser_id,
	output reg [3:0] o_axi_awlen,
	input i_axi_awready,
	output reg o_axi_awvalid,
	output reg [255:0] o_axi_wdata,
	output reg [31:0] o_axi_wstrb,
	input i_axi_wready,
	output reg o_frame_written,
	output reg o_frame_failed,
	output reg [15:0] o_words_written
);
	localparam [2:0] ST_IDLE = 3'd0;
	localparam [2:0] ST_WAIT = 3'd1;
	localparam [2:0] ST_AW   = 3'd2;
	localparam [2:0] ST_W    = 3'd3;
	localparam [2:0] ST_DONE = 3'd4;
	reg [2:0] reg_state;
	reg reg_active;
	reg reg_commit_seen;
	reg reg_abort_pending;

	function [27:0] slot_base;
		input [1:0] slot;
		begin
			case (slot)
				2'd0: slot_base = C_SLOT0_BASE;
				2'd1: slot_base = C_SLOT1_BASE;
				default: slot_base = C_SLOT2_BASE;
			endcase
		end
	endfunction

	assign o_axi_awuser_ap = 1'b0;
	assign o_axi_awuser_id = 4'd0;

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			reg_state <= ST_IDLE;
			reg_active <= 1'b0;
			reg_commit_seen <= 1'b0;
			reg_abort_pending <= 1'b0;
			o_word_pop <= 1'b0;
			o_axi_awaddr <= C_SLOT0_BASE;
			o_axi_awlen <= 4'd0;
			o_axi_awvalid <= 1'b0;
			o_axi_wdata <= 256'd0;
			o_axi_wstrb <= 32'hffff_ffff;
			o_frame_written <= 1'b0;
			o_frame_failed <= 1'b0;
			o_words_written <= 16'd0;
		end else begin
			o_word_pop <= 1'b0;
			o_frame_written <= 1'b0;
			o_frame_failed <= 1'b0;
			if (i_frame_prepare && !reg_active) begin
				reg_active <= 1'b1;
				reg_commit_seen <= 1'b0;
				reg_abort_pending <= 1'b0;
				o_axi_awaddr <= slot_base(i_write_slot);
				o_words_written <= 16'd0;
				reg_state <= ST_WAIT;
			end
			if (i_frame_commit && reg_active) reg_commit_seen <= 1'b1;
			if (i_frame_abort && reg_active) reg_abort_pending <= 1'b1;

			case (reg_state)
				ST_IDLE: o_axi_awvalid <= 1'b0;
				ST_WAIT: begin
					o_axi_awvalid <= 1'b0;
					if (reg_abort_pending || i_frame_abort) begin
						reg_active <= 1'b0;
						reg_abort_pending <= 1'b0;
						o_frame_failed <= 1'b1;
						reg_state <= ST_IDLE;
					end else if (i_ddr_init_done && !i_word_empty) begin
						o_word_pop <= 1'b1;
						o_axi_wdata <= i_word_data;
						reg_state <= ST_AW;
					end
				end
				ST_AW: begin
					if (o_axi_awvalid && i_axi_awready) begin
						o_axi_awvalid <= 1'b0;
						reg_state <= ST_W;
					end else if (reg_abort_pending || i_frame_abort) begin
						o_axi_awvalid <= 1'b0;
						reg_active <= 1'b0;
						reg_abort_pending <= 1'b0;
						o_frame_failed <= 1'b1;
						reg_state <= ST_IDLE;
					end else begin
						o_axi_awvalid <= 1'b1;
					end
				end
				ST_W: begin
					if (i_axi_wready) begin
						if (reg_abort_pending || i_frame_abort) begin
							reg_active <= 1'b0;
							reg_abort_pending <= 1'b0;
							o_frame_failed <= 1'b1;
							reg_state <= ST_IDLE;
						end else if (o_words_written == C_FRAME_WORDS - 1) begin
							o_words_written <= C_FRAME_WORDS;
							if (reg_commit_seen || i_frame_commit) begin
								reg_active <= 1'b0;
								o_frame_written <= 1'b1;
								reg_state <= ST_IDLE;
							end else begin
								reg_state <= ST_DONE;
							end
						end else begin
							o_words_written <= o_words_written + 1'b1;
							o_axi_awaddr <= o_axi_awaddr + C_WORD_ADDR_STRIDE;
							reg_state <= ST_WAIT;
						end
					end
				end
				default: begin
					if (reg_abort_pending || i_frame_abort) begin
						reg_active <= 1'b0;
						reg_abort_pending <= 1'b0;
						o_frame_failed <= 1'b1;
						reg_state <= ST_IDLE;
					end else if (reg_commit_seen || i_frame_commit) begin
						reg_active <= 1'b0;
						o_frame_written <= 1'b1;
						reg_state <= ST_IDLE;
					end
				end
			endcase
		end
	end
endmodule
