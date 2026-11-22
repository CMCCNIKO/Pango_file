//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-11
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 旧 C7 单源 DISPLAYING 帧槽读调度器，从当前槽地址发出 HMIC 读请求并将有效返回交给输出 FIFO。
// Module Name: c7_ddr_read_scheduler
// File Name: source/memory/c7_ddr_read_scheduler.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 旧 C7 单源 DISPLAYING 帧槽读调度器，从当前槽地址发出 HMIC 读请求并将有效返回交给输出 FIFO。
// display_switch 后从新槽首地址重启；已发出但未完成的旧槽响应先排空并丢弃，防止旧帧尾部进入新帧。
//
// Dependencies: HMIC 读口和 c7_ddr_to_hdmi_fifo_wrapper。
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
// Module Name: c7_ddr_read_scheduler
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: C7 DISPLAYING 槽读取器。display_switch 时从新槽首地址重新读，
//              所有已发出但尚未返回的旧槽读响应都会丢弃，绝不压入新帧 FIFO。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module c7_ddr_read_scheduler #(
	parameter integer C_FRAME_WORDS = 12800,
	parameter [27:0] C_SLOT0_BASE = 28'h0000000,
	parameter [27:0] C_SLOT1_BASE = 28'h0040000,
	parameter [27:0] C_SLOT2_BASE = 28'h0080000,
	parameter [27:0] C_WORD_ADDR_STRIDE = 28'd8
)(
	input i_clk,
	input i_rst_n,
	input i_display_valid,
	input [1:0] i_display_slot,
	input i_display_switch,
	input i_output_full,
	output reg o_output_push,
	output reg [255:0] o_output_data,
	output reg [27:0] o_axi_araddr,
	output o_axi_aruser_ap,
	output [3:0] o_axi_aruser_id,
	output reg [3:0] o_axi_arlen,
	input i_axi_arready,
	output reg o_axi_arvalid,
	input [255:0] i_axi_rdata,
	input i_axi_rvalid,
	output reg o_switch_done,
	output reg [15:0] o_words_read
);
	localparam [2:0] ST_WAIT = 3'd0;
	localparam [2:0] ST_AR = 3'd1;
	localparam [2:0] ST_R = 3'd2;
	localparam [2:0] ST_PUSH = 3'd3;
	localparam [2:0] ST_DROP = 3'd4;
	reg [2:0] reg_state;
	reg [255:0] reg_response_data;

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

	assign o_axi_aruser_ap = 1'b0;
	assign o_axi_aruser_id = 4'd0;

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			reg_state <= ST_WAIT;
			reg_response_data <= 256'd0;
			o_output_push <= 1'b0;
			o_output_data <= 256'd0;
			o_axi_araddr <= C_SLOT0_BASE;
			o_axi_arlen <= 4'd0;
			o_axi_arvalid <= 1'b0;
			o_switch_done <= 1'b0;
			o_words_read <= 16'd0;
		end else begin
			o_output_push <= 1'b0;
			o_switch_done <= 1'b0;
			if (i_display_switch) begin
				o_axi_arvalid <= 1'b0;
				o_axi_araddr <= slot_base(i_display_slot);
				o_words_read <= 16'd0;
				if ((reg_state == ST_R) && i_axi_rvalid) begin
					// 本拍返回的是旧槽数据，切换时直接丢弃。
					reg_state <= ST_WAIT;
					o_switch_done <= 1'b1;
				end else if ((reg_state == ST_R) ||
					((reg_state == ST_AR) && o_axi_arvalid && i_axi_arready)) begin
					reg_state <= ST_DROP;
				end else begin
					reg_state <= ST_WAIT;
					o_switch_done <= 1'b1;
				end
			end else begin
				case (reg_state)
					ST_WAIT: begin
						o_axi_arvalid <= 1'b0;
						if (i_display_valid && !i_output_full) reg_state <= ST_AR;
					end
					ST_AR: begin
						if (i_output_full) begin
							o_axi_arvalid <= 1'b0;
							reg_state <= ST_WAIT;
						end else if (o_axi_arvalid && i_axi_arready) begin
							o_axi_arvalid <= 1'b0;
							reg_state <= ST_R;
						end else begin
							o_axi_arvalid <= 1'b1;
						end
					end
					ST_R: begin
						if (i_axi_rvalid) begin
							reg_response_data <= i_axi_rdata;
							reg_state <= ST_PUSH;
						end
					end
					ST_PUSH: begin
						if (!i_output_full) begin
							o_output_data <= reg_response_data;
							o_output_push <= 1'b1;
							if (o_words_read == C_FRAME_WORDS - 1) begin
								o_words_read <= 16'd0;
								o_axi_araddr <= slot_base(i_display_slot);
							end else begin
								o_words_read <= o_words_read + 1'b1;
								o_axi_araddr <= o_axi_araddr + C_WORD_ADDR_STRIDE;
							end
							reg_state <= ST_WAIT;
						end
					end
					default: begin
						if (i_axi_rvalid) begin
							reg_state <= ST_WAIT;
							o_switch_done <= 1'b1;
						end
					end
				endcase
			end
		end
	end
endmodule
