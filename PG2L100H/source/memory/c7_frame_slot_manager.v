//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-11
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 旧 C7 三帧槽管理器，维护 FREE、RECEIVING、READY、DISPLAYING、RETIRING 所有权。
// Module Name: c7_frame_slot_manager
// File Name: source/memory/c7_frame_slot_manager.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 旧 C7 三帧槽管理器，维护 FREE、RECEIVING、READY、DISPLAYING、RETIRING 所有权。
// 仅向网络域发布一个稳定 grant；完整写入后成为 READY，在 HDMI 帧边界切换显示，旧显示槽待读请求排空再释放。
//
// Dependencies: c7_ddr_write_scheduler、c7_ddr_read_scheduler 和已完成 CDC 的帧事件。
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
// Module Name: c7_frame_slot_manager
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Description: C7 三帧槽所有权管理。网络侧只会收到一个稳定的 grant 描述符；
//              写完成后才成为 READY，HDMI 帧边界才允许 READY->DISPLAYING。
//              被替换的 DISPLAYING 槽在读请求排空前保持 RETIRING。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module c7_frame_slot_manager(
	input i_clk,
	input i_rst_n,
	input i_frame_prepare,
	input i_frame_written,
	input i_frame_failed,
	input i_vblank,
	input i_retire_done,
	output reg [1:0] o_grant_slot,
	output reg o_grant_toggle,
	output reg [1:0] o_display_slot,
	output reg o_display_valid,
	output reg o_display_switch,
	output reg [31:0] o_valid_frame_count,
	output reg [31:0] o_dropped_frame_count,
	output reg [31:0] o_display_commit_count,
	output [2:0] o_slot0_state,
	output [2:0] o_slot1_state,
	output [2:0] o_slot2_state
);
	localparam [2:0] ST_FREE       = 3'd0;
	localparam [2:0] ST_RECEIVING  = 3'd1;
	localparam [2:0] ST_READY      = 3'd2;
	localparam [2:0] ST_DISPLAYING = 3'd3;
	localparam [2:0] ST_RETIRING   = 3'd4;

	reg [2:0] reg_slot0_state;
	reg [2:0] reg_slot1_state;
	reg [2:0] reg_slot2_state;
	reg reg_grant_outstanding;
	reg reg_receiving_valid;
	reg [1:0] reg_receiving_slot;
	reg reg_retiring_valid;
	reg [1:0] reg_retiring_slot;

	assign o_slot0_state = reg_slot0_state;
	assign o_slot1_state = reg_slot1_state;
	assign o_slot2_state = reg_slot2_state;

	task set_slot_state;
		input [1:0] slot;
		input [2:0] state;
		begin
			case (slot)
				2'd0: reg_slot0_state <= state;
				2'd1: reg_slot1_state <= state;
				default: reg_slot2_state <= state;
			endcase
		end
	endtask

	task issue_grant;
		input [1:0] slot;
		begin
			o_grant_slot <= slot;
			o_grant_toggle <= ~o_grant_toggle;
			reg_grant_outstanding <= 1'b1;
		end
	endtask

	always @(posedge i_clk or negedge i_rst_n) begin
		if (!i_rst_n) begin
			reg_slot0_state <= ST_FREE;
			reg_slot1_state <= ST_FREE;
			reg_slot2_state <= ST_FREE;
			reg_grant_outstanding <= 1'b0;
			reg_receiving_valid <= 1'b0;
			reg_receiving_slot <= 2'd0;
			reg_retiring_valid <= 1'b0;
			reg_retiring_slot <= 2'd0;
			o_grant_slot <= 2'd0;
			o_grant_toggle <= 1'b0;
			o_display_slot <= 2'd0;
			o_display_valid <= 1'b0;
			o_display_switch <= 1'b0;
			o_valid_frame_count <= 32'd0;
			o_dropped_frame_count <= 32'd0;
			o_display_commit_count <= 32'd0;
		end else begin
			o_display_switch <= 1'b0;

			if (i_frame_prepare) begin
				if (reg_grant_outstanding && !reg_receiving_valid) begin
					set_slot_state(o_grant_slot, ST_RECEIVING);
					reg_grant_outstanding <= 1'b0;
					reg_receiving_valid <= 1'b1;
					reg_receiving_slot <= o_grant_slot;
				end else begin
					o_dropped_frame_count <= o_dropped_frame_count + 1'b1;
				end
			end

			if (i_frame_written && reg_receiving_valid) begin
				set_slot_state(reg_receiving_slot, ST_READY);
				reg_receiving_valid <= 1'b0;
				o_valid_frame_count <= o_valid_frame_count + 1'b1;
			end else if (i_frame_failed && reg_receiving_valid) begin
				set_slot_state(reg_receiving_slot, ST_FREE);
				reg_receiving_valid <= 1'b0;
				o_dropped_frame_count <= o_dropped_frame_count + 1'b1;
			end

			if (i_retire_done && reg_retiring_valid) begin
				set_slot_state(reg_retiring_slot, ST_FREE);
				reg_retiring_valid <= 1'b0;
			end

			// 不允许尚有旧槽 RETIRING 时再次切换，避免同一 HDMI FIFO
			// 的残留读事务覆盖新画面。
			if (i_vblank && !reg_retiring_valid) begin
				if (reg_slot0_state == ST_READY) begin
					if (o_display_valid) begin
						set_slot_state(o_display_slot, ST_RETIRING);
						reg_retiring_valid <= 1'b1;
						reg_retiring_slot <= o_display_slot;
					end
					reg_slot0_state <= ST_DISPLAYING;
					o_display_slot <= 2'd0;
					o_display_valid <= 1'b1;
					o_display_switch <= 1'b1;
					o_display_commit_count <= o_display_commit_count + 1'b1;
				end else if (reg_slot1_state == ST_READY) begin
					if (o_display_valid) begin
						set_slot_state(o_display_slot, ST_RETIRING);
						reg_retiring_valid <= 1'b1;
						reg_retiring_slot <= o_display_slot;
					end
					reg_slot1_state <= ST_DISPLAYING;
					o_display_slot <= 2'd1;
					o_display_valid <= 1'b1;
					o_display_switch <= 1'b1;
					o_display_commit_count <= o_display_commit_count + 1'b1;
				end else if (reg_slot2_state == ST_READY) begin
					if (o_display_valid) begin
						set_slot_state(o_display_slot, ST_RETIRING);
						reg_retiring_valid <= 1'b1;
						reg_retiring_slot <= o_display_slot;
					end
					reg_slot2_state <= ST_DISPLAYING;
					o_display_slot <= 2'd2;
					o_display_valid <= 1'b1;
					o_display_switch <= 1'b1;
					o_display_commit_count <= o_display_commit_count + 1'b1;
				end
			end

			// 每次只提前授权一帧；grant 后 slot_id 一直稳定，直到该帧
			// 完成或作废，构成完整的多位描述符 CDC 契约。
			if (!reg_grant_outstanding && !reg_receiving_valid) begin
				if (reg_slot0_state == ST_FREE) begin
					issue_grant(2'd0);
				end else if (reg_slot1_state == ST_FREE) begin
					issue_grant(2'd1);
				end else if (reg_slot2_state == ST_FREE) begin
					issue_grant(2'd2);
				end
			end
		end
	end
endmodule
