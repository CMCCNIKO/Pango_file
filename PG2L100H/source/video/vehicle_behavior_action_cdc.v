//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 通过稳定载荷单项邮箱将用户行为动作从 Ethernet 域发送到行为判定域，并返回一致性状态快照。
// Module Name: vehicle_behavior_action_cdc
// File Name: source/video/vehicle_behavior_action_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 通过稳定载荷单项邮箱将用户行为动作从 Ethernet 域发送到行为判定域，并返回一致性状态快照。
// 任一端复位清空两端会话；ready/online 指示可接收性，未确认载荷保持稳定。
//
// Dependencies: 上层动作接收及 vehicle_behavior_decision；双域时钟与复位。
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
// Create Date: 2026-10-18
// Module Name: vehicle_behavior_action_cdc
// Project Name: PG2L100H_AVM_PCIE
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Stable-payload mailbox; either reset flushes both endpoints.
// Revision: 0.02 - 2026-10-20 Ready/online and coherent state snapshots.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module vehicle_behavior_action_cdc #(
	parameter integer C_DATA_WIDTH = 21,
	parameter integer C_CHANGE_ONLY = 0
)(
	input i_src_clk,
	input i_src_rst_n,
	input i_src_valid,
	input [C_DATA_WIDTH - 1:0] i_src_payload,
	output o_src_ready,
	output o_src_online,
	input i_dst_clk,
	input i_dst_rst_n,
	input i_dst_online,
	output o_dst_valid,
	output [C_DATA_WIDTH - 1:0] o_dst_payload
);
	wire common_rst_n;
	wire src_rst_n;
	wire dst_rst_n;
	reg [2:0] src_reset_sync;
	reg [2:0] dst_reset_sync;
	reg [1:0] src_online_sync;
	reg [C_DATA_WIDTH - 1:0] src_payload /* synthesis syn_preserve = 1 */;
	reg src_toggle /* synthesis syn_preserve = 1 */;
	reg src_sent;
	reg [1:0] src_ack_sync;
	reg [1:0] dst_toggle_sync;
	reg dst_seen;
	reg dst_valid_o;
	reg [C_DATA_WIDTH - 1:0] dst_payload_o /* synthesis syn_preserve = 1 */;
	assign common_rst_n = i_src_rst_n && i_dst_rst_n && i_dst_online;
	assign src_rst_n = src_reset_sync[2];
	assign dst_rst_n = dst_reset_sync[2];
	assign o_src_online = src_rst_n && src_online_sync[1];
	assign o_src_ready = o_src_online && src_toggle == src_ack_sync[1];
	assign o_dst_valid = dst_valid_o;
	assign o_dst_payload = dst_payload_o;

	// Flush both toggle phases on a reset or link loss; release in each domain.
	always@(posedge i_src_clk or negedge common_rst_n)begin
		if(!common_rst_n)begin
			src_reset_sync <= 3'd0;
		end else begin
			src_reset_sync <= {src_reset_sync[1:0],1'b1};
		end
	end
	always@(posedge i_dst_clk or negedge common_rst_n)begin
		if(!common_rst_n)begin
			dst_reset_sync <= 3'd0;
		end else begin
			dst_reset_sync <= {dst_reset_sync[1:0],1'b1};
		end
	end
	always@(posedge i_src_clk or negedge src_rst_n)begin
		if(!src_rst_n)begin
			src_payload <= {C_DATA_WIDTH{1'b0}};
			src_toggle <= 1'b0;
			src_sent <= 1'b0;
			src_ack_sync <= 2'd0;
			src_online_sync <= 2'd0;
		end else begin
			src_ack_sync <= {src_ack_sync[0],dst_seen};
			src_online_sync <= {src_online_sync[0],dst_rst_n};
			if(i_src_valid && o_src_ready &&
				(C_CHANGE_ONLY == 0 || !src_sent || i_src_payload != src_payload))begin
				src_payload <= i_src_payload;
				src_toggle <= !src_toggle;
				src_sent <= 1'b1;
			end
		end
	end
	always@(posedge i_dst_clk or negedge dst_rst_n)begin
		if(!dst_rst_n)begin
			dst_toggle_sync <= 2'd0;
			dst_seen <= 1'b0;
			dst_valid_o <= 1'b0;
			dst_payload_o <= {C_DATA_WIDTH{1'b0}};
		end else begin
			dst_valid_o <= 1'b0;
			dst_toggle_sync <= {dst_toggle_sync[0],src_toggle};
			if(dst_toggle_sync[1] != dst_seen)begin
				dst_seen <= dst_toggle_sync[1];
				dst_payload_o <= src_payload;
				dst_valid_o <= 1'b1;
			end
		end
	end
endmodule
