//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 通过稳定 bundled-data 载荷和 req/ack toggle 将算法配置跨到像素域。
// Module Name: algo_config_cdc_v2
// File Name: source/video/control_v2/algo_config_cdc_v2.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 通过稳定 bundled-data 载荷和 req/ack toggle 将算法配置跨到像素域。
// 目标域先捕获请求，只在此后的帧边界应用并返回确认；目的时钟暂停时保持 busy，要求双域协调复位。
//
// Dependencies: 调用方的复位同步器、帧边界及 CDC 时序约束。
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
// Create Date: 2026-10-10
// Design Name: HDMI algorithm control
// Module Name: algo_config_cdc_v2
// Project Name: PG2L100H_PCIe_RK3566
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: Stable bundled payload with request/acknowledge toggle handshake.
//              Apply only on a boundary AFTER destination captures the request.
//              Both domains require coordinated reset, released synchronously per domain.
//              Stopped destination clock holds busy. Independent reset is unsupported.
// Dependencies: source and destination reset synchronizers; CDC timing constraints
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module algo_config_cdc_v2
(
	input i_src_clk,
	input i_src_rst_n,
	input i_dst_clk,
	input i_dst_rst_n,
	input i_update,
	input [63:0] i_config,
	input i_frame_boundary,
	output o_busy,
	output o_applied,
	output [63:0] o_active_config,
	output [15:0] o_config_epoch,
	output o_pending
);
	reg [63:0] reg_payload;
	reg flag_request;
	reg flag_ack;
	(* syn_preserve = 1 *) reg [1:0] reg_ack_sync;
	(* syn_preserve = 1 *) reg [1:0] reg_request_sync;
	reg reg_ack_previous;
	reg applied_o;
	reg pending_o;
	reg [63:0] reg_pending_payload;
	reg [63:0] active_config_o;
	reg [15:0] config_epoch_o;

	// Keep the accepted sequence stable until the source consumes applied_o.
	assign o_busy = (flag_request != reg_ack_sync[1]) ||
		(reg_ack_sync[1] != reg_ack_previous) || applied_o;
	assign o_applied = applied_o;
	assign o_pending = pending_o;
	assign o_active_config = active_config_o;
	assign o_config_epoch = config_epoch_o;

	// Payload stays fixed until the destination applies it and ACK returns.
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			reg_payload <= 64'h000fe619dc010080;
		end else if(i_update && !o_busy)begin
			reg_payload <= i_config;
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			flag_request <= 1'b0;
		end else if(i_update && !o_busy)begin
			flag_request <= !flag_request;
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			reg_ack_sync <= 2'd0;
		end else begin
			reg_ack_sync <= {reg_ack_sync[0], flag_ack};
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			reg_ack_previous <= 1'b0;
		end else begin
			reg_ack_previous <= reg_ack_sync[1];
		end
	end
	always@(posedge i_src_clk or negedge i_src_rst_n)begin
		if(!i_src_rst_n)begin
			applied_o <= 1'b0;
		end else begin
			applied_o <= reg_ack_sync[1] != reg_ack_previous;
		end
	end
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			reg_request_sync <= 2'd0;
		end else begin
			reg_request_sync <= {reg_request_sync[0], flag_request};
		end
	end
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			pending_o <= 1'b0;
		end else if(pending_o && i_frame_boundary)begin
			pending_o <= 1'b0;
		end else if(reg_request_sync[1] != flag_ack)begin
			pending_o <= 1'b1;
		end
	end
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			reg_pending_payload <= 64'h000fe619dc010080;
		end else if(!pending_o && reg_request_sync[1] != flag_ack)begin
			reg_pending_payload <= reg_payload;
		end
	end
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			active_config_o <= 64'h000fe619dc010080;
			config_epoch_o <= 16'd0;
		end else if(pending_o && i_frame_boundary)begin
			active_config_o <= reg_pending_payload;
			if(active_config_o != reg_pending_payload)begin
				config_epoch_o <= config_epoch_o + 16'd1;
			end
		end
	end
	always@(posedge i_dst_clk or negedge i_dst_rst_n)begin
		if(!i_dst_rst_n)begin
			flag_ack <= 1'b0;
		end else if(pending_o && i_frame_boundary)begin
			flag_ack <= reg_request_sync[1];
		end
	end
endmodule
