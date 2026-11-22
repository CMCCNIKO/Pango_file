//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 双域稳定 payload 与 toggle mailbox，将像素帧 ID 请求发送到 DDR，再将读取源槽/有效性/epoch 快照返回像素域。
// Module Name: quad_frame_content_cdc
// File Name: source/memory/quad_frame_content_cdc.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 双域稳定 payload 与 toggle mailbox，将像素帧 ID 请求发送到 DDR，再将读取源槽/有效性/epoch 快照返回像素域。
// 像素帧开始清除旧 valid，仅接受同帧 ID 回复；每显示帧最多一次 DDR 读启动，不给像素数据路径增加寄存级。
//
// Dependencies: DDR/像素域协调复位与稳定描述符；无存储 IP。
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
// Design Name: Frame matched source-presence sideband
// Module Name: quad_frame_content_cdc
// Project Name: PG2L100H_PCIe_RK3566_Algorithm
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 像素帧ID请求与DDR读快照回复均为稳定payload+toggle握手。
//              每帧开始清valid，仅接受同一帧ID回复；不影响任何像素路径。
// Dependencies: 双域协调复位，每显示帧最多一次DDR读启动
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module quad_frame_content_cdc
(
	input i_pixel_clk,
	input i_pixel_rstn,
	input i_pixel_frame_boundary,
	input i_ddr_clk,
	input i_ddr_rstn,
	input i_snapshot,
	input [3:0] i_source_present,
	output [3:0] o_source_present,
	output o_source_present_valid
);
	reg [31:0] reg_pixel_frame;
	reg [31:0] reg_frame_payload;
	reg flag_frame_request;
	reg flag_frame_ack;
	(* syn_preserve = 1 *) reg [1:0] reg_frame_ack_sync;
	(* syn_preserve = 1 *) reg [1:0] reg_frame_request_sync;
	reg flag_frame_seen;
	reg [31:0] reg_ddr_frame;
	reg [35:0] reg_response_payload;
	reg flag_response;
	reg flag_ack;
	(* syn_preserve = 1 *) reg [1:0] reg_ack_sync;
	(* syn_preserve = 1 *) reg [1:0] reg_response_sync;
	reg [3:0] source_present_o;
	reg source_present_valid_o;
	assign o_source_present = source_present_o;
	assign o_source_present_valid = source_present_valid_o;
	// 帧ID在下一帧开始前稳定；停止像素钟不会生成下一帧请求。
	always@(posedge i_pixel_clk or negedge i_pixel_rstn)begin
		if(!i_pixel_rstn)begin
			reg_pixel_frame <= 32'd0;
			reg_frame_payload <= 32'd0;
			flag_frame_request <= 1'b0;
			reg_frame_ack_sync <= 2'd0;
		end else begin
			reg_frame_ack_sync <= {reg_frame_ack_sync[0],flag_frame_ack};
			if(i_pixel_frame_boundary)begin
				reg_pixel_frame <= reg_pixel_frame + 32'd1;
				if(flag_frame_request == reg_frame_ack_sync[1])begin
					reg_frame_payload <= reg_pixel_frame + 32'd1;
					flag_frame_request <= !flag_frame_request;
				end
			end
		end
	end
	always@(posedge i_ddr_clk or negedge i_ddr_rstn)begin
		if(!i_ddr_rstn)begin
			reg_frame_request_sync <= 2'd0;
			flag_frame_seen <= 1'b0;
			flag_frame_ack <= 1'b0;
			reg_ddr_frame <= 32'd0;
			reg_ack_sync <= 2'd0;
		end else begin
			reg_frame_request_sync <= {reg_frame_request_sync[0],flag_frame_request};
			reg_ack_sync <= {reg_ack_sync[0],flag_ack};
			if(reg_frame_request_sync[1] != flag_frame_seen)begin
				reg_ddr_frame <= reg_frame_payload;
				flag_frame_seen <= reg_frame_request_sync[1];
				flag_frame_ack <= reg_frame_request_sync[1];
			end
		end
	end
	// 未消费的回复payload保持稳定。busy时只丢侧带，不停原reader。
	always@(posedge i_ddr_clk or negedge i_ddr_rstn)begin
		if(!i_ddr_rstn)begin
			reg_response_payload <= 36'd0;
			flag_response <= 1'b0;
		end else if(i_snapshot && flag_response == reg_ack_sync[1])begin
			reg_response_payload <= {reg_ddr_frame,i_source_present};
			flag_response <= !flag_response;
		end
	end
	always@(posedge i_pixel_clk or negedge i_pixel_rstn)begin
		if(!i_pixel_rstn)begin
			reg_response_sync <= 2'd0;
			flag_ack <= 1'b0;
			source_present_o <= 4'd0;
			source_present_valid_o <= 1'b0;
		end else begin
			reg_response_sync <= {reg_response_sync[0],flag_response};
			if(i_pixel_frame_boundary)begin
				source_present_o <= 4'd0;
				source_present_valid_o <= 1'b0;
			end
			if(reg_response_sync[1] != flag_ack)begin
				flag_ack <= reg_response_sync[1];
				if(!i_pixel_frame_boundary && reg_response_payload[35:4] == reg_pixel_frame)begin
					source_present_o <= reg_response_payload[3:0];
					source_present_valid_o <= 1'b1;
				end
			end
		end
	end
endmodule
