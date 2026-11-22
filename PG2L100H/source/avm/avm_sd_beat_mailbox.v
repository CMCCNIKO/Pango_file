//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-14
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: SD 系统域到 DDR 域单 beat 稳定数据 mailbox，发送端先保持地址/数据再翻转请求，接收端同步后额外等待两拍捕获。
// Module Name: avm_sd_beat_mailbox
// File Name: source/avm/avm_sd_beat_mailbox.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// SD 系统域到 DDR 域单 beat 稳定数据 mailbox，发送端先保持地址/数据再翻转请求，接收端同步后额外等待两拍捕获。
// ACK 仅由实际 DDR W 接受产生，写事务未排空时不得单独复位；多位数据总线须保留对应 max_delay 约束。
//
// Dependencies: avm_single_beat_writer、配套 mailbox 数据路径约束。
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
// Create Date: 2026-10-14
// Design Name: SD LUT stable-data mailbox
// Module Name: avm_sd_beat_mailbox
// Project Name: PG2L100H SD AVM
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 发送端先稳定数据再翻转请求，接收端同步后再等待两拍捕获。
//              ACK仅由实际DDR W接受产生；禁止在写事务未排空时复位。
// Dependencies: avm_single_beat_writer; 数据总线配套max_delay约束。
// Revision: 0.01 - 首次实现。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module avm_sd_beat_mailbox
(
	input i_clk,
	input i_rstn,
	input i_enable,
	input i_request,
	input [27:0] i_address_hold,
	input [255:0] i_data_hold,
	output o_ack,
	output o_valid,
	output [27:0] o_address,
	output [255:0] o_data,
	input i_ready,
	output o_idle
);
	localparam [1:0] ST_IDLE = 2'd0;
	localparam [1:0] ST_SETTLE = 2'd1;
	localparam [1:0] ST_CAPTURE = 2'd2;
	localparam [1:0] ST_SEND = 2'd3;
	reg [1:0] state_current;
	reg [1:0] state_next;
	reg reg_request_meta /* synthesis syn_preserve = 1 */;
	reg reg_request_sync /* synthesis syn_preserve = 1 */;
	reg ack_o;
	reg [27:0] address_o;
	reg [255:0] data_o;
	assign o_ack = ack_o;
	assign o_valid = state_current == ST_SEND;
	assign o_address = address_o;
	assign o_data = data_o;
	assign o_idle = state_current == ST_IDLE;
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			reg_request_meta <= 1'b0;
			reg_request_sync <= 1'b0;
		end else begin
			reg_request_meta <= i_request;
			reg_request_sync <= reg_request_meta;
		end
	end
	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_IDLE:begin
				if(i_enable && (reg_request_sync != ack_o))begin
					state_next = ST_SETTLE;
				end
			end
			ST_SETTLE:begin state_next = ST_CAPTURE; end
			ST_CAPTURE:begin state_next = ST_SEND; end
			ST_SEND:begin
				if(i_ready)begin state_next = ST_IDLE; end
			end
			default:begin state_next = ST_IDLE; end
		endcase
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin state_current <= ST_IDLE; end
		else begin state_current <= state_next; end
	end
	// 请求同步两拍，再经过SETTLE/CAPTURE窗口才采样整条稳定总线。
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin
			address_o <= 28'd0;
			data_o <= 256'd0;
		end else if(state_current == ST_CAPTURE)begin
			address_o <= i_address_hold;
			data_o <= i_data_hold;
		end
	end
	always@(posedge i_clk or negedge i_rstn)begin
		if(!i_rstn)begin ack_o <= 1'b0; end
		else if((state_current == ST_SEND) && i_ready)begin
			ack_o <= reg_request_sync;
		end
	end
endmodule
