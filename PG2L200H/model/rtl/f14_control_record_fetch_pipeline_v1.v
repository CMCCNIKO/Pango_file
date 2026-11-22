//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 从运行资产 ROM 的控制记录区按需读取每条 32 字记录，逐字段输出索引和数据。
// Module Name: f14_control_record_fetch
// File Name: model/rtl/f14_control_record_fetch_pipeline_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 从运行资产 ROM 的控制记录区按需读取每条 32 字记录，逐字段输出索引和数据。
// 仅保留一个 32 位缓冲字，校验首字记录标记；仲裁后的有序响应允许等待，由上层 watchdog 约束响应丢失。
//
// Dependencies: 外部 f14_runtime_asset_rom_r4 对应记录区及 ROM 仲裁器
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 原修订0.02：复位保留 ROM 地址操作数，接受新 job 时初始化；调用方与 ROM 流水共同清除在途 valid。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 on-demand control record reader
// Module Name: f14_control_record_fetch
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: One32bit buffer,32 words/record; fixed one-cycle ROM response.
// Dependencies: External initialized official asset/control ROM r4
// Revision: 0.02 - Keep ROM address operands across reset; initialize on accepted job
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

// 调用方接收当前记录的字段；此处不复制完整descriptor表，也不保存全部32字。
// ROM端口仅在本job期间归本模块独占；复位时调用方同步清除在途valid。
module f14_control_record_fetch
(
	input wire i_clk,
	input wire i_rst_n,
	input wire i_job_valid,
	output wire o_job_ready,
	input wire [4:0] i_record_index,
	output wire o_job_done,
	output wire o_job_error,
	output wire o_busy,
	output wire o_rom_rd_en,
	output wire [16:0] o_rom_rd_addr,
	input wire i_rom_rd_valid,
	input wire [31:0] i_rom_rd_data,
	output wire o_word_valid,
	input wire i_word_ready,
	output wire [4:0] o_word_index,
	output wire [31:0] o_word_data
);
	localparam [2:0] ST_IDLE = 3'd0;
	localparam [2:0] ST_ISSUE = 3'd1;
	localparam [2:0] ST_WAIT = 3'd2;
	localparam [2:0] ST_SEND = 3'd3;
	localparam [2:0] ST_DONE = 3'd4;
	reg [2:0] state_current;
	reg [2:0] state_next;
	reg active_q;
	reg [4:0] record_q;
	reg [4:0] index_q;
	reg [31:0] data_q;
	reg error_q;
	wire job_fire;
	wire response_error;
	wire tag_error;

	assign o_job_ready = active_q && (state_current == ST_IDLE);
	assign o_job_done = active_q && (state_current == ST_DONE);
	assign o_job_error = error_q;
	assign o_busy = active_q && (state_current != ST_IDLE);
	assign job_fire = i_job_valid && o_job_ready;
	assign o_rom_rd_en = active_q && (state_current == ST_ISSUE);
	assign o_rom_rd_addr = 17'd128192 + {7'd0,record_q,5'd0} + {12'd0,index_q};
	assign response_error = 1'b0; // 仲裁后的有序响应允许延迟；整层watchdog约束丢失响应。
	assign tag_error = i_rom_rd_valid && (index_q == 5'd0) &&
		(i_rom_rd_data != (32'hf14c0200 | {27'd0,record_q}));
	assign o_word_valid = active_q && (state_current == ST_SEND);
	assign o_word_index = index_q;
	assign o_word_data = data_q;

	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_IDLE:begin
				if(job_fire)begin
					state_next = (i_record_index < 5'd28) ? ST_ISSUE : ST_DONE;
				end
			end
			ST_ISSUE:begin state_next = ST_WAIT; end
			ST_WAIT:begin
				if(response_error || tag_error)begin state_next = ST_DONE; end
				else if(i_rom_rd_valid)begin state_next = ST_SEND; end
			end
			ST_SEND:begin
				if(i_word_ready)begin
					state_next = (index_q == 5'd31) ? ST_DONE : ST_ISSUE;
				end
			end
			ST_DONE:begin state_next = ST_IDLE; end
			default:begin state_next = ST_IDLE; end
		endcase
	end
	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			state_current <= ST_IDLE;
			active_q <= 1'b0;
			data_q <= 32'd0;
			error_q <= 1'b0;
		end else begin
			state_current <= state_next;
			active_q <= 1'b1;
			if(job_fire)begin
				error_q <= i_record_index >= 5'd28;
			end
			if(state_current == ST_WAIT && i_rom_rd_valid)begin
				if(response_error || tag_error)begin error_q <= 1'b1; end
				else begin data_q <= i_rom_rd_data; end
			end
		end
	end
	// BRAM地址操作数不随异步复位跳变；valid先清除，新job在首次读前赋值。
	always@(posedge i_clk)begin
		if(i_rst_n)begin
			if(job_fire)begin
				record_q <= i_record_index;
				index_q <= 5'd0;
			end else if((state_current == ST_SEND) && i_word_ready)begin
				index_q <= index_q + 5'd1;
			end
		end
	end
endmodule
`default_nettype wire
