//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 按 job 把 32 位资产 ROM 的连续字组装成原 tile 加载端口所需的 PW512、DW128、QBF32 或 scale16 字。
// Module Name: f14_asset_word_loader
// File Name: model/rtl/f14_asset_word_loader.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 按 job 把 32 位资产 ROM 的连续字组装成原 tile 加载端口所需的 PW512、DW128、QBF32 或 scale16 字。
// 仅使用最多 512 位暂存，逐输出字握手，检查数量及 ROM 地址范围，不缓存整层权重。
//
// Dependencies: 外部资产 ROM 与 tile 的 PW/DW/QBF/qscale 加载接口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// job_count 单位为输出字，ROM 地址单位为 32 位字；ROM 固定单拍并可连续读，输出可背压。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: 32-bit asset ROM to existing tile load ports, no arithmetic array.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

// 每个job搬一段PW/DW/QBF/scale。ROM固定一拍、支持每拍读取；输出可背压。
// 仅512位暂存寄存器，不保存整层权重。kind:0=PW512,1=DW128,2=QBF32,3=scale16。
// 最后一类的ROM字低16位有效。job_count单位为输出字，地址单位为32位ROM字。
module f14_asset_word_loader (
	input wire i_clk,
	input wire i_rst_n,
	input wire i_job_valid,
	output wire o_job_ready,
	input wire [16:0] i_job_base,
	input wire [10:0] i_job_count,
	input wire [1:0] i_job_kind,
	output wire o_job_done,
	output wire o_job_error,
	output wire o_busy,
	output wire o_rom_rd_en,
	output wire [16:0] o_rom_rd_addr,
	input wire i_rom_rd_valid,
	input wire [31:0] i_rom_rd_data,
	output wire o_word_valid,
	input wire i_word_ready,
	output wire [1:0] o_word_kind,
	output wire [9:0] o_word_addr,
	output wire [511:0] o_word_data
);
	localparam [1:0] ST_IDLE = 2'd0;
	localparam [1:0] ST_FILL = 2'd1;
	localparam [1:0] ST_PUSH = 2'd2;
	reg [1:0] state_q;
	reg [1:0] state_d;
	reg [1:0] kind_q;
	reg [10:0] count_q;
	reg [9:0] index_q;
	reg [4:0] beats_q;
	reg [4:0] issued_q;
	reg [4:0] received_q;
	reg [16:0] rom_addr_q;
	reg [511:0] buffer_q;
	reg done_o;
	reg error_o;
	wire [2:0] request_shift;
	wire [20:0] request_end;
	wire invalid_request;
	wire unexpected_response;
	wire final_response;
	wire final_word;

	assign request_shift = (i_job_kind == 2'd0) ? 3'd4 :
		((i_job_kind == 2'd1) ? 3'd2 : 3'd0);
	assign request_end = {4'd0,i_job_base} + ({10'd0,i_job_count} << request_shift);
	assign invalid_request = (i_job_count > 11'd1024) ||
		((i_job_kind == 2'd1) && (i_job_count > 11'd512)) ||
		(request_end > 21'd131072);
	assign unexpected_response = i_rom_rd_valid && (received_q >= issued_q);
	assign final_response = i_rom_rd_valid && ((received_q + 5'd1) == beats_q);
	assign final_word = ({1'b0,index_q} + 11'd1) == count_q;
	assign o_job_ready = state_q == ST_IDLE;
	assign o_job_done = done_o;
	assign o_job_error = error_o;
	assign o_busy = state_q != ST_IDLE;
	assign o_rom_rd_en = (state_q == ST_FILL) && (issued_q < beats_q);
	assign o_rom_rd_addr = rom_addr_q;
	assign o_word_valid = state_q == ST_PUSH;
	assign o_word_kind = kind_q;
	assign o_word_addr = index_q;
	assign o_word_data = buffer_q;

	always@(*)begin
		state_d = state_q;
		case(state_q)
			ST_IDLE:begin
				if(i_job_valid && !invalid_request && (i_job_count != 0))begin
					state_d = ST_FILL;
				end
			end
			ST_FILL:begin
				if(unexpected_response)begin
					state_d = ST_IDLE;
				end else if(final_response)begin
					state_d = ST_PUSH;
				end
			end
			ST_PUSH:begin
				if(i_word_ready)begin
					state_d = final_word ? ST_IDLE : ST_FILL;
				end
			end
			default:state_d = ST_IDLE;
		endcase
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			state_q <= ST_IDLE;
		end else begin
			state_q <= state_d;
		end
	end

	always@(posedge i_clk or negedge i_rst_n)begin
		if(!i_rst_n)begin
			kind_q <= 2'd0;
			count_q <= 11'd0;
			index_q <= 10'd0;
			beats_q <= 5'd0;
			issued_q <= 5'd0;
			received_q <= 5'd0;
			rom_addr_q <= 17'd0;
			buffer_q <= 512'd0;
			done_o <= 1'b0;
			error_o <= 1'b0;
		end else begin
			done_o <= 1'b0;
			error_o <= 1'b0;
			case(state_q)
				ST_IDLE:begin
					if(i_job_valid)begin
						if(invalid_request || (i_job_count == 0))begin
							done_o <= 1'b1;
							error_o <= invalid_request;
						end else begin
							kind_q <= i_job_kind;
							count_q <= i_job_count;
							index_q <= 10'd0;
							beats_q <= (i_job_kind == 0) ? 5'd16 : ((i_job_kind == 1) ? 5'd4 : 5'd1);
							issued_q <= 5'd0;
							received_q <= 5'd0;
							rom_addr_q <= i_job_base;
							buffer_q <= 512'd0;
						end
					end
				end
				ST_FILL:begin
					if(o_rom_rd_en)begin
						issued_q <= issued_q + 5'd1;
						rom_addr_q <= rom_addr_q + 17'd1;
					end
					if(unexpected_response)begin
						done_o <= 1'b1;
						error_o <= 1'b1;
					end else if(i_rom_rd_valid)begin
						buffer_q[received_q * 32 +: 32] <= i_rom_rd_data;
						received_q <= received_q + 5'd1;
					end
				end
				ST_PUSH:begin
					if(i_word_ready)begin
						if(final_word)begin
							done_o <= 1'b1;
						end else begin
							index_q <= index_q + 10'd1;
							issued_q <= 5'd0;
							received_q <= 5'd0;
							buffer_q <= 512'd0;
						end
					end
				end
				default:begin
					done_o <= 1'b1;
					error_o <= 1'b1;
				end
			endcase
		end
	end
endmodule
`default_nettype wire
