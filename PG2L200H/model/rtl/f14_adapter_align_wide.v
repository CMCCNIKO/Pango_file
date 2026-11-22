//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将 CHW U8 的 64×11×14 adapter 特征，以最近邻方式对齐为 64×21×28，在线生成字节地址及 packed 地址。
// Module Name: f14_adapter_align
// File Name: model/rtl/f14_adapter_align_wide.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将 CHW U8 的 64×11×14 adapter 特征，以最近邻方式对齐为 64×21×28，在线生成字节地址及 packed 地址。
// 每源像素沿 x 复制两次，y 使用 floor(y/2)，最后一行仅复制一次；读回后在输出背压期间保持写地址和数据。
//
// Dependencies: 外部单拍字节 BRAM 接口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 复位低至少两拍并共同清空在途响应；缺响应按原契约报错，job 初始化计数，不保存整幅特征图。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 adapter nearest alignment
// Module Name: f14_adapter_align
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: CHW U8 64x11x14 A[0,9856) to 64x21x28 B[0,37632).
// Dependencies: External official one-cycle byte BRAM; no local feature array.
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

// 每个源像素沿x复制两次，y使用floor(y/2)，最后一个源行只复制一次。
// 复位低至少两拍，丢弃在途事务但保持地址操作数；新job初始化全部计数。
// 读响应固定一拍，缺响应报错并只完成一次；写背压时地址和数据保持。
module f14_adapter_align
(
	input wire i_clk,
	input wire i_rst_n,
	input wire i_job_valid,
	output wire o_job_ready,
	output wire o_job_done,
	output wire o_job_error,
	output wire o_busy,
	output wire o_rd_en,
	output wire [17:0] o_rd_addr,
	input wire i_rd_valid,
	input wire [7:0] i_rd_data,
	output wire o_wr_valid,
	input wire i_wr_ready,
	output wire [17:0] o_wr_addr,
	output wire [17:0] o_wr_packed_addr,
	output wire [7:0] o_wr_data
);
	localparam [2:0] ST_IDLE = 3'd0;
	localparam [2:0] ST_PREP = 3'd1;
	localparam [2:0] ST_READ = 3'd2;
	localparam [2:0] ST_WAIT = 3'd3;
	localparam [2:0] ST_FIRST = 3'd4;
	localparam [2:0] ST_SECOND = 3'd5;
	localparam [2:0] ST_DONE = 3'd6;
	reg [2:0] state_current;
	reg [2:0] state_next;
	reg [5:0] channel_q;
	reg [4:0] row_q;
	reg [3:0] column_q;
	reg [17:0] channel_base_q;
	reg [17:0] row_base_q;
	reg [17:0] rd_addr_q;
	reg [17:0] wr_addr_q;
	reg [17:0] packed_addr_q;
	reg [7:0] data_q;
	reg error_q;
	wire job_fire;
	wire last_pixel;

	assign o_job_ready = i_rst_n && (state_current == ST_IDLE);
	assign o_job_done = i_rst_n && (state_current == ST_DONE);
	assign o_job_error = error_q;
	assign o_busy = i_rst_n && (state_current != ST_IDLE);
	assign o_rd_en = i_rst_n && (state_current == ST_READ);
	assign o_rd_addr = rd_addr_q;
	assign o_wr_valid = i_rst_n && ((state_current == ST_FIRST) || (state_current == ST_SECOND));
	assign o_wr_addr = wr_addr_q;
	assign o_wr_packed_addr = packed_addr_q;
	assign o_wr_data = data_q;
	assign job_fire = i_job_valid && o_job_ready;
	assign last_pixel = (channel_q == 6'd63) && (row_q == 5'd20) && (column_q == 4'd13);

	always@(*)begin
		state_next = state_current;
		case(state_current)
			ST_IDLE:begin
				if(job_fire)begin state_next = ST_PREP; end
			end
			ST_PREP:begin state_next = ST_READ; end
			ST_READ:begin state_next = ST_WAIT; end
			ST_WAIT:begin
				state_next = i_rd_valid ? ST_FIRST : ST_DONE;
			end
			ST_FIRST:begin
				if(i_wr_ready)begin state_next = ST_SECOND; end
			end
			ST_SECOND:begin
				if(i_wr_ready)begin state_next = last_pixel ? ST_DONE : ST_PREP; end
			end
			ST_DONE:begin state_next = ST_IDLE; end
			default:begin state_next = ST_IDLE; end
		endcase
	end

	always@(posedge i_clk)begin
		if(!i_rst_n)begin
			state_current <= ST_IDLE;
			error_q <= 1'b0;
		end else begin
			state_current <= state_next;
			if(job_fire)begin error_q <= 1'b0; end
			if((state_current == ST_WAIT) && !i_rd_valid)begin error_q <= 1'b1; end
		end
	end

	// 数据/地址不在复位时跳变，避免禁用的BRAM端口发生无条件hold检查错误。
	always@(posedge i_clk)begin
		if(i_rst_n)begin
			if(job_fire)begin
				channel_q <= 6'd0;
				row_q <= 5'd0;
				column_q <= 4'd0;
				channel_base_q <= 18'd0;
				row_base_q <= 18'd0;
				wr_addr_q <= 18'd0;
			end
			if(state_current == ST_PREP)begin
				rd_addr_q <= row_base_q + {14'd0,column_q};
				packed_addr_q <= ((row_q * 18'd112 + column_q * 18'd8 + channel_q[5:3]) << 4) + channel_q[2:0];
			end
			if((state_current == ST_WAIT) && i_rd_valid)begin data_q <= i_rd_data; end
			if(o_wr_valid && i_wr_ready)begin
				wr_addr_q <= wr_addr_q + 18'd1;
				packed_addr_q <= packed_addr_q + 18'd8;
			end
			if((state_current == ST_SECOND) && i_wr_ready)begin
				if(column_q != 4'd13)begin
					column_q <= column_q + 4'd1;
				end else begin
					column_q <= 4'd0;
					if(row_q == 5'd20)begin
						row_q <= 5'd0;
						channel_q <= channel_q + 6'd1;
						channel_base_q <= channel_base_q + 18'd154;
						row_base_q <= channel_base_q + 18'd154;
					end else begin
						row_q <= row_q + 5'd1;
						if(row_q[0])begin row_base_q <= row_base_q + 18'd14; end
					end
				end
			end
		end
	end
endmodule
`default_nettype wire
