//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-06
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 封装官方 ASYN_FIFO 128×4096，在 HDMI 像素时钟与 PCIe 时钟间传输视频字，按真实 clocked-read 延后一拍采样读数据。
// Module Name: pcie_video_async_fifo_wrapper
// File Name: source/pcie/pcie_video_async_fifo_wrapper.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 封装官方 ASYN_FIFO 128×4096，在 HDMI 像素时钟与 PCIe 时钟间传输视频字，按真实 clocked-read 延后一拍采样读数据。
// 双域共同异步断言复位并独立同步释放，flush 使用 toggle 握手；支持同沿消费暂存字并发出下一次读请求。
//
// Dependencies: 官方 ipcore/ASYN_FIFO/ASYN_FIFO.v 及其 rtl 依赖。
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
// Create Date: 2026-10-06
// Design Name: pcie_video_async_fifo_wrapper
// Module Name: pcie_video_async_fifo_wrapper
// Project Name: FPGA_PCIE_RK
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: 当前工程 ASYN_FIFO 128x4096 的双时钟封装；读数据严格按生成 IP 的
//              clocked-read 时序延后一拍采样。flush 以 toggle 握手，双域异步置位、同步释放。
// Dependencies: ipcore/ASYN_FIFO/ASYN_FIFO.v 及其 rtl 依赖。
// Revision: 0.03 - Joint reset assertion with independent synchronous local release.
// Revision: 0.04 - 2026-10-08 Consume held word and issue next read on the same edge.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module pcie_video_async_fifo_wrapper(
	input i_wr_clk,
	input i_wr_rst_n,
	input i_wr_en,
	input [127:0] i_wr_data,
	output o_wr_full,
	output o_wr_almost_full,
	output [12:0] o_wr_water_level,
	input i_rd_clk,
	input i_rd_rst_n,
	input i_rd_ready,
	output o_rd_valid,
	output [127:0] o_rd_data,
	output o_rd_empty,
	output o_rd_almost_empty,
	output [12:0] o_rd_water_level,
	input i_flush_req_toggle,
	output o_flush_ack_toggle,
	output o_flush_busy
);
	reg [1:0] wr_local_reset_sync;
	reg [1:0] rd_local_reset_sync;
	reg [1:0] wr_flush_req_sync;
	reg wr_flush_req_seen;
	reg [2:0] wr_flush_count;
	reg wr_flush_active;
	reg wr_flush_done_toggle;
	reg [1:0] wr_reset_sync;
	reg [1:0] rd_flush_req_sync;
	reg rd_flush_req_seen;
	reg [2:0] rd_flush_count;
	reg rd_flush_active;
	reg rd_flush_ack_toggle;
	reg [1:0] rd_reset_sync;
	reg rd_pending;
	reg rd_data_valid;
	reg [127:0] rd_data_hold;
	reg [1:0] wr_done_sync;
	wire fifo_wr_rst;
	wire fifo_rd_rst;
	wire fifo_wr_full;
	wire fifo_wr_almost_full;
	wire [12:0] fifo_wr_level;
	wire fifo_rd_empty;
	wire fifo_rd_almost_empty;
	wire [12:0] fifo_rd_level;
	wire [127:0] fifo_rd_data;
	wire fifo_wr_en;
	wire fifo_rd_en;
	wire rd_flush_seen_change;
	wire rd_side_ready;
	wire rd_issue;
	wire common_rst_n;
	wire wr_local_rst_n;
	wire rd_local_rst_n;
	// 任一域复位均取消整条FIFO事务，避免指针只清一侧。
	assign common_rst_n = i_wr_rst_n && i_rd_rst_n;
	assign wr_local_rst_n = wr_local_reset_sync[1];
	assign rd_local_rst_n = rd_local_reset_sync[1];

	assign fifo_wr_rst = !wr_local_rst_n || (wr_flush_count != 3'd0) || !wr_reset_sync[1];
	assign fifo_rd_rst = !rd_local_rst_n || (rd_flush_count != 3'd0) || !rd_reset_sync[1];
	assign fifo_wr_en = i_wr_en && !fifo_wr_full && i_wr_rst_n &&
		!fifo_wr_rst && !wr_flush_active && (wr_flush_req_sync[1] == wr_flush_req_seen);
	assign rd_flush_seen_change = (rd_flush_req_sync[1] != rd_flush_req_seen);
	assign rd_side_ready = i_rd_rst_n && (rd_flush_count == 3'd0) &&
		rd_reset_sync[1] && !rd_flush_active && !rd_flush_seen_change && rd_local_rst_n &&
		(i_flush_req_toggle == rd_flush_req_seen);
	// 生成 IP 在 rd_en 采样沿更新读地址，rd_data 随后才改变；rd_pending
	// 保持一整拍，在下一沿采样 fifo_rd_data，避免把旧字当作新字。
	// 当前字在本沿被消费时，保持寄存器已为下一拍的读返回预留空间。
	// 仍只允许一个 pending；下拍 ready 撤销也不影响返回数据的保持。
	// 同沿消费并发起读，将持续供给间隔从三拍缩为两拍，不改变 IP 读延迟。
	assign rd_issue = rd_side_ready && (!rd_data_valid || i_rd_ready) && !rd_pending && !fifo_rd_empty;
	assign fifo_rd_en = rd_issue;
	assign o_wr_full = fifo_wr_full || fifo_wr_rst || wr_flush_active ||
		(wr_flush_req_sync[1] != wr_flush_req_seen);
	assign o_wr_almost_full = fifo_wr_almost_full || o_wr_full;
	assign o_wr_water_level = fifo_wr_level;
	assign o_rd_empty = fifo_rd_empty;
	assign o_rd_almost_empty = fifo_rd_almost_empty;
	assign o_rd_water_level = fifo_rd_level;
	assign o_rd_valid = rd_data_valid && rd_side_ready;
	assign o_rd_data = rd_data_hold;
	assign o_flush_ack_toggle = rd_flush_ack_toggle;
	assign o_flush_busy = !rd_local_rst_n || !rd_reset_sync[1] ||
		(i_flush_req_toggle != rd_flush_ack_toggle) || rd_flush_active || (rd_flush_count != 3'd0) ||
		(wr_done_sync[1] != rd_flush_req_seen);

	ASYN_FIFO u_ASYN_FIFO(
		.wr_clk(i_wr_clk), .wr_rst(fifo_wr_rst), .wr_en(fifo_wr_en), .wr_data(i_wr_data),
		.wr_full(fifo_wr_full), .wr_water_level(fifo_wr_level), .almost_full(fifo_wr_almost_full),
		.rd_clk(i_rd_clk), .rd_rst(fifo_rd_rst), .rd_en(fifo_rd_en), .rd_data(fifo_rd_data),
		.rd_empty(fifo_rd_empty), .rd_water_level(fifo_rd_level), .almost_empty(fifo_rd_almost_empty)
	);

	// 独立的释放同步链仅由原始联合复位置零，禁止由自身输出复位自身。
	// 控制寄存器和 FIFO reset 延长链均在本域同步释放以后才工作。
	always @(posedge i_wr_clk or negedge common_rst_n) begin
		if (!common_rst_n) wr_local_reset_sync <= 2'b00;
		else wr_local_reset_sync <= {wr_local_reset_sync[0], 1'b1};
	end
	always @(posedge i_rd_clk or negedge common_rst_n) begin
		if (!common_rst_n) rd_local_reset_sync <= 2'b00;
		else rd_local_reset_sync <= {rd_local_reset_sync[0], 1'b1};
	end

	// 写域：先异步复位 FIFO，再保持三个写时钟，最后用两级寄存器释放复位。
	always @(posedge i_wr_clk or negedge wr_local_rst_n) begin
		if (!wr_local_rst_n) begin
			wr_flush_req_sync <= 2'b00;
			wr_flush_req_seen <= 1'b0;
			wr_flush_count <= 3'd0;
			wr_flush_active <= 1'b0;
			wr_flush_done_toggle <= 1'b0;
			wr_reset_sync <= 2'b00;
		end else begin
			wr_flush_req_sync <= {wr_flush_req_sync[0], i_flush_req_toggle};
			if (wr_flush_req_sync[1] != wr_flush_req_seen) begin
				wr_flush_req_seen <= wr_flush_req_sync[1];
				wr_flush_count <= 3'd3;
				wr_flush_active <= 1'b1;
				wr_reset_sync <= 2'b00;
			end else if (wr_flush_count != 3'd0) begin
				wr_flush_count <= wr_flush_count - 1'b1;
			end else begin
				wr_reset_sync <= {wr_reset_sync[0], 1'b1};
				if (wr_flush_active && wr_reset_sync[1]) begin
					wr_flush_done_toggle <= wr_flush_req_seen;
					wr_flush_active <= 1'b0;
				end
			end
		end
	end

	// 读域：请求沿到达前即禁止新 rd_en；flush 期间 valid/pending 清零。
	always @(posedge i_rd_clk or negedge rd_local_rst_n) begin
		if (!rd_local_rst_n) begin
			rd_flush_req_sync <= 2'b00;
			rd_flush_req_seen <= 1'b0;
			rd_flush_count <= 3'd0;
			rd_flush_active <= 1'b0;
			rd_flush_ack_toggle <= 1'b0;
			rd_reset_sync <= 2'b00;
			rd_pending <= 1'b0;
			rd_data_valid <= 1'b0;
			rd_data_hold <= 128'd0;
			wr_done_sync <= 2'b00;
		end else begin
			rd_flush_req_sync <= {rd_flush_req_sync[0], i_flush_req_toggle};
			wr_done_sync <= {wr_done_sync[0], wr_flush_done_toggle};
			if (rd_flush_req_sync[1] != rd_flush_req_seen) begin
				rd_flush_req_seen <= rd_flush_req_sync[1];
				rd_flush_count <= 3'd3;
				rd_flush_active <= 1'b1;
				rd_reset_sync <= 2'b00;
				rd_pending <= 1'b0;
				rd_data_valid <= 1'b0;
				rd_data_hold <= 128'd0;
			end else if (rd_flush_count != 3'd0) begin
				rd_flush_count <= rd_flush_count - 1'b1;
			end else begin
				rd_reset_sync <= {rd_reset_sync[0], 1'b1};
				if (rd_flush_active && rd_reset_sync[1] &&
					(wr_done_sync[1] == rd_flush_req_seen)) begin
					rd_flush_ack_toggle <= rd_flush_req_seen;
					rd_flush_active <= 1'b0;
				end
				if (rd_side_ready) begin
					if (rd_data_valid && i_rd_ready) rd_data_valid <= 1'b0;
					if (rd_pending) begin
						rd_data_hold <= fifo_rd_data;
						rd_data_valid <= 1'b1;
					rd_pending <= 1'b0;
				end else if (rd_issue) begin
						rd_pending <= 1'b1;
				end
				end
			end
		end
	end
endmodule


