//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-09-10
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 官方 256-bit 异步 FIFO 的网络/视频源域到 DDR 域封装，prepare 或 abort 清空两端 FIFO 指针。
// Module Name: c6_net_to_ddr_fifo_wrapper
// File Name: source/memory/c6_net_to_ddr_fifo_wrapper.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 官方 256-bit 异步 FIFO 的网络/视频源域到 DDR 域封装，prepare 或 abort 清空两端 FIFO 指针。
// DDR 读侧提前预取一字并保持 show-ahead 输出，word_pop 时交付稳定数据；计数与可见预取字须配套解释。
//
// Dependencies: c6_net_to_ddr_fifo 官方 FIFO IP。
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
// Create Date: 2026-09-10
// Design Name: Ethernet_HDMI
// Module Name: c6_net_to_ddr_fifo_wrapper
// Project Name: Ethernet_HDMI
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: C6 PGV1 网络字节域到 DDR 用户时钟域的官方异步 FIFO 封装。
//              prepare/abort 会清空两端 FIFO 指针；读端预取一拍，使 DDR
//              写调度器在 word_pop 时始终取得稳定的 256-bit 数据。
//
// Dependencies: ipcore/c6_net_to_ddr_fifo/c6_net_to_ddr_fifo.idf
//
// Revision:
// Revision 0.01 - File Created
//
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps

module c6_net_to_ddr_fifo_wrapper(
	input i_net_clk,
	input i_net_rst_n,
	input i_frame_prepare,
	input i_frame_abort,
	input i_word_valid,
	input [255:0] i_word_data,
	output o_word_ready,
	input i_ddr_clk,
	input i_ddr_rst_n,
	input i_word_pop,
	output [255:0] o_word_data,
	output o_word_empty,
	output [15:0] o_wr_water_level,
	output [15:0] o_rd_water_level
);
	//---------------寄存器信号---------------//
	reg [1:0] fifo_wr_rst_sync;
	reg fifo_clear_toggle;
	// 保留两个同步级的独立实现与层次名称，供 FDC 仅豁免异步输入级。
	reg [1:0] fifo_clear_sync /* synthesis syn_preserve = 1 */;
	reg fifo_clear_seen;
	reg [1:0] fifo_wr_release_sync;
	reg [1:0] fifo_rd_rst_sync;
	reg fifo_rd_pending;
	reg word_valid;
	reg [255:0] word_data;

	//---------------其他信号---------------//
	wire fifo_full;
	wire fifo_wr_en;
	wire fifo_wr_rst;
	wire fifo_rd_rst;
	wire fifo_rd_empty;
	wire [255:0] fifo_rd_data;
	wire fifo_rd_en;
	wire fifo_clear_changed;
	wire [9:0] fifo_wr_water_level;
	wire [9:0] fifo_rd_water_level;

	//-------------其他信号连线-------------//
	assign fifo_wr_rst = fifo_wr_rst_sync[1];
	assign fifo_rd_rst = fifo_rd_rst_sync[1];
	assign fifo_clear_changed = fifo_clear_sync[1] ^ fifo_clear_seen;
	assign fifo_wr_en = i_word_valid && o_word_ready;
	assign fifo_rd_en = !fifo_rd_rst && !fifo_rd_empty && !fifo_rd_pending &&
					 (!word_valid || i_word_pop);

	//-------------输出信号连线-------------//
	assign o_word_ready = !fifo_wr_rst && !fifo_full;
	assign o_word_data = word_data;
	assign o_word_empty = fifo_rd_rst || !word_valid;
	assign o_wr_water_level = {6'd0, fifo_wr_water_level};
	assign o_rd_water_level = {6'd0, fifo_rd_water_level};

	// 每次开始接收或作废当前帧，写端立即复位；异步 FIFO 仅在复位释放后接收 payload。
	always @(posedge i_net_clk or negedge i_net_rst_n) begin
		if (!i_net_rst_n) begin
			fifo_wr_rst_sync <= 2'b11;
			fifo_clear_toggle <= 1'b0;
		end else begin
			if (i_frame_prepare || i_frame_abort) begin
				fifo_wr_rst_sync <= 2'b11;
				fifo_clear_toggle <= ~fifo_clear_toggle;
			end else begin
				fifo_wr_rst_sync <= {fifo_wr_rst_sync[0], 1'b0};
			end
		end
	end

	// 读复位先等待写端释放，再在 DDR 域同步释放；清空事件会重新拉起读复位。
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if (!i_ddr_rst_n) begin
			fifo_clear_sync <= 2'b00;
			fifo_clear_seen <= 1'b0;
			fifo_wr_release_sync <= 2'b00;
			fifo_rd_rst_sync <= 2'b11;
		end else begin
			fifo_clear_sync <= {fifo_clear_sync[0], fifo_clear_toggle};
			fifo_wr_release_sync <= {fifo_wr_release_sync[0], !fifo_wr_rst};
			if (fifo_clear_changed) begin
				fifo_clear_seen <= fifo_clear_sync[1];
				fifo_rd_rst_sync <= 2'b11;
			end else if (!fifo_wr_release_sync[1]) begin
				fifo_rd_rst_sync <= 2'b11;
			end else begin
				fifo_rd_rst_sync <= {fifo_rd_rst_sync[0], 1'b0};
			end
		end
	end

	// 官方 IP 的 rd_data 在 rd_en 后一个 DDR 时钟周期稳定；本寄存器提供 show-ahead。
	always @(posedge i_ddr_clk or negedge i_ddr_rst_n) begin
		if (!i_ddr_rst_n) begin
			fifo_rd_pending <= 1'b0;
			word_valid <= 1'b0;
			word_data <= 256'd0;
		end else if (fifo_rd_rst) begin
			fifo_rd_pending <= 1'b0;
			word_valid <= 1'b0;
			word_data <= 256'd0;
		end else begin
			fifo_rd_pending <= fifo_rd_en;
			if (fifo_rd_pending) begin
				word_data <= fifo_rd_data;
				word_valid <= 1'b1;
			end else if (i_word_pop && word_valid) begin
				word_valid <= 1'b0;
			end
		end
	end

	// PDS 2022.2-SP6.4 在当前 PG2L100H 工程生成并登记的 256-bit 异步 FIFO。
	c6_net_to_ddr_fifo u_c6_net_to_ddr_fifo(
		.wr_clk(i_net_clk),
		.wr_rst(fifo_wr_rst),
		.wr_en(fifo_wr_en),
		.wr_data(i_word_data),
		.wr_full(fifo_full),
		.almost_full(),
		.rd_clk(i_ddr_clk),
		.rd_rst(fifo_rd_rst),
		.rd_en(fifo_rd_en),
		.rd_data(fifo_rd_data),
		.rd_empty(fifo_rd_empty),
		.rd_water_level(fifo_rd_water_level),
		.wr_water_level(fifo_wr_water_level),
		.almost_empty()
	);
endmodule
