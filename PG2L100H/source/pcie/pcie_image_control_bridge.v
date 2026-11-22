//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-23
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 将 BAR1 ISP/算法控制寄存器与两条独立稳定载荷事务邮箱连接，跨越 PCIe 域和算法控制域。
// Module Name: pcie_image_control_bridge
// File Name: source/pcie/pcie_image_control_bridge.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 将 BAR1 ISP/算法控制寄存器与两条独立稳定载荷事务邮箱连接，跨越 PCIe 域和算法控制域。
// 链路/本域复位只清空本控制会话；算法接收端需排空旧像素事务后重新开放 READY。
//
// Dependencies: pcie_image_control_regs、pcie_image_control_cdc。
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
// Create Date: 2026-10-23
// Design Name: Dual transport image control
// Module Name: pcie_image_control_bridge
// Project Name: PG2L100H_AVM_ISP_DUAL_CONTROL
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2022.2-SP6.4
// Description: BAR1 CSR and two independent stable transaction mailboxes.
//              PCIe link/reset and algorithm reset flush this session only.
//              The algorithm owner drains old pixel transactions before READY.
// Dependencies: pcie_image_control_regs, pcie_image_control_cdc
// Revision: 0.01 - File Created
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module pcie_image_control_bridge(
	input i_pcie_clk,
	input i_pcie_rst_n,
	input i_link_up,
	input i_control_clk,
	input i_control_rst_n,
	input i_wr_en,
	input [11:0] i_wr_addr,
	input [31:0] i_wr_data,
	input [3:0] i_wr_strb,
	input [11:0] i_rd_addr,
	output [127:0] o_rd_window_data,
	output o_isp_ext_valid,
	output o_isp_ext_query,
	output [519:0] o_isp_ext_config,
	input i_isp_ext_ready,
	input i_isp_ext_done,
	input [7:0] i_isp_ext_status,
	input [519:0] i_isp_ext_snapshot,
	input [15:0] i_isp_ext_epoch,
	input [1:0] i_isp_ext_source,
	output o_algo_ext_valid,
	output o_algo_ext_query,
	output [63:0] o_algo_ext_config,
	input i_algo_ext_ready,
	input i_algo_ext_done,
	input [7:0] i_algo_ext_status,
	input [63:0] i_algo_ext_snapshot,
	input [15:0] i_algo_ext_epoch,
	input [1:0] i_algo_ext_source
);
	wire isp_valid;
	wire isp_query;
	wire [519:0] isp_config;
	wire isp_ready;
	wire isp_done;
	wire [7:0] isp_status;
	wire [519:0] isp_snapshot;
	wire [15:0] isp_epoch;
	wire [1:0] isp_source;
	wire algo_valid;
	wire algo_query;
	wire [63:0] algo_config;
	wire algo_ready;
	wire algo_done;
	wire [7:0] algo_status;
	wire [63:0] algo_snapshot;
	wire [15:0] algo_epoch;
	wire [1:0] algo_source;

	wire session_async_rst_n;
	(* syn_preserve = 1 *) reg [1:0] reg_pcie_reset_sync;
	(* syn_preserve = 1 *) reg [1:0] reg_control_reset_sync;
	wire session_pcie_rst_n;
	wire session_control_rst_n;
	assign session_async_rst_n = i_pcie_rst_n && i_link_up && i_control_rst_n;
	assign session_pcie_rst_n = reg_pcie_reset_sync[1];
	assign session_control_rst_n = reg_control_reset_sync[1];
	// 同时异步断言，两域各两拍同步释放；不复位视频/以太网算法控制。
	always @(posedge i_pcie_clk or negedge session_async_rst_n) begin
		if (!session_async_rst_n) begin
			reg_pcie_reset_sync <= 2'b00;
		end else begin
			reg_pcie_reset_sync <= {reg_pcie_reset_sync[0], 1'b1};
		end
	end
	always @(posedge i_control_clk or negedge session_async_rst_n) begin
		if (!session_async_rst_n) begin
			reg_control_reset_sync <= 2'b00;
		end else begin
			reg_control_reset_sync <= {reg_control_reset_sync[0], 1'b1};
		end
	end

	pcie_image_control_regs u_image_regs(
		.i_clk(i_pcie_clk),
		.i_rst_n(session_pcie_rst_n),
		.i_wr_en(i_wr_en),
		.i_wr_addr(i_wr_addr),
		.i_wr_data(i_wr_data),
		.i_wr_strb(i_wr_strb),
		.i_rd_addr(i_rd_addr),
		.o_rd_window_data(o_rd_window_data),
		.o_isp_valid(isp_valid),
		.o_isp_query(isp_query),
		.o_isp_config(isp_config),
		.i_isp_ready(isp_ready),
		.i_isp_done(isp_done),
		.i_isp_status(isp_status),
		.i_isp_snapshot(isp_snapshot),
		.i_isp_epoch(isp_epoch),
		.i_isp_source(isp_source),
		.o_algo_valid(algo_valid),
		.o_algo_query(algo_query),
		.o_algo_config(algo_config),
		.i_algo_ready(algo_ready),
		.i_algo_done(algo_done),
		.i_algo_status(algo_status),
		.i_algo_snapshot(algo_snapshot),
		.i_algo_epoch(algo_epoch),
		.i_algo_source(algo_source)
	);

	pcie_image_control_cdc #(.C_WIDTH(520)) u_isp_mailbox(
		.i_src_clk(i_pcie_clk),
		.i_src_rst_n(session_pcie_rst_n),
		.i_dst_clk(i_control_clk),
		.i_dst_rst_n(session_control_rst_n),
		.i_src_valid(isp_valid),
		.i_src_query(isp_query),
		.i_src_config(isp_config),
		.o_src_ready(isp_ready),
		.o_src_done(isp_done),
		.o_src_status(isp_status),
		.o_src_snapshot(isp_snapshot),
		.o_src_epoch(isp_epoch),
		.o_src_source(isp_source),
		.o_dst_valid(o_isp_ext_valid),
		.o_dst_query(o_isp_ext_query),
		.o_dst_config(o_isp_ext_config),
		.i_dst_ready(i_isp_ext_ready),
		.i_dst_done(i_isp_ext_done),
		.i_dst_status(i_isp_ext_status),
		.i_dst_snapshot(i_isp_ext_snapshot),
		.i_dst_epoch(i_isp_ext_epoch),
		.i_dst_source(i_isp_ext_source)
	);

	pcie_image_control_cdc #(.C_WIDTH(64)) u_algo_mailbox(
		.i_src_clk(i_pcie_clk),
		.i_src_rst_n(session_pcie_rst_n),
		.i_dst_clk(i_control_clk),
		.i_dst_rst_n(session_control_rst_n),
		.i_src_valid(algo_valid),
		.i_src_query(algo_query),
		.i_src_config(algo_config),
		.o_src_ready(algo_ready),
		.o_src_done(algo_done),
		.o_src_status(algo_status),
		.o_src_snapshot(algo_snapshot),
		.o_src_epoch(algo_epoch),
		.o_src_source(algo_source),
		.o_dst_valid(o_algo_ext_valid),
		.o_dst_query(o_algo_ext_query),
		.o_dst_config(o_algo_ext_config),
		.i_dst_ready(i_algo_ext_ready),
		.i_dst_done(i_algo_ext_done),
		.i_dst_status(i_algo_ext_status),
		.i_dst_snapshot(i_algo_ext_snapshot),
		.i_dst_epoch(i_algo_ext_epoch),
		.i_dst_source(i_algo_ext_source)
	);
endmodule
