//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明
// Update Date: 2026-11-01（文件用途说明整理）
// Design Name: 当前工程唯一 DDR3 HMIC_S v1.12 包装层，接入差分参考时钟、DDR3 物理引脚和 256-bit 应用读写口。
// Module Name: ddr3_subsystem
// File Name: source/memory/ddr3_subsystem.v
// Project Name: PG2L100H_PRJ / portable
// Target Devices: Pango Logos2 PG2L100H-6FBG676
// Tool Versions: PDS 2025.2 build 211867
// Header Version: 1.00（文件说明版本；功能版本保留原修订）
// Description:
// 当前工程唯一 DDR3 HMIC_S v1.12 包装层，接入差分参考时钟、DDR3 物理引脚和 256-bit 应用读写口。
// 保留生成 IP 的复位/初始化契约；实例名 I_ips_ddr_top 受官方 DDR 约束引用，移植时须与约束层次保持一致。
//
// Dependencies: ddr3_hmic_s 官方 HMIC_S 1.12 IP、GTP_INBUFDS 及对应生成约束。
//
// Revision:
// 2026-11-01 / Header 1.00 - 补充本文件具体功能、依赖、工程版本；保留功能逻辑。
// 原创建日期、功能修订、编译指令和版权保留在下方。
// Additional Comments:
// 当前用途见本说明；下方原始注释作为历史记录保留。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
// Sole project wrapper for the current PDS-registered HMIC_S 1.12 instance.
// The name I_ips_ddr_top is retained because the generated DDR constraints
// reference it; all application logic uses only this wrapper's narrow port.
module ddr3_subsystem(
	input ref_clk_p,
	input ref_clk_n,
	input i_rst_n,
	output o_core_clk,
	output o_init_done,
	output o_init_fail,
	input [27:0] i_axi_awaddr,
	input i_axi_awuser_ap,
	input [3:0] i_axi_awuser_id,
	input [3:0] i_axi_awlen,
	output o_axi_awready,
	input i_axi_awvalid,
	input [255:0] i_axi_wdata,
	input [31:0] i_axi_wstrb,
	output o_axi_wready,
	input [27:0] i_axi_araddr,
	input i_axi_aruser_ap,
	input [3:0] i_axi_aruser_id,
	input [3:0] i_axi_arlen,
	output o_axi_arready,
	input i_axi_arvalid,
	output [255:0] o_axi_rdata,
	output o_axi_rvalid,
	output mem_cs_n,
	output mem_rst_n,
	output mem_ck,
	output mem_ck_n,
	output mem_cke,
	output mem_ras_n,
	output mem_cas_n,
	output mem_we_n,
	output mem_odt,
	output [14:0] mem_a,
	output [2:0] mem_ba,
	inout [3:0] mem_dqs,
	inout [3:0] mem_dqs_n,
	inout [31:0] mem_dq,
	output [3:0] mem_dm
);
	wire ref_clk;
	GTP_INBUFDS u_refclk_inbuf(
		.O(ref_clk),
		.I(ref_clk_p),
		.IB(ref_clk_n)
	);

	ddr3_hmic_s #(
		.MEM_ROW_WIDTH(15),
		.MEM_COLUMN_WIDTH(10),
		.MEM_BANK_WIDTH(3),
		.MEM_DQ_WIDTH(32),
		.MEM_DM_WIDTH(4),
		.MEM_DQS_WIDTH(4),
		.CTRL_ADDR_WIDTH(28),
		.DEVICE_400(0)
	) I_ips_ddr_top(
		.ref_clk(ref_clk),
		.resetn(i_rst_n),
		.core_clk(o_core_clk),
		.pll_lock(),
		.phy_pll_lock(),
		.gpll_lock(),
		.rst_gpll_lock(),
		.ddrphy_cpd_lock(),
		.ddr_init_done(o_init_done),
		.axi_awaddr(i_axi_awaddr),
		.axi_awuser_ap(i_axi_awuser_ap),
		.axi_awuser_id(i_axi_awuser_id),
		.axi_awlen(i_axi_awlen),
		.axi_awready(o_axi_awready),
		.axi_awvalid(i_axi_awvalid),
		.axi_wdata(i_axi_wdata),
		.axi_wstrb(i_axi_wstrb),
		.axi_wready(o_axi_wready),
		.axi_wusero_id(),
		.axi_wusero_last(),
		.axi_araddr(i_axi_araddr),
		.axi_aruser_ap(i_axi_aruser_ap),
		.axi_aruser_id(i_axi_aruser_id),
		.axi_arlen(i_axi_arlen),
		.axi_arready(o_axi_arready),
		.axi_arvalid(i_axi_arvalid),
		.axi_rdata(o_axi_rdata),
		.axi_rid(),
		.axi_rlast(),
		.axi_rvalid(o_axi_rvalid),
		.apb_clk(1'b0),
		.apb_rst_n(1'b0),
		.apb_sel(1'b0),
		.apb_enable(1'b0),
		.apb_addr(8'd0),
		.apb_write(1'b0),
		.apb_ready(),
		.apb_wdata(16'd0),
		.apb_rdata(),
		.mem_cs_n(mem_cs_n),
		.mem_rst_n(mem_rst_n),
		.mem_ck(mem_ck),
		.mem_ck_n(mem_ck_n),
		.mem_cke(mem_cke),
		.mem_ras_n(mem_ras_n),
		.mem_cas_n(mem_cas_n),
		.mem_we_n(mem_we_n),
		.mem_odt(mem_odt),
		.mem_a(mem_a),
		.mem_ba(mem_ba),
		.mem_dqs(mem_dqs),
		.mem_dqs_n(mem_dqs_n),
		.mem_dq(mem_dq),
		.mem_dm(mem_dm),
		.dbg_gate_start(1'b0),
		.dbg_cpd_start(1'b0),
		.dbg_ddrphy_rst_n(1'b1),
		.dbg_gpll_scan_rst(1'b0),
		.samp_position_dyn_adj(1'b0),
		.init_samp_position_even(32'd0),
		.init_samp_position_odd(32'd0),
		.wrcal_position_dyn_adj(1'b0),
		.init_wrcal_position(32'd0),
		.force_read_clk_ctrl(1'b0),
		.init_slip_step(16'd0),
		.init_read_clk_ctrl(12'd0),
		.debug_calib_ctrl(),
		.dbg_slice_state(),
		.debug_data(),
		.dbg_dll_upd_state(),
		.debug_gpll_dps_phase(),
		.dbg_rst_dps_state(),
		.dbg_tran_err_rst_cnt(),
		.dbg_ddrphy_init_fail(o_init_fail),
		.debug_cpd_offset_adj(1'b0),
		.debug_cpd_offset_dir(1'b0),
		.debug_cpd_offset(10'd0),
		.debug_dps_cnt_dir0(),
		.debug_dps_cnt_dir1(),
		.ck_dly_en(1'b1),
		.init_ck_dly_step(8'd0),
		.ck_dly_set_bin(),
		.align_error(),
		.debug_rst_state(),
		.debug_cpd_state()
	);
endmodule
