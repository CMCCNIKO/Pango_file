//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将工程 32 位数据与四位 K 契约适配到官方 HSST CH2 发送 IP，导出 fabric 时钟及真实 PLL/通道/看门狗状态。
// Module Name: host_hsst_tx_adapter
// File Name: host/rtl/fiber/host_hsst_tx_adapter.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将工程 32 位数据与四位 K 契约适配到官方 HSST CH2 发送 IP，导出 fabric 时钟及真实 PLL/通道/看门狗状态。
// 仅在 IP 与数据有效时透传，其他情况输出 IDLE；按既有官方接口回环 fabric 时钟，并保持 SFP TX_DISABLE 为低。
//
// Dependencies: hsst_test 官方生成 IP、fiber_video_v1_defs.vh；HOST_HSST_SIM_STUB 仅用于边界仿真，不能代替真实硬核。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "fiber_video_v1_defs.vh"
// Boundary wrapper for the regenerated PG2L200H HSST TX IP.
// The generated module is intentionally kept vendor-owned in ipcore/; this
// file only adapts the project stream contract to its CH2 TX-only interface.
module host_hsst_tx_adapter (
    input wire i_tx_clk,
    input wire i_rst_n,
    input wire i_ip_ready,
    input wire i_refckn,
    input wire i_refckp,
    input wire i_tx_valid,
    input wire [31:0] i_tx_data,
    input wire [3:0] i_tx_k,
    output wire o_tx_ready,
    output wire o_fabric_clk,
    output wire [31:0] o_fabric_data,
    output wire [3:0] o_fabric_k,
    output wire o_hsst_txp,
    output wire o_hsst_txn,
    output wire o_tx_disable,
    // Real TX status exported for the management-domain diagnostic monitor.
    // These are direct taps of already-existing internal signals; no logic change.
    output wire o_tx_pll_done,
    output wire o_tx_pll_lock,
    output wire o_tx_lane_done,
    output wire [1:0] o_tx_watchdog_status
);

    wire [1:0] tx_watchdog_status;
    wire tx_pll_done;
    wire tx_lane_done;
    wire tx_pll_lock;
    wire tx_fabric_clk;
    wire tx_clk_from_core;
    wire [31:0] tx_data_to_ip;
    wire [3:0] tx_k_to_ip;
    wire o_ip_txp;
    wire o_ip_txn;

    // The Pango example connects the CH2 fabric clock back to the generated
    // core-clock input.  Keep that exact loop here; it is required even while
    // the host stream is held in IDLE_ONLY by i_ip_ready.
    assign tx_clk_from_core = tx_fabric_clk;
    assign tx_data_to_ip = (i_ip_ready && i_tx_valid) ? i_tx_data : `FVV1_IDLE;
    assign tx_k_to_ip = (i_ip_ready && i_tx_valid) ? i_tx_k : `FVV1_K_CTRL;
    assign o_fabric_data = tx_data_to_ip;
    assign o_fabric_k = tx_k_to_ip;
    assign o_fabric_clk = tx_fabric_clk;
    assign o_tx_ready = i_ip_ready && i_rst_n && tx_pll_done && tx_lane_done && tx_pll_lock;
    assign o_hsst_txp = o_ip_txp;
    assign o_hsst_txn = o_ip_txn;
    assign o_tx_pll_done = tx_pll_done;
    assign o_tx_pll_lock = tx_pll_lock;
    assign o_tx_lane_done = tx_lane_done;
    assign o_tx_watchdog_status = tx_watchdog_status;
    // PG2L200H SFP reference design drives TX_DISABLE low (V5 is the first
    // SFP control pin), matching the SFP active-high disable convention.
    assign o_tx_disable = 1'b0;

`ifdef HOST_HSST_SIM_STUB
    // The boundary suite does not compile vendor hard macros.  This stub
    // preserves the stream/IDLE contract; PDS builds use the real instance.
    assign tx_pll_done = 1'b1;
    assign tx_lane_done = 1'b1;
    assign tx_pll_lock = 1'b1;
    assign tx_watchdog_status = 2'b00;
    assign tx_fabric_clk = i_tx_clk;
    assign o_ip_txp = 1'b0;
    assign o_ip_txn = 1'b0;
`else
    hsst_test u_hsst_test (
        .i_free_clk          (i_tx_clk),
        .i_p_refckn_1        (i_refckn),
        .i_p_refckp_1        (i_refckp),
        .i_wtchdg_clr_0      (1'b0),
        .o_wtchdg_st_0       (tx_watchdog_status),
        .o_pll_done_0        (tx_pll_done),
        .o_txlane_done_2     (tx_lane_done),
        .o_p_clk2core_tx_2   (tx_fabric_clk),
        .i_p_tx2_clk_fr_core (tx_clk_from_core),
        .o_p_pll_lock_0      (tx_pll_lock),
        .o_p_l2txn           (o_ip_txn),
        .o_p_l2txp           (o_ip_txp),
        .i_txd_2             (tx_data_to_ip),
        .i_tdispsel_2        (4'b0000),
        .i_tdispctrl_2       (4'b0000),
        .i_txk_2             (tx_k_to_ip),
        .i_pll_rst_0         (~i_rst_n)
    );
`endif
endmodule
