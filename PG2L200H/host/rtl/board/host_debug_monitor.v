//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 系统管理域诊断模块：同步 HSST PLL/通道就绪状态，检测视频与 fabric 时钟心跳，锁存超时故障并输出可选 LED 状态。
// Module Name: host_debug_monitor
// File Name: host/rtl/board/host_debug_monitor.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 系统管理域诊断模块：同步 HSST PLL/通道就绪状态，检测视频与 fabric 时钟心跳，锁存超时故障并输出可选 LED 状态。
// 在 fabric 域统计实际送出的 IDLE/VSYNC/DATA 及帧事件，将保持稳定的计数快照通过 toggle 同步送回系统域，同时导出模式号与构建标识。
//
// Dependencies: i_sys_clk/i_rst_n 与 i_fabric_clk/i_fabric_rst_n 两个时钟域；异步电平两级同步，多位计数用稳定快照交接。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
// Management-domain diagnostic monitor for the PG2L200H HOST.
//
// The 50 MHz sys_clk domain stays alive even when the HDMI pixel clock or the
// HSST fabric clock stops, so it is the right place to sample TX status and
// detect clock loss. Asynchronous levels cross through 2-FF synchronizers;
// the multi-bit "presented" counters are latched into a stable snapshot in the
// fabric domain and crossed with a toggle handshake, never read directly.
//
// Observable layout:
//   sys_clk domain  : synchronized TX status, fabric/pixel clock alive,
//                     timeout sticky, active_mode, build_id, LED select.
//   fabric_clk domain: presented IDLE/VSYNC/DATA counts and frame event counts.
module host_debug_monitor #(
    parameter integer C_SYS_TIMEOUT_CYCLES  = 250000,  // 5 ms @ 50 MHz
    parameter integer C_FAB_TIMEOUT_CYCLES  = 250000,
    parameter integer C_SNAPSHOT_PERIOD     = 78125,   // ~1 ms @ 78.125 MHz
    parameter integer C_LED_SELECT          = 1        // 0=mode_valid,1=tx_ready,2=fabric_alive
) (
    input  wire        i_sys_clk,
    input  wire        i_rst_n,
    // Raw TX status (fabric-domain async levels from the adapter).
    input  wire        i_tx_pll_done,
    input  wire        i_tx_pll_lock,
    input  wire        i_tx_lane_done,
    input  wire        i_tx_ready,
    input  wire        i_mode_valid,       // pixel-domain level, for LED select
    // Slow heartbeats, each toggling in its own source domain.
    input  wire        i_fabric_heartbeat, // fabric_clk domain
    input  wire        i_pixel_heartbeat,  // pixel_clk domain
    // Fabric-domain clock and the stream actually presented to the HSST IP.
    input  wire        i_fabric_clk,
    input  wire        i_fabric_rst_n,
    input  wire [31:0] i_tx_data,
    input  wire [3:0]  i_tx_k,
    input  wire        i_frame_done,
    input  wire        i_frame_abort,
    input  wire        i_bad_desc,
    // Static build identity.  These are registered in the sys_clk domain and
    // re-exported so the Fabric Inserter can read them out of a bitstream (the
    // mode number and build id embedded at elaboration time).
    input  wire [2:0]  i_active_mode,
    input  wire [7:0]  i_build_id,
    // Synchronized sys_clk-domain status.
    output wire        o_tx_pll_done_sync,
    output wire        o_tx_pll_lock_sync,
    output wire        o_tx_lane_done_sync,
    output wire        o_tx_ready_sync,
    output wire        o_fabric_clock_alive,
    output wire        o_pixel_clock_alive,
    output wire        o_timeout_sticky,
    // Snapshot counters (stable in sys_clk domain).
    output reg  [31:0] o_presented_idle_count,
    output reg  [31:0] o_presented_vsync_count,
    output reg  [31:0] o_presented_data_count,
    output reg  [15:0] o_frame_done_count,
    output reg  [15:0] o_frame_abort_count,
    output reg  [15:0] o_bad_desc_count,
    // Registered build identity (sys_clk domain), exported for debug readout.
    output wire [2:0]  o_active_mode,
    output wire [7:0]  o_build_id,
    // Single constrained LED selection.
    output wire        o_led
);
    localparam FVV1_IDLE  = 32'h0000_00BC;
    localparam FVV1_VSYNC = 32'hFFAA_55BC;
    localparam K_CTRL     = 4'b0001;
    localparam K_DATA     = 4'b0000;

    // ------------------------------------------------------------------
    // sys_clk domain: synchronizers and clock-alive detection.
    // ------------------------------------------------------------------
    reg pll_done_m, pll_done_s;
    reg pll_lock_m, pll_lock_s;
    reg lane_done_m, lane_done_s;
    reg ready_m,    ready_s;
    reg mode_m,     mode_s;
    reg fab_hb_m,   fab_hb_s,   fab_hb_prev;
    reg pix_hb_m,   pix_hb_s,   pix_hb_prev;
    reg fab_alive, pix_alive;
    reg [31:0] fab_timeout_cnt;
    reg [31:0] pix_timeout_cnt;
    reg timeout_sticky;
    reg [2:0]  active_mode_r;
    reg [7:0]  build_id_r;

    assign o_tx_pll_done_sync  = pll_done_s;
    assign o_tx_pll_lock_sync  = pll_lock_s;
    assign o_tx_lane_done_sync = lane_done_s;
    assign o_tx_ready_sync     = ready_s;
    assign o_fabric_clock_alive = fab_alive;
    assign o_pixel_clock_alive  = pix_alive;
    assign o_timeout_sticky     = timeout_sticky;
    assign o_active_mode        = active_mode_r;
    assign o_build_id           = build_id_r;

    always @(posedge i_sys_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            pll_done_m <= 1'b0; pll_done_s <= 1'b0;
            pll_lock_m <= 1'b0; pll_lock_s <= 1'b0;
            lane_done_m <= 1'b0; lane_done_s <= 1'b0;
            ready_m <= 1'b0; ready_s <= 1'b0;
            mode_m <= 1'b0; mode_s <= 1'b0;
            fab_hb_m <= 1'b0; fab_hb_s <= 1'b0; fab_hb_prev <= 1'b0;
            pix_hb_m <= 1'b0; pix_hb_s <= 1'b0; pix_hb_prev <= 1'b0;
            fab_alive <= 1'b0; pix_alive <= 1'b0;
            fab_timeout_cnt <= 0; pix_timeout_cnt <= 0;
            timeout_sticky <= 1'b0;
            active_mode_r <= 3'd0; build_id_r <= 8'd0;
        end else begin
            // 2-FF synchronizers for async status levels.
            pll_done_m <= i_tx_pll_done; pll_done_s <= pll_done_m;
            pll_lock_m <= i_tx_pll_lock; pll_lock_s <= pll_lock_m;
            lane_done_m <= i_tx_lane_done; lane_done_s <= lane_done_m;
            ready_m    <= i_tx_ready;     ready_s    <= ready_m;
            mode_m     <= i_mode_valid;   mode_s     <= mode_m;

            // Heartbeat change detection (heartbeats are slow toggles).
            fab_hb_m <= i_fabric_heartbeat; fab_hb_s <= fab_hb_m; fab_hb_prev <= fab_hb_s;
            pix_hb_m <= i_pixel_heartbeat;  pix_hb_s <= pix_hb_m;  pix_hb_prev <= pix_hb_s;

            if (fab_hb_s != fab_hb_prev) begin
                fab_alive <= 1'b1;
                fab_timeout_cnt <= 0;
            end else if (fab_timeout_cnt < C_SYS_TIMEOUT_CYCLES)
                fab_timeout_cnt <= fab_timeout_cnt + 1'b1;
            else begin
                fab_alive <= 1'b0;
                timeout_sticky <= 1'b1;
            end

            if (pix_hb_s != pix_hb_prev) begin
                pix_alive <= 1'b1;
                pix_timeout_cnt <= 0;
            end else if (pix_timeout_cnt < C_SYS_TIMEOUT_CYCLES)
                pix_timeout_cnt <= pix_timeout_cnt + 1'b1;
            else
                pix_alive <= 1'b0;

            active_mode_r <= i_active_mode;
            build_id_r    <= i_build_id;
        end
    end

    // ------------------------------------------------------------------
    // fabric_clk domain: presented counters and stable snapshot.
    // ------------------------------------------------------------------
    reg [31:0] idle_cnt, vsync_cnt, data_cnt;
    reg [15:0] frame_done_cnt, frame_abort_cnt, bad_desc_cnt;
    reg [15:0] snap_period_cnt;
    reg        snap_valid;
    // Snapshot latch registers (held stable between toggles).
    reg [31:0] snap_idle, snap_vsync, snap_data;
    reg [15:0] snap_done, snap_abort, snap_bad;

    always @(posedge i_fabric_clk or negedge i_fabric_rst_n) begin
        if (!i_fabric_rst_n) begin
            idle_cnt <= 0; vsync_cnt <= 0; data_cnt <= 0;
            frame_done_cnt <= 0; frame_abort_cnt <= 0; bad_desc_cnt <= 0;
            snap_period_cnt <= 0; snap_valid <= 1'b0;
            snap_idle <= 0; snap_vsync <= 0; snap_data <= 0;
            snap_done <= 0; snap_abort <= 0; snap_bad <= 0;
        end else begin
            if (i_tx_k == K_CTRL && i_tx_data == FVV1_IDLE)
                idle_cnt <= idle_cnt + 1'b1;
            if (i_tx_k == K_CTRL && i_tx_data == FVV1_VSYNC)
                vsync_cnt <= vsync_cnt + 1'b1;
            if (i_tx_k == K_DATA)
                data_cnt <= data_cnt + 1'b1;
            if (i_frame_done)  frame_done_cnt  <= frame_done_cnt + 1'b1;
            if (i_frame_abort) frame_abort_cnt <= frame_abort_cnt + 1'b1;
            if (i_bad_desc)    bad_desc_cnt    <= bad_desc_cnt + 1'b1;

            if (snap_period_cnt == 0) begin
                snap_period_cnt <= C_SNAPSHOT_PERIOD - 1;
                // Latch a stable snapshot, then toggle the valid flag.
                snap_idle <= idle_cnt; snap_vsync <= vsync_cnt; snap_data <= data_cnt;
                snap_done <= frame_done_cnt; snap_abort <= frame_abort_cnt;
                snap_bad <= bad_desc_cnt;
                snap_valid <= ~snap_valid;
            end else
                snap_period_cnt <= snap_period_cnt - 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Snapshot crossing: synchronize the toggle, then read stable latches.
    // ------------------------------------------------------------------
    reg snap_valid_m, snap_valid_s, snap_valid_prev;
    always @(posedge i_sys_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            snap_valid_m <= 1'b0; snap_valid_s <= 1'b0; snap_valid_prev <= 1'b0;
            o_presented_idle_count <= 0; o_presented_vsync_count <= 0;
            o_presented_data_count <= 0; o_frame_done_count <= 0;
            o_frame_abort_count <= 0; o_bad_desc_count <= 0;
        end else begin
            snap_valid_m <= snap_valid;
            snap_valid_s <= snap_valid_m;
            snap_valid_prev <= snap_valid_s;
            if (snap_valid_s != snap_valid_prev) begin
                o_presented_idle_count  <= snap_idle;
                o_presented_vsync_count <= snap_vsync;
                o_presented_data_count  <= snap_data;
                o_frame_done_count      <= snap_done;
                o_frame_abort_count     <= snap_abort;
                o_bad_desc_count        <= snap_bad;
            end
        end
    end

    // ------------------------------------------------------------------
    // Single LED select (V9, LVCMOS15, active-high). Never fabricate extra
    // LED pins. C_LED_SELECT: 0 -> mode_valid (sync), 1 -> tx_ready (default),
    // 2 -> fabric_clock_alive.
    // ------------------------------------------------------------------
    wire led_src;
    assign led_src = (C_LED_SELECT == 2) ? fab_alive :
                     (C_LED_SELECT == 1) ? ready_s   : mode_s;
    assign o_led = led_src;
endmodule
