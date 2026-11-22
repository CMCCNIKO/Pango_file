//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 系统时钟域的 ADV7611 上电复位与输入看门狗：延迟释放接收芯片复位，根据像素心跳、TMDS 锁定和初始化完成状态生成像素复位及超时标志。
// Module Name: host_reset_supervisor
// File Name: host/rtl/board/host_reset_supervisor.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 系统时钟域的 ADV7611 上电复位与输入看门狗：延迟释放接收芯片复位，根据像素心跳、TMDS 锁定和初始化完成状态生成像素复位及超时标志。
// 管理过程使用系统时钟，避免像素时钟停振后无法执行复位管理。
//
// Dependencies: i_sys_clk、低有效异步复位 i_rst_n；输入像素心跳与 TMDS/ADV 初始化状态。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
// System-clock reset supervisor.  Pixel clocks may stop; ADV7611 reset and
// watchdog operation therefore never depend on a pixel-clock always block.
module host_reset_supervisor #(
    parameter integer C_RELEASE_CYCLES = 50000,
    parameter integer C_WATCHDOG_CYCLES = 250000
) (
    input wire i_sys_clk,
    input wire i_rst_n,
    input wire i_pixel_heartbeat,
    input wire i_tmds_lock,
    input wire i_adv_init_done,
    output reg o_adv_reset_n,
    output reg o_pixel_reset_n,
    output reg o_input_timeout
);
    reg [31:0] release_count;
    reg [31:0] watchdog_count;
    reg heartbeat_d;
    reg heartbeat_seen;
    always @(posedge i_sys_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            release_count <= 0;
            watchdog_count <= 0;
            heartbeat_d <= 0;
            heartbeat_seen <= 0;
            o_adv_reset_n <= 0;
            o_pixel_reset_n <= 0;
            o_input_timeout <= 0;
        end else begin
            heartbeat_d <= i_pixel_heartbeat;
            if (i_pixel_heartbeat != heartbeat_d) begin
                heartbeat_seen <= 1;
                watchdog_count <= 0;
                o_input_timeout <= 0;
            end else if (watchdog_count < C_WATCHDOG_CYCLES)
                watchdog_count <= watchdog_count + 1'b1;
            else begin
                o_input_timeout <= 1;
                o_pixel_reset_n <= 0;
            end
            if (release_count < C_RELEASE_CYCLES)
                release_count <= release_count + 1'b1;
            else
                o_adv_reset_n <= 1;
            o_pixel_reset_n <= o_adv_reset_n && (i_tmds_lock || !heartbeat_seen);
            if (i_adv_init_done && i_tmds_lock && heartbeat_seen)
                o_pixel_reset_n <= 1;
        end
    end
endmodule
