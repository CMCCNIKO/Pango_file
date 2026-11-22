//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将像素域一拍帧中止转为保持的 flush 电平并同步到发送域；只有像素侧和发送侧均完成排空，才清除 flush、允许接收下一帧。
// Module Name: host_frame_control_cdc
// File Name: host/rtl/fiber/host_frame_control_cdc.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将像素域一拍帧中止转为保持的 flush 电平并同步到发送域；只有像素侧和发送侧均完成排空，才清除 flush、允许接收下一帧。
// 进行中的中止请求合并，跨域传递电平与完成状态，避免窄脉冲丢失。
//
// Dependencies: 像素域 i_pix_clk/i_pix_rst_n 与发送域 i_tx_clk/i_tx_rst_n；依赖 host_line_buffer_cdc 返回两域 flush_done。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
// Frame-abort / flush handshake with real completion, not request-loopback.
//
// The normalizer asserts a one-cycle o_frame_abort in the pixel domain when a
// frame is malformed (e.g. a line ended with the wrong DE pixel count).  That
// event must (a) make the packetizer drop the frame and return to IDLE, and
// (b) make the line buffer drain both FIFOs and release all banks, before any
// new row0 is admitted.
//
// This module turns the abort pulse into a level 'flush' in both clock domains
// and releases it only after BOTH domains report their own drain complete:
//
//   i_frame_abort_pix  ->  o_flush_pix  == 1   (pixel domain, held)
//   o_flush_pix  ->  2FF sync  ->  o_flush_tx  == 1  (fabric domain)
//   line buffer drains each side, then:
//       i_flush_done_pix (pixel)  &&  i_flush_done_tx (fabric, synced back)
//   -> o_flush_pix/o_flush_tx fall, a new frame may start.
//
// Successive aborts during an ongoing flush are coalesced (the level simply
// stays asserted).  The pixel-domain abort is never a narrow pulse crossing a
// domain: only the level o_flush_pix is synchronized.
module host_frame_control_cdc (
    input  wire i_pix_clk,
    input  wire i_pix_rst_n,
    input  wire i_frame_abort_pix,   // one-cycle abort pulse (normalizer)
    output wire o_flush_pix,         // pixel-domain flush level
    input  wire i_flush_done_pix,    // pixel-domain drain complete (line buffer)
    input  wire i_tx_clk,
    input  wire i_tx_rst_n,
    output wire o_flush_tx,          // fabric-domain flush level
    input  wire i_flush_done_tx      // fabric-domain drain complete (line buffer)
);
    reg flush_pix_q;
    reg tx_done_meta, tx_done_sync;
    reg flush_meta, flush_tx_q;

    assign o_flush_pix = flush_pix_q;
    assign o_flush_tx  = flush_tx_q;

    // Pixel domain: abort sets the level; both domains' drain-done clears it.
    always @(posedge i_pix_clk or negedge i_pix_rst_n) begin
        if (!i_pix_rst_n) begin
            flush_pix_q <= 1'b0;
            tx_done_meta <= 1'b0;
            tx_done_sync <= 1'b0;
        end else begin
            tx_done_meta <= i_flush_done_tx;
            tx_done_sync <= tx_done_meta;
            if (i_frame_abort_pix)
                flush_pix_q <= 1'b1;
            else if (i_flush_done_pix && tx_done_sync)
                flush_pix_q <= 1'b0;
        end
    end

    // Fabric domain: two-stage sync of the pixel-domain flush level.
    always @(posedge i_tx_clk or negedge i_tx_rst_n) begin
        if (!i_tx_rst_n) begin
            flush_meta <= 1'b0;
            flush_tx_q <= 1'b0;
        end else begin
            flush_meta <= o_flush_pix;
            flush_tx_q <= flush_meta;
        end
    end
endmodule
