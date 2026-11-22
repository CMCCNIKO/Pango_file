//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将跨域行描述符及 32 位像素流封装为 HSST-VIDEO-V1 的 VSYNC/HSTART/DATA/HEND 帧流，空闲期输出控制字。
// Module Name: fiber_video_tx_packetizer
// File Name: host/rtl/fiber/fiber_video_tx_packetizer.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将跨域行描述符及 32 位像素流封装为 HSST-VIDEO-V1 的 VSYNC/HSTART/DATA/HEND 帧流，空闲期输出控制字。
// 每次描述符握手明确分类为使用或丢弃；异常帧号/行号通过 o_desc_drop 归还对应行资源，帧中止立即返回等待状态。
//
// Dependencies: fiber_video_v1_defs.vh；fabric 时钟 i_clk 与低有效 i_rst_n；上游描述符及数据 ready/valid 握手。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "fiber_video_v1_defs.vh"
// HSST-VIDEO-V1 packetizer with an explicit descriptor accept/drop contract.
//
// The prior revision asserted o_desc_ready in ST_WAIT but only advanced state
// when the descriptor matched, so the upstream saw a completed handshake while
// the packetizer silently ignored the word ("bus accepted, business ignored").
// Now every descriptor accepted on (desc_valid && desc_ready) is either used
// (drop=0) or explicitly discarded (drop=1), and the line buffer releases the
// discarded row's bank through its normal return queue.
module fiber_video_tx_packetizer #(
    parameter integer C_DATA_PER_ROW = 320,
    parameter integer C_ROW_COUNT = 320,
    parameter integer C_DATA_PERIOD = 1
) (
    input wire         i_clk,
    input wire         i_rst_n,
    input wire         i_frame_abort,
    input wire         i_desc_valid,
    output wire        o_desc_ready,
    output wire        o_desc_drop,
    input wire [15:0]  i_desc_frame_id,
    input wire [8:0]   i_desc_row,
    input wire [1:0]   i_desc_bank,
    input wire         i_data_valid,
    output wire        o_data_ready,
    input wire [31:0]  i_data32,
    output reg [31:0]  o_tx_data,
    output reg [3:0]   o_tx_k,
    output reg         o_frame_done,
    output wire [15:0] o_frame_done_id
);
    localparam [2:0] ST_WAIT = 3'd0, ST_VSYNC = 3'd1, ST_HSTART = 3'd2,
                     ST_DATA = 3'd3, ST_HEND = 3'd4;
    reg [2:0] state;
    reg [15:0] frame_id;
    assign o_frame_done_id = frame_id;
    reg [8:0] row_index;
    reg [8:0] data_count;
    reg [7:0] pace_count;
    reg have_frame;

    // Descriptor accept/drop classification (combinational, valid on the
    // same cycle as desc_valid && desc_ready).
    wire desc_is_row0   = (i_desc_row == 9'd0);
    wire desc_continue  = have_frame && (i_desc_frame_id == frame_id) && (i_desc_row == row_index);
    wire desc_new_frame = (!have_frame && desc_is_row0) ||
                          (have_frame && desc_is_row0 && (i_desc_frame_id != frame_id));

    assign o_desc_ready = (state == ST_WAIT) && !i_frame_abort;
    assign o_desc_drop  = o_desc_ready && i_desc_valid && !desc_continue && !desc_new_frame;
    assign o_data_ready = (state == ST_DATA) && (pace_count == 0) && !i_frame_abort;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state        <= ST_WAIT;
            frame_id     <= 16'd0;
            row_index    <= 9'd0;
            data_count   <= 9'd0;
            pace_count   <= 8'd0;
            have_frame   <= 1'b0;
            o_tx_data    <= `FVV1_IDLE;
            o_tx_k       <= `FVV1_K_CTRL;
            o_frame_done <= 1'b0;
        end else begin
            o_tx_data    <= `FVV1_IDLE;
            o_tx_k       <= `FVV1_K_CTRL;
            o_frame_done <= 1'b0;
            if (i_frame_abort) begin
                state      <= ST_WAIT;
                have_frame <= 1'b0;
                pace_count <= 8'd0;
            end else begin
                if (pace_count != 0)
                    pace_count <= pace_count - 1'b1;
                case (state)
                    ST_WAIT: begin
                        if (i_desc_valid && o_desc_ready) begin
                            if (desc_new_frame) begin
                                frame_id   <= i_desc_frame_id;
                                row_index  <= 9'd0;
                                data_count <= 9'd0;
                                have_frame <= 1'b1;
                                state      <= ST_VSYNC;
                            end else if (desc_continue) begin
                                data_count <= 9'd0;
                                state      <= ST_HSTART;
                            end
                            // else: dropped (o_desc_drop=1); stay in ST_WAIT.
                        end
                    end
                    ST_VSYNC: begin
                        o_tx_data <= `FVV1_VSYNC;
                        o_tx_k    <= `FVV1_K_CTRL;
                        state     <= ST_HSTART;
                    end
                    ST_HSTART: begin
                        o_tx_data <= `FVV1_HSTART;
                        o_tx_k    <= `FVV1_K_CTRL;
                        data_count <= 0;
                        state     <= ST_DATA;
                    end
                    ST_DATA: begin
                        if (i_data_valid && o_data_ready) begin
                            o_tx_data <= i_data32;
                            o_tx_k    <= `FVV1_K_DATA;
                            data_count <= data_count + 1'b1;
                            pace_count <= (C_DATA_PERIOD > 0) ? C_DATA_PERIOD - 1 : 0;
                            if (data_count == C_DATA_PER_ROW-1)
                                state <= ST_HEND;
                        end
                    end
                    ST_HEND: begin
                        o_tx_data <= `FVV1_HEND;
                        o_tx_k    <= `FVV1_K_CTRL;
                        if (row_index == C_ROW_COUNT-1) begin
                            o_frame_done <= 1'b1;
                            state <= ST_WAIT;
                            have_frame <= 1'b0;
                        end else begin
                            row_index <= row_index + 1'b1;
                            state <= ST_WAIT;
                        end
                    end
                    default: state <= ST_WAIT;
                endcase
            end
        end
    end
endmodule
