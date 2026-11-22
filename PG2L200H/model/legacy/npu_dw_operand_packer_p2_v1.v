//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将两个像素的 3×3 深度卷积窗口和八滤波器权重，组合打包为 16 lane×8 term 的共享 MAC 操作数。
// Module Name: npu_dw_operand_packer_p2_v1
// File Name: model/legacy/npu_dw_operand_packer_p2_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将两个像素的 3×3 深度卷积窗口和八滤波器权重，组合打包为 16 lane×8 term 的共享 MAC 操作数。
// chunk0 装入 tap0..7，chunk1 仅使用 term0 装入 tap8；依据 stride、像素有效及边界位置产生 padding、lane_valid 和 term_valid。
//
// Dependencies: 共享 16×8 MAC 的 lane/term 总线契约
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 纯组合接线与选择逻辑，不含时钟、存储或运算流水；lane=filter×2+pixel。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

// Fixed-index depthwise operand packet packer for the 8-filter, 2-pixel
// backend shape. Chunk 0 carries taps 0 through 7; chunk 1 carries tap 8 in
// term 0 and leaves the remaining terms empty.
module npu_dw_operand_packer_p2_v1 (
    input  wire [575:0]  window0,
    input  wire [575:0]  window1,
    input  wire [639:0]  weight_cache,
    input  wire          issue_chunk,
    input  wire          cfg_stride2,
    input  wire [5:0]    issue_pair,
    input  wire          pixel0_valid,
    input  wire          pixel1_valid,
    input  wire [7:0]    pad_zero,
    output wire [15:0]   lane_valid,
    output wire [127:0]  term_valid,
    output wire [1023:0] qd,
    output wire [1023:0] qw
);

    genvar filter_gen;
    genvar pixel_gen;
    genvar term_gen;
    generate
        for (filter_gen = 0; filter_gen < 8;
             filter_gen = filter_gen + 1) begin : gen_filter
            for (pixel_gen = 0; pixel_gen < 2;
                 pixel_gen = pixel_gen + 1) begin : gen_pixel
                localparam integer LANE_INDEX = filter_gen*2 + pixel_gen;
                wire lane_input_valid;

                if (pixel_gen == 0) begin : gen_pixel0
                    assign lane_input_valid = pixel0_valid;
                end else begin : gen_pixel1
                    assign lane_input_valid = pixel1_valid;
                end

                assign lane_valid[LANE_INDEX] = lane_input_valid;

                for (term_gen = 0; term_gen < 8;
                     term_gen = term_gen + 1) begin : gen_term
                    localparam integer TERM_INDEX = LANE_INDEX*8 + term_gen;
                    localparam integer PACKET_LSB = TERM_INDEX*8;
                    localparam integer CHUNK0_TAP_Y = term_gen/3;
                    localparam integer CHUNK0_TAP_X = term_gen%3;
                    localparam integer CHUNK0_SOURCE_LSB =
                        term_gen*64 + filter_gen*8;
                    localparam integer CHUNK1_SOURCE_LSB =
                        8*64 + filter_gen*8;

                    wire [7:0] chunk0_window_data;
                    wire [7:0] chunk1_window_data;
                    wire [7:0] chunk0_data;
                    wire chunk0_pad;

                    if (pixel_gen == 0) begin : gen_window0
                        assign chunk0_window_data = window0[
                            CHUNK0_SOURCE_LSB +: 8
                        ];
                        assign chunk1_window_data = window0[
                            CHUNK1_SOURCE_LSB +: 8
                        ];
                    end else begin : gen_window1
                        assign chunk0_window_data = window1[
                            CHUNK0_SOURCE_LSB +: 8
                        ];
                        assign chunk1_window_data = window1[
                            CHUNK1_SOURCE_LSB +: 8
                        ];
                    end

                    // For stride 1, pair[5:2] is the row and pair[1:0]
                    // selects the two-pixel column. Pixel 1 can never touch
                    // the left border because its x coordinate is x0 + 1.
                    assign chunk0_pad = !cfg_stride2
                        && (((CHUNK0_TAP_Y == 0)
                             && (issue_pair[5:2] == 4'd0))
                            || ((pixel_gen == 0)
                                && (CHUNK0_TAP_X == 0)
                                && (issue_pair[1:0] == 2'd0)));
                    assign chunk0_data = chunk0_pad
                        ? pad_zero : chunk0_window_data;

                    if (term_gen == 0) begin : gen_term0
                        assign term_valid[TERM_INDEX] = lane_input_valid;
                        assign qd[PACKET_LSB +: 8] = issue_chunk
                            ? chunk1_window_data : chunk0_data;
                        assign qw[PACKET_LSB +: 8] = issue_chunk
                            ? weight_cache[CHUNK1_SOURCE_LSB +: 8]
                            : weight_cache[CHUNK0_SOURCE_LSB +: 8];
                    end else begin : gen_term1_to_7
                        assign term_valid[TERM_INDEX] = issue_chunk
                            ? 1'b0 : lane_input_valid;
                        assign qd[PACKET_LSB +: 8] = issue_chunk
                            ? 8'd0 : chunk0_data;
                        assign qw[PACKET_LSB +: 8] = issue_chunk
                            ? 8'd0
                            : weight_cache[CHUNK0_SOURCE_LSB +: 8];
                    end
                end
            end
        end
    endgenerate

endmodule

`default_nettype wire
