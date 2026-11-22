//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 八lane固定延迟仿射UINT8量化流水，将16lane累加批次分两组进行weight-zero修正、bias、激活、主缩放及可选shortcut缩放。
// Module Name: npu_pq8_stream_pipeline_v2
// File Name: model/rtl/f14_sop_pq8_stream_pipeline_v2.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 八lane固定延迟仿射UINT8量化流水，将16lane累加批次分两组进行weight-zero修正、bias、激活、主缩放及可选shortcut缩放。
// 支持逐lane量化系数、舍入与饱和到U8，接收批次时预留FIFO输出位置，使用窄结果队列吸收下游背压。
//
// Dependencies: npu_mult17s16u_pq、npu_mult34s16u_pq 等当前在用兼容算术模块
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 内部valid/data固定推进，不向DSP时钟使能传播任意下游停顿；保留GROUPS_PER_BATCH等配置契约。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// F14 isolated adaptation; original SOP source is unchanged.
// F8/P2 bias sharing; signed34 activation and exact 50-bit product.
`timescale 1ns/1ps
`default_nettype none

// Eight-lane fixed-latency affine UINT8 quantization pipeline.
//
// This keeps the useful structure of the paper-reproduction RTL: each stage
// performs one class of work and valid/data advance without internal
// backpressure. Unlike the original 144-lane implementation, this module
// accepts a 16-result batch as two consecutive eight-lane groups.
//
// Three independent eight-lane multiplier banks operate concurrently:
//   1. dynamic weight-zero correction,
//   2. optional shortcut scaling,
//   3. main Q1.14 scaling.
//
// The arithmetic is byte-exact with npu_pq8_physical_compact:
//   corrected = sum(qd*qw) - Zw*sum(qd)
//   affine    = corrected + q_b_f
//   activate  = LINEAR or ReLU6
//   output    = round(main*M) + round((qs-Zs)*Ms) + Zo
//   saturate to UINT8.
//
// Accepted batches reserve two output FIFO entries. Once a group enters the
// pipeline it never stalls; output backpressure is absorbed by the narrow
// 8-lane FIFO rather than fed into the DSP clock-enable network.
module npu_pq8_stream_pipeline_v2 #(
    // The legacy interface accepts one 16-lane batch and emits two groups.
    // A value of one turns the block into a fixed eight-lane slice; only the
    // lower eight input lanes are consumed and the wide group-select mux is
    // compiled out. Two such slices form the local-control PQ16 backend.
    parameter integer GROUPS_PER_BATCH = 2,
    // When enabled, batch_valid_q is only a sequencing token. Multiplier
    // operands are always driven from the registered payload and invalid
    // results are discarded by the existing valid pipeline.
    parameter integer TOKEN_ONLY_VALID = 0,
    // Wide metadata is always qualified by the valid pipeline. Disabling its
    // reset removes an initialization-only clear network from inferred shift
    // RAMs while all valid/FIFO/state registers remain reset normally.
    parameter integer RESET_WIDE_META = 1,
    // A ready/valid producer may change its payload immediately after an
    // accepted batch. Enable this mode when the two output groups can overlap
    // the next producer transaction; the accepted payload then remains stable
    // until both groups have entered the fixed pipeline.
    parameter integer CAPTURE_ON_ACCEPT = 0,
    // Logical FIFO geometry.  The defaults preserve the accepted eight-entry
    // implementation; larger values let the shared backend hide fixed PQ
    // return latency without changing the arithmetic pipeline.
    parameter integer FIFO_DEPTH = 8,
    parameter integer FIFO_AW = 3,
    // When set, the main requantization multiplier is supplied per lane.
    // The legacy scalar port remains valid when this is zero.
    parameter integer USE_PER_LANE_MULTIPLIER = 0,
    // Legacy scalar multipliers use Q1.14.  The per-lane path uses Q2.12
    // so the 16-bit unsigned operand can represent the larger scales that
    // occur in depthwise channels without clipping.
    parameter integer MAIN_MULTIPLIER_SHIFT = 14
) (
    input  wire                         clk,
    input  wire                         rst_n,
    input  wire                         clear,

    input  wire                         in_valid,
    output wire                         in_ready,
    input  wire [15:0]                  in_lane_valid_bus,
    input  wire [15:0]                  in_acc_pre_corrected_bus,
    input  wire [16*32-1:0]             in_acc_bus,
    input  wire [16*32-1:0]             in_sum_qd_bus,
    input  wire [7:0]                   in_weight_zero,
    input  wire [8*32-1:0]              in_bias_f8,
    input  wire                         in_relu6_en,
    input  wire [31:0]                  in_relu6_limit,
    input  wire [15:0]                  in_multiplier,
    input  wire [16*16-1:0]              in_multiplier_bus,
    input  wire [3:0]                    in_multiplier_shift,
    input  wire [7:0]                   in_output_zero,
    input  wire                         in_shortcut_en,
    input  wire [16*8-1:0]              in_shortcut_q_c8p2,
    input  wire [7:0]                   in_shortcut_zero,
    input  wire [15:0]                  in_shortcut_multiplier,

    output wire                         out_valid,
    input  wire                         out_ready,
    output wire [7:0]                   out_lane_valid_bus,
    output wire [8*8-1:0]               out_q_bus,

    output reg  [6:0]                   fifo_level_results,
    output wire                         fifo_high_watermark,
    output wire                         fifo_full,
    output wire                         fifo_empty,
    output reg                          status_error,
    output reg  [9:0]                   debug_error_code
);

    localparam integer LANES      = 8;
    localparam integer META_DELAY = 3;

    // ---------------------------------------------------------------------
    // One batch register feeds group 0 and group 1 in consecutive cycles.
    // A new batch may replace it on the same edge that group 1 is issued.
    // ---------------------------------------------------------------------
    reg                         batch_valid_q;
    // Keep one physically distinct group-select bit per output lane. A single
    // scalar select otherwise fans out through all 16-to-8 metadata muxes and
    // the eight correction multipliers.
    (* preserve *) reg [LANES-1:0] batch_group_lane_q;
    wire batch_group_q = batch_group_lane_q[0];
    reg [15:0]                  batch_lane_valid_q;
    reg [15:0]                  batch_acc_pre_corrected_q;
    reg [16*32-1:0]             batch_acc_q;
    reg [16*32-1:0]             batch_sum_qd_q;
    reg [7:0]                   batch_weight_zero_q;
    reg [8*32-1:0]              batch_bias_q;
    reg                         batch_relu6_en_q;
    reg [31:0]                  batch_relu6_limit_q;
    reg [16*16-1:0]             batch_multiplier_q;
    reg [3:0]                   batch_multiplier_shift_q;
    reg [7:0]                   batch_output_zero_q;
    reg                         batch_shortcut_en_q;
    reg [16*8-1:0]              batch_shortcut_q_q;
    reg [7:0]                   batch_shortcut_zero_q;
    reg [15:0]                  batch_shortcut_multiplier_q;
    reg                         feeder_accept_credit_q;

    wire [16*16-1:0] effective_multiplier_bus =
        (USE_PER_LANE_MULTIPLIER != 0)
            ? in_multiplier_bus : {16{in_multiplier}};

    function [16:0] shortcut_delta17;
        input shortcut_enable;
        input [7:0] shortcut_q;
        input [7:0] shortcut_zero;
        reg signed [8:0] delta;
        begin
            if (shortcut_enable) begin
                delta = $signed({1'b0, shortcut_q})
                    - $signed({1'b0, shortcut_zero});
                shortcut_delta17 = {{8{delta[8]}}, delta};
            end else begin
                shortcut_delta17 = 17'd0;
            end
        end
    endfunction

    wire issue_valid = batch_valid_q;

    // ---------------------------------------------------------------------
    // Narrow output FIFO and credit reservation.
    // ---------------------------------------------------------------------
    (* ramstyle = "M4K" *) reg [7:0] fifo_valid [0:FIFO_DEPTH-1];
    (* ramstyle = "M4K" *) reg [63:0] fifo_q [0:FIFO_DEPTH-1];
    reg [FIFO_AW-1:0] fifo_wr_ptr;
    reg [FIFO_AW-1:0] fifo_rd_ptr;
    reg [FIFO_AW:0]   fifo_count;
    reg [FIFO_AW:0]   reserved_groups;
    reg                fifo_space_available_q;

    wire [FIFO_AW+1:0] committed_groups =
        {1'b0, fifo_count} + {1'b0, reserved_groups};

    wire fifo_pop;
    wire fifo_push;
    wire accept_batch;

    // Space is registered before leaving the PQ block. After accepting a
    // batch, the two-group feeder cannot accept another batch on the next
    // cycle, so the one-cycle-old credit cannot overbook the eight-entry FIFO.
    assign in_ready = !clear && feeder_accept_credit_q
                    && fifo_space_available_q;
    assign accept_batch = in_valid && in_ready;

    assign out_valid          = !clear && (fifo_count != 0);
    assign out_lane_valid_bus = fifo_valid[fifo_rd_ptr];
    assign out_q_bus          = fifo_q[fifo_rd_ptr];
    assign fifo_pop           = out_valid && out_ready;

    reg sum_qd_width_bad;
    reg multiplier_sign_bad;
    integer diagnostic_lane;
    always @(*) begin
        sum_qd_width_bad = 1'b0;
        multiplier_sign_bad = 1'b0;
        for (diagnostic_lane = 0; diagnostic_lane < 16;
             diagnostic_lane = diagnostic_lane + 1) begin
            if (in_sum_qd_bus[diagnostic_lane*32 + 17 +: 15] != 0)
                sum_qd_width_bad = 1'b1;
        end
        multiplier_sign_bad = in_shortcut_multiplier[15];
        for (diagnostic_lane = 0; diagnostic_lane < 16;
             diagnostic_lane = diagnostic_lane + 1) begin
            if (effective_multiplier_bus[diagnostic_lane*16 + 15])
                multiplier_sign_bad = 1'b1;
        end
    end

    assign fifo_full = (committed_groups >= FIFO_DEPTH);
    assign fifo_empty = clear
        || ((fifo_count == 0) && (reserved_groups == 0));
    assign fifo_high_watermark =
        (committed_groups >= (FIFO_DEPTH - GROUPS_PER_BATCH));

    always @(*) begin
        if (fifo_count > 15)
            fifo_level_results = 7'h7f;
        else
            fifo_level_results = {fifo_count[3:0], 3'b000};
    end

    // Per-operation shift has a single sequential owner, including reset.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) batch_multiplier_shift_q <= MAIN_MULTIPLIER_SHIFT;
        else if (clear) batch_multiplier_shift_q <= MAIN_MULTIPLIER_SHIFT;
        else if ((CAPTURE_ON_ACCEPT == 0) || accept_batch)
            batch_multiplier_shift_q <= (in_multiplier_shift == 0)
                ? MAIN_MULTIPLIER_SHIFT : in_multiplier_shift;
    end

    // The legacy physical baseline samples unconditionally to avoid wide
    // clock enables. Streaming producers enable CAPTURE_ON_ACCEPT so group 1
    // cannot observe the following transaction's payload.
    always @(posedge clk) begin
        if ((CAPTURE_ON_ACCEPT == 0) || accept_batch) begin
            batch_lane_valid_q <= in_lane_valid_bus;
            batch_acc_pre_corrected_q <= in_acc_pre_corrected_bus;
            batch_acc_q <= in_acc_bus;
            batch_sum_qd_q <= in_sum_qd_bus;
            batch_weight_zero_q <= in_weight_zero;
            batch_bias_q <= in_bias_f8;
            batch_relu6_en_q <= in_relu6_en;
            batch_relu6_limit_q <= in_relu6_limit;
            batch_multiplier_q <= effective_multiplier_bus;
            batch_output_zero_q <= in_output_zero;
            batch_shortcut_en_q <= in_shortcut_en;
            batch_shortcut_q_q <= in_shortcut_q_c8p2;
            batch_shortcut_zero_q <= in_shortcut_zero;
            batch_shortcut_multiplier_q <= in_shortcut_multiplier;
        end
    end

    // ---------------------------------------------------------------------
    // Correction and shortcut multiplier banks.
    // ---------------------------------------------------------------------
    reg signed [16:0] corr_mult_dataa [0:LANES-1];
    reg        [15:0] corr_mult_datab [0:LANES-1];
    wire       [32:0] corr_mult_result[0:LANES-1];

    reg signed [16:0] sc_mult_dataa [0:LANES-1];
    reg        [15:0] sc_mult_datab [0:LANES-1];
    wire       [32:0] sc_mult_result[0:LANES-1];

    integer issue_lane;
    integer issue_source;
    always @(*) begin
        issue_source = 0;
        for (issue_lane = 0; issue_lane < LANES;
             issue_lane = issue_lane + 1) begin
            issue_source = ((GROUPS_PER_BATCH == 1)
                ? 0 : (batch_group_lane_q[issue_lane] ? LANES : 0))
                + issue_lane;

            corr_mult_dataa[issue_lane] = 17'sd0;
            corr_mult_datab[issue_lane] = 16'd0;
            sc_mult_dataa[issue_lane] = 17'sd0;
            sc_mult_datab[issue_lane] = 16'd0;

            if ((TOKEN_ONLY_VALID != 0) || issue_valid) begin
                corr_mult_dataa[issue_lane] = $signed({
                    9'd0,
                    batch_weight_zero_q
                });
                corr_mult_datab[issue_lane] =
                    batch_sum_qd_q[issue_source*32 +: 16];

                sc_mult_dataa[issue_lane] = $signed(
                    shortcut_delta17(
                        batch_shortcut_en_q,
                        batch_shortcut_q_q[
                            ({issue_source[0], issue_source[3:1]}*8) +: 8
                        ],
                        batch_shortcut_zero_q
                    )
                );
                sc_mult_datab[issue_lane] =
                    batch_shortcut_multiplier_q;
            end
        end
    end

    genvar corr_lane;
    generate
        for (corr_lane = 0; corr_lane < LANES;
             corr_lane = corr_lane + 1) begin : g_corr_multiplier
            npu_mult17s16u_pq u_corr_multiplier (
                .clock0(clk),
                .dataa_0(corr_mult_dataa[corr_lane]),
                .datab_0(corr_mult_datab[corr_lane]),
                .ena0(1'b1),
                .result(corr_mult_result[corr_lane])
            );
        end
    endgenerate

    genvar shortcut_lane;
    generate
        for (shortcut_lane = 0; shortcut_lane < LANES;
             shortcut_lane = shortcut_lane + 1) begin : g_shortcut_multiplier
            npu_mult17s16u_pq u_shortcut_multiplier (
                .clock0(clk),
                .dataa_0(sc_mult_dataa[shortcut_lane]),
                .datab_0(sc_mult_datab[shortcut_lane]),
                .ena0(1'b1),
                .result(sc_mult_result[shortcut_lane])
            );
        end
    endgenerate

    // Metadata follows the three-cycle multiplier latency.
    reg [META_DELAY-1:0] pre_valid_pipe;
    reg [7:0]   pre_lane_valid [0:META_DELAY-1];
    reg [7:0]   pre_acc_pre_corrected [0:META_DELAY-1];
    reg [255:0] pre_acc [0:META_DELAY-1];
    reg [255:0] pre_sum_qd [0:META_DELAY-1];
    reg [7:0]   pre_weight_zero [0:META_DELAY-1];
    // Bias is a per-output-lane 32-bit quantity.  Keep all eight lanes
    // through the multiplier/correction pipeline; packing two lanes into a
    // shared 32-bit word silently applied lane 0's bias to lane 1, etc.
    reg [255:0] pre_bias [0:META_DELAY-1];
    reg         pre_relu6_en [0:META_DELAY-1];
    reg [31:0]  pre_relu6_limit [0:META_DELAY-1];
    reg [LANES*16-1:0] pre_multiplier [0:META_DELAY-1];
    reg [3:0]   pre_multiplier_shift [0:META_DELAY-1];
    reg [7:0]   pre_output_zero [0:META_DELAY-1];

    // ---------------------------------------------------------------------
    // Correction stage: assemble Zw*sum(qd) and subtract it from the MAC
    // accumulator. Bias addition is registered separately below so the
    // integrated design does not place two wide add/sub chains in one cycle.
    // ---------------------------------------------------------------------
    reg        correction_valid_q;
    reg [7:0]  correction_lane_valid_q;
    reg signed [41:0] correction_value_q [0:LANES-1];
    reg [255:0] correction_bias_q;
    reg         correction_relu6_en_q;
    reg [31:0]  correction_relu6_limit_q;
    reg [LANES*16-1:0] correction_multiplier_q;
    reg [3:0]   correction_multiplier_shift_q;
    reg [7:0]   correction_output_zero_q;
    reg signed [32:0] correction_shortcut_product_q [0:LANES-1];

    reg [40:0] correction_calc [0:LANES-1];
    reg signed [41:0] corrected_calc [0:LANES-1];
    integer correction_lane;
    always @(*) begin
        for (correction_lane = 0; correction_lane < LANES;
             correction_lane = correction_lane + 1) begin
            correction_calc[correction_lane] = {
                8'd0,
                corr_mult_result[correction_lane]
            };
            if (pre_sum_qd[META_DELAY-1][correction_lane*32 + 16])
                correction_calc[correction_lane] =
                    correction_calc[correction_lane]
                    + ({
                        33'd0,
                        pre_weight_zero[META_DELAY-1]
                    } << 16);

            if (pre_acc_pre_corrected[META_DELAY-1][correction_lane]) begin
                corrected_calc[correction_lane] =
                    $signed({
                        {10{
                            pre_acc[META_DELAY-1][
                                correction_lane*32 + 31
                            ]
                        }},
                        pre_acc[META_DELAY-1][
                            correction_lane*32 +: 32
                        ]
                    });
            end else begin
                corrected_calc[correction_lane] =
                    $signed({
                        {10{
                            pre_acc[META_DELAY-1][
                                correction_lane*32 + 31
                            ]
                        }},
                        pre_acc[META_DELAY-1][
                            correction_lane*32 +: 32
                        ]
                    })
                    - $signed({
                        1'b0,
                        correction_calc[correction_lane]
                    });
            end
        end
    end

    // ---------------------------------------------------------------------
    // Affine stage: add q_b_f to the registered corrected accumulator.
    // ---------------------------------------------------------------------
    reg        affine_valid_q;
    reg [7:0]  affine_lane_valid_q;
    reg signed [41:0] affine_value_q [0:LANES-1];
    reg         affine_relu6_en_q;
    reg [31:0]  affine_relu6_limit_q;
    reg [LANES*16-1:0] affine_multiplier_q;
    reg [3:0]   affine_multiplier_shift_q;
    reg [7:0]   affine_output_zero_q;
    reg signed [32:0] affine_shortcut_product_q [0:LANES-1];

    reg signed [41:0] affine_calc [0:LANES-1];
    integer affine_lane;
    always @(*) begin
        for (affine_lane = 0; affine_lane < LANES;
             affine_lane = affine_lane + 1) begin
            affine_calc[affine_lane] =
                correction_value_q[affine_lane]
                + $signed({
                    {10{
                        correction_bias_q[
                            affine_lane*32 + 31
                        ]
                    }},
                    correction_bias_q[affine_lane*32 +: 32]
                });
        end
    end

    // ---------------------------------------------------------------------
    // Activation clamp-decision stage. The exact 42-bit comparison is split
    // into six parallel 7-bit chunks. Registered gt/eq flags keep each
    // affine-to-clamp path short while preserving the existing stage count.
    // ---------------------------------------------------------------------
    reg        clamp_valid_q;
    reg [7:0]  clamp_lane_valid_q;
    reg [7:0]  clamp_force_zero_q;
    reg [LANES*6-1:0] clamp_compare_gt_q;
    reg [LANES*6-1:0] clamp_compare_eq_q;
    reg signed [33:0] clamp_affine_q [0:LANES-1];
    reg signed [33:0] clamp_limit_q;
    reg [LANES*16-1:0] clamp_multiplier_q;
    reg [3:0]   clamp_multiplier_shift_q;
    reg [7:0]   clamp_output_zero_q;
    reg signed [32:0] clamp_shortcut_product_q [0:LANES-1];

    reg signed [41:0] affine_limit_calc [0:LANES-1];
    reg [LANES-1:0] clamp_force_zero_calc;
    reg [LANES*6-1:0] clamp_compare_gt_calc;
    reg [LANES*6-1:0] clamp_compare_eq_calc;
    reg [LANES-1:0] range_signed_bad_lo_calc;
    reg [LANES-1:0] range_signed_bad_hi_calc;
    reg [LANES-1:0] range_affine_pos_bad_lo_calc;
    reg [LANES-1:0] range_affine_pos_bad_hi_calc;
    reg [LANES-1:0] range_limit_pos_bad_lo_calc;
    reg [LANES-1:0] range_limit_pos_bad_hi_calc;
    reg [LANES-1:0] range_affine_sign_calc;
    integer clamp_lane;
    always @(*) begin
        clamp_force_zero_calc = {LANES{1'b0}};
        clamp_compare_gt_calc = {(LANES*6){1'b0}};
        clamp_compare_eq_calc = {(LANES*6){1'b0}};
        range_signed_bad_lo_calc = {LANES{1'b0}};
        range_signed_bad_hi_calc = {LANES{1'b0}};
        range_affine_pos_bad_lo_calc = {LANES{1'b0}};
        range_affine_pos_bad_hi_calc = {LANES{1'b0}};
        range_limit_pos_bad_lo_calc = {LANES{1'b0}};
        range_limit_pos_bad_hi_calc = {LANES{1'b0}};
        range_affine_sign_calc = {LANES{1'b0}};
        for (clamp_lane = 0; clamp_lane < LANES;
             clamp_lane = clamp_lane + 1) begin
            affine_limit_calc[clamp_lane] = $signed({
                {10{affine_relu6_limit_q[31]}},
                affine_relu6_limit_q
            });

            clamp_force_zero_calc[clamp_lane] =
                affine_relu6_en_q
                && (affine_value_q[clamp_lane][41]
                    || affine_limit_calc[clamp_lane][41]);
            if (affine_relu6_en_q) begin
                clamp_compare_gt_calc[clamp_lane*6] =
                    $unsigned(affine_value_q[clamp_lane][6:0])
                    > $unsigned(affine_limit_calc[clamp_lane][6:0]);
                clamp_compare_eq_calc[clamp_lane*6] =
                    affine_value_q[clamp_lane][6:0]
                    == affine_limit_calc[clamp_lane][6:0];
                clamp_compare_gt_calc[clamp_lane*6 + 1] =
                    $unsigned(affine_value_q[clamp_lane][13:7])
                    > $unsigned(affine_limit_calc[clamp_lane][13:7]);
                clamp_compare_eq_calc[clamp_lane*6 + 1] =
                    affine_value_q[clamp_lane][13:7]
                    == affine_limit_calc[clamp_lane][13:7];
                clamp_compare_gt_calc[clamp_lane*6 + 2] =
                    $unsigned(affine_value_q[clamp_lane][20:14])
                    > $unsigned(affine_limit_calc[clamp_lane][20:14]);
                clamp_compare_eq_calc[clamp_lane*6 + 2] =
                    affine_value_q[clamp_lane][20:14]
                    == affine_limit_calc[clamp_lane][20:14];
                clamp_compare_gt_calc[clamp_lane*6 + 3] =
                    $unsigned(affine_value_q[clamp_lane][27:21])
                    > $unsigned(affine_limit_calc[clamp_lane][27:21]);
                clamp_compare_eq_calc[clamp_lane*6 + 3] =
                    affine_value_q[clamp_lane][27:21]
                    == affine_limit_calc[clamp_lane][27:21];
                clamp_compare_gt_calc[clamp_lane*6 + 4] =
                    $unsigned(affine_value_q[clamp_lane][34:28])
                    > $unsigned(affine_limit_calc[clamp_lane][34:28]);
                clamp_compare_eq_calc[clamp_lane*6 + 4] =
                    affine_value_q[clamp_lane][34:28]
                    == affine_limit_calc[clamp_lane][34:28];
                clamp_compare_gt_calc[clamp_lane*6 + 5] =
                    $unsigned(affine_value_q[clamp_lane][41:35])
                    > $unsigned(affine_limit_calc[clamp_lane][41:35]);
                clamp_compare_eq_calc[clamp_lane*6 + 5] =
                    affine_value_q[clamp_lane][41:35]
                    == affine_limit_calc[clamp_lane][41:35];
            end

            // Range diagnostics use the pre-clamp operands and are pipelined
            // separately from the functional activation path. With ReLU6,
            // min(affine, limit) exceeds signed34 range only when both positive
            // operands exceed it. Without ReLU6, signed extension is exact.
            range_affine_sign_calc[clamp_lane] =
                affine_value_q[clamp_lane][41];
            range_signed_bad_lo_calc[clamp_lane] =
                |(affine_value_q[clamp_lane][37:34]
                ^ {4{affine_value_q[clamp_lane][33]}});
            range_signed_bad_hi_calc[clamp_lane] =
                |(affine_value_q[clamp_lane][41:38]
                ^ {4{affine_value_q[clamp_lane][33]}});
            range_affine_pos_bad_lo_calc[clamp_lane] =
                !affine_value_q[clamp_lane][41]
                && (|affine_value_q[clamp_lane][36:33]);
            range_affine_pos_bad_hi_calc[clamp_lane] =
                !affine_value_q[clamp_lane][41]
                && (|affine_value_q[clamp_lane][40:37]);
            range_limit_pos_bad_lo_calc[clamp_lane] =
                1'b0;
            range_limit_pos_bad_hi_calc[clamp_lane] =
                1'b0;
        end
    end

    // ---------------------------------------------------------------------
    // Narrow activation selection stage.
    // ---------------------------------------------------------------------
    reg        activation_valid_q;
    reg [7:0]  activation_lane_valid_q;
    reg signed [33:0] activation_value_q [0:LANES-1];
    reg [LANES*16-1:0] activation_multiplier_q;
    reg [3:0]   activation_multiplier_shift_q;
    reg [7:0]   activation_output_zero_q;
    reg signed [32:0] activation_shortcut_product_q [0:LANES-1];

    reg signed [33:0] activation_calc [0:LANES-1];
    reg [LANES-1:0] activation_use_limit_calc;
    reg [LANES-1:0] activation_upper_gt_calc;
    reg [LANES-1:0] activation_upper_eq_calc;
    reg [LANES-1:0] activation_lower_gt_calc;
    reg                 range_diag_valid_q;
    reg [LANES-1:0]     range_diag_lane_valid_q;
    reg [LANES-1:0]     range_diag_relu6_q;
    reg [LANES-1:0]     range_signed_bad_lo_q;
    reg [LANES-1:0]     range_signed_bad_hi_q;
    reg [LANES-1:0]     range_affine_pos_bad_lo_q;
    reg [LANES-1:0]     range_affine_pos_bad_hi_q;
    reg [LANES-1:0]     range_limit_pos_bad_lo_q;
    reg [LANES-1:0]     range_limit_pos_bad_hi_q;
    reg [LANES-1:0]     range_diag_affine_sign_q;
    reg [LANES-1:0]     activation_range_current_calc;
    reg [LANES-1:0] activation_range_bad_q;
    integer activation_lane;
    always @(*) begin
        activation_use_limit_calc = {LANES{1'b0}};
        activation_upper_gt_calc = {LANES{1'b0}};
        activation_upper_eq_calc = {LANES{1'b0}};
        activation_lower_gt_calc = {LANES{1'b0}};
        activation_range_current_calc =
            (((range_diag_relu6_q
                    & (range_affine_pos_bad_lo_q
                        | range_affine_pos_bad_hi_q)
                    & (range_limit_pos_bad_lo_q
                        | range_limit_pos_bad_hi_q))
                | (~range_diag_relu6_q
                    & (range_signed_bad_lo_q
                        | range_signed_bad_hi_q)))
                & range_diag_lane_valid_q);
        for (activation_lane = 0; activation_lane < LANES;
             activation_lane = activation_lane + 1) begin
            activation_upper_gt_calc[activation_lane] =
                clamp_compare_gt_q[activation_lane*6 + 5]
                || (clamp_compare_eq_q[activation_lane*6 + 5]
                    && clamp_compare_gt_q[activation_lane*6 + 4])
                || (clamp_compare_eq_q[activation_lane*6 + 5]
                    && clamp_compare_eq_q[activation_lane*6 + 4]
                    && clamp_compare_gt_q[activation_lane*6 + 3]);
            activation_upper_eq_calc[activation_lane] =
                clamp_compare_eq_q[activation_lane*6 + 5]
                && clamp_compare_eq_q[activation_lane*6 + 4]
                && clamp_compare_eq_q[activation_lane*6 + 3];
            activation_lower_gt_calc[activation_lane] =
                clamp_compare_gt_q[activation_lane*6 + 2]
                || (clamp_compare_eq_q[activation_lane*6 + 2]
                    && clamp_compare_gt_q[activation_lane*6 + 1])
                || (clamp_compare_eq_q[activation_lane*6 + 2]
                    && clamp_compare_eq_q[activation_lane*6 + 1]
                    && clamp_compare_gt_q[activation_lane*6]);
            activation_use_limit_calc[activation_lane] =
                activation_upper_gt_calc[activation_lane]
                || (activation_upper_eq_calc[activation_lane]
                    && activation_lower_gt_calc[activation_lane]);
            if (clamp_force_zero_q[activation_lane])
                activation_calc[activation_lane] = 34'sd0;
            else if (activation_range_current_calc[activation_lane]) begin
                if (range_diag_relu6_q[activation_lane]
                        || !range_diag_affine_sign_q[activation_lane])
                    activation_calc[activation_lane] = 34'sd8589934591;
                else
                    activation_calc[activation_lane] = 34'sh200000000;
            end
            else if (activation_use_limit_calc[activation_lane])
                activation_calc[activation_lane] =
                    clamp_limit_q;
            else
                activation_calc[activation_lane] =
                    clamp_affine_q[activation_lane];
        end
    end

    // ---------------------------------------------------------------------
    // Main multiplier bank and its aligned shortcut/output metadata.
    // ---------------------------------------------------------------------
    reg signed [33:0] main_mult_dataa [0:LANES-1];
    reg        [15:0] main_mult_datab [0:LANES-1];
    wire       [49:0] main_mult_result[0:LANES-1];

    integer main_issue_lane;
    always @(*) begin
        for (main_issue_lane = 0; main_issue_lane < LANES;
             main_issue_lane = main_issue_lane + 1) begin
            main_mult_dataa[main_issue_lane] =
                activation_valid_q
                ? activation_value_q[main_issue_lane] : 34'sd0;
            main_mult_datab[main_issue_lane] =
                activation_valid_q
                ? activation_multiplier_q[main_issue_lane*16 +: 16]
                : 16'd0;
        end
    end

    genvar main_lane;
    generate
        for (main_lane = 0; main_lane < LANES;
             main_lane = main_lane + 1) begin : g_main_multiplier
            npu_mult34s16u_pq u_main_multiplier (
                .clock0(clk),
                .dataa_0(main_mult_dataa[main_lane]),
                .datab_0(main_mult_datab[main_lane]),
                .ena0(1'b1),
                .result(main_mult_result[main_lane])
            );
        end
    endgenerate

    reg [META_DELAY-1:0] main_valid_pipe;
    reg [7:0]  main_lane_valid_pipe [0:META_DELAY-1];
    reg [7:0] main_output_zero_pipe [0:META_DELAY-1];
    reg [3:0]  main_multiplier_shift_pipe [0:META_DELAY-1];
    reg signed [32:0] main_shortcut_product_pipe
        [0:META_DELAY-1] [0:LANES-1];

    // ---------------------------------------------------------------------
    // Q1.14 round stage.
    // ---------------------------------------------------------------------
    function automatic signed [32:0] round_q14;
        input signed [32:0] value;
        reg signed [33:0] wide;
        reg signed [33:0] magnitude;
        reg signed [33:0] shifted;
        begin
            wide = {value[32], value};
            if (wide < 0) begin
                magnitude = -wide;
                shifted = (magnitude + 34'sd8192) >>> 14;
                round_q14 = -$signed(shifted[32:0]);
            end else begin
                shifted = (wide + 34'sd8192) >>> 14;
                round_q14 = $signed(shifted[32:0]);
            end
        end
    endfunction

    function automatic signed [49:0] round_scaled_wide;
        input signed [49:0] value;
        input integer shift;
        reg signed [50:0] wide;
        reg signed [50:0] magnitude;
        reg signed [50:0] shifted;
        reg signed [50:0] rounding_bias;
        begin
            wide = {value[49], value};
            rounding_bias = 51'sd1 <<< (shift - 1);
            if (wide < 0) begin
                magnitude = -wide;
                shifted = (magnitude + rounding_bias) >>> shift;
                round_scaled_wide = -$signed(shifted[49:0]);
            end else begin
                shifted = (wide + rounding_bias) >>> shift;
                round_scaled_wide = $signed(shifted[49:0]);
            end
        end
    endfunction

    reg        round_valid_q;
    reg [7:0]  round_lane_valid_q;
    reg [7:0] round_output_zero_q;
    reg signed [49:0] round_main_scaled_q [0:LANES-1];
    reg signed [32:0] round_shortcut_scaled_q [0:LANES-1];

    // 两条分支分别量化饱和，再做残差加法，与冻结软件契约一致。
    // 先作常量范围判断，避免在饱和比较前增加50位加法器。
    function automatic [7:0] branch_code;
        input signed [49:0] value;
        input [7:0] zero_point;
        reg signed [9:0] code;
        begin
            if (value < -50'sd255) branch_code = 8'd0;
            else if (value > 50'sd255) branch_code = 8'd255;
            else begin
                code = $signed({value[8], value[8:0]})
                     + $signed({2'b00, zero_point});
                if (code < 0) branch_code = 8'd0;
                else if (code > 10'sd255) branch_code = 8'd255;
                else branch_code = code[7:0];
            end
        end
    endfunction

    reg        final_sum_valid_q;
    reg [7:0]  final_sum_lane_valid_q;
    // main/sc各为U8，main+sc-zero范围[-255,510]，10位有符号足够。
    reg signed [9:0] final_sum_q [0:LANES-1];
    reg [7:0] final_q_calc [0:LANES-1];
    reg signed [9:0] final_sum_calc [0:LANES-1];
    reg [7:0] main_code_calc [0:LANES-1];
    reg [7:0] shortcut_code_calc [0:LANES-1];
    integer final_lane;
    always @(*) begin
        for (final_lane = 0; final_lane < LANES;
             final_lane = final_lane + 1) begin
            main_code_calc[final_lane] = branch_code(
                round_main_scaled_q[final_lane], round_output_zero_q);
            // 无shortcut时round_shortcut为0，branch_code(0,z)=z，自动抵消。
            shortcut_code_calc[final_lane] = branch_code(
                $signed({{17{round_shortcut_scaled_q[final_lane][32]}},
                           round_shortcut_scaled_q[final_lane]}), round_output_zero_q);
            final_sum_calc[final_lane] =
                $signed({2'b00, main_code_calc[final_lane]})
              + $signed({2'b00, shortcut_code_calc[final_lane]})
              - $signed({2'b00, round_output_zero_q});
            if (final_sum_q[final_lane] < 0)
                final_q_calc[final_lane] = 8'd0;
            else if (final_sum_q[final_lane] > 10'sd255)
                final_q_calc[final_lane] = 8'd255;
            else final_q_calc[final_lane] = final_sum_q[final_lane][7:0];
        end
    end

    assign fifo_push = final_sum_valid_q;

    // ---------------------------------------------------------------------
    // Fixed pipeline, feeder, reservation, and FIFO state.
    // ---------------------------------------------------------------------
    integer seq_lane;
    integer seq_stage;
    integer data_lane;
    integer data_stage;
    integer data_source_lane;

    // Wide metadata is qualified by pre_valid_pipe/main_valid_pipe, so it may
    // continue shifting while clear resets only the narrow valid tokens.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            if (RESET_WIDE_META != 0) begin
                for (data_stage = 0; data_stage < META_DELAY;
                     data_stage = data_stage + 1) begin
                    pre_lane_valid[data_stage] <= 0;
                    pre_acc_pre_corrected[data_stage] <= 0;
                    pre_acc[data_stage] <= 0;
                    pre_sum_qd[data_stage] <= 0;
                    pre_weight_zero[data_stage] <= 0;
                    pre_bias[data_stage] <= 0;
                    pre_relu6_en[data_stage] <= 0;
                    pre_relu6_limit[data_stage] <= 0;
                    pre_multiplier[data_stage] <= 0;
                    pre_multiplier_shift[data_stage] <= 0;
                    pre_output_zero[data_stage] <= 0;
                    main_lane_valid_pipe[data_stage] <= 0;
                    main_output_zero_pipe[data_stage] <= 0;
                    main_multiplier_shift_pipe[data_stage] <= 0;
                    for (data_lane = 0; data_lane < LANES;
                         data_lane = data_lane + 1)
                        main_shortcut_product_pipe[
                            data_stage
                        ][data_lane] <= 0;
                end
            end
        end else begin
            for (data_stage = 1; data_stage < META_DELAY;
                 data_stage = data_stage + 1) begin
                pre_lane_valid[data_stage] <=
                    pre_lane_valid[data_stage-1];
                pre_acc_pre_corrected[data_stage] <=
                    pre_acc_pre_corrected[data_stage-1];
                pre_acc[data_stage] <= pre_acc[data_stage-1];
                pre_sum_qd[data_stage] <= pre_sum_qd[data_stage-1];
                pre_weight_zero[data_stage] <=
                    pre_weight_zero[data_stage-1];
                pre_bias[data_stage] <= pre_bias[data_stage-1];
                pre_relu6_en[data_stage] <= pre_relu6_en[data_stage-1];
                pre_relu6_limit[data_stage] <=
                    pre_relu6_limit[data_stage-1];
                pre_multiplier[data_stage] <=
                    pre_multiplier[data_stage-1];
                pre_multiplier_shift[data_stage] <=
                    pre_multiplier_shift[data_stage-1];
                pre_output_zero[data_stage] <=
                    pre_output_zero[data_stage-1];
            end

            for (data_lane = 0; data_lane < LANES;
                 data_lane = data_lane + 1) begin
                data_source_lane = ((GROUPS_PER_BATCH == 1)
                    ? 0 : (batch_group_lane_q[data_lane] ? LANES : 0))
                    + data_lane;
                pre_lane_valid[0][data_lane] <=
                    batch_lane_valid_q[data_source_lane];
                pre_acc_pre_corrected[0][data_lane] <=
                    batch_acc_pre_corrected_q[data_source_lane];
                pre_acc[0][data_lane*32 +: 32] <=
                    batch_acc_q[data_source_lane*32 +: 32];
                pre_sum_qd[0][data_lane*32 +: 32] <=
                    batch_sum_qd_q[data_source_lane*32 +: 32];
                pre_bias[0][data_lane*32 +: 32] <=
                    batch_bias_q[data_source_lane[3:1]*32 +: 32];
            end
            pre_weight_zero[0] <= batch_weight_zero_q;
            pre_relu6_en[0] <= batch_relu6_en_q;
            pre_relu6_limit[0] <= batch_relu6_limit_q;
            pre_multiplier_shift[0] <= batch_multiplier_shift_q;
            for (data_lane = 0; data_lane < LANES;
                 data_lane = data_lane + 1) begin
                data_source_lane = ((GROUPS_PER_BATCH == 1)
                    ? 0 : (batch_group_lane_q[data_lane] ? LANES : 0))
                    + data_lane;
                pre_multiplier[0][data_lane*16 +: 16] <=
                    batch_multiplier_q[data_source_lane*16 +: 16];
            end
            pre_output_zero[0] <= batch_output_zero_q;

            main_lane_valid_pipe[0] <= activation_lane_valid_q;
            main_output_zero_pipe[0] <= activation_output_zero_q;
            main_multiplier_shift_pipe[0] <= activation_multiplier_shift_q;
            for (data_lane = 0; data_lane < LANES;
                 data_lane = data_lane + 1)
                main_shortcut_product_pipe[0][data_lane] <=
                    activation_shortcut_product_q[data_lane];

            for (data_stage = 1; data_stage < META_DELAY;
                 data_stage = data_stage + 1) begin
                main_lane_valid_pipe[data_stage] <=
                    main_lane_valid_pipe[data_stage-1];
                main_output_zero_pipe[data_stage] <=
                    main_output_zero_pipe[data_stage-1];
                main_multiplier_shift_pipe[data_stage] <=
                    main_multiplier_shift_pipe[data_stage-1];
                for (data_lane = 0; data_lane < LANES;
                     data_lane = data_lane + 1)
                    main_shortcut_product_pipe[
                        data_stage
                    ][data_lane] <= main_shortcut_product_pipe[
                        data_stage-1
                    ][data_lane];
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            batch_valid_q <= 1'b0;
            batch_group_lane_q <= {LANES{1'b0}};

            pre_valid_pipe <= 0;
            correction_valid_q <= 1'b0;
            affine_valid_q <= 1'b0;
            clamp_valid_q <= 1'b0;
            activation_valid_q <= 1'b0;
            main_valid_pipe <= 0;
            round_valid_q <= 1'b0;
            final_sum_valid_q <= 1'b0;

            fifo_wr_ptr <= 0;
            fifo_rd_ptr <= 0;
            fifo_count <= 0;
            reserved_groups <= 0;
            fifo_space_available_q <= 1'b1;
            feeder_accept_credit_q <= 1'b1;
            status_error <= 1'b0;
            debug_error_code <= 10'd0;
            activation_range_bad_q <= 0;
            range_diag_valid_q <= 1'b0;
            range_diag_lane_valid_q <= 0;
            range_diag_relu6_q <= 0;
            range_signed_bad_lo_q <= 0;
            range_signed_bad_hi_q <= 0;
            range_affine_pos_bad_lo_q <= 0;
            range_affine_pos_bad_hi_q <= 0;
            range_limit_pos_bad_lo_q <= 0;
            range_limit_pos_bad_hi_q <= 0;
            range_diag_affine_sign_q <= 0;

            correction_lane_valid_q <= 0;
            correction_bias_q <= 0;
            correction_relu6_en_q <= 0;
            correction_relu6_limit_q <= 0;
            correction_multiplier_q <= 0;
            correction_multiplier_shift_q <= 0;
            correction_output_zero_q <= 0;
            affine_lane_valid_q <= 0;
            affine_relu6_en_q <= 0;
            affine_relu6_limit_q <= 0;
            affine_multiplier_q <= 0;
            affine_multiplier_shift_q <= 0;
            affine_output_zero_q <= 0;
            clamp_lane_valid_q <= 0;
            clamp_force_zero_q <= 0;
            clamp_compare_gt_q <= 0;
            clamp_compare_eq_q <= 0;
            clamp_multiplier_q <= 0;
            clamp_multiplier_shift_q <= 0;
            clamp_output_zero_q <= 0;
            activation_lane_valid_q <= 0;
            activation_multiplier_q <= 0;
            activation_multiplier_shift_q <= 0;
            activation_output_zero_q <= 0;
            round_lane_valid_q <= 0;
            round_output_zero_q <= 0;
            final_sum_lane_valid_q <= 0;

            for (seq_lane = 0; seq_lane < LANES;
                 seq_lane = seq_lane + 1) begin
                affine_value_q[seq_lane] <= 0;
                affine_shortcut_product_q[seq_lane] <= 0;
                clamp_affine_q[seq_lane] <= 0;
                clamp_limit_q <= 0;
                clamp_shortcut_product_q[seq_lane] <= 0;
                activation_value_q[seq_lane] <= 0;
                activation_shortcut_product_q[seq_lane] <= 0;
                round_main_scaled_q[seq_lane] <= 0;
                round_shortcut_scaled_q[seq_lane] <= 0;
                final_sum_q[seq_lane] <= 0;
                correction_value_q[seq_lane] <= 0;
                correction_shortcut_product_q[seq_lane] <= 0;
                fifo_valid[seq_lane] <= 0;
                fifo_q[seq_lane] <= 0;
            end
        end else if (clear) begin
            batch_valid_q <= 1'b0;
            batch_group_lane_q <= {LANES{1'b0}};

            pre_valid_pipe <= 0;
            correction_valid_q <= 1'b0;
            affine_valid_q <= 1'b0;
            clamp_valid_q <= 1'b0;
            activation_valid_q <= 1'b0;
            main_valid_pipe <= 0;
            round_valid_q <= 1'b0;
            final_sum_valid_q <= 1'b0;

            fifo_wr_ptr <= 0;
            fifo_rd_ptr <= 0;
            fifo_count <= 0;
            reserved_groups <= 0;
            fifo_space_available_q <= 1'b1;
            feeder_accept_credit_q <= 1'b1;
            status_error <= 1'b0;
            debug_error_code <= 10'd0;
            activation_range_bad_q <= 0;
            range_diag_valid_q <= 1'b0;
            range_diag_lane_valid_q <= 0;
            range_diag_relu6_q <= 0;
            range_signed_bad_lo_q <= 0;
            range_signed_bad_hi_q <= 0;
            range_affine_pos_bad_lo_q <= 0;
            range_affine_pos_bad_hi_q <= 0;
            range_limit_pos_bad_lo_q <= 0;
            range_limit_pos_bad_hi_q <= 0;
            range_diag_affine_sign_q <= 0;

            correction_lane_valid_q <= 0;
            affine_lane_valid_q <= 0;
            clamp_lane_valid_q <= 0;
            activation_lane_valid_q <= 0;
            round_lane_valid_q <= 0;
            final_sum_lane_valid_q <= 0;
        end else begin
            range_diag_valid_q <= affine_valid_q;
            range_diag_lane_valid_q <= affine_lane_valid_q;
            range_diag_relu6_q <= {LANES{affine_relu6_en_q}};
            range_signed_bad_lo_q <= range_signed_bad_lo_calc;
            range_signed_bad_hi_q <= range_signed_bad_hi_calc;
            range_affine_pos_bad_lo_q <= range_affine_pos_bad_lo_calc;
            range_affine_pos_bad_hi_q <= range_affine_pos_bad_hi_calc;
            range_limit_pos_bad_lo_q <= range_limit_pos_bad_lo_calc;
            range_limit_pos_bad_hi_q <= range_limit_pos_bad_hi_calc;
            range_diag_affine_sign_q <= range_affine_sign_calc;
            activation_range_bad_q <= activation_range_current_calc;
            // Values beyond the native signed-18 DSP input are saturated by
            // the functional selector above. Keep the event visible without
            // converting that deterministic fallback into a fatal error.
            if (debug_error_code == 10'd0) begin
                if (|activation_range_bad_q)
                    debug_error_code <= 10'd631;
                else if (fifo_push && (reserved_groups == 0))
                    debug_error_code <= 10'd632;
                else if (fifo_push && (fifo_count >= FIFO_DEPTH)
                        && !fifo_pop)
                    debug_error_code <= 10'd633;
                else if (accept_batch && sum_qd_width_bad)
                    debug_error_code <= 10'd634;
                else if (accept_batch && multiplier_sign_bad)
                    debug_error_code <= 10'd635;
            end

            fifo_space_available_q <= committed_groups <=
                (FIFO_DEPTH - GROUPS_PER_BATCH);
            // A batch occupies the two-group feeder for exactly one cycle
            // after acceptance. This token preserves one batch every two
            // cycles while removing batch state from the in_ready path.
            if (GROUPS_PER_BATCH == 1)
                feeder_accept_credit_q <= 1'b1;
            else
                feeder_accept_credit_q <= !accept_batch;

            // Narrow FIFO occupancy.
            case ({fifo_push, fifo_pop})
                2'b10: fifo_count <= fifo_count + 1'b1;
                2'b01: fifo_count <= fifo_count - 1'b1;
                default: ;
            endcase
            if (fifo_pop) begin
                if (fifo_rd_ptr == FIFO_DEPTH - 1)
                    fifo_rd_ptr <= 0;
                else
                    fifo_rd_ptr <= fifo_rd_ptr + 1'b1;
            end
            if (fifo_push) begin
                fifo_valid[fifo_wr_ptr] <= final_sum_lane_valid_q;
                for (seq_lane = 0; seq_lane < LANES;
                     seq_lane = seq_lane + 1)
                    fifo_q[fifo_wr_ptr][seq_lane*8 +: 8] <=
                        final_q_calc[seq_lane];
                if (fifo_wr_ptr == FIFO_DEPTH - 1)
                    fifo_wr_ptr <= 0;
                else
                    fifo_wr_ptr <= fifo_wr_ptr + 1'b1;
            end

            // Reserve one entry for a fixed slice or two for the legacy
            // 16-lane/two-group interface.
            case ({accept_batch, fifo_push})
                2'b10: reserved_groups <= reserved_groups
                    + GROUPS_PER_BATCH;
                2'b01: reserved_groups <= reserved_groups - 1'b1;
                2'b11: reserved_groups <= reserved_groups
                    + GROUPS_PER_BATCH - 1'b1;
                default: ;
            endcase

            if (fifo_push && (reserved_groups == 0))
                status_error <= 1'b1;
            if (fifo_push && (fifo_count >= FIFO_DEPTH) && !fifo_pop)
                status_error <= 1'b1;

            // Batch feeder. Current group is sampled by the multiplier IPs on
            // this edge before a replacement batch is loaded.
            if (GROUPS_PER_BATCH == 1) begin
                batch_group_lane_q <= {LANES{1'b0}};
                if (batch_valid_q) begin
                    if (accept_batch)
                        batch_valid_q <= 1'b1;
                    else
                        batch_valid_q <= 1'b0;
                end else if (accept_batch) begin
                    batch_valid_q <= 1'b1;
                end
            end else begin
                if (batch_valid_q) begin
                    if (!batch_group_q) begin
                        batch_group_lane_q <= {LANES{1'b1}};
                    end else if (accept_batch) begin
                        batch_valid_q <= 1'b1;
                        batch_group_lane_q <= {LANES{1'b0}};
                    end else begin
                        batch_valid_q <= 1'b0;
                        batch_group_lane_q <= {LANES{1'b0}};
                    end
                end else if (accept_batch) begin
                    batch_valid_q <= 1'b1;
                    batch_group_lane_q <= {LANES{1'b0}};
                end
            end

            if (accept_batch) begin
                for (seq_lane = 0; seq_lane < 16;
                     seq_lane = seq_lane + 1) begin
                    if (in_sum_qd_bus[seq_lane*32 + 17 +: 15] != 0)
                        status_error <= 1'b1;
                end
                if (multiplier_sign_bad)
                    status_error <= 1'b1;
            end

            // Metadata aligned to correction and shortcut DSP latency.
            pre_valid_pipe[0] <= issue_valid;
            for (seq_stage = 1; seq_stage < META_DELAY;
                 seq_stage = seq_stage + 1)
                pre_valid_pipe[seq_stage] <=
                    pre_valid_pipe[seq_stage-1];

            // Register zero-point correction before the bias addition.
            correction_valid_q <= pre_valid_pipe[META_DELAY-1];
            correction_lane_valid_q <= pre_lane_valid[META_DELAY-1];
            correction_bias_q <= pre_bias[META_DELAY-1];
            correction_relu6_en_q <= pre_relu6_en[META_DELAY-1];
            correction_relu6_limit_q <= pre_relu6_limit[META_DELAY-1];
            correction_multiplier_q <= pre_multiplier[META_DELAY-1];
            correction_multiplier_shift_q <=
                pre_multiplier_shift[META_DELAY-1];
            correction_output_zero_q <= pre_output_zero[META_DELAY-1];
            for (seq_lane = 0; seq_lane < LANES;
                 seq_lane = seq_lane + 1) begin
                correction_value_q[seq_lane] <= corrected_calc[seq_lane];
                correction_shortcut_product_q[seq_lane] <=
                    $signed(sc_mult_result[seq_lane]);
            end

            // Add q_b_f in its own registered stage.
            affine_valid_q <= correction_valid_q;
            affine_lane_valid_q <= correction_lane_valid_q;
            affine_relu6_en_q <= correction_relu6_en_q;
            affine_relu6_limit_q <= correction_relu6_limit_q;
            affine_multiplier_q <= correction_multiplier_q;
            affine_multiplier_shift_q <= correction_multiplier_shift_q;
            affine_output_zero_q <= correction_output_zero_q;
            for (seq_lane = 0; seq_lane < LANES;
                 seq_lane = seq_lane + 1) begin
                affine_value_q[seq_lane] <= affine_calc[seq_lane];
                affine_shortcut_product_q[seq_lane] <=
                    correction_shortcut_product_q[seq_lane];
            end

            // Register independent 7-bit comparison chunks. The following
            // activation stage merges them most-significant chunk first.
            clamp_valid_q <= affine_valid_q;
            clamp_lane_valid_q <= affine_lane_valid_q;
            clamp_force_zero_q <= clamp_force_zero_calc;
            clamp_compare_gt_q <= clamp_compare_gt_calc;
            clamp_compare_eq_q <= clamp_compare_eq_calc;
            clamp_multiplier_q <= affine_multiplier_q;
            clamp_multiplier_shift_q <= affine_multiplier_shift_q;
            clamp_output_zero_q <= affine_output_zero_q;
            for (seq_lane = 0; seq_lane < LANES;
                 seq_lane = seq_lane + 1) begin
                clamp_affine_q[seq_lane] <=
                    affine_value_q[seq_lane][33:0];
                clamp_shortcut_product_q[seq_lane] <=
                    affine_shortcut_product_q[seq_lane];
            end
            clamp_limit_q <= {{2{affine_relu6_limit_q[31]}}, affine_relu6_limit_q};

            // Select between the registered 34-bit candidates.
            activation_valid_q <= clamp_valid_q;
            activation_lane_valid_q <= clamp_lane_valid_q;
            activation_multiplier_q <= clamp_multiplier_q;
            activation_multiplier_shift_q <= clamp_multiplier_shift_q;
            activation_output_zero_q <= clamp_output_zero_q;
            for (seq_lane = 0; seq_lane < LANES;
                 seq_lane = seq_lane + 1) begin
                activation_value_q[seq_lane] <= activation_calc[seq_lane];
                activation_shortcut_product_q[seq_lane] <=
                    clamp_shortcut_product_q[seq_lane];
            end

            // Metadata aligned with the main DSP bank.
            main_valid_pipe[0] <= activation_valid_q;
            for (seq_stage = 1; seq_stage < META_DELAY;
                 seq_stage = seq_stage + 1)
                main_valid_pipe[seq_stage] <=
                    main_valid_pipe[seq_stage-1];

            // Symmetric round-to-nearest, exact ties away from zero.
            round_valid_q <= main_valid_pipe[META_DELAY-1];
            round_lane_valid_q <= main_lane_valid_pipe[META_DELAY-1];
            round_output_zero_q <= main_output_zero_pipe[META_DELAY-1];
            for (seq_lane = 0; seq_lane < LANES;
                 seq_lane = seq_lane + 1) begin
                round_main_scaled_q[seq_lane] <=
                    round_scaled_wide(
                        $signed(main_mult_result[seq_lane]),
                        main_multiplier_shift_pipe[META_DELAY-1]
                    );
                round_shortcut_scaled_q[seq_lane] <=
                    round_q14(
                        main_shortcut_product_pipe[META_DELAY-1][seq_lane]
                    );
            end

            // Register the wide add before saturation and FIFO write.
            final_sum_valid_q <= round_valid_q;
            final_sum_lane_valid_q <= round_lane_valid_q;
            for (seq_lane = 0; seq_lane < LANES;
                 seq_lane = seq_lane + 1)
                final_sum_q[seq_lane] <= final_sum_calc[seq_lane];
        end
    end

endmodule

`default_nettype wire
