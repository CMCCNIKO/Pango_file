//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: head专用16lane×8term UINT8 MAC事务累加器，扩展为覆盖64通道3×3卷积最多576项的累加范围。
// Module Name: f14_head_u8_mac16x8_dsp_pipeline
// File Name: model/rtl/f14_sop_u8_mac16x8_dsp_pipeline_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// head专用16lane×8term UINT8 MAC事务累加器，扩展为覆盖64通道3×3卷积最多576项的累加范围。
// 分别积累sum(qd×qw)与sum(qd)，默认26位raw和18位qd，再零扩展到每lane32位输出；固定DSP延迟与元数据对齐。
//
// Dependencies: npu_multadd4_u8；f14_head_u8_mac16x8_dsp_pipeline
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 保留FIXED_THROUGHPUT和双本地结果额度；该模块名与骨干MAC分开，避免两个不同累加范围的实现冲突。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// F14 isolated adaptation: maximum576 U8 terms; original SOP is unchanged.
`timescale 1ns/1ps
`default_nettype none

// Physical 16-lane by 8-term UINT8 MAC transaction accumulator.
//
// Each lane uses two npu_multadd4_u8 instances. The generated IP has a
// measured two-edge latency and accepts one vector per enabled cycle. The
// surrounding metadata and qd path are delayed to the same transaction, then
// registered once more before the 32-bit transaction accumulator.
//
// Bus order is lane-major. For lane L and term T:
//   flat index = L * 8 + T
//   qd byte    = qd[(flat index)*8 +: 8]
//   qw byte    = qw[(flat index)*8 +: 8]
//
// Completed transactions use the output registers plus the accumulator bank
// as two local result credits. Downstream ready therefore does not directly
// gate the DSP pipeline; the pipeline stops only when both credits are full.
// Accumulation wraps modulo 2^32. Saturation and affine output quantization
// remain outside this module.
// Head-only widened MAC.  Keep the production backbone's original SOP module
// name free so the integrated design can compile both contracts side by side.
module f14_head_u8_mac16x8_dsp_pipeline #(
    parameter integer PREZEROED_OPERANDS = 0,
    parameter integer FIXED_THROUGHPUT = 0,
    parameter integer RAW_ACC_WIDTH = 26,
    parameter integer QD_ACC_WIDTH = 18
) (
    input  wire          clk,
    input  wire          rst_n,

    input  wire          uop_valid,
    output wire          uop_ready,
    input  wire          uop_first,
    input  wire          uop_last,
    input  wire [15:0]   lane_valid,
    input  wire [127:0]  term_valid,
    input  wire [1023:0] qd,
    input  wire [1023:0] qw,

    output reg           result_valid,
    input  wire          result_ready,
    output reg  [15:0]   result_lane_valid,
    output reg  [511:0]  raw_sum,
    output reg  [511:0]  sum_qd
);

    localparam integer LANES = 16;
    localparam integer TERMS = 8;
    localparam integer PAIRS = LANES * 4;
    localparam integer QUADS = LANES * 2;
    localparam integer OUTPUT_WIDTH = 32;
    // F14 first head:64 input channels*3*3=576 terms (72 C8 uops).
    // raw <=37,454,400 needs26 bits; sum_qd<=146,880 needs18 bits.
    // Build preflight requires26<=RAW_ACC_WIDTH<=32,18<=QD_ACC_WIDTH<=32.

    reg final_pending;
    reg [LANES-1:0] final_lane_valid;
    wire result_slot_ready;
    wire final_to_result;
    wire pipeline_advance;
    wire dsp_enable;
    assign result_slot_ready = !result_valid || result_ready;
    assign final_to_result = final_pending && result_slot_ready;
    assign pipeline_advance = FIXED_THROUGHPUT != 0
        ? 1'b1 : !(result_valid && final_pending);
    assign dsp_enable = FIXED_THROUGHPUT != 0
        ? 1'b1 : rst_n && pipeline_advance;
    assign uop_ready = rst_n && pipeline_advance;

    wire [17:0] dsp_group0 [0:LANES-1];
    wire [17:0] dsp_group1 [0:LANES-1];

    genvar lane_gen;
    generate
        for (lane_gen = 0; lane_gen < LANES; lane_gen = lane_gen + 1) begin : g_lane_dsp
            wire lane_accept;
            wire [7:0] qd_0;
            wire [7:0] qd_1;
            wire [7:0] qd_2;
            wire [7:0] qd_3;
            wire [7:0] qd_4;
            wire [7:0] qd_5;
            wire [7:0] qd_6;
            wire [7:0] qd_7;
            wire [7:0] qw_0;
            wire [7:0] qw_1;
            wire [7:0] qw_2;
            wire [7:0] qw_3;
            wire [7:0] qw_4;
            wire [7:0] qw_5;
            wire [7:0] qw_6;
            wire [7:0] qw_7;

            // The fixed fused feeder already writes zero into every inactive
            // operand slot. Its specialized instance can therefore bypass
            // these 256 masks; the generic instance retains them.
            assign lane_accept = lane_valid[lane_gen];
            if (PREZEROED_OPERANDS != 0) begin : g_prezeroed_operands
                assign qd_0 = qd[(lane_gen*TERMS + 0)*8 +: 8];
                assign qd_1 = qd[(lane_gen*TERMS + 1)*8 +: 8];
                assign qd_2 = qd[(lane_gen*TERMS + 2)*8 +: 8];
                assign qd_3 = qd[(lane_gen*TERMS + 3)*8 +: 8];
                assign qd_4 = qd[(lane_gen*TERMS + 4)*8 +: 8];
                assign qd_5 = qd[(lane_gen*TERMS + 5)*8 +: 8];
                assign qd_6 = qd[(lane_gen*TERMS + 6)*8 +: 8];
                assign qd_7 = qd[(lane_gen*TERMS + 7)*8 +: 8];
                assign qw_0 = qw[(lane_gen*TERMS + 0)*8 +: 8];
                assign qw_1 = qw[(lane_gen*TERMS + 1)*8 +: 8];
                assign qw_2 = qw[(lane_gen*TERMS + 2)*8 +: 8];
                assign qw_3 = qw[(lane_gen*TERMS + 3)*8 +: 8];
                assign qw_4 = qw[(lane_gen*TERMS + 4)*8 +: 8];
                assign qw_5 = qw[(lane_gen*TERMS + 5)*8 +: 8];
                assign qw_6 = qw[(lane_gen*TERMS + 6)*8 +: 8];
                assign qw_7 = qw[(lane_gen*TERMS + 7)*8 +: 8];
            end else begin : g_masked_operands
                assign qd_0 =
                    lane_accept && term_valid[lane_gen*TERMS + 0]
                    ? qd[(lane_gen*TERMS + 0)*8 +: 8] : 8'd0;
                assign qd_1 =
                    lane_accept && term_valid[lane_gen*TERMS + 1]
                    ? qd[(lane_gen*TERMS + 1)*8 +: 8] : 8'd0;
                assign qd_2 =
                    lane_accept && term_valid[lane_gen*TERMS + 2]
                    ? qd[(lane_gen*TERMS + 2)*8 +: 8] : 8'd0;
                assign qd_3 =
                    lane_accept && term_valid[lane_gen*TERMS + 3]
                    ? qd[(lane_gen*TERMS + 3)*8 +: 8] : 8'd0;
                assign qd_4 =
                    lane_accept && term_valid[lane_gen*TERMS + 4]
                    ? qd[(lane_gen*TERMS + 4)*8 +: 8] : 8'd0;
                assign qd_5 =
                    lane_accept && term_valid[lane_gen*TERMS + 5]
                    ? qd[(lane_gen*TERMS + 5)*8 +: 8] : 8'd0;
                assign qd_6 =
                    lane_accept && term_valid[lane_gen*TERMS + 6]
                    ? qd[(lane_gen*TERMS + 6)*8 +: 8] : 8'd0;
                assign qd_7 =
                    lane_accept && term_valid[lane_gen*TERMS + 7]
                    ? qd[(lane_gen*TERMS + 7)*8 +: 8] : 8'd0;
                assign qw_0 =
                    lane_accept && term_valid[lane_gen*TERMS + 0]
                    ? qw[(lane_gen*TERMS + 0)*8 +: 8] : 8'd0;
                assign qw_1 =
                    lane_accept && term_valid[lane_gen*TERMS + 1]
                    ? qw[(lane_gen*TERMS + 1)*8 +: 8] : 8'd0;
                assign qw_2 =
                    lane_accept && term_valid[lane_gen*TERMS + 2]
                    ? qw[(lane_gen*TERMS + 2)*8 +: 8] : 8'd0;
                assign qw_3 =
                    lane_accept && term_valid[lane_gen*TERMS + 3]
                    ? qw[(lane_gen*TERMS + 3)*8 +: 8] : 8'd0;
                assign qw_4 =
                    lane_accept && term_valid[lane_gen*TERMS + 4]
                    ? qw[(lane_gen*TERMS + 4)*8 +: 8] : 8'd0;
                assign qw_5 =
                    lane_accept && term_valid[lane_gen*TERMS + 5]
                    ? qw[(lane_gen*TERMS + 5)*8 +: 8] : 8'd0;
                assign qw_6 =
                    lane_accept && term_valid[lane_gen*TERMS + 6]
                    ? qw[(lane_gen*TERMS + 6)*8 +: 8] : 8'd0;
                assign qw_7 =
                    lane_accept && term_valid[lane_gen*TERMS + 7]
                    ? qw[(lane_gen*TERMS + 7)*8 +: 8] : 8'd0;
            end

            npu_multadd4_u8 u_multadd_low (
                .clock0(clk),
                .ena0(dsp_enable),
                .dataa_0(qd_0),
                .dataa_1(qd_1),
                .dataa_2(qd_2),
                .dataa_3(qd_3),
                .datab_0(qw_0),
                .datab_1(qw_1),
                .datab_2(qw_2),
                .datab_3(qw_3),
                .result(dsp_group0[lane_gen])
            );

            npu_multadd4_u8 u_multadd_high (
                .clock0(clk),
                .ena0(dsp_enable),
                .dataa_0(qd_4),
                .dataa_1(qd_5),
                .dataa_2(qd_6),
                .dataa_3(qd_7),
                .datab_0(qw_4),
                .datab_1(qw_5),
                .datab_2(qw_6),
                .datab_3(qw_7),
                .result(dsp_group1[lane_gen])
            );
        end
    endgenerate

    reg s0_valid;
    reg s0_first;
    reg s0_last;
    reg [LANES-1:0] s0_lane_valid;
    reg [8:0] qd_pair_s0 [0:PAIRS-1];
    wire [8:0] qd_pair_input [0:PAIRS-1];

    genvar pair_gen;
    generate
        for (pair_gen = 0; pair_gen < PAIRS;
             pair_gen = pair_gen + 1) begin : g_qd_pair_input
            if (PREZEROED_OPERANDS != 0) begin : g_prezeroed_qd
                assign qd_pair_input[pair_gen] =
                    {1'b0, qd[(pair_gen*2)*8 +: 8]}
                    + {1'b0, qd[(pair_gen*2 + 1)*8 +: 8]};
            end else begin : g_masked_qd
                assign qd_pair_input[pair_gen] =
                    (lane_valid[pair_gen / 4]
                        && term_valid[pair_gen*2]
                        ? {1'b0, qd[(pair_gen*2)*8 +: 8]}
                        : 9'd0)
                    + (lane_valid[pair_gen / 4]
                        && term_valid[pair_gen*2 + 1]
                        ? {1'b0, qd[(pair_gen*2 + 1)*8 +: 8]}
                        : 9'd0);
            end
        end
    endgenerate

    reg s1_valid;
    reg s1_first;
    reg s1_last;
    reg [LANES-1:0] s1_lane_valid;
    reg [9:0] qd_quad_s1 [0:QUADS-1];

    reg s2_valid;
    reg s2_first;
    reg s2_last;
    reg [LANES-1:0] s2_lane_valid;
    reg [10:0] qd_lane_s2 [0:LANES-1];

    reg s3_valid;
    reg s3_first;
    reg s3_last;
    reg [LANES-1:0] s3_lane_valid;
    reg [18:0] raw_lane_s3 [0:LANES-1];
    reg [10:0] qd_lane_s3 [0:LANES-1];

    reg [RAW_ACC_WIDTH-1:0] raw_accum [0:LANES-1];
    reg [QD_ACC_WIDTH-1:0] qd_accum [0:LANES-1];
    reg [LANES-1:0] lane_valid_accum;

    integer lane_index;
    integer pair_index;
    integer quad_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s0_valid <= 1'b0;
            s1_valid <= 1'b0;
            s2_valid <= 1'b0;
            s3_valid <= 1'b0;
            s0_first <= 1'b0;
            s1_first <= 1'b0;
            s2_first <= 1'b0;
            s3_first <= 1'b0;
            s0_last <= 1'b0;
            s1_last <= 1'b0;
            s2_last <= 1'b0;
            s3_last <= 1'b0;
            s0_lane_valid <= {LANES{1'b0}};
            s1_lane_valid <= {LANES{1'b0}};
            s2_lane_valid <= {LANES{1'b0}};
            s3_lane_valid <= {LANES{1'b0}};
            result_valid <= 1'b0;
            result_lane_valid <= {LANES{1'b0}};
            raw_sum <= {LANES*OUTPUT_WIDTH{1'b0}};
            sum_qd <= {LANES*OUTPUT_WIDTH{1'b0}};
            final_pending <= 1'b0;
            final_lane_valid <= {LANES{1'b0}};
            lane_valid_accum <= {LANES{1'b0}};
            for (lane_index = 0;
                 lane_index < LANES;
                 lane_index = lane_index + 1) begin
                raw_accum[lane_index] <= {RAW_ACC_WIDTH{1'b0}};
                qd_accum[lane_index] <= {QD_ACC_WIDTH{1'b0}};
            end
        end else begin
            if (final_to_result) begin
                result_valid <= 1'b1;
                result_lane_valid <= final_lane_valid;
                for (lane_index = 0;
                     lane_index < LANES;
                     lane_index = lane_index + 1) begin
                    if (final_lane_valid[lane_index]) begin
                        raw_sum[
                            lane_index*OUTPUT_WIDTH +: OUTPUT_WIDTH
                        ] <= {
                            {(OUTPUT_WIDTH-RAW_ACC_WIDTH){1'b0}},
                            raw_accum[lane_index]
                        };
                        sum_qd[
                            lane_index*OUTPUT_WIDTH +: OUTPUT_WIDTH
                        ] <= {
                            {(OUTPUT_WIDTH-QD_ACC_WIDTH){1'b0}},
                            qd_accum[lane_index]
                        };
                    end else begin
                        raw_sum[
                            lane_index*OUTPUT_WIDTH +: OUTPUT_WIDTH
                        ] <= {OUTPUT_WIDTH{1'b0}};
                        sum_qd[
                            lane_index*OUTPUT_WIDTH +: OUTPUT_WIDTH
                        ] <= {OUTPUT_WIDTH{1'b0}};
                    end
                end
            end else if (result_valid && result_ready) begin
                result_valid <= 1'b0;
            end

            if (pipeline_advance) begin
                if (s3_valid) begin
                    lane_valid_accum <= s3_first
                        ? s3_lane_valid
                        : lane_valid_accum | s3_lane_valid;

                    for (lane_index = 0;
                         lane_index < LANES;
                         lane_index = lane_index + 1) begin
                        if (s3_first) begin
                            raw_accum[lane_index] <=
                                {{(RAW_ACC_WIDTH-19){1'b0}}, raw_lane_s3[lane_index]};
                            qd_accum[lane_index] <=
                                {{(QD_ACC_WIDTH-11){1'b0}}, qd_lane_s3[lane_index]};
                        end else begin
                            raw_accum[lane_index] <=
                                raw_accum[lane_index]
                                + {{(RAW_ACC_WIDTH-19){1'b0}}, raw_lane_s3[lane_index]};
                            qd_accum[lane_index] <=
                                qd_accum[lane_index]
                                + {{(QD_ACC_WIDTH-11){1'b0}}, qd_lane_s3[lane_index]};
                        end
                    end
                end

                if (s3_valid && s3_last) begin
                    final_pending <= 1'b1;
                    final_lane_valid <= s3_first
                        ? s3_lane_valid
                        : lane_valid_accum | s3_lane_valid;
                end else if (final_to_result) begin
                    final_pending <= 1'b0;
                end

                // The DSP outputs observed here belong to the metadata in S2.
                s3_valid <= s2_valid;
                s3_first <= s2_first;
                s3_last <= s2_last;
                s3_lane_valid <= s2_lane_valid;
                for (lane_index = 0;
                     lane_index < LANES;
                     lane_index = lane_index + 1) begin
                    raw_lane_s3[lane_index]
                        <= {1'b0, dsp_group0[lane_index]}
                        + {1'b0, dsp_group1[lane_index]};
                    qd_lane_s3[lane_index] <= qd_lane_s2[lane_index];
                end

                s2_valid <= s1_valid;
                s2_first <= s1_first;
                s2_last <= s1_last;
                s2_lane_valid <= s1_lane_valid;
                for (lane_index = 0;
                     lane_index < LANES;
                     lane_index = lane_index + 1) begin
                    qd_lane_s2[lane_index]
                        <= {1'b0, qd_quad_s1[lane_index*2]}
                        + {1'b0, qd_quad_s1[lane_index*2 + 1]};
                end

                s1_valid <= s0_valid;
                s1_first <= s0_first;
                s1_last <= s0_last;
                s1_lane_valid <= s0_lane_valid;
                for (quad_index = 0;
                     quad_index < QUADS;
                     quad_index = quad_index + 1) begin
                    qd_quad_s1[quad_index]
                        <= {1'b0, qd_pair_s0[quad_index*2]}
                        + {1'b0, qd_pair_s0[quad_index*2 + 1]};
                end

                s0_valid <= uop_valid;
                s0_first <= uop_first;
                s0_last <= uop_last;
                s0_lane_valid <= lane_valid;
                for (pair_index = 0;
                     pair_index < PAIRS;
                     pair_index = pair_index + 1) begin
                    qd_pair_s0[pair_index] <=
                        qd_pair_input[pair_index];
                end
            end else if (final_to_result) begin
                final_pending <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
