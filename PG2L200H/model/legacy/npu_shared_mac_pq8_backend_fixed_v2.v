//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 组合 16×8 UINT8 MAC 与八 lane 量化流水，把有首尾标记的微操作累加成带目标地址的 128 位结果字。
// Module Name: npu_shared_mac_pq8_backend_fixed_v2
// File Name: model/legacy/npu_shared_mac_pq8_backend_fixed_v2.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 组合 16×8 UINT8 MAC 与八 lane 量化流水，把有首尾标记的微操作累加成带目标地址的 128 位结果字。
// 在事务第一条微操作前预留最终输出额度，内部 MAC/PQ 不受下游 ready 直接停顿；支持 bias、weight-zero 修正、ReLU6、shortcut 与逐通道量化系数。
//
// Dependencies: npu_u8_mac16x8_dsp_pipeline、npu_pq8_stream_pipeline_v2
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 事务元数据沿固定 MAC 延迟传播；输出排队吸收背压，不能把事务中途 ready 理解为可随意取消。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

// Interface-compatible shared backend with a non-stallable MAC/PQ interior.
//
// Backpressure is accepted only before the first uop of a transaction. Once a
// transaction starts, one final 128-bit output slot has already been reserved,
// so bubbles are allowed but downstream ready never propagates into MAC/PQ.
// Wide transaction metadata follows the fixed MAC latency instead of passing
// through an asynchronously indexed register FIFO.
module npu_shared_mac_pq8_backend_fixed_v2 #(
    parameter integer META_DEPTH = 8,
    parameter integer META_PTR_WIDTH = 3,
    parameter integer DEST_ADDR_WIDTH = 11,
    parameter integer PREZEROED_OPERANDS = 0,
    parameter integer MAC_FIXED_THROUGHPUT = 1,
    parameter integer REGISTER_MAC_RESULT_READY = 0,
    parameter integer UNCONDITIONAL_UOP_DATA = 0,
    parameter integer UNCONDITIONAL_UOP_BIAS = 0,
    parameter integer PQ_CAPTURE_ON_ACCEPT = 0,
    parameter integer MAX_OUTSTANDING = 4,
    parameter integer OUTSTANDING_COUNT_WIDTH = 6,
    parameter integer PQ_FIFO_DEPTH = 8,
    parameter integer PQ_FIFO_AW = 3,
    // Enable per-output-channel requantization multipliers for F14.
    parameter integer PER_CHANNEL_MULTIPLIER = 0,
    parameter integer MAIN_MULTIPLIER_SHIFT = 14
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          clear,

    input  wire          uop_valid,
    output wire          uop_ready,
    input  wire          uop_first,
    input  wire          uop_last,
    input  wire [15:0]   uop_lane_valid,
    input  wire [127:0]  uop_term_valid,
    input  wire [1023:0] uop_qd,
    input  wire [1023:0] uop_qw,

    input  wire          txn_meta_valid,
    input  wire [1:0]    txn_dest_kind,
    input  wire [DEST_ADDR_WIDTH-1:0] txn_dest_addr,
    input  wire [255:0]  txn_bias_f8,
    input  wire [127:0]  txn_shortcut_c8p2,

    input  wire [7:0]    cfg_weight_zero,
    input  wire          cfg_relu6_en,
    input  wire [31:0]   cfg_relu6_limit,
    input  wire [15:0]   cfg_multiplier_q14,
    input  wire [16*16-1:0] cfg_multiplier_q14_bus,
    input  wire [3:0]    cfg_multiplier_shift,
    input  wire [7:0]    cfg_output_zero,
    input  wire          cfg_shortcut_en,
    input  wire [7:0]    cfg_shortcut_zero,
    input  wire [15:0]   cfg_shortcut_multiplier_q14,
    input  wire [7:0]    cfg_word_fill,

    output wire          word_valid,
    input  wire          word_ready,
    output wire [1:0]    word_dest_kind,
    output wire [DEST_ADDR_WIDTH-1:0] word_dest_addr,
    output wire [127:0]  word_data,

    output wire          idle,
    output wire          status_error,
    output wire [9:0]    debug_error_code,
    output reg  [31:0]   accepted_uops,
    output reg  [31:0]   accepted_transactions,
    output reg  [31:0]   emitted_words
);

    localparam integer MAC_META_STAGES = 6;

    // ------------------------------------------------------------------
    // Registered input boundary. It always drains on the next cycle because
    // the fixed-throughput MAC never receives downstream backpressure.
    // ------------------------------------------------------------------
    reg          uop_stage_valid_q;
    reg          uop_stage_first_q;
    reg          uop_stage_last_q;
    reg [15:0]   uop_stage_lane_valid_q;
    reg [127:0]  uop_stage_term_valid_q;
    reg [1023:0] uop_stage_qd_q;
    reg [1023:0] uop_stage_qw_q;
    reg [1:0]    uop_stage_kind_q;
    reg [DEST_ADDR_WIDTH-1:0] uop_stage_addr_q;
    reg [255:0]  uop_stage_bias_q;
    reg [127:0]  uop_stage_shortcut_q;
    reg [7:0]    uop_stage_weight_zero_q;
    reg          uop_stage_relu6_en_q;
    reg [31:0]   uop_stage_relu6_limit_q;
    reg [16*16-1:0] uop_stage_multiplier_q;
    reg [3:0]   uop_stage_multiplier_shift_q;
    reg [7:0]    uop_stage_output_zero_q;
    reg          uop_stage_shortcut_en_q;
    reg [7:0]    uop_stage_shortcut_zero_q;
    reg [15:0]   uop_stage_shortcut_multiplier_q;
    reg [7:0]    uop_stage_word_fill_q;

    reg          transaction_active_q;
    reg          last_cooldown_q;
    reg [OUTSTANDING_COUNT_WIDTH-1:0] outstanding_words_q;

    wire word_fire;
    wire mac_uop_ready;
    wire uop_stage_ready = !uop_stage_valid_q || mac_uop_ready;
    wire launch_credit_available =
        outstanding_words_q < MAX_OUTSTANDING;
    wire transaction_credit_ok = transaction_active_q
        || launch_credit_available;
    // A multi-uop transaction may start immediately after the previous last
    // uop. A one-uop transaction waits one cycle so completed batches retain
    // the PQ8 initiation interval of two cycles.
    wire completion_spacing_ok = transaction_active_q
        || !last_cooldown_q || !uop_last;
    wire input_meta_allowed = !uop_last || txn_meta_valid;

    assign uop_ready = rst_n && !clear && uop_stage_ready
        && transaction_credit_ok && completion_spacing_ok
        && input_meta_allowed;

    wire uop_stage_accept = uop_valid && uop_ready;
    wire uop_stage_data_load = UNCONDITIONAL_UOP_DATA != 0
        ? 1'b1 : uop_stage_accept;
    wire reserve_word = uop_stage_accept && uop_first;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uop_stage_valid_q <= 1'b0;
            transaction_active_q <= 1'b0;
            last_cooldown_q <= 1'b0;
            outstanding_words_q <= 0;
        end else if (clear) begin
            uop_stage_valid_q <= 1'b0;
            transaction_active_q <= 1'b0;
            last_cooldown_q <= 1'b0;
            outstanding_words_q <= 0;
        end else begin
            if (uop_stage_ready)
                uop_stage_valid_q <= uop_stage_accept;

            last_cooldown_q <= uop_stage_accept && uop_last;
            if (uop_stage_accept) begin
                if (uop_first)
                    transaction_active_q <= !uop_last;
                else if (uop_last)
                    transaction_active_q <= 1'b0;
            end

            case ({reserve_word, word_fire})
                2'b10: outstanding_words_q <=
                    outstanding_words_q + 1'b1;
                2'b01: outstanding_words_q <=
                    outstanding_words_q - 1'b1;
                default: ;
            endcase
        end
    end

    always @(posedge clk) begin
        if (uop_stage_data_load) begin
            uop_stage_first_q <= uop_first;
            uop_stage_last_q <= uop_last;
            uop_stage_lane_valid_q <= uop_lane_valid;
            uop_stage_term_valid_q <= uop_term_valid;
            uop_stage_qd_q <= uop_qd;
            uop_stage_qw_q <= uop_qw;
            uop_stage_kind_q <= txn_dest_kind;
            uop_stage_addr_q <= txn_dest_addr;
            uop_stage_shortcut_q <= txn_shortcut_c8p2;
            uop_stage_weight_zero_q <= cfg_weight_zero;
            uop_stage_relu6_en_q <= cfg_relu6_en;
            uop_stage_relu6_limit_q <= cfg_relu6_limit;
            uop_stage_multiplier_q <=
                (PER_CHANNEL_MULTIPLIER != 0)
                    ? cfg_multiplier_q14_bus : {16{cfg_multiplier_q14}};
            uop_stage_multiplier_shift_q <= cfg_multiplier_shift;
            uop_stage_output_zero_q <= cfg_output_zero;
            uop_stage_shortcut_en_q <= cfg_shortcut_en;
            uop_stage_shortcut_zero_q <= cfg_shortcut_zero;
            uop_stage_shortcut_multiplier_q <=
                cfg_shortcut_multiplier_q14;
            uop_stage_word_fill_q <= cfg_word_fill;
        end
    end

    generate
        if (UNCONDITIONAL_UOP_BIAS != 0) begin : gen_unconditional_uop_bias
            always @(posedge clk)
                uop_stage_bias_q <= txn_bias_f8;
        end else begin : gen_accepted_uop_bias
            always @(posedge clk)
                if (uop_stage_data_load)
                    uop_stage_bias_q <= txn_bias_f8;
        end
    endgenerate

    // ------------------------------------------------------------------
    // Fixed-throughput MAC. result_ready is tied high and result_valid is a
    // one-cycle completion token; it cannot gate the DSP or QD pipelines.
    // ------------------------------------------------------------------
    wire          mac_uop_valid = uop_stage_valid_q;
    wire          mac_uop_fire = mac_uop_valid && mac_uop_ready;
    wire          mac_result_valid;
    wire [15:0]   mac_result_lane_valid;
    wire [511:0]  mac_raw_sum;
    wire [511:0]  mac_sum_qd;

    npu_u8_mac16x8_dsp_pipeline #(
        .PREZEROED_OPERANDS(PREZEROED_OPERANDS),
        .FIXED_THROUGHPUT(1)
    ) u_mac (
        .clk(clk),
        .rst_n(rst_n),
        .uop_valid(mac_uop_valid),
        .uop_ready(mac_uop_ready),
        .uop_first(uop_stage_first_q),
        .uop_last(uop_stage_last_q),
        .lane_valid(uop_stage_lane_valid_q),
        .term_valid(uop_stage_term_valid_q),
        .qd(uop_stage_qd_q),
        .qw(uop_stage_qw_q),
        .result_valid(mac_result_valid),
        .result_ready(1'b1),
        .result_lane_valid(mac_result_lane_valid),
        .raw_sum(mac_raw_sum),
        .sum_qd(mac_sum_qd)
    );

    // ------------------------------------------------------------------
    // Fixed metadata pipeline aligned with the MAC completion token.
    // Wide data has no reset or valid-driven clock enable; only the narrow
    // token is cleared. This replaces the pointer-selected wide issue FIFO.
    // ------------------------------------------------------------------
    reg [MAC_META_STAGES-1:0] meta_valid_pipe_q;
    reg [1:0]   meta_kind_pipe [0:MAC_META_STAGES-1];
    reg [DEST_ADDR_WIDTH-1:0] meta_addr_pipe [0:MAC_META_STAGES-1];
    reg [255:0] meta_bias_pipe [0:MAC_META_STAGES-1];
    reg [127:0] meta_shortcut_pipe [0:MAC_META_STAGES-1];
    reg [7:0]   meta_weight_zero_pipe [0:MAC_META_STAGES-1];
    reg         meta_relu6_en_pipe [0:MAC_META_STAGES-1];
    reg [31:0]  meta_relu6_limit_pipe [0:MAC_META_STAGES-1];
    reg [16*16-1:0] meta_multiplier_pipe [0:MAC_META_STAGES-1];
    reg [3:0]  meta_multiplier_shift_pipe [0:MAC_META_STAGES-1];
    reg [7:0]   meta_output_zero_pipe [0:MAC_META_STAGES-1];
    reg         meta_shortcut_en_pipe [0:MAC_META_STAGES-1];
    reg [7:0]   meta_shortcut_zero_pipe [0:MAC_META_STAGES-1];
    reg [15:0]  meta_shortcut_multiplier_pipe
        [0:MAC_META_STAGES-1];
    reg [7:0]   meta_word_fill_pipe [0:MAC_META_STAGES-1];

    wire meta_inject = mac_uop_fire && uop_stage_last_q;
    wire meta_head_valid = meta_valid_pipe_q[MAC_META_STAGES-1];
    wire [1:0] meta_head_kind =
        meta_kind_pipe[MAC_META_STAGES-1];
    wire [DEST_ADDR_WIDTH-1:0] meta_head_addr =
        meta_addr_pipe[MAC_META_STAGES-1];
    wire [255:0] meta_head_bias =
        meta_bias_pipe[MAC_META_STAGES-1];
    wire [127:0] meta_head_shortcut =
        meta_shortcut_pipe[MAC_META_STAGES-1];
    wire [7:0] meta_head_weight_zero =
        meta_weight_zero_pipe[MAC_META_STAGES-1];
    wire meta_head_relu6_en =
        meta_relu6_en_pipe[MAC_META_STAGES-1];
    wire [31:0] meta_head_relu6_limit =
        meta_relu6_limit_pipe[MAC_META_STAGES-1];
    wire [16*16-1:0] meta_head_multiplier =
        meta_multiplier_pipe[MAC_META_STAGES-1];
    wire [3:0] meta_head_multiplier_shift =
        meta_multiplier_shift_pipe[MAC_META_STAGES-1];
    wire [7:0] meta_head_output_zero =
        meta_output_zero_pipe[MAC_META_STAGES-1];
    wire meta_head_shortcut_en =
        meta_shortcut_en_pipe[MAC_META_STAGES-1];
    wire [7:0] meta_head_shortcut_zero =
        meta_shortcut_zero_pipe[MAC_META_STAGES-1];
    wire [15:0] meta_head_shortcut_multiplier =
        meta_shortcut_multiplier_pipe[MAC_META_STAGES-1];
    wire [7:0] meta_head_word_fill =
        meta_word_fill_pipe[MAC_META_STAGES-1];

    integer meta_stage;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            meta_valid_pipe_q <= 0;
        else if (clear)
            meta_valid_pipe_q <= 0;
        else
            meta_valid_pipe_q <= {
                meta_valid_pipe_q[MAC_META_STAGES-2:0], meta_inject
            };
    end

    always @(posedge clk) begin
        meta_kind_pipe[0] <= uop_stage_kind_q;
        meta_addr_pipe[0] <= uop_stage_addr_q;
        meta_bias_pipe[0] <= uop_stage_bias_q;
        meta_shortcut_pipe[0] <= uop_stage_shortcut_q;
        meta_weight_zero_pipe[0] <= uop_stage_weight_zero_q;
        meta_relu6_en_pipe[0] <= uop_stage_relu6_en_q;
        meta_relu6_limit_pipe[0] <= uop_stage_relu6_limit_q;
        meta_multiplier_pipe[0] <= uop_stage_multiplier_q;
        meta_multiplier_shift_pipe[0] <= uop_stage_multiplier_shift_q;
        meta_output_zero_pipe[0] <= uop_stage_output_zero_q;
        meta_shortcut_en_pipe[0] <= uop_stage_shortcut_en_q;
        meta_shortcut_zero_pipe[0] <= uop_stage_shortcut_zero_q;
        meta_shortcut_multiplier_pipe[0] <=
            uop_stage_shortcut_multiplier_q;
        meta_word_fill_pipe[0] <= uop_stage_word_fill_q;
        for (meta_stage = 1; meta_stage < MAC_META_STAGES;
             meta_stage = meta_stage + 1) begin
            meta_kind_pipe[meta_stage] <=
                meta_kind_pipe[meta_stage-1];
            meta_addr_pipe[meta_stage] <=
                meta_addr_pipe[meta_stage-1];
            meta_bias_pipe[meta_stage] <=
                meta_bias_pipe[meta_stage-1];
            meta_shortcut_pipe[meta_stage] <=
                meta_shortcut_pipe[meta_stage-1];
            meta_weight_zero_pipe[meta_stage] <=
                meta_weight_zero_pipe[meta_stage-1];
            meta_relu6_en_pipe[meta_stage] <=
                meta_relu6_en_pipe[meta_stage-1];
            meta_relu6_limit_pipe[meta_stage] <=
                meta_relu6_limit_pipe[meta_stage-1];
            meta_multiplier_pipe[meta_stage] <=
                meta_multiplier_pipe[meta_stage-1];
            meta_multiplier_shift_pipe[meta_stage] <=
                meta_multiplier_shift_pipe[meta_stage-1];
            meta_output_zero_pipe[meta_stage] <=
                meta_output_zero_pipe[meta_stage-1];
            meta_shortcut_en_pipe[meta_stage] <=
                meta_shortcut_en_pipe[meta_stage-1];
            meta_shortcut_zero_pipe[meta_stage] <=
                meta_shortcut_zero_pipe[meta_stage-1];
            meta_shortcut_multiplier_pipe[meta_stage] <=
                meta_shortcut_multiplier_pipe[meta_stage-1];
            meta_word_fill_pipe[meta_stage] <=
                meta_word_fill_pipe[meta_stage-1];
        end
    end

    wire quant_in_ready;
    wire quant_out_valid;
    wire quant_out_ready;
    wire [7:0] quant_out_lane_valid;
    wire [63:0] quant_out_q;
    wire [6:0] quant_fifo_level;
    wire quant_fifo_high;
    wire quant_fifo_full;
    wire quant_fifo_empty;
    wire quant_status_error;
    wire [9:0] quant_debug_error_code;
    wire quant_in_valid = mac_result_valid && meta_head_valid;
    wire quant_in_fire = quant_in_valid && quant_in_ready;

    npu_pq8_stream_pipeline_v2 #(
        .CAPTURE_ON_ACCEPT(PQ_CAPTURE_ON_ACCEPT),
        .USE_PER_LANE_MULTIPLIER(PER_CHANNEL_MULTIPLIER),
        .MAIN_MULTIPLIER_SHIFT(MAIN_MULTIPLIER_SHIFT),
        .FIFO_DEPTH(PQ_FIFO_DEPTH),
        .FIFO_AW(PQ_FIFO_AW)
    ) u_pq8 (
        .clk(clk),
        .rst_n(rst_n),
        .clear(clear),
        .in_valid(quant_in_valid),
        .in_ready(quant_in_ready),
        .in_lane_valid_bus(mac_result_lane_valid),
        .in_acc_pre_corrected_bus(16'd0),
        .in_acc_bus(mac_raw_sum),
        .in_sum_qd_bus(mac_sum_qd),
        .in_weight_zero(meta_head_weight_zero),
        .in_bias_f8(meta_head_bias),
        .in_relu6_en(meta_head_relu6_en),
        .in_relu6_limit(meta_head_relu6_limit),
        .in_multiplier(meta_head_multiplier[15:0]),
        .in_multiplier_bus(meta_head_multiplier),
        .in_multiplier_shift(meta_head_multiplier_shift),
        .in_output_zero(meta_head_output_zero),
        .in_shortcut_en(meta_head_shortcut_en),
        .in_shortcut_q_c8p2(meta_head_shortcut),
        .in_shortcut_zero(meta_head_shortcut_zero),
        .in_shortcut_multiplier(meta_head_shortcut_multiplier),
        .out_valid(quant_out_valid),
        .out_ready(quant_out_ready),
        .out_lane_valid_bus(quant_out_lane_valid),
        .out_q_bus(quant_out_q),
        .fifo_level_results(quant_fifo_level),
        .fifo_high_watermark(quant_fifo_high),
        .fifo_full(quant_fifo_full),
        .fifo_empty(quant_fifo_empty),
        .status_error(quant_status_error),
        .debug_error_code(quant_debug_error_code)
    );

    // ------------------------------------------------------------------
    // Only narrow destination metadata remains queued because output FIFO
    // backpressure can vary its residence time.
    // ------------------------------------------------------------------
    reg [1:0] out_kind [0:META_DEPTH-1];
    reg [DEST_ADDR_WIDTH-1:0] out_addr [0:META_DEPTH-1];
    reg [7:0] out_fill [0:META_DEPTH-1];
    reg [META_PTR_WIDTH-1:0] out_wptr_q;
    reg [META_PTR_WIDTH-1:0] out_rptr_q;
    reg [META_PTR_WIDTH:0] out_count_q;
    wire out_empty = out_count_q == 0;
    wire out_full = out_count_q >= META_DEPTH;
    wire [1:0] out_head_kind = out_kind[out_rptr_q];
    wire [DEST_ADDR_WIDTH-1:0] out_head_addr = out_addr[out_rptr_q];
    wire [7:0] out_head_fill = out_fill[out_rptr_q];

    reg fetch_group_q;
    reg group_stage_valid_q;
    reg group_stage_phase_q;
    reg [7:0] group_stage_lane_valid_q;
    reg [63:0] group_stage_q_q;
    reg [1:0] group_stage_kind_q;
    reg [DEST_ADDR_WIDTH-1:0] group_stage_addr_q;
    reg [7:0] group_stage_fill_q;
    reg [127:0] pack_q;
    reg word_valid_q;
    reg [1:0] word_kind_q;
    reg [DEST_ADDR_WIDTH-1:0] word_addr_q;
    reg [127:0] word_data_q;

    assign word_valid = word_valid_q;
    assign word_dest_kind = word_kind_q;
    assign word_dest_addr = word_addr_q;
    assign word_data = word_data_q;
    assign word_fire = word_valid_q && word_ready;

    wire group_stage_consume = group_stage_valid_q
        && (!group_stage_phase_q || !word_valid_q || word_ready);
    wire group_stage_ready = !group_stage_valid_q
        || group_stage_consume;
    assign quant_out_ready = !out_empty && group_stage_ready;
    wire quant_out_fire = quant_out_valid && quant_out_ready;
    wire out_push = quant_in_fire;
    wire out_pop = quant_out_fire && fetch_group_q;

    reg [127:0] group0_word_calc;
    reg [127:0] group1_word_calc;
    integer output_pair;
    always @(*) begin
        group0_word_calc = {16{group_stage_fill_q}};
        group1_word_calc = pack_q;
        for (output_pair = 0; output_pair < 4;
             output_pair = output_pair + 1) begin
            if (group_stage_lane_valid_q[output_pair*2])
                group0_word_calc[output_pair*8 +: 8] =
                    group_stage_q_q[(output_pair*2)*8 +: 8];
            if (group_stage_lane_valid_q[output_pair*2 + 1])
                group0_word_calc[(8 + output_pair)*8 +: 8] =
                    group_stage_q_q[(output_pair*2 + 1)*8 +: 8];
            if (group_stage_lane_valid_q[output_pair*2])
                group1_word_calc[(4 + output_pair)*8 +: 8] =
                    group_stage_q_q[(output_pair*2)*8 +: 8];
            if (group_stage_lane_valid_q[output_pair*2 + 1])
                group1_word_calc[(12 + output_pair)*8 +: 8] =
                    group_stage_q_q[(output_pair*2 + 1)*8 +: 8];
        end
    end

    reg error_q;
    reg [9:0] backend_debug_error_code_q;
    assign status_error = error_q || quant_status_error;
    assign debug_error_code = (quant_debug_error_code != 10'd0)
        ? quant_debug_error_code : backend_debug_error_code_q;
    assign idle = !uop_stage_valid_q && !transaction_active_q
        && (outstanding_words_q == 0) && (meta_valid_pipe_q == 0)
        && out_empty && quant_fifo_empty && !mac_result_valid
        && !fetch_group_q && !group_stage_valid_q && !word_valid_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_wptr_q <= 0;
            out_rptr_q <= 0;
            out_count_q <= 0;
            fetch_group_q <= 1'b0;
            group_stage_valid_q <= 1'b0;
            group_stage_phase_q <= 1'b0;
            pack_q <= 0;
            word_valid_q <= 1'b0;
            word_kind_q <= 0;
            word_addr_q <= 0;
            word_data_q <= 0;
            error_q <= 1'b0;
            backend_debug_error_code_q <= 10'd0;
            accepted_uops <= 0;
            accepted_transactions <= 0;
            emitted_words <= 0;
        end else if (clear) begin
            out_wptr_q <= 0;
            out_rptr_q <= 0;
            out_count_q <= 0;
            fetch_group_q <= 1'b0;
            group_stage_valid_q <= 1'b0;
            group_stage_phase_q <= 1'b0;
            word_valid_q <= 1'b0;
            error_q <= 1'b0;
            backend_debug_error_code_q <= 10'd0;
            accepted_uops <= 0;
            accepted_transactions <= 0;
            emitted_words <= 0;
        end else begin
            if (mac_uop_fire)
                accepted_uops <= accepted_uops + 1'b1;
            if (meta_inject)
                accepted_transactions <= accepted_transactions + 1'b1;
            if (word_fire)
                emitted_words <= emitted_words + 1'b1;

            if (out_push) begin
                out_kind[out_wptr_q] <= meta_head_kind;
                out_addr[out_wptr_q] <= meta_head_addr;
                out_fill[out_wptr_q] <= meta_head_word_fill;
                if (out_wptr_q == META_DEPTH - 1)
                    out_wptr_q <= 0;
                else
                    out_wptr_q <= out_wptr_q + 1'b1;
            end
            if (out_pop) begin
                if (out_rptr_q == META_DEPTH - 1)
                    out_rptr_q <= 0;
                else
                    out_rptr_q <= out_rptr_q + 1'b1;
            end
            case ({out_push, out_pop})
                2'b10: out_count_q <= out_count_q + 1'b1;
                2'b01: out_count_q <= out_count_q - 1'b1;
                default: ;
            endcase

            case ({quant_out_fire, group_stage_consume})
                2'b10: group_stage_valid_q <= 1'b1;
                2'b01: group_stage_valid_q <= 1'b0;
                2'b11: group_stage_valid_q <= 1'b1;
                default: ;
            endcase
            if (quant_out_fire) begin
                group_stage_phase_q <= fetch_group_q;
                group_stage_lane_valid_q <= quant_out_lane_valid;
                group_stage_q_q <= quant_out_q;
                group_stage_kind_q <= out_head_kind;
                group_stage_addr_q <= out_head_addr;
                group_stage_fill_q <= out_head_fill;
                fetch_group_q <= !fetch_group_q;
            end

            // Preserve group 0 while group 1 is backpressured.  Updating the
            // partial word from a stalled phase-1 payload corrupts the lower
            // half before the complete 128-bit word can be committed.
            if (group_stage_consume && !group_stage_phase_q)
                pack_q <= group0_word_calc;

            if (group_stage_consume && group_stage_phase_q) begin
                word_data_q <= group1_word_calc;
                word_kind_q <= group_stage_kind_q;
                word_addr_q <= group_stage_addr_q;
            end

            if (word_fire
                && !(group_stage_consume && group_stage_phase_q))
                word_valid_q <= 1'b0;

            if (group_stage_consume && group_stage_phase_q)
                word_valid_q <= 1'b1;

            if ((uop_stage_accept && !transaction_active_q
                    && !uop_first)
                || (uop_stage_accept && transaction_active_q
                    && uop_first)
                || (uop_valid && uop_last && !txn_meta_valid)
                || (reserve_word && !launch_credit_available)
                || (word_fire && (outstanding_words_q == 0))
                || (mac_result_valid != meta_head_valid)
                || (mac_result_valid && !quant_in_ready)
                || (out_push && out_full && !out_pop)
                || (out_pop && out_empty))
                error_q <= 1'b1;

            if (backend_debug_error_code_q == 10'd0) begin
                if ((uop_stage_accept && !transaction_active_q
                        && !uop_first)
                        || (uop_stage_accept && transaction_active_q
                            && uop_first))
                    backend_debug_error_code_q <= 10'd621;
                else if (uop_valid && uop_last && !txn_meta_valid)
                    backend_debug_error_code_q <= 10'd622;
                else if (reserve_word && !launch_credit_available)
                    backend_debug_error_code_q <= 10'd623;
                else if (word_fire && (outstanding_words_q == 0))
                    backend_debug_error_code_q <= 10'd624;
                else if (mac_result_valid != meta_head_valid)
                    backend_debug_error_code_q <= 10'd625;
                else if (mac_result_valid && !quant_in_ready)
                    backend_debug_error_code_q <= 10'd626;
                else if (out_push && out_full && !out_pop)
                    backend_debug_error_code_q <= 10'd627;
                else if (out_pop && out_empty)
                    backend_debug_error_code_q <= 10'd628;
            end
        end
    end

endmodule

`default_nettype wire
