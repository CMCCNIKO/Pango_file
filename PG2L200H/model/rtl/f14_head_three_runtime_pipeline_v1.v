//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 复用一个描述符序列器及一个通用head计算核，串行执行cls、vertical、offset三分支的12条描述符。
// Module Name: f14_head_three_runtime_v1
// File Name: model/rtl/f14_head_three_runtime_pipeline_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 复用一个描述符序列器及一个通用head计算核，串行执行cls、vertical、offset三分支的12条描述符。
// 装载各描述符的bias/量化参数，以共享特征/ROM口运行；输出同时携带bank及head身份，使三最终图可共用写回路径。
//
// Dependencies: f14_head_descriptor_sequencer_v1、f14_head_generic_runtime_core_v1、f14_head_qparams_all_v1.vh
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 配置与启动分拍，等待算子done且核idle后推进；普通字与signed16写回通道分别握手。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// F14 three-head runtime.  One descriptor sequencer, one feeder and one
// signed16 MAC/PQ backend are reused for cls, vertical and offset (40..51).
// The branch identity is carried with output words so the three final maps can
// share the same output path without ambiguous bank numbers.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none
module f14_head_three_runtime_v1 #(
    parameter integer C_PARTITION_RESET = 0
) (
    input wire i_clk, input wire i_rst_n, input wire i_clear,
    input wire [7:0] i_reset_partition_n,
    input wire i_start,
    output wire o_start_ready,
    output wire o_feature_rd_en, output wire [17:0] o_feature_rd_addr,
    output wire [1:0] o_feature_bank,
    input wire i_feature_rd_valid, input wire [127:0] i_feature_rd_data,
    output wire o_rom_rd_en, output wire [16:0] o_rom_rd_addr,
    input wire i_rom_rd_ready, input wire i_run_enable,
    input wire i_rom_rd_valid, input wire [31:0] i_rom_rd_data,
    output wire o_word_valid, input wire i_word_ready,
    output wire [1:0] o_word_bank, output wire [1:0] o_word_head,
    output wire [10:0] o_word_dest_addr, output wire [127:0] o_word_data,
    output wire o_main_s16_valid, input wire i_main_s16_ready,
    output wire [1:0] o_main_s16_bank, output wire [1:0] o_main_s16_head,
    output wire [10:0] o_main_s16_dest_addr, output wire [127:0] o_main_s16_data,
    output wire o_busy, output wire o_done, output wire o_status_error,
    output wire [9:0] o_debug_error_code,
    output wire [31:0] o_accepted_uops, output wire [31:0] o_accepted_transactions,
    output wire [31:0] o_emitted_words, output wire [31:0] o_emitted_main_s16_words,
    output wire [3:0] o_descriptor_slot
);
    `include "f14_head_qparams_all_v1.vh"
    wire seq_busy, seq_done, seq_error, seq_op_valid;
    wire [3:0] seq_slot;
    wire [1:0] seq_stage, seq_input_bank, seq_output_bank;
    wire [7:0] seq_input_h, seq_input_w, seq_input_c;
    wire [7:0] seq_output_h, seq_output_w, seq_output_c;
    wire [3:0] seq_kernel_h, seq_kernel_w, seq_stride_y, seq_stride_x;
    wire [3:0] seq_pad_y, seq_pad_x;
    wire [16:0] seq_weight_base;
    wire core_ready, core_done, core_idle, core_error;
    wire core_feature_en, core_rom_en, core_word_valid, core_s16_valid;
    wire [17:0] core_feature_addr; wire [16:0] core_rom_addr;
    wire [1:0] core_word_kind, core_s16_kind;
    wire [10:0] core_word_addr, core_s16_addr;
    wire [127:0] core_word_data, core_s16_data;
    wire [31:0] core_uops, core_txns, core_words, core_s16_words;
    wire [9:0] core_error_code;
    reg core_done_seen_q, core_cfg_pending_q;
    wire core_cfg_valid = seq_op_valid && core_ready && !core_done_seen_q && !core_cfg_pending_q;
    wire core_start = core_cfg_pending_q && core_ready;
    wire seq_done_ack = core_done_seen_q && core_idle;
    wire [1023:0] cfg_bias_bus = {
        f14_head_bias_group(seq_slot, 2'd3),
        f14_head_bias_group(seq_slot, 2'd2),
        f14_head_bias_group(seq_slot, 2'd1),
        f14_head_bias_group(seq_slot, 2'd0)};
    wire [1023:0] cfg_mult_bus = {
        f14_head_mult_group(seq_slot, 2'd3),
        f14_head_mult_group(seq_slot, 2'd2),
        f14_head_mult_group(seq_slot, 2'd1),
        f14_head_mult_group(seq_slot, 2'd0)};

    f14_head_descriptor_sequencer_v1 #(.P_LAST_SLOT(4'd11)) u_seq (
        .i_clk(i_clk), .i_rst_n(i_rst_n), .i_clear(i_clear), .i_start(i_start),
        .i_op_ready(core_ready && !core_done_seen_q), .i_op_done(seq_done_ack),
        .o_start_ready(o_start_ready), .o_busy(seq_busy), .o_done(seq_done),
        .o_error(seq_error), .o_op_valid(seq_op_valid), .o_descriptor_slot(seq_slot),
        .o_stage_kind(seq_stage), .o_input_bank(seq_input_bank), .o_output_bank(seq_output_bank),
        .o_input_h(seq_input_h), .o_input_w(seq_input_w), .o_input_c(seq_input_c),
        .o_output_h(seq_output_h), .o_output_w(seq_output_w), .o_output_c(seq_output_c),
        .o_kernel_h(seq_kernel_h), .o_kernel_w(seq_kernel_w),
        .o_stride_y(seq_stride_y), .o_stride_x(seq_stride_x),
        .o_pad_y(seq_pad_y), .o_pad_x(seq_pad_x),
        .o_weight_base_word32(seq_weight_base), .o_bias_base_word32(), .o_qscale_base_word32());

    f14_head_generic_runtime_core_v1 #(.C_PARTITION_RESET(C_PARTITION_RESET)) u_core (
        .i_reset_partition_n(i_reset_partition_n),
        .i_clk(i_clk), .i_rst_n(i_rst_n), .i_clear(i_clear),
        .i_cfg_valid(core_cfg_valid), .i_cfg_input_h(seq_input_h), .i_cfg_input_w(seq_input_w),
        .i_cfg_input_c(seq_input_c), .i_cfg_output_h(seq_output_h), .i_cfg_output_w(seq_output_w),
        .i_cfg_output_c(seq_output_c), .i_cfg_kernel_h(seq_kernel_h), .i_cfg_kernel_w(seq_kernel_w),
        .i_cfg_stride_y(seq_stride_y), .i_cfg_stride_x(seq_stride_x),
        .i_cfg_pad_y(seq_pad_y), .i_cfg_pad_x(seq_pad_x),
        .i_cfg_rom_word_base(seq_weight_base), .i_cfg_dest_kind(2'd0), .i_cfg_dest_base(11'd0),
        .i_cfg_bias_bus(cfg_bias_bus), .i_cfg_multiplier_bus(cfg_mult_bus),
        .i_cfg_weight_zero(8'd128), .i_cfg_output_zero(8'd0), .i_cfg_multiplier_shift(4'd14),
        .i_cfg_relu6_en(1'b0), .i_cfg_relu6_limit(32'd0), .i_start(core_start),
        .o_feature_rd_en(core_feature_en), .o_feature_rd_addr(core_feature_addr),
        .i_feature_rd_valid(i_feature_rd_valid), .i_feature_rd_data(i_feature_rd_data),
        .o_rom_rd_en(core_rom_en), .o_rom_rd_addr(core_rom_addr),
        .i_rom_rd_ready(i_rom_rd_ready), .i_run_enable(i_run_enable), .i_rom_rd_valid(i_rom_rd_valid), .i_rom_rd_data(i_rom_rd_data),
        .o_start_ready(core_ready), .o_done(core_done), .o_idle(core_idle),
        .o_status_error(core_error), .o_debug_error_code(core_error_code),
        .o_accepted_uops(core_uops), .o_accepted_transactions(core_txns),
        .o_emitted_words(core_words), .o_word_valid(core_word_valid), .i_word_ready(i_word_ready),
        .o_word_dest_kind(core_word_kind), .o_word_dest_addr(core_word_addr), .o_word_data(core_word_data),
        .o_main_s16_valid(core_s16_valid), .i_main_s16_ready(i_main_s16_ready),
        .o_main_s16_dest_kind(core_s16_kind), .o_main_s16_dest_addr(core_s16_addr),
        .o_main_s16_data(core_s16_data), .o_emitted_main_s16_words(core_s16_words));

    assign o_feature_rd_en = core_feature_en;
    assign o_feature_rd_addr = core_feature_addr;
    assign o_feature_bank = seq_input_bank;
    assign o_rom_rd_en = core_rom_en;
    assign o_rom_rd_addr = core_rom_addr;
    assign o_word_valid = core_word_valid;
    assign o_word_bank = seq_output_bank;
    assign o_word_head = seq_stage;
    assign o_word_dest_addr = core_word_addr;
    assign o_word_data = core_word_data;
    assign o_main_s16_valid = core_s16_valid;
    assign o_main_s16_bank = seq_output_bank;
    assign o_main_s16_head = seq_stage;
    assign o_main_s16_dest_addr = core_s16_addr;
    assign o_main_s16_data = core_s16_data;
    assign o_busy = seq_busy;
    assign o_done = seq_done;
    assign o_status_error = seq_error | core_error;
    assign o_debug_error_code = core_error_code;
    assign o_accepted_uops = core_uops;
    assign o_accepted_transactions = core_txns;
    assign o_emitted_words = core_words;
    assign o_emitted_main_s16_words = core_s16_words;
    assign o_descriptor_slot = seq_slot;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            core_done_seen_q <= 1'b0;
            core_cfg_pending_q <= 1'b0;
        end else if (i_clear) begin
            core_done_seen_q <= 1'b0;
            core_cfg_pending_q <= 1'b0;
        end else begin
            if (core_cfg_valid) core_cfg_pending_q <= 1'b1;
            else if (core_start) core_cfg_pending_q <= 1'b0;
            if (core_done) core_done_seen_q <= 1'b1;
            if (seq_done_ack) core_done_seen_q <= 1'b0;
        end
    end
endmodule

`default_nettype wire
