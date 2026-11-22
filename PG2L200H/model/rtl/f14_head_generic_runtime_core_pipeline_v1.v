//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将运行时几何scheduler、特征/权重feeder与支持signed16的共享MAC/PQ组合成单个通用head算子。
// Module Name: f14_head_generic_runtime_core_v1
// File Name: model/rtl/f14_head_generic_runtime_core_pipeline_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将运行时几何scheduler、特征/权重feeder与支持signed16的共享MAC/PQ组合成单个通用head算子。
// 按描述符配置生成卷积微操作，选择各输出通道组的bias/量化系数，输出普通128位字或主signed16结果以及计数/错误状态。
//
// Dependencies: f14_head_uop_scheduler_runtime_v1、f14_head_descriptor_feeder_runtime_v1、f14_sop_shared_mac_pq8_backend_signed16_v1
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// C_PARTITION_RESET控制选用分区复位；配置、clear及外部请求响应流水须按原契约共同清空。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// One shared runtime-configured generic head operation.
// The module owns one scheduler, one feeder and one signed16-capable SOP
// backend.  A descriptor manager loads the configuration, then pulses start.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

module f14_head_generic_runtime_core_v1 #(
    parameter integer C_PARTITION_RESET = 0
) (
    input wire i_clk, input wire i_rst_n, input wire i_clear,
    input wire [7:0] i_reset_partition_n,
    input wire i_cfg_valid,
    input wire [7:0] i_cfg_input_h, input wire [7:0] i_cfg_input_w,
    input wire [7:0] i_cfg_input_c, input wire [7:0] i_cfg_output_h,
    input wire [7:0] i_cfg_output_w, input wire [7:0] i_cfg_output_c,
    input wire [3:0] i_cfg_kernel_h, input wire [3:0] i_cfg_kernel_w,
    input wire [3:0] i_cfg_stride_y, input wire [3:0] i_cfg_stride_x,
    input wire [3:0] i_cfg_pad_y, input wire [3:0] i_cfg_pad_x,
    input wire [16:0] i_cfg_rom_word_base,
    input wire [1:0] i_cfg_dest_kind, input wire [10:0] i_cfg_dest_base,
    input wire [1023:0] i_cfg_bias_bus,
    input wire [1023:0] i_cfg_multiplier_bus,
    input wire [7:0] i_cfg_weight_zero, input wire [7:0] i_cfg_output_zero,
    input wire [3:0] i_cfg_multiplier_shift,
    input wire i_cfg_relu6_en, input wire [31:0] i_cfg_relu6_limit,
    input wire i_start,
    output wire o_feature_rd_en, output wire [17:0] o_feature_rd_addr,
    input wire i_feature_rd_valid, input wire [127:0] i_feature_rd_data,
    output wire o_rom_rd_en, output wire [16:0] o_rom_rd_addr,
    input wire i_rom_rd_ready, input wire i_run_enable,
    input wire i_rom_rd_valid, input wire [31:0] i_rom_rd_data,
    output wire o_start_ready, output wire o_done, output wire o_idle,
    output wire o_status_error, output wire [9:0] o_debug_error_code,
    output wire [31:0] o_accepted_uops, output wire [31:0] o_accepted_transactions,
    output wire [31:0] o_emitted_words, output wire o_word_valid,
    input wire i_word_ready, output wire [1:0] o_word_dest_kind,
    output wire [10:0] o_word_dest_addr, output wire [127:0] o_word_data,
    output wire o_main_s16_valid, input wire i_main_s16_ready,
    output wire [1:0] o_main_s16_dest_kind, output wire [10:0] o_main_s16_dest_addr,
    output wire [127:0] o_main_s16_data, output wire [31:0] o_emitted_main_s16_words
);
    wire sched_busy, sched_uop_valid, sched_uop_ready, sched_first, sched_last;
    wire [7:0] sched_output_y, sched_output_x, sched_output_group, sched_input_group;
    wire [7:0] sched_tap_y, sched_tap_x;
    wire signed [15:0] sched_input_y, sched_input_x;
    wire sched_pixel0_valid, sched_pixel1_valid;
    wire [31:0] sched_transaction_index;
    wire sched_error, feeder_error;
    wire feature_rd_en, rom_rd_en;
    wire [17:0] feature_rd_addr; wire [16:0] rom_rd_addr;
    wire uop_valid, uop_ready, uop_first, uop_last;
    wire [15:0] uop_lane_valid; wire [127:0] uop_term_valid;
    wire [1023:0] uop_qd, uop_qw; wire txn_meta_valid;
    wire [1:0] txn_dest_kind; wire [10:0] txn_dest_addr;
    wire [255:0] txn_bias_f8;
    wire backend_dest_kind; // intentionally unused alias guard
    wire [1:0] backend_main_s16_kind;
    wire [10:0] backend_main_s16_addr;
    wire [127:0] backend_main_s16_data;
    wire backend_main_s16_valid;
    wire backend_status_error;
    wire [127:0] txn_shortcut_c8p2 = 128'd0;
    // transaction_index is ordered as output_y, pixel_pair, output_group.
    // Recover the output group using the number of groups in this descriptor.
    // Taking the low two address bits unconditionally rotates through empty
    // multiplier groups for C8/C4 heads after every pixel pair.
    wire [1:0] cfg_output_group =
        (i_cfg_output_c >= 8'd32) ? txn_dest_addr[1:0] :
        (i_cfg_output_c >= 8'd16) ? {1'b0, txn_dest_addr[0]} : 2'd0;
    wire [255:0] cfg_multiplier_group =
        (cfg_output_group == 2'd0) ? i_cfg_multiplier_bus[255:0] :
        (cfg_output_group == 2'd1) ? i_cfg_multiplier_bus[511:256] :
        (cfg_output_group == 2'd2) ? i_cfg_multiplier_bus[767:512] :
        i_cfg_multiplier_bus[1023:768];

    f14_head_uop_scheduler_runtime_v1 u_scheduler (
        .i_clk(i_clk), .i_rst_n(i_rst_n), .i_start(i_start),
        .i_cfg_input_h(i_cfg_input_h), .i_cfg_input_w(i_cfg_input_w),
        .i_cfg_input_c(i_cfg_input_c), .i_cfg_output_h(i_cfg_output_h),
        .i_cfg_output_w(i_cfg_output_w), .i_cfg_output_c(i_cfg_output_c),
        .i_cfg_kernel_h(i_cfg_kernel_h), .i_cfg_kernel_w(i_cfg_kernel_w),
        .i_cfg_stride_y(i_cfg_stride_y), .i_cfg_stride_x(i_cfg_stride_x),
        .i_cfg_pad_y(i_cfg_pad_y), .i_cfg_pad_x(i_cfg_pad_x),
        .o_start_ready(o_start_ready), .o_busy(sched_busy),
        .i_uop_ready(sched_uop_ready), .o_uop_valid(sched_uop_valid),
        .o_uop_first(sched_first), .o_uop_last(sched_last),
        .o_output_y(sched_output_y), .o_output_x(sched_output_x),
        .o_output_group(sched_output_group), .o_input_group(sched_input_group),
        .o_tap_y(sched_tap_y), .o_tap_x(sched_tap_x),
        .o_input_y(sched_input_y), .o_input_x(sched_input_x),
        .o_pixel0_valid(sched_pixel0_valid), .o_pixel1_valid(sched_pixel1_valid),
        .o_transaction_index(sched_transaction_index), .o_done(o_done),
        .o_error(sched_error));

    f14_head_descriptor_feeder_runtime_v1 u_feeder (
        .i_clk(i_clk), .i_rst_n(C_PARTITION_RESET ? i_reset_partition_n[5] : i_rst_n), .i_clear(i_clear),
        .i_cfg_valid(i_cfg_valid), .i_cfg_input_h(i_cfg_input_h),
        .i_cfg_input_w(i_cfg_input_w), .i_cfg_input_c(i_cfg_input_c),
        .i_cfg_kernel_h(i_cfg_kernel_h), .i_cfg_kernel_w(i_cfg_kernel_w),
        .i_cfg_stride_x(i_cfg_stride_x), .i_cfg_rom_word_base(i_cfg_rom_word_base),
        .i_cfg_dest_kind(i_cfg_dest_kind), .i_cfg_dest_base(i_cfg_dest_base),
        .i_cfg_bias_bus(i_cfg_bias_bus),
        .i_sched_valid(sched_uop_valid), .o_sched_ready(sched_uop_ready),
        .i_sched_first(sched_first), .i_sched_last(sched_last),
        .i_sched_output_group(sched_output_group), .i_sched_input_group(sched_input_group),
        .i_sched_transaction_index(sched_transaction_index),
        .i_sched_tap_y(sched_tap_y), .i_sched_tap_x(sched_tap_x),
        .i_sched_input_y(sched_input_y), .i_sched_input_x(sched_input_x),
        .i_sched_pixel0_valid(sched_pixel0_valid), .i_sched_pixel1_valid(sched_pixel1_valid),
        .o_feature_rd_en(feature_rd_en), .o_feature_rd_addr(feature_rd_addr),
        .i_feature_rd_valid(i_feature_rd_valid), .i_feature_rd_data(i_feature_rd_data),
        .o_rom_rd_en(rom_rd_en), .o_rom_rd_addr(rom_rd_addr),
        .i_rom_rd_ready(i_rom_rd_ready), .i_run_enable(i_run_enable), .i_rom_rd_valid(i_rom_rd_valid), .i_rom_rd_data(i_rom_rd_data),
        .i_backend_ready(uop_ready), .o_uop_valid(uop_valid),
        .o_uop_first(uop_first), .o_uop_last(uop_last),
        .o_uop_lane_valid(uop_lane_valid), .o_uop_term_valid(uop_term_valid),
        .o_uop_qd(uop_qd), .o_uop_qw(uop_qw), .o_txn_meta_valid(txn_meta_valid),
        .o_txn_dest_kind(txn_dest_kind), .o_txn_dest_addr(txn_dest_addr),
        .o_txn_bias_f8(txn_bias_f8), .o_busy(), .o_error(feeder_error));

    f14_sop_shared_mac_pq8_backend_signed16_v1 #(
        .PER_CHANNEL_MULTIPLIER(1), .MAIN_MULTIPLIER_SHIFT(14),
        .PQ_CAPTURE_ON_ACCEPT(1), .UNCONDITIONAL_UOP_DATA(0),
        .UNCONDITIONAL_UOP_BIAS(0)) u_backend (
        .clk(i_clk), .rst_n(C_PARTITION_RESET ? i_reset_partition_n[6] : i_rst_n), .clear(i_clear),
        .uop_valid(uop_valid), .uop_ready(uop_ready), .uop_first(uop_first),
        .uop_last(uop_last), .uop_lane_valid(uop_lane_valid),
        .uop_term_valid(uop_term_valid), .uop_qd(uop_qd), .uop_qw(uop_qw),
        .txn_meta_valid(txn_meta_valid), .txn_dest_kind(txn_dest_kind),
        .txn_dest_addr(txn_dest_addr), .txn_bias_f8(txn_bias_f8),
        .txn_shortcut_c8p2(txn_shortcut_c8p2), .cfg_weight_zero(i_cfg_weight_zero),
        .cfg_relu6_en(i_cfg_relu6_en), .cfg_relu6_limit(i_cfg_relu6_limit),
        .cfg_multiplier_q14(16'd16384), .cfg_multiplier_q14_bus(cfg_multiplier_group),
        .cfg_multiplier_shift(i_cfg_multiplier_shift), .cfg_output_zero(i_cfg_output_zero),
        .cfg_shortcut_en(1'b0), .cfg_shortcut_zero(8'd0),
        .cfg_shortcut_multiplier_q14(16'd0), .cfg_word_fill(8'd0),
        .word_valid(o_word_valid), .word_ready(i_word_ready),
        .word_dest_kind(o_word_dest_kind), .word_dest_addr(o_word_dest_addr),
        .word_data(o_word_data), .main_s16_valid(backend_main_s16_valid),
        .main_s16_ready(i_main_s16_ready), .main_s16_dest_kind(backend_main_s16_kind),
        .main_s16_dest_addr(backend_main_s16_addr), .main_s16_data(backend_main_s16_data),
        .emitted_main_s16_words(o_emitted_main_s16_words), .idle(o_idle),
        .status_error(backend_status_error), .debug_error_code(o_debug_error_code),
        .accepted_uops(o_accepted_uops), .accepted_transactions(o_accepted_transactions),
        .emitted_words(o_emitted_words));

    assign o_feature_rd_en = feature_rd_en;
    assign o_feature_rd_addr = feature_rd_addr;
    assign o_rom_rd_en = rom_rd_en;
    assign o_rom_rd_addr = rom_rd_addr;
    assign o_main_s16_valid = backend_main_s16_valid;
    assign o_main_s16_dest_kind = backend_main_s16_kind;
    assign o_main_s16_dest_addr = backend_main_s16_addr;
    assign o_main_s16_data = backend_main_s16_data;
    assign o_status_error = sched_error | feeder_error | backend_status_error;
endmodule

`default_nettype wire
