//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 复用一套局部存储与共享MAC/PQ，按token顺序执行EXP→DW→PROJ数值tile，支持逐通道线性EXP及EXP后的shortcut重排。
// Module Name: npu_streaming_numeric_tile_6_8
// File Name: model/rtl/f14_sop_streaming_numeric_tile_overlap_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 复用一套局部存储与共享MAC/PQ，按token顺序执行EXP→DW→PROJ数值tile，支持逐通道线性EXP及EXP后的shortcut重排。
// 接收配置和A/PW/DW/QBF/qscale资产加载，预取同步RAM数据、生成深度卷积窗口并打包微操作；双输入槽和输出接口服务上层重叠调度。
//
// Dependencies: npu_streaming_uop_generator、depthwise SWG、operand packer、npu_shared_mac_pq8_backend_fixed_v2、SOP RAM兼容层
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// C_PARTITION_RESET可单独选用backend复位；存储、token与后端须按原所有权/flush契约衔接，不可独立取消有效事务。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// F14 isolated adaptation v2: per-channel linear EXP and post-EXP shortcut repack.
`timescale 1ns/1ps
`default_nettype none

// Token-driven descriptor 6 -> 7 -> 8 numeric tile engine.
//
// The module consumes the frozen R1 token order, uses the accepted R2 SWG for
// depthwise windows, and submits byte-exact affine-u8 packets to the accepted
// shared MAC/PQ backend. All persistent local data stores are explicit M4K
// banks; a one-entry prefetch stage aligns synchronous reads with each uop.
module npu_streaming_numeric_tile_6_8 #(
    parameter integer BACKEND_WORD_CREDITS = 4,
    parameter integer C_PARTITION_RESET = 0,
    parameter integer BACKEND_META_DEPTH = 8,
    parameter integer BACKEND_META_PTR_WIDTH = 3,
    parameter integer BACKEND_PQ_FIFO_DEPTH = 8,
    parameter integer BACKEND_PQ_FIFO_AW = 3,
    parameter integer A_WORDS = 1024,
    parameter integer PW_WORDS = 1024,
    parameter integer DW_WORDS = 512,
    parameter integer QBF_ROW_DEPTH = 512,
    parameter integer QBF_ROW_ADDR_WIDTH = 9,
    parameter integer B_WORDS = 2304,
    parameter integer C_WORDS = 1792,
    parameter integer D_WORDS = 512,
    parameter integer SHORTCUT_BASE_WORDS = 512,
    parameter integer DIRECT_PW_ROM = 0,
    // Enable a per-output-channel multiplier row for each output C8 group.
    parameter integer PER_CHANNEL_MULTIPLIER = 0,
    // Q1.14 is retained for the scalar path.  The per-channel path uses
    // unsigned Q2.12 so depthwise outliers do not saturate the multiplier.
    parameter integer MAIN_MULTIPLIER_SHIFT = 14
) (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          i_backend_rst_n,

    input  wire          tile_start,
    output wire          tile_start_ready,
    input  wire          tile_shift_y,
    input  wire          tile_shift_x,
    input  wire [3:0]    tile_valid_h,
    input  wire [3:0]    tile_valid_w,
    input  wire          tile_far_pad_y,
    input  wire          tile_far_pad_x,
    input  wire          shortcut_repack,
    input  wire          tile_storage_slot,

    input  wire          cfg_wr_en,
    output wire          cfg_ready,
    input  wire [3:0]    cfg_wr_addr,
    input  wire [31:0]   cfg_wr_data,
    input  wire          cfg_commit,
    output reg           cfg_commit_done,

    input  wire          a_load_en,
    input  wire [9:0]    a_load_addr,
    input  wire [127:0]  a_load_data,
    input  wire          a_load_storage_slot,
    input  wire          pw_load_en,
    input  wire [9:0]    pw_load_addr,
    input  wire [511:0]  pw_load_data,
    input  wire          dw_load_en,
    input  wire [8:0]    dw_load_addr,
    input  wire [127:0]  dw_load_data,
    input  wire          qbf_load_en,
    input  wire [9:0]    qbf_load_addr,
    input  wire [31:0]   qbf_load_data,
    input  wire          qscale_load_en,
    input  wire [9:0]    qscale_load_addr,
    input  wire [15:0]   qscale_load_data,

    output wire          pw_direct_rd_en,
    output wire [9:0]    pw_direct_rd_addr,
    input  wire [511:0]  pw_direct_rd_data,

    input  wire          token_valid,
    output wire          token_ready,
    input  wire [31:0]   token_uop,
    input  wire [63:0]   token_prefetch_data,

    output wire          write_valid,
    input  wire          write_ready,
    output wire [1:0]    write_kind,
    output wire [11:0]   write_addr,
    output wire [127:0]  write_data,
    output reg  [15:0]   write_byte_valid,

    input  wire          b_debug_en,
    input  wire [11:0]   b_debug_addr,
    output reg           b_debug_valid,
    output wire [127:0]  b_debug_data,
    input  wire          d_debug_en,
    input  wire [8:0]    d_debug_addr,
    input  wire          d_debug_storage_slot,
    output reg           d_debug_valid,
    output wire [127:0]  d_debug_data,

    output reg           tile_done,
    output wire          busy,
    output wire          a_compute_write_busy,
    output wire          status_error,
    output reg  [9:0]    debug_error_code,
    output wire [31:0]   accepted_uops,
    output wire [31:0]   accepted_transactions,
    output wire [31:0]   emitted_words,
    output reg  [31:0]   expansion_words,
    output reg  [31:0]   depthwise_words,
    output reg  [31:0]   projection_words
);

    localparam [1:0] OP_EXP = 2'd0;
    localparam [1:0] OP_DW = 2'd1;
    localparam [1:0] OP_PROJ = 2'd2;

    localparam [2:0] PH_IDLE = 3'd0;
    localparam [2:0] PH_EXP = 3'd1;
    localparam [2:0] PH_WAIT_EXP = 3'd2;
    localparam [2:0] PH_DW = 3'd3;
    localparam [2:0] PH_WAIT_DW = 3'd4;
    localparam [2:0] PH_PROJ = 3'd5;
    localparam [2:0] PH_WAIT_PROJ = 3'd6;
    localparam [2:0] PH_SC_REPACK = 3'd7;

    localparam [2:0] RP_IDLE  = 3'd0;
    localparam [2:0] RP_REQ0  = 3'd1;
    localparam [2:0] RP_WAIT0 = 3'd2;
    localparam [2:0] RP_REQ1  = 3'd3;
    localparam [2:0] RP_WAIT1 = 3'd4;
    localparam [2:0] RP_WRITE = 3'd5;

    localparam integer QBF_WORDS = QBF_ROW_DEPTH * 8;

    wire [127:0] a_mem_rd_data;
    wire [511:0] pw_mem_rd_data;
    wire [127:0] dw_mem_rd_data;
    wire [255:0] b_mem_rd_data2;
    wire [127:0] b_mem_rd_data = b_mem_rd_data2[127:0];
    wire [127:0] c_mem_rd_data;
    // D is only live after the B read phase has completed.  The physical
    // B bank is therefore reused for D writeback/debug storage; this saves
    // one complete 128-bit DRM bank set without changing the B double-word
    // read contract.  The phase controller guarantees no B/D overlap.
    wire [127:0] d_mem_rd_data = b_mem_rd_data2[127:0];
    wire [255:0] qbf_mem_rd_vector;
    wire [127:0] qscale_mem_rd_vector;

    reg [7:0] cfg_exp_weight_zero_q;
    reg [31:0] cfg_exp_relu6_limit_q;
    reg [15:0] cfg_exp_multiplier_q;
    reg [3:0] cfg_exp_multiplier_shift_q;
    reg [7:0] cfg_exp_output_zero_q;
    reg [7:0] cfg_dw_weight_zero_q;
    reg [31:0] cfg_dw_relu6_limit_q;
    reg [15:0] cfg_dw_multiplier_q;
    reg [3:0] cfg_dw_multiplier_shift_q;
    reg [7:0] cfg_dw_output_zero_q;
    reg [7:0] cfg_proj_weight_zero_q;
    reg [15:0] cfg_proj_multiplier_q;
    reg [3:0] cfg_proj_multiplier_shift_q;
    reg [7:0] cfg_proj_output_zero_q;
    reg [7:0] cfg_shortcut_zero_q;
    reg [15:0] cfg_shortcut_multiplier_q;
    reg [31:0] cfg_geometry_q;
    reg [9:0] cfg_written_q;
    reg cfg_committed_q;
    reg cfg_error_q;

    wire cfg_shortcut_en_q = cfg_geometry_q[18];
    wire cfg_stride2_q = cfg_geometry_q[29];
    wire cfg_exp_only_q = cfg_geometry_q[30];
    wire cfg_linear_exp_q = cfg_geometry_q[31];
    wire [5:0] input_groups_q = cfg_geometry_q[5:0];
    wire [5:0] expanded_groups_q = cfg_geometry_q[11:6];
    wire [5:0] output_groups_q = cfg_geometry_q[17:12];
    wire [9:0] proj_pw_base_q = cfg_geometry_q[28:19];
    wire [5:0] input_group_last = input_groups_q - 1'b1;
    wire [5:0] expanded_group_last = expanded_groups_q - 1'b1;
    wire [5:0] output_group_last = output_groups_q - 1'b1;
    wire [11:0] computed_proj_pw_base =
        {6'd0, input_groups_q} * {6'd0, expanded_groups_q};
    wire [11:0] computed_proj_pw_words =
        {6'd0, output_groups_q} * {6'd0, expanded_groups_q};
    wire [9:0] shortcut_base_words = SHORTCUT_BASE_WORDS[9:0];
    wire [5:0] output_pair_last = cfg_stride2_q ? 6'd7 : 6'd31;
    wire [6:0] output_pairs = cfg_stride2_q ? 7'd8 : 7'd32;
    wire [5:0] expansion_pair_last = cfg_exp_only_q ? 6'd31 : 6'd40;
    wire [6:0] expansion_pairs = cfg_exp_only_q ? 7'd32 : 7'd41;
    wire [31:0] expected_expansion_words =
        expanded_groups_q * expansion_pairs;
    wire [31:0] expected_depthwise_words = cfg_exp_only_q
        ? 32'd0 : expanded_groups_q * output_pairs;
    wire [31:0] expected_projection_words = cfg_exp_only_q
        ? 32'd0 : output_groups_q * output_pairs;
    wire [31:0] expected_uops =
        expected_expansion_words * input_groups_q
        + (cfg_exp_only_q ? 32'd0 : expected_depthwise_words * 2)
        + (cfg_exp_only_q ? 32'd0
                          : expected_projection_words * expanded_groups_q);
    wire [31:0] expected_transactions = expected_expansion_words
        + expected_depthwise_words + expected_projection_words;

    reg [2:0] phase_q;
    reg tile_active_q;
    reg tile_storage_slot_q;
    reg tile_shift_y_q;
    reg tile_shift_x_q;
    reg [3:0] tile_valid_h_q;
    reg [3:0] tile_valid_w_q;
    reg tile_far_pad_y_q;
    reg tile_far_pad_x_q;

    reg [5:0] exp_group_q;
    reg [5:0] exp_pair_q;
    reg [5:0] exp_chunk_q;
    reg [11:0] exp_pw_group_base_q;
    reg [3:0] exp_pair_y_q;
    reg [3:0] exp_pair_x_q;
    reg [5:0] proj_group_q;
    reg [5:0] proj_pair_q;
    reg [5:0] proj_chunk_q;
    reg [11:0] proj_pw_group_base_q;

    reg shortcut_repack_q;
    reg [2:0] repack_state_q;
    reg [5:0] repack_group_q;
    reg [5:0] repack_pair_q;
    reg [127:0] repack_word0_q;
    reg [127:0] repack_word_q;

    reg dw_group_active_q;
    reg [5:0] dw_next_group_q;
    reg [5:0] dw_current_group_q;
    reg [6:0] dw_feed_pixel_q;
    reg [5:0] dw_pair_q;
    reg [5:0] dw_swg_pair_q;
    reg dw_chunk_q;
    reg dw_window_half_q;
    reg dw_pair_ready_q;
    reg dw_lookahead_window0_valid_q;
    reg [575:0] dw_window0_q;
    reg [575:0] dw_window1_q;
    reg [575:0] dw_lookahead_window0_q;
    reg dw_tokens_done_q;
    reg dw_swg_done_q;
    reg [639:0] dw_weight_cache_q;
    reg [2:0] dw_weight_request_index_q;
    reg dw_weight_response_valid_q;
    reg [2:0] dw_weight_response_index_q;
    reg dw_weight_cache_valid_q;
    reg b_swg_word_valid_q;
    reg [4:0] dw_stride2_request_beat_q;
    reg dw_stride2_response_valid_q;

    reg protocol_error_q;

    wire [1:0] token_op = token_uop[1:0];
    wire [5:0] token_group = token_uop[7:2];
    wire [5:0] token_pair = token_uop[13:8];
    wire [5:0] token_chunk = token_uop[19:14];

    reg prefetch_valid_q;
    reg [31:0] prefetch_uop_q;
    // 首尾判断与token同拍寄存，避免配置比较串入后端ready和RAM使能。
    reg prefetch_first_q, prefetch_last_q;
    reg prefetch_pixel0_valid_q;
    reg prefetch_pixel1_valid_q;
    wire [1:0] issue_op = prefetch_uop_q[1:0];
    wire [5:0] issue_group = prefetch_uop_q[7:2];
    wire [5:0] issue_pair = prefetch_uop_q[13:8];
    wire [5:0] issue_chunk = prefetch_uop_q[19:14];

    wire backend_idle;
    wire backend_error;
    wire [9:0] backend_debug_error_code;
    wire backend_uop_ready;
    reg backend_uop_valid;
    reg backend_uop_first;
    reg backend_uop_last;
    reg [15:0] backend_lane_valid;
    reg [127:0] backend_term_valid;
    reg [1023:0] backend_qd;
    reg [1023:0] backend_qw;
    reg [1:0] backend_dest_kind;
    reg [11:0] backend_dest_addr;
    reg [255:0] backend_bias;
    reg [127:0] backend_shortcut;
    reg [7:0] backend_cfg_weight_zero;
    reg backend_cfg_relu6_en;
    reg [31:0] backend_cfg_relu6_limit;
    reg [15:0] backend_cfg_multiplier;
    // The PQ8 backend consumes a 16-lane multiplier vector.  The tile
    // operand layout is eight output channels by two spatial pixels, so the
    // multiplier stored in the C8 qscale row must be duplicated for the two
    // pixels of each channel.  Keeping this as a full 256-bit signal avoids
    // silently zero-extending the upper eight lanes (the old 128-bit signal
    // did exactly that for the second PQ group).
    reg [255:0] backend_cfg_multiplier_bus;
    reg [3:0] backend_cfg_multiplier_shift;
    integer multiplier_lane;
    reg [7:0] backend_cfg_output_zero;
    reg backend_cfg_shortcut_en;
    reg [7:0] backend_cfg_shortcut_zero;
    reg [15:0] backend_cfg_shortcut_multiplier;
    reg [7:0] backend_cfg_word_fill;

    wire backend_word_valid;
    wire [1:0] backend_word_kind;
    wire [11:0] backend_word_addr;
    wire [127:0] backend_word_data;
    wire backend_word_fire = backend_word_valid && write_ready;
    wire backend_uop_fire = backend_uop_valid && backend_uop_ready;

    wire start_fire = tile_start && tile_start_ready;
    wire backend_clear = start_fire;
    wire backend_pipeline_idle = backend_idle && !prefetch_valid_q;

    assign cfg_ready = !tile_active_q;
    // Tile completion already waits for backend_pipeline_idle. Keeping that
    // combinational status in start_ready feeds start_fire/backend_clear back
    // into the backend idle logic and creates a physical combinational loop.
    // dw_group_active_q is only asserted while tile_active_q is asserted;
    // removing the redundant term breaks the cross-hierarchy start/clear cone.
    assign tile_start_ready = cfg_committed_q && !tile_active_q;
    assign busy = tile_active_q;
    assign a_compute_write_busy = repack_state_q != RP_IDLE;
    assign status_error = protocol_error_q || cfg_error_q || backend_error;

    assign write_valid = backend_word_valid;
    assign write_kind = backend_word_kind;
    assign write_addr = backend_word_addr;
    assign write_data = backend_word_data;

    wire stride2_pair_valid;
    wire [2:0] stride2_pair_index;
    wire [1151:0] stride2_pair_windows;
    wire stride2_next_pair_valid;
    wire [2:0] stride2_next_pair_index;
    wire stride2_scan_done;
    wire stride2_swg_in_ready;
    wire stride2_pair_retire;
    wire last_dw_backend_fire = backend_uop_fire && (issue_op == OP_DW)
        && (issue_pair == output_pair_last) && (issue_chunk == 1);
    wire dw_second_backend_fire = backend_uop_fire
        && (issue_op == OP_DW) && (issue_chunk == 1);
    wire dw_stride1_retire_forward = !cfg_stride2_q
        && dw_second_backend_fire;
    wire [5:0] dw_stride1_expected_pair = dw_stride1_retire_forward
        ? (issue_pair + 1'b1) : dw_pair_q;
    wire dw_stride1_expected_chunk = dw_stride1_retire_forward
        ? 1'b0 : dw_chunk_q;
    wire stride2_head_matches_token = stride2_pair_valid
        && (token_pair == {3'd0, stride2_pair_index});
    wire stride2_next_matches_token = stride2_next_pair_valid
        && dw_second_backend_fire
        && (token_pair == {3'd0, stride2_next_pair_index});
    wire stride2_pair_available = stride2_head_matches_token
        || stride2_next_matches_token;
    wire dw_pair_promote_available;
    wire dw_stride1_current_pair_ready = dw_pair_ready_q
        && (token_pair == dw_pair_q);
    wire dw_stride1_next_pair_ready = dw_stride1_retire_forward
        && (token_pair == (issue_pair + 1'b1))
        && dw_pair_promote_available;
    wire dw_stride1_token_pair_ready = dw_stride1_current_pair_ready
        || dw_stride1_next_pair_ready;
    wire dw_operand_ready = cfg_stride2_q
        ? stride2_pair_available : dw_stride1_token_pair_ready;
    wire engine_can_submit =
        ((phase_q == PH_EXP) || (phase_q == PH_PROJ))
        || ((phase_q == PH_DW) && dw_group_active_q && dw_operand_ready
            && dw_weight_cache_valid_q);
    wire prefetch_slot_ready = !prefetch_valid_q || backend_uop_ready;
    wire token_fire = token_valid && token_ready;
    // A missing look-ahead pair is the only remaining stride-1 drain case.
    // A live second look-ahead window may retire the current pair and admit
    // the next pair's chunk 0 on the same cycle.
    wire dw_pair_drain = !cfg_stride2_q && (phase_q == PH_DW)
        && prefetch_valid_q && backend_uop_ready
        && (issue_op == OP_DW) && (issue_chunk == 1)
        && !dw_stride1_token_pair_ready;
    assign token_ready = engine_can_submit && prefetch_slot_ready
        && !dw_pair_drain;
    wire [63:0] swg_in_data_word = dw_feed_pixel_q[0]
        ? b_mem_rd_data[127:64] : b_mem_rd_data[63:0];
    wire swg_in_valid = !cfg_stride2_q && dw_group_active_q
        && b_swg_word_valid_q
        && (dw_feed_pixel_q < 81);
    wire swg_in_ready;
    wire swg_out_valid;
    wire swg_out_ready;
    wire [575:0] swg_out_window;
    wire [15:0] swg_out_x;
    wire [15:0] swg_out_y;
    wire swg_selected = cfg_stride2_q
        ? ((swg_out_x < 8) && (swg_out_y < 8)
           && swg_out_x[0] && swg_out_y[0])
        : ((swg_out_x < 8) && (swg_out_y < 8));
    wire dw_prefetch_uses_window = prefetch_valid_q
        && (issue_op == OP_DW);
    assign dw_pair_promote_available = swg_out_valid && swg_selected
        && dw_window_half_q && dw_lookahead_window0_valid_q;
    assign swg_out_ready = !cfg_stride2_q && dw_group_active_q
        && (!swg_selected
            || !dw_pair_ready_q
            || !dw_window_half_q
            || dw_second_backend_fire);
    wire swg_out_fire = swg_out_valid && swg_out_ready;
    wire swg_in_fire = swg_in_valid && swg_in_ready;
    wire swg_last_fire = swg_out_fire
        && (swg_out_x == 8) && (swg_out_y == 8);
    wire finish_dw_group = dw_group_active_q
        && (dw_tokens_done_q || last_dw_backend_fire)
        && (cfg_stride2_q
            ? stride2_scan_done : (dw_swg_done_q || swg_last_fire));
    assign stride2_pair_retire = cfg_stride2_q
        && dw_second_backend_fire;

    wire [3:0] repack_y = {1'd0, repack_pair_q[4:2]};
    wire [3:0] repack_x0 = {1'd0, repack_pair_q[1:0], 1'b0};
    wire [3:0] repack_x1 = repack_x0 + 1'b1;
    wire [6:0] repack_linear0 =
        ({3'd0, repack_y} << 3) + {3'd0, repack_y} +
        {3'd0, repack_x0};
    wire [6:0] repack_linear1 = repack_linear0 + 1'b1;
    wire [5:0] repack_source_pair0 = repack_linear0[6:1];
    wire [5:0] repack_source_pair1 = repack_linear1[6:1];
    wire [9:0] repack_group_base =
        ({4'd0, repack_group_q} << 5) +
        ({4'd0, repack_group_q} << 3) +
        {4'd0, repack_group_q};
    wire [9:0] repack_source_addr0 = repack_group_base +
        {4'd0, repack_source_pair0};
    wire [9:0] repack_source_addr1 = repack_group_base +
        {4'd0, repack_source_pair1};
    wire [9:0] repack_target_addr = shortcut_base_words +
        {repack_group_q, 5'b00000} + {4'd0, repack_pair_q};
    wire repack_spatial0_valid =
        (repack_y >= {3'd0, tile_shift_y_q}) &&
        (repack_y < ({3'd0, tile_shift_y_q} + tile_valid_h_q)) &&
        (repack_x0 >= {3'd0, tile_shift_x_q}) &&
        (repack_x0 < ({3'd0, tile_shift_x_q} + tile_valid_w_q));
    wire repack_spatial1_valid =
        (repack_y >= {3'd0, tile_shift_y_q}) &&
        (repack_y < ({3'd0, tile_shift_y_q} + tile_valid_h_q)) &&
        (repack_x1 >= {3'd0, tile_shift_x_q}) &&
        (repack_x1 < ({3'd0, tile_shift_x_q} + tile_valid_w_q));
    wire [63:0] repack_fill64 = {8{cfg_shortcut_zero_q}};
    wire repack_read0 = (phase_q == PH_SC_REPACK) &&
        (repack_state_q == RP_REQ0);
    wire repack_read1 = (phase_q == PH_SC_REPACK) &&
        (repack_state_q == RP_REQ1);
    wire repack_write = (phase_q == PH_SC_REPACK) &&
        (repack_state_q == RP_WRITE);
    wire repack_last_word = (repack_group_q == output_group_last) &&
        (repack_pair_q == 6'd31);

    wire [11:0] token_exp_a_addr =
        ({6'd0, token_chunk} << 5)
        + ({6'd0, token_chunk} << 3)
        + {6'd0, token_chunk} + {6'd0, token_pair};
    wire [11:0] token_exp_pw_addr =
        exp_pw_group_base_q + {6'd0, token_chunk};
    wire [11:0] token_proj_c_addr = cfg_stride2_q
        ? (token_chunk * 8 + token_pair)
        : (token_chunk * 32 + token_pair);
    wire [11:0] token_proj_pw_addr = proj_pw_base_q
        + proj_pw_group_base_q + {6'd0, token_chunk};
    wire [11:0] token_shortcut_addr = shortcut_base_words
        + token_group * 32 + token_pair;
    wire [8:0] token_qbf_row = (token_op == OP_EXP)
        ? {3'd0, token_group}
        : ((token_op == OP_DW)
            ? ({3'd0, expanded_groups_q} + {3'd0, token_group})
            : (({3'd0, expanded_groups_q} << 1)
                + {3'd0, token_group}));

    wire a_token_rd_en = token_fire && ((token_op == OP_EXP)
        || ((token_op == OP_PROJ) && cfg_shortcut_en_q));
    wire [9:0] a_token_rd_addr = (token_op == OP_EXP)
        ? token_exp_a_addr[9:0] : token_shortcut_addr[9:0];
    wire a_mem_rd_en = repack_read0 || repack_read1 || a_token_rd_en;
    wire [9:0] a_mem_logical_rd_addr = repack_read0
        ? repack_source_addr0
        : (repack_read1 ? repack_source_addr1 : a_token_rd_addr);
    wire [9:0] a_mem_rd_addr =
        {a_mem_logical_rd_addr[8:0], tile_storage_slot_q};
    wire [9:0] a_load_physical_addr =
        {a_load_addr[8:0], a_load_storage_slot};
    wire [9:0] repack_target_physical_addr =
        {repack_target_addr[8:0], tile_storage_slot_q};
    wire pw_mem_rd_en = token_fire
        && ((token_op == OP_EXP) || (token_op == OP_PROJ));
    wire pw_mem_addr_valid = token_valid
        && ((token_op == OP_EXP) || (token_op == OP_PROJ));
    wire [9:0] pw_mem_rd_addr = (token_op == OP_EXP)
        ? token_exp_pw_addr[9:0] : token_proj_pw_addr[9:0];
    wire c_mem_rd_en = token_fire && (token_op == OP_PROJ);
    wire [10:0] c_mem_rd_addr = token_proj_c_addr[10:0];
    wire qbf_mem_rd_en = token_fire;
    wire qscale_mem_rd_en = token_fire && (PER_CHANNEL_MULTIPLIER != 0);

    wire dw_weight_read_request = dw_group_active_q
        && !dw_weight_cache_valid_q
        && (dw_weight_request_index_q < 5);
    wire [8:0] dw_mem_rd_addr =
        ({3'd0, dw_current_group_q} << 2)
        + {3'd0, dw_current_group_q}
        + {6'd0, dw_weight_request_index_q};

    wire b_swg_initial_request = !cfg_stride2_q && dw_group_active_q
        && !b_swg_word_valid_q && (dw_feed_pixel_q < 81);
    wire b_swg_next_request = swg_in_fire && dw_feed_pixel_q[0]
        && (dw_feed_pixel_q < 80);
    wire b_swg_read_request = b_swg_initial_request
        || b_swg_next_request;
    wire [11:0] b_group_base =
        ({6'd0, dw_current_group_q} << 5)
        + ({6'd0, dw_current_group_q} << 3)
        + {6'd0, dw_current_group_q};
    // 当前/下一字地址在组启动或消费奇数像素时预计算。
    // ready只选择已寄存地址，不再经过pixel+1与group_base相加。
    reg [11:0] b_swg_addr_q, b_swg_next_addr_q;
    wire [11:0] b_start_base = ({6'd0, token_group} << 5)
        + ({6'd0, token_group} << 3) + {6'd0, token_group};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_swg_addr_q <= 0; b_swg_next_addr_q <= 1;
        end else if (backend_clear) begin
            b_swg_addr_q <= 0; b_swg_next_addr_q <= 1;
        end else if ((phase_q == PH_DW) && !dw_group_active_q && token_valid) begin
            b_swg_addr_q <= b_start_base;
            b_swg_next_addr_q <= b_start_base + 1'b1;
        end else if (!cfg_stride2_q && b_swg_next_request) begin
            b_swg_addr_q <= b_swg_next_addr_q;
            b_swg_next_addr_q <= b_swg_next_addr_q + 1'b1;
        end
    end
    wire [11:0] b_swg_rd_addr = b_swg_next_request
        ? b_swg_next_addr_q : b_swg_addr_q;
    wire stride2_response_slot_ready = !dw_stride2_response_valid_q
        || stride2_swg_in_ready;
    wire b_stride2_read_request = cfg_stride2_q && dw_group_active_q
        && (dw_stride2_request_beat_q < 21)
        && stride2_response_slot_ready;
    wire [11:0] b_stride2_rd_addr0 = b_group_base
        + {6'd0, dw_stride2_request_beat_q, 1'b0};
    wire b_debug_read_request = b_debug_en && !tile_active_q;
    wire d_debug_read_request = d_debug_en;
    // 输出由外部双槽捕获；内部D不再随输入A槽翻转。
    wire [8:0] d_write_physical_addr = backend_word_addr[8:0];
    wire [8:0] d_debug_physical_addr = d_debug_addr |
        {d_debug_storage_slot, 8'd0};

    npu_m4k_wide_sdp_bank #(
        .WORD_BYTES(16), .DEPTH_WORDS(A_WORDS), .ADDR_WIDTH(10)
    ) u_a_mem (
        .clk(clk), .rst_n(rst_n),
        .wr_data(repack_write ? repack_word_q : a_load_data),
        .wr_addr(repack_write ? repack_target_physical_addr
                              : a_load_physical_addr),
        .wr_en(repack_write || a_load_en),
        .rd_addr(a_mem_rd_addr), .rd_en(a_mem_rd_en),
        .rd_data(a_mem_rd_data)
    );

    generate
        if (DIRECT_PW_ROM != 0) begin : g_direct_pw_rom
            assign pw_direct_rd_en = pw_mem_addr_valid;
            assign pw_direct_rd_addr = pw_mem_rd_addr;
            assign pw_mem_rd_data = pw_direct_rd_data;
        end else begin : g_local_pw_mem
            assign pw_direct_rd_en = 1'b0;
            assign pw_direct_rd_addr = 10'd0;
            npu_m4k_wide_sdp_bank #(
                .WORD_BYTES(64), .DEPTH_WORDS(PW_WORDS), .ADDR_WIDTH(10)
            ) u_pw_mem (
                .clk(clk), .rst_n(rst_n),
                .wr_data(pw_load_data), .wr_addr(pw_load_addr),
                .wr_en(pw_load_en && cfg_ready),
                .rd_addr(pw_mem_rd_addr), .rd_en(pw_mem_rd_en),
                .rd_data(pw_mem_rd_data)
            );
        end
    endgenerate

    npu_m4k_wide_sdp_bank #(
        .WORD_BYTES(16), .DEPTH_WORDS(DW_WORDS), .ADDR_WIDTH(9),
        .SINGLE_PORT(1)
    ) u_dw_mem (
        .clk(clk), .rst_n(rst_n),
        .wr_data(dw_load_data), .wr_addr(dw_load_addr),
        .wr_en(dw_load_en && cfg_ready),
        .rd_addr(dw_mem_rd_addr), .rd_en(dw_weight_read_request),
        .rd_data(dw_mem_rd_data)
    );

    // QBF and qscale are loaded as eight narrow channel words per row but
    // always consumed together.  The packed bank keeps their original
    // one-cycle read contract while using three 128-bit DRM banks instead of
    // sixteen narrow banks.
    npu_m4k_qparams_bank #(
        .ROW_DEPTH(QBF_ROW_DEPTH),
        .ROW_ADDR_WIDTH(QBF_ROW_ADDR_WIDTH),
        .SINGLE_PORT(1)
    ) u_qparams_mem (
        .clk(clk), .rst_n(rst_n),
        .qbf_wr_en(qbf_load_en && cfg_ready),
        .qbf_wr_addr(qbf_load_addr), .qbf_wr_data(qbf_load_data),
        .qscale_wr_en(qscale_load_en && cfg_ready),
        .qscale_wr_addr(qscale_load_addr), .qscale_wr_data(qscale_load_data),
        .rd_en(qbf_mem_rd_en || qscale_mem_rd_en),
        .rd_row(token_qbf_row),
        .qbf_rd_data(qbf_mem_rd_vector),
        .qscale_rd_data(qscale_mem_rd_vector)
    );

    wire bd_d_write = backend_word_fire && (backend_word_kind == OP_PROJ)
        && (backend_word_addr < expected_projection_words)
        && (backend_word_addr < D_WORDS);
    wire bd_b_write = backend_word_fire && (backend_word_kind == OP_EXP)
        && (backend_word_addr < B_WORDS);
    wire bd_read_d = d_debug_read_request;
    wire bd_read_b = b_debug_read_request || b_swg_read_request
        || b_stride2_read_request;
    wire [11:0] bd_wr_addr = bd_d_write
        ? {{3{1'b0}}, d_write_physical_addr}
        : backend_word_addr;
    wire [11:0] bd_rd_base_addr = bd_read_d
        ? {{3{1'b0}}, d_debug_physical_addr}
        : (b_debug_read_request ? b_debug_addr
            : (cfg_stride2_q ? b_stride2_rd_addr0 : b_swg_rd_addr));

    npu_m4k_wide_banked2 #(
        .WORD_BYTES(16), .DEPTH_WORDS(B_WORDS), .ADDR_WIDTH(12),
        .LOCAL_ADDR_WIDTH(9), .SINGLE_PORT(1)
    ) u_b_d_mem (
        .clk(clk), .rst_n(rst_n),
        .wr_data(backend_word_data), .wr_addr(bd_wr_addr),
        .wr_en(bd_b_write || bd_d_write),
        .rd_base_addr(bd_rd_base_addr),
        .rd_en(bd_read_b || bd_read_d),
        .rd_data2(b_mem_rd_data2)
    );

    npu_m4k_wide_sdp_bank #(
        .WORD_BYTES(16), .DEPTH_WORDS(C_WORDS), .ADDR_WIDTH(11),
        .SINGLE_PORT(1)
    ) u_c_mem (
        .clk(clk), .rst_n(rst_n),
        .wr_data(backend_word_data),
        .wr_addr(backend_word_addr[10:0]),
        .wr_en(backend_word_fire && (backend_word_kind == OP_DW)
            && (backend_word_addr < C_WORDS)),
        .rd_addr(c_mem_rd_addr), .rd_en(c_mem_rd_en),
        .rd_data(c_mem_rd_data)
    );

    assign b_debug_data = b_mem_rd_data;
    assign d_debug_data = d_mem_rd_data;

    npu_depthwise_swg_3x3 #(
        .IMAGE_WIDTH(9), .IMAGE_HEIGHT(9),
        .LANE_WIDTH(8), .CHANNELS(8), .COORD_WIDTH(16)
    ) u_swg (
        .clk(clk), .rst_n(rst_n && !backend_clear),
        .in_valid(swg_in_valid), .in_ready(swg_in_ready),
        .in_data(swg_in_data_word),
        .out_valid(swg_out_valid), .out_ready(swg_out_ready),
        .out_window(swg_out_window), .out_x(swg_out_x),
        .out_y(swg_out_y)
    );

    npu_depthwise_swg_stride2_4px u_stride2_swg (
        .clk(clk),
        .rst_n(rst_n),
        .clear(backend_clear || finish_dw_group ||
            ((phase_q == PH_DW) && !dw_group_active_q && token_valid)),
        .in_valid(dw_stride2_response_valid_q),
        .in_ready(stride2_swg_in_ready),
        .in_data(b_mem_rd_data2),
        .pair_valid(stride2_pair_valid),
        .pair_ready(stride2_pair_retire),
        .pair_index(stride2_pair_index),
        .pair_windows(stride2_pair_windows),
        .next_pair_valid(stride2_next_pair_valid),
        .next_pair_index(stride2_next_pair_index),
        .scan_done(stride2_scan_done)
    );

    // Fixed-index operand layouts avoid the deep replicated mux cone created
    // by the former procedural loops across all 128 MAC terms.
    wire [15:0] exp_pack_lane_valid;
    wire [127:0] exp_pack_term_valid;
    wire [1023:0] exp_pack_qd;
    wire [1023:0] exp_pack_qw;
    wire [15:0] proj_pack_lane_valid;
    wire [127:0] proj_pack_term_valid;
    wire [1023:0] proj_pack_qd;
    wire [1023:0] proj_pack_qw;
    wire [15:0] dw_pack_lane_valid;
    wire [127:0] dw_pack_term_valid;
    wire [1023:0] dw_pack_qd;
    wire [1023:0] dw_pack_qw;

    genvar pack_filter;
    genvar pack_pixel;
    genvar pack_term;
    generate
        for (pack_filter = 0; pack_filter < 8;
             pack_filter = pack_filter + 1) begin : g_pack_filter
            for (pack_pixel = 0; pack_pixel < 2;
                 pack_pixel = pack_pixel + 1) begin : g_pack_pixel
                localparam integer PACK_LANE = pack_filter*2 + pack_pixel;
                wire pack_pixel_valid = pack_pixel
                    ? prefetch_pixel1_valid_q : prefetch_pixel0_valid_q;

                assign exp_pack_lane_valid[PACK_LANE] = pack_pixel_valid;
                assign proj_pack_lane_valid[PACK_LANE] = pack_pixel_valid;
                for (pack_term = 0; pack_term < 8;
                     pack_term = pack_term + 1) begin : g_pack_term
                    localparam integer PACK_TERM = PACK_LANE*8 + pack_term;
                    assign exp_pack_term_valid[PACK_TERM] = pack_pixel_valid;
                    assign proj_pack_term_valid[PACK_TERM] = pack_pixel_valid;
                    assign exp_pack_qd[PACK_TERM*8 +: 8] = a_mem_rd_data[
                        (pack_pixel*8 + pack_term)*8 +: 8
                    ];
                    assign proj_pack_qd[PACK_TERM*8 +: 8] = c_mem_rd_data[
                        (pack_pixel*8 + pack_term)*8 +: 8
                    ];
                    assign exp_pack_qw[PACK_TERM*8 +: 8] = pw_mem_rd_data[
                        (pack_filter*8 + pack_term)*8 +: 8
                    ];
                    assign proj_pack_qw[PACK_TERM*8 +: 8] = pw_mem_rd_data[
                        (pack_filter*8 + pack_term)*8 +: 8
                    ];
                end
            end
        end
    endgenerate

    wire [575:0] dw_pack_window0 = cfg_stride2_q
        ? stride2_pair_windows[575:0] : dw_window0_q;
    wire [575:0] dw_pack_window1 = cfg_stride2_q
        ? stride2_pair_windows[1151:576] : dw_window1_q;

    npu_dw_operand_packer_p2_v1 u_dw_operand_packer (
        .window0(dw_pack_window0),
        .window1(dw_pack_window1),
        .weight_cache(dw_weight_cache_q),
        .issue_chunk(issue_chunk[0]),
        .issue_pair(issue_pair),
        .cfg_stride2(cfg_stride2_q),
        .pixel0_valid(prefetch_pixel0_valid_q),
        .pixel1_valid(prefetch_pixel1_valid_q),
        .pad_zero(cfg_exp_output_zero_q),
        .lane_valid(dw_pack_lane_valid),
        .term_valid(dw_pack_term_valid),
        .qd(dw_pack_qd),
        .qw(dw_pack_qw)
    );

    integer exp_y0;
    integer exp_x0;
    integer exp_y1;
    integer exp_x1;
    integer exp_linear0;
    integer exp_linear1;
    integer exp_output_y;
    integer exp_output_x0;
    integer exp_output_x1;
    integer proj_y;
    integer proj_x0;
    integer proj_x1;
    integer dw_y;
    integer dw_x0;
    integer dw_x1;
    reg exp_pixel0_valid;
    reg exp_pixel1_valid;
    reg proj_pixel0_valid;
    reg proj_pixel1_valid;
    reg dw_pixel0_valid;
    reg dw_pixel1_valid;

    always @(*) begin
        backend_uop_valid = prefetch_valid_q;
        backend_uop_first = 1'b0;
        backend_uop_last = 1'b0;
        backend_lane_valid = 16'd0;
        backend_term_valid = 128'd0;
        backend_qd = 1024'd0;
        backend_qw = 1024'd0;
        backend_dest_kind = issue_op;
        backend_dest_addr = 11'd0;
        backend_bias = 256'd0;
        backend_shortcut = 128'd0;
        backend_cfg_weight_zero = 8'd0;
        backend_cfg_relu6_en = 1'b0;
        backend_cfg_relu6_limit = 32'd0;
        backend_cfg_multiplier = 16'd0;
        backend_cfg_multiplier_bus = 256'd0;
        backend_cfg_multiplier_shift = MAIN_MULTIPLIER_SHIFT;
        backend_cfg_output_zero = 8'd0;
        backend_cfg_shortcut_en = 1'b0;
        backend_cfg_shortcut_zero = 8'd0;
        backend_cfg_shortcut_multiplier = 16'd0;
        backend_cfg_word_fill = 8'd0;
        exp_y0 = exp_pair_y_q;
        exp_x0 = exp_pair_x_q;
        exp_y1 = (exp_pair_x_q == 8) ? exp_pair_y_q + 1 : exp_pair_y_q;
        exp_x1 = (exp_pair_x_q == 8) ? 0 : exp_pair_x_q + 1;
        exp_linear0 = exp_pair_q * 2;
        exp_linear1 = exp_linear0 + 1;
        exp_output_y = exp_pair_q >> 2;
        exp_output_x0 = (exp_pair_q & 3) << 1;
        exp_output_x1 = exp_output_x0 + 1;
        if (cfg_exp_only_q) begin
            exp_pixel0_valid = (exp_output_y < tile_valid_h_q)
                && (exp_output_x0 < tile_valid_w_q);
            exp_pixel1_valid = (exp_output_y < tile_valid_h_q)
                && (exp_output_x1 < tile_valid_w_q);
        end else if (cfg_stride2_q) begin
            exp_pixel0_valid = (exp_linear0 < 81)
                && (exp_y0 >= (tile_shift_y_q ? 0 : 1))
                && (exp_y0 <= ({tile_valid_h_q, 1'b0}
                    - {4'd0, tile_far_pad_y_q}))
                && (exp_x0 >= (tile_shift_x_q ? 0 : 1))
                && (exp_x0 <= ({tile_valid_w_q, 1'b0}
                    - {4'd0, tile_far_pad_x_q}));
            exp_pixel1_valid = (exp_linear1 < 81)
                && (exp_y1 >= (tile_shift_y_q ? 0 : 1))
                && (exp_y1 <= ({tile_valid_h_q, 1'b0}
                    - {4'd0, tile_far_pad_y_q}))
                && (exp_x1 >= (tile_shift_x_q ? 0 : 1))
                && (exp_x1 <= ({tile_valid_w_q, 1'b0}
                    - {4'd0, tile_far_pad_x_q}));
        end else begin
            exp_pixel0_valid = (exp_y0 < 9) && (exp_x0 < 9)
                && (exp_y0 <= tile_shift_y_q + tile_valid_h_q
                    - tile_far_pad_y_q)
                && (exp_x0 <= tile_shift_x_q + tile_valid_w_q
                    - tile_far_pad_x_q);
            exp_pixel1_valid = (exp_y1 < 9) && (exp_x1 < 9)
                && (exp_y1 <= tile_shift_y_q + tile_valid_h_q
                    - tile_far_pad_y_q)
                && (exp_x1 <= tile_shift_x_q + tile_valid_w_q
                    - tile_far_pad_x_q);
        end

        proj_y = cfg_stride2_q ? (proj_pair_q >> 1) : (proj_pair_q >> 2);
        proj_x0 = cfg_stride2_q
            ? ((proj_pair_q & 1) << 1) : ((proj_pair_q & 3) << 1);
        proj_x1 = proj_x0 + 1;
        if (cfg_stride2_q) begin
            proj_pixel0_valid = (proj_y < tile_valid_h_q)
                && (proj_x0 < tile_valid_w_q);
            proj_pixel1_valid = (proj_y < tile_valid_h_q)
                && (proj_x1 < tile_valid_w_q);
        end else begin
            proj_pixel0_valid = (proj_y >= tile_shift_y_q)
                && (proj_y < tile_shift_y_q + tile_valid_h_q)
                && (proj_x0 >= tile_shift_x_q)
                && (proj_x0 < tile_shift_x_q + tile_valid_w_q);
            proj_pixel1_valid = (proj_y >= tile_shift_y_q)
                && (proj_y < tile_shift_y_q + tile_valid_h_q)
                && (proj_x1 >= tile_shift_x_q)
                && (proj_x1 < tile_shift_x_q + tile_valid_w_q);
        end

        dw_y = cfg_stride2_q ? (token_pair >> 1) : (token_pair >> 2);
        dw_x0 = cfg_stride2_q
            ? ((token_pair & 1) << 1) : ((token_pair & 3) << 1);
        dw_x1 = dw_x0 + 1;
        if (cfg_stride2_q) begin
            dw_pixel0_valid = (dw_y < tile_valid_h_q)
                && (dw_x0 < tile_valid_w_q);
            dw_pixel1_valid = (dw_y < tile_valid_h_q)
                && (dw_x1 < tile_valid_w_q);
        end else begin
            dw_pixel0_valid = (dw_y >= tile_shift_y_q)
                && (dw_y < tile_shift_y_q + tile_valid_h_q)
                && (dw_x0 >= tile_shift_x_q)
                && (dw_x0 < tile_shift_x_q + tile_valid_w_q);
            dw_pixel1_valid = (dw_y >= tile_shift_y_q)
                && (dw_y < tile_shift_y_q + tile_valid_h_q)
                && (dw_x1 >= tile_shift_x_q)
                && (dw_x1 < tile_shift_x_q + tile_valid_w_q);
        end

        if (issue_op == OP_EXP) begin
            backend_uop_first = prefetch_first_q;
            backend_uop_last = prefetch_last_q;
            backend_dest_kind = OP_EXP;
            backend_dest_addr = issue_group * 41 + issue_pair;
            backend_cfg_weight_zero = cfg_exp_weight_zero_q;
            backend_cfg_relu6_en = !cfg_linear_exp_q;
            backend_cfg_relu6_limit = cfg_exp_relu6_limit_q;
            backend_cfg_multiplier = cfg_exp_multiplier_q;
            backend_cfg_multiplier_shift = cfg_exp_multiplier_shift_q;
			if (PER_CHANNEL_MULTIPLIER != 0) begin
                for (multiplier_lane = 0; multiplier_lane < 8; multiplier_lane = multiplier_lane + 1) begin
                    backend_cfg_multiplier_bus[(multiplier_lane*2)*16 +: 16] =
                        qscale_mem_rd_vector[multiplier_lane*16 +: 16];
                    backend_cfg_multiplier_bus[(multiplier_lane*2+1)*16 +: 16] =
                        qscale_mem_rd_vector[multiplier_lane*16 +: 16];
                end
            end else begin
                backend_cfg_multiplier_bus = {16{cfg_exp_multiplier_q}};
            end
            backend_cfg_output_zero = cfg_exp_output_zero_q;
            backend_cfg_word_fill = cfg_exp_output_zero_q;
            backend_bias = qbf_mem_rd_vector;
            backend_lane_valid = exp_pack_lane_valid;
            backend_term_valid = exp_pack_term_valid;
            backend_qd = exp_pack_qd;
            backend_qw = exp_pack_qw;
        end else if (issue_op == OP_DW) begin
            backend_uop_first = prefetch_first_q;
            backend_uop_last = prefetch_last_q;
            backend_dest_kind = OP_DW;
            backend_dest_addr = cfg_stride2_q
                ? (issue_group * 8 + issue_pair)
                : (issue_group * 32 + issue_pair);
            backend_cfg_weight_zero = cfg_dw_weight_zero_q;
            backend_cfg_relu6_en = 1'b1;
            backend_cfg_relu6_limit = cfg_dw_relu6_limit_q;
            backend_cfg_multiplier = cfg_dw_multiplier_q;
            backend_cfg_multiplier_shift = cfg_dw_multiplier_shift_q;
            if (PER_CHANNEL_MULTIPLIER != 0) begin
                for (multiplier_lane = 0; multiplier_lane < 8; multiplier_lane = multiplier_lane + 1) begin
                    backend_cfg_multiplier_bus[(multiplier_lane*2)*16 +: 16] =
                        qscale_mem_rd_vector[multiplier_lane*16 +: 16];
                    backend_cfg_multiplier_bus[(multiplier_lane*2+1)*16 +: 16] =
                        qscale_mem_rd_vector[multiplier_lane*16 +: 16];
                end
            end else begin
                backend_cfg_multiplier_bus = {16{cfg_dw_multiplier_q}};
            end
            backend_cfg_output_zero = cfg_dw_output_zero_q;
            backend_cfg_word_fill = cfg_dw_output_zero_q;
            backend_bias = qbf_mem_rd_vector;
            backend_lane_valid = dw_pack_lane_valid;
            backend_term_valid = dw_pack_term_valid;
            backend_qd = dw_pack_qd;
            backend_qw = dw_pack_qw;
        end else if (issue_op == OP_PROJ) begin
            backend_uop_first = prefetch_first_q;
            backend_uop_last = prefetch_last_q;
            backend_dest_kind = OP_PROJ;
            backend_dest_addr = cfg_stride2_q
                ? (issue_group * 8 + issue_pair)
                : (issue_group * 32 + issue_pair);
            backend_cfg_weight_zero = cfg_proj_weight_zero_q;
            backend_cfg_multiplier = cfg_proj_multiplier_q;
            backend_cfg_multiplier_shift = cfg_proj_multiplier_shift_q;
            if (PER_CHANNEL_MULTIPLIER != 0) begin
                for (multiplier_lane = 0; multiplier_lane < 8; multiplier_lane = multiplier_lane + 1) begin
                    backend_cfg_multiplier_bus[(multiplier_lane*2)*16 +: 16] =
                        qscale_mem_rd_vector[multiplier_lane*16 +: 16];
                    backend_cfg_multiplier_bus[(multiplier_lane*2+1)*16 +: 16] =
                        qscale_mem_rd_vector[multiplier_lane*16 +: 16];
                end
            end else begin
                backend_cfg_multiplier_bus = {16{cfg_proj_multiplier_q}};
            end
            backend_cfg_output_zero = cfg_proj_output_zero_q;
            backend_cfg_shortcut_en = cfg_shortcut_en_q;
            backend_cfg_shortcut_zero = cfg_shortcut_zero_q;
            backend_cfg_shortcut_multiplier = cfg_shortcut_multiplier_q;
            backend_cfg_word_fill = cfg_proj_output_zero_q;
            backend_shortcut = cfg_shortcut_en_q
                ? a_mem_rd_data : 128'd0;
            backend_bias = qbf_mem_rd_vector;
            backend_lane_valid = proj_pack_lane_valid;
            backend_term_valid = proj_pack_term_valid;
            backend_qd = proj_pack_qd;
            backend_qw = proj_pack_qw;
        end
    end

    // A token is accepted together with all synchronous M4K read requests.
    // On the following cycle, this elastic slot presents the matching token
    // and memory outputs to the backend. It holds both whenever the backend
    // applies backpressure.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prefetch_valid_q <= 1'b0;
            prefetch_uop_q <= 32'd0;
            prefetch_first_q <= 0; prefetch_last_q <= 0;
            prefetch_pixel0_valid_q <= 1'b0;
            prefetch_pixel1_valid_q <= 1'b0;
        end else if (backend_clear) begin
            prefetch_valid_q <= 1'b0;
            prefetch_uop_q <= 32'd0;
            prefetch_first_q <= 0; prefetch_last_q <= 0;
            prefetch_pixel0_valid_q <= 1'b0;
            prefetch_pixel1_valid_q <= 1'b0;
        end else if (token_fire) begin
            prefetch_valid_q <= 1'b1;
            prefetch_uop_q <= token_uop;
            prefetch_first_q <= token_chunk == 0;
            prefetch_last_q <= (token_op == OP_EXP) ? token_chunk == input_group_last
                : ((token_op == OP_DW) ? token_chunk == 1
                    : ((token_op == OP_PROJ) && token_chunk == expanded_group_last));
            case (token_op)
                OP_EXP: begin
                    prefetch_pixel0_valid_q <= exp_pixel0_valid;
                    prefetch_pixel1_valid_q <= exp_pixel1_valid;
                end
                OP_DW: begin
                    prefetch_pixel0_valid_q <= dw_pixel0_valid;
                    prefetch_pixel1_valid_q <= dw_pixel1_valid;
                end
                OP_PROJ: begin
                    prefetch_pixel0_valid_q <= proj_pixel0_valid;
                    prefetch_pixel1_valid_q <= proj_pixel1_valid;
                end
                default: begin
                    prefetch_pixel0_valid_q <= 1'b0;
                    prefetch_pixel1_valid_q <= 1'b0;
                end
            endcase
        end else if (backend_uop_fire) begin
            prefetch_valid_q <= 1'b0;
        end
    end

    // Five 128-bit words contain the 3x3 depthwise weights for one group.
    // Fetch them once at group start, then reuse the cache for every pair.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dw_weight_cache_q <= 640'd0;
            dw_weight_request_index_q <= 3'd0;
            dw_weight_response_valid_q <= 1'b0;
            dw_weight_response_index_q <= 3'd0;
            dw_weight_cache_valid_q <= 1'b0;
        end else if (backend_clear) begin
            dw_weight_cache_q <= 640'd0;
            dw_weight_request_index_q <= 3'd0;
            dw_weight_response_valid_q <= 1'b0;
            dw_weight_response_index_q <= 3'd0;
            dw_weight_cache_valid_q <= 1'b0;
        end else if ((phase_q == PH_DW) && !dw_group_active_q
                && token_valid) begin
            dw_weight_request_index_q <= 3'd0;
            dw_weight_response_valid_q <= 1'b0;
            dw_weight_response_index_q <= 3'd0;
            dw_weight_cache_valid_q <= 1'b0;
        end else if (finish_dw_group) begin
            dw_weight_request_index_q <= 3'd0;
            dw_weight_response_valid_q <= 1'b0;
            dw_weight_response_index_q <= 3'd0;
            dw_weight_cache_valid_q <= 1'b0;
        end else begin
            dw_weight_response_valid_q <= dw_weight_read_request;
            if (dw_weight_read_request) begin
                dw_weight_response_index_q <= dw_weight_request_index_q;
                dw_weight_request_index_q <= dw_weight_request_index_q + 1'b1;
            end
            if (dw_weight_response_valid_q) begin
                dw_weight_cache_q[
                    dw_weight_response_index_q*128 +: 128
                ] <= dw_mem_rd_data;
                if (dw_weight_response_index_q == 3'd4)
                    dw_weight_cache_valid_q <= 1'b1;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dw_stride2_request_beat_q <= 5'd0;
            dw_stride2_response_valid_q <= 1'b0;
        end else if (backend_clear || finish_dw_group ||
                ((phase_q == PH_DW) && !dw_group_active_q
                 && token_valid)) begin
            dw_stride2_request_beat_q <= 5'd0;
            dw_stride2_response_valid_q <= 1'b0;
        end else if (cfg_stride2_q && dw_group_active_q) begin
            if (stride2_response_slot_ready)
                dw_stride2_response_valid_q <= b_stride2_read_request;
            if (b_stride2_read_request)
                dw_stride2_request_beat_q <=
                    dw_stride2_request_beat_q + 1'b1;
        end else begin
            dw_stride2_response_valid_q <= 1'b0;
        end
    end

    // One B-bank word carries two adjacent SWG pixels on the stride-1 path.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_swg_word_valid_q <= 1'b0;
        end else if (backend_clear) begin
            b_swg_word_valid_q <= 1'b0;
        end else if ((phase_q == PH_DW) && !dw_group_active_q
                && token_valid) begin
            b_swg_word_valid_q <= 1'b0;
        end else if (finish_dw_group) begin
            b_swg_word_valid_q <= 1'b0;
        end else if (!cfg_stride2_q && b_swg_read_request) begin
            b_swg_word_valid_q <= 1'b1;
        end else if (!cfg_stride2_q && swg_in_fire
                && ((dw_feed_pixel_q == 7'd80) || dw_feed_pixel_q[0])) begin
            b_swg_word_valid_q <= 1'b0;
        end
    end

    // The current pair occupies two windows. While it is consumed, only the
    // first window of the next pair is buffered; its second window is held by
    // the SWG and promoted on the current pair's retirement cycle.
    always @(posedge clk) begin
        if (swg_out_fire && swg_selected) begin
            if (!dw_window_half_q) begin
                if (dw_pair_ready_q)
                    dw_lookahead_window0_q <= swg_out_window;
                else
                    dw_window0_q <= swg_out_window;
            end else begin
                if (dw_lookahead_window0_valid_q)
                    dw_window0_q <= dw_lookahead_window0_q;
                dw_window1_q <= swg_out_window;
            end
        end
    end

    npu_shared_mac_pq8_backend_fixed_v2 #(
        .PER_CHANNEL_MULTIPLIER(PER_CHANNEL_MULTIPLIER),
        .MAIN_MULTIPLIER_SHIFT(MAIN_MULTIPLIER_SHIFT),
        .META_DEPTH(BACKEND_META_DEPTH),
        .META_PTR_WIDTH(BACKEND_META_PTR_WIDTH),
        .DEST_ADDR_WIDTH(12), .PREZEROED_OPERANDS(1),
        .MAC_FIXED_THROUGHPUT(1), .REGISTER_MAC_RESULT_READY(0),
        .UNCONDITIONAL_UOP_DATA(0), .UNCONDITIONAL_UOP_BIAS(1),
        .PQ_CAPTURE_ON_ACCEPT(1),
        .MAX_OUTSTANDING(BACKEND_WORD_CREDITS),
        .PQ_FIFO_DEPTH(BACKEND_PQ_FIFO_DEPTH),
        .PQ_FIFO_AW(BACKEND_PQ_FIFO_AW)
    ) u_backend (
        .clk(clk), .rst_n(C_PARTITION_RESET ? i_backend_rst_n : rst_n), .clear(backend_clear),
        .uop_valid(backend_uop_valid), .uop_ready(backend_uop_ready),
        .uop_first(backend_uop_first), .uop_last(backend_uop_last),
        .uop_lane_valid(backend_lane_valid),
        .uop_term_valid(backend_term_valid),
        .uop_qd(backend_qd), .uop_qw(backend_qw),
        .txn_meta_valid(backend_uop_last),
        .txn_dest_kind(backend_dest_kind),
        .txn_dest_addr(backend_dest_addr),
        .txn_bias_f8(backend_bias),
        .txn_shortcut_c8p2(backend_shortcut),
        .cfg_weight_zero(backend_cfg_weight_zero),
        .cfg_relu6_en(backend_cfg_relu6_en),
        .cfg_relu6_limit(backend_cfg_relu6_limit),
        .cfg_multiplier_q14(backend_cfg_multiplier),
        .cfg_multiplier_q14_bus(backend_cfg_multiplier_bus),
        .cfg_multiplier_shift(backend_cfg_multiplier_shift),
        .cfg_output_zero(backend_cfg_output_zero),
        .cfg_shortcut_en(backend_cfg_shortcut_en),
        .cfg_shortcut_zero(backend_cfg_shortcut_zero),
        .cfg_shortcut_multiplier_q14(backend_cfg_shortcut_multiplier),
        .cfg_word_fill(backend_cfg_word_fill),
        .word_valid(backend_word_valid), .word_ready(write_ready),
        .word_dest_kind(backend_word_kind),
        .word_dest_addr(backend_word_addr),
        .word_data(backend_word_data),
        .idle(backend_idle), .status_error(backend_error),
        .debug_error_code(backend_debug_error_code),
        .accepted_uops(accepted_uops),
        .accepted_transactions(accepted_transactions),
        .emitted_words(emitted_words)
    );

    integer mask_pair;
    integer mask_y;
    integer mask_x0;
    integer mask_x1;
    reg mask_pixel0_valid;
    reg mask_pixel1_valid;
    always @(*) begin
        write_byte_valid = 16'hffff;
        mask_pair = 0;
        mask_y = 0;
        mask_x0 = 0;
        mask_x1 = 0;
        mask_pixel0_valid = 1'b0;
        mask_pixel1_valid = 1'b0;
        if (backend_word_kind == OP_PROJ) begin
            mask_pair = cfg_stride2_q
                ? backend_word_addr[2:0] : backend_word_addr[4:0];
            mask_y = cfg_stride2_q ? (mask_pair >> 1) : (mask_pair >> 2);
            mask_x0 = cfg_stride2_q
                ? ((mask_pair & 1) << 1) : ((mask_pair & 3) << 1);
            mask_x1 = mask_x0 + 1;
            if (cfg_stride2_q) begin
                mask_pixel0_valid = (mask_y < tile_valid_h_q)
                    && (mask_x0 < tile_valid_w_q);
                mask_pixel1_valid = (mask_y < tile_valid_h_q)
                    && (mask_x1 < tile_valid_w_q);
            end else begin
                mask_pixel0_valid = (mask_y >= tile_shift_y_q)
                    && (mask_y < tile_shift_y_q + tile_valid_h_q)
                    && (mask_x0 >= tile_shift_x_q)
                    && (mask_x0 < tile_shift_x_q + tile_valid_w_q);
                mask_pixel1_valid = (mask_y >= tile_shift_y_q)
                    && (mask_y < tile_shift_y_q + tile_valid_h_q)
                    && (mask_x1 >= tile_shift_x_q)
                    && (mask_x1 < tile_shift_x_q + tile_valid_w_q);
            end
            write_byte_valid[7:0] = mask_pixel0_valid ? 8'hff : 8'h00;
            write_byte_valid[15:8] = mask_pixel1_valid ? 8'hff : 8'h00;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_exp_weight_zero_q <= 0;
            cfg_exp_relu6_limit_q <= 0;
            cfg_exp_multiplier_q <= 0;
            cfg_exp_multiplier_shift_q <= MAIN_MULTIPLIER_SHIFT;
            cfg_exp_output_zero_q <= 0;
            cfg_dw_weight_zero_q <= 0;
            cfg_dw_relu6_limit_q <= 0;
            cfg_dw_multiplier_q <= 0;
            cfg_dw_multiplier_shift_q <= MAIN_MULTIPLIER_SHIFT;
            cfg_dw_output_zero_q <= 0;
            cfg_proj_weight_zero_q <= 0;
            cfg_proj_multiplier_q <= 0;
            cfg_proj_multiplier_shift_q <= MAIN_MULTIPLIER_SHIFT;
            cfg_proj_output_zero_q <= 0;
            cfg_shortcut_zero_q <= 0;
            cfg_shortcut_multiplier_q <= 0;
            cfg_geometry_q <= 0;
            cfg_written_q <= 0;
            cfg_committed_q <= 1'b0;
            cfg_commit_done <= 1'b0;
            cfg_error_q <= 1'b0;
        end else begin
            cfg_commit_done <= 1'b0;
            if (cfg_wr_en && cfg_ready) begin
                cfg_written_q[cfg_wr_addr] <= 1'b1;
                case (cfg_wr_addr)
                    0: cfg_exp_weight_zero_q <= cfg_wr_data[7:0];
                    1: cfg_exp_relu6_limit_q <= cfg_wr_data;
                    2: begin
                        cfg_exp_multiplier_q <= cfg_wr_data[15:0];
                        cfg_exp_multiplier_shift_q <=
                            (cfg_wr_data[31:24] == 0)
                                ? MAIN_MULTIPLIER_SHIFT : cfg_wr_data[27:24];
                        cfg_exp_output_zero_q <= cfg_wr_data[23:16];
                    end
                    3: cfg_dw_weight_zero_q <= cfg_wr_data[7:0];
                    4: cfg_dw_relu6_limit_q <= cfg_wr_data;
                    5: begin
                        cfg_dw_multiplier_q <= cfg_wr_data[15:0];
                        cfg_dw_multiplier_shift_q <=
                            (cfg_wr_data[31:24] == 0)
                                ? MAIN_MULTIPLIER_SHIFT : cfg_wr_data[27:24];
                        cfg_dw_output_zero_q <= cfg_wr_data[23:16];
                    end
                    6: cfg_proj_weight_zero_q <= cfg_wr_data[7:0];
                    7: begin
                        cfg_proj_multiplier_q <= cfg_wr_data[15:0];
                        cfg_proj_multiplier_shift_q <=
                            (cfg_wr_data[31:24] == 0)
                                ? MAIN_MULTIPLIER_SHIFT : cfg_wr_data[27:24];
                        cfg_proj_output_zero_q <= cfg_wr_data[23:16];
                    end
                    8: begin
                        cfg_shortcut_zero_q <= cfg_wr_data[7:0];
                        cfg_shortcut_multiplier_q <= cfg_wr_data[23:8];
                    end
                    9: cfg_geometry_q <= cfg_wr_data;
                    default: cfg_error_q <= 1'b1;
                endcase
                cfg_committed_q <= 1'b0;
            end else if (cfg_wr_en) begin
                cfg_error_q <= 1'b1;
            end

            if (cfg_commit && cfg_ready) begin
                cfg_commit_done <= 1'b1;
                if ((cfg_written_q == 10'h3ff)
                        && (input_groups_q != 0) && (input_groups_q <= 9)
                        && (expanded_groups_q != 0)
                        && (expanded_groups_q <= 54)
                        && (output_groups_q != 0) && (output_groups_q <= 9)
                        && (cfg_exp_only_q
                            ? ((proj_pw_base_q == 0)
                               && (output_groups_q == expanded_groups_q)
                               && (expanded_groups_q*8 <= QBF_WORDS))
                            : ((proj_pw_base_q
                                == computed_proj_pw_base[9:0])
                               && (computed_proj_pw_base
                                   + computed_proj_pw_words <= PW_WORDS)
                               && (expanded_groups_q*5 <= DW_WORDS)
                               && (expanded_groups_q*16
                                   + output_groups_q*8 <= QBF_WORDS)))
                        && !(cfg_exp_only_q && cfg_stride2_q)
                        && !(cfg_shortcut_en_q
                            && (cfg_stride2_q || cfg_exp_only_q))) begin
                    cfg_committed_q <= 1'b1;
                end else begin
                    cfg_error_q <= 1'b1;
                end
            end else if (cfg_commit) begin
                cfg_error_q <= 1'b1;
            end

        end
    end

    // Residual jobs already have the complete source tile in A memory. Repack
    // After EXP drains, repack its 9-wide C8xP2 rows into the 8-wide
    // shortcut area (physical address bit 9 aliases A). Never repack before
    // EXP: doing so overwrites unread expansion inputs. This avoids asking
    // the global feature bank for the same bytes again.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            repack_state_q <= RP_IDLE;
            repack_group_q <= 6'd0;
            repack_pair_q <= 6'd0;
            repack_word0_q <= 128'd0;
            repack_word_q <= 128'd0;
        end else if (start_fire) begin
            repack_group_q <= 6'd0;
            repack_pair_q <= 6'd0;
            repack_word0_q <= 128'd0;
            repack_word_q <= 128'd0;
            repack_state_q <= RP_IDLE;
        end else if ((phase_q == PH_WAIT_EXP) && backend_pipeline_idle
                && !cfg_exp_only_q && shortcut_repack_q) begin
            repack_state_q <= RP_REQ0;
        end else if (phase_q == PH_SC_REPACK) begin
            case (repack_state_q)
                RP_REQ0: repack_state_q <= RP_WAIT0;

                RP_WAIT0: begin
                    repack_word0_q <= a_mem_rd_data;
                    if (repack_y[0]) begin
                        repack_state_q <= RP_REQ1;
                    end else begin
                        repack_word_q <= {
                            repack_spatial1_valid
                                ? a_mem_rd_data[127:64] : repack_fill64,
                            repack_spatial0_valid
                                ? a_mem_rd_data[63:0] : repack_fill64
                        };
                        repack_state_q <= RP_WRITE;
                    end
                end

                RP_REQ1: repack_state_q <= RP_WAIT1;

                RP_WAIT1: begin
                    repack_word_q <= {
                        repack_spatial1_valid
                            ? a_mem_rd_data[63:0] : repack_fill64,
                        repack_spatial0_valid
                            ? repack_word0_q[127:64] : repack_fill64
                    };
                    repack_state_q <= RP_WRITE;
                end

                RP_WRITE: begin
                    if (repack_last_word) begin
                        repack_state_q <= RP_IDLE;
                    end else if (repack_pair_q == 6'd31) begin
                        repack_pair_q <= 6'd0;
                        repack_group_q <= repack_group_q + 1'b1;
                        repack_state_q <= RP_REQ0;
                    end else begin
                        repack_pair_q <= repack_pair_q + 1'b1;
                        repack_state_q <= RP_REQ0;
                    end
                end

                default: repack_state_q <= RP_IDLE;
            endcase
        end else begin
            repack_state_q <= RP_IDLE;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_q <= PH_IDLE;
            tile_active_q <= 1'b0;
            tile_storage_slot_q <= 1'b0;
            shortcut_repack_q <= 1'b0;
            tile_shift_y_q <= 0;
            tile_shift_x_q <= 0;
            tile_valid_h_q <= 0;
            tile_valid_w_q <= 0;
            tile_far_pad_y_q <= 0;
            tile_far_pad_x_q <= 0;
            exp_group_q <= 0;
            exp_pair_q <= 0;
            exp_chunk_q <= 0;
            exp_pw_group_base_q <= 0;
            exp_pair_y_q <= 0;
            exp_pair_x_q <= 0;
            proj_group_q <= 0;
            proj_pair_q <= 0;
            proj_chunk_q <= 0;
            proj_pw_group_base_q <= 0;
            dw_group_active_q <= 1'b0;
            dw_next_group_q <= 0;
            dw_current_group_q <= 0;
            dw_feed_pixel_q <= 0;
            dw_pair_q <= 0;
            dw_swg_pair_q <= 0;
            dw_chunk_q <= 0;
            dw_window_half_q <= 0;
            dw_pair_ready_q <= 0;
            dw_lookahead_window0_valid_q <= 0;
            dw_tokens_done_q <= 0;
            dw_swg_done_q <= 0;
            protocol_error_q <= 1'b0;
            debug_error_code <= 10'd0;
            tile_done <= 1'b0;
            expansion_words <= 0;
            depthwise_words <= 0;
            projection_words <= 0;
        end else begin
            tile_done <= 1'b0;

            if (start_fire)
                debug_error_code <= 10'd0;
            else if (debug_error_code == 10'd0) begin
                if (protocol_error_q)
                    debug_error_code <= 10'd611;
                else if (cfg_error_q)
                    debug_error_code <= 10'd612;
                else if (backend_debug_error_code != 10'd0)
                    debug_error_code <= backend_debug_error_code;
                else if (backend_error)
                    debug_error_code <= 10'd618;
            end

            if (start_fire) begin
                phase_q <= PH_EXP;
                shortcut_repack_q <= shortcut_repack;
                tile_active_q <= 1'b1;
                tile_storage_slot_q <= tile_storage_slot;
                tile_shift_y_q <= tile_shift_y;
                tile_shift_x_q <= tile_shift_x;
                tile_valid_h_q <= tile_valid_h;
                tile_valid_w_q <= tile_valid_w;
                tile_far_pad_y_q <= tile_far_pad_y;
                tile_far_pad_x_q <= tile_far_pad_x;
                exp_group_q <= 0;
                exp_pair_q <= 0;
                exp_chunk_q <= 0;
                exp_pw_group_base_q <= 0;
                exp_pair_y_q <= 0;
                exp_pair_x_q <= 0;
                proj_group_q <= 0;
                proj_pair_q <= 0;
                proj_chunk_q <= 0;
                proj_pw_group_base_q <= 0;
                dw_group_active_q <= 1'b0;
                dw_next_group_q <= 0;
                dw_current_group_q <= 0;
                dw_feed_pixel_q <= 0;
                dw_pair_q <= 0;
                dw_swg_pair_q <= 0;
                dw_chunk_q <= 0;
                dw_window_half_q <= 0;
                dw_pair_ready_q <= 0;
                dw_lookahead_window0_valid_q <= 0;
                dw_tokens_done_q <= 0;
                dw_swg_done_q <= 0;
                protocol_error_q <= (tile_valid_h == 0)
                    || (tile_valid_w == 0)
                    || (cfg_stride2_q
                        ? ((tile_valid_h > 4) || (tile_valid_w > 4))
                        : ((tile_valid_h > 8) || (tile_valid_w > 8)
                           || (tile_shift_y + tile_valid_h > 8)
                           || (tile_shift_x + tile_valid_w > 8)));
                expansion_words <= 0;
                depthwise_words <= 0;
                projection_words <= 0;
            end

            if (repack_write && a_load_en)
                protocol_error_q <= 1'b1;
            if (tile_active_q && a_load_en &&
                    (a_load_storage_slot == tile_storage_slot_q))
                protocol_error_q <= 1'b1;
            if (tile_active_q && d_debug_en &&
                    (d_debug_storage_slot == tile_storage_slot_q))
                protocol_error_q <= 1'b1;

            if ((phase_q == PH_SC_REPACK) && repack_write &&
                repack_last_word)
                phase_q <= PH_DW;

            if (backend_word_fire) begin
                case (backend_word_kind)
                    OP_EXP: begin
                        if (backend_word_addr < B_WORDS) begin
                            expansion_words <= expansion_words + 1'b1;
                        end else begin
                            protocol_error_q <= 1'b1;
                        end
                    end
                    OP_DW: begin
                        if (backend_word_addr < C_WORDS) begin
                            depthwise_words <= depthwise_words + 1'b1;
                        end else begin
                            protocol_error_q <= 1'b1;
                        end
                    end
                    OP_PROJ: begin
                        if ((backend_word_addr < expected_projection_words)
                                && (backend_word_addr < D_WORDS)) begin
                            projection_words <= projection_words + 1'b1;
                        end else begin
                            protocol_error_q <= 1'b1;
                        end
                    end
                    default: protocol_error_q <= 1'b1;
                endcase
            end

            if ((phase_q == PH_DW) && !dw_group_active_q
                    && token_valid) begin
                dw_group_active_q <= 1'b1;
                dw_current_group_q <= dw_next_group_q;
                dw_feed_pixel_q <= 0;
                dw_pair_q <= 0;
                dw_swg_pair_q <= 0;
                dw_chunk_q <= 0;
                dw_window_half_q <= 0;
                dw_pair_ready_q <= 0;
                dw_lookahead_window0_valid_q <= 0;
                dw_tokens_done_q <= 0;
                dw_swg_done_q <= 0;
                if ((token_op != OP_DW)
                        || (token_group != dw_next_group_q)
                        || (token_pair != 0) || (token_chunk != 0))
                    protocol_error_q <= 1'b1;
            end

            if (swg_in_fire)
                dw_feed_pixel_q <= dw_feed_pixel_q + 1'b1;

            if (swg_out_fire) begin
                if (swg_selected) begin
                    if (cfg_stride2_q) begin
                        if ((swg_out_x != ((({10'd0, dw_pair_q} * 2
                                             + dw_window_half_q) % 4) * 2 + 1))
                                || (swg_out_y != ((({10'd0, dw_pair_q} * 2
                                                   + dw_window_half_q) / 4)
                                                  * 2 + 1)))
                            protocol_error_q <= 1'b1;
                    end else begin
                        if ((swg_out_x != ({10'd0, dw_swg_pair_q} * 2
                                            + dw_window_half_q) % 8)
                                || (swg_out_y != ({10'd0, dw_swg_pair_q} * 2
                                                  + dw_window_half_q) / 8))
                            protocol_error_q <= 1'b1;
                    end
                    if (!dw_window_half_q) begin
                        dw_window_half_q <= 1'b1;
                        dw_lookahead_window0_valid_q <= dw_pair_ready_q;
                    end else begin
                        dw_window_half_q <= 1'b0;
                        dw_pair_ready_q <= 1'b1;
                        dw_lookahead_window0_valid_q <= 1'b0;
                        if (dw_swg_pair_q != output_pair_last)
                            dw_swg_pair_q <= dw_swg_pair_q + 1'b1;
                    end
                end
                if ((swg_out_x == 8) && (swg_out_y == 8))
                    dw_swg_done_q <= 1'b1;
            end

            if (token_fire) begin
                case (phase_q)
                    PH_EXP: begin
                        if ((token_op != OP_EXP)
                                || (token_group != exp_group_q)
                                || (token_pair != exp_pair_q)
                                || (token_chunk != exp_chunk_q))
                            protocol_error_q <= 1'b1;
                        if (exp_chunk_q == input_group_last) begin
                            exp_chunk_q <= 0;
                            if (exp_pair_q == expansion_pair_last) begin
                                exp_pair_q <= 0;
                                exp_pair_y_q <= 0;
                                exp_pair_x_q <= 0;
                                if (exp_group_q == expanded_group_last) begin
                                    exp_group_q <= 0;
                                    exp_pw_group_base_q <= 0;
                                    phase_q <= PH_WAIT_EXP;
                                end else begin
                                    exp_group_q <= exp_group_q + 1'b1;
                                    exp_pw_group_base_q <= exp_pw_group_base_q
                                        + {6'd0, input_groups_q};
                                end
                            end else begin
                                exp_pair_q <= exp_pair_q + 1'b1;
                                if (exp_pair_x_q == 8) begin
                                    exp_pair_y_q <= exp_pair_y_q + 1'b1;
                                    exp_pair_x_q <= 1;
                                end else if (exp_pair_x_q == 7) begin
                                    exp_pair_y_q <= exp_pair_y_q + 1'b1;
                                    exp_pair_x_q <= 0;
                                end else begin
                                    exp_pair_x_q <= exp_pair_x_q + 2;
                                end
                            end
                        end else begin
                            exp_chunk_q <= exp_chunk_q + 1'b1;
                        end
                    end

                    PH_DW: begin
                        if ((token_op != OP_DW)
                                || (token_group != dw_current_group_q)
                                || (token_pair != (cfg_stride2_q
                                    ? dw_pair_q
                                    : dw_stride1_expected_pair))
                                || (token_chunk != {5'd0,
                                    (cfg_stride2_q ? dw_chunk_q
                                        : dw_stride1_expected_chunk)}))
                            protocol_error_q <= 1'b1;
                        if (!dw_chunk_q) begin
                            dw_chunk_q <= 1'b1;
                        end else if (cfg_stride2_q) begin
                            dw_chunk_q <= 1'b0;
                            if (dw_pair_q != output_pair_last)
                                dw_pair_q <= dw_pair_q + 1'b1;
                        end
                    end

                    PH_PROJ: begin
                        if ((token_op != OP_PROJ)
                                || (token_group != proj_group_q)
                                || (token_pair != proj_pair_q)
                                || (token_chunk != proj_chunk_q))
                            protocol_error_q <= 1'b1;
                        if (proj_chunk_q == expanded_group_last) begin
                            proj_chunk_q <= 0;
                            if (proj_pair_q == output_pair_last) begin
                                proj_pair_q <= 0;
                                if (proj_group_q == output_group_last) begin
                                    proj_group_q <= 0;
                                    proj_pw_group_base_q <= 0;
                                    phase_q <= PH_WAIT_PROJ;
                                end else begin
                                    proj_group_q <= proj_group_q + 1'b1;
                                    proj_pw_group_base_q <= proj_pw_group_base_q
                                        + {6'd0, expanded_groups_q};
                                end
                            end else begin
                                proj_pair_q <= proj_pair_q + 1'b1;
                            end
                        end else begin
                            proj_chunk_q <= proj_chunk_q + 1'b1;
                        end
                    end
                    default: protocol_error_q <= 1'b1;
                endcase
            end

            if (last_dw_backend_fire)
                dw_tokens_done_q <= 1'b1;

            // Retire at backend consumption. A completed live look-ahead pair
            // remains ready and may feed prefetch on this same cycle.
            if (!cfg_stride2_q && dw_second_backend_fire) begin
                dw_chunk_q <= token_fire ? 1'b1 : 1'b0;
                if (!dw_pair_promote_available)
                    dw_pair_ready_q <= 1'b0;
                if (issue_pair != output_pair_last)
                    dw_pair_q <= issue_pair + 1'b1;
            end

            if (cfg_stride2_q && backend_uop_fire && (issue_op == OP_DW)
                    && (!stride2_pair_valid
                        || (issue_pair != {3'd0, stride2_pair_index})))
                protocol_error_q <= 1'b1;

            if (finish_dw_group) begin
                dw_group_active_q <= 1'b0;
                dw_pair_ready_q <= 1'b0;
                dw_lookahead_window0_valid_q <= 1'b0;
                dw_tokens_done_q <= 1'b0;
                dw_swg_done_q <= 1'b0;
                if (dw_current_group_q == expanded_group_last) begin
                    phase_q <= PH_WAIT_DW;
                    dw_next_group_q <= 0;
                end else begin
                    dw_next_group_q <= dw_current_group_q + 1'b1;
                end
            end

            if ((phase_q == PH_WAIT_EXP) && backend_pipeline_idle) begin
                if (cfg_exp_only_q) begin
                    if ((accepted_uops != expected_uops)
                            || (accepted_transactions != expected_transactions)
                            || (emitted_words != expected_transactions)
                            || (expansion_words != expected_expansion_words)
                            || (depthwise_words != 0)
                            || (projection_words != 0))
                        protocol_error_q <= 1'b1;
                    phase_q <= PH_IDLE;
                    tile_active_q <= 1'b0;
                    tile_done <= 1'b1;
                end else begin
                    phase_q <= shortcut_repack_q ? PH_SC_REPACK : PH_DW;
                end
            end
            if ((phase_q == PH_WAIT_DW) && backend_pipeline_idle)
                phase_q <= PH_PROJ;
            if ((phase_q == PH_WAIT_PROJ) && backend_pipeline_idle) begin
                if ((accepted_uops != expected_uops)
                        || (accepted_transactions != expected_transactions)
                        || (emitted_words != expected_transactions)
                        || (expansion_words != expected_expansion_words)
                        || (depthwise_words != expected_depthwise_words)
                        || (projection_words != expected_projection_words))
                    protocol_error_q <= 1'b1;
                phase_q <= PH_IDLE;
                tile_active_q <= 1'b0;
                tile_done <= 1'b1;
            end

        end
    end

    // Full-network writeback reads only completed tiles. Data comes directly
    // from the registered read side of the explicit M4K banks.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_debug_valid <= 1'b0;
            d_debug_valid <= 1'b0;
        end else begin
            b_debug_valid <= b_debug_en && !tile_active_q;
            d_debug_valid <= d_debug_en;
        end
    end

endmodule

`default_nettype wire
