//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: HDMI 与 F14 推理之间的整帧桥接：将输入 RGB565 按确定性相位采样为 224×168，经双时钟双采集槽按 FIFO 顺序交给模型。
// Module Name: host_f14_frame_engine
// File Name: lane/rtl/host_f14_frame_engine.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// HDMI 与 F14 推理之间的整帧桥接：将输入 RGB565 按确定性相位采样为 224×168，经双时钟双采集槽按 FIFO 顺序交给模型。
// 协调骨干/检测头结果、车道解码、线型分类和几何/时域后处理，输出带帧号的四车道坐标、有效性、缺口和类型表；首层读完即可释放输入采集槽。
//
// Dependencies: f14_pipeline_core、host_f14_capture_ram、host_f14_result_decoder、线型/后处理相关模块；f14_bringup_contract.vh
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 像素域与系统域须共同复位；完成 toggle 后 bank ID 与像素保持到 ack 返回，输入槽和结果槽通过握手管理所有权。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none
`include "f14_bringup_contract.vh"

// HDMI-to-F14 frame bridge.  The normalizer supplies 640x320 RGB565.  A
// deterministic phase accumulator selects exactly 224x168 samples, which
// are stored in two explicit dual-clock capture RAM banks.
// Company: HRB_CSLAB_501; Engineer: VinnLee
// Target: PG2L200H-6FBB484 / PDS 2022.2-SP6.4.
// 完整帧按FIFO顺序交接；首层读完即释放本帧输入，不等检测头/分类。
// 两个时钟域必须共同复位。完成toggle之后，bank ID和像素保持到ack返回。
// The capture RAM is explicitly packed into PG2L RAM IP in implementation;
// ModelSim selects its behavioral branch with F14_SIMULATION.
module host_f14_frame_engine #(
    parameter integer C_ENABLE_LINETYPE = 1,
    parameter integer C_PARTITION_RESET = 0,
    parameter integer C_LINETYPE_CUTOFF_Y = 320,
    parameter integer C_INPUT_WIDTH = 640,
    parameter integer C_INPUT_HEIGHT = 320
) (
    input wire i_pixel_clk, input wire i_pixel_rst_n,
    input wire i_pixel_valid, input wire [15:0] i_pixel_data,
    input wire [8:0] i_pixel_row, input wire [9:0] i_pixel_col,
    input wire i_pixel_frame_start,
    input wire [15:0] i_pixel_frame_id,
    input wire i_pixel_frame_abort,
    input wire i_sys_clk, input wire i_sys_rst_n,
    input wire [7:0] i_reset_partition_n,
    input wire i_result_ready,
    input wire i_scene_valid,input wire [15:0] i_scene_frame,input wire i_scene_active,
    output reg o_result_valid, output reg [15:0] o_result_frame_id,
    output reg [839:0] o_lane_x_table,
    output reg [83:0] o_lane_valid_table,
    output reg [83:0] o_lane_edge_table,
    output reg [7:0] o_lane_type_table,
    output wire o_busy,
    output wire [7:0] o_diag_sys,
    output wire o_diag_captured
);
    localparam integer IMAGE_PIXELS = `F14_INPUT_PIXELS;

    reg [7:0] sample_x_q, sample_y_q;
    reg [15:0] sample_addr_q;
    reg [9:0] x_phase_q;
    reg [8:0] y_phase_q;
    reg capture_q, row_take_q;
    reg [1:0] frame_toggle_q;
    reg capture_bank_q, read_bank_q;
    reg [15:0] bank_frame_id_q [0:1];
    wire busy_sys_q;
    reg [15:0] captured_frame_id_q;
    reg [1:0] frame_ack_q;
    (* async_reg = "true" *) reg [1:0] ack_pix_meta_q, ack_pix_sync_q;
    wire capture_available = (frame_toggle_q[capture_bank_q] == ack_pix_sync_q[capture_bank_q]);
    wire capture_now = !i_pixel_frame_abort &&
                       (i_pixel_frame_start ? capture_available : capture_q);
    wire [8:0] y_phase_now = i_pixel_frame_start ? 9'd0 : y_phase_q;
    wire row_take_now = (i_pixel_col == 0) ?
        ((y_phase_now + 9'd168) >= C_INPUT_HEIGHT) : row_take_q;
    wire [9:0] x_phase_now = (i_pixel_col == 0) ? 10'd0 : x_phase_q;
    wire x_sample_now = ((x_phase_now + 10'd224) >= C_INPUT_WIDTH);
    wire direct_input = C_INPUT_WIDTH==224 && C_INPUT_HEIGHT==168;
    wire ram_wr_en = capture_now && !i_pixel_frame_abort && i_pixel_valid &&
        (direct_input || (row_take_now && x_sample_now));

    // Pixel-domain sampler and RAM writer.  sample_addr_q is incremented
    // only when a selected pixel is written, so it can never exceed 37631.
    always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
        if (!i_pixel_rst_n) begin
            sample_x_q <= 0; sample_y_q <= 0; sample_addr_q <= 0;
            x_phase_q <= 0; y_phase_q <= 0;
            capture_q <= 1'b0; row_take_q <= 1'b0; frame_toggle_q <= 2'b00;
            capture_bank_q <= 0;
            ack_pix_meta_q <= 0; ack_pix_sync_q <= 0;
            bank_frame_id_q[0] <= 0; bank_frame_id_q[1] <= 0;
        end else begin
            ack_pix_meta_q <= frame_ack_q;
            ack_pix_sync_q <= ack_pix_meta_q;
            if (i_pixel_frame_start) capture_q <= 1'b0;
            if (i_pixel_frame_start && capture_available && !i_pixel_frame_abort) begin
                sample_x_q <= 0; sample_y_q <= 0; sample_addr_q <= 0;
                x_phase_q <= 0; y_phase_q <= 0; row_take_q <= 1'b0;
                capture_q <= 1'b1;
                bank_frame_id_q[capture_bank_q] <= i_pixel_frame_id;
            end
            if (capture_now && i_pixel_valid && C_INPUT_WIDTH==224 && C_INPUT_HEIGHT==168) begin
                sample_addr_q <= i_pixel_frame_start ? 16'd1 : sample_addr_q+1'b1;
                sample_x_q <= i_pixel_col==223 ? 0 : i_pixel_col[7:0]+1'b1;
                sample_y_q <= i_pixel_row[7:0];
                if (i_pixel_row==167 && i_pixel_col==223) begin
                    sample_addr_q<=0;capture_q<=0;frame_toggle_q[capture_bank_q]<=~frame_toggle_q[capture_bank_q];capture_bank_q<=~capture_bank_q;
                end
            end else if (capture_now && i_pixel_valid) begin
                if (i_pixel_col == 0) begin
                    if ((y_phase_now + 9'd168) >= C_INPUT_HEIGHT) begin
                        y_phase_q <= y_phase_now + 9'd168 - C_INPUT_HEIGHT;
                        row_take_q <= 1'b1;
                        sample_x_q <= 0;
                        x_phase_q <= 0;
                    end else begin
                        y_phase_q <= y_phase_now + 9'd168;
                        row_take_q <= 1'b0;
                    end
                end
                if (row_take_now) begin
                    if (x_sample_now) begin
                        x_phase_q <= x_phase_now + 10'd224 - C_INPUT_WIDTH;
                        if (sample_x_q == 8'd223) begin
                            sample_x_q <= 0;
                            if (sample_y_q == 8'd167) begin
                                sample_y_q <= 0;
                                sample_addr_q <= 0;
                                capture_q <= 1'b0;
                                frame_toggle_q[capture_bank_q] <= ~frame_toggle_q[capture_bank_q];
                                capture_bank_q <= ~capture_bank_q;
                            end else begin
                                sample_y_q <= sample_y_q + 1'b1;
                                sample_addr_q <= sample_addr_q + 1'b1;
                            end
                        end else begin
                            sample_x_q <= sample_x_q + 1'b1;
                            sample_addr_q <= i_pixel_frame_start ? 16'd1 : sample_addr_q + 1'b1;
                        end
                    end else begin
                        x_phase_q <= x_phase_now + 10'd224;
                    end
                end
            end
            if (i_pixel_frame_abort) capture_q <= 1'b0;
        end
    end

    (* async_reg = "true" *) reg [1:0] frame_toggle_meta_q, frame_toggle_sync_q;
    reg frame_toggle_seen_q;
    reg [15:0] image_read_addr_q;
    reg [15:0] image_send_count_q;
    reg image_rd_valid_q;
    reg [2:0] core_state_q;
    localparam [2:0] ST_IDLE=0, ST_ACCEPT=1, ST_SEND=2,
                     ST_WAIT=3, ST_DECODE=4, ST_PUBLISH=5, ST_CLASSIFY=6;
    reg allocation_slot_q;
    reg [1:0] slot_owned_q;
    reg [2:0] result_state_q;
    localparam [2:0] OUT_IDLE=0, OUT_DECODE=1, OUT_CLASSIFY=2, OUT_PUBLISH=3, OUT_FILTER=4;
    wire core_frame_valid = (core_state_q == ST_ACCEPT) && !slot_owned_q[allocation_slot_q];
    wire core_result_valid;
    wire decoder_start = (result_state_q == OUT_IDLE) && core_result_valid;
    wire input_release;
    wire [31:0] s8_frame_id;
    assign busy_sys_q = (core_state_q != ST_IDLE) || (|slot_owned_q) || result_state_q!=OUT_IDLE;
    wire core_pixel_valid = (core_state_q == ST_SEND) && image_rd_valid_q;
    wire core_pixel_ready;
    wire core_status_error;
    wire core_pixel_fire = core_pixel_valid && core_pixel_ready;
    // Issue the first read immediately after entering ST_SEND, then issue a
    // new address on every consumed pixel.  The capture RAM has one-cycle
    // read latency, so image_rd_valid_q is the core's valid pipeline bit.
    wire image_rd_en = (core_state_q == ST_SEND) &&
                       (!image_rd_valid_q || core_pixel_fire) &&
                       (image_read_addr_q < IMAGE_PIXELS);
    wire [15:0] image_rd_data;
    wire image_rd_valid_unused;
    wire backbone_image_rd_en;
    wire [15:0] backbone_image_rd_addr;
    assign o_busy = busy_sys_q;

    wire [15:0] bank_rd_data [0:1];
    wire [1:0] bank_rd_valid;
    genvar image_bank;
    generate for (image_bank=0; image_bank<2; image_bank=image_bank+1) begin : g_input_bank
        host_f14_capture_ram #(.C_DEPTH(IMAGE_PIXELS)) u_capture_ram (
            .i_wr_clk(i_pixel_clk), .i_wr_en(ram_wr_en && capture_bank_q==image_bank),
            .i_wr_addr(i_pixel_frame_start ? 16'd0 : sample_addr_q), .i_wr_data(i_pixel_data),
            .i_rd_clk(i_sys_clk), .i_rd_en((image_rd_en || backbone_image_rd_en) && read_bank_q==image_bank),
            .i_rd_addr(backbone_image_rd_en ? backbone_image_rd_addr : image_read_addr_q),
            .o_rd_data(bank_rd_data[image_bank]), .o_rd_valid(bank_rd_valid[image_bank])
        );
    end endgenerate
    // 读bank从接收描述符到首层最后读响应之后始终不变；仅input_release允许切换。
    assign image_rd_data = read_bank_q ? bank_rd_data[1] : bank_rd_data[0];
    assign image_rd_valid_unused = read_bank_q ? bank_rd_valid[1] : bank_rd_valid[0];

    wire result_req;
    wire [12:0] result_addr;
    wire result_read_valid;
    wire [15:0] result_read_data;
    wire decoder_done;
    wire [839:0] decoded_x_table;
    wire [83:0] decoded_valid_table;
    wire [7:0] decoded_type_table;
    wire core_frame_accept;
    wire [31:0] core_frame_id;
    wire s8_wr_en;
    wire [13:0] s8_wr_addr;
    wire [7:0] s8_wr_data;
    wire linetype_done;
    wire [7:0] linetype_types;
    wire consensus_done;
    wire [7:0] consensus_types;
    wire post_done;
    wire [839:0] post_x_table;
    wire [83:0] post_valid_table;
    wire [83:0] post_edge_table;
    wire [7:0] post_type_table;
    // 几何与取样/分类并行；最后策略等待二者，不阻塞S8/骨干槽释放。
    wire post_start = decoder_done;
    wire spatial_valid,spatial_kept;
    wire [1:0] spatial_lane;
    wire [4:0] spatial_row,spatial_segment;
    wire [9:0] spatial_x;
    reg transaction_seen_q,transaction_contiguous_q,transaction_slot_q;
    reg [15:0] transaction_frame_q,previous_source_q,logical_frame_q;
    wire [15:0] transaction_delta=core_frame_id[15:0]-previous_source_q;
    always @(posedge i_sys_clk or negedge i_sys_rst_n)begin
        if(!i_sys_rst_n)begin transaction_seen_q<=0;transaction_contiguous_q<=0;transaction_slot_q<=0;transaction_frame_q<=0;previous_source_q<=0;logical_frame_q<=0;end
        else if(decoder_start)begin
            transaction_seen_q<=1;transaction_frame_q<=core_frame_id[15:0];transaction_slot_q<=core_frame_id[16];
            transaction_contiguous_q<=transaction_seen_q && transaction_delta==2;
            previous_source_q<=core_frame_id[15:0];
            logical_frame_q<=transaction_seen_q ? logical_frame_q+{1'b0,transaction_delta[15:1]} : {1'b0,core_frame_id[15:1]};
        end
    end
    wire result_release = C_ENABLE_LINETYPE ? linetype_done : decoder_done;
    // Stem直接读取当前capture bank；模型内部不再复制输入图像。
    f14_pipeline_core #(.C_EXTERNAL_IMAGE(1),.C_PARTITION_RESET(C_PARTITION_RESET)) u_f14 (
        .i_reset_partition_n(i_reset_partition_n),
        .i_clk(i_sys_clk), .i_rst_n(i_sys_rst_n),
        .i_frame_valid(core_frame_valid), .i_frame_id({15'd0,allocation_slot_q,captured_frame_id_q}),
        .o_frame_accept(core_frame_accept), .i_head_enable(1'b1),
        .i_pixel_valid(core_pixel_valid), .i_pixel_data(image_rd_data),
        .o_pixel_ready(core_pixel_ready), .o_result_valid(core_result_valid),
        .i_result_release(result_release), .o_result_frame_id(core_frame_id),
        .o_result_cycles(),
        .i_boot_rd_en(1'b0), .i_boot_rd_addr(17'd0),
        .o_boot_rd_ready(), .o_boot_rd_valid(), .o_boot_rd_data(), .o_status_error(core_status_error),
        .i_result_req(result_req), .i_result_addr(result_addr),
        .o_result_read_valid(result_read_valid), .o_result_data(result_read_data),
        .o_image_rd_en(backbone_image_rd_en), .o_image_rd_addr(backbone_image_rd_addr),
        .i_image_rd_data(image_rd_data),
        .o_s8_wr_en(s8_wr_en), .o_s8_wr_addr(s8_wr_addr), .o_s8_wr_data(s8_wr_data),
        .o_input_release(input_release), .o_s8_frame_id(s8_frame_id)
    );

    generate if (C_ENABLE_LINETYPE) begin : g_linetype
        wire feature_req, feature_valid, snapshot_complete;
        wire [13:0] feature_addr;
        wire [7:0] feature_data;
        wire [7:0] snapshot_data [0:1];
        wire [1:0] snapshot_valid, snapshot_done;
        genvar snapshot_slot;
        for (snapshot_slot=0;snapshot_slot<2;snapshot_slot=snapshot_slot+1) begin : g_slot
            host_f14_s8_snapshot u_snapshot (
                .i_clk(i_sys_clk), .i_rst_n(i_sys_rst_n),
                .i_frame_accept(core_frame_accept && allocation_slot_q==snapshot_slot),
                .i_wr_en(s8_wr_en && s8_frame_id[16]==snapshot_slot),
                .i_wr_addr(s8_wr_addr), .i_wr_data(s8_wr_data),
                .i_rd_en(feature_req && core_frame_id[16]==snapshot_slot), .i_rd_addr(feature_addr),
                .o_rd_data(snapshot_data[snapshot_slot]), .o_rd_valid(snapshot_valid[snapshot_slot]),
                .o_complete(snapshot_done[snapshot_slot])
            );
        end
        assign feature_data = core_frame_id[16] ? snapshot_data[1] : snapshot_data[0];
        assign feature_valid = core_frame_id[16] ? snapshot_valid[1] : snapshot_valid[0];
        assign snapshot_complete = core_frame_id[16] ? snapshot_done[1] : snapshot_done[0];
        wire head_error,quality_done;
        wire [1:0] head_supported,head_high,head_medium,quality_supported,quality_hard,quality_visible;
        wire scene_valid,scene_active;
        reg head_seen_q,quality_seen_q,finish_sent_q,scene_q,transaction_error_q;
        wire finish_start=head_seen_q && quality_seen_q && !finish_sent_q;
        wire [1:0] track_visible={|post_valid_table[62:42],|post_valid_table[41:21]};
        always @(posedge i_sys_clk or negedge i_sys_rst_n)begin
            if(!i_sys_rst_n)begin head_seen_q<=0;quality_seen_q<=0;finish_sent_q<=0;scene_q<=0;transaction_error_q<=0;end
            else begin
                // 分类读完会释放快照；质量支路使用开始解码时锁存的本帧状态。
                if(decoder_start)begin head_seen_q<=0;quality_seen_q<=0;finish_sent_q<=0;scene_q<=0;transaction_error_q<=core_status_error || !snapshot_complete;end
                if(linetype_done)begin head_seen_q<=1;scene_q<=scene_valid && scene_active;end
                if(quality_done)quality_seen_q<=1;
                if(finish_start)finish_sent_q<=1;
            end
        end
        host_f14_scene_slots u_scene(
            .i_clk(i_sys_clk),.i_rst_n(i_sys_rst_n),.i_scene_valid(i_scene_valid),.i_scene_frame(i_scene_frame),.i_scene_active(i_scene_active),
            .i_claim(core_frame_accept),.i_claim_slot(allocation_slot_q),.i_claim_frame(captured_frame_id_q),
            .i_release(linetype_done),.i_release_slot(transaction_slot_q),.i_query_slot(transaction_slot_q),.i_query_frame(transaction_frame_q),
            .o_valid(scene_valid),.o_active(scene_active));
        host_f14_linetype_tiny_head #(.C_CUTOFF_Y(C_LINETYPE_CUTOFF_Y)) u_head (
            .i_clk(i_sys_clk),.i_rst_n(i_sys_rst_n),.i_start(decoder_done),.i_error(core_status_error || !snapshot_complete),
            .i_x_table(decoded_x_table),.i_valid_table(decoded_valid_table),.i_geometry_types(decoded_type_table),
            .o_done(linetype_done),.o_types(linetype_types),.o_feature_req(feature_req),.o_feature_addr(feature_addr),
            .i_feature_valid(feature_valid),.i_feature_data(feature_data),.o_busy(),.o_error(head_error),
            .o_supported(head_supported),.o_high(head_high),.o_medium(head_medium),
            .o_sample_valid(),.o_slot(),.o_sample_addr(),.o_sample_data(),.o_logit_valid(),.o_logit_solid(),.o_logit_dashed(),.o_cycles());
        host_f14_linetype_quality u_quality(
            .i_clk(i_sys_clk),.i_rst_n(i_sys_rst_n),.i_spatial_valid(spatial_valid),.i_spatial_lane(spatial_lane),.i_spatial_row(spatial_row),
            .i_spatial_x(spatial_x),.i_spatial_kept(spatial_kept),.i_spatial_segment(spatial_segment),
            .i_start(post_done),.i_contiguous(transaction_contiguous_q),.i_error(transaction_error_q),
            .i_x_table(decoded_x_table),.i_valid_table(decoded_valid_table),.o_busy(),.o_done(quality_done),
            .o_supported(quality_supported),.o_hard(quality_hard),.o_visible(quality_visible));
        host_f14_linetype_finish u_finish(
            .i_clk(i_sys_clk),.i_rst_n(i_sys_rst_n),.i_start(finish_start),.i_contiguous(transaction_contiguous_q),.i_frame_id(logical_frame_q),
            .i_x_table(decoded_x_table),.i_valid_table(decoded_valid_table),.i_raw_types(linetype_types),.i_high(head_high),.i_medium(head_medium),
            .i_supported(quality_supported & head_supported & {2{!head_error}}),.i_hard(quality_hard | {2{head_error}}),
            .i_visible(quality_visible),.i_visible_track(track_visible),.i_scene(scene_q),
            .o_busy(),.o_done(consensus_done),.o_decision(),.o_display(consensus_types));
    end else begin : g_no_linetype
        assign consensus_done=1'b0;
        assign consensus_types=decoded_type_table;
        assign linetype_done=1'b0;
        assign linetype_types=decoded_type_table;
    end endgenerate
    host_f14_result_decoder u_decoder (
        .i_clk(i_sys_clk), .i_rst_n(i_sys_rst_n),
        .i_start(decoder_start),
        .i_error(core_status_error),
        .o_req(result_req), .o_addr(result_addr),
        .i_read_valid(result_read_valid), .i_read_data(result_read_data),
        .o_done(decoder_done),
        .o_lane_x_table(decoded_x_table),
        .o_lane_valid_table(decoded_valid_table),
        .o_lane_type_table(decoded_type_table)
    );

    // 分类只读原始坐标；几何支路并行完成空间/重捕获/时间平滑，最终发布前合并。
    host_lane_postprocess_track u_postprocess (
        .i_clk(i_sys_clk), .i_rst_n(i_sys_rst_n), .i_start(post_start),
        .i_x_table(decoded_x_table), .i_valid_table(decoded_valid_table),
        .i_type_table(decoded_type_table),.i_contiguous(transaction_contiguous_q),
        .o_spatial_valid(spatial_valid),.o_spatial_lane(spatial_lane),.o_spatial_row(spatial_row),
        .o_spatial_x(spatial_x),.o_spatial_kept(spatial_kept),.o_spatial_segment(spatial_segment),
        .o_done(post_done), .o_x_table(post_x_table),
        .o_valid_table(post_valid_table), .o_edge_table(post_edge_table), .o_type_table(post_type_table)
    );

    // 输入流与结果流独立；输入释放来自首层完成，不再等待分类/显示。
    always @(posedge i_sys_clk or negedge i_sys_rst_n) begin
        if (!i_sys_rst_n) begin
            frame_toggle_meta_q<=0; frame_toggle_sync_q<=0; frame_toggle_seen_q<=0; frame_ack_q<=0;
            read_bank_q<=0; captured_frame_id_q<=0;
            image_read_addr_q<=0; image_send_count_q<=0; image_rd_valid_q<=0;
            core_state_q<=ST_IDLE; allocation_slot_q<=0; slot_owned_q<=0;
            result_state_q<=OUT_IDLE;
            o_result_valid<=0; o_result_frame_id<=0;
            o_lane_x_table<=0; o_lane_valid_table<=0; o_lane_edge_table<=0; o_lane_type_table<=0;
        end else begin
            frame_toggle_meta_q<=frame_toggle_q;
            frame_toggle_sync_q<=frame_toggle_meta_q;
            if (input_release) begin
                frame_ack_q[read_bank_q]<=frame_toggle_seen_q;
                read_bank_q<=~read_bank_q;
            end
            if (core_state_q==ST_SEND) begin
                image_rd_valid_q<=(image_rd_valid_q && !core_pixel_fire) || image_rd_en;
                if(image_rd_en) image_read_addr_q<=image_read_addr_q+1'b1;
            end else image_rd_valid_q<=0;
            if(result_release) slot_owned_q[core_frame_id[16]]<=0;
            if(core_frame_accept) begin
                slot_owned_q[allocation_slot_q]<=1;
                allocation_slot_q<=~allocation_slot_q;
            end
            case(core_state_q)
                ST_IDLE: if(frame_toggle_sync_q[read_bank_q]!=frame_ack_q[read_bank_q]) begin
                    frame_toggle_seen_q<=frame_toggle_sync_q[read_bank_q];
                    captured_frame_id_q<=bank_frame_id_q[read_bank_q];
                    image_read_addr_q<=0;image_send_count_q<=0;core_state_q<=ST_ACCEPT;
                end
                ST_ACCEPT: if(core_frame_accept) core_state_q<=ST_SEND;
                ST_SEND: if(core_pixel_fire) begin
                    if(image_send_count_q==IMAGE_PIXELS-1) begin
                        core_state_q<=ST_WAIT;image_rd_valid_q<=0;
                    end else image_send_count_q<=image_send_count_q+1'b1;
                end
                // SEND只提交帧，后续stem仍读同一个bank；不得提前切到已就绪的下一帧。
                ST_WAIT: if(input_release) core_state_q<=ST_IDLE;
                default: core_state_q<=ST_IDLE;
            endcase
            o_result_valid<=0;
            case(result_state_q)
                OUT_IDLE: if(decoder_start) result_state_q<=OUT_DECODE;
                OUT_DECODE: if(decoder_done) begin
                    o_lane_x_table<=decoded_x_table; o_lane_valid_table<=decoded_valid_table;
                    o_lane_type_table<=decoded_type_table; o_result_frame_id<=core_frame_id[15:0];
                    result_state_q<=C_ENABLE_LINETYPE ? OUT_CLASSIFY : OUT_FILTER;
                end
                OUT_CLASSIFY: if(consensus_done) begin
                    o_lane_x_table<=post_x_table;o_lane_valid_table<=post_valid_table;o_lane_edge_table<=post_edge_table;
                    o_lane_type_table<=consensus_types;result_state_q<=OUT_PUBLISH;
                end
                OUT_FILTER: if(post_done) begin
                    o_lane_x_table<=post_x_table;
                    o_lane_valid_table<=post_valid_table;
                    o_lane_edge_table<=post_edge_table;
                    o_lane_type_table<=post_type_table;
                    result_state_q<=OUT_PUBLISH;
                end
                OUT_PUBLISH: if(i_result_ready) begin
                    o_result_valid<=1; result_state_q<=OUT_IDLE;
                end
                default: result_state_q<=OUT_IDLE;
            endcase
        end
    end
    // 只观察已有事件，不参与推理控制或数据通路。
    reg diag_captured_q;
    always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
        if (!i_pixel_rst_n) diag_captured_q <= 1'b0;
        else if (ram_wr_en && sample_addr_q == IMAGE_PIXELS-1) diag_captured_q <= 1'b1;
    end
    assign o_diag_captured = diag_captured_q;
    reg [6:0] diag_seen_q;
    reg [3:0] diag_result_count_q;
    always @(posedge i_sys_clk or negedge i_sys_rst_n) begin
        if (!i_sys_rst_n) begin
            diag_seen_q <= 0;
            diag_result_count_q <= 0;
        end else begin
            diag_seen_q <= diag_seen_q | {core_status_error,
                (decoder_done && |decoded_valid_table), o_result_valid,
                linetype_done, decoder_done, decoder_start, core_frame_accept};
            if (decoder_start) diag_result_count_q <= diag_result_count_q + 1'b1;
        end
    end
    assign o_diag_sys = {diag_result_count_q[3], diag_seen_q};

endmodule
`default_nettype wire
