//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 真实视频车道处理总成：模型输入交给双槽帧引擎执行 F14 与线型后处理，叠线前图像另行执行斑马线场景识别，再经曲线桥、元数据收集和 RGB565 叠线输出。
// Module Name: host_f14_video_lane_path
// File Name: host/rtl/video/host_f14_video_lane_path.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 真实视频车道处理总成：模型输入交给双槽帧引擎执行 F14 与线型后处理，叠线前图像另行执行斑马线场景识别，再经曲线桥、元数据收集和 RGB565 叠线输出。
// 视频像素与行/帧描述符先共同寄存，再配合曲线一拍读取和 overlay 一拍寄存保持对齐；视频透传复位独立于模型 PLL 状态。
// 模型采集/结果跨域状态由像素与模型域共同异步断言、像素域三级同步释放；可选择八路系统域分区复位，并为元数据单独附加复位条件。
//
// Dependencies: host_f14_crosswalk_path、host_f14_frame_engine、host_f14_curve_bridge、host_lane_meta_collect、host_lane_overlay；像素域及模型系统域。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// Company: HRB_CSLAB_501; Engineer: VinnLee
// PG2L200H-6FBB484 / PDS 2022.2-SP6.4
// 实际视频车道检测路径；顶层和链路测试使用同一个模块。
`timescale 1ns/1ps
module host_f14_video_lane_path #(
    parameter integer C_PARTITION_RESET = 0
) (
    input wire i_pixel_clk, input wire i_pixel_rst_n,
    input wire i_sys_clk, input wire i_sys_rst_n,
    input wire [7:0] i_reset_partition_n,
    input wire i_valid, input wire [15:0] i_pixel,
    input wire [8:0] i_row, input wire [9:0] i_col,
    input wire i_row_start,i_row_commit,i_frame_done,i_frame_abort,
    input wire [15:0] i_frame_id,
    output wire o_valid, output wire [15:0] o_pixel,
    output wire [8:0] o_row, output wire [9:0] o_col,
    output wire o_row_start,o_row_commit,o_frame_done,
    output wire [15:0] o_frame_id,
    output wire o_busy,
    input wire i_cnn_valid, input wire [15:0] i_cnn_pixel,
    input wire [8:0] i_cnn_row, input wire [9:0] i_cnn_col,
    input wire [15:0] i_cnn_frame_id, input wire i_cnn_abort,
    output wire [7:0] o_diag_sys, output wire [1:0] o_diag_pixel,
    input wire i_meta_rst_n, input wire i_meta_ready,
    output wire o_meta_valid, output wire [31:0] o_meta_seq,
    output wire [31:0] o_meta_payload
);
    // 0:帧控制/后处理，1:骨干搬运，2:tile，3:骨干MAC/PQ，
    // 4:检测头控制，5:feeder，6:检测头MAC/PQ，7:曲线。
    wire [7:0] partition_rst_n = C_PARTITION_RESET ? i_reset_partition_n : {8{i_sys_rst_n}};
    wire rv,rr;
    wire [15:0] rid;
    wire [839:0] xs;
    wire [83:0] vs;
    wire [83:0] edges;
    wire [7:0] ts;
    wire [3:0] lane_valid;
    wire [39:0] lane_x;
    wire [7:0] lane_type;
    wire [15:0] curve_frame_id;
    wire curve_available;
    // 模型失锁只清除采集/结果跨域状态，像素与行描述符仍沿视频时钟透传。
    // 两个模型域共同异步断言；像素侧独立同步释放，避免toggle所有权失配。
    wire model_async_rst_n = i_pixel_rst_n && i_sys_rst_n;
    (* async_reg = "true" *) reg [2:0] model_pixel_reset_q;
    always @(posedge i_pixel_clk or negedge model_async_rst_n) begin
        if (!model_async_rst_n) model_pixel_reset_q <= 3'b000;
        else model_pixel_reset_q <= {model_pixel_reset_q[1:0],1'b1};
    end
    wire model_pixel_rst_n = model_pixel_reset_q[2];
    reg diag_curve_seen_q;
    always @(posedge i_pixel_clk or negedge model_pixel_rst_n) begin
        if (!model_pixel_rst_n) diag_curve_seen_q <= 1'b0;
        else if (|lane_valid) diag_curve_seen_q <= 1'b1;
    end
    assign o_diag_pixel[1] = diag_curve_seen_q;
    reg [54:0] pixels_q;
    always @(posedge i_pixel_clk or negedge i_pixel_rst_n) begin
        if(!i_pixel_rst_n) pixels_q<=0;
        else pixels_q<={i_valid,i_pixel,i_row,i_col,i_row_start,i_row_commit,i_frame_done,i_frame_id};
    end
    wire scene_valid,scene_active;
    wire [15:0] scene_frame;
    host_f14_crosswalk_path u_crosswalk(
        .i_pixel_clk(i_pixel_clk),.i_pixel_rst_n(model_pixel_rst_n),.i_valid(i_valid),.i_rgb565(i_pixel),
        .i_row(i_row),.i_col(i_col),.i_frame_id(i_frame_id),.i_abort(i_frame_abort),
        .i_sys_clk(i_sys_clk),.i_sys_rst_n(partition_rst_n[0]),.o_valid(scene_valid),.o_frame_id(scene_frame),.o_active(scene_active),
        .o_scan_cycles(),.o_capture_drops());
    host_f14_frame_engine #(.C_INPUT_WIDTH(224),.C_INPUT_HEIGHT(168),.C_PARTITION_RESET(C_PARTITION_RESET)) u_engine (
        .i_pixel_clk(i_pixel_clk),.i_pixel_rst_n(model_pixel_rst_n),
        .i_pixel_valid(i_cnn_valid),.i_pixel_data(i_cnn_pixel),.i_pixel_row(i_cnn_row),.i_pixel_col(i_cnn_col),
        .i_pixel_frame_start(i_cnn_valid && i_cnn_row==0 && i_cnn_col==0),
        .i_pixel_frame_id(i_cnn_frame_id),.i_pixel_frame_abort(i_cnn_abort),
        .i_sys_clk(i_sys_clk),.i_sys_rst_n(partition_rst_n[0]),.i_result_ready(rr),
        .i_reset_partition_n(partition_rst_n),
        .i_scene_valid(scene_valid),.i_scene_frame(scene_frame),.i_scene_active(scene_active),
        .o_result_valid(rv),.o_result_frame_id(rid),.o_lane_x_table(xs),
        .o_lane_valid_table(vs),.o_lane_edge_table(edges),.o_lane_type_table(ts),.o_busy(o_busy),
        .o_diag_sys(o_diag_sys),.o_diag_captured(o_diag_pixel[0])
    );
    host_f14_curve_bridge u_curve (
        .i_result_clk(i_sys_clk),.i_result_rst_n(partition_rst_n[7]),
        .i_result_valid(rv),.i_result_frame_id(rid),.i_result_x_table(xs),
        .i_result_valid_table(vs),.i_result_edge_table(edges),.i_result_type_table(ts),.o_result_ready(rr),
        .i_pixel_clk(i_pixel_clk),.i_pixel_rst_n(model_pixel_rst_n),
        .i_pixel_row_start(i_row_start),.i_pixel_row(i_row),.o_frame_id(curve_frame_id),
        .o_lane_valid(lane_valid),.o_lane_x(lane_x),.o_lane_type(lane_type),
        .o_result_available(curve_available)
    );
    host_lane_meta_collect u_meta (
        .i_clk(i_pixel_clk),.i_rst_n(model_pixel_rst_n && i_meta_rst_n),
        .i_pixel_valid(pixels_q[54]),.i_row(pixels_q[37:29]),.i_col(pixels_q[28:19]),
        .i_frame_done(pixels_q[16]),
        .i_result_available(curve_available),.i_source_id(curve_frame_id),
        .i_lane_valid(lane_valid),.i_lane_type(lane_type),
        .o_valid(o_meta_valid),.i_ready(i_meta_ready),
        .o_seq(o_meta_seq),.o_payload(o_meta_payload)
    );
    host_lane_overlay u_overlay (
        .i_pixel_clk(i_pixel_clk),.i_rst_n(i_pixel_rst_n),
        .i_pixel_valid(pixels_q[54]),.i_pixel_rgb565(pixels_q[53:38]),
        .i_row_index(pixels_q[37:29]),.i_col_index(pixels_q[28:19]),
        .i_row_start(pixels_q[18]),.i_row_commit(pixels_q[17]),
        .i_frame_done(pixels_q[16]),.i_frame_id(pixels_q[15:0]),
        .i_lane0_valid(lane_valid[0]),.i_lane0_x(lane_x[9:0]),.i_lane0_type(lane_type[1:0]),
        .i_lane1_valid(lane_valid[1]),.i_lane1_x(lane_x[19:10]),.i_lane1_type(lane_type[3:2]),
        .i_lane2_valid(lane_valid[2]),.i_lane2_x(lane_x[29:20]),.i_lane2_type(lane_type[5:4]),
        .i_lane3_valid(lane_valid[3]),.i_lane3_x(lane_x[39:30]),.i_lane3_type(lane_type[7:6]),
        .o_pixel_valid(o_valid),.o_pixel_rgb565(o_pixel),.o_row_index(o_row),.o_col_index(o_col),
        .o_row_start(o_row_start),.o_row_commit(o_row_commit),.o_frame_done(o_frame_done),.o_frame_id(o_frame_id)
    );
endmodule
