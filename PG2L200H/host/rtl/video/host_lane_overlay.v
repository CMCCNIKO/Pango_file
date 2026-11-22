//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 在 640×320 RGB565 视频上根据四车道逐行 x、有效位和类型绘制连续曲线，以颜色区分实线、虚线和未知；重叠时按槽位固定优先级选择。
// Module Name: host_lane_overlay
// File Name: host/rtl/video/host_lane_overlay.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 在 640×320 RGB565 视频上根据四车道逐行 x、有效位和类型绘制连续曲线，以颜色区分实线、虚线和未知；重叠时按槽位固定优先级选择。
// 先将像素、坐标、车道信息及行/帧标记共同寄存一拍，再组合判断命中距离与颜色；不存储整帧，旧结果年龄由上游曲线桥管理。
//
// Dependencies: host_lane_type_defs.vh；i_pixel_clk/i_rst_n；调用方需对齐逐行曲线坐标。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 原开头提到 adds no latency 与当前实现不符，新说明按正文寄存逻辑标明一拍。
// 当前正文所有类型都绘制连续曲线，C_DASH_ON_ROWS 仅为兼容保留，不按行周期生成虚线外观。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`include "video/host_lane_type_defs.vh"
// Pixel-domain lane overlay for the 640x320 HSST video stream.
//
// The decoder supplies one video-space x coordinate per lane for the current
// output row.  Keeping the decoder separate is intentional: F14's raw
// cls/vertical/offset tensors can first be decoded and checked independently,
// while this block remains a small, timing-friendly pixel operation.
//
// Coordinates are in the outgoing video domain (x=0..639, y=0..319).  The
// block has no frame storage and adds no latency; it overlays the most recent
// accepted F14 result on the current video frame.  Frame age/ID policy is
// handled by the result bridge that drives the lane_x/lane_valid inputs.
module host_lane_overlay #(
    parameter integer C_LINE_HALF_WIDTH = 2,
    parameter integer C_DASH_ON_ROWS = 8,
    parameter [15:0] C_UNKNOWN_COLOR565 = 16'hFFFF,
    parameter [15:0] C_SOLID_COLOR565 = 16'hFFE0,
    parameter [15:0] C_DASHED_COLOR565 = 16'h001F,
    parameter [15:0] C_NONE_COLOR565 = 16'hFFFF
) (
    input  wire        i_pixel_clk,
    input  wire        i_rst_n,
    input  wire        i_pixel_valid,
    input  wire [15:0] i_pixel_rgb565,
    input  wire [8:0]  i_row_index,
    input  wire [9:0]  i_col_index,
    input  wire        i_row_start,
    input  wire        i_row_commit,
    input  wire        i_frame_done,
    input  wire [15:0] i_frame_id,

    input  wire        i_lane0_valid,
    input  wire [9:0]  i_lane0_x,
    input  wire [1:0]  i_lane0_type,
    input  wire        i_lane1_valid,
    input  wire [9:0]  i_lane1_x,
    input  wire [1:0]  i_lane1_type,
    input  wire        i_lane2_valid,
    input  wire [9:0]  i_lane2_x,
    input  wire [1:0]  i_lane2_type,
    input  wire        i_lane3_valid,
    input  wire [9:0]  i_lane3_x,
    input  wire [1:0]  i_lane3_type,

    output wire        o_pixel_valid,
    output wire [15:0] o_pixel_rgb565,
    output wire [8:0]  o_row_index,
    output wire [9:0]  o_col_index,
    output wire        o_row_start,
    output wire        o_row_commit,
    output wire        o_frame_done,
    output wire [15:0] o_frame_id
);

    // Register the normalizer metadata and decoder coordinates before the
    // hit-distance arithmetic. This adds one pixel-clock cycle while keeping
    // row and commit markers aligned with the corresponding pixel.
    reg        pixel_valid_r;
    reg [15:0] pixel_rgb565_r;
    reg [8:0]  row_index_r;
    reg [9:0]  col_index_r;
    reg        row_start_r, row_commit_r, frame_done_r;
    reg [15:0] frame_id_r;
    reg lane0_valid_r, lane1_valid_r, lane2_valid_r, lane3_valid_r;
    reg [9:0] lane0_x_r, lane1_x_r, lane2_x_r, lane3_x_r;
    reg [1:0] lane0_type_r, lane1_type_r, lane2_type_r, lane3_type_r;

    always @(posedge i_pixel_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            pixel_valid_r <= 1'b0;
            pixel_rgb565_r <= 16'd0;
            row_index_r <= 9'd0;
            col_index_r <= 10'd0;
            row_start_r <= 1'b0;
            row_commit_r <= 1'b0;
            frame_done_r <= 1'b0;
            frame_id_r <= 16'd0;
            lane0_valid_r <= 1'b0;
            lane1_valid_r <= 1'b0;
            lane2_valid_r <= 1'b0;
            lane3_valid_r <= 1'b0;
            lane0_x_r <= 10'd0;
            lane1_x_r <= 10'd0;
            lane2_x_r <= 10'd0;
            lane3_x_r <= 10'd0;
            lane0_type_r <= `HOST_LANE_TYPE_NONE;
            lane1_type_r <= `HOST_LANE_TYPE_NONE;
            lane2_type_r <= `HOST_LANE_TYPE_NONE;
            lane3_type_r <= `HOST_LANE_TYPE_NONE;
        end else begin
            pixel_valid_r <= i_pixel_valid;
            pixel_rgb565_r <= i_pixel_rgb565;
            row_index_r <= i_row_index;
            col_index_r <= i_col_index;
            row_start_r <= i_row_start;
            row_commit_r <= i_row_commit;
            frame_done_r <= i_frame_done;
            frame_id_r <= i_frame_id;
            lane0_valid_r <= i_lane0_valid;
            lane1_valid_r <= i_lane1_valid;
            lane2_valid_r <= i_lane2_valid;
            lane3_valid_r <= i_lane3_valid;
            lane0_x_r <= i_lane0_x;
            lane1_x_r <= i_lane1_x;
            lane2_x_r <= i_lane2_x;
            lane3_x_r <= i_lane3_x;
            lane0_type_r <= i_lane0_type;
            lane1_type_r <= i_lane1_type;
            lane2_type_r <= i_lane2_type;
            lane3_type_r <= i_lane3_type;
        end
    end

    function hit_x;
        input [9:0] x_pixel;
        input [9:0] x_lane;
        reg [10:0] delta;
        begin
            if (x_pixel >= x_lane)
                delta = {1'b0, x_pixel} - {1'b0, x_lane};
            else
                delta = {1'b0, x_lane} - {1'b0, x_pixel};
            hit_x = (delta <= C_LINE_HALF_WIDTH);
        end
    endfunction

    // All classes draw continuous curves; color alone identifies lane type.
    // C_DASH_ON_ROWS is retained only for source compatibility.
    wire hit0 = lane0_valid_r &&
                hit_x(col_index_r, lane0_x_r);
    wire hit1 = lane1_valid_r &&
                hit_x(col_index_r, lane1_x_r);
    wire hit2 = lane2_valid_r &&
                hit_x(col_index_r, lane2_x_r);
    wire hit3 = lane3_valid_r &&
                hit_x(col_index_r, lane3_x_r);

    // 颜色由类别决定，与左右槽位无关；NONE 仍需几何 valid 才显示。
    function [15:0] type_color;
        input [1:0] lane_type;
        begin
            case (lane_type)
                `HOST_LANE_TYPE_SOLID: type_color = C_SOLID_COLOR565;
                `HOST_LANE_TYPE_DASHED: type_color = C_DASHED_COLOR565;
                `HOST_LANE_TYPE_NONE: type_color = C_NONE_COLOR565;
                default: type_color = C_UNKNOWN_COLOR565;
            endcase
        end
    endfunction

    reg [15:0] overlay_pixel;
    always @* begin
        overlay_pixel = pixel_rgb565_r;
        if (pixel_valid_r) begin
            // Fixed priority keeps the result deterministic if two lanes are
            // closer than the configured overlay width at a crossing.
            if (hit0)
                overlay_pixel = type_color(lane0_type_r);
            else if (hit1)
                overlay_pixel = type_color(lane1_type_r);
            else if (hit2)
                overlay_pixel = type_color(lane2_type_r);
            else if (hit3)
                overlay_pixel = type_color(lane3_type_r);
        end
    end

    assign o_pixel_valid    = pixel_valid_r;
    assign o_pixel_rgb565   = overlay_pixel;
    assign o_row_index      = row_index_r;
    assign o_col_index      = col_index_r;
    assign o_row_start      = row_start_r;
    assign o_row_commit     = row_commit_r;
    assign o_frame_done     = frame_done_r;
    assign o_frame_id       = frame_id_r;

endmodule
