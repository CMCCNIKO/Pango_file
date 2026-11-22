//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 将 1920×1080 RGB888 输入裁剪为 y=60..1019 的 960 行，再按横纵三倍抽取输出 640×320 RGB565；默认每两帧选一帧。
// Module Name: host_hdmi_tile_normalizer
// File Name: host/rtl/video/host_hdmi_tile_normalizer.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 将 1920×1080 RGB888 输入裁剪为 y=60..1019 的 960 行，再按横纵三倍抽取输出 640×320 RGB565；默认每两帧选一帧。
// 使用模三相位与行列计数器产生像素有效、行起始/提交、帧完成及帧号，检测异常像素长度后输出帧中止，不在像素路径引入除法。
//
// Dependencies: 像素时钟 i_pixel_clk/i_rst_n；i_mode_valid；帧尺寸、裁剪起点、抽取率与帧选择由参数配置。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
// 1920x1080p60 RGB888 -> crop y=60..1019 (960 lines), 3:1 decimate both axes
// -> 640x320 RGB565.
//
// The 3:1 decimation uses explicit mod-3 phase counters (x_phase/y_phase)
// instead of division in the pixel path.  o_row_index comes from a plain
// output-row counter that increments on every committed line, so no divides
// or modulos reach the pixel-clock domain logic.
//
// VS polarity: the frame-reset edge is i_vs && !vs_d (rising).  With the
// ADV7611's INV_VS_POL=1 the pin is high during the sync pulse, so the rising
// edge is the frame start (line 0); with an active-low VS the same edge is
// the end of the 5-line sync pulse, i.e. the first active line -- in both
// cases the reset lands on or before the first DE row, so the coordinate
// counters stay correct for either polarity.
module host_hdmi_tile_normalizer #(
    parameter integer C_FRAME_DIV = 2,
    parameter integer C_SRC_WIDTH = 1920,
    parameter integer C_SRC_HEIGHT = 1080,
    parameter integer C_CROP_Y = 60,
    parameter integer C_DECIM = 3,
    parameter integer C_OUT_WIDTH = 640,
    parameter integer C_OUT_HEIGHT = 320
) (
    input wire i_pixel_clk,
    input wire i_rst_n,
    input wire i_de,
    input wire i_hs,
    input wire i_vs,
    input wire [23:0] i_rgb888,
    input wire i_mode_valid,
    output reg o_pixel_valid,
    output reg [15:0] o_pixel_rgb565,
    output reg [8:0] o_row_index,
    output reg [9:0] o_col_index,
    output reg o_row_start,
    output reg o_row_commit,
    output reg o_frame_done,
    output reg o_frame_abort,
    output reg [15:0] o_frame_id,
    output reg o_frame_selected
);
    reg de_d, hs_d, vs_d, line_seen;
    reg [11:0] src_x, src_y;
    reg [1:0] x_phase, y_phase;
    reg [9:0] out_row;
    reg [9:0] out_col;
    reg [15:0] frame_counter;
    reg [15:0] rgb565;
    always @* begin
        rgb565 = {i_rgb888[23:19], i_rgb888[15:10], i_rgb888[7:3]};
    end
    wire in_crop = (src_y >= C_CROP_Y) && (src_y < C_CROP_Y + C_OUT_HEIGHT*C_DECIM);
    wire take    = (x_phase == 2'd0) && (y_phase == 2'd0);
    wire last_take_x = (src_x == C_SRC_WIDTH - C_DECIM);
    wire last_take_y = (src_y == C_CROP_Y + (C_OUT_HEIGHT-1)*C_DECIM);

    always @(posedge i_pixel_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            de_d <= 0; hs_d <= 0; vs_d <= 0; line_seen <= 0;
            src_x <= 0; src_y <= 0; x_phase <= 0; y_phase <= 0; out_row <= 0;
            out_col <= 0;
            frame_counter <= 0; o_frame_id <= 0; o_frame_selected <= 0;
            o_pixel_valid <= 0; o_pixel_rgb565 <= 0; o_row_index <= 0; o_col_index <= 0;
            o_row_start <= 0; o_row_commit <= 0; o_frame_done <= 0; o_frame_abort <= 0;
        end else begin
            de_d <= i_de;
            hs_d <= i_hs;
            vs_d <= i_vs;
            o_pixel_valid <= 0;
            o_row_start <= 0;
            o_row_commit <= 0;
            o_frame_done <= 0;
            o_frame_abort <= 0;
            if (i_vs && !vs_d) begin
                frame_counter <= frame_counter + 1'b1;
                o_frame_id <= frame_counter + 1'b1;
                // 首版只允许整除 1 或 2；不把除法/模运算放入像素关键路径。
                if (C_FRAME_DIV == 1)
                    o_frame_selected <= 1'b1;
                else if (C_FRAME_DIV == 2)
                    o_frame_selected <= ((frame_counter + 1'b1) & 16'h0001) == 0;
                else
                    o_frame_selected <= 1'b0;
                src_x <= 0;
                src_y <= 0;
                line_seen <= 0;
                x_phase <= 0;
                y_phase <= 0;
                out_row <= 0;
                out_col <= 0;
            end
            if (i_hs && !hs_d) begin
                src_x <= 0;
                x_phase <= 0;
                out_col <= 0;
                line_seen <= 1;
            end
            // 只统计DE有效图像行；HS在垂直消隐期间仍会跳变，不能用它累计y。
            if (!i_de && de_d) begin
                src_y <= src_y + 1'b1;
                if (y_phase == 2'd2) y_phase <= 2'd0;
                else y_phase <= y_phase + 1'b1;
                if (src_x != C_SRC_WIDTH)
                    o_frame_abort <= 1;
            end
            if (i_de) begin
                if (i_mode_valid && o_frame_selected && in_crop &&
                    (src_y < C_SRC_HEIGHT) && take) begin
                    o_pixel_valid <= 1'b1;
                    o_pixel_rgb565 <= rgb565;
                    o_row_index <= out_row[8:0];
                    o_col_index <= out_col;
                    if (src_x == 0)
                        o_row_start <= 1'b1;
                    if (last_take_x) begin
                        o_row_commit <= 1'b1;
                        if (last_take_y)
                            o_frame_done <= 1'b1;
                        out_row <= out_row + 1'b1;
                        out_col <= 0;
                    end else begin
                        out_col <= out_col + 1'b1;
                    end
                end
                if (src_x < C_SRC_WIDTH)
                    src_x <= src_x + 1'b1;
                if (x_phase == 2'd2)
                    x_phase <= 2'd0;
                else
                    x_phase <= x_phase + 1'b1;
            end
        end
    end
endmodule
