//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 统计输入每行 DE 像素和每帧有效行数，默认连续三帧满足 1920×1080 后确认模式；遇异常帧立即撤销有效状态并输出帧错误和实测尺寸。
// Module Name: host_hdmi_mode_monitor
// File Name: host/rtl/video/host_hdmi_mode_monitor.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 统计输入每行 DE 像素和每帧有效行数，默认连续三帧满足 1920×1080 后确认模式；遇异常帧立即撤销有效状态并输出帧错误和实测尺寸。
// 以任意 VS 边沿判定有有效行的帧结束，忽略无数据的同一同步脉冲另一边沿，对 VS 极性保持兼容。
//
// Dependencies: 像素时钟 i_pixel_clk、低有效 i_rst_n；默认 C_EXPECTED_WIDTH=1920、C_EXPECTED_HEIGHT=1080、C_GOOD_FRAMES=3。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 原注释中 frame_lines==720 为历史示例，实际逻辑使用 C_EXPECTED_HEIGHT。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
// 1920x1080p60 input monitor.  A mode is accepted only after three complete
// frames with 1920 DE pixels on 1080 lines; any malformed frame clears the
// acceptance state immediately.
//
// VS polarity note: the ADV7611 configuration used on this board sets
// INV_VS_POL=1 (IO Map 0x06 = 0xa6), so the VS pin toggles HIGH during the
// sync pulse.  This monitor is polarity-agnostic instead: at *every* VS edge
// it classifies the frame_lines counter --
//   frame_lines == 720 && !bad_frame -> a full frame completed (good)
//   frame_lines != 0                 -> a full but malformed frame (error)
//   frame_lines == 0                 -> the opposite edge of the same pulse;
//                                       no DE rows exist inside the 5-line
//                                       sync pulse, so it is simply the
//                                       frame-start edge and is ignored.
// Both polarities therefore converge on the same accept/reject sequence.
module host_hdmi_mode_monitor #(
    parameter integer C_EXPECTED_WIDTH = 1920,
    parameter integer C_EXPECTED_HEIGHT = 1080,
    parameter integer C_GOOD_FRAMES = 3
) (
    input wire i_pixel_clk,
    input wire i_rst_n,
    input wire i_de,
    input wire i_hs,
    input wire i_vs,
    output reg o_mode_valid,
    output reg o_frame_start,
    output reg o_frame_done,
    output reg o_frame_error,
    output reg [11:0] o_measured_width,
    output reg [11:0] o_measured_height
);
    reg de_d, vs_d;
    reg [11:0] line_pixels, frame_lines;
    reg [1:0] good_frames;
    reg bad_frame;
    wire vs_edge = i_vs ^ vs_d;
    always @(posedge i_pixel_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            de_d <= 0; vs_d <= 0; line_pixels <= 0; frame_lines <= 0;
            good_frames <= 0; bad_frame <= 0; o_mode_valid <= 0;
            o_frame_start <= 0; o_frame_done <= 0; o_frame_error <= 0;
            o_measured_width <= 0; o_measured_height <= 0;
        end else begin
            o_frame_start <= 0;
            o_frame_done <= 0;
            o_frame_error <= 0;
            vs_d <= i_vs;
            de_d <= i_de;
            if (i_de) begin
                line_pixels <= line_pixels + 1'b1;
                if (line_pixels >= C_EXPECTED_WIDTH)
                    bad_frame <= 1;
            end
            if (!i_de && de_d) begin
                o_measured_width <= line_pixels;
                if (line_pixels != C_EXPECTED_WIDTH)
                    bad_frame <= 1;
                frame_lines <= frame_lines + 1'b1;
                line_pixels <= 0;
            end
            if (vs_edge) begin
                o_measured_height <= frame_lines;
                if (frame_lines == C_EXPECTED_HEIGHT && !bad_frame) begin
                    o_frame_done <= 1;
                    if (good_frames < C_GOOD_FRAMES)
                        good_frames <= good_frames + 1'b1;
                    if (good_frames + 1 >= C_GOOD_FRAMES)
                        o_mode_valid <= 1;
                end else if (frame_lines != 0) begin
                    o_frame_done <= 1;
                    o_frame_error <= 1;
                    good_frames <= 0;
                    o_mode_valid <= 0;
                end else begin
                    o_frame_start <= 1;
                end
                line_pixels <= 0;
                frame_lines <= 0;
                bad_frame <= 0;
            end
        end
    end
endmodule
