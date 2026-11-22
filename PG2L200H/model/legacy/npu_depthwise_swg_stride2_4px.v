//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 固定 9×9 输入到 4×4 输出的 stride-2 深度卷积滑窗器，每拍接收四个光栅像素。
// Module Name: npu_depthwise_swg_stride2_4px
// File Name: model/legacy/npu_depthwise_swg_stride2_4px.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 固定 9×9 输入到 4×4 输出的 stride-2 深度卷积滑窗器，每拍接收四个光栅像素。
// 按固定序列输出八组双像素 3×3 窗口，结合 pending 窗口与双级队列支持输出背压，并给出下一组索引和扫描完成。
//
// Dependencies: 内部历史像素、pending window 与双级输出队列
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// clk 域低有效复位；clear 用于清空本次扫描状态，输入和输出仅在各自有效/就绪握手时推进。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

// Fixed 9x9-to-4x4 stride-2 window generator. Four raster pixels are
// accepted per cycle and the selected windows are emitted as eight pairs.
module npu_depthwise_swg_stride2_4px #(
    parameter integer PIXEL_WIDTH = 64
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       clear,
    input  wire                       in_valid,
    output wire                       in_ready,
    input  wire [(PIXEL_WIDTH*4)-1:0] in_data,
    output wire                       pair_valid,
    input  wire                       pair_ready,
    output wire [2:0]                 pair_index,
    output wire [(PIXEL_WIDTH*18)-1:0] pair_windows,
    output wire                       next_pair_valid,
    output wire [2:0]                 next_pair_index,
    output reg                        scan_done
);

    localparam integer WINDOW_WIDTH = PIXEL_WIDTH * 9;
    localparam integer PAIR_WIDTH = WINDOW_WIDTH * 2;

    reg [4:0] beat_q;
    reg [PIXEL_WIDTH-1:0] history_q [0:19];
    reg [WINDOW_WIDTH-1:0] pending_window_q;
    reg pending_window_valid_q;

    reg fifo0_valid_q;
    reg [2:0] fifo0_index_q;
    reg [PAIR_WIDTH-1:0] fifo0_windows_q;
    reg fifo1_valid_q;
    reg [2:0] fifo1_index_q;
    reg [PAIR_WIDTH-1:0] fifo1_windows_q;

    wire [PIXEL_WIDTH-1:0] pixel0 = in_data[0*PIXEL_WIDTH +: PIXEL_WIDTH];
    wire [PIXEL_WIDTH-1:0] pixel1 = in_data[1*PIXEL_WIDTH +: PIXEL_WIDTH];
    wire [PIXEL_WIDTH-1:0] pixel2 = in_data[2*PIXEL_WIDTH +: PIXEL_WIDTH];
    wire [PIXEL_WIDTH-1:0] pixel3 = in_data[3*PIXEL_WIDTH +: PIXEL_WIDTH];

    wire [WINDOW_WIDTH-1:0] window_at_pixel0 = {
        pixel0, history_q[0], history_q[1],
        history_q[8], history_q[9], history_q[10],
        history_q[17], history_q[18], history_q[19]
    };
    wire [WINDOW_WIDTH-1:0] window_at_pixel2 = {
        pixel2, pixel1, pixel0,
        history_q[6], history_q[7], history_q[8],
        history_q[15], history_q[16], history_q[17]
    };

    reg emit_pair;
    reg [2:0] emit_pair_index;
    reg [PAIR_WIDTH-1:0] emit_pair_windows;
    reg save_pending;
    reg clear_pending;
    reg [WINDOW_WIDTH-1:0] next_pending_window;

    always @* begin
        emit_pair = 1'b0;
        emit_pair_index = 3'd0;
        emit_pair_windows = {PAIR_WIDTH{1'b0}};
        save_pending = 1'b0;
        clear_pending = 1'b0;
        next_pending_window = {WINDOW_WIDTH{1'b0}};

        case (beat_q)
            5'd5: begin
                emit_pair = 1'b1;
                emit_pair_index = 3'd0;
                emit_pair_windows = {window_at_pixel2, window_at_pixel0};
            end
            5'd6: begin
                emit_pair = 1'b1;
                emit_pair_index = 3'd1;
                emit_pair_windows = {window_at_pixel2, window_at_pixel0};
            end
            5'd9: begin
                save_pending = 1'b1;
                next_pending_window = window_at_pixel2;
            end
            5'd10: begin
                emit_pair = 1'b1;
                emit_pair_index = 3'd2;
                emit_pair_windows = {window_at_pixel0, pending_window_q};
                save_pending = 1'b1;
                next_pending_window = window_at_pixel2;
            end
            5'd11: begin
                emit_pair = 1'b1;
                emit_pair_index = 3'd3;
                emit_pair_windows = {window_at_pixel0, pending_window_q};
                clear_pending = 1'b1;
            end
            5'd14: begin
                emit_pair = 1'b1;
                emit_pair_index = 3'd4;
                emit_pair_windows = {window_at_pixel2, window_at_pixel0};
            end
            5'd15: begin
                emit_pair = 1'b1;
                emit_pair_index = 3'd5;
                emit_pair_windows = {window_at_pixel2, window_at_pixel0};
            end
            5'd18: begin
                save_pending = 1'b1;
                next_pending_window = window_at_pixel2;
            end
            5'd19: begin
                emit_pair = 1'b1;
                emit_pair_index = 3'd6;
                emit_pair_windows = {window_at_pixel0, pending_window_q};
                save_pending = 1'b1;
                next_pending_window = window_at_pixel2;
            end
            5'd20: begin
                emit_pair = 1'b1;
                emit_pair_index = 3'd7;
                emit_pair_windows = {window_at_pixel0, pending_window_q};
                clear_pending = 1'b1;
            end
            default: begin
                emit_pair = 1'b0;
            end
        endcase
    end

    wire fifo_dequeue = fifo0_valid_q && pair_ready;
    wire fifo_full = fifo0_valid_q && fifo1_valid_q;
    wire fifo_can_enqueue = !fifo_full || fifo_dequeue;
    assign in_ready = !emit_pair || fifo_can_enqueue;
    wire in_fire = in_valid && in_ready;
    wire fifo_enqueue = in_fire && emit_pair;

    assign pair_valid = fifo0_valid_q;
    assign pair_index = fifo0_index_q;
    assign pair_windows = fifo0_windows_q;
    assign next_pair_valid = fifo1_valid_q;
    assign next_pair_index = fifo1_index_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beat_q <= 5'd0;
            pending_window_valid_q <= 1'b0;
            fifo0_valid_q <= 1'b0;
            fifo0_index_q <= 3'd0;
            fifo1_valid_q <= 1'b0;
            fifo1_index_q <= 3'd0;
            scan_done <= 1'b0;
        end else if (clear) begin
            beat_q <= 5'd0;
            pending_window_valid_q <= 1'b0;
            fifo0_valid_q <= 1'b0;
            fifo1_valid_q <= 1'b0;
            scan_done <= 1'b0;
        end else begin
            if (in_fire) begin
                if (beat_q == 5'd20) begin
                    scan_done <= 1'b1;
                end else begin
                    beat_q <= beat_q + 1'b1;
                end

                if (save_pending) begin
                    pending_window_valid_q <= 1'b1;
                end else if (clear_pending) begin
                    pending_window_valid_q <= 1'b0;
                end
            end

            case ({fifo_dequeue, fifo_enqueue})
                2'b01: begin
                    if (!fifo0_valid_q) begin
                        fifo0_valid_q <= 1'b1;
                        fifo0_index_q <= emit_pair_index;
                    end else begin
                        fifo1_valid_q <= 1'b1;
                        fifo1_index_q <= emit_pair_index;
                    end
                end
                2'b10: begin
                    fifo0_valid_q <= fifo1_valid_q;
                    fifo0_index_q <= fifo1_index_q;
                    fifo1_valid_q <= 1'b0;
                end
                2'b11: begin
                    if (fifo1_valid_q) begin
                        fifo0_valid_q <= 1'b1;
                        fifo0_index_q <= fifo1_index_q;
                        fifo1_valid_q <= 1'b1;
                        fifo1_index_q <= emit_pair_index;
                    end else begin
                        fifo0_valid_q <= 1'b1;
                        fifo0_index_q <= emit_pair_index;
                        fifo1_valid_q <= 1'b0;
                    end
                end
                default: begin
                    fifo0_valid_q <= fifo0_valid_q;
                    fifo1_valid_q <= fifo1_valid_q;
                end
            endcase
        end
    end

    integer history_index;
    always @(posedge clk) begin
        if (in_fire) begin
            history_q[0] <= pixel3;
            history_q[1] <= pixel2;
            history_q[2] <= pixel1;
            history_q[3] <= pixel0;
            for (history_index = 4; history_index < 20;
                 history_index = history_index + 1)
                history_q[history_index] <= history_q[history_index-4];

            if (save_pending)
                pending_window_q <= next_pending_window;
        end
    end

    always @(posedge clk) begin
        case ({fifo_dequeue, fifo_enqueue})
            2'b01: begin
                if (!fifo0_valid_q)
                    fifo0_windows_q <= emit_pair_windows;
                else
                    fifo1_windows_q <= emit_pair_windows;
            end
            2'b10: begin
                fifo0_windows_q <= fifo1_windows_q;
            end
            2'b11: begin
                if (fifo1_valid_q) begin
                    fifo0_windows_q <= fifo1_windows_q;
                    fifo1_windows_q <= emit_pair_windows;
                end else begin
                    fifo0_windows_q <= emit_pair_windows;
                end
            end
            default: begin
            end
        endcase
    end

`ifndef SYNTHESIS
    always @(posedge clk) begin
        if (rst_n && !clear && in_fire) begin
            if ((beat_q == 5'd10 || beat_q == 5'd11 ||
                 beat_q == 5'd19 || beat_q == 5'd20) &&
                !pending_window_valid_q)
                $error("stride2 SWG missing pending window at beat %0d", beat_q);
        end
    end
`endif

endmodule

`default_nettype wire
