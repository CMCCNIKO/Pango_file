//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 在启动时锁存运行时卷积描述符，按输出行、像素对、输出/输入通道组及kernel tap枚举head微操作。
// Module Name: f14_head_uop_scheduler_runtime_v1
// File Name: model/rtl/f14_head_uop_scheduler_runtime_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 在启动时锁存运行时卷积描述符，按输出行、像素对、输出/输入通道组及kernel tap枚举head微操作。
// 计算含padding的输入坐标与两个像素有效，产生事务first/last和transaction_index，valid&&ready时推进且不使用可变除法。
//
// Dependencies: 外部feeder的i_uop_ready及运行时几何配置
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 检测零尺寸/非法分组等配置；低有效复位丢弃本次调度，不含MAC或特征存储。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Runtime-configured scheduler for the F14 generic head window path.
// This is the runtime counterpart of f14_head_uop_scheduler.  The existing
// parameterized scheduler is preserved for old gates; this module latches one
// descriptor at start and emits the same uop contract with no variable divide.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

module f14_head_uop_scheduler_runtime_v1 (
    input wire i_clk,
    input wire i_rst_n,
    input wire i_start,
    input wire [7:0] i_cfg_input_h,
    input wire [7:0] i_cfg_input_w,
    input wire [7:0] i_cfg_input_c,
    input wire [7:0] i_cfg_output_h,
    input wire [7:0] i_cfg_output_w,
    input wire [7:0] i_cfg_output_c,
    input wire [3:0] i_cfg_kernel_h,
    input wire [3:0] i_cfg_kernel_w,
    input wire [3:0] i_cfg_stride_y,
    input wire [3:0] i_cfg_stride_x,
    input wire [3:0] i_cfg_pad_y,
    input wire [3:0] i_cfg_pad_x,
    output wire o_start_ready,
    output wire o_busy,
    input wire i_uop_ready,
    output wire o_uop_valid,
    output wire o_uop_first,
    output wire o_uop_last,
    output wire [7:0] o_output_y,
    output wire [7:0] o_output_x,
    output wire [7:0] o_output_group,
    output wire [7:0] o_input_group,
    output wire [7:0] o_tap_y,
    output wire [7:0] o_tap_x,
    output wire signed [15:0] o_input_y,
    output wire signed [15:0] o_input_x,
    output wire o_pixel0_valid,
    output wire o_pixel1_valid,
    output wire [31:0] o_transaction_index,
    output wire o_done,
    output wire o_error
);
    reg busy_q, done_q, error_q;
    reg [7:0] input_h_q, input_w_q, input_c_q;
    reg [7:0] output_h_q, output_w_q, output_c_q;
    reg [3:0] kernel_h_q, kernel_w_q, stride_y_q, stride_x_q;
    reg [3:0] pad_y_q, pad_x_q;
    reg [7:0] output_y_q, pair_q, output_group_q, input_group_q;
    reg [3:0] tap_y_q, tap_x_q;
    reg [31:0] transaction_index_q;
    integer signed input_y_calc, input_x_calc;
    wire [7:0] input_groups = (input_c_q + 8'd7) >> 3;
    wire [7:0] output_groups = (output_c_q + 8'd7) >> 3;
    wire [7:0] pair_count = (output_w_q + 8'd1) >> 1;
    wire uop_fire = o_uop_valid && i_uop_ready;
    wire first_uop = (input_group_q == 0) && (tap_y_q == 0) && (tap_x_q == 0);
    wire last_uop = (input_group_q == input_groups - 1'b1)
        && (tap_y_q == kernel_h_q - 1'b1)
        && (tap_x_q == kernel_w_q - 1'b1);
    wire last_transaction = (output_y_q == output_h_q - 1'b1)
        && (pair_q == pair_count - 1'b1)
        && (output_group_q == output_groups - 1'b1);
    wire cfg_valid = (i_cfg_input_h != 0) && (i_cfg_input_w != 0)
        && (i_cfg_input_c != 0) && (i_cfg_output_h != 0)
        && (i_cfg_output_w != 0) && (i_cfg_output_c != 0)
        && (i_cfg_kernel_h != 0) && (i_cfg_kernel_w != 0)
        && (i_cfg_stride_y != 0) && (i_cfg_stride_x != 0)
        && (((i_cfg_input_c + 7) >> 3) <= 63)
        && (((i_cfg_output_c + 7) >> 3) <= 63);

    assign o_start_ready = !busy_q;
    assign o_busy = busy_q;
    assign o_uop_valid = busy_q && !error_q;
    assign o_uop_first = o_uop_valid && first_uop;
    assign o_uop_last = o_uop_valid && last_uop;
    assign o_output_y = output_y_q;
    assign o_output_x = pair_q << 1;
    assign o_output_group = output_group_q;
    assign o_input_group = input_group_q;
    assign o_tap_y = tap_y_q;
    assign o_tap_x = tap_x_q;
    always @(*) begin
        input_y_calc = $signed({1'b0, output_y_q}) * $signed({1'b0, stride_y_q})
            + $signed({1'b0, tap_y_q}) - $signed({1'b0, pad_y_q});
        input_x_calc = $signed({1'b0, pair_q}) * 2 * $signed({1'b0, stride_x_q})
            + $signed({1'b0, tap_x_q}) - $signed({1'b0, pad_x_q});
    end
    assign o_input_y = input_y_calc;
    assign o_input_x = input_x_calc;
    // Pixel validity has two independent parts: the output pair must contain
    // a real output coordinate, and the corresponding input tap must be in
    // bounds.  The old expression checked only the input tap, so the second
    // lane of an odd-width output (for example vertical W=3) was marked valid
    // whenever its padded input coordinate happened to be in range.  That
    // leaked stale data into the unused half of the output word.
    assign o_pixel0_valid = (pair_q * 2 < output_w_q)
        && (input_y_calc >= 0) && (input_y_calc < input_h_q)
        && (input_x_calc >= 0) && (input_x_calc < input_w_q);
    assign o_pixel1_valid = (pair_q * 2 + 1 < output_w_q)
        && (input_y_calc >= 0) && (input_y_calc < input_h_q)
        && ((input_x_calc + $signed({1'b0, stride_x_q})) >= 0)
        && ((input_x_calc + $signed({1'b0, stride_x_q})) < input_w_q);
    assign o_transaction_index = transaction_index_q;
    assign o_done = done_q;
    assign o_error = error_q;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            busy_q <= 1'b0;
            done_q <= 1'b0;
            error_q <= 1'b0;
            input_h_q <= 0; input_w_q <= 0; input_c_q <= 0;
            output_h_q <= 0; output_w_q <= 0; output_c_q <= 0;
            kernel_h_q <= 0; kernel_w_q <= 0;
            stride_y_q <= 0; stride_x_q <= 0;
            pad_y_q <= 0; pad_x_q <= 0;
            output_y_q <= 0; pair_q <= 0; output_group_q <= 0;
            input_group_q <= 0; tap_y_q <= 0; tap_x_q <= 0;
            transaction_index_q <= 0;
        end else begin
            done_q <= 1'b0;
            if (!busy_q && i_start) begin
                input_h_q <= i_cfg_input_h; input_w_q <= i_cfg_input_w;
                input_c_q <= i_cfg_input_c; output_h_q <= i_cfg_output_h;
                output_w_q <= i_cfg_output_w; output_c_q <= i_cfg_output_c;
                kernel_h_q <= i_cfg_kernel_h; kernel_w_q <= i_cfg_kernel_w;
                stride_y_q <= i_cfg_stride_y; stride_x_q <= i_cfg_stride_x;
                pad_y_q <= i_cfg_pad_y; pad_x_q <= i_cfg_pad_x;
                output_y_q <= 0; pair_q <= 0; output_group_q <= 0;
                input_group_q <= 0; tap_y_q <= 0; tap_x_q <= 0;
                transaction_index_q <= 0;
                error_q <= !cfg_valid;
                busy_q <= 1'b1;
            end else if (uop_fire && !error_q) begin
                if (last_uop) begin
                    input_group_q <= 0;
                    tap_y_q <= 0;
                    tap_x_q <= 0;
                    if (last_transaction) begin
                        busy_q <= 1'b0;
                        done_q <= 1'b1;
                    end else if (output_group_q < output_groups - 1'b1) begin
                        output_group_q <= output_group_q + 1'b1;
                        transaction_index_q <= transaction_index_q + 1'b1;
                    end else begin
                        output_group_q <= 0;
                        transaction_index_q <= transaction_index_q + 1'b1;
                        if (pair_q < pair_count - 1'b1) begin
                            pair_q <= pair_q + 1'b1;
                        end else begin
                            pair_q <= 0;
                            output_y_q <= output_y_q + 1'b1;
                        end
                    end
                end else if (input_group_q < input_groups - 1'b1) begin
                    input_group_q <= input_group_q + 1'b1;
                end else if (tap_x_q < kernel_w_q - 1'b1) begin
                    input_group_q <= 0;
                    tap_x_q <= tap_x_q + 1'b1;
                end else begin
                    input_group_q <= 0;
                    tap_x_q <= 0;
                    tap_y_q <= tap_y_q + 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
