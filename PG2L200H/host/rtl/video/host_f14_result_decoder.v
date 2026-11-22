//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 顺序读取 F14 串行结果 RAM，逐车道/锚点进行分类列选择与 offset 解码，输出四车道各 21 点的 x、有效及类型表。
// Module Name: host_f14_result_decoder
// File Name: host/rtl/video/host_f14_result_decoder.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 顺序读取 F14 串行结果 RAM，逐车道/锚点进行分类列选择与 offset 解码，输出四车道各 21 点的 x、有效及类型表。
// 通过冻结整数 sigmoid 阈值及整数坐标缩放处理原始 signed16 偏移，避免大规模并行结果总线与 84 路组合 argmax。
//
// Dependencies: host_f14_decode_contract.vh；i_clk/i_rst_n；o_req/o_addr 与 read_valid/read_data 结果读口。
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

// Reads the F14 serial result BRAM and decodes one anchor per lane/row.
// No 76-kbit parallel result bus or 84-way combinational argmax is needed.
module host_f14_result_decoder (
    input wire i_clk,
    input wire i_rst_n,
    input wire i_start,
    input wire i_error,
    output wire o_req,
    output wire [12:0] o_addr,
    input wire i_read_valid,
    input wire [15:0] i_read_data,
    output reg o_done,
    output reg [839:0] o_lane_x_table,
    output reg [83:0] o_lane_valid_table,
    output reg [7:0] o_lane_type_table
);
    `include "video/host_f14_decode_contract.vh"
    localparam [3:0] ST_IDLE = 0, ST_BASE = 1,
        ST_OFFSET = 2, ST_DONE = 3, ST_OFFSET_WAIT = 4,
        ST_SIGMOID = 5, ST_SCALE = 6, ST_DIVIDE = 7, ST_STORE = 8;
    reg [3:0] state_q;
    reg [12:0] base_req_q, base_resp_q;
    reg [6:0] off_req_q, off_resp_q;
    reg [4:0] cls_col_q;
    reg [6:0] cls_row_q;
    reg [4:0] off_row_q;
    reg [1:0] off_lane_q;
    reg signed [15:0] best_score [0:83];
    reg [4:0] best_col [0:83];
    reg signed [15:0] offset_raw_q;
    reg [7:0] low_q, high_q;
    wire [8:0] midpoint_sum = {1'b0,low_q} + {1'b0,high_q};
    wire [7:0] midpoint = midpoint_sum[8:1];
    reg [14:0] x_numerator_q;
    reg [26:0] x_product_q;
    wire [10:0] x_floor = x_product_q[26:16];
    wire [15:0] x_remainder = {1'b0,x_numerator_q} - x_floor*16'd28;
    wire [10:0] x_rounded = x_floor + (x_remainder >= 28);
    integer k;

    assign o_req = (state_q == ST_BASE && base_req_q < 13'd2436) ||
                   (state_q == ST_OFFSET && off_req_q < 7'd84);
    assign o_addr = (state_q == ST_OFFSET) ?
        (13'd2436 + ({11'd0, off_lane_q} * 13'd588) +
         ({8'd0, off_row_q} * 13'd28) + best_col[off_req_q]) : base_req_q;

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state_q <= ST_IDLE;
            base_req_q <= 0; base_resp_q <= 0;
            off_req_q <= 0; off_resp_q <= 0;
            cls_col_q <= 0; cls_row_q <= 0;
            off_row_q <= 0; off_lane_q <= 0;
            offset_raw_q <= 0; low_q <= 0; high_q <= 0;
            x_numerator_q <= 0; x_product_q <= 0;
            o_done <= 1'b0;
            o_lane_x_table <= 0;
            o_lane_valid_table <= 0;
            o_lane_type_table <= 0;
        end else begin
            o_done <= 1'b0;
            case (state_q)
                ST_IDLE: if (i_start) begin
                    base_req_q <= 0; base_resp_q <= 0;
                    off_req_q <= 0; off_resp_q <= 0;
                    cls_col_q <= 0; cls_row_q <= 0;
                    off_row_q <= 0; off_lane_q <= 0;
                    o_lane_x_table <= 0;
                    o_lane_valid_table <= 0;
                    o_lane_type_table <= 0;
                    for (k = 0; k < 84; k = k + 1) begin
                        best_score[k] <= -16'sd32768;
                        best_col[k] <= 0;
                    end
                    state_q <= i_error ? ST_DONE : ST_BASE;
                end
                ST_BASE: begin
                    if (o_req)
                        base_req_q <= base_req_q + 1'b1;
                    if (i_read_valid) begin
                        base_resp_q <= base_resp_q + 1'b1;
                        if (base_resp_q < 13'd2352) begin
                            if ($signed(i_read_data) > best_score[cls_row_q]) begin
                                best_score[cls_row_q] <= $signed(i_read_data);
                                best_col[cls_row_q] <= cls_col_q;
                            end
                            if (cls_col_q == 5'd27) begin
                                cls_col_q <= 0;
                                cls_row_q <= cls_row_q + 1'b1;
                            end else
                                cls_col_q <= cls_col_q + 1'b1;
                        end else begin
                            // Frozen INT16 output scale maps probability 0.35
                            // to logit ~= -318.  Keep the comparison integer
                            // and avoid a sigmoid/divide in the datapath.
                            if ($signed(i_read_data) >= -16'sd318)
                                o_lane_valid_table[base_resp_q-13'd2352] <= 1'b1;
                        end
                        if (base_resp_q == 13'd2435) begin
                            off_req_q <= 0; off_resp_q <= 0;
                            off_row_q <= 0; off_lane_q <= 0;
                            state_q <= ST_OFFSET;
                        end
                    end
                end
                ST_OFFSET: begin
                    if (o_req) begin
                        state_q <= ST_OFFSET_WAIT;
                    end
                end
                ST_OFFSET_WAIT: begin
                    if (i_read_valid) begin
                        offset_raw_q <= $signed(i_read_data);
                        low_q <= 0; high_q <= 128;
                        state_q <= ST_SIGMOID;
                    end
                end
                ST_SIGMOID: begin
                    if (low_q == high_q) state_q <= ST_SCALE;
                    else if (offset_raw_q >= offset_sigmoid_cut(midpoint[6:0]))
                        low_q <= midpoint + 1'b1;
                    else high_q <= midpoint;
                end
                ST_SCALE: begin
                    // Pixel-center inverse transform, rounded to nearest:
                    // floor((5*(column*128+offset_Q4)+40)/28).
                    x_numerator_q <= ({10'd0,best_col[off_resp_q]} * 15'd640) +
                                     ({7'd0,low_q} * 15'd5) + 15'd40;
                    state_q <= ST_DIVIDE;
                end
                ST_DIVIDE: begin
                    x_product_q <= x_numerator_q * 12'd2340;
                    state_q <= ST_STORE;
                end
                ST_STORE: begin
                    o_lane_x_table[off_resp_q*10 +: 10] <= x_rounded[9:0];
                    if (x_rounded >= 640) o_lane_valid_table[off_resp_q] <= 0;
                    off_resp_q <= off_resp_q + 1'b1;
                    off_req_q <= off_req_q + 1'b1;
                    if (off_row_q == 5'd20) begin
                        off_row_q <= 0;
                        off_lane_q <= off_lane_q + 1'b1;
                    end else off_row_q <= off_row_q + 1'b1;
                    if (off_resp_q == 7'd83) state_q <= ST_DONE;
                    else state_q <= ST_OFFSET;
                end
                ST_DONE: begin
                    for (k = 0; k < 4; k = k + 1)
                        o_lane_type_table[k*2 +: 2] <=
                            (|o_lane_valid_table[k*21 +: 21]) ? 2'b11 : 2'b00;
                    o_done <= 1'b1;
                    state_q <= ST_IDLE;
                end
                default: state_q <= ST_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
