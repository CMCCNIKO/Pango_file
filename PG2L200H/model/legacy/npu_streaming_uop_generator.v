//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 从单个几何配置字按固定 EXP→DW→PROJ 次序生成流式微操作 token。
// Module Name: npu_streaming_uop_generator
// File Name: model/legacy/npu_streaming_uop_generator.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 从单个几何配置字按固定 EXP→DW→PROJ 次序生成流式微操作 token。
// 维护组、像素对和 chunk 计数，仅在 token_valid&&token_ready 时推进，支持 stride2/exp_only 并检测非法几何参数。
//
// Dependencies: 32 位 cfg_geometry 与 token_uop 布局
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 低有效复位清空在途 token；不含计算阵列和完整描述符表。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns/1ps
`default_nettype none

// Generate the frozen EXP -> DW -> PROJ token order from one local geometry
// word. The generator advances only on ready/valid handshakes, so downstream
// stalls cannot skip or duplicate a micro-op.
module npu_streaming_uop_generator (
    input  wire          clk,
    input  wire          rst_n,
    input  wire          start,
    output wire          start_ready,
    input  wire [31:0]   cfg_geometry,

    output wire          token_valid,
    input  wire          token_ready,
    output wire [31:0]   token_uop,

    output wire          busy,
    output reg           done,
    output reg           status_error,
    output reg  [31:0]   emitted_uops
);

    localparam [1:0] OP_EXP = 2'd0;
    localparam [1:0] OP_DW = 2'd1;
    localparam [1:0] OP_PROJ = 2'd2;

    reg active_q;
    reg [1:0] op_q;
    reg [5:0] group_q;
    reg [5:0] pair_q;
    reg [5:0] chunk_q;
    reg [5:0] input_groups_q;
    reg [5:0] expanded_groups_q;
    reg [5:0] output_groups_q;
    reg stride2_q;
    reg exp_only_q;

    wire [5:0] cfg_input_groups = cfg_geometry[5:0];
    wire [5:0] cfg_expanded_groups = cfg_geometry[11:6];
    wire [5:0] cfg_output_groups = cfg_geometry[17:12];
    wire cfg_stride2 = cfg_geometry[29];
    wire cfg_exp_only = cfg_geometry[30];
    wire cfg_valid = (cfg_input_groups != 0)
        && (cfg_input_groups <= 9)
        && (cfg_expanded_groups != 0)
        && (cfg_expanded_groups <= 54)
        && (cfg_output_groups != 0)
        && (cfg_output_groups <= 9)
        && (!cfg_exp_only || (!cfg_stride2
            && (cfg_expanded_groups == cfg_output_groups)));

    wire [5:0] pair_last = (op_q == OP_EXP)
        ? (exp_only_q ? 6'd31 : 6'd40)
        : (stride2_q ? 6'd7 : 6'd31);
    wire [5:0] chunk_last = (op_q == OP_EXP)
        ? (input_groups_q - 1'b1)
        : ((op_q == OP_DW) ? 6'd1 : (expanded_groups_q - 1'b1));
    wire [5:0] group_last = (op_q == OP_PROJ)
        ? (output_groups_q - 1'b1)
        : (expanded_groups_q - 1'b1);
    wire token_fire = token_valid && token_ready;
    wire token_last_chunk = chunk_q == chunk_last;
    wire token_last_pair = pair_q == pair_last;
    wire token_last_group = group_q == group_last;

    assign start_ready = !active_q;
    assign busy = active_q;
    assign token_valid = active_q;
    assign token_uop = {
        12'd0, chunk_q, pair_q, group_q, op_q
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_q <= 1'b0;
            op_q <= OP_EXP;
            group_q <= 6'd0;
            pair_q <= 6'd0;
            chunk_q <= 6'd0;
            input_groups_q <= 6'd0;
            expanded_groups_q <= 6'd0;
            output_groups_q <= 6'd0;
            stride2_q <= 1'b0;
            exp_only_q <= 1'b0;
            done <= 1'b0;
            status_error <= 1'b0;
            emitted_uops <= 32'd0;
        end else begin
            done <= 1'b0;

            if (start && start_ready) begin
                input_groups_q <= cfg_input_groups;
                expanded_groups_q <= cfg_expanded_groups;
                output_groups_q <= cfg_output_groups;
                stride2_q <= cfg_stride2;
                exp_only_q <= cfg_exp_only;
                op_q <= OP_EXP;
                group_q <= 6'd0;
                pair_q <= 6'd0;
                chunk_q <= 6'd0;
                emitted_uops <= 32'd0;
                status_error <= !cfg_valid;
                active_q <= cfg_valid;
                if (!cfg_valid)
                    done <= 1'b1;
            end else if (start) begin
                status_error <= 1'b1;
            end

            if (token_fire) begin
                emitted_uops <= emitted_uops + 1'b1;
                if (!token_last_chunk) begin
                    chunk_q <= chunk_q + 1'b1;
                end else begin
                    chunk_q <= 6'd0;
                    if (!token_last_pair) begin
                        pair_q <= pair_q + 1'b1;
                    end else begin
                        pair_q <= 6'd0;
                        if (!token_last_group) begin
                            group_q <= group_q + 1'b1;
                        end else begin
                            group_q <= 6'd0;
                            if (op_q == OP_EXP) begin
                                if (exp_only_q) begin
                                    active_q <= 1'b0;
                                    done <= 1'b1;
                                end else begin
                                    op_q <= OP_DW;
                                end
                            end else if (op_q == OP_DW) begin
                                op_q <= OP_PROJ;
                            end else begin
                                active_q <= 1'b0;
                                done <= 1'b1;
                            end
                        end
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
