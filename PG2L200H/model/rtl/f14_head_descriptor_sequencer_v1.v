//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 2026-10-18
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 顺序发出描述符40..51的检测头配置，依次运行 cls、vertical、offset 各四个算子。
// Module Name: f14_head_descriptor_sequencer_v1
// File Name: model/rtl/f14_head_descriptor_sequencer_v1.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 顺序发出描述符40..51的检测头配置，依次运行 cls、vertical、offset 各四个算子。
// 产生输入/输出bank、张量/卷积几何及权重地址，等待当前算子完成后再推进，不实例化计算或特征存储。
//
// Dependencies: 单个共享 generic head operation 的 ready/done 接口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 每个分支从 aligned adapter 开始，经 scratch_A/B 轮换到最终输出；P_LAST_SLOT决定终止槽。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Create Date: 2026-10-18
// Design Name: F14 head descriptor sequencer
// Module Name: f14_head_descriptor_sequencer_v1
// Project Name: QuantLaneNet F14 PG2L200H bring-up
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Description: Serial control shell for descriptors 40..51.  It emits one
//              descriptor configuration at a time to a single shared generic
//              window/MAC/PQ backend.  No arithmetic or feature storage is
//              instantiated here.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
`default_nettype none

module f14_head_descriptor_sequencer_v1 #(
    parameter [3:0] P_LAST_SLOT = 4'd11
) (
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_clear,
    input  wire        i_start,
    input  wire        i_op_ready,
    input  wire        i_op_done,
    output wire        o_start_ready,
    output wire        o_busy,
    output reg         o_done,
    output wire        o_error,
    output wire        o_op_valid,
    output wire [3:0]  o_descriptor_slot,
    output wire [1:0]  o_stage_kind,
    output wire [1:0]  o_input_bank,
    output wire [1:0]  o_output_bank,
    output wire [7:0]  o_input_h,
    output wire [7:0]  o_input_w,
    output wire [7:0]  o_input_c,
    output wire [7:0]  o_output_h,
    output wire [7:0]  o_output_w,
    output wire [7:0]  o_output_c,
    output wire [3:0]  o_kernel_h,
    output wire [3:0]  o_kernel_w,
    output wire [3:0]  o_stride_y,
    output wire [3:0]  o_stride_x,
    output wire [3:0]  o_pad_y,
    output wire [3:0]  o_pad_x,
    output wire [16:0] o_weight_base_word32,
    output wire [16:0] o_bias_base_word32,
    output wire [16:0] o_qscale_base_word32
);
    localparam [1:0] ST_IDLE = 2'd0;
    localparam [1:0] ST_READY = 2'd1;
    localparam [1:0] ST_WAIT = 2'd2;

    reg [1:0] state_q;
    reg [3:0] slot_q;
    reg error_q;
    wire op_fire = o_op_valid && i_op_ready;
    wire done_fire = (state_q == ST_WAIT) && i_op_done;

    assign o_start_ready = (state_q == ST_IDLE) && i_rst_n && !i_clear;
    assign o_busy = (state_q != ST_IDLE) && i_rst_n;
    assign o_op_valid = (state_q == ST_READY) && i_rst_n && !i_clear;
    assign o_descriptor_slot = slot_q;
    assign o_error = error_q;

    // stage_kind: 0=cls, 1=vertical, 2=offset.
    assign o_stage_kind = (slot_q < 4) ? 2'd0 :
                          (slot_q < 8) ? 2'd1 : 2'd2;

    // bank codes: 0=aligned_adapter, 1=scratch_A, 2=scratch_B,
    // 3=final output writer.  Each branch starts from the aligned adapter.
    assign o_input_bank =
        (slot_q == 0 || slot_q == 4 || slot_q == 8) ? 2'd0 :
        ((slot_q == 1 || slot_q == 5 || slot_q == 9) ? 2'd1 :
         ((slot_q == 2 || slot_q == 6 || slot_q == 10) ? 2'd2 : 2'd1));
    assign o_output_bank =
        (slot_q == 0 || slot_q == 4 || slot_q == 8) ? 2'd1 :
        ((slot_q == 1 || slot_q == 5 || slot_q == 9) ? 2'd2 :
         ((slot_q == 2 || slot_q == 6 || slot_q == 10) ? 2'd1 : 2'd3));

    // The following table is generated from f14_head_golden_manifest_v1.
    // All dimensions are unsigned and fit the runtime descriptor contract.
    assign o_input_h = 8'd21;
    assign o_output_h = 8'd21;
    assign o_input_w =
        (slot_q >= 4 && slot_q <= 7) ?
        ((slot_q == 4) ? 8'd28 : (slot_q == 5) ? 8'd14 : (slot_q == 6) ? 8'd7 : 8'd3) : 8'd28;
    assign o_output_w =
        (slot_q >= 4 && slot_q <= 7) ?
        ((slot_q == 4) ? 8'd14 : (slot_q == 5) ? 8'd7 : (slot_q == 6) ? 8'd3 : 8'd1) : 8'd28;
    assign o_input_c =
        (slot_q == 0 || slot_q == 4 || slot_q == 8) ? 8'd64 :
        (slot_q == 1 || slot_q == 5 || slot_q == 9) ? 8'd32 :
        (slot_q == 2 || slot_q == 6 || slot_q == 10) ? 8'd16 : 8'd8;
    assign o_output_c =
        (slot_q == 0 || slot_q == 4 || slot_q == 8) ? 8'd32 :
        (slot_q == 1 || slot_q == 5 || slot_q == 9) ? 8'd16 :
        (slot_q == 2 || slot_q == 6 || slot_q == 10) ? 8'd8 : 8'd4;
    assign o_kernel_h = 4'd3;
    // vertical.0..2 are 3x2/stride-2 reductions.  The terminal vertical
    // descriptor (47) is a 3x3/stride-1 projection to W=1, matching the
    // software contract and its ROM word count.
    assign o_kernel_w = (slot_q >= 4 && slot_q <= 6) ? 4'd2 : 4'd3;
    assign o_stride_y = 4'd1;
    assign o_stride_x = (slot_q >= 4 && slot_q <= 6) ? 4'd2 : 4'd1;
    assign o_pad_y = 4'd1;
    assign o_pad_x = (slot_q >= 4 && slot_q <= 7) ? 4'd0 : 4'd1;

    // Unified runtime ROM word addresses.  Weight, bias and qscale sections
    // are kept separate so the backend can use one ROM port with a small
    // request arbiter.
    function [16:0] weight_base;
        input [3:0] s;
        begin
            case (s)
                4'd0: weight_base = 17'd92416;
                4'd1: weight_base = 17'd97088;
                4'd2: weight_base = 17'd98272;
                4'd3: weight_base = 17'd98576;
                4'd4: weight_base = 17'd98736;
                4'd5: weight_base = 17'd101872;
                4'd6: weight_base = 17'd102672;
                4'd7: weight_base = 17'd102880;
                4'd8: weight_base = 17'd103040;
                4'd9: weight_base = 17'd107712;
                4'd10: weight_base = 17'd108896;
                default: weight_base = 17'd109200;
            endcase
        end
    endfunction
    function [16:0] bias_base;
        input [3:0] s;
        begin
            case (s)
                4'd0: bias_base = 17'd97024;
                4'd1: bias_base = 17'd98240;
                4'd2: bias_base = 17'd98560;
                4'd3: bias_base = 17'd98720;
                4'd4: bias_base = 17'd101808;
                4'd5: bias_base = 17'd102640;
                4'd6: bias_base = 17'd102864;
                4'd7: bias_base = 17'd103024;
                4'd8: bias_base = 17'd107648;
                4'd9: bias_base = 17'd108864;
                4'd10: bias_base = 17'd109184;
                default: bias_base = 17'd109344;
            endcase
        end
    endfunction
    function [16:0] qscale_base;
        input [3:0] s;
        begin
            case (s)
                4'd0: qscale_base = 17'd97056;
                4'd1: qscale_base = 17'd98256;
                4'd2: qscale_base = 17'd98568;
                4'd3: qscale_base = 17'd98728;
                4'd4: qscale_base = 17'd101840;
                4'd5: qscale_base = 17'd102656;
                4'd6: qscale_base = 17'd102872;
                4'd7: qscale_base = 17'd103032;
                4'd8: qscale_base = 17'd107680;
                4'd9: qscale_base = 17'd108880;
                4'd10: qscale_base = 17'd109192;
                default: qscale_base = 17'd109352;
            endcase
        end
    endfunction
    assign o_weight_base_word32 = weight_base(slot_q);
    assign o_bias_base_word32 = bias_base(slot_q);
    assign o_qscale_base_word32 = qscale_base(slot_q);

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state_q <= ST_IDLE;
            slot_q <= 4'd0;
            error_q <= 1'b0;
            o_done <= 1'b0;
        end else if (i_clear) begin
            state_q <= ST_IDLE;
            slot_q <= 4'd0;
            error_q <= 1'b0;
            o_done <= 1'b0;
        end else begin
            o_done <= 1'b0;
            case (state_q)
                ST_IDLE: begin
                    if (i_start) begin
                        slot_q <= 4'd0;
                        error_q <= 1'b0;
                        state_q <= ST_READY;
                    end
                end
                ST_READY: begin
                    if (op_fire)
                        state_q <= ST_WAIT;
                end
                ST_WAIT: begin
                    if (i_op_done) begin
                        if (slot_q == P_LAST_SLOT) begin
                            state_q <= ST_IDLE;
                            o_done <= 1'b1;
                        end else begin
                            slot_q <= slot_q + 1'b1;
                            state_q <= ST_READY;
                        end
                    end
                end
                default: begin
                    state_q <= ST_IDLE;
                    error_q <= 1'b1;
                end
            endcase
        end
    end
endmodule

`default_nettype wire
