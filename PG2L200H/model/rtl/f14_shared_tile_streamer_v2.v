//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
//
// Create Date: 未注明（沿用原文件）
// Update Date: 2026-10-30（本次文件说明整理）
// Design Name: 从完成的tile局部C8/P2 128位字中逐lane散射为全局NCHW字节写回，支持有效裁边和stride2紧凑D地址。
// Module Name: f14_shared_tile_streamer
// File Name: model/rtl/f14_shared_tile_streamer_v2.v
// Project Name: F14 lane reset partition / portable sources v2
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2022.2-SP6.4
// Header Version: 1.00（文件说明版本；RTL功能版本沿用原修订）
// Description:
// 从完成的tile局部C8/P2 128位字中逐lane散射为全局NCHW字节写回，支持有效裁边和stride2紧凑D地址。
// 每个128位源字仅读取一次，再以可背压字节流输出地址、bank和last；采用串行地址乘法避免存整图索引表。
//
// Dependencies: tile的B/D调试读口及全局字节写回口
//
// Revision:
// 2026-10-30 / Header 1.00 - 补充具体用途、依赖和版本说明；未改动功能逻辑。
// 原创建/功能修订及编译指令保留在下方历史内容中。
// Additional Comments:
// 沿用SOP npu_full43_d_streamer.v来源说明；仅支持字节特征，不支持最终signed16 head写回。
// 下方原始说明作为历史留档；当前用途和依赖以本文件新头及实际代码为准。
//////////////////////////////////////////////////////////////////////////////////

// F14 isolated adaptation, 2026-10-18.
// Source: unchanged SOP npu_full43_d_streamer.v; provenance in adjacent manifest.
// V2 additionally uses compact group*8 D addresses for stride2, as emitted
// by the reused numeric tile. Existing C8/P2 NCHW
// scatter, handshake, multipliers and four-bit channel-group counter retained.
// F14 final signed16 heads are NOT supported by this byte-stream module.
`timescale 1ns/1ps
`default_nettype none

// Stream one completed core tile back to byte-addressed NCHW storage. The
// local memories use C8xP2 words, so each 128-bit word is fetched once and
// its valid lanes are scattered to NCHW at one byte per accepted cycle.
module f14_shared_tile_streamer (
    input  wire          clk,
    input  wire          rst_n,

    input  wire          start,
    output wire          start_ready,
    input  wire          source_b,
    input  wire          stride2,
    input  wire [15:0]   tile_y,
    input  wire [15:0]   tile_x,
    input  wire          tile_shift_y,
    input  wire          tile_shift_x,
    input  wire [3:0]    tile_valid_h,
    input  wire [3:0]    tile_valid_w,
    input  wire [31:0]   output_base_addr,
    input  wire [15:0]   output_h,
    input  wire [15:0]   output_w,
    input  wire [15:0]   output_c,
    input  wire          transaction_last,

    output wire          b_debug_en,
    output wire [11:0]   b_debug_addr,
    input  wire          b_debug_valid,
    input  wire [127:0]  b_debug_data,

    output wire          d_debug_en,
    output wire [8:0]    d_debug_addr,
    input  wire          d_debug_valid,
    input  wire [127:0]  d_debug_data,

    output wire          out_valid,
    input  wire          out_ready,
    output reg  [31:0]   out_addr_byte,
    output reg           out_bank_a_valid,
    output reg           out_bank_b_valid,
    output reg  [7:0]    out_data,
    output reg           out_last,

    output wire          busy,
    output reg           done,
    output reg           protocol_error,
    output reg  [23:0]   emitted_bytes
);

    localparam [2:0] ST_IDLE    = 3'd0;
    localparam [2:0] ST_MUL     = 3'd1;
    localparam [2:0] ST_FIND    = 3'd2;
    localparam [2:0] ST_REQUEST = 3'd3;
    localparam [2:0] ST_WAIT    = 3'd4;
    localparam [2:0] ST_OUTPUT  = 3'd5;

    localparam [31:0] BANK_A_BASE = 32'h00030000;
    localparam [31:0] BANK_B_BASE = 32'h00070000;
    localparam [31:0] BANK_A_LIMIT = BANK_A_BASE + 32'd225792;
    localparam [31:0] BANK_B_LIMIT = BANK_B_BASE + 32'd150528;

    reg [2:0] state_q;
    reg source_b_q;
    reg stride2_q;
    reg [15:0] tile_y_q;
    reg [15:0] tile_x_q;
    reg tile_shift_y_q;
    reg tile_shift_x_q;
    reg [3:0] tile_valid_h_q;
    reg [3:0] tile_valid_w_q;
    reg [31:0] output_base_addr_q;
    reg [15:0] output_h_q;
    reg [15:0] output_w_q;
    reg [15:0] output_c_q;
    reg transaction_last_q;

    reg [4:0] mul_count_q;
    reg [31:0] mul_multiplicand_q;
    reg [15:0] mul_plane_multiplier_q;
    reg [15:0] mul_tile_multiplier_q;
    reg [31:0] mul_plane_acc_q;
    reg [31:0] mul_tile_acc_q;
    reg [31:0] plane_bytes_q;
    reg [31:0] tile_offset_q;

    reg [3:0] block_q;
    reg [5:0] pair_q;
    reg spatial_sel_q;
    reg [2:0] lane_channel_q;
    reg [127:0] word_data_q;
    reg [31:0] block_base_q;
    reg [23:0] expected_bytes_q;

    function [23:0] output_byte_count;
        input [15:0] channels;
        input [3:0] height;
        input [3:0] width;
        integer bit_index;
        reg [23:0] row_bytes;
        reg [23:0] total_bytes;
        begin
            row_bytes = 24'd0;
            for (bit_index = 0; bit_index < 4;
                 bit_index = bit_index + 1) begin
                if (width[bit_index])
                    row_bytes = row_bytes +
                        ({8'd0, channels} << bit_index);
            end
            total_bytes = 24'd0;
            for (bit_index = 0; bit_index < 4;
                 bit_index = bit_index + 1) begin
                if (height[bit_index])
                    total_bytes = total_bytes + (row_bytes << bit_index);
            end
            output_byte_count = total_bytes;
        end
    endfunction

    function [31:0] row_offset;
        input [3:0] row;
        input [15:0] width;
        integer bit_index;
        reg [31:0] value;
        begin
            value = 32'd0;
            for (bit_index = 0; bit_index < 4;
                 bit_index = bit_index + 1) begin
                if (row[bit_index])
                    value = value + ({16'd0, width} << bit_index);
            end
            row_offset = value;
        end
    endfunction

    function bank_a_address_valid;
        input [31:0] address;
        begin
            bank_a_address_valid =
                (address >= BANK_A_BASE) && (address < BANK_A_LIMIT);
        end
    endfunction

    function bank_b_address_valid;
        input [31:0] address;
        begin
            bank_b_address_valid =
                (address >= BANK_B_BASE) && (address < BANK_B_LIMIT);
        end
    endfunction

    wire [31:0] mul_plane_next = mul_plane_acc_q +
        (mul_plane_multiplier_q[0] ? mul_multiplicand_q : 32'd0);
    wire [31:0] mul_tile_next = mul_tile_acc_q +
        (mul_tile_multiplier_q[0] ? mul_multiplicand_q : 32'd0);

    wire [3:0] pair_local_y = stride2_q
        ? {1'd0, pair_q[3:1]} : {1'd0, pair_q[4:2]};
    wire [3:0] pair_local_x0 = stride2_q
        ? {2'd0, pair_q[0], 1'b0}
        : {1'd0, pair_q[1:0], 1'b0};
    wire [3:0] pair_local_x1 = pair_local_x0 + 1'b1;

    wire pair_y_valid = stride2_q
        ? (pair_local_y < tile_valid_h_q)
        : ((pair_local_y >= {3'd0, tile_shift_y_q}) &&
           (pair_local_y <
            ({3'd0, tile_shift_y_q} + tile_valid_h_q)));
    wire pair_x0_valid = stride2_q
        ? (pair_local_x0 < tile_valid_w_q)
        : ((pair_local_x0 >= {3'd0, tile_shift_x_q}) &&
           (pair_local_x0 <
            ({3'd0, tile_shift_x_q} + tile_valid_w_q)));
    wire pair_x1_valid = stride2_q
        ? (pair_local_x1 < tile_valid_w_q)
        : ((pair_local_x1 >= {3'd0, tile_shift_x_q}) &&
           (pair_local_x1 <
            ({3'd0, tile_shift_x_q} + tile_valid_w_q)));
    wire pair_spatial0_valid = pair_y_valid && pair_x0_valid;
    wire pair_spatial1_valid = pair_y_valid && pair_x1_valid;
    wire pair_has_output = pair_spatial0_valid || pair_spatial1_valid;

    wire [3:0] output_rel_y = stride2_q
        ? pair_local_y
        : pair_local_y - {3'd0, tile_shift_y_q};
    wire [3:0] output_rel_x0 = stride2_q
        ? pair_local_x0
        : pair_local_x0 - {3'd0, tile_shift_x_q};
    wire [3:0] output_rel_x1 = stride2_q
        ? pair_local_x1
        : pair_local_x1 - {3'd0, tile_shift_x_q};
    wire [31:0] spatial_offset0 = tile_offset_q +
        row_offset(output_rel_y, output_w_q) + {28'd0, output_rel_x0};
    wire [31:0] spatial_offset1 = tile_offset_q +
        row_offset(output_rel_y, output_w_q) + {28'd0, output_rel_x1};

    wire [15:0] absolute_channel =
        {9'd0, block_q, 3'b000} + {13'd0, lane_channel_q};
    wire next_lane_valid = (lane_channel_q != 3'd7) &&
        ((absolute_channel + 1'b1) < output_c_q);
    wire block_last = ({9'd0, block_q, 3'b000} + 16'd8) >= output_c_q;
    wire pair_last = stride2_q ? (pair_q == 6'd7) : (pair_q == 6'd31);
    wire [3:0] current_word_lane = {spatial_sel_q, lane_channel_q};
    wire [31:0] next_block_base = block_base_q + (plane_bytes_q << 3);

    wire [11:0] current_b_word_addr =
        ({8'd0, block_q} << 5) +
        ({8'd0, block_q} << 3) +
        {8'd0, block_q} + {6'd0, pair_q};
    wire [8:0] current_d_word_addr =
        (stride2_q ? {2'd0, block_q, 3'b000} : {block_q, 5'b00000}) + {3'd0, pair_q};
    wire selected_debug_valid = source_b_q
        ? b_debug_valid : d_debug_valid;
    wire [127:0] selected_debug_data = source_b_q
        ? b_debug_data : d_debug_data;

    wire [4:0] physical_side = stride2 ? 5'd4 : 5'd8;
    wire dimensions_valid =
        (tile_valid_h != 0) && (tile_valid_w != 0) &&
        (output_h != 0) && (output_w != 0) &&
        (output_c != 0) && (output_c <= 16'd72) &&
        ({1'b0, tile_valid_h} <= physical_side) &&
        ({1'b0, tile_valid_w} <= physical_side) &&
        ({1'b0, tile_y} + {13'd0, tile_valid_h} <= {1'b0, output_h}) &&
        ({1'b0, tile_x} + {13'd0, tile_valid_w} <= {1'b0, output_w}) &&
        (stride2 ||
         (({1'b0, tile_valid_h} + {4'd0, tile_shift_y}) <= 5'd8 &&
          ({1'b0, tile_valid_w} + {4'd0, tile_shift_x}) <= 5'd8));

    assign start_ready = state_q == ST_IDLE;
    assign busy = state_q != ST_IDLE;
    assign b_debug_en = (state_q == ST_REQUEST) && source_b_q;
    assign b_debug_addr = current_b_word_addr;
    assign d_debug_en = (state_q == ST_REQUEST) && !source_b_q;
    assign d_debug_addr = current_d_word_addr;
    assign out_valid = state_q == ST_OUTPUT;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q <= ST_IDLE;
            source_b_q <= 1'b0;
            stride2_q <= 1'b0;
            tile_y_q <= 16'd0;
            tile_x_q <= 16'd0;
            tile_shift_y_q <= 1'b0;
            tile_shift_x_q <= 1'b0;
            tile_valid_h_q <= 4'd0;
            tile_valid_w_q <= 4'd0;
            output_base_addr_q <= 32'd0;
            output_h_q <= 16'd0;
            output_w_q <= 16'd0;
            output_c_q <= 16'd0;
            transaction_last_q <= 1'b0;
            mul_count_q <= 5'd0;
            mul_multiplicand_q <= 32'd0;
            mul_plane_multiplier_q <= 16'd0;
            mul_tile_multiplier_q <= 16'd0;
            mul_plane_acc_q <= 32'd0;
            mul_tile_acc_q <= 32'd0;
            plane_bytes_q <= 32'd0;
            tile_offset_q <= 32'd0;
            block_q <= 4'd0;
            pair_q <= 6'd0;
            spatial_sel_q <= 1'b0;
            lane_channel_q <= 3'd0;
            word_data_q <= 128'd0;
            block_base_q <= 32'd0;
            expected_bytes_q <= 24'd0;
            out_addr_byte <= 32'd0;
            out_bank_a_valid <= 1'b0;
            out_bank_b_valid <= 1'b0;
            out_data <= 8'd0;
            out_last <= 1'b0;
            done <= 1'b0;
            protocol_error <= 1'b0;
            emitted_bytes <= 24'd0;
        end else begin
            done <= 1'b0;
            case (state_q)
                ST_IDLE: begin
                    if (start) begin
                        if (!dimensions_valid) begin
                            protocol_error <= 1'b1;
                            done <= 1'b1;
                        end else begin
                            source_b_q <= source_b;
                            stride2_q <= stride2;
                            tile_y_q <= tile_y;
                            tile_x_q <= tile_x;
                            tile_shift_y_q <= tile_shift_y;
                            tile_shift_x_q <= tile_shift_x;
                            tile_valid_h_q <= tile_valid_h;
                            tile_valid_w_q <= tile_valid_w;
                            output_base_addr_q <= output_base_addr;
                            output_h_q <= output_h;
                            output_w_q <= output_w;
                            output_c_q <= output_c;
                            transaction_last_q <= transaction_last;
                            mul_count_q <= 5'd0;
                            mul_multiplicand_q <= {16'd0, output_w};
                            mul_plane_multiplier_q <= output_h;
                            mul_tile_multiplier_q <= tile_y;
                            mul_plane_acc_q <= 32'd0;
                            mul_tile_acc_q <= 32'd0;
                            block_q <= 4'd0;
                            pair_q <= 6'd0;
                            block_base_q <= output_base_addr;
                            expected_bytes_q <= output_byte_count(
                                output_c, tile_valid_h, tile_valid_w);
                            emitted_bytes <= 24'd0;
                            out_last <= 1'b0;
                            state_q <= ST_MUL;
                        end
                    end
                end

                ST_MUL: begin
                    mul_plane_acc_q <= mul_plane_next;
                    mul_tile_acc_q <= mul_tile_next;
                    mul_multiplicand_q <= mul_multiplicand_q << 1;
                    mul_plane_multiplier_q <= mul_plane_multiplier_q >> 1;
                    mul_tile_multiplier_q <= mul_tile_multiplier_q >> 1;
                    if (mul_count_q == 5'd15) begin
                        plane_bytes_q <= mul_plane_next;
                        tile_offset_q <= mul_tile_next + {16'd0, tile_x_q};
                        state_q <= ST_FIND;
                    end else begin
                        mul_count_q <= mul_count_q + 1'b1;
                    end
                end

                ST_FIND: begin
                    if (pair_has_output) begin
                        state_q <= ST_REQUEST;
                    end else if (!pair_last) begin
                        pair_q <= pair_q + 1'b1;
                    end else if (!block_last) begin
                        block_q <= block_q + 1'b1;
                        pair_q <= 6'd0;
                        block_base_q <= next_block_base;
                    end else begin
                        protocol_error <= 1'b1;
                        done <= 1'b1;
                        state_q <= ST_IDLE;
                    end
                end

                ST_REQUEST: state_q <= ST_WAIT;

                ST_WAIT: begin
                    if (selected_debug_valid) begin
                        word_data_q <= selected_debug_data;
                        lane_channel_q <= 3'd0;
                        if (pair_spatial0_valid) begin
                            spatial_sel_q <= 1'b0;
                            out_addr_byte <= block_base_q + spatial_offset0;
                            out_bank_a_valid <= bank_a_address_valid(
                                block_base_q + spatial_offset0);
                            out_bank_b_valid <= bank_b_address_valid(
                                block_base_q + spatial_offset0);
                            out_data <= selected_debug_data[7:0];
                        end else begin
                            spatial_sel_q <= 1'b1;
                            out_addr_byte <= block_base_q + spatial_offset1;
                            out_bank_a_valid <= bank_a_address_valid(
                                block_base_q + spatial_offset1);
                            out_bank_b_valid <= bank_b_address_valid(
                                block_base_q + spatial_offset1);
                            out_data <= selected_debug_data[71:64];
                        end
                        out_last <= transaction_last_q &&
                            (emitted_bytes + 1'b1 == expected_bytes_q);
                        state_q <= ST_OUTPUT;
                    end
                end

                ST_OUTPUT: begin
                    if (out_valid && out_ready) begin
                        emitted_bytes <= emitted_bytes + 1'b1;
                        if (emitted_bytes + 1'b1 == expected_bytes_q) begin
                            done <= 1'b1;
                            state_q <= ST_IDLE;
                        end else if (next_lane_valid) begin
                            lane_channel_q <= lane_channel_q + 1'b1;
                            out_addr_byte <= out_addr_byte + plane_bytes_q;
                            out_data <= word_data_q[
                                ({spatial_sel_q, lane_channel_q} + 1'b1)*8
                                +: 8
                            ];
                            out_last <= transaction_last_q &&
                                (emitted_bytes + 2'd2 == expected_bytes_q);
                        end else if (!spatial_sel_q &&
                                     pair_spatial1_valid) begin
                            spatial_sel_q <= 1'b1;
                            lane_channel_q <= 3'd0;
                            out_addr_byte <= block_base_q + spatial_offset1;
                            out_bank_a_valid <= bank_a_address_valid(
                                block_base_q + spatial_offset1);
                            out_bank_b_valid <= bank_b_address_valid(
                                block_base_q + spatial_offset1);
                            out_data <= word_data_q[71:64];
                            out_last <= transaction_last_q &&
                                (emitted_bytes + 2'd2 == expected_bytes_q);
                        end else begin
                            out_last <= 1'b0;
                            if (!pair_last) begin
                                pair_q <= pair_q + 1'b1;
                            end else begin
                                block_q <= block_q + 1'b1;
                                pair_q <= 6'd0;
                                block_base_q <= next_block_base;
                            end
                            state_q <= ST_FIND;
                        end
                    end
                end

                default: begin
                    state_q <= ST_IDLE;
                    protocol_error <= 1'b1;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
