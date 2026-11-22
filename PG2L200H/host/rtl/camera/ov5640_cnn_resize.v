//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Target: Pango Logos2 PG2L200H-6FBB484
// Module: ov5640_cnn_resize
// Description: Streaming nearest-neighbor adapter from the default OV5640
// 480x272 RGB565 frame to the F14 224x168 CNN contract. The first input row
// mapped to each output row is selected; repeated horizontal source samples
// are collapsed so each output row emits exactly 224 pixels.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`default_nettype none

module ov5640_cnn_resize #(
    parameter integer C_IN_WIDTH = 480,
    parameter integer C_IN_HEIGHT = 272,
    parameter integer C_OUT_WIDTH = 224,
    parameter integer C_OUT_HEIGHT = 168
) (
    input wire i_clk,
    input wire i_rst_n,
    input wire i_valid,
    input wire [15:0] i_pixel,
    input wire [8:0] i_row,
    input wire [9:0] i_col,
    input wire i_frame_start,
    input wire i_line_start,
    input wire i_line_end,
    input wire i_frame_end,
    input wire [15:0] i_frame_id,
    input wire i_abort,
    output reg o_valid,
    output reg [15:0] o_pixel,
    output reg [8:0] o_row,
    output reg [9:0] o_col,
    output reg o_frame_start,
    output reg o_line_start,
    output reg o_line_end,
    output reg o_frame_end,
    output reg [15:0] o_frame_id,
    output reg o_abort
);
    reg [8:0] last_in_row_q;
    reg [8:0] last_out_row_q;
    reg [9:0] last_out_col_q;
    reg selected_row_q;
    reg frame_seen_q;
    reg [8:0] target_row;
    reg [9:0] target_col;
    reg new_output_row;
    reg current_row_selected;
    reg current_emit;
    reg current_last_output;

    always @* begin
        target_row = (i_row * C_OUT_HEIGHT) / C_IN_HEIGHT;
        target_col = (i_col * C_OUT_WIDTH) / C_IN_WIDTH;
        new_output_row = ((target_row != last_out_row_q) || i_frame_start) && (i_col == 0);
        current_row_selected = (i_col == 0) ? ((target_row != last_out_row_q) || i_frame_start) : selected_row_q;
        current_emit = i_valid && current_row_selected &&
                       ((i_col == 0) ? (target_col == 0) : (target_col != last_out_col_q));
        current_last_output = current_emit && (target_row == C_OUT_HEIGHT-1) &&
                              (target_col == C_OUT_WIDTH-1);
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            last_in_row_q <= 9'h1ff;
            last_out_row_q <= 9'h1ff;
            last_out_col_q <= 10'h3ff;
            selected_row_q <= 1'b0;
            frame_seen_q <= 1'b0;
            o_valid <= 1'b0;
            o_pixel <= 16'd0;
            o_row <= 9'd0;
            o_col <= 10'd0;
            o_frame_start <= 1'b0;
            o_line_start <= 1'b0;
            o_line_end <= 1'b0;
            o_frame_end <= 1'b0;
            o_frame_id <= 16'd0;
            o_abort <= 1'b0;
        end else begin
            o_valid <= 1'b0;
            o_frame_start <= 1'b0;
            o_line_start <= 1'b0;
            o_line_end <= 1'b0;
            o_frame_end <= 1'b0;
            o_abort <= i_abort;

            if (i_frame_start) begin
                last_in_row_q <= 9'h1ff;
                last_out_row_q <= 9'h1ff;
                last_out_col_q <= 10'h3ff;
                selected_row_q <= 1'b0;
                frame_seen_q <= 1'b1;
            end

            if (i_valid && (frame_seen_q || i_frame_start) && !i_abort) begin
                if (i_col == 0) begin
                    last_in_row_q <= i_row;
                    selected_row_q <= (target_row != last_out_row_q) || i_frame_start;
                    last_out_col_q <= 10'h3ff;
                    if ((target_row != last_out_row_q) || i_frame_start) begin
                        last_out_row_q <= target_row;
                    end
                    if (((target_row != last_out_row_q) || i_frame_start) && (target_col == 0)) begin
                        o_valid <= 1'b1;
                        o_pixel <= i_pixel;
                        o_row <= target_row;
                        o_col <= target_col;
                        o_frame_id <= i_frame_id;
                        o_frame_start <= (target_row == 0);
                        o_line_start <= 1'b1;
                        o_line_end <= (target_col == C_OUT_WIDTH-1);
                        o_frame_end <= (target_row == C_OUT_HEIGHT-1) && (target_col == C_OUT_WIDTH-1);
                        last_out_col_q <= target_col;
                    end
                end else if (selected_row_q && (target_col != last_out_col_q)) begin
                    o_valid <= 1'b1;
                    o_pixel <= i_pixel;
                    o_row <= target_row;
                    o_col <= target_col;
                    o_frame_id <= i_frame_id;
                    o_frame_start <= (target_row == 0) && (target_col == 0);
                    o_line_start <= (target_col == 0);
                    o_line_end <= (target_col == C_OUT_WIDTH-1);
                    o_frame_end <= (target_row == C_OUT_HEIGHT-1) && (target_col == C_OUT_WIDTH-1);
                    last_out_col_q <= target_col;
                end
            end

            if (i_frame_end) begin
                if (!current_last_output &&
                    (last_out_row_q != C_OUT_HEIGHT-1 || last_out_col_q != C_OUT_WIDTH-1))
                    o_abort <= 1'b1;
                frame_seen_q <= 1'b0;
                selected_row_q <= 1'b0;
            end
            if(i_abort)begin
                frame_seen_q<=1'b0;selected_row_q<=1'b0;
                o_valid<=1'b0;o_frame_start<=1'b0;o_line_start<=1'b0;
                o_line_end<=1'b0;o_frame_end<=1'b0;
            end
        end
    end
endmodule
`default_nettype wire
