//////////////////////////////////////////////////////////////////////////////////
// Company: HRBUST_CSLAB_501
// Engineer: VinnLee
// Target: Pango Logos2 PG2L200H-6FBB484
// Module: ov5640_dvp_rgb565_unpack
// Description: OV5640 DVP 8-bit bus to one-pixel-per-cam_pclk RGB565 stream.
//              The reference OV5640 RGB565 configuration sends the high byte
//              first. Coordinates are carried with every emitted pixel so a
//              later CDC stage never has to infer frame boundaries.
//              C_WAIT_FRAMES keeps the original 38_ov5640_lcd bring-up rule
//              (wait ten complete frames after SCCB configuration) optional.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`default_nettype none

module ov5640_dvp_rgb565_unpack #(
    parameter integer C_WIDTH = 480,
    parameter integer C_HEIGHT = 272,
    parameter integer C_WAIT_FRAMES = 10
) (
    input  wire        i_cam_pclk,
    input  wire        i_cam_rst_n,
    input  wire        i_cam_vsync,
    input  wire        i_cam_href,
    input  wire [7:0]  i_cam_data,
    output reg         o_valid,
    output reg  [15:0] o_pixel,
    output reg  [8:0]  o_row,
    output reg  [9:0]  o_col,
    output reg         o_frame_start,
    output reg         o_line_start,
    output reg         o_line_end,
    output reg         o_frame_end,
    output reg  [15:0] o_frame_id,
    output reg         o_stream_ready
);
    reg vs_d;
    reg href_d;
    reg byte_phase_q;
    reg [7:0] high_byte_q;
    reg [9:0] x_q;
    reg [8:0] y_q;
    reg [3:0] warm_q;
    reg frame_ready_q;
    reg frame_start_pending_q;

    wire vs_rise = i_cam_vsync && !vs_d;
    wire href_fall = !i_cam_href && href_d;

    always @(posedge i_cam_pclk or negedge i_cam_rst_n) begin
        if (!i_cam_rst_n) begin
            vs_d <= 1'b0;
            href_d <= 1'b0;
            byte_phase_q <= 1'b0;
            high_byte_q <= 8'd0;
            x_q <= 10'd0;
            y_q <= 9'd0;
            warm_q <= 4'd0;
            frame_ready_q <= (C_WAIT_FRAMES == 0);
            frame_start_pending_q <= 1'b0;
            o_valid <= 1'b0;
            o_pixel <= 16'd0;
            o_row <= 9'd0;
            o_col <= 10'd0;
            o_frame_start <= 1'b0;
            o_line_start <= 1'b0;
            o_line_end <= 1'b0;
            o_frame_end <= 1'b0;
            o_frame_id <= 16'd0;
            o_stream_ready <= (C_WAIT_FRAMES == 0);
        end else begin
            vs_d <= i_cam_vsync;
            href_d <= i_cam_href;
            o_valid <= 1'b0;
            o_frame_start <= 1'b0;
            o_line_start <= 1'b0;
            o_line_end <= 1'b0;
            o_frame_end <= 1'b0;

            if (vs_rise) begin
                byte_phase_q <= 1'b0;
                x_q <= 10'd0;
                y_q <= 9'd0;
                frame_start_pending_q <= 1'b0;
                if (C_WAIT_FRAMES == 0) begin
                    frame_ready_q <= 1'b1;
                    frame_start_pending_q <= 1'b1;
                    o_frame_id <= o_frame_id + 1'b1;
                end else if (warm_q < C_WAIT_FRAMES) begin
                    warm_q <= warm_q + 1'b1;
                    if (warm_q == C_WAIT_FRAMES-1) begin
                        frame_ready_q <= 1'b1;
                        frame_start_pending_q <= 1'b1;
                        o_frame_id <= o_frame_id + 1'b1;
                    end
                end else begin
                    frame_ready_q <= 1'b1;
                    frame_start_pending_q <= 1'b1;
                    o_frame_id <= o_frame_id + 1'b1;
                end
            end

            if (href_fall) begin
                byte_phase_q <= 1'b0;
                x_q <= 10'd0;
                if (y_q < C_HEIGHT)
                    y_q <= y_q + 1'b1;
            end

            if (!i_cam_href) begin
                byte_phase_q <= 1'b0;
            end else begin
                if (!byte_phase_q) begin
                    high_byte_q <= i_cam_data;
                    byte_phase_q <= 1'b1;
                end else begin
                    byte_phase_q <= 1'b0;
                    if (frame_ready_q && (x_q < C_WIDTH) && (y_q < C_HEIGHT)) begin
                        o_valid <= 1'b1;
                        o_pixel <= {high_byte_q, i_cam_data};
                        o_row <= y_q;
                        o_col <= x_q;
                        o_frame_start <= frame_start_pending_q && (x_q == 0) && (y_q == 0);
                        o_line_start <= (x_q == 0);
                        o_line_end <= (x_q == C_WIDTH-1);
                        o_frame_end <= (x_q == C_WIDTH-1) && (y_q == C_HEIGHT-1);
                        frame_start_pending_q <= 1'b0;
                    end
                    if (x_q < C_WIDTH)
                        x_q <= x_q + 1'b1;
                end
            end
            o_stream_ready <= frame_ready_q;
        end
    end
endmodule
`default_nettype wire
