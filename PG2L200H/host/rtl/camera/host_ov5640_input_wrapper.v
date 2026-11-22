`timescale 1ns/1ps
`default_nettype none
// OV5640 provider: continuous 640x320 display and 224x168 CNN streams.
// All read-side logic uses the independent 50 MHz system video clock.
module host_ov5640_input_wrapper #(
    parameter integer C_CAM_WIDTH=640,
    parameter integer C_CAM_HEIGHT=320,
    parameter integer C_CAM_WAIT_FRAMES=10,
    parameter integer C_FIFO_ADDR_WIDTH=11
)(
    input wire i_sys_clk, input wire i_sys_rst_n,
    input wire i_cam_pclk, input wire i_cam_rst_n,
    input wire i_cam_vsync, input wire i_cam_href, input wire [7:0] i_cam_data,
    output wire o_cam_ready, output wire o_cam_overflow,
    output wire o_display_valid, output wire [15:0] o_display_pixel,
    output wire [8:0] o_display_row, output wire [9:0] o_display_col,
    output wire o_display_frame_start, output wire o_display_line_start,
    output wire o_display_line_end, output wire o_display_frame_end,
    output wire [15:0] o_display_frame_id, output wire o_display_abort,
    output wire o_cnn_valid, output wire [15:0] o_cnn_pixel,
    output wire [8:0] o_cnn_row, output wire [9:0] o_cnn_col,
    output wire o_cnn_frame_start, output wire o_cnn_line_start,
    output wire o_cnn_line_end, output wire o_cnn_frame_end,
    output wire [15:0] o_cnn_frame_id, output wire o_cnn_abort
);
    ov5640_cdc_stream #(
        .C_WIDTH(C_CAM_WIDTH), .C_HEIGHT(C_CAM_HEIGHT),
        .C_WAIT_FRAMES(C_CAM_WAIT_FRAMES), .C_FIFO_ADDR_WIDTH(C_FIFO_ADDR_WIDTH)
    ) u_cam_stream (
        .i_cam_pclk(i_cam_pclk), .i_cam_rst_n(i_cam_rst_n),
        .i_cam_vsync(i_cam_vsync), .i_cam_href(i_cam_href), .i_cam_data(i_cam_data),
        .i_pixel_clk(i_sys_clk), .i_pixel_rst_n(i_sys_rst_n), .i_rd_ready(1'b1),
        .o_valid(o_display_valid), .o_pixel(o_display_pixel),
        .o_row(o_display_row), .o_col(o_display_col),
        .o_frame_start(o_display_frame_start), .o_line_start(o_display_line_start),
        .o_line_end(o_display_line_end), .o_frame_end(o_display_frame_end),
        .o_frame_id(o_display_frame_id), .o_overflow(o_display_abort), .o_cam_ready(o_cam_ready)
    );
    ov5640_cnn_resize #(
        .C_IN_WIDTH(C_CAM_WIDTH), .C_IN_HEIGHT(C_CAM_HEIGHT),
        .C_OUT_WIDTH(224), .C_OUT_HEIGHT(168)
    ) u_resize (
        .i_clk(i_sys_clk), .i_rst_n(i_sys_rst_n),
        .i_valid(o_display_valid), .i_pixel(o_display_pixel),
        .i_row(o_display_row), .i_col(o_display_col),
        .i_frame_start(o_display_frame_start), .i_line_start(o_display_line_start),
        .i_line_end(o_display_line_end), .i_frame_end(o_display_frame_end),
        .i_frame_id(o_display_frame_id), .i_abort(o_display_abort),
        .o_valid(o_cnn_valid), .o_pixel(o_cnn_pixel), .o_row(o_cnn_row), .o_col(o_cnn_col),
        .o_frame_start(o_cnn_frame_start), .o_line_start(o_cnn_line_start),
        .o_line_end(o_cnn_line_end), .o_frame_end(o_cnn_frame_end),
        .o_frame_id(o_cnn_frame_id), .o_abort(o_cnn_abort)
    );
    assign o_cam_overflow=o_display_abort;
endmodule
`default_nettype wire
