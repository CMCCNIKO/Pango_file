//////////////////////////////////////////////////////////////////////////////////
// OV5640 camera stream bridge: DVP unpack in cam_pclk domain, Gray-pointer
// asynchronous FIFO, and a pixel-clock-domain stream with explicit markers.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`default_nettype none

module ov5640_cdc_stream #(
    parameter integer C_WIDTH = 480,
    parameter integer C_HEIGHT = 272,
    parameter integer C_WAIT_FRAMES = 10,
    parameter integer C_FIFO_ADDR_WIDTH = 10
) (
    input  wire        i_cam_pclk,
    input  wire        i_cam_rst_n,
    input  wire        i_cam_vsync,
    input  wire        i_cam_href,
    input  wire [7:0]  i_cam_data,
    input  wire        i_pixel_clk,
    input  wire        i_pixel_rst_n,
    input  wire        i_rd_ready,
    output wire        o_valid,
    output wire [15:0] o_pixel,
    output wire [8:0]  o_row,
    output wire [9:0]  o_col,
    output wire        o_frame_start,
    output wire        o_line_start,
    output wire        o_line_end,
    output wire        o_frame_end,
    output wire [15:0] o_frame_id,
    output wire        o_overflow,
    output wire        o_cam_ready
);
    localparam integer C_DATA_WIDTH = 54;
    wire cam_valid;
    wire [15:0] cam_pixel;
    wire [8:0] cam_row;
    wire [9:0] cam_col;
    wire cam_frame_start, cam_line_start, cam_line_end, cam_frame_end;
    wire [15:0] cam_frame_id;
    wire cam_ready;
    wire [C_DATA_WIDTH-1:0] fifo_wr_data;
    wire [C_DATA_WIDTH-1:0] fifo_rd_data;
    wire fifo_full;
    wire fifo_valid;
    wire fifo_overflow;
    (* async_reg="true" *) reg [1:0] cam_reset_q;
    always @(posedge i_cam_pclk or negedge i_cam_rst_n)
        if(!i_cam_rst_n)cam_reset_q<=0;else cam_reset_q<={cam_reset_q[0],1'b1};

    ov5640_dvp_rgb565_unpack #(
        .C_WIDTH(C_WIDTH), .C_HEIGHT(C_HEIGHT), .C_WAIT_FRAMES(C_WAIT_FRAMES)
    ) u_unpack (
        .i_cam_pclk(i_cam_pclk), .i_cam_rst_n(cam_reset_q[1]),
        .i_cam_vsync(i_cam_vsync), .i_cam_href(i_cam_href), .i_cam_data(i_cam_data),
        .o_valid(cam_valid), .o_pixel(cam_pixel), .o_row(cam_row), .o_col(cam_col),
        .o_frame_start(cam_frame_start), .o_line_start(cam_line_start),
        .o_line_end(cam_line_end), .o_frame_end(cam_frame_end),
        .o_frame_id(cam_frame_id), .o_stream_ready(cam_ready)
    );

    (* async_reg="true" *) reg [1:0] ready_sync_q;
    always @(posedge i_pixel_clk or negedge i_pixel_rst_n)
        if(!i_pixel_rst_n)ready_sync_q<=0;else ready_sync_q<={ready_sync_q[0],cam_ready};
    assign o_cam_ready=ready_sync_q[1];
    host_video_stream_fifo #(.C_ADDR_WIDTH(C_FIFO_ADDR_WIDTH)) u_fifo (
        .i_wr_clk(i_cam_pclk),.i_wr_rst_n(i_cam_rst_n),
        .i_wr_valid(cam_valid),.i_wr_pixel(cam_pixel),.i_wr_row(cam_row),.i_wr_col(cam_col),
        .i_wr_frame_start(cam_frame_start),.i_wr_line_start(cam_line_start),
        .i_wr_line_end(cam_line_end),.i_wr_frame_end(cam_frame_end),.i_wr_frame_id(cam_frame_id),
        .i_wr_abort(1'b0),.o_wr_overflow(),
        .i_rd_clk(i_pixel_clk),.i_rd_rst_n(i_pixel_rst_n),.i_rd_ready(i_rd_ready),
        .o_rd_valid(o_valid),.o_rd_pixel(o_pixel),.o_rd_row(o_row),.o_rd_col(o_col),
        .o_rd_frame_start(o_frame_start),.o_rd_line_start(o_line_start),
        .o_rd_line_end(o_line_end),.o_rd_frame_end(o_frame_end),.o_rd_frame_id(o_frame_id),
        .o_rd_abort(o_overflow)
    );
endmodule
`default_nettype wire
