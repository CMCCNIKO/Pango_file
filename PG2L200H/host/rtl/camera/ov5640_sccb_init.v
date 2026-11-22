`timescale 1ns/1ps
`default_nettype none

// OV5640 SCCB power-up/configuration for the PG2L200H camera header.
// The register table is copied into this version from 38_ov5640_lcd and
// parameterized for 640x320 RGB565 DVP output.  Pixel capture remains in
// ov5640_cdc_stream, so this block owns only reset/power/SCCB.
module ov5640_sccb_init #(
    parameter integer C_CLOCK_HZ = 50000000,
    parameter integer C_I2C_HZ = 250000,
    parameter integer C_WIDTH = 640,
    parameter integer C_HEIGHT = 320,
    parameter integer C_TOTAL_H = 1800,
    parameter integer C_TOTAL_V = 1000,
    parameter integer C_SENSOR_Y_START = 316,
    parameter integer C_SENSOR_Y_END = 1627
)(
    input  wire i_clk,
    input  wire i_rst_n,
    output wire o_cam_rst_n,
    output wire o_cam_pwdn,
    output wire o_cam_scl,
    inout  wire o_cam_sda,
    output wire o_init_done
);
    wire i2c_exec;
    wire [23:0] i2c_data;
    wire i2c_rh_wl;
    wire i2c_done;
    wire [7:0] i2c_data_r;
    wire i2c_dri_clk;

    assign o_cam_pwdn = 1'b0;
    assign o_cam_rst_n = i_rst_n;

    ov5640_i2c_rgb565_cfg u_cfg (
        .clk(i2c_dri_clk), .rst_n(i_rst_n),
        .i2c_data_r(i2c_data_r), .i2c_done(i2c_done),
        .cmos_h_pixel(C_WIDTH[12:0]), .cmos_v_pixel(C_HEIGHT[12:0]),
        .total_h_pixel(C_TOTAL_H[12:0]), .total_v_pixel(C_TOTAL_V[12:0]),
        // Keep a centered 2624x1312 sensor window for the 2:1 output aspect.
        // The reference LCD table uses the same full horizontal sensor span.
        .y_addr_st(C_SENSOR_Y_START[12:0]), .y_addr_end(C_SENSOR_Y_END[12:0]),
        .i2c_exec(i2c_exec), .i2c_data(i2c_data),
        .i2c_rh_wl(i2c_rh_wl), .init_done(o_init_done)
    );

    ov5640_i2c_dri #(
        .SLAVE_ADDR(7'h3c), .CLK_FREQ(C_CLOCK_HZ), .I2C_FREQ(C_I2C_HZ)
    ) u_i2c (
        .clk(i_clk), .rst_n(i_rst_n),
        .i2c_exec(i2c_exec), .bit_ctrl(1'b1), .i2c_rh_wl(i2c_rh_wl),
        .i2c_addr(i2c_data[23:8]), .i2c_data_w(i2c_data[7:0]),
        .i2c_data_r(i2c_data_r), .i2c_done(i2c_done), .i2c_ack(),
        .scl(o_cam_scl), .sda(o_cam_sda), .dri_clk(i2c_dri_clk)
    );
endmodule
`default_nettype wire


