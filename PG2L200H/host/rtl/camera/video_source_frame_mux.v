//////////////////////////////////////////////////////////////////////////////////
// Frame-safe runtime source selector.  i_select_cam may change at any time;
// active source changes at the current source frame end, or at the explicit
// force/timeout escape when the old source has stopped producing frames.
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`default_nettype none

module video_source_frame_mux #(
    parameter integer C_SWITCH_TIMEOUT = 32'd25_000_000
) (
    input  wire        i_clk,
    input  wire        i_rst_n,
    input  wire        i_select_cam,
    input  wire        i_force_switch,
    input  wire        i_hdmi_valid,
    input  wire [15:0] i_hdmi_pixel,
    input  wire [8:0]  i_hdmi_row,
    input  wire [9:0]  i_hdmi_col,
    input  wire        i_hdmi_frame_start,
    input  wire        i_hdmi_line_start,
    input  wire        i_hdmi_line_end,
    input  wire        i_hdmi_frame_end,
    input  wire [15:0] i_hdmi_frame_id,
    input  wire        i_hdmi_abort,
    input  wire        i_cam_valid,
    input  wire [15:0] i_cam_pixel,
    input  wire [8:0]  i_cam_row,
    input  wire [9:0]  i_cam_col,
    input  wire        i_cam_frame_start,
    input  wire        i_cam_line_start,
    input  wire        i_cam_line_end,
    input  wire        i_cam_frame_end,
    input  wire [15:0] i_cam_frame_id,
    input  wire        i_cam_abort,
    output reg         o_select_cam,
    output reg         o_switch_pulse,
    output wire        o_valid,
    output wire [15:0] o_pixel,
    output wire [8:0]  o_row,
    output wire [9:0]  o_col,
    output wire        o_frame_start,
    output wire        o_line_start,
    output wire        o_line_end,
    output wire        o_frame_end,
    output wire [15:0] o_frame_id,
    output wire        o_abort,
    output wire        o_pending
);
    reg pending_q;
    reg [31:0] wait_q;
    wire active_frame_end = o_select_cam ? i_cam_frame_end : i_hdmi_frame_end;
    wire timeout_hit = (C_SWITCH_TIMEOUT == 0) || (wait_q >= C_SWITCH_TIMEOUT-1);
    wire switch_now = pending_q && (active_frame_end || i_force_switch || timeout_hit);

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_select_cam <= 1'b0;
            o_switch_pulse <= 1'b0;
            pending_q <= 1'b0;
            wait_q <= 32'd0;
        end else begin
            o_switch_pulse <= 1'b0;
            if (i_select_cam != o_select_cam)
                pending_q <= 1'b1;
            if (pending_q) begin
                if (switch_now) begin
                    o_select_cam <= i_select_cam;
                    pending_q <= 1'b0;
                    wait_q <= 32'd0;
                    o_switch_pulse <= 1'b1;
                end else if (wait_q != 32'hffff_ffff) begin
                    wait_q <= wait_q + 1'b1;
                end
            end else begin
                wait_q <= 32'd0;
            end
        end
    end

    assign o_pending = pending_q;
    assign o_valid = o_select_cam ? i_cam_valid : i_hdmi_valid;
    assign o_pixel = o_select_cam ? i_cam_pixel : i_hdmi_pixel;
    assign o_row = o_select_cam ? i_cam_row : i_hdmi_row;
    assign o_col = o_select_cam ? i_cam_col : i_hdmi_col;
    assign o_frame_start = o_select_cam ? i_cam_frame_start : i_hdmi_frame_start;
    assign o_line_start = o_select_cam ? i_cam_line_start : i_hdmi_line_start;
    assign o_line_end = o_select_cam ? i_cam_line_end : i_hdmi_line_end;
    assign o_frame_end = o_select_cam ? i_cam_frame_end : i_hdmi_frame_end;
    assign o_frame_id = o_select_cam ? i_cam_frame_id : i_hdmi_frame_id;
    // A timeout/forced switch may occur without an old-source frame_end;
    // surface the switch pulse as a frame abort so downstream CNN state
    // cannot consume a mixed-source frame.
    assign o_abort = (o_select_cam ? i_cam_abort : i_hdmi_abort) | o_switch_pulse;
endmodule
`default_nettype wire
