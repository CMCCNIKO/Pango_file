`timescale 1ns/1ps
`default_nettype none

// Shared source selector for display and CNN streams.  Each stream is read
// only after its target source exposes a SOF, so a switch cannot begin with a
// stale mid-frame token.  The two streams share one pending/active source and
// are both required to reach a target SOF before the request is acknowledged.
module video_dual_source_frame_mux #(
    parameter integer C_SWITCH_TIMEOUT = 32'd25_000_000
) (
    input wire i_clk, input wire i_rst_n,
    input wire i_select_cam,
    input wire i_hdmi_ready, input wire i_cam_ready,
    input wire i_hdmi_disp_valid, input wire [15:0] i_hdmi_disp_pixel,
    input wire [8:0] i_hdmi_disp_row, input wire [9:0] i_hdmi_disp_col,
    input wire i_hdmi_disp_frame_start, input wire i_hdmi_disp_line_start,
    input wire i_hdmi_disp_line_end, input wire i_hdmi_disp_frame_end,
    input wire [15:0] i_hdmi_disp_frame_id, input wire i_hdmi_disp_abort,
    output wire o_hdmi_disp_ready,
    input wire i_cam_disp_valid, input wire [15:0] i_cam_disp_pixel,
    input wire [8:0] i_cam_disp_row, input wire [9:0] i_cam_disp_col,
    input wire i_cam_disp_frame_start, input wire i_cam_disp_line_start,
    input wire i_cam_disp_line_end, input wire i_cam_disp_frame_end,
    input wire [15:0] i_cam_disp_frame_id, input wire i_cam_disp_abort,
    output wire o_cam_disp_ready,
    input wire i_hdmi_cnn_valid, input wire [15:0] i_hdmi_cnn_pixel,
    input wire [8:0] i_hdmi_cnn_row, input wire [9:0] i_hdmi_cnn_col,
    input wire i_hdmi_cnn_frame_start, input wire i_hdmi_cnn_line_start,
    input wire i_hdmi_cnn_line_end, input wire i_hdmi_cnn_frame_end,
    input wire [15:0] i_hdmi_cnn_frame_id, input wire i_hdmi_cnn_abort,
    output wire o_hdmi_cnn_ready,
    input wire i_cam_cnn_valid, input wire [15:0] i_cam_cnn_pixel,
    input wire [8:0] i_cam_cnn_row, input wire [9:0] i_cam_cnn_col,
    input wire i_cam_cnn_frame_start, input wire i_cam_cnn_line_start,
    input wire i_cam_cnn_line_end, input wire i_cam_cnn_frame_end,
    input wire [15:0] i_cam_cnn_frame_id, input wire i_cam_cnn_abort,
    output wire o_cam_cnn_ready,
    output reg o_active_cam, output reg o_switch_pulse,
    output reg o_switch_reject, output wire o_switch_pending,
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
    localparam [1:0] ST_RUN=2'd0, ST_WAIT_OLD=2'd1, ST_WAIT_TARGET=2'd2;
    reg [1:0] state_q;
    reg target_q;
    reg [31:0] timer_q;
    reg old_disp_done_q, old_cnn_done_q;
    reg disp_started_q, cnn_started_q;
    reg boundary_abort_q;
    wire timeout_hit=(C_SWITCH_TIMEOUT==0)||(timer_q>=C_SWITCH_TIMEOUT-1);
    wire data_cam=(state_q==ST_WAIT_TARGET)?target_q:o_active_cam;
    wire dv=data_cam?i_cam_disp_valid:i_hdmi_disp_valid;
    wire cv=data_cam?i_cam_cnn_valid:i_hdmi_cnn_valid;
    wire ds=data_cam?i_cam_disp_frame_start:i_hdmi_disp_frame_start;
    wire cs=data_cam?i_cam_cnn_frame_start:i_hdmi_cnn_frame_start;
    wire de=data_cam?i_cam_disp_frame_end:i_hdmi_disp_frame_end;
    wire ce=data_cam?i_cam_cnn_frame_end:i_hdmi_cnn_frame_end;
    wire da=data_cam?i_cam_disp_abort:i_hdmi_disp_abort;
    wire ca=data_cam?i_cam_cnn_abort:i_hdmi_cnn_abort;
    wire disp_gate=(disp_started_q||ds)&&!(state_q==ST_WAIT_OLD&&old_disp_done_q);
    wire cnn_gate=(cnn_started_q||cs)&&!(state_q==ST_WAIT_OLD&&old_cnn_done_q);
    // All source FIFOs continuously drain.  Source switching only gates output.
    assign o_hdmi_disp_ready=1'b1;
    assign o_cam_disp_ready=1'b1;
    assign o_hdmi_cnn_ready=1'b1;
    assign o_cam_cnn_ready=1'b1;
    assign o_switch_pending=(state_q!=ST_RUN);
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            state_q<=ST_RUN; target_q<=0; timer_q<=0; o_active_cam<=0;
            o_switch_pulse<=0; o_switch_reject<=0; boundary_abort_q<=0;
            old_disp_done_q<=0;old_cnn_done_q<=0;disp_started_q<=0;cnn_started_q<=0;
        end else begin
            o_switch_pulse<=0;o_switch_reject<=0;boundary_abort_q<=0;
            if(dv&&ds&&!boundary_abort_q)disp_started_q<=1;
            if(cv&&cs&&!boundary_abort_q)cnn_started_q<=1;
            if(da)disp_started_q<=0;
            if(ca)cnn_started_q<=0;
            // A rejected/expired UART request restores the desired level to
            // the actual source.  Cancel its latched target before a later
            // target frame could accidentally apply that obsolete request.
            if(state_q!=ST_RUN&&i_select_cam!=target_q)begin
                state_q<=ST_RUN;timer_q<=0;boundary_abort_q<=1;
                disp_started_q<=0;cnn_started_q<=0;
            end else case(state_q)
                ST_RUN:begin
                    timer_q<=0;
                    if(i_select_cam!=o_active_cam)begin
                        target_q<=i_select_cam;state_q<=ST_WAIT_OLD;
                        old_disp_done_q<=0;old_cnn_done_q<=0;
                    end
                end
                ST_WAIT_OLD:begin
                    if(dv&&de)old_disp_done_q<=1;
                    if(cv&&ce)old_cnn_done_q<=1;
                    if((old_disp_done_q||(dv&&de))&&(old_cnn_done_q||(cv&&ce))||timeout_hit)begin
                        state_q<=ST_WAIT_TARGET;timer_q<=0;
                        disp_started_q<=0;cnn_started_q<=0;boundary_abort_q<=1;
                    end else timer_q<=timer_q+1'b1;
                end
                ST_WAIT_TARGET:begin
                    if(!boundary_abort_q&&(disp_started_q||(dv&&ds))&&(cnn_started_q||(cv&&cs)))begin
                        o_active_cam<=target_q;state_q<=ST_RUN;timer_q<=0;o_switch_pulse<=1;
                    end else if(timeout_hit)begin
                        state_q<=ST_RUN;timer_q<=0;o_switch_reject<=1;boundary_abort_q<=1;
                        disp_started_q<=0;cnn_started_q<=0;
                    end else timer_q<=timer_q+1'b1;
                end
                default:begin state_q<=ST_RUN;disp_started_q<=0;cnn_started_q<=0;end
            endcase
        end
    end
    assign o_display_valid=dv&&disp_gate&&!boundary_abort_q&&!da;
    assign o_cnn_valid=cv&&cnn_gate&&!boundary_abort_q&&!ca;
    assign o_display_pixel=data_cam?i_cam_disp_pixel:i_hdmi_disp_pixel;
    assign o_display_row=data_cam?i_cam_disp_row:i_hdmi_disp_row;
    assign o_display_col=data_cam?i_cam_disp_col:i_hdmi_disp_col;
    assign o_display_frame_start=o_display_valid&&ds;
    assign o_display_line_start=o_display_valid&&(data_cam?i_cam_disp_line_start:i_hdmi_disp_line_start);
    assign o_display_line_end=o_display_valid&&(data_cam?i_cam_disp_line_end:i_hdmi_disp_line_end);
    assign o_display_frame_end=o_display_valid&&de;
    assign o_display_frame_id=data_cam?i_cam_disp_frame_id:i_hdmi_disp_frame_id;
    assign o_display_abort=boundary_abort_q||(da&&disp_started_q);
    assign o_cnn_pixel=data_cam?i_cam_cnn_pixel:i_hdmi_cnn_pixel;
    assign o_cnn_row=data_cam?i_cam_cnn_row:i_hdmi_cnn_row;
    assign o_cnn_col=data_cam?i_cam_cnn_col:i_hdmi_cnn_col;
    assign o_cnn_frame_start=o_cnn_valid&&cs;
    assign o_cnn_line_start=o_cnn_valid&&(data_cam?i_cam_cnn_line_start:i_hdmi_cnn_line_start);
    assign o_cnn_line_end=o_cnn_valid&&(data_cam?i_cam_cnn_line_end:i_hdmi_cnn_line_end);
    assign o_cnn_frame_end=o_cnn_valid&&ce;
    assign o_cnn_frame_id=data_cam?i_cam_cnn_frame_id:i_hdmi_cnn_frame_id;
    assign o_cnn_abort=boundary_abort_q||(ca&&cnn_started_q);
endmodule
`default_nettype wire
