//////////////////////////////////////////////////////////////////////////////////
// Company: HRB_CSLAB_501
// Engineer: VinnLee
// Update Date: 2026-11-15
// Module Name: host_hdmi_to_fiber_top
// File Name: host/rtl/host_hdmi_to_fiber_top.v
// Project Name: PG2L200H HDMI/OV5640 input and CNN/HSST output V011
// Target Devices: Pango Logos2 PG2L200H-6FBB484
// Tool Versions: PDS 2025.2-ads
// Description:
// HDMI（ADV7611并行视频）和OV5640输入在固定50MHz视频域选择，
// 生成640x320显示流与224x168 CNN流，经原CNN/HSST链路发往PG2L100H。
// PG2L100H通过原UART控制输入源，两条目标流实际进入后返回Applied ACK。
// 本版移除PG2L200H本地HDMI输出，不实例化TMDS PLL、编码或串行发送器。
// ADV7611初始化、输入像素时钟、独立F14时钟和HSST时钟保持各自原路径。
// Dependencies: fiber/host_build_config.vh, HDMI input/SCCB/UART/video RTL,
//               F14 clock/CNN, official PG2L200H HSST/RAM/FIFO IP.
// Revision: 2026-11-15 V011 - PDS2025统一工程，取消本地HDMI输出。
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns/1ps
`include "fiber/host_build_config.vh"
// PG2L200H HOST: HDMI/OV5640 input -> CNN -> HSST TX to PG2L100H.
// No local HDMI output is instantiated. The active TX mode (IDLE_ONLY / PATTERN /
// HDMI) is decided by exactly one HOST_TX_MODE_* define in
// rtl/fiber/host_mode_select.vh (the single source of truth, validated by
// scripts/build_host_modes.py). No mode define is hardcoded in this file.
module host_hdmi_to_fiber_top (
    input wire sys_clk /* synthesis PAP_MARK_DEBUG="<0/c0/0>" */,
    input wire rst_n,
    input wire hdmi_clk_in,
    input wire hdmi_de_in,
    input wire hdmi_vs_in,
    input wire hdmi_hs_in,
    input wire [23:0] hdmi_data_in,
    // OV5640 DVP input. The local PG2L200H ov5640_sccb_init instance below
    // drives reset/power/SCCB; verify the camera-header wiring before board use.
    input wire cam_pclk,
    input wire cam_vsync,
    input wire cam_href,
    input wire [7:0] cam_data,
    output wire cam_rst_n,
    output wire cam_pwdn,
    inout wire cam_scl,
    inout wire cam_sda,
    // UART command link from the PG2L100H AV bridge.
    input wire i_uart_rx,
    output wire o_uart_tx,
    input wire i_p_refckn_1,
    input wire i_p_refckp_1,
    output wire hdmi_rst_n_out,
    output wire hdmi_scl,
    inout wire hdmi_sda,
    output wire led,
    output wire hsst_txp,
    output wire hsst_txn,
    output wire tx_disable
);
    (* async_reg = "true" *) reg [1:0] meta_pixel_rst_q,meta_tx_rst_q;
    wire pixel_clk, video_hs, video_vs, video_de;
    wire [23:0] video_rgb;
    wire adv_init_done /* synthesis PAP_MARK_DEBUG="true" */;
    wire adv_reset_n;
    reg pixel_heartbeat;
    wire mode_valid /* synthesis PAP_MARK_DEBUG="true" */, frame_start, frame_done, frame_error;
    wire pix_valid, row_start, row_commit, norm_done, frame_abort, frame_selected;
    wire [15:0] pix565, frame_id;
    wire [8:0] row_index;
    wire [9:0] col_index;
    wire cnn_valid,cnn_abort;
    wire [15:0] cnn_pixel,cnn_id;
    wire [8:0] cnn_row;
    wire [9:0] cnn_col;
    wire cnn_frame_start,cnn_line_start,cnn_line_end,cnn_frame_end;
    wire selected_cnn_valid,selected_cnn_abort;
    wire [15:0] selected_cnn_pixel,selected_cnn_id;
    wire [8:0] selected_cnn_row;
    wire [9:0] selected_cnn_col;
    wire selected_cnn_frame_start,selected_cnn_line_start,selected_cnn_line_end,selected_cnn_frame_end;
    wire source_select_cam_sys, source_select_valid_sys;
    wire cam_init_done;
    wire source_active_cam, source_switch_pulse, source_switch_pending, source_switch_reject;
    wire cam_stream_ready, cam_overflow;
    wire video_rst_n;
    (* async_reg="true" *) reg [1:0] video_reset_q;
    always @(posedge sys_clk or negedge rst_n)
        if(!rst_n)video_reset_q<=0;else video_reset_q<={video_reset_q[0],1'b1};
    assign video_rst_n=video_reset_q[1];
    wire camera_capture_reset_n=rst_n&&cam_init_done;
    wire hdmi_disp_valid, hdmi_disp_abort;
    wire [15:0] hdmi_disp_pixel,hdmi_disp_id;
    wire [8:0] hdmi_disp_row;wire [9:0] hdmi_disp_col;
    wire hdmi_disp_sof,hdmi_disp_ls,hdmi_disp_le,hdmi_disp_eof;
    wire hdmi_cnn_valid,hdmi_cnn_abort;
    wire [15:0] hdmi_cnn_pixel,hdmi_cnn_id;
    wire [8:0] hdmi_cnn_row;wire [9:0] hdmi_cnn_col;
    wire hdmi_cnn_sof,hdmi_cnn_ls,hdmi_cnn_le,hdmi_cnn_eof;
    wire cam_disp_valid,cam_disp_abort;
    wire [15:0] cam_disp_pixel,cam_disp_id;
    wire [8:0] cam_disp_row;wire [9:0] cam_disp_col;
    wire cam_disp_sof,cam_disp_ls,cam_disp_le,cam_disp_eof;
    wire cam_cnn_valid,cam_cnn_abort;
    wire [15:0] cam_cnn_pixel,cam_cnn_id;
    wire [8:0] cam_cnn_row;wire [9:0] cam_cnn_col;
    wire cam_cnn_sof,cam_cnn_ls,cam_cnn_le,cam_cnn_eof;
    wire selected_disp_valid,selected_disp_abort;
    wire [15:0] selected_disp_pixel,selected_disp_id;
    wire [8:0] selected_disp_row;wire [9:0] selected_disp_col;
    wire selected_disp_sof,selected_disp_ls,selected_disp_le,selected_disp_eof;
    (* async_reg="true" *) reg [1:0] hdmi_ready_sync_q;
    always @(posedge sys_clk or negedge rst_n)
        if(!rst_n)hdmi_ready_sync_q<=0;else hdmi_ready_sync_q<={hdmi_ready_sync_q[0],mode_valid};
    wire [15:0] overlay_pix565;
    wire overlay_pix_valid;
    wire [8:0] overlay_row_index;
    wire [9:0] overlay_col_index;
    wire overlay_row_start, overlay_row_commit, overlay_frame_done;
    wire [15:0] overlay_frame_id;
    wire f14_busy;
    wire [7:0] f14_diag_sys;
    wire [1:0] f14_diag_pixel;
    wire [31:0] tx_data;
    wire [3:0] tx_k;
    wire [31:0] meta_tx_data;
    wire [3:0] meta_tx_k;
    wire meta_valid,meta_ready,meta_tx_valid,meta_tx_ready,meta_error,packet_done;
    wire meta_hold_next_frame,packet_desc_ready;
    wire [15:0] packet_frame_id;
    wire [31:0] meta_seq,meta_payload;
    wire [63:0] meta_tx_payload;
    wire tx_ready /* synthesis PAP_MARK_DEBUG="true" */, fabric_clk, fabric_rst_n;
    wire desc_valid, desc_ready, desc_drop, data_valid, data_ready;
    wire [15:0] desc_frame_id;
    wire [8:0] desc_row;
    wire [1:0] desc_bank;
    wire [31:0] data32;
    wire frame_abort_tx;
    wire flush_pix, flush_tx;
    wire flush_done_pix, flush_done_tx, buffer_abort;

    // --- Build-mode selection and exported TX status ---------------------
    wire [31:0] selected_tx_data;
    wire [3:0]  selected_tx_k;
    // Diagnostic probes (kept + marked so Fabric Inserter can observe them).
    wire tx_pll_done   /* synthesis PAP_MARK_DEBUG="true" */;
    wire tx_pll_lock   /* synthesis PAP_MARK_DEBUG="true" */;
    wire tx_lane_done  /* synthesis PAP_MARK_DEBUG="true" */;
    wire [1:0] tx_watchdog_status /* synthesis PAP_MARK_DEBUG="true" */;
    wire dbg_fabric_alive   /* synthesis PAP_MARK_DEBUG="true" */;
    wire dbg_pixel_alive    /* synthesis PAP_MARK_DEBUG="true" */;
    wire dbg_timeout_sticky /* synthesis PAP_MARK_DEBUG="true" */;
    wire dbg_tx_ready_sync  /* synthesis PAP_MARK_DEBUG="true" */;
    wire [31:0] dbg_idle_cnt /* synthesis PAP_MARK_DEBUG="true" */;
    wire [11:0] dbg_meas_width  /* synthesis PAP_MARK_DEBUG="true" */;
    wire [11:0] dbg_meas_height /* synthesis PAP_MARK_DEBUG="true" */;
    wire [2:0]  dbg_active_mode /* synthesis PAP_MARK_DEBUG="true" */;
    wire [7:0]  dbg_build_id    /* synthesis PAP_MARK_DEBUG="true" */;

`ifdef HOST_TX_MODE_PATTERN
    // Bypass HDMI / line buffer with an independent protocol test pattern
    // (rtl/fiber/host_hsst_pattern_tx.v).
    wire [31:0] pattern_tx_data;
    wire [3:0]  pattern_tx_k;
    wire pattern_frame_done;
    host_hsst_pattern_tx u_pattern(
        .i_clk(fabric_clk),
        .i_rst_n(fabric_rst_n),
        .i_enable(tx_ready),
        .o_tx_data(pattern_tx_data),
        .o_tx_k(pattern_tx_k),
        .o_frame_done(pattern_frame_done)
    );
    assign selected_tx_data = pattern_tx_data;
    assign selected_tx_k    = pattern_tx_k;
`else
`ifdef HOST_TX_MODE_HDMI
    // HOST_TX_HDMI: existing packetizer/line-buffer path.
    assign selected_tx_data = meta_tx_data;
    assign selected_tx_k    = meta_tx_k;
`else
    // HOST_TX_MODE_IDLE (and the no-define default): HSST is asked to
    // transmit K/IDLE (DATA=32'h0000_00BC, K=4'b0001) regardless of HDMI,
    // pixel clock, mode_valid or line-buffer state. This isolates HSST PHY
    // bring-up from every application-layer module under repair.
    assign selected_tx_data = 32'h0000_00BC;
    assign selected_tx_k    = 4'b0001;
`endif
`endif

    // ADV7611 supplies the HDMI input pixel clock directly. The local HDMI
    // transmitter and its PLL are removed; input capture does not need them.
    i2c_ctrl u_adv7611_init(.adv_clk(sys_clk), .adv_rst(adv_reset_n), .adv_sclk(hdmi_scl), .adv_sdat(hdmi_sda), .init_done(adv_init_done));
    hdmi_rx u_hdmi_rx(.hdmi_clk_in(hdmi_clk_in), .hdmi_de_in(hdmi_de_in), .hdmi_vs_in(hdmi_vs_in), .hdmi_hs_in(hdmi_hs_in), .hdmi_data_in(hdmi_data_in), .pclk(pixel_clk), .hsync(video_hs), .vsync(video_vs), .de(video_de), .rgb_data(video_rgb));
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) pixel_heartbeat <= 1'b0;
        else pixel_heartbeat <= ~pixel_heartbeat;
    end
    // i_tmds_lock only gates the unused monitor reset output of this shared
    // supervisor. ADV7611 reset still uses its independent release counter.
    host_reset_supervisor #(.C_RELEASE_CYCLES(50000), .C_WATCHDOG_CYCLES(250000)) u_reset(.i_sys_clk(sys_clk), .i_rst_n(rst_n), .i_pixel_heartbeat(pixel_heartbeat), .i_tmds_lock(1'b1), .i_adv_init_done(adv_init_done), .o_adv_reset_n(adv_reset_n), .o_pixel_reset_n(), .o_input_timeout());
    host_hdmi_mode_monitor u_mode(.i_pixel_clk(pixel_clk), .i_rst_n(rst_n), .i_de(video_de), .i_hs(video_hs), .i_vs(video_vs), .o_mode_valid(mode_valid), .o_frame_start(frame_start), .o_frame_done(frame_done), .o_frame_error(frame_error), .o_measured_width(dbg_meas_width), .o_measured_height(dbg_meas_height));
    host_hdmi_tile_normalizer u_norm(.i_pixel_clk(pixel_clk), .i_rst_n(rst_n), .i_de(video_de), .i_hs(video_hs), .i_vs(video_vs), .i_rgb888(video_rgb), .i_mode_valid(mode_valid), .o_pixel_valid(pix_valid), .o_pixel_rgb565(pix565), .o_row_index(row_index), .o_col_index(col_index), .o_row_start(row_start), .o_row_commit(row_commit), .o_frame_done(norm_done), .o_frame_abort(frame_abort), .o_frame_id(frame_id), .o_frame_selected(frame_selected));
    host_f14_area_resize u_cnn_resize (
        .i_clk(pixel_clk),.i_rst_n(rst_n),.i_vs(video_vs),.i_de(video_de),.i_rgb(video_rgb),
        .i_enable(mode_valid && frame_selected),.i_frame_id(frame_id),
        .o_valid(cnn_valid),.o_rgb(),.o_rgb565(cnn_pixel),.o_row(cnn_row),.o_col(cnn_col),
        .o_frame_id(cnn_id),.o_abort(cnn_abort)
    );
    assign cnn_frame_start = cnn_valid && (cnn_row == 0) && (cnn_col == 0);
    assign cnn_line_start = cnn_valid && (cnn_col == 0);
    assign cnn_line_end = cnn_valid && (cnn_col == 223);
    assign cnn_frame_end = cnn_valid && (cnn_row == 167) && (cnn_col == 223);

    host_av_uart_source_select #(.C_APPLY_TIMEOUT(100000000)) u_av_uart_source_select (
        .i_clk(sys_clk), .i_rst_n(video_rst_n), .i_uart_rx(i_uart_rx), .o_uart_tx(o_uart_tx),
        .i_active_source(source_active_cam), .i_switch_pending(source_switch_pending),
        .i_switch_applied(source_switch_pulse), .i_switch_reject(source_switch_reject),
        .i_hdmi_ready(hdmi_ready_sync_q[1]), .i_cam_ready(cam_stream_ready),
        .o_source_select(source_select_cam_sys), .o_source_valid(source_select_valid_sys),
        .o_sequence(), .o_command_valid()
    );
    // HDMI normalization retains the input clock; all downstream streams cross
    // to the independent 50 MHz system clock.  Native aborts cross as events.
    host_video_stream_fifo #(.C_ADDR_WIDTH(11)) u_hdmi_display_cdc (
        .i_wr_clk(pixel_clk),.i_wr_rst_n(rst_n),.i_wr_valid(pix_valid),
        .i_wr_pixel(pix565),.i_wr_row(row_index),.i_wr_col(col_index),
        .i_wr_frame_start(pix_valid&&row_index==0&&col_index==0),
        .i_wr_line_start(row_start),.i_wr_line_end(row_commit),.i_wr_frame_end(norm_done),
        .i_wr_frame_id({1'b0,frame_id[14:0]}),.i_wr_abort(frame_abort),.o_wr_overflow(),
        .i_rd_clk(sys_clk),.i_rd_rst_n(video_rst_n),.i_rd_ready(1'b1),
        .o_rd_valid(hdmi_disp_valid),.o_rd_pixel(hdmi_disp_pixel),
        .o_rd_row(hdmi_disp_row),.o_rd_col(hdmi_disp_col),
        .o_rd_frame_start(hdmi_disp_sof),.o_rd_line_start(hdmi_disp_ls),
        .o_rd_line_end(hdmi_disp_le),.o_rd_frame_end(hdmi_disp_eof),
        .o_rd_frame_id(hdmi_disp_id),.o_rd_abort(hdmi_disp_abort)
    );
    host_video_stream_fifo #(.C_ADDR_WIDTH(10)) u_hdmi_cnn_cdc (
        .i_wr_clk(pixel_clk),.i_wr_rst_n(rst_n),.i_wr_valid(cnn_valid),
        .i_wr_pixel(cnn_pixel),.i_wr_row(cnn_row),.i_wr_col(cnn_col),
        .i_wr_frame_start(cnn_frame_start),.i_wr_line_start(cnn_line_start),
        .i_wr_line_end(cnn_line_end),.i_wr_frame_end(cnn_frame_end),
        .i_wr_frame_id({1'b0,cnn_id[14:0]}),.i_wr_abort(cnn_abort|frame_abort),.o_wr_overflow(),
        .i_rd_clk(sys_clk),.i_rd_rst_n(video_rst_n),.i_rd_ready(1'b1),
        .o_rd_valid(hdmi_cnn_valid),.o_rd_pixel(hdmi_cnn_pixel),
        .o_rd_row(hdmi_cnn_row),.o_rd_col(hdmi_cnn_col),
        .o_rd_frame_start(hdmi_cnn_sof),.o_rd_line_start(hdmi_cnn_ls),
        .o_rd_line_end(hdmi_cnn_le),.o_rd_frame_end(hdmi_cnn_eof),
        .o_rd_frame_id(hdmi_cnn_id),.o_rd_abort(hdmi_cnn_abort)
    );
    host_ov5640_input_wrapper u_ov5640_input (
        .i_sys_clk(sys_clk),.i_sys_rst_n(video_rst_n),
        .i_cam_pclk(cam_pclk),.i_cam_rst_n(camera_capture_reset_n),
        .i_cam_vsync(cam_vsync),.i_cam_href(cam_href),.i_cam_data(cam_data),
        .o_cam_ready(cam_stream_ready),.o_cam_overflow(cam_overflow),
        .o_display_valid(cam_disp_valid),.o_display_pixel(cam_disp_pixel),
        .o_display_row(cam_disp_row),.o_display_col(cam_disp_col),
        .o_display_frame_start(cam_disp_sof),.o_display_line_start(cam_disp_ls),
        .o_display_line_end(cam_disp_le),.o_display_frame_end(cam_disp_eof),
        .o_display_frame_id(cam_disp_id),.o_display_abort(cam_disp_abort),
        .o_cnn_valid(cam_cnn_valid),.o_cnn_pixel(cam_cnn_pixel),
        .o_cnn_row(cam_cnn_row),.o_cnn_col(cam_cnn_col),
        .o_cnn_frame_start(cam_cnn_sof),.o_cnn_line_start(cam_cnn_ls),
        .o_cnn_line_end(cam_cnn_le),.o_cnn_frame_end(cam_cnn_eof),
        .o_cnn_frame_id(cam_cnn_id),.o_cnn_abort(cam_cnn_abort)
    );
    video_dual_source_frame_mux #(.C_SWITCH_TIMEOUT(25000000)) u_source_mux (
        .i_clk(sys_clk),.i_rst_n(video_rst_n),.i_select_cam(source_select_cam_sys),
        .i_hdmi_ready(hdmi_ready_sync_q[1]),.i_cam_ready(cam_stream_ready),
        .i_hdmi_disp_valid(hdmi_disp_valid),.i_hdmi_disp_pixel(hdmi_disp_pixel),
        .i_hdmi_disp_row(hdmi_disp_row),.i_hdmi_disp_col(hdmi_disp_col),
        .i_hdmi_disp_frame_start(hdmi_disp_sof),.i_hdmi_disp_line_start(hdmi_disp_ls),
        .i_hdmi_disp_line_end(hdmi_disp_le),.i_hdmi_disp_frame_end(hdmi_disp_eof),
        .i_hdmi_disp_frame_id(hdmi_disp_id),.i_hdmi_disp_abort(hdmi_disp_abort),.o_hdmi_disp_ready(),
        .i_cam_disp_valid(cam_disp_valid),.i_cam_disp_pixel(cam_disp_pixel),
        .i_cam_disp_row(cam_disp_row),.i_cam_disp_col(cam_disp_col),
        .i_cam_disp_frame_start(cam_disp_sof),.i_cam_disp_line_start(cam_disp_ls),
        .i_cam_disp_line_end(cam_disp_le),.i_cam_disp_frame_end(cam_disp_eof),
        .i_cam_disp_frame_id({1'b1,cam_disp_id[14:0]}),.i_cam_disp_abort(cam_disp_abort),.o_cam_disp_ready(),
        .i_hdmi_cnn_valid(hdmi_cnn_valid),.i_hdmi_cnn_pixel(hdmi_cnn_pixel),
        .i_hdmi_cnn_row(hdmi_cnn_row),.i_hdmi_cnn_col(hdmi_cnn_col),
        .i_hdmi_cnn_frame_start(hdmi_cnn_sof),.i_hdmi_cnn_line_start(hdmi_cnn_ls),
        .i_hdmi_cnn_line_end(hdmi_cnn_le),.i_hdmi_cnn_frame_end(hdmi_cnn_eof),
        .i_hdmi_cnn_frame_id(hdmi_cnn_id),.i_hdmi_cnn_abort(hdmi_cnn_abort),.o_hdmi_cnn_ready(),
        .i_cam_cnn_valid(cam_cnn_valid),.i_cam_cnn_pixel(cam_cnn_pixel),
        .i_cam_cnn_row(cam_cnn_row),.i_cam_cnn_col(cam_cnn_col),
        .i_cam_cnn_frame_start(cam_cnn_sof),.i_cam_cnn_line_start(cam_cnn_ls),
        .i_cam_cnn_line_end(cam_cnn_le),.i_cam_cnn_frame_end(cam_cnn_eof),
        .i_cam_cnn_frame_id({1'b1,cam_cnn_id[14:0]}),.i_cam_cnn_abort(cam_cnn_abort),.o_cam_cnn_ready(),
        .o_active_cam(source_active_cam),.o_switch_pulse(source_switch_pulse),
        .o_switch_reject(source_switch_reject),.o_switch_pending(source_switch_pending),
        .o_display_valid(selected_disp_valid),.o_display_pixel(selected_disp_pixel),
        .o_display_row(selected_disp_row),.o_display_col(selected_disp_col),
        .o_display_frame_start(selected_disp_sof),.o_display_line_start(selected_disp_ls),
        .o_display_line_end(selected_disp_le),.o_display_frame_end(selected_disp_eof),
        .o_display_frame_id(selected_disp_id),.o_display_abort(selected_disp_abort),
        .o_cnn_valid(selected_cnn_valid),.o_cnn_pixel(selected_cnn_pixel),
        .o_cnn_row(selected_cnn_row),.o_cnn_col(selected_cnn_col),
        .o_cnn_frame_start(selected_cnn_frame_start),.o_cnn_line_start(selected_cnn_line_start),
        .o_cnn_line_end(selected_cnn_line_end),.o_cnn_frame_end(selected_cnn_frame_end),
        .o_cnn_frame_id(selected_cnn_id),.o_cnn_abort(selected_cnn_abort)
    );
    wire f14_clk75, f14_rst_n, f14_pixel_rst_n, f14_locked;
    wire [7:0] f14_reset_partition_n;
    f14_board_clock75 u_f14_clock (
        .i_clk50(sys_clk),.i_pixel_clk(sys_clk),.i_rst_n(rst_n),
        .o_clk75(f14_clk75),.o_rst_n(f14_rst_n),
        .o_reset_partition_n(f14_reset_partition_n),
        .o_pixel_rst_n(f14_pixel_rst_n),.o_locked(f14_locked)
    );
    host_f14_video_lane_path #(.C_PARTITION_RESET(1)) u_lane_path (
        // ?????????PLL??????????lane_path??????
        .i_pixel_clk(sys_clk),.i_pixel_rst_n(video_rst_n),.i_sys_clk(f14_clk75),.i_sys_rst_n(f14_rst_n),
        .i_reset_partition_n(f14_reset_partition_n),
        .i_valid(selected_disp_valid),.i_pixel(selected_disp_pixel),.i_row(selected_disp_row),.i_col(selected_disp_col),
        .i_row_start(selected_disp_ls),.i_row_commit(selected_disp_le),.i_frame_done(selected_disp_eof),
        .i_frame_abort(selected_disp_abort),.i_frame_id(selected_disp_id),
        .i_cnn_valid(selected_cnn_valid),.i_cnn_pixel(selected_cnn_pixel),.i_cnn_row(selected_cnn_row),.i_cnn_col(selected_cnn_col),
        .i_cnn_frame_id(selected_cnn_id),.i_cnn_abort(selected_cnn_abort),
        .o_valid(overlay_pix_valid),.o_pixel(overlay_pix565),
        .o_row(overlay_row_index),.o_col(overlay_col_index),
        .o_row_start(overlay_row_start),.o_row_commit(overlay_row_commit),
        .o_frame_done(overlay_frame_done),.o_frame_id(overlay_frame_id),.o_busy(f14_busy),
        .o_diag_sys(f14_diag_sys),.o_diag_pixel(f14_diag_pixel),
        .i_meta_rst_n(meta_pixel_rst_q[1]),.i_meta_ready(meta_ready),
        .o_meta_valid(meta_valid),.o_meta_seq(meta_seq),.o_meta_payload(meta_payload)
    );
    // ?????????????????????????????
    wire meta_async_rst_n=rst_n && tx_ready && f14_rst_n;
    always @(posedge sys_clk or negedge meta_async_rst_n)
        if(!meta_async_rst_n) meta_pixel_rst_q<=0;
        else meta_pixel_rst_q<={meta_pixel_rst_q[0],1'b1};
    always @(posedge fabric_clk or negedge meta_async_rst_n)
        if(!meta_async_rst_n) meta_tx_rst_q<=0;
        else meta_tx_rst_q<={meta_tx_rst_q[0],1'b1};
    host_frame_mailbox #(.C_WIDTH(64)) u_meta_cdc (
        .i_src_clk(sys_clk),.i_src_rst_n(meta_pixel_rst_q[1]),.i_valid(meta_valid),
        .o_ready(meta_ready),.i_data({meta_seq,meta_payload}),
        .i_dst_clk(fabric_clk),.i_dst_rst_n(meta_tx_rst_q[1]),
        .o_valid(meta_tx_valid),.i_ready(meta_tx_ready),.o_data(meta_tx_payload)
    );
    assign fabric_rst_n = rst_n & tx_ready;
    host_frame_control_cdc u_frame_ctrl(.i_pix_clk(sys_clk), .i_pix_rst_n(video_rst_n), .i_frame_abort_pix(selected_disp_abort | buffer_abort), .o_flush_pix(flush_pix), .i_flush_done_pix(flush_done_pix), .i_tx_clk(fabric_clk), .i_tx_rst_n(fabric_rst_n), .o_flush_tx(flush_tx), .i_flush_done_tx(flush_done_tx));
    host_line_buffer_cdc u_lines(.i_pix_clk(sys_clk), .i_pix_rst_n(video_rst_n), .i_pixel_valid(overlay_pix_valid), .i_pixel_rgb565(overlay_pix565), .i_row_start(overlay_row_start), .i_row_commit(overlay_row_commit), .i_frame_id(overlay_frame_id), .i_row_index(overlay_row_index), .i_tx_clk(fabric_clk), .i_tx_rst_n(fabric_rst_n), .o_desc_valid(desc_valid), .i_desc_ready(desc_ready), .i_desc_drop(desc_drop), .o_desc_frame_id(desc_frame_id), .o_desc_row(desc_row), .o_desc_bank(desc_bank), .o_data_valid(data_valid), .i_data_ready(data_ready), .o_data32(data32), .i_bank_return_valid(1'b0), .i_bank_return(2'b0), .i_flush_pix(flush_pix), .o_flush_done_pix(flush_done_pix), .i_flush_tx(flush_tx), .o_flush_done_tx(flush_done_tx), .o_buffer_abort(buffer_abort));
    assign frame_abort_tx = flush_tx;
    assign desc_ready=packet_desc_ready && !meta_hold_next_frame;
    fiber_video_tx_packetizer u_packet(.i_clk(fabric_clk), .i_rst_n(fabric_rst_n), .i_frame_abort(frame_abort_tx), .i_desc_valid(desc_valid && !meta_hold_next_frame), .o_desc_ready(packet_desc_ready), .o_desc_drop(desc_drop), .i_desc_frame_id(desc_frame_id), .i_desc_row(desc_row), .i_desc_bank(desc_bank), .i_data_valid(data_valid), .o_data_ready(data_ready), .i_data32(data32), .o_tx_data(tx_data), .o_tx_k(tx_k), .o_frame_done(packet_done),.o_frame_done_id(packet_frame_id));
    host_hsst_lane_meta_tx u_meta_tx (
        .i_clk(fabric_clk),.i_rst_n(meta_tx_rst_q[1]),.i_abort(frame_abort_tx),
        .i_video_data(tx_data),.i_video_k(tx_k),.i_frame_done(packet_done),
        .i_meta_valid(meta_tx_valid),.o_meta_ready(meta_tx_ready),
        .i_meta_seq(meta_tx_payload[63:32]),.i_meta_payload(meta_tx_payload[31:0]),
        .o_data(meta_tx_data),.o_k(meta_tx_k),.o_hold_next_frame(meta_hold_next_frame),.o_error(meta_error)
    );
    // The generated HSST core exposes the 78.125 MHz fabric clock.  It is
    // looped back into the core and is now the sole clock for the packetizer
    // and RAM read side.  i_ip_ready/i_tx_valid stay asserted; only the data
    // and K word are selected by HOST_TX_MODE.
    host_hsst_tx_adapter u_hsst(.i_tx_clk(sys_clk), .i_rst_n(rst_n), .i_ip_ready(1'b1), .i_refckn(i_p_refckn_1), .i_refckp(i_p_refckp_1), .i_tx_valid(1'b1), .i_tx_data(selected_tx_data), .i_tx_k(selected_tx_k), .o_tx_ready(tx_ready), .o_fabric_clk(fabric_clk), .o_fabric_data(), .o_fabric_k(), .o_hsst_txp(hsst_txp), .o_hsst_txn(hsst_txn), .o_tx_disable(tx_disable), .o_tx_pll_done(tx_pll_done), .o_tx_pll_lock(tx_pll_lock), .o_tx_lane_done(tx_lane_done), .o_tx_watchdog_status(tx_watchdog_status));

    // --- Slow heartbeats for clock-alive detection ------------------------
    // Use a free-running 16-bit counter's MSB as a slow toggle so the 50 MHz
    // management domain can reliably sample it (no fast single-cycle toggle).
    reg [15:0] fabric_hb_cnt;
    reg [15:0] pixel_hb_cnt;
    always @(posedge fabric_clk or negedge rst_n) begin
        if (!rst_n) fabric_hb_cnt <= 16'd0;
        else fabric_hb_cnt <= fabric_hb_cnt + 1'b1;
    end
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) pixel_hb_cnt <= 16'd0;
        else pixel_hb_cnt <= pixel_hb_cnt + 1'b1;
    end

    // --- Management-domain diagnostic monitor ----------------------------
    host_debug_monitor #(
        .C_LED_SELECT(`HOST_LED_SELECT)
    ) u_debug(
        .i_sys_clk(sys_clk),
        .i_rst_n(rst_n),
        .i_tx_pll_done(tx_pll_done),
        .i_tx_pll_lock(tx_pll_lock),
        .i_tx_lane_done(tx_lane_done),
        .i_tx_ready(tx_ready),
        .i_mode_valid(mode_valid),
        .i_fabric_heartbeat(fabric_hb_cnt[15]),
        .i_pixel_heartbeat(pixel_hb_cnt[15]),
        .i_fabric_clk(fabric_clk),
        .i_fabric_rst_n(fabric_rst_n),
        .i_tx_data(selected_tx_data),
        .i_tx_k(selected_tx_k),
        .i_frame_done(frame_done),
        .i_frame_abort(frame_abort_tx),
        .i_bad_desc(1'b0),
        .i_active_mode(`HOST_TX_MODE_VAL),
        .i_build_id(`HOST_BUILD_ID),
        .o_tx_pll_done_sync(),
        .o_tx_pll_lock_sync(),
        .o_tx_lane_done_sync(),
        .o_tx_ready_sync(dbg_tx_ready_sync),
        .o_fabric_clock_alive(dbg_fabric_alive),
        .o_pixel_clock_alive(dbg_pixel_alive),
        .o_timeout_sticky(dbg_timeout_sticky),
        .o_presented_idle_count(dbg_idle_cnt),
        .o_presented_vsync_count(),
        .o_presented_data_count(),
        .o_frame_done_count(),
        .o_frame_abort_count(),
        .o_bad_desc_count(),
        .o_active_mode(dbg_active_mode),
        .o_build_id(dbg_build_id),
        .o_led(led)
    );

    assign hdmi_rst_n_out = adv_reset_n;
    // Camera reset/power/SCCB are owned by the local OV5640 initializer.
    // The DVP stream is still consumed by host_ov5640_input_wrapper.
    ov5640_sccb_init #(
        .C_CLOCK_HZ(50000000), .C_I2C_HZ(250000),
        .C_WIDTH(640), .C_HEIGHT(320), .C_TOTAL_H(1800), .C_TOTAL_V(1000)
    ) u_ov5640_sccb_init (
        .i_clk(sys_clk), .i_rst_n(rst_n),
        .o_cam_rst_n(cam_rst_n), .o_cam_pwdn(cam_pwdn),
        .o_cam_scl(cam_scl), .o_cam_sda(cam_sda),
        .o_init_done(cam_init_done)
    );

endmodule





