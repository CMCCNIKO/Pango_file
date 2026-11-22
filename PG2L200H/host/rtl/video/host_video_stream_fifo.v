`timescale 1ns/1ps
`default_nettype none

// Dual-clock FIFO for one normalized video stream.  It supports consumer
// backpressure, but errors override ready to drain the damaged queue.  A
// stopped-writer/drained-reader handshake then re-arms at a fresh source SOF.
module host_video_stream_fifo #(
    parameter integer C_ADDR_WIDTH = 12
) (
    input  wire        i_wr_clk,
    input  wire        i_wr_rst_n,
    input  wire        i_wr_valid,
    input  wire [15:0] i_wr_pixel,
    input  wire [8:0]  i_wr_row,
    input  wire [9:0]  i_wr_col,
    input  wire        i_wr_frame_start,
    input  wire        i_wr_line_start,
    input  wire        i_wr_line_end,
    input  wire        i_wr_frame_end,
    input  wire [15:0] i_wr_frame_id,
    input  wire        i_wr_abort,
    output wire        o_wr_overflow,
    input  wire        i_rd_clk,
    input  wire        i_rd_rst_n,
    input  wire        i_rd_ready,
    output wire        o_rd_valid,
    output wire [15:0] o_rd_pixel,
    output wire [8:0]  o_rd_row,
    output wire [9:0]  o_rd_col,
    output wire        o_rd_frame_start,
    output wire        o_rd_line_start,
    output wire        o_rd_line_end,
    output wire        o_rd_frame_end,
    output wire [15:0] o_rd_frame_id,
    output wire        o_rd_abort
);
    localparam integer C_DATA_WIDTH = 56;
    wire [C_DATA_WIDTH-1:0] wr_data = {
        i_wr_abort, i_wr_frame_id, i_wr_frame_end, i_wr_line_end,
        i_wr_line_start, i_wr_frame_start, i_wr_row, i_wr_col, i_wr_pixel
    };
    wire [C_DATA_WIDTH-1:0] rd_data;
    wire fifo_valid;
    wire fifo_drained;
    wire raw_overflow;
    reg error_toggle_q;
    reg wr_flush_wait_q,wr_sof_wait_q;
    (* async_reg="true" *) reg [1:0] ack_sync_q;
    (* async_reg="true" *) reg [2:0] error_sync_q;
    reg error_seen_q,flush_ack_q,rd_flush_q,drained_seen_q;
    reg [3:0] settle_q;
    reg drop_frame_q;
    wire error_event=error_sync_q[2]^error_seen_q;
    wire raw_frame_start=rd_data[35];
    wire read_flushing=rd_flush_q||error_event;
    wire fifo_read_ready=read_flushing||
                         (drop_frame_q&&!raw_frame_start)||i_rd_ready;
    wire writer_enabled=!wr_flush_wait_q&&
                        (!wr_sof_wait_q||i_wr_frame_start)&&!i_wr_abort;
    wire common_reset_n=i_wr_rst_n&&i_rd_rst_n;
    (* async_reg="true" *) reg [1:0] wr_reset_q,rd_reset_q;
    always @(posedge i_wr_clk or negedge common_reset_n)
        if(!common_reset_n)wr_reset_q<=0;else wr_reset_q<={wr_reset_q[0],1'b1};
    always @(posedge i_rd_clk or negedge common_reset_n)
        if(!common_reset_n)rd_reset_q<=0;else rd_reset_q<={rd_reset_q[0],1'b1};

    // On the first error stop all writes, including later source frames.
    // No new SOF can enter until the reader has drained the entire old queue
    // and acknowledged this request.  Then wait for a fresh source SOF.
    always @(posedge i_wr_clk or negedge common_reset_n) begin
        if(!common_reset_n)begin
            error_toggle_q<=0;wr_flush_wait_q<=0;wr_sof_wait_q<=1;ack_sync_q<=0;
        end else begin
            ack_sync_q<={ack_sync_q[0],flush_ack_q};
            if(wr_flush_wait_q)begin
                if(ack_sync_q[1]==error_toggle_q)begin
                    wr_flush_wait_q<=0;wr_sof_wait_q<=1;
                end
            end else if(i_wr_abort||raw_overflow)begin
                error_toggle_q<=~error_toggle_q;
                wr_flush_wait_q<=1;wr_sof_wait_q<=1;
            end else if(i_wr_valid&&i_wr_frame_start)begin
                wr_sof_wait_q<=0;
            end
        end
    end

    // Flush ignores consumer backpressure.  Eight read clocks allow the final
    // stopped write pointer to cross the official FIFO CDC before empty is
    // considered; require drained on two consecutive clocks before ACK.
    always @(posedge i_rd_clk or negedge common_reset_n)begin
        if(!common_reset_n)begin
            error_sync_q<=0;error_seen_q<=0;flush_ack_q<=0;
            rd_flush_q<=0;settle_q<=0;drained_seen_q<=0;drop_frame_q<=1;
        end else begin
            error_sync_q<={error_sync_q[1:0],error_toggle_q};
            error_seen_q<=error_sync_q[2];
            if(error_event)begin
                rd_flush_q<=1;settle_q<=8;drained_seen_q<=0;drop_frame_q<=1;
            end else if(rd_flush_q)begin
                if(settle_q!=0)begin
                    settle_q<=settle_q-1'b1;drained_seen_q<=0;
                end else if(fifo_drained)begin
                    if(drained_seen_q)begin
                        flush_ack_q<=error_sync_q[2];rd_flush_q<=0;
                        drained_seen_q<=0;
                    end else drained_seen_q<=1;
                end else drained_seen_q<=0;
            end else if(fifo_valid&&raw_frame_start&&i_rd_ready)begin
                drop_frame_q<=0;
            end
        end
    end
    assign o_wr_overflow=raw_overflow;
    assign o_rd_valid=fifo_valid&&!read_flushing&&(!drop_frame_q||raw_frame_start);    // C_ADDR_WIDTH is retained at the wrapper boundary for compatibility.
    // The verified target-specific implementation has a fixed 512-token depth.
    host_vendor_video_fifo #(.C_DATA_WIDTH(C_DATA_WIDTH)) u_fifo (
        .i_wr_clk(i_wr_clk), .i_wr_rst_n(wr_reset_q[1]), .i_wr_valid(i_wr_valid&&writer_enabled),
        .i_wr_data(wr_data), .o_wr_full(), .o_wr_overflow(raw_overflow),
        .i_rd_clk(i_rd_clk), .i_rd_rst_n(rd_reset_q[1]), .o_rd_valid(fifo_valid),
        .o_rd_data(rd_data), .o_rd_drained(fifo_drained), .i_rd_ready(fifo_read_ready)
    );
    wire token_abort;
    assign o_rd_abort=error_event||(fifo_valid&&token_abort&&!read_flushing);
    assign {token_abort, o_rd_frame_id, o_rd_frame_end, o_rd_line_end,
            o_rd_line_start, o_rd_frame_start, o_rd_row, o_rd_col, o_rd_pixel} = rd_data;
endmodule
`default_nettype wire

