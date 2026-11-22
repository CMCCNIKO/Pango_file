`timescale 1ns/1ps
// PG2L200H UART endpoint for the PG2L100H AV source-select bridge.
// RX frame: 7e, 08, AV v1 opcode4 seq_hi seq_lo source reserved.
// source 0 selects HDMI and source 1 selects OV5640. ACK is 7e, 0c,
// AV v1 opcode84 seq event2 on success, event3 on reject, error plus four reserved bytes.
module host_av_uart_source_select #(
    parameter integer C_CLOCK_HZ = 50000000,
    parameter integer C_BAUD_RATE = 115200,
    parameter integer C_APPLY_TIMEOUT = 100000000
)(
    input wire i_clk,
    input wire i_rst_n,
    input wire i_uart_rx,
    output wire o_uart_tx,
    input wire i_active_source,
    input wire i_switch_pending,
    input wire i_switch_applied,
    input wire i_switch_reject,
    input wire i_hdmi_ready,
    input wire i_cam_ready,
    output reg o_source_select,
    output reg o_source_valid,
    output reg [15:0] o_sequence,
    output reg o_command_valid
);
    localparam [1:0] ST_WAIT_SOF=2'd0, ST_WAIT_LEN=2'd1, ST_PAYLOAD=2'd2;
    localparam [7:0] SOF=8'h7e, AV0=8'h41, AV1=8'h56;
    wire rx_valid;
    wire [7:0] rx_data;
    wire rx_error;
    wire tx_busy;
    reg tx_start;
    reg [7:0] tx_data;
    reg tx_active;
    reg tx_started;
    reg tx_busy_seen;
    reg [3:0] tx_index;
    reg [95:0] ack_payload;
    reg [1:0] rx_state;
    reg [3:0] rx_count;
    reg [63:0] payload_shift;
    reg [7:0] rx_error_code;
    reg [15:0] rx_seq;
    reg [7:0] rx_opcode;
    reg [7:0] rx_source;
    reg [7:0] rx_reserved;
    reg [7:0] ack_error;
    reg apply_wait_q;
    reg [31:0] apply_timer_q;
    reg [15:0] apply_seq_q;
    reg apply_source_q;
    wire apply_ready=apply_source_q?i_cam_ready:i_hdmi_ready;
    wire [31:0] status_word={7'd0,i_active_source,6'd0,i_cam_ready,i_hdmi_ready,7'd0,i_switch_pending,8'd0};

    uart_rx_8n1 #(.CLOCK_HZ(C_CLOCK_HZ), .BAUD_RATE(C_BAUD_RATE)) u_rx (
        .i_clk(i_clk), .i_rstn(i_rst_n), .i_rx(i_uart_rx),
        .o_valid(rx_valid), .o_data(rx_data), .o_frame_error(rx_error));
    uart_tx_8n1 #(.CLOCK_HZ(C_CLOCK_HZ), .BAUD_RATE(C_BAUD_RATE)) u_tx (
        .i_clk(i_clk), .i_rstn(i_rst_n), .i_start(tx_start), .i_data(tx_data),
        .o_busy(tx_busy), .o_tx(o_uart_tx));

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_source_select <= 1'b0;
            o_source_valid <= 1'b0;
            o_sequence <= 16'd0;
            o_command_valid <= 1'b0;
            tx_start <= 1'b0;
            tx_data <= 8'd0;
            tx_active <= 1'b0;
            tx_started <= 1'b0;
            tx_busy_seen <= 1'b0;
            tx_index <= 4'd0;
            ack_payload <= 96'd0;
            rx_state <= ST_WAIT_SOF;
            rx_count <= 4'd0;
            payload_shift <= 64'd0;
            rx_error_code <= 8'd0;
            rx_seq <= 16'd0;
            rx_opcode <= 8'd0;
            rx_source <= 8'd0;
            rx_reserved <= 8'd0;
            ack_error <= 8'd0;
            apply_wait_q<=0;apply_timer_q<=0;apply_seq_q<=0;apply_source_q<=0;
        end else begin
            tx_start <= 1'b0;
            o_source_valid <= 1'b0;
            o_command_valid <= 1'b0;

            // uart_tx observes tx_start/tx_data on the following clock. Keep
            // an explicit busy-seen handshake so a byte is not skipped while
            // o_busy is still low in the launch cycle.
            if (tx_active) begin
                if (!tx_started && !tx_busy) begin
                    tx_start <= 1'b1;
                    tx_started <= 1'b1;
                    tx_busy_seen <= 1'b0;
                    case (tx_index)
                        4'd0: tx_data <= SOF;
                        4'd1: tx_data <= 8'd12;
                        4'd2: tx_data <= ack_payload[95:88];
                        4'd3: tx_data <= ack_payload[87:80];
                        4'd4: tx_data <= ack_payload[79:72];
                        4'd5: tx_data <= ack_payload[71:64];
                        4'd6: tx_data <= ack_payload[63:56];
                        4'd7: tx_data <= ack_payload[55:48];
                        4'd8: tx_data <= ack_payload[47:40];
                        4'd9: tx_data <= ack_payload[39:32];
                        4'd10: tx_data <= ack_payload[31:24];
                        4'd11: tx_data <= ack_payload[23:16];
                        4'd12: tx_data <= ack_payload[15:8];
                        default: tx_data <= ack_payload[7:0];
                    endcase
                end else if (tx_started) begin
                    if (tx_busy) begin
                        tx_busy_seen <= 1'b1;
                    end else if (tx_busy_seen) begin
                        tx_started <= 1'b0;
                        tx_busy_seen <= 1'b0;
                        if (tx_index == 4'd13) tx_active <= 1'b0;
                        else tx_index <= tx_index + 1'b1;
                    end
                end
            end

            if(apply_wait_q)begin
                if((i_switch_applied||(i_active_source==apply_source_q&&!i_switch_pending))&&apply_ready)begin
                    ack_payload<={16'h4156,8'h01,8'h84,apply_seq_q,8'h02,8'h00,status_word};
                    tx_active<=1;tx_index<=0;apply_wait_q<=0;apply_timer_q<=0;
                end else if(i_switch_reject||apply_timer_q>=C_APPLY_TIMEOUT-1)begin
                    ack_payload<={16'h4156,8'h01,8'h84,apply_seq_q,8'h03,8'h07,status_word};
                    tx_active<=1;tx_index<=0;apply_wait_q<=0;apply_timer_q<=0;
                    o_source_select<=i_active_source;
                end else apply_timer_q<=apply_timer_q+1'b1;
            end
            if (rx_valid) begin
                if (rx_error) begin
                    rx_state <= ST_WAIT_SOF;
                    rx_count <= 0;
                    payload_shift <= 0;
                end else begin
                    case (rx_state)
                        ST_WAIT_SOF: begin
                            if (rx_data == SOF) rx_state <= ST_WAIT_LEN;
                        end
                        ST_WAIT_LEN: begin
                            if (rx_data == 8'd8) begin
                                rx_state <= ST_PAYLOAD;
                                rx_count <= 0;
                                payload_shift <= 0;
                            end else begin
                                rx_state <= ST_WAIT_SOF;
                            end
                        end
                        ST_PAYLOAD: begin
                            if (rx_count == 4'd7) begin
                                rx_state <= ST_WAIT_SOF;
                                rx_count <= 0;
                                rx_opcode <= payload_shift[31:24];
                                rx_seq <= payload_shift[23:8];
                                rx_source <= payload_shift[7:0];
                                rx_reserved <= rx_data;
                                if (!tx_active && !tx_busy && !apply_wait_q) begin
                                    if (payload_shift[55:48] != 8'h41 || payload_shift[47:40] != 8'h56) ack_error <= 8'd3;
                                    else if (payload_shift[39:32] != 8'd1) ack_error <= 8'd2;
                                    else if (payload_shift[31:24] != 8'd4) ack_error <= 8'd3;
                                    else if (payload_shift[7:0] > 8'd1) ack_error <= 8'd4;
                                    else if (rx_data != 8'd0) ack_error <= 8'd4;
                                    else ack_error <= 8'd0;
                                    rx_error_code <= (payload_shift[55:48] != 8'h41 || payload_shift[47:40] != 8'h56) ? 8'd3 :
                                                     (payload_shift[39:32] != 8'd1) ? 8'd2 :
                                                     (payload_shift[31:24] != 8'd4) ? 8'd3 :
                                                     (payload_shift[7:0] > 8'd1 || rx_data != 8'd0) ? 8'd4 : 8'd0;
                                    if(payload_shift[55:48]==8'h41&&payload_shift[47:40]==8'h56&&
                                       payload_shift[39:32]==8'd1&&payload_shift[31:24]==8'd4&&
                                       payload_shift[7:0]<=8'd1&&rx_data==8'd0)begin
                                        o_source_select<=payload_shift[0];o_source_valid<=1;
                                        o_command_valid<=1;o_sequence<=payload_shift[23:8];
                                        apply_seq_q<=payload_shift[23:8];apply_source_q<=payload_shift[0];
                                        apply_wait_q<=1;apply_timer_q<=0;
                                    end else begin
                                        ack_payload<={16'h4156,8'h01,8'h84,payload_shift[23:8],8'h03,
                                            (payload_shift[55:48]!=8'h41||payload_shift[47:40]!=8'h56)?8'd3:
                                            (payload_shift[39:32]!=8'd1)?8'd2:
                                            (payload_shift[31:24]!=8'd4)?8'd3:8'd4,status_word};
                                        tx_active<=1;tx_index<=0;
                                    end
                                end
                            end else begin
                                payload_shift <= {payload_shift[55:0],rx_data};
                                rx_count <= rx_count + 1'b1;
                            end
                        end
                        default: rx_state <= ST_WAIT_SOF;
                    endcase
                end
            end
        end
    end
endmodule




