`timescale 1ns / 1ps
// 8N1 UART receiver with a two-flop input synchronizer.
module uart_rx_8n1 #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD_RATE = 115200
)(
    input  wire       i_clk,
    input  wire       i_rstn,
    input  wire       i_rx,
    output reg        o_valid,
    output reg [7:0]  o_data,
    output reg        o_frame_error
);
    localparam integer CLKS_PER_BIT = (CLOCK_HZ / BAUD_RATE < 1) ? 1 : (CLOCK_HZ / BAUD_RATE);
    localparam integer HALF_BIT = (CLKS_PER_BIT / 2 < 1) ? 1 : (CLKS_PER_BIT / 2);
    localparam [1:0] S_IDLE=2'd0, S_START=2'd1, S_DATA=2'd2, S_STOP=2'd3;
    reg rx_meta, rx_sync;
    reg [1:0] state;
    reg [31:0] bit_count;
    reg [3:0] bit_index;
    reg [7:0] data_reg;

    always @(posedge i_clk or negedge i_rstn) begin
        if (!i_rstn) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
            state <= S_IDLE;
            bit_count <= 32'd0;
            bit_index <= 4'd0;
            data_reg <= 8'd0;
            o_valid <= 1'b0;
            o_data <= 8'd0;
            o_frame_error <= 1'b0;
        end else begin
            rx_meta <= i_rx;
            rx_sync <= rx_meta;
            o_valid <= 1'b0;
            o_frame_error <= 1'b0;
            case (state)
                S_IDLE: begin
                    bit_count <= 32'd0;
                    bit_index <= 4'd0;
                    if (!rx_sync) state <= S_START;
                end
                S_START: begin
                    if (bit_count == HALF_BIT-1) begin
                        bit_count <= 32'd0;
                        if (!rx_sync) begin
                            state <= S_DATA;
                            bit_index <= 4'd0;
                        end else begin
                            state <= S_IDLE;
                        end
                    end else bit_count <= bit_count + 1'b1;
                end
                S_DATA: begin
                    if (bit_count == CLKS_PER_BIT-1) begin
                        bit_count <= 32'd0;
                        data_reg[bit_index] <= rx_sync;
                        if (bit_index == 4'd7) state <= S_STOP;
                        else bit_index <= bit_index + 1'b1;
                    end else bit_count <= bit_count + 1'b1;
                end
                S_STOP: begin
                    if (bit_count == CLKS_PER_BIT-1) begin
                        bit_count <= 32'd0;
                        state <= S_IDLE;
                        o_data <= data_reg;
                        o_valid <= 1'b1;
                        o_frame_error <= !rx_sync;
                    end else bit_count <= bit_count + 1'b1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
