`timescale 1ns / 1ps
// 8N1 UART transmitter. A one-cycle i_start pulse captures i_data while idle.
module uart_tx_8n1 #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD_RATE = 115200
)(
    input  wire       i_clk,
    input  wire       i_rstn,
    input  wire       i_start,
    input  wire [7:0] i_data,
    output reg        o_busy,
    output reg        o_tx
);
    localparam integer CLKS_PER_BIT = (CLOCK_HZ / BAUD_RATE < 1) ? 1 : (CLOCK_HZ / BAUD_RATE);
    reg [31:0] bit_count;
    reg [3:0]  bit_index;
    reg [9:0]  shift_reg;

    always @(posedge i_clk or negedge i_rstn) begin
        if (!i_rstn) begin
            o_busy    <= 1'b0;
            o_tx      <= 1'b1;
            bit_count <= 32'd0;
            bit_index <= 4'd0;
            shift_reg <= 10'h3ff;
        end else begin
            if (!o_busy) begin
                o_tx <= 1'b1;
                bit_count <= 32'd0;
                bit_index <= 4'd0;
                if (i_start) begin
                    // start(0), data LSB first, stop(1)
                    shift_reg <= {1'b1, i_data, 1'b0};
                    o_busy <= 1'b1;
                    o_tx <= 1'b0;
                end
            end else if (bit_count == CLKS_PER_BIT-1) begin
                bit_count <= 32'd0;
                if (bit_index == 4'd9) begin
                    o_busy <= 1'b0;
                    o_tx <= 1'b1;
                    bit_index <= 4'd0;
                end else begin
                    bit_index <= bit_index + 1'b1;
                    o_tx <= shift_reg[bit_index + 1'b1];
                end
            end else begin
                bit_count <= bit_count + 1'b1;
            end
        end
    end
endmodule
