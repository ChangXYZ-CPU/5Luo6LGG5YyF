`timescale 1ns / 1ps
/*****************************************************************
 * uart_tx_115200.v -- plain 8N1 UART transmitter (2026-10-18)
 *
 * - TX only, no flow control
 * - One start pulse on I_start sends one byte (I_data), LSB first
 * - O_busy is high from the start bit until the stop bit finishes
 * - CLK_HZ / BAUD are parameters so the same file works on any clock
 *   (24 MHz / 115200 = 208.33 -> DIV = 208, actual 115385 bps, +0.16%)
 *****************************************************************/
module uart_tx_115200 #(
    parameter integer CLK_HZ = 24000000,
    parameter integer BAUD   = 115200
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire       I_start,      // one-clock pulse: send I_data
    input  wire [7:0] I_data,
    output reg        O_uart_tx,    // serial line (idle high)
    output reg        O_busy
);
    // clocks per bit (rounded), at least 2
    localparam integer DIV = (CLK_HZ + BAUD/2) / BAUD;

    reg [15:0] div_cnt;
    reg [3:0]  bit_idx;        // 0..9 : start, d0..d7, stop
    reg [9:0]  shift_reg;      // {stop, d7..d0, start}
    reg        running;

    wire bit_end = (div_cnt == DIV - 1);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_uart_tx <= 1'b1;
            O_busy    <= 1'b0;
            running   <= 1'b0;
            div_cnt   <= 16'd0;
            bit_idx   <= 4'd0;
            shift_reg <= 10'h3ff;
        end else if (!running) begin
            O_uart_tx <= 1'b1;
            div_cnt   <= 16'd0;
            if (I_start) begin
                shift_reg <= {1'b1, I_data, 1'b0};   // stop, data[7:0], start
                O_uart_tx <= 1'b0;                   // start bit goes out now
                bit_idx   <= 4'd0;                   // 0 = start bit period
                running   <= 1'b1;
                O_busy    <= 1'b1;
            end
        end else begin
            if (bit_end) begin
                div_cnt <= 16'd0;
                if (bit_idx == 4'd9) begin
                    running   <= 1'b0;               // end of the stop bit period
                    O_busy    <= 1'b0;
                    O_uart_tx <= 1'b1;               // back to idle high
                end else begin
                    O_uart_tx <= shift_reg[bit_idx + 4'd1];   // drive the NEXT bit at its own start
                    bit_idx   <= bit_idx + 4'd1;
                end
            end else begin
                div_cnt <= div_cnt + 16'd1;
            end
        end
    end
endmodule
