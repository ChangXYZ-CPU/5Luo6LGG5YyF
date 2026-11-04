// ============================================================================
// colour_probe.v  (2026-10-24)
//
// Averages a 16x16 window centred on (CX, CY) and dumps the three channel
// averages over the existing UART transmitter when I_trig pulses.
//
// Packet:  A5 5A R G B 0D      (R/G/B are the 8-bit window averages)
// The average is latched at vsync, so a trigger always reports the last
// COMPLETE frame.
//
// The window sits on the LSC-corrected pixel stream (the same data the colour
// detectors see), so the reported colour already includes whatever E/C/b and
// saturation are doing.  Channels are averaged separately; normalise them to
// chromaticity on the host if brightness-independent numbers are needed.
// ============================================================================
`timescale 1ns / 1ps

module colour_probe #(
    parameter integer CX   = 640,
    parameter integer CY   = 360,
    parameter integer HALF = 8          // 8 -> 16x16 window
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    input  wire        I_trig,
    output reg         O_busy,
    output reg         O_done,
    output reg         O_tx_valid,
    output reg  [7:0]  O_tx_data,
    input  wire        I_tx_ready
);

    localparam integer XLO = CX - HALF;
    localparam integer XHI = CX + HALF - 1;
    localparam integer YLO = CY - HALF;
    localparam integer YHI = CY + HALF - 1;

    // ---- vsync rising-edge detector (synthesis-friendly: two separate blocks) ----
    reg vs_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) vs_d <= 1'b0;
        else          vs_d <= I_vsync;
    end
    wire vs_edge = I_vsync & ~vs_d;

    // ---- internal x/y counters (the probe only needs its own window) ---------
    reg [11:0] px, py;
    reg        line_seen;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            px <= 12'd0; py <= 12'd0; line_seen <= 1'b0;
        end else if (vs_edge) begin
            px <= 12'd0; py <= 12'd0; line_seen <= 1'b0;
        end else if (I_de) begin
            if (!line_seen) begin
                px <= 12'd1; py <= py + 12'd1; line_seen <= 1'b1;
            end else begin
                px <= px + 12'd1;
            end
        end else begin
            line_seen <= 1'b0;
        end
    end

    // ---- window accumulation -------------------------------------------------
    wire in_win = I_de && (px >= XLO) && (px <= XHI) && (py >= YLO) && (py <= YHI);

    reg [17:0] acc_r, acc_g, acc_b;
    reg [7:0]  avg_r, avg_g, avg_b;
    reg        avg_valid;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            acc_r <= 18'd0; acc_g <= 18'd0; acc_b <= 18'd0;
            avg_r <= 8'd0;  avg_g <= 8'd0;  avg_b <= 8'd0;
            avg_valid <= 1'b0;
        end else if (vs_edge) begin
            avg_r     <= acc_r[15:8];       // 256 pixels
            avg_g     <= acc_g[15:8];
            avg_b     <= acc_b[15:8];
            avg_valid <= 1'b1;
            acc_r <= 18'd0; acc_g <= 18'd0; acc_b <= 18'd0;
        end else if (in_win) begin
            acc_r <= acc_r + {10'd0, I_rgb[23:16]};
            acc_g <= acc_g + {10'd0, I_rgb[15:8]};
            acc_b <= acc_b + {10'd0, I_rgb[7:0]};
        end
    end

    // ---- packet transmitter (4-phase handshake) ------------------------------
    // The host side asserts I_tx_ready while free; it registers the byte and drops ready for
    // one cycle.  Advance only after seeing that drop, otherwise the next byte overwrites the
    // previous one before it is taken (that is what truncated A5 5A R G B 0D to A5 + 2 bytes).
    localparam [2:0] P_IDLE = 3'd0, P_PRESENT = 3'd1, P_TAKE = 3'd2, P_GAP = 3'd3;

    reg [2:0] st;
    reg [2:0] bidx;
    reg       trig_d;
    wire      trig_rise = I_trig & ~trig_d;

    function [7:0] byte_of;
        input [2:0] b;
        begin
            case (b)
                3'd0: byte_of = 8'hA5;
                3'd1: byte_of = 8'h5A;
                3'd2: byte_of = avg_r;
                3'd3: byte_of = avg_g;
                3'd4: byte_of = avg_b;
                default: byte_of = 8'h0D;
            endcase
        end
    endfunction

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st         <= P_IDLE;
            bidx       <= 3'd0;
            trig_d     <= 1'b0;
            O_tx_valid <= 1'b0;
            O_tx_data  <= 8'd0;
            O_busy     <= 1'b0;
            O_done     <= 1'b0;
        end else begin
            trig_d <= I_trig;
            O_done <= 1'b0;
            case (st)
                P_IDLE: begin
                    O_tx_valid <= 1'b0;
                    O_busy     <= 1'b0;
                    if (trig_rise && avg_valid) begin
                        bidx   <= 3'd0;
                        st     <= P_PRESENT;
                        O_busy <= 1'b1;
                    end
                end
                P_PRESENT: begin
                    O_tx_valid <= 1'b1;
                    O_tx_data  <= byte_of(bidx);
                    st         <= P_TAKE;
                end
                P_TAKE: begin
                    if (!I_tx_ready) begin      // receiver took the byte
                        O_tx_valid <= 1'b0;
                        st         <= P_GAP;
                    end
                end
                P_GAP: begin
                    if (I_tx_ready) begin       // transmitter is free again
                        if (bidx == 3'd5) begin
                            st     <= P_IDLE;
                            O_done <= 1'b1;
                            O_busy <= 1'b0;
                        end else begin
                            bidx <= bidx + 3'd1;
                            st   <= P_PRESENT;
                        end
                    end
                end
                default: st <= P_IDLE;
            endcase
        end
    end

endmodule
