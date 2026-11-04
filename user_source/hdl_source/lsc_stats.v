`timescale 1ns / 1ps
/*****************************************************************
 * lsc_stats.v -- radial lens-shading statistics collector (2026-10-18)
 *
 * Purpose: measure the radial per-channel brightness profile so that an
 * LSC gain table can be calibrated without a capture card.
 *
 * - 32 radial bins: bin = clamp(d2 >> 14, 0, 31)
 *                        d2 = (x-CX)^2 + (y-CY)^2    (1280x720 assumed)
 * - 3 independent accumulators per bin (R, G, B), each a single-port
 *   read-modify-write RAM, so one pixel per clock is enough
 * - accumulates NFRAMES frames, then streams 96 values (32 bins x 3 ch)
 *   as 2 bytes each, framed A5 5A <data...> 0D 0A
 * - dump uses valid/ready so it can drive a UART transmitter directly
 *****************************************************************/
module lsc_stats #(
    parameter integer NFRAMES = 8,      // frames to accumulate
    parameter integer SHIFT   = 9,      // sum >> SHIFT -> 16-bit report
    parameter integer CX      = 640,
    parameter integer CY      = 360
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    // video timing (pixel domain, 1 pixel per clock)
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire [7:0]  I_r,
    input  wire [7:0]  I_g,
    input  wire [7:0]  I_b,
    // control
    input  wire        I_trig,          // one-clock pulse: start a capture
    output wire        O_busy,
    output reg         O_done,          // one-clock pulse after the dump
    // dump stream
    output reg         O_tx_valid,
    output reg  [7:0]  O_tx_data,
    input  wire        I_tx_ready
);
    localparam integer ADDRS = 96;      // 32 bins x 3 channels

    // ---------------- x / y position counters ----------------
    reg [10:0] x_cnt;
    reg [9:0]  y_cnt;
    reg        de_d;

    wire de_rise = I_de & ~de_d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            x_cnt <= 11'd0;
            y_cnt <= 10'd0;
            de_d  <= 1'b0;
        end else begin
            de_d <= I_de;
            if (de_rise)            x_cnt <= 11'd0;
            else if (I_de)          x_cnt <= x_cnt + 11'd1;
            if (!I_de && de_d)      y_cnt <= y_cnt + 10'd1;
            else if (I_vsync)       y_cnt <= 10'd0;
        end
    end

    // ---------------- radius squared (2 multipliers) ----------------
    wire signed [12:0] dx = $signed({2'b0, x_cnt}) - $signed(CX[10:0]);
    wire signed [11:0] dy = $signed({2'b0, y_cnt}) - $signed(CY[9:0]);
    wire [23:0] dx2 = dx * dx;
    wire [23:0] dy2 = dy * dy;
    wire [23:0] d2  = dx2 + dy2;

    wire [6:0] bin_raw = d2[20:14];
    wire [4:0] bin     = (bin_raw > 7'd31) ? 5'd31 : bin_raw[4:0];

    // ---------------- three accumulator banks ----------------
    reg [25:0] acc_r [0:31];
    reg [25:0] acc_g [0:31];
    reg [25:0] acc_b [0:31];

    reg        collecting;
    reg        captured;               // at least one capture finished
    reg [3:0]  frame_cnt;
    reg        vs_d;
    integer    i;

    localparam [2:0] D_IDLE = 3'd0, D_HDR1 = 3'd1, D_HDR2 = 3'd2,
                     D_ADDR = 3'd3, D_DATA = 3'd4, D_END = 3'd5, D_CR = 3'd6;

    reg [2:0]  st;

    always @(posedge I_clk) begin
        if (I_trig && !collecting) begin
            for (i = 0; i < 32; i = i + 1) begin
                acc_r[i] <= 26'd0;
                acc_g[i] <= 26'd0;
                acc_b[i] <= 26'd0;
            end
        end else if (I_de && collecting) begin
            acc_r[bin] <= acc_r[bin] + {18'd0, I_r};
            acc_g[bin] <= acc_g[bin] + {18'd0, I_g};
            acc_b[bin] <= acc_b[bin] + {18'd0, I_b};
        end
    end

    // dump_done: one-cycle pulse at the end of the dump, used to clear `captured`
    wire dump_done = (st == D_END);

    // ---------------- frame counting ----------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            collecting <= 1'b0;
            captured   <= 1'b0;
            frame_cnt  <= 4'd0;
            vs_d       <= 1'b0;
        end else begin
            vs_d <= I_vsync;
            if (I_trig && !collecting) begin
                collecting <= 1'b1;
                captured   <= 1'b0;
                frame_cnt  <= 4'd0;
            end else if (collecting && I_vsync && !vs_d) begin
                if (frame_cnt >= NFRAMES-1) begin
                    collecting <= 1'b0;
                    captured   <= 1'b1;     // dump may start
                end else begin
                    frame_cnt <= frame_cnt + 4'd1;
                end
            end else if (dump_done) begin
                captured <= 1'b0;           // single shot: do not dump again
            end
        end
    end

    // ---------------- dump state machine ----------------
    reg [6:0]  rd_addr;                 // {channel[1:0], bin[4:0]}
    reg [15:0] val_r;
    reg [1:0]  byte_sel;

    // O_busy driven from one place only
    assign O_busy = collecting || (st != D_IDLE);

    wire [4:0]  rbin = rd_addr[4:0];
    wire [1:0]  rch  = rd_addr[6:5];
    wire [25:0] racc = (rch == 2'd1) ? acc_g[rbin] :
                       (rch == 2'd2) ? acc_b[rbin] : acc_r[rbin];
    wire [15:0] report = racc >> SHIFT;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            st         <= D_IDLE;
            rd_addr    <= 7'd0;
            val_r      <= 16'd0;
            byte_sel   <= 2'd0;
            O_tx_valid <= 1'b0;
            O_tx_data  <= 8'd0;
            O_done     <= 1'b0;
        end else begin
            O_tx_valid <= 1'b0;
            O_done     <= 1'b0;
            case (st)
                D_IDLE: begin
                    rd_addr  <= 7'd0;
                    byte_sel <= 2'd0;
                    if (captured && !collecting) st <= D_HDR1;
                end
                D_HDR1: if (I_tx_ready) begin O_tx_valid <= 1'b1; O_tx_data <= 8'hA5; st <= D_HDR2; end
                D_HDR2: if (I_tx_ready) begin O_tx_valid <= 1'b1; O_tx_data <= 8'h5A; st <= D_ADDR; end
                D_ADDR: begin
                    val_r <= report;                    // latch this accumulator
                    st    <= D_DATA;
                end
                D_DATA: begin
                    if (I_tx_ready) begin
                        O_tx_valid <= 1'b1;
                        O_tx_data  <= byte_sel[0] ? val_r[7:0] : val_r[15:8];
                        if (byte_sel[0]) begin
                            byte_sel <= 2'd0;
                            if (rd_addr == ADDRS-1) st <= D_CR;
                            else begin rd_addr <= rd_addr + 7'd1; st <= D_ADDR; end
                        end else begin
                            byte_sel <= 2'd1;
                        end
                    end
                end
                D_CR:   if (I_tx_ready) begin O_tx_valid <= 1'b1; O_tx_data <= 8'h0D; st <= D_END; end
                D_END:  begin O_done <= 1'b1; st <= D_IDLE; end
                default: st <= D_IDLE;
            endcase
        end
    end
endmodule
