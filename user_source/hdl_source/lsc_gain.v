`timescale 1ns / 1ps
/*****************************************************************
 * lsc_gain.v -- affine radial colour-shading correction (2026-10-18)
 *
 * Model (per channel, s = r2 normalised to 0..255):
 *     Gc(s) = (256 - C)      + ( +1.00 * E * s >> 8)
 *     Rc(s) = (256 + C/2)    + ( -0.45 * E * s >> 8)
 *     Bc(s) = 256                                   (blue untouched)
 *
 * Pipelined in 3 stages so that no stage contains more than one multiplier:
 *   p1: dx*dx, dy*dy  (registered)
 *   p2: s = (dx2+dy2) >> 12, clamped
 *   p3: gain = a + (b*s >> 8), and pixel * gain, both registered
 * The pixel is delayed by 3 cycles to stay aligned with its gain.
 * Latency = 3 pixel clocks (image shifts right by 3 px relative to x/y counters).
 *****************************************************************/
module lsc_gain #(
    parameter integer CX = 640,
    parameter integer CY = 360,
    parameter integer ANCHOR = 0        // 0 = affine (A link), 1 = anchored (B link)
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_de,
    input  wire [10:0] I_x,
    input  wire [9:0]  I_y,
    input  wire [23:0] I_rgb,
    input  wire        I_en,          // 0 = passthrough
    input  wire [7:0]  I_edge,        // E : 0..255  (0 .. 1.0)
    input  wire [7:0]  I_centre,      // C : 0..255  (0 .. 1.0)
    input  wire [7:0]  I_lift,        // b : 0..255  (0 .. 1.0) global blue cut
    input  wire [4:0]  I_sat,         // saturation, 16 = unity (applied before detection)
    output reg  [23:0] O_rgb
);
    // ---------------- stage 1: squares ----------------
    wire signed [12:0] dx = $signed({2'b0, I_x}) - $signed(CX[10:0]);
    wire signed [11:0] dy = $signed({2'b0, I_y}) - $signed(CY[9:0]);

    reg [23:0] dx2_r, dy2_r;
    reg [23:0] rgb_d1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            dx2_r  <= 24'd0;
            dy2_r  <= 24'd0;
            rgb_d1 <= 24'd0;
        end else begin
            dx2_r  <= dx * dx;
            dy2_r  <= dy * dy;
            rgb_d1 <= I_rgb;
        end
    end

    // ---------------- stage 2: normalised radius ----------------
    wire [23:0] s_sum = dx2_r + dy2_r;
    reg [7:0]  s_r;
    reg [23:0] rgb_d2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            s_r    <= 8'd0;
            rgb_d2 <= 24'd0;
        end else begin
            // r2_max = 640^2+360^2 = 539200.  >>11 gives 263 at the corner, so clamp at 255
            // (>>12 only reached 131 = 51%, which left the diagonal corners under-corrected)
            s_r    <= (s_sum[19:11] > 8'd255) ? 8'd255 : s_sum[19:11];
            rgb_d2 <= rgb_d1;
        end
    end

    // ---------------- stage 3: gains and apply ----------------
    // Model A14 (2026-10-23): K is hard-wired (see K_Q8 below); the three OSD knobs are E/C/b.
    //   G(s) = (1 - C) + (C + K)*s
    //   R(s) =    1    + (K - E)*s
    //   B(s) = (1 - b) + K*s
    //
    // Model A13 (2026-10-23):
    //   G(s) = (1 - C) + C*s        C = centre green cut, fades back to 1.0 at the corner
    //   R(s) =    1    - E*s        E = edge red cut
    //   B(s) =    (1 - b)           b = GLOBAL blue cut (constant, no radial term)
    // item13 = E, item14 = C, item15 = b (was K)
    // A global blue cut changes the B/R and B/G ratios exactly like raising R and G by the
    // same factor, but it never pushes red/green further into clipping.
    wire signed [11:0] a_g = 12'sd256 - {4'd0, I_centre};    // 1 - C
    wire signed [11:0] b_g = {4'd0, I_centre};               // +C  (fades back to 1.0)
    wire signed [11:0] a_r = 12'sd256;                       // 1
    wire signed [11:0] b_r = -{4'd0, I_edge};                // -E  (edge red cut)
    wire signed [11:0] b_const = 12'sd256 - {4'd0, I_lift};  // 1 - b (global blue)
    // K is a fixed common brightness lift (0.195), no OSD knob
    localparam signed [11:0] K_Q8 = 12'sd50;
    wire signed [19:0] lift_slope = (K_Q8 * $signed({4'd0, s_r})) >>> 8;

    // Exact range 1..305 for every 8-bit E/C/b and radius. Signed 10 bits suffice.
    reg signed [9:0] gain_g_r, gain_r_r, gain_b_r;
    reg [23:0]        rgb_d3;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            gain_g_r <= 20'sd256;
            gain_r_r <= 20'sd256;
            gain_b_r <= 20'sd256;
            rgb_d3   <= 24'd0;
        end else begin
            gain_g_r <= $signed({8'd0, a_g}) + ((b_g * $signed({4'd0, s_r})) >>> 8) + lift_slope;
            gain_r_r <= $signed({8'd0, a_r}) + ((b_r * $signed({4'd0, s_r})) >>> 8) + lift_slope;
            // All three gains belong to rgb_d3, not the next pixel's radius.
            gain_b_r <= b_const + lift_slope;
            rgb_d3   <= rgb_d2;
        end
    end

    // ---------------- output stage ----------------
    wire signed [19:0] g_out = ($signed({12'd0, rgb_d3[15:8]}) * gain_g_r) >>> 8;
    wire signed [19:0] r_out = ($signed({12'd0, rgb_d3[23:16]}) * gain_r_r) >>> 8;

    wire signed [19:0] b_out = ($signed({12'd0, rgb_d3[7:0]}) * gain_b_r) >>> 8;
    wire [7:0] b_clip = (b_out > 20'sd255) ? 8'd255 : ((b_out < 20'sd0) ? 8'd0 : b_out[7:0]);

    wire [7:0] g_clip = (g_out > 20'sd255) ? 8'd255 : ((g_out < 20'sd0) ? 8'd0 : g_out[7:0]);
    wire [7:0] r_clip = (r_out > 20'sd255) ? 8'd255 : ((r_out < 20'sd0) ? 8'd0 : r_out[7:0]);

    // ---------------- stage 4: saturation -------------------------------------------------
    // C' = Y + (C - Y) * sat / 16, Y = (77R + 150G + 29B) >> 8.
    // Sign extension MUST be explicit: {6'd0, dr[11:0]} would turn -10 into +1014 and wreck
    // the colours. Use {{6{dr[11]}}, dr}.
    reg [23:0] rgb_d4;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)  rgb_d4 <= 24'd0;
        else if (I_en) rgb_d4 <= {r_clip, g_clip, b_clip};
        else           rgb_d4 <= rgb_d3;
    end

    wire [15:0] lum_s = 16'd77  * {8'd0, rgb_d4[23:16]}
                      + 16'd150 * {8'd0, rgb_d4[15:8]}
                      + 16'd29  * {8'd0, rgb_d4[7:0]};
    wire [7:0]  y_s   = lum_s[15:8];
    wire signed [11:0] dr_s = $signed({4'd0, rgb_d4[23:16]}) - $signed({4'd0, y_s});
    wire signed [11:0] dg_s = $signed({4'd0, rgb_d4[15:8]})  - $signed({4'd0, y_s});
    wire signed [11:0] db_s = $signed({4'd0, rgb_d4[7:0]})   - $signed({4'd0, y_s});
    wire signed [17:0] dr_e = {{6{dr_s[11]}}, dr_s};
    wire signed [17:0] dg_e = {{6{dg_s[11]}}, dg_s};
    wire signed [17:0] db_e = {{6{db_s[11]}}, db_s};
    wire signed [22:0] r_p = dr_e * $signed({1'b0, I_sat});
    wire signed [22:0] g_p = dg_e * $signed({1'b0, I_sat});
    wire signed [22:0] b_p = db_e * $signed({1'b0, I_sat});
    wire signed [18:0] r_s = $signed({11'd0, y_s}) + (r_p >>> 4);
    wire signed [18:0] g_s = $signed({11'd0, y_s}) + (g_p >>> 4);
    wire signed [18:0] b_s = $signed({11'd0, y_s}) + (b_p >>> 4);
    wire [7:0] r_sc = (r_s > 19'sd255) ? 8'd255 : ((r_s < 19'sd0) ? 8'd0 : r_s[7:0]);
    wire [7:0] g_sc = (g_s > 19'sd255) ? 8'd255 : ((g_s < 19'sd0) ? 8'd0 : g_s[7:0]);
    wire [7:0] b_sc = (b_s > 19'sd255) ? 8'd255 : ((b_s < 19'sd0) ? 8'd0 : b_s[7:0]);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) O_rgb <= 24'd0;
        else          O_rgb <= {r_sc, g_sc, b_sc};
    end
endmodule
