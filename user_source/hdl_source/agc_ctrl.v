`timescale 1ns / 1ps
// agc_ctrl -- automatic gain control for the SC520CS, driven by luma_stats.
//
// Control law (2026-10-13, "keep it a little bright" revision):
//   * ASYMMETRIC tolerance. Brightening starts only UP_BAND below the target, but darkening
//     waits until DOWN_BAND above it -- and DOWN_BAND is much larger on purpose. A bright
//     picture therefore does NOT trigger a dimming correction, so the loop parks slightly
//     bright and the "bright -> dim -> dark -> brighten" cycle cannot start. This was the
//     user's request: widen the tolerance towards "too bright".
//   * BRIGHT-SIDE HYSTERESIS: when it does darken, it stops as soon as the error falls back to
//     DOWN_STOP, so a correction removes only a little brightness.
//   * FAST ATTACK is for BRIGHTENING ONLY (an abrupt darkening is what hurts visibility);
//     a sudden over-exposure is handled promptly by the sat_high rule instead.
//   * PERSISTENCE: between the dead band and the fast threshold the same direction must repeat
//     for PERSIST windows, so content jitter cannot pump the gain.
//   * ASYMMETRIC STEPS: brighten fast (|err|>>DIV_FAST, capped STEP_MAX_FAST), darken gently
//     (|err|>>DIV_SLOW, capped STEP_MAX_DOWN).
//   * Manual mode takes over seamlessly (I_load). O_req is a one-clock write request matching
//     the I2C handshake in uicfgcs520; the controller owns only the gain control value.
module agc_ctrl #(
    parameter [7:0]   UP_BAND       = 8'd20,   // brighten when mean < TARGET - UP_BAND
    parameter [7:0]   DOWN_BAND     = 8'd60,   // darken only when mean > TARGET + DOWN_BAND
                                               // (wide: this is the "keep it bright" margin)
    parameter [7:0]   STOP_BAND     = 8'd20,   // dark-side hysteresis: stop brightening here
    parameter [7:0]   DOWN_STOP     = 8'd40,   // bright-side hysteresis: stop darkening here
    parameter [7:0]   FAST_BAND     = 8'd60,   // brightening faster than this acts at once
    parameter [9:0]   STEP_MAX_FAST = 10'd24,
    parameter [9:0]   STEP_MAX_SLOW = 10'd16,
    parameter [9:0]   STEP_MAX_DOWN = 10'd16,
    parameter [3:0]   DIV_FAST      = 4'd2,
    parameter [3:0]   DIV_SLOW      = 4'd2,
    parameter [3:0]   PERSIST       = 4'd2,
    parameter [23:0]  MIN_VALID     = 24'd32768,
    parameter [15:0]  GAIN_MIN      = 16'd16,
    parameter [15:0]  GAIN_MAX      = 16'd175,
    parameter [15:0]  GAIN_RESET    = 16'd96,
    parameter [23:0]  SAT_LIMIT     = 24'd50000,
    parameter integer FRAME_SHIFT   = 4             //减小加响应速度
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        I_enable,
    input  wire        I_hold,          // 1 = freeze the gain step (dual-loop arbiter)
    input  wire        I_stats_valid,
    input  wire [7:0]  I_y_mean,
    input  wire [7:0]  I_target_y,     // runtime brightness target (shared)
    input  wire [23:0] I_sat_count,
    input  wire [23:0] I_valid_count,
    input  wire        I_load,
    input  wire [15:0] I_load_val,
    input  wire        I_coord,        // 1 = coordinated AEC/AGC (both AE & AG auto): gain is a filler
    input  wire        I_exp_sat,      // 1 = exposure at comfort cap (COORD_EXP_MAX) or 50 Hz flicker pin -> gain may rise
    output reg  [15:0] O_gain_val,
    output reg         O_req
);
    reg  [24:0] acc_y;
    reg  [23+FRAME_SHIFT:0] acc_sat;   // 2^FRAME_SHIFT samples of 24 bits
    reg  [3:0]  fr_cnt;
    wire [24:0] sum_y   = acc_y   + {17'd0, I_y_mean};
    wire [23+FRAME_SHIFT:0] sum_sat = acc_sat + {{FRAME_SHIFT{1'b0}}, I_sat_count};
    wire [7:0]  avg_y   = sum_y[FRAME_SHIFT+7 : FRAME_SHIFT];
    wire [23:0] avg_sat = sum_sat[FRAME_SHIFT+23 : FRAME_SHIFT];

    wire sat_high = (avg_sat > SAT_LIMIT);
    wire trust    = (I_valid_count >= MIN_VALID);

    wire signed [9:0] err = $signed({2'b0, avg_y}) - $signed({2'b0, I_target_y});
    wire [9:0] abs_err = (err > 0) ? err[9:0] : (~err[9:0] + 10'd1);
    wire too_dark   = (err < -$signed({2'b0, UP_BAND}));
    wire too_bright = (err >  $signed({2'b0, DOWN_BAND}));
    wire inside_stop= (err <= $signed({2'b0, DOWN_STOP})) && (err >= -$signed({2'b0, STOP_BAND}));
    wire fast_err   = (err < -$signed({2'b0, FAST_BAND}));   // brightening fast path only

    wire [9:0] up_fast_raw = abs_err >> DIV_FAST;
    wire [9:0] up_slow_raw = abs_err >> DIV_SLOW;
    wire [9:0] dn_raw      = abs_err >> DIV_SLOW;
    wire [9:0] step_up_fast = (up_fast_raw < 10'd1) ? 10'd1 : ((up_fast_raw > STEP_MAX_FAST) ? STEP_MAX_FAST : up_fast_raw);
    wire [9:0] step_up_slow = (up_slow_raw < 10'd1) ? 10'd1 : ((up_slow_raw > STEP_MAX_SLOW) ? STEP_MAX_SLOW : up_slow_raw);
    wire [9:0] step_down    = (dn_raw      < 10'd1) ? 10'd1 : ((dn_raw      > STEP_MAX_DOWN) ? STEP_MAX_DOWN : dn_raw);

    reg  [1:0] dir_prev;
    reg  [2:0] dir_run;
    reg        acting;
    wire [1:0] dir_now = too_bright ? 2'b10 : (too_dark ? 2'b01 : 2'b00);
    wire persist_ok = (dir_now != 2'b00) && (dir_now == dir_prev) && (dir_run >= PERSIST - 1);
    wire act_now = fast_err || persist_ok || sat_high ||
                   (acting && !inside_stop && (dir_now == dir_prev) && (dir_now != 2'b00));
    wire want_down = (err > 0) || sat_high;

    // ---- coordinated AEC/AGC (both auto): the gain loop becomes a FILLER ----
    // Exposure (ae_ctrl) is moved first. Gain may only RISE as a last resort (too dark AND
    // exposure already saturated); otherwise it BLEEDS DOWN toward GAIN_MIN so that exposure
    // does the brightness work. This realises the user's "exposure-first, gain-last" ladder
    // and stops the two independent loops from fighting over the same brightness target.
    wire exp_sat_term = I_exp_sat;
    wire coord_rise   = too_dark && exp_sat_term && (O_gain_val < GAIN_MAX);
    wire coord_bleed  = (err > 0) && (O_gain_val > GAIN_MIN);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            O_gain_val <= GAIN_RESET;
            O_req      <= 1'b0;
            acc_y <= 25'd0; acc_sat <= {24+FRAME_SHIFT{1'b0}}; fr_cnt <= 4'd0;
            dir_prev <= 2'b00; dir_run <= 3'd0; acting <= 1'b0;
        end else begin
            O_req <= 1'b0;
            if (I_load) begin
                O_gain_val <= I_load_val;
                acc_y <= 25'd0; acc_sat <= {24+FRAME_SHIFT{1'b0}}; fr_cnt <= 4'd0;
                dir_prev <= 2'b00; dir_run <= 3'd0; acting <= 1'b0;
            end else if (I_enable && I_stats_valid && trust) begin
                acc_y   <= sum_y;
                acc_sat <= sum_sat;
                if (fr_cnt == ((1<<FRAME_SHIFT)-1)) begin
                    fr_cnt  <= 4'd0;
                    acc_y   <= 25'd0;
                    acc_sat <= {24+FRAME_SHIFT{1'b0}};
                    if (dir_now == dir_prev) dir_run <= (dir_run == 3'd7) ? 3'd7 : (dir_run + 3'd1);
                    else begin dir_run <= 3'd1; acting <= 1'b0; end
                    dir_prev <= dir_now;
                    if (!acting && (fast_err || persist_ok || sat_high)) acting <= 1'b1;
                    else if (acting && inside_stop && !sat_high)         acting <= 1'b0;
                    if (I_coord) begin
                        // ---- coordinated AEC/AGC: exposure-first, gain is a minimal filler ----
                        if (!I_hold) begin   // dual-loop arbiter: freeze stepping
                        if (coord_rise) begin
                            O_gain_val <= (O_gain_val < (GAIN_MAX - {6'd0, step_up_slow})) ?
                                          (O_gain_val + {6'd0, step_up_slow}) : GAIN_MAX;
                            O_req      <= 1'b1;
                        end else if (coord_bleed) begin
                            O_gain_val <= (O_gain_val > (GAIN_MIN + {6'd0, step_down})) ?
                                          (O_gain_val - {6'd0, step_down}) : GAIN_MIN;
                            O_req      <= 1'b1;
                        end
                        end
                    end else begin
                        // ---- independent AGC (AG-only auto): chase its own brightness target ----
                        if (!I_hold) begin   // dual-loop arbiter: freeze stepping
                        if (act_now && want_down && (O_gain_val > GAIN_MIN)) begin
                            O_gain_val <= (O_gain_val > (GAIN_MIN + {6'd0, step_down})) ? (O_gain_val - {6'd0, step_down}) : GAIN_MIN;
                            O_req      <= 1'b1;
                        end else if (act_now && !want_down && (O_gain_val < GAIN_MAX)) begin
                            O_gain_val <= (O_gain_val < (GAIN_MAX - {6'd0, (fast_err ? step_up_fast : step_up_slow)})) ?
                                          (O_gain_val + {6'd0, (fast_err ? step_up_fast : step_up_slow)}) : GAIN_MAX;
                            O_req      <= 1'b1;
                        end
                        end
                    end
                end else begin
                    fr_cnt <= fr_cnt + 4'd1;
                end
            end
        end
    end
endmodule
