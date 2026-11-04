`timescale 1ns / 1ps
// ae_ctrl -- exposure (AE) control for the SC520CS, driven by the same luma_stats as the gain.
//
// Exposure unit is HALF a line (datasheet), i.e. 4.1667 us at 720p60 (VTS = 2000, 8.333 us per
// line, 16.67 ms per frame). The register field is {0x3e00[3:0], 0x3e01[7:0], 0x3e02[7:4]} and
// must never exceed 2*VTS-9 = 3991, otherwise the frame rate drops -- clamped here.
//
// Manual mode walks a 15-rung ladder (user request 2026-10-16): the OSD shows rung 1..15, mapped
// linearly to exposure 100..1991 (1/2-line units). Power-up rung = 10 (exp 1315 ~ 11.0 ms).
//   rung :  1    2    3    4    5    6    7    8    9   10   11   12   13   14   15
//   exp  : 100  235  370  505  640  775  910 1045 1180 1315 1450 1585 1720 1856 1991  (1/2-line)
// Preset 4 (2400) is exactly half of the 50 Hz mains period, so with anti-flicker enabled the
// exposure is pinned there. Note the consequence: at 60 fps only 10 ms fits, so with flicker
// control on the exposure cannot be adjusted at all and brightness must come from the gain.
//
// The displayed value is in 0.1 ms units (the font has no decimal point): 10, 20, 40, 80, 100,
// 166. Automatic mode steps one preset per decision window (2^FRAME_SHIFT frames), with the same
// persistence rule as the gain loop so that content jitter cannot move it.
module ae_ctrl #(
    parameter [3:0]   DEFAULT_IDX  = 4'd15,       // power-up rung 15 (top rung, exposure-first)
    parameter [15:0]  AE_MAX       = 16'd1991,    // 2*VTS-9 with VTS = 1000 (as programmed)
    parameter [15:0]  FLICKER_VAL  = 16'd1200,    // 10 ms: half of the 50 Hz period
    parameter [3:0]   COORD_EXP_MAX= 4'd15,       // coordinated auto-exposure comfort cap (rung 1..15):
                                                 // caps rolling-shutter blur; gain fills beyond it
    // 15-rung ladder, rung 1..15 mapped linearly to 100..1991 (1/2-line units)
    parameter [15:0]  PRE1  = 16'd100,  PRE2  = 16'd235,  PRE3  = 16'd370,
    parameter [15:0]  PRE4  = 16'd505,  PRE5  = 16'd640,  PRE6  = 16'd775,
    parameter [15:0]  PRE7  = 16'd910,  PRE8  = 16'd1045, PRE9  = 16'd1180,
    parameter [15:0]  PRE10 = 16'd1315, PRE11 = 16'd1450, PRE12 = 16'd1585,
    parameter [15:0]  PRE13 = 16'd1720, PRE14 = 16'd1856, PRE15 = 16'd1991,
    parameter [7:0]   DSP1  = 8'd8,    DSP2  = 8'd20,   DSP3  = 8'd31,
    parameter [7:0]   DSP4  = 8'd42,   DSP5  = 8'd53,   DSP6  = 8'd65,
    parameter [7:0]   DSP7  = 8'd76,   DSP8  = 8'd87,   DSP9  = 8'd98,
    parameter [7:0]   DSP10 = 8'd110,  DSP11 = 8'd121,  DSP12 = 8'd132,
    parameter [7:0]   DSP13 = 8'd143,  DSP14 = 8'd155,  DSP15 = 8'd166,
    parameter [7:0]   ACT_BAND    = 8'd20,        // start correcting beyond this error
    parameter [7:0]   STOP_BAND   = 8'd12,        // ...stop once back inside this (hysteresis)
    parameter [3:0]   PERSIST     = 4'd2,         // same-direction windows before acting
    parameter [23:0]  MIN_VALID   = 24'd32768,    // trust gate on the measured pixel count
    parameter integer FRAME_SHIFT = 2             // decide once every 2^FRAME_SHIFT frames (4)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        I_enable,        // 1 = automatic exposure, 0 = manual preset
    input  wire        I_hold,          // 1 = freeze the automatic step (dual-loop arbiter)
    input  wire [3:0]  I_man_idx,       // manual preset index from the UI (1..15)
    input  wire        I_flicker,       // 1 = 50 Hz anti-flicker: pin the exposure to 10 ms
    input  wire        I_stats_valid,
    input  wire [7:0]  I_y_mean,        // IIR-filtered window mean
    input  wire [23:0] I_valid_count,
    input  wire [7:0]  I_target_y,      // runtime brightness target (shared)
    input  wire        I_load,          // one-clock pulse: snap to the preset nearest I_load_val
    input  wire [15:0] I_load_val,
    input  wire        I_go,            // 1 once sensor config is done: safe to write registers
    output reg  [15:0] O_ae_val,        // exposure control value (1/2 line units)
    output reg  [7:0]  O_disp,          // value to show, in 0.1 ms units
    output reg         O_req,           // one-clock write request
    output wire [3:0]  O_idx,           // applied rung (1..15): OSD display + coordination
    output wire        O_exp_sat        // exposure effectively saturated (rung 15 or flicker-pinned)
);
    function [15:0] pre_val(input [3:0] i);
        case (i)
            4'd1:  pre_val = PRE1;  4'd2:  pre_val = PRE2;  4'd3:  pre_val = PRE3;
            4'd4:  pre_val = PRE4;  4'd5:  pre_val = PRE5;  4'd6:  pre_val = PRE6;
            4'd7:  pre_val = PRE7;  4'd8:  pre_val = PRE8;  4'd9:  pre_val = PRE9;
            4'd10: pre_val = PRE10; 4'd11: pre_val = PRE11; 4'd12: pre_val = PRE12;
            4'd13: pre_val = PRE13; 4'd14: pre_val = PRE14; default: pre_val = PRE15;
        endcase
    endfunction
    function [7:0] pre_dsp(input [3:0] i);
        case (i)
            4'd1:  pre_dsp = DSP1;  4'd2:  pre_dsp = DSP2;  4'd3:  pre_dsp = DSP3;
            4'd4:  pre_dsp = DSP4;  4'd5:  pre_dsp = DSP5;  4'd6:  pre_dsp = DSP6;
            4'd7:  pre_dsp = DSP7;  4'd8:  pre_dsp = DSP8;  4'd9:  pre_dsp = DSP9;
            4'd10: pre_dsp = DSP10; 4'd11: pre_dsp = DSP11; 4'd12: pre_dsp = DSP12;
            4'd13: pre_dsp = DSP13; 4'd14: pre_dsp = DSP14; default: pre_dsp = DSP15;
        endcase
    endfunction

    reg  [3:0]  idx_r;
    reg  [24:0] acc_y;
    reg  [3:0]  fr_cnt;
    reg  [1:0]  dir_prev;
    reg  [2:0]  dir_run;
    reg  [3:0]  man_prev;    // the UI index is an EVENT: only a change takes over
    reg  [15:0] wr_val;      // last value we asked the sensor to take (closed-loop write)
    wire [24:0] sum_y = acc_y + {17'd0, I_y_mean};
    wire [7:0]  avg_y = sum_y[FRAME_SHIFT+7 : FRAME_SHIFT];

    wire trust   = (I_valid_count >= MIN_VALID);
    wire signed [9:0] err = $signed({2'b0, avg_y}) - $signed({2'b0, I_target_y});
    wire too_dark   = (err < -$signed({2'b0, ACT_BAND}));
    wire too_bright = (err >  $signed({2'b0, ACT_BAND}));
    wire inside_stop= (err <= $signed({2'b0, STOP_BAND})) && (err >= -$signed({2'b0, STOP_BAND}));

    wire [1:0] dir_now  = too_bright ? 2'b10 : (too_dark ? 2'b01 : 2'b00);
    wire persist_ok = (dir_now != 2'b00) && (dir_now == dir_prev) && (dir_run >= PERSIST - 1);

    // what manual mode publishes (and what the UI index asks for, clamped to the ladder)
    wire [3:0] man_idx = (I_man_idx > 4'd15) ? 4'd15 : ((I_man_idx < 4'd1) ? 4'd1 : I_man_idx);
    // In manual mode the published value follows the UI rung DIRECTLY (no register in the path),
    // so a stuck rung -- or a display that disagrees with what is published -- is impossible.
    wire [3:0]  sel_idx  = I_enable ? idx_r : man_idx;
    wire [15:0] pub_val  = I_flicker ? FLICKER_VAL : pre_val(sel_idx);
    wire [7:0]  pub_disp = I_flicker ? 8'd100      : pre_dsp(sel_idx);
    assign O_idx     = sel_idx;                          // rung actually applied (auto idx_r / manual man_idx)
    assign O_exp_sat = (idx_r >= COORD_EXP_MAX) || I_flicker;  // exposure capped (blur-safe) or flicker-pinned

    // nearest preset to a handover value
    function [3:0] near_idx(input [15:0] v);
        reg [15:0] d, best;
        integer k;
        begin
            near_idx = 4'd1; best = 16'hFFFF;
            for (k = 1; k < 16; k = k + 1) begin
                d = (pre_val(k[3:0]) > v) ? (pre_val(k[3:0]) - v) : (v - pre_val(k[3:0]));
                if (d < best) begin best = d; near_idx = k[3:0]; end
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx_r    <= DEFAULT_IDX;
            O_ae_val <= pre_val(DEFAULT_IDX);
            O_disp   <= pre_dsp(DEFAULT_IDX);
            O_req    <= 1'b0;
            wr_val   <= 16'hFFFF;             // force the first post-config write (lab1 pattern)
            acc_y <= 25'd0; fr_cnt <= 4'd0; dir_prev <= 2'b00; dir_run <= 3'd0;
            man_prev <= 4'd10;
        end else begin
            O_req <= 1'b0;
            if (I_load) begin
                idx_r <= (I_load_val[3:0] == 4'd0) ? 4'd1 : I_load_val[3:0];  // I_load_val is a RUNG
                                                          // (the linear search here cost
                                                          // 28 adders / -19.6ns timing);          // handover lands on a ladder rung
                acc_y  <= 25'd0; fr_cnt <= 4'd0; dir_prev <= 2'b00; dir_run <= 3'd0;
            end else if (!I_enable) begin
                // manual: nothing to do -- sel_idx above already follows the UI rung
            end else if (I_stats_valid && trust) begin
                acc_y <= sum_y;
                if (fr_cnt == ((1<<FRAME_SHIFT)-1)) begin
                    fr_cnt <= 4'd0;
                    acc_y  <= 25'd0;
                    if (dir_now == dir_prev) dir_run <= (dir_run == 3'd7) ? 3'd7 : (dir_run + 3'd1);
                    else                     dir_run <= 3'd1;
                    dir_prev <= dir_now;
                    if (persist_ok && !I_flicker) begin
                        if (!I_hold && too_dark   && (idx_r < COORD_EXP_MAX)) idx_r <= idx_r + 4'd1;  // brighter: longer (blur-capped)
                        else if (!I_hold && too_bright && (idx_r > 4'd1)) idx_r <= idx_r - 4'd1;
                    end
                end else begin
                    fr_cnt <= fr_cnt + 4'd1;
                end
            end
            man_prev <= man_idx;
            // publish the desired value every cycle (no register blocks the path)
            O_ae_val <= pub_val;
            O_disp   <= pub_disp;
            // Request a write ONCE CONFIG IS DONE and the desired value differs from what we
            // last asked the sensor to take.  This is the lab1-ae_set closed-loop pattern:
            //   - wr_val starts at 0xFFFF, so the FIRST post-config value is always written
            //     (the old bug: the only request was the reset pulse, which the wrapper drops
            //      because cfg_done is still 0 then, leaving the sensor stuck at the init table)
            //   - every later rung change re-triggers a single write, then stops
            if (I_go && (pub_val !== wr_val)) begin
                O_req  <= 1'b1;
                wr_val <= pub_val;
            end
        end
    end
endmodule
