`timescale 1ns / 1ps
// Monotone SC520CS gain map.
//
// Replaces the demo table (SC520GainTbl.v) whose coarse branches both waste the
// digital fine range and drop the 31.5x..32.0x segment.
//
// Datasheet V1.6, tables 2-4 / 2-5:
//   0x3e09 ANA GAIN       : 0x00=1x 0x08=2x 0x09=4x 0x0B=8x 0x0F=16x 0x1F=32x  (6.02 dB each)
//   0x3e06 DIG GAIN       : x2 integer stages (1x, 2x, 4x)
//   0x3e07 DIG FINE GAIN  : 1/32 resolution, 0x80=1.000, code/128 = gain
//
// Schedule: sweep the digital fine gain from 0x80 up to 0xFC inside one coarse
// stage, then step the coarse gain x2 and reset the fine gain. Total gain is
// therefore strictly increasing over the whole control range, with a residual
// ~0.13 dB dip at each stage boundary (fine max 1.969 vs coarse step 2.000).
//
// FINE_STEP selects the fine-gain granularity:
//   4 (default) = only the codes enumerated in datasheet table 2-5 (0x80..0xFC,
//                 32 entries per stage, 0.27 dB per press)
//   1           = every fine LSB (0x80..0xFF, 128 entries per stage, 0.068 dB),
//                 which follows the code/128 formula but uses codes above the
//                 highest one listed in the datasheet -- verify on hardware first
//
// I_ctrl counts fine entries: stage = I_ctrl/(128/FINE_STEP), fine index = remainder.
module sc520_gain_map #(
    parameter integer FINE_STEP      = 4,   // 4 = only the codes listed in table 2-5; 1 = every LSB
    parameter integer USE_FINE       = 1,   // 1 = interpolate with 0x3e07, 0 = pure analogue only
    parameter integer USE_DIG_COARSE = 0    // 1 = also allow the 0x3e06 x2/x4 stages (not used now)
)(
    input  wire [9:0]  I_ctrl,       // control value, saturating at the top
    output reg  [23:0] O,            // {0x3e09, 0x3e06, 0x3e07}
    output reg  [19:0] O_gain_q10,   // total gain x 1024 (exact, no rounding)
    output reg  [3:0]  O_stage,      // coarse stage 0..7, for telemetry
    output reg  [19:0] O_gain_x1000, // total gain x 1000 (truncated to 1/1000)
    output reg  [9:0]  O_ctrl_max    // highest usable I_ctrl value
);
    localparam integer PER_STAGE = (USE_FINE != 0) ? (128 / FINE_STEP) : 1;
    localparam integer N_STAGE   = (USE_DIG_COARSE != 0) ? 8 : 6;

    integer st, fi, code;
    reg [24:0] tmp_x1000;
    reg [7:0] ana, dig;

    always @(*) begin
        st = I_ctrl / PER_STAGE;
        fi = I_ctrl % PER_STAGE;
        if (st >= N_STAGE) begin          // saturate at the top
            st = N_STAGE - 1;
            fi = PER_STAGE - 1;
        end
        code = (USE_FINE != 0) ? (8'h80 + fi * FINE_STEP) : 8'h80;

        case (st)
            0: begin ana = 8'h00; dig = 8'h00; end
            1: begin ana = 8'h08; dig = 8'h00; end
            2: begin ana = 8'h09; dig = 8'h00; end
            3: begin ana = 8'h0B; dig = 8'h00; end
            4: begin ana = 8'h0F; dig = 8'h00; end
            5: begin ana = 8'h1F; dig = 8'h00; end
            6: begin ana = 8'h1F; dig = 8'h01; end
            default: begin ana = 8'h1F; dig = 8'h02; end   // only with USE_DIG_COARSE=1
        endcase

        O          = {ana, dig, code[7:0]};
        O_stage    = st[3:0];
        O_gain_q10 = ({8'd0, code[7:0]} << st) << 3;   // 2^st * code/128 * 1024
        // x1000 = x1024 * 1000/1024 = x1024 * 125/128, exact up to the final truncation
        tmp_x1000  = O_gain_q10 * 25'd125;
        O_gain_x1000 = tmp_x1000[24:7];
        O_ctrl_max   = (N_STAGE * PER_STAGE) - 1;
    end
endmodule
