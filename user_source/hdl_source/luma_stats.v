`timescale 1ns / 1ps
// luma_stats -- per-frame brightness statistics for the automatic gain loop.
//
// Tapped inside hdmi_mixer BEFORE video_process, i.e. on the camera picture itself, so that
// invert / cartoon / emboss / edge / fill modes, saturation and brightness cannot bias the
// loop (a non-monotonic effect such as invert used to flip the sign of the loop entirely).
//
// The mean counts EVERY pixel of the centre window. Excluding saturated pixels was tried and
// had to be reverted: in a high-contrast scene (half black / half white) more pixels cross the
// clip threshold as the gain rises, so they left the sample and the measured mean FELL while
// the picture got brighter. Every rule in the loop assumes the mean rises with gain, so it
// pumped up and down -- that was the uneven-brightness flicker. Over-exposure is handled
// instead by the sat_count / SAT_LIMIT rule inside agc_ctrl, which has its own hysteresis.
//
//   Y          = (R + 2G + B) >> 2
//   O_y_mean   = sum(Y of all window pixels) / window pixels  (8-step shift-subtract divide)
//   O_y_filt   = first-order IIR of O_y_mean, seeded with the first frame (what the loop uses)
//   O_pix_count = measured count (controller trust); O_valid_count = unclipped count only.
//   A bad reader pixel in the window rejects the event and leaves the IIR untouched.
module luma_stats #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720,
    parameter integer WIN_X0     = 128,     // 128..1151 = 1024 pixels
    parameter integer WIN_X1     = 1151,
    parameter integer WIN_Y0     = 104,     // 104..615  = 512 lines
    parameter integer WIN_Y1     = 615,
    parameter [7:0]   SAT_LEVEL  = 8'd248,
    parameter [2:0]   FILT_SHIFT = 3'd2      // IIR smoothing strength: filt += dif >>> this (was 3'd1; raised to kill high-contrast mean jitter)
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_de,
    input  wire        I_pixel_good, // aligned reader quality; bad metering frames are rejected
    input  wire        I_vsync,
    input  wire [23:0] I_rgb,
    input  wire [11:0] I_x,
    input  wire [11:0] I_y,
    output reg  [7:0]  O_y_mean,
    output reg  [7:0]  O_y_filt,
    output reg  [23:0] O_sat_count,
    output reg  [23:0] O_pix_count,
    output reg  [23:0] O_valid_count,
    output reg         O_stats_valid
);
    wire in_window = I_de && (I_x >= WIN_X0) && (I_x <= WIN_X1) &&
                            (I_y >= WIN_Y0) && (I_y <= WIN_Y1);
    wire [9:0] y_luma = (I_rgb[23:16] + (I_rgb[15:8] << 1) + I_rgb[7:0]) >> 2;
    wire clipped = (I_rgb[23:16] >= SAT_LEVEL) || (I_rgb[15:8] >= SAT_LEVEL) ||
                   (I_rgb[7:0]  >= SAT_LEVEL);

    reg [31:0] acc_y;        // sum of Y over ALL window pixels (monotone in gain)
    reg [23:0] acc_n, acc_valid, acc_sat;
    reg        acc_bad;
    reg        vsync_d;

    reg        dv_run;
    reg [3:0]  dv_i;
    reg [31:0] dv_sum, dv_den;
    reg [7:0]  dv_mean;
    reg        filt_seeded;
    wire [31:0] dv_test    = dv_den << dv_i;
    wire [7:0]  mean_final = (dv_sum >= dv_test) ? (dv_mean | 8'd1) : dv_mean;
    wire signed [9:0] filt_dif = $signed({2'b0, mean_final}) - $signed({2'b0, O_y_filt});

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            acc_y <= 32'd0; acc_n <= 24'd0; acc_valid <= 24'd0; acc_sat <= 24'd0;
            acc_bad <= 1'b0;
            vsync_d <= 1'b0; dv_run <= 1'b0; dv_i <= 4'd0; dv_sum <= 32'd0;
            dv_den <= 32'd0; dv_mean <= 8'd0; filt_seeded <= 1'b0;
            O_y_mean <= 8'd0; O_y_filt <= 8'd0; O_sat_count <= 24'd0;
            O_pix_count <= 24'd0; O_valid_count <= 24'd0; O_stats_valid <= 1'b0;
        end else begin
            vsync_d <= I_vsync;
            if (in_window) begin
                if (!I_pixel_good) acc_bad <= 1'b1;
                acc_n <= acc_n + 24'd1;
                acc_y <= acc_y + {22'd0, y_luma};          // every pixel counts
                if (clipped) acc_sat <= acc_sat + 24'd1;
                else         acc_valid <= acc_valid + 24'd1;
            end
            O_stats_valid <= 1'b0;
            if (!dv_run) begin
                if (I_vsync && !vsync_d) begin
                    O_pix_count   <= acc_n;
                    O_sat_count   <= acc_sat;
                    O_valid_count <= acc_valid;
                    // Saturation is valid scene information, not a transport error.
                    // A transport-bad window must update neither the IIR nor controllers.
                    if (acc_bad || (in_window && !I_pixel_good)) begin end
                    else if (acc_n == 24'd0) O_y_mean <= 8'd0;
                    else begin
                        dv_run  <= 1'b1;
                        dv_i    <= 4'd7;
                        dv_sum  <= acc_y;
                        dv_den  <= {8'd0, acc_n};
                        dv_mean <= 8'd0;
                    end
                    acc_y <= 32'd0; acc_n <= 24'd0; acc_valid <= 24'd0; acc_sat <= 24'd0;
                    acc_bad <= 1'b0;
                end
            end else begin
                if (dv_sum >= dv_test) dv_sum <= dv_sum - dv_test;
                dv_mean <= (dv_sum >= dv_test) ? (dv_mean | (8'd1 << dv_i)) : dv_mean;
                if (dv_i == 4'd0) begin
                    dv_run        <= 1'b0;
                    O_y_mean      <= mean_final;
                    if (!filt_seeded) begin                  // seed the filter with frame 1
                        O_y_filt     <= mean_final;
                        filt_seeded  <= 1'b1;
                    end else begin
                        O_y_filt     <= O_y_filt + (filt_dif >>> FILT_SHIFT);
                    end
                    O_stats_valid <= 1'b1;
                end else dv_i <= dv_i - 4'd1;
            end
        end
    end
endmodule
