// ============================================================================
// fps_meter -- on-screen frame-rate readout ("FPS xx.x", new line under CNT).
//
// Measures the DISPLAY frame rate.  I_frame_start must be the same once-per-frame
// marker that the top-right CNT counter counts (I_video_user in hdmi_mixer), so
// the two readouts can never disagree with each other.
//
// One decimal WITHOUT a divider.  A one-second window can only ever give an
// integer frame count, so ten one-second buckets are kept and summed.  That sum
// is the number of frames in the last ten seconds, i.e. fps*10 -- exactly the
// value the OSD prints once its decimal point is placed by position.  The readout
// therefore advances once per second in steps of 0.1 fps.
//
// Time base: I_tick_tog is a one-hertz TOGGLE generated in the 24 MHz domain
// (S_24m_clk = pll_clk_24m, 42.000 ns period = 23.809524 MHz, PLL-locked to the
// 50 MHz crystal, so the reading is an absolute measurement and does not depend
// on the pixel clock really being 75 MHz).  Only that single bit crosses into the
// pixel domain, and it is crossed with the same three-stage synchroniser + XOR
// edge detect that the AE/AG bridges in design_top_wrapper.v use.  The fps value
// itself is produced entirely inside the pixel domain, so this feature contains
// no multi-bit CDC anywhere.
//
// This module is deliberately single-clock so it can be verified in isolation.
// It adds no stage to the display pipeline: O_fps_bcd is a slow-moving, already
// stable input to the OSD, exactly like I_counter_bcd.
// ============================================================================
module fps_meter (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_frame_start,   // one pulse per display frame (I_video_user)
    input  wire        I_tick_tog,      // 1 Hz toggle from the 24 MHz domain
    output reg  [11:0] O_fps_bcd        // fps*10 as {hundreds, tens, units} BCD
);

    // ---- 1 Hz toggle -> one-cycle pulse in this domain ----------------------
    reg [2:0] tog_sync;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) tog_sync <= 3'd0;
        else         tog_sync <= {tog_sync[1:0], I_tick_tog};
    end
    wire tick_1s = tog_sync[2] ^ tog_sync[1];

    // ---- frames seen in the current second ---------------------------------
    // Rising-edge detect, so a marker that stays high for several clocks still
    // counts once.  Saturating, so a stuck marker cannot wrap the bucket.
    // On the tick the next second starts immediately and absorbs a frame pulse
    // that happens to land on the tick itself.
    reg fs_d;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) fs_d <= 1'b0;
        else         fs_d <= I_frame_start;
    end
    wire fs_pulse = I_frame_start & ~fs_d;

    reg [7:0] cur_cnt;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)                           cur_cnt <= 8'd0;
        else if(tick_1s)                       cur_cnt <= fs_pulse ? 8'd1 : 8'd0;
        else if(fs_pulse && cur_cnt != 8'hff)  cur_cnt <= cur_cnt + 8'd1;
    end

    // ---- ten one-second buckets; their sum is fps*10 ------------------------
    reg [7:0]  bucket [0:9];
    reg [3:0]  bp;
    reg [11:0] sum10;                       // <= 10*255 = 2550, fits in 12 bits
    // Total of the window AFTER this tick advances it.  It must be combinational:
    // loading the converter from the sum10 register instead would sample the value
    // from before the bucket update, i.e. convert the PREVIOUS window, which makes
    // the readout one second stale.  tb_fps_meter T1 caught exactly that
    // (10 frames/second displayed as 9.0 instead of 10.0).
    reg window_seeded;
    // Empty seconds before camera startup must not consume the warm start.
    // Once seeded, an idle/resume interval remains a true ten-second average.
    wire warm_start = !window_seeded && (cur_cnt != 8'd0);
    // Window state and BCD must consume the SAME next value, including seeding.
    wire [11:0] seeded_sum = ({4'd0,cur_cnt} << 3) + ({4'd0,cur_cnt} << 1);
    wire [11:0] sum10_next = warm_start ? seeded_sum :
        sum10 - {4'd0,bucket[bp]} + {4'd0,cur_cnt};
    integer i;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            bp    <= 4'd0;
            window_seeded <= 1'b0;
            sum10 <= 12'd0;
            for(i=0;i<10;i=i+1) bucket[i] <= 8'd0;
        end else if(tick_1s) begin
            // Read-before-write at the same address, so bucket[bp] here is the
            // oldest bucket and is exactly the one leaving the ten-second window.
            // WARM START (2026-10-16): on the very first tick the window is still empty, so the
            // readout used to crawl up for ~10 s.  Seed all ten buckets with the first measured
            // second; afterwards the normal rotating average takes over -> correct after ~1 s
            // while keeping the 10 s stability and the 0.1 resolution.
            if (warm_start) begin
                window_seeded <= 1'b1;
                for(i=0;i<10;i=i+1) bucket[i] <= cur_cnt;
                sum10 <= sum10_next;
            end else begin
                sum10      <= sum10_next;
                bucket[bp] <= cur_cnt;
            end
            bp         <= (bp==4'd9) ? 4'd0 : bp + 4'd1;
        end
    end

    // ---- 12-bit binary -> three BCD digits, double dabble over 12 clocks ----
    // Runs once a second, so a multi-cycle conversion costs nothing and keeps the
    // combinational depth tiny.  (ae_ctrl's near_idx() showed what a long serial
    // compare/subtract chain does to timing; this deliberately avoids one.)
    reg [11:0] dd_bin, dd_bcd;
    reg [3:0]  dd_cnt;
    reg        dd_run;
    wire [11:0] dd_adj = {
        (dd_bcd[11:8] >= 4'd5) ? (dd_bcd[11:8] + 4'd3) : dd_bcd[11:8],
        (dd_bcd[7:4]  >= 4'd5) ? (dd_bcd[7:4]  + 4'd3) : dd_bcd[7:4],
        (dd_bcd[3:0]  >= 4'd5) ? (dd_bcd[3:0]  + 4'd3) : dd_bcd[3:0] };
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            dd_bin <= 12'd0; dd_bcd <= 12'd0; dd_cnt <= 4'd0; dd_run <= 1'b0;
            O_fps_bcd <= 12'd0;
        end else if(tick_1s) begin
            // Clamp: only 0..999 is representable in three digits (99.9 fps).
            dd_bin <= (sum10_next > 12'd999) ? 12'd999 : sum10_next;
            dd_bcd <= 12'd0;
            dd_cnt <= 4'd12;
            dd_run <= 1'b1;
        end else if(dd_run) begin
            dd_bcd <= {dd_adj[10:0], dd_bin[11]};
            dd_bin <= {dd_bin[10:0], 1'b0};
            dd_cnt <= dd_cnt - 4'd1;
            if(dd_cnt == 4'd1) begin        // twelfth and last iteration
                dd_run    <= 1'b0;
                O_fps_bcd <= {dd_adj[10:0], dd_bin[11]};
            end
        end
    end

endmodule
