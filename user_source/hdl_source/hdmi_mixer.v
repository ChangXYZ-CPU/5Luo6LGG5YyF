module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0,
    parameter integer DISPLAY_ENHANCE = 1, // picture branch only, after all analysis inputs
    parameter integer SHAPE_BUILD = 1, // 0: omit shape engine; restore at top after revalidation.
    parameter integer LSC_GRID_ENABLE=1,GREEN_WEAK_ENABLE=1,SHARE_GREEN_FIELD=0,
    parameter GREEN_GAIN_FILE="gain_rg_1280_720.mem",
    parameter LSC_GRID_FILE="field_1280_720.mem",
    parameter integer GUARD_CLK_HZ=75000000,GUARD_TIMEOUT_MS=500,GUARD_ALARM_MS=2000,
    parameter integer GUARD_X0=400,GUARD_X1=880,GUARD_Y0=220,GUARD_Y1=500,
    parameter integer GUARD_BUZZER_ON_MS=200,GUARD_BUZZER_OFF_MS=800,
    parameter integer GUARD_BUZZER_TONE_HZ=2000,GUARD_BUZZER_ACTIVE=0,
    parameter integer COLOUR_DIAG_ENABLE=1,COLOUR_DIAG_PERIOD=16
)(
    input wire        I_clk,
    input wire        I_rst_n,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,
    input wire        I_tick_1hz_tog,   // 1 Hz toggle from the 24 MHz domain (FPS readout)

    input wire[3:0]   I_debug_status,
    input wire        I_osd_en,
    input wire        I_invert_en,
    input wire        I_emboss_en,
    input wire[4:0]   I_sat_level,
    input wire[8:0]   I_bright_level,
    input wire[3:0]   I_mode,
    input wire[3:0]    I_adj_idx,
    input wire         I_show_temp,
    input wire[12:0]   I_wb_temp,
    input wire[7:0]    I_gain_level,
    input wire signed [6:0] I_wb_trim,
    input wire         I_wb_is_trim,
    input wire         I_wb_trim_neg,
    input wire         I_wb_trim_en,
    output wire[7:0]   O_y_mean,
    output wire[7:0]   O_y_filt,
    output wire[23:0]  O_sat_count,
    output wire[23:0]  O_valid_count,
    output wire        O_stats_valid,
    output wire        O_detect_now,     // ��ǰ�Ƿ��ڼ�⣨�� MCU �� STATUS��
    output wire        O_target_now,     // ��ǰ֡�Ƿ��ҵ�Ŀ��
    output wire [19:0] O_target_area,    // largest colour component area (0x2C)
    output wire [1:0]  O_shape_class,    // largest CONFIRMED shape class (0x34[1:0])
    output wire        O_shape_valid,    // that class is temporally confirmed (0x34[2])
    output wire [1:0]  O_shape_count,    // targets in the published list, 0..3 (0x34[4:3])
    input wire[7:0]   I_r_min, I_r_dom, I_g_min, I_g_dom, I_b_min, I_b_dom,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,
    input wire [7:0]  I_lsc_edge,      // 2026-10-18
    input wire [7:0]  I_lsc_centre,
    input wire [7:0]  I_lsc_lift,        // K (common brightness lift), item 15
    input wire        I_video_pixel_good,
    input wire [31:0] I_source_frame_id,
    input wire        I_source_valid,I_source_rst_n,

    // L03: 公共时基（本域已同步好的值）与测量描述符（与 I_video_de 同帧）
    input wire [31:0] I_tick_now,
    input wire [15:0] I_session_now,
    input wire        I_time_ready,
    input wire [31:0] I_lat_start_tick,
    input wire [15:0] I_lat_session,
    input wire        I_lat_valid,

    output wire        O_hdmi_vsync,
    output wire        O_hdmi_hsync,
    output wire [23:0] O_probe_rgb,      // LSC-corrected pixel for the colour probe
    output wire        O_probe_de,
    output wire        O_probe_vsync,
    output wire        O_hdmi_de,
    output wire[23:0]  O_hdmi_data,

    // L03: 测量结果（L04 接 OSD；当前引出供仿真/调试）
    output wire [2:0]  O_lat_state,
    output wire [15:0] O_lat_bcd,
    output wire [31:0] O_lat_delta,
    output wire        O_lat_snap_tog,

    input wire [7:0] I_shape_threshold,I_paper_threshold,
    input wire [1:0] I_shape_profile,I_shape_view,
    input wire [4:0] I_area_min,I_area_max,
    output wire O_alarm_led,
    output wire O_alarm_buzzer_n,
    output wire O_colour_diag_valid,
    output wire [7:0] O_colour_diag_data,
    input wire I_colour_diag_ready
);

    reg S_video_vsync_1d, S_video_vsync_2d;
    reg S_video_hsync_1d, S_video_hsync_2d, S_video_de_1d, S_video_de_2d;
    reg [1:0] S_user_delay, S_last_delay;
    reg [11:0] S_x, S_y, S_x_1d, S_x_2d, S_y_1d, S_y_2d;

    // ---- LSC: affine radial colour-shading correction (2026-10-18) ----------
    // LSC_EN = 0 -> pixel path identical to before (A01 bypass build)
    // LSC_EN = 1 -> corrected base RGB feeds display; colour masks retain PRE.
    // Shape/motion/statistics retain their complete pre-LSC reader packet.
    localparam       LSC_EN     = 1'b1;   // A02: enabled (affine model)
    localparam       LSC_ANCHOR = 1'b0;   // A: affine (B chain abandoned)
    wire [23:0] S_lsc_rgb;
    // mode 1111 (4'b1111 / 15) = LSC bypass for A/B comparison
    wire S_lsc_on = LSC_EN && (I_mode != 4'd15);
    // Bypass still traverses all five registers inside lsc_gain. Never select
    // an undelayed pixel while its DE/coordinates are five clocks later.
    wire [23:0] S_video_use = S_lsc_rgb;


    // Opt-in only: host must prove full RGB table projects to GREEN_GAIN_FILE.
    wire[17:0] S_raw_field;
    wire S_lsc_de,S_lsc_user,S_lsc_last,S_lsc_good;
    wire [11:0] S_lsc_x,S_lsc_y;
    wire [7:0] S_lsc_frame_edge,S_lsc_frame_centre,S_lsc_frame_lift;
    generate if(LSC_GRID_ENABLE)begin: calibrated_display
      lsc_calibrated_rom_r14 #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT),.DEFER_CONTROLS(1),.MEM_FILE(LSC_GRID_FILE)) u_lsc_gain(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(S_video_de_2d),.I_user(S_user_delay[1]),.I_last(S_last_delay[1]),
        .I_good(I_video_pixel_good&&I_source_valid&&I_source_rst_n),.I_enable(S_lsc_on),
        .I_x(S_x_2d),.I_y(S_y_2d),.I_rgb(I_video_rd_data),.I_source_id(I_source_frame_id),
        .I_edge(I_lsc_edge),.I_centre(I_lsc_centre),.I_lift(I_lsc_lift),.I_sat(5'd16),
        .O_de(S_lsc_de),.O_user(S_lsc_user),.O_last(S_lsc_last),.O_good(S_lsc_good),.O_config_bad(),
        .O_x(S_lsc_x),.O_y(S_lsc_y),.O_rgb(S_lsc_rgb),.O_source_id(),
        .O_raw_gain_rg(S_raw_field),.O_frame_edge(S_lsc_frame_edge),.O_frame_centre(S_lsc_frame_centre),.O_frame_lift(S_lsc_frame_lift));
    end else begin: original_display
    lsc_gain #(.CX(640), .CY(360), .ANCHOR(LSC_ANCHOR)) u_lsc_gain (
        .I_clk    ( I_clk        ),
        .I_rst_n  ( I_rst_n      ),
        .I_de     ( S_video_de_2d ),
        .I_x      ( S_x_2d[10:0] ),
        .I_y      ( S_y_2d[9:0]  ),
        .I_rgb    ( I_video_rd_data ),
        .I_en     ( S_lsc_on     ),
        .I_edge   ( I_lsc_edge   ),
        .I_centre ( I_lsc_centre ),
        .I_lift   ( I_lsc_lift   ),
        .I_sat    ( 5'd16        ), // style saturation belongs to display only
        .O_rgb    ( S_lsc_rgb    )
    );


      assign {S_lsc_de,S_lsc_user,S_lsc_last,S_lsc_good,S_lsc_x,S_lsc_y}=28'd0;
      assign S_raw_field=18'd0;
      assign S_lsc_frame_edge=I_lsc_edge;assign S_lsc_frame_centre=I_lsc_centre;assign S_lsc_frame_lift=I_lsc_lift;
    end endgenerate

    // Colour recognition uses the measured PRE-reader samples. Radial LSC
    // remains on the picture path and cannot create a foreground colour ring.
    // Five clocks to chroma, then two to denoise: shared mask is LSC+2.
    wire S_colour_de,S_colour_good,S_colour_mask;
    wire [11:0] S_colour_x,S_colour_y;
    wire [3:0] S_colour_mode;
    wire [7:0] S_colour_min,S_colour_sat;
    rgb_colour_frontend_r14 #(.USE_EXTERNAL_FIELD(SHARE_GREEN_FIELD&&LSC_GRID_ENABLE),.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT),.GREEN_WEAK_ENABLE(GREEN_WEAK_ENABLE),.GAIN_FILE(GREEN_GAIN_FILE)) u_colour_frontend(
        .I_field_word(S_raw_field),.I_clk(I_clk),.I_rst_n(I_rst_n&&I_source_rst_n),
        .I_de(S_video_de_2d),.I_user(S_user_delay[1]),.I_last(S_last_delay[1]),
        .I_good(I_video_pixel_good&&I_source_valid),.I_rgb(I_video_rd_data),.I_source_id(I_source_frame_id),.I_x(S_x_2d),.I_y(S_y_2d),
        .I_mode(I_mode),.I_r_min(I_r_min),.I_r_sat(I_r_dom),.I_g_min(I_g_min),.I_g_sat(I_g_dom),.I_b_min(I_b_min),.I_b_sat(I_b_dom),
        .O_de(S_colour_de),.O_good(S_colour_good),.O_mask(S_colour_mask),.O_x(S_colour_x),.O_y(S_colour_y),
        .O_frame_mode(S_colour_mode),.O_frame_min(S_colour_min),.O_frame_sat(S_colour_sat));

    // ---- align the video timing with the LSC pipeline latency (2026-10-23) -----------------
    localparam integer LSC_LAT = 5;   // rgb_d1/d2/d3 + saturation + output register
    reg [LSC_LAT-1:0] S_hs_lat,S_vs_lat;
    integer li;
    always @(posedge I_clk or negedge I_rst_n)begin
      if(!I_rst_n)begin S_hs_lat<=0;S_vs_lat<=0;end
      else begin S_hs_lat<={S_hs_lat[LSC_LAT-2:0],S_video_hsync_2d};S_vs_lat<={S_vs_lat[LSC_LAT-2:0],S_video_vsync_2d};end
    end
    wire S_de_al,S_user_al,S_last_al,S_good_al;
    wire [11:0] S_x_al,S_y_al;
    wire S_hs_al=S_hs_lat[LSC_LAT-1],S_vs_al=S_vs_lat[LSC_LAT-1];
    generate if(LSC_GRID_ENABLE)begin: reuse_calibration_tags
      assign S_de_al=S_lsc_de;assign S_user_al=S_lsc_user;assign S_last_al=S_lsc_last;assign S_good_al=S_lsc_good;
      assign S_x_al=S_lsc_x;assign S_y_al=S_lsc_y;
    end else begin: legacy_tags
      reg [11:0] xlat[0:LSC_LAT-1],ylat[0:LSC_LAT-1];
      reg [LSC_LAT-1:0] de_lat,us_lat,ls_lat,good_lat;integer j;
      always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
          de_lat<=0;us_lat<=0;ls_lat<=0;good_lat<=0;
          for(j=0;j<LSC_LAT;j=j+1)begin xlat[j]<=0;ylat[j]<=0;end
        end else begin
          de_lat<={de_lat[LSC_LAT-2:0],S_video_de_2d};us_lat<={us_lat[LSC_LAT-2:0],S_user_delay[1]};
          ls_lat<={ls_lat[LSC_LAT-2:0],S_last_delay[1]};good_lat<={good_lat[LSC_LAT-2:0],I_video_pixel_good&&I_source_valid};
          xlat[0]<=S_x_2d;ylat[0]<=S_y_2d;
          for(j=1;j<LSC_LAT;j=j+1)begin xlat[j]<=xlat[j-1];ylat[j]<=ylat[j-1];end
        end
      end
      assign S_de_al=de_lat[LSC_LAT-1];assign S_user_al=us_lat[LSC_LAT-1];assign S_last_al=ls_lat[LSC_LAT-1];assign S_good_al=good_lat[LSC_LAT-1];
      assign S_x_al=xlat[LSC_LAT-1];assign S_y_al=ylat[LSC_LAT-1];
    end endgenerate
    // probe taps the LSC output (same data the detectors see), not the final HDMI picture
    assign O_probe_rgb   = S_video_use;
    assign O_probe_de    = S_de_al;
    assign O_probe_vsync = S_vs_al;
    reg [31:0] S_frame_cnt;
    reg [3:0] S_cnt_d0, S_cnt_d1, S_cnt_d2, S_cnt_d3, S_cnt_d4, S_cnt_d5;
    wire S_vp_de, S_vp_vsync, S_vp_hsync;
    wire [23:0] S_vp_data;
    wire [11:0] S_vp_x, S_vp_y;
    wire S_vp_user,S_vp_last;
    wire S_detect,S_target;
    assign O_detect_now = S_detect;
    assign O_target_now = S_target;
    wire [2:0] S_det_status;
    wire [11:0] S_det_xmin,S_det_xmax,S_det_ymin,S_det_ymax;
    wire [19:0] S_det_area;
    assign O_target_area = S_det_area;   // same SOF-latched value shown in the OSD
    wire [31:0] S_det_source_frame_id,S_det_mode_epoch;
    wire [31:0] S_det_capture_seq;
    wire S_det_frame_fault;
    wire S_coords_valid;
    wire [11:0] S_center_x,S_center_y;
    wire [15:0] S_center_x_bcd,S_center_y_bcd;
    wire [3:0] S_result_mode;
    // Parallel metadata only: never add another delay to reader RGB.
    // Queue each complete snapshot on the SAME edge as the nine-clock ISP.
    // Keep legacy [52:0] intact; prefix one atomic coordinate/provenance packet.
    // P04 will consume these fields in the OSD; never retime display RGB here.
    reg [177:0] S_det_delay[0:8];
    integer det_stage;

    assign O_video_rd_en = I_video_de;

    // Reader RGB is already registered twice. Retain the proven reader/ISP
    // interface exactly; only the downstream OSD pipeline is retimed.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d<=0; S_video_vsync_2d<=0;
            S_video_hsync_1d<=0; S_video_hsync_2d<=0;
            S_video_de_1d<=0; S_video_de_2d<=0;
        end else begin
            S_video_vsync_1d<=I_video_vsync; S_video_vsync_2d<=S_video_vsync_1d;
            S_video_hsync_1d<=I_video_hsync; S_video_hsync_2d<=S_video_hsync_1d;
            S_video_de_1d<=I_video_de; S_video_de_2d<=S_video_de_1d;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= I_video_de ? 12'd1 : 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_frame_cnt <= 32'd0;
            S_cnt_d0 <= 4'd0;
            S_cnt_d1 <= 4'd0;
            S_cnt_d2 <= 4'd0;
            S_cnt_d3 <= 4'd0;
            S_cnt_d4 <= 4'd0;
            S_cnt_d5 <= 4'd0;
        end
        else if(I_video_user) begin
            S_frame_cnt <= S_frame_cnt + 32'd1;
            if(S_cnt_d0 == 4'd9) begin
                S_cnt_d0 <= 4'd0;
                if(S_cnt_d1 == 4'd9) begin
                    S_cnt_d1 <= 4'd0;
                    if(S_cnt_d2 == 4'd9) begin
                        S_cnt_d2 <= 4'd0;
                        if(S_cnt_d3 == 4'd9) begin
                            S_cnt_d3 <= 4'd0;
                            if(S_cnt_d4 == 4'd9) begin
                                S_cnt_d4 <= 4'd0;
                                if(S_cnt_d5 == 4'd9)
                                    S_cnt_d5 <= 4'd0;
                                else
                                    S_cnt_d5 <= S_cnt_d5 + 4'd1;
                            end
                            else
                                S_cnt_d4 <= S_cnt_d4 + 4'd1;
                        end
                        else
                            S_cnt_d3 <= S_cnt_d3 + 4'd1;
                    end
                    else
                        S_cnt_d2 <= S_cnt_d2 + 4'd1;
                end
                else
                    S_cnt_d1 <= S_cnt_d1 + 4'd1;
            end
            else
                S_cnt_d0 <= S_cnt_d0 + 4'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end
        else begin
            S_x_1d <= I_video_user ? 12'd0 : S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= I_video_user ? 12'd0 : S_y;
            S_y_2d <= S_y_1d;
        end
    end

    // DDR reader already registers RGB twice. Delay controls only, then
    // sample RGB directly; delaying it again shifts the displayed columns.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin S_user_delay<=0; S_last_delay<=0; end
        else begin
            S_user_delay<={S_user_delay[0],I_video_user};
            S_last_delay<={S_last_delay[0],I_video_last};
        end
    end
    // Detect original reader RGB before brightness/saturation/fill. Modes 6/8/10
    // share one CCA; selected colour uses the same MIN/DOM as its fill mode.
    // B07 publishes a complete previous-frame result; SOF bypass is sampled
    // into stage 0 here so the first processed pixel cannot see an old result.
    wire [7:0] S_det_min=(I_mode==4'd8)?I_b_min:((I_mode==4'd10)?I_g_min:I_r_min);
    // M03 provisional frame difference; M04 independent motion region/OSD.
    reg S_motion_vs_d;
    always @(posedge I_clk or negedge I_rst_n)
        if(!I_rst_n) S_motion_vs_d<=0; else S_motion_vs_d<=S_video_vsync_2d;
    wire S_motion_valid,S_motion_first,S_motion_final,S_motion_compare,S_motion_mask,S_motion_done;
    wire S_motion_abort,S_motion_reference_clear,S_motion_reference_valid,S_motion_busy;
    wire [7:0] S_motion_gray,S_motion_reference_gray,S_motion_diff,S_motion_x;
    wire [6:0] S_motion_y;
    wire [31:0] S_motion_source_id,S_motion_reference_id,S_motion_config_epoch;
    wire S_motion_enable=I_mode==4'd11 || I_mode==4'd13;
    motion_diff_stream #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT)) u_motion_stream(
        .I_clk(I_clk),.I_rst_n(I_rst_n && I_source_rst_n),.I_enable(S_motion_enable),
        .I_boundary(S_motion_vs_d && !S_video_vsync_2d),
        .I_de(S_video_de_2d),.I_user(S_user_delay[1]),.I_last(S_last_delay[1]),
        .I_pixel_good(I_video_pixel_good),.I_source_valid(I_source_valid),
        .I_source_id(I_source_frame_id),.I_config({28'd0,I_mode}),
        .I_rgb(I_video_rd_data),.I_x(S_x_2d),.I_y(S_y_2d),
        .O_valid(S_motion_valid),.O_first(S_motion_first),.O_final(S_motion_final),
        .O_compare_valid(S_motion_compare),.O_mask(S_motion_mask),.O_done(S_motion_done),
        .O_abort(S_motion_abort),.O_reference_clear(S_motion_reference_clear),
        .O_reference_valid(S_motion_reference_valid),.O_busy(S_motion_busy),
        .O_gray(S_motion_gray),.O_reference_gray(S_motion_reference_gray),.O_diff(S_motion_diff),
        .O_x(S_motion_x),.O_y(S_motion_y),.O_source_id(S_motion_source_id),
        .O_reference_id(S_motion_reference_id),.O_epoch(S_motion_config_epoch)
    );
    wire [177:0] S_motion_snapshot;
    wire S_motion_pending,S_motion_region_fault;
    wire [2:0] S_motion_region_error;
    motion_regions #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT)) u_motion_regions(
        .I_clk(I_clk),.I_rst_n(I_rst_n && I_source_rst_n),.I_enable(S_motion_enable),
        .I_raw_sof(S_video_de_2d && S_user_delay[1]),.I_reference_clear(S_motion_reference_clear),
        .I_valid(S_motion_valid),.I_first(S_motion_first),.I_final(S_motion_final),
        .I_compare_valid(S_motion_compare),.I_mask(S_motion_mask),.I_done(S_motion_done),
        .I_x(S_motion_x),.I_y(S_motion_y),.I_source_id(S_motion_source_id),
        .I_reference_id(S_motion_reference_id),.I_epoch(S_motion_config_epoch),
        .O_snapshot(S_motion_snapshot),.O_reference_id(),.O_pending_ready(S_motion_pending),
        .O_candidate(),.O_candidate_area(),
        .O_busy(),.O_fault(S_motion_region_fault),.O_error(S_motion_region_error)
    );
    // 2026-11-02: 屏上框稳定器（motion_display_stabilizer）按用户要求移除，
    // 只为省资源（Slice 96.15% 时它占 643 LUT / 718 slice）。
    // 代价：模式 11 的框回到"原始帧结果直接上屏"，会重新出现抖动/碎框。
    // 模块源码仍在 motion_regions.v 里，需要恢复时把实例接回去即可。
    wire [177:0] S_motion_overlay_snapshot=S_motion_snapshot;
    wire [7:0] S_det_dom=(I_mode==4'd8)?I_b_dom:((I_mode==4'd10)?I_g_dom:I_r_dom);
    wire [31:0] S_diag_capture_epoch,S_diag_capture_seq,S_diag_epoch;
    wire S_diag_event;
    wire [3:0] S_diag_mode;
    wire [2:0] S_diag_status,S_diag_error;
    wire [7:0] S_diag_runs;
    wire [9:0] S_diag_labels;
    wire [15:0] S_diag_row,S_diag_fold,S_diag_tail;
    red_cca_stream #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT),.AREA_CONTROL(1),.EXTERNAL_MASK(1)) u_red_detector(
        .I_area_min(I_area_min),
        .I_th_min(S_colour_min),.I_th_dom(S_colour_sat),
        .I_mask_de(S_colour_de),.I_mask_good(S_colour_good),.I_mask(S_colour_mask),.I_mask_x(S_colour_x),.I_mask_y(S_colour_y),
        .I_clk(I_clk),.I_rst_n(I_rst_n && I_source_rst_n),.I_enable(1'b1),
        .I_de(S_de_al),.I_user(S_user_al),.I_last(S_last_al),
        .I_pixel_good(S_good_al),.I_rgb(S_video_use),
        .I_x(S_x_al),.I_y(S_y_al),.I_mode(S_colour_mode),
        .O_detect(S_detect),.O_target_valid(S_target),.O_status(S_det_status),
        .O_xmin(S_det_xmin),.O_xmax(S_det_xmax),.O_ymin(S_det_ymin),.O_ymax(S_det_ymax),
        .O_area(S_det_area),.O_source_frame_id(S_det_source_frame_id),.O_mode_epoch(S_det_mode_epoch),
        .O_tail_cycles(),.O_fold_cycles(),.O_max_row_cycles(),.O_pending_ready(),
        .O_coords_valid(S_coords_valid),.O_center_x(S_center_x),.O_center_y(S_center_y),
        .O_center_x_bcd(S_center_x_bcd),.O_center_y_bcd(S_center_y_bcd),.O_result_mode(S_result_mode),
        .O_capture_frame_seq(S_det_capture_seq),.O_frame_fault(S_det_frame_fault),
        .O_capture_epoch(S_diag_capture_epoch),.O_diag_valid(S_diag_event),
        .O_diag_capture_seq(S_diag_capture_seq),.O_diag_epoch(S_diag_epoch),.O_diag_mode(S_diag_mode),
        .O_diag_status(S_diag_status),.O_diag_error(S_diag_error),.O_diag_max_runs(S_diag_runs),.O_diag_labels(S_diag_labels),
        .O_diag_max_row_cycles(S_diag_row),.O_diag_fold_cycles(S_diag_fold),.O_diag_tail_cycles(S_diag_tail)
    );
    generate if(COLOUR_DIAG_ENABLE&&IMG_WIDTH>=12&&IMG_HEIGHT>=12)begin: colour_diagnostics
        wire [7:0] diag_min=(I_mode==7||I_mode==8)?I_b_min:(I_mode==9||I_mode==10)?I_g_min:I_r_min;
        wire [7:0] diag_dom=(I_mode==7||I_mode==8)?I_b_dom:(I_mode==9||I_mode==10)?I_g_dom:I_r_dom;
        // Context schema 2: PRE 3x3/shared mask; DOM is relative chroma Q0.8.
        wire [63:0] diag_context=LSC_GRID_ENABLE?{(GREEN_WEAK_ENABLE?6'd6:6'd5),I_area_min,I_sat_level,8'd1,S_lsc_frame_edge,S_lsc_frame_centre,S_lsc_frame_lift,diag_min,diag_dom}:{(GREEN_WEAK_ENABLE?6'd6:6'd5),I_area_min,I_sat_level,8'd0,I_lsc_edge,I_lsc_centre,I_lsc_lift,diag_min,diag_dom};
        colour_diag_link #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT),.PRE_DELAY(LSC_LAT),.PERIOD_FRAMES(COLOUR_DIAG_PERIOD)) link(
            .I_clk(I_clk),.I_rst_n(I_rst_n&&I_source_rst_n),
            .I_reader_rgb(I_video_rd_data),.I_reader_source_id(I_source_frame_id),.I_reader_source_valid(I_source_valid),
            .I_post_rgb(S_video_use),.I_post_de(S_de_al),.I_post_user(S_user_al),.I_post_last(S_last_al),.I_post_good(S_good_al),
            .I_post_x(S_x_al),.I_post_y(S_y_al),.I_mode(I_mode),.I_capture_seq(S_det_capture_seq),.I_capture_epoch(S_diag_capture_epoch),.I_context(diag_context),
            .I_diag_valid(S_diag_event),.I_diag_capture_seq(S_diag_capture_seq),.I_diag_epoch(S_diag_epoch),.I_diag_mode(S_diag_mode),
            .I_diag_status(S_diag_status),.I_diag_error(S_diag_error),.I_diag_max_runs(S_diag_runs),.I_diag_labels(S_diag_labels),
            .I_diag_row_cycles(S_diag_row),.I_diag_fold_cycles(S_diag_fold),.I_diag_tail_cycles(S_diag_tail),
            .O_valid(O_colour_diag_valid),.O_data(O_colour_diag_data),.I_ready(I_colour_diag_ready));
    end else begin: no_colour_diagnostics
        assign O_colour_diag_valid=1'b0;
        assign O_colour_diag_data=8'd0;
    end endgenerate
    // Mode 13 shares the proven mode-11 motion engine. Mode 6 is once again
    // plain red detection: no guard state, rectangle, LED or buzzer overlay.
    wire S_guard_event,S_guard_bad;
    wire S_guard_alarm,S_guard_fault,S_guard_center_valid;
    wire [2:0] S_guard_state;
    wire [3:0] S_guard_reason;
    wire [11:0] S_guard_center_x,S_guard_center_y;
    motion_guard_link #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT)) u_guard_link(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_mode==4'd13),
        .I_source_rst_n(I_source_rst_n),.I_de(S_video_de_2d),.I_sof(S_user_delay[1]),
        .I_pixel_good(I_video_pixel_good),.I_source_valid(I_source_valid),
        .I_x(S_x_2d),.I_y(S_y_2d),.I_stream_abort(S_motion_abort),
        .I_reference_clear(S_motion_reference_clear),.I_pending(S_motion_pending),
        .I_region_fault(S_motion_region_fault),.I_region_error(S_motion_region_error),
        .I_snapshot(S_motion_snapshot),.I_control_fault(S_guard_fault),
        .O_result_event(S_guard_event),.O_fault_event(S_guard_bad),.O_reason(S_guard_reason));
    motion_guard_oneshot #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT),
        .X0(GUARD_X0),.X1(GUARD_X1),.Y0(GUARD_Y0),.Y1(GUARD_Y1),
        .CLK_HZ(GUARD_CLK_HZ),.TIMEOUT_MS(GUARD_TIMEOUT_MS),.ALARM_MS(GUARD_ALARM_MS)) u_guard_control(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_mode==4'd13),
        .I_reconfigure(1'b0),.I_source_reset(!I_source_rst_n),
        .I_fault_event(S_guard_bad),.I_result_event(S_guard_event),
        .I_target_valid(S_motion_snapshot[51]),.I_coords_valid(S_motion_snapshot[177]),
        .I_source_id(S_motion_snapshot[116:85]),
        .I_center_x(S_motion_snapshot[176:165]),.I_center_y(S_motion_snapshot[164:153]),
        .O_alarm(S_guard_alarm),.O_fault(S_guard_fault),
        .O_state(S_guard_state),.O_accept(),.O_trigger(),.O_locked(),.O_center_valid(S_guard_center_valid),
        .O_center_x(S_guard_center_x),.O_center_y(S_guard_center_y));
    assign O_alarm_led=I_rst_n && I_mode==4'd13 && S_guard_alarm;
    motion_guard_tone #(.CLK_HZ(GUARD_CLK_HZ),.TONE_HZ(GUARD_BUZZER_TONE_HZ),
        .ACTIVE_BUZZER(GUARD_BUZZER_ACTIVE)) u_guard_buzzer(
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_mode==4'd13),
        .I_alarm(S_guard_alarm),.O_buzzer_n(O_alarm_buzzer_n));
    wire [19:0] S_shape_diag;
    wire [177:0] S_shape_packet;
    wire[248:0] S_multi_packets;
    wire [1:0] S_shape_class;
    // Per-target packet is 83 bits: {valid,class,xmin,xmax,ymin,ymax,X_BCD,Y_BCD}.
    // Count what the operator actually sees as boxes, not just classified ones.
    wire [1:0] S_shape_count={1'b0,S_multi_packets[82]}+{1'b0,S_multi_packets[165]}+
                             {1'b0,S_multi_packets[248]};
    // Shape capture is fixed 720p. Small-raster legacy test configurations do
    // not instantiate a mismatched observer, and keep every old mode unchanged.
    // LITE_MODE (2026-10-18): build-time switch for quick debug builds.
    //   1 = full design (shape classification built)
    //   0 = omit shape_720p; no_shape ties annotation outputs to zero.
    generate if(SHAPE_BUILD && IMG_WIDTH==1280 && IMG_HEIGHT==720)begin: shape_720p
        reg active;
        reg [31:0] source;
        wire sof=S_video_de_2d && S_user_delay[1];
        wire final_pixel=S_video_de_2d && S_last_delay[1] && S_y_2d==719;
        wire source_bad=S_video_de_2d && (!I_source_valid ||
            (active && !sof && I_source_frame_id!=source));
        always @(posedge I_clk or negedge I_rst_n)begin
            if(!I_rst_n)begin active<=0;source<=0;end
            else if(!I_source_rst_n || I_mode!=12 || source_bad)active<=0;
            else if(sof)begin active<=!final_pixel;source<=I_source_frame_id;end
            else if(final_pixel)active<=0;
        end
        wire target,fault;
        wire diag_pvalid,diag_kvalid;
        wire [33:0] diag_pbox,diag_kbox;
        wire [31:0] diag_source,diag_epoch;
        wire diag_page=I_shape_view==1||I_shape_view==2;
        wire diag_target=I_shape_view==2?diag_pvalid:diag_kvalid;
        wire [33:0] diag_box=I_shape_view==2?diag_pbox:diag_kbox;
        wire [11:0] diag_l=I_shape_view==2?{diag_box[33:25],3'b000}:{1'b0,diag_box[33:25],2'b00};
        wire [11:0] diag_r=I_shape_view==2?{diag_box[24:16],3'b111}:{1'b0,diag_box[24:16],2'b11};
        wire [11:0] diag_t=I_shape_view==2?{1'b0,diag_box[15:8],3'b000}:{2'b0,diag_box[15:8],2'b00};
        wire [11:0] diag_b=I_shape_view==2?{1'b0,diag_box[7:0],3'b111}:{2'b0,diag_box[7:0],2'b11};
        wire [12:0] diag_sum_x={1'b0,diag_l}+{1'b0,diag_r};
        wire [12:0] diag_sum_y={1'b0,diag_t}+{1'b0,diag_b};
        wire [2:0] status;
        wire [1:0] category;
        wire [11:0] xl,xr,yt,yb,cx,cy;
        wire [31:0] seq,epoch,source_result;
        // Reader source_valid qualifies RGB pixels, not the whole raster.
        // It is normally low in horizontal/vertical blanking. source_bad
        // already rejects invalid ACTIVE pixels and mid-frame ID changes;
        // source reset separately resets the observer and flushes metadata.
        wire shape_cancel=source_bad;
        wire shape_enable=I_mode==12;
        reg [31:0] previous_epoch;
        reg previous_valid;
        reg [27:0] previous_config;
        wire [27:0] shape_config={I_area_min,I_area_max,I_shape_threshold,8'd0,I_shape_profile};
        wire annotation_flush=!shape_enable || !I_source_rst_n || shape_cancel || fault ||
            shape_config!=previous_config || epoch!=previous_epoch || (previous_valid&&!target);
        always @(posedge I_clk or negedge I_rst_n)begin
            if(!I_rst_n)begin previous_epoch<=0;previous_valid<=0;previous_config<={5'd6,5'd16,8'd128,8'd0,2'd1};end
            else begin previous_epoch<=epoch;previous_valid<=target;previous_config<=shape_config;end
        end
        shape_snapshot_r05 #(.PAPER_REQUIRED(0),.MULTI_TARGETS(3)) observer(.I_clk(I_clk),.I_rst_n(I_rst_n && I_source_rst_n),
            .I_enable(shape_enable),.I_cancel(shape_cancel),
            .I_frame_tick(S_motion_vs_d && !S_video_vsync_2d),
            .I_de(S_video_de_2d),.I_sof(S_user_delay[1]),.I_eol(S_last_delay[1]),
            .I_pixel_good(I_video_pixel_good && I_source_valid),.I_x(S_x_2d),.I_y(S_y_2d),
            .I_rgb(I_video_rd_data),.I_threshold(I_shape_threshold),.I_paper_threshold(8'd0),.I_profile(I_shape_profile),
            .I_area_min(I_area_min),.I_area_max(I_area_max),
            .I_source_frame_id(I_source_frame_id),.O_valid(target),.O_status(status),
            .O_class(category),.O_xmin(xl),.O_ymin(yt),.O_xmax(xr),.O_ymax(yb),
            .O_center_x(cx),.O_center_y(cy),.O_seq(seq),.O_source(source_result),
            .O_epoch(epoch),.O_multi_packets(S_multi_packets),.O_fault(fault),.O_busy(),.O_capture_drop(),
            .O_result_valid(),.O_result_paper(),.O_result_status(),.O_result_class(),.O_result_source(),.O_job_cycles(),
            .O_diag_code(S_shape_diag),.O_diag_pvalid(diag_pvalid),.O_diag_kvalid(diag_kvalid),
            .O_diag_pbox(diag_pbox),.O_diag_kbox(diag_kbox),.O_diag_source(diag_source),.O_diag_epoch(diag_epoch));
        // All three annotations now come from the frame-atomic list.
        // The former one-target annotation path is intentionally unused.
        assign S_shape_packet=178'd0;assign S_shape_class=2'd0;
        // 0x34 stays on the primary result: the largest confirmed target.
        assign O_shape_class=category;assign O_shape_valid=target;assign O_shape_count=S_shape_count;
    end else begin: no_shape
        assign S_multi_packets=249'd0;assign S_shape_diag=20'd0;assign S_shape_packet=178'd0;
        assign S_shape_class=2'd0;
        assign O_shape_class=2'd0;assign O_shape_valid=1'b0;assign O_shape_count=2'd0;
    end endgenerate
    // Selection uses the metadata's matching nine-clock mode, not live keys.
    reg [3:0] S_overlay_mode[0:8];
    reg [8:0] S_det_initialized;
    integer det_payload_stage;
    // Preserve the exact nine-clock packet API; reset only observation tokens.
    // Payload shifts without reset and is hidden until all stages are initialized.
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            S_det_initialized<=0;
            for(det_stage=0;det_stage<9;det_stage=det_stage+1)S_overlay_mode[det_stage]<=0;
        end else begin
            S_det_initialized<={S_det_initialized[7:0],1'b1};
            S_overlay_mode[0]<=I_mode;
            for(det_stage=1;det_stage<9;det_stage=det_stage+1)S_overlay_mode[det_stage]<=S_overlay_mode[det_stage-1];
        end
    end
    always @(posedge I_clk)begin
            S_det_delay[0]<=S_motion_enable ?
                {S_motion_overlay_snapshot[177:121],I_mode,S_motion_overlay_snapshot[116:0]} :
                {S_coords_valid,S_center_x,S_center_y,S_center_x_bcd,S_center_y_bcd,
                S_result_mode,S_det_source_frame_id,S_det_mode_epoch,
                S_detect,S_target,S_det_status,S_det_xmin,S_det_xmax,S_det_ymin,S_det_ymax};
        for(det_payload_stage=1;det_payload_stage<9;det_payload_stage=det_payload_stage+1)
            S_det_delay[det_payload_stage]<=S_det_delay[det_payload_stage-1];
    end
    wire [177:0] S_det_packet_aligned=S_det_initialized[8]?S_det_delay[8]:178'd0;
    // user now traverses the same LSC/ISP registers as DE. DE rising alone
    // marks every line, not every frame, and must not latch guard snapshots.
    wire S_guard_sof = S_vp_de && S_vp_user;

    wire [177:0] S_overlay_packet=S_overlay_mode[8]==12?S_shape_packet:S_det_packet_aligned;
    // The observer thresholds pre-LSC reader RGB. Preview the SAME samples,
    // then delay mask bits by LSC(5)+ISP(9), not only ISP(9).
    localparam integer SHAPE_PREVIEW_LAT=LSC_LAT+9;
    wire [9:0] S_shape_gray_sum={2'd0,I_video_rd_data[23:16]}+
        {1'd0,I_video_rd_data[15:8],1'b0}+{2'd0,I_video_rd_data[7:0]}+10'd2;
    reg [SHAPE_PREVIEW_LAT-1:0] S_shape_black_delay,S_shape_paper_delay;
    reg [1:0] S_shape_view_delay[0:8];
    integer shape_view_stage;
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            S_shape_black_delay<=0;S_shape_paper_delay<=0;
            for(shape_view_stage=0;shape_view_stage<9;shape_view_stage=shape_view_stage+1)S_shape_view_delay[shape_view_stage]<=0;
        end else begin
            S_shape_black_delay<={S_shape_black_delay[SHAPE_PREVIEW_LAT-2:0],S_shape_gray_sum[9:2]<I_shape_threshold};
            S_shape_paper_delay<={S_shape_paper_delay[SHAPE_PREVIEW_LAT-2:0],S_shape_gray_sum[9:2]>=I_paper_threshold};
            S_shape_view_delay[0]<=I_shape_view;
            for(shape_view_stage=1;shape_view_stage<9;shape_view_stage=shape_view_stage+1)
                S_shape_view_delay[shape_view_stage]<=S_shape_view_delay[shape_view_stage-1];
        end
    end
    wire [23:0] S_display_rgb=S_overlay_mode[8]!=12?S_vp_data:
        S_shape_view_delay[8]==1?(S_shape_black_delay[SHAPE_PREVIEW_LAT-1]?24'h000000:24'hffffff):
        S_shape_view_delay[8]==2?(S_shape_paper_delay[SHAPE_PREVIEW_LAT-1]?24'hffffff:24'h000000):S_vp_data;
    // 3x gain only for the picture path; the OSD keeps showing the raw -10..+10 value.
    wire signed [8:0] S_bright_video = $signed(I_bright_level) * 9'sd3;
    video_process #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT),.EXTERNAL_COLOUR_MASK(1),.DISPLAY_ENHANCE(DISPLAY_ENHANCE)) u_video_process(
        .I_colour_mask(S_colour_mask),
        .I_r_min     ( I_r_min        ),
        .I_r_dom     ( I_r_dom        ),
        .I_g_min     ( I_g_min        ),
        .I_g_dom     ( I_g_dom        ),
        .I_b_min     ( I_b_min        ),
        .I_b_dom     ( I_b_dom        ),
        .I_clk         ( I_clk            ),
        .I_rst_n       ( I_rst_n          ),
        .I_de          ( S_de_al         ),
        .I_data        ( S_video_use  ),
        .I_x           ( S_x_al          ),
        .I_y           ( S_y_al          ),
        .I_vsync       ( S_vs_al         ),
        .I_hsync       ( S_hs_al         ),
        .I_user        ( S_user_al        ),
        .I_last        ( S_last_al        ),
        .I_mode        ( I_mode           ),
        .I_invert_en   ( I_invert_en      ),
        .I_emboss_en   ( I_emboss_en      ),
        .I_sat_level   ( I_sat_level     ), // masks are captured before style changes
        .I_bright_level( S_bright_video     ),
        .I_wb_trim     ( I_wb_trim          ),
        .I_wb_trim_en  ( I_wb_trim_en       ),
        .O_de          ( S_vp_de          ),
        .O_data        ( S_vp_data        ),
        .O_x           ( S_vp_x           ),
        .O_y           ( S_vp_y           ),
        .O_vsync       ( S_vp_vsync       ),
        .O_hsync       ( S_vp_hsync       ),
        .O_user        ( S_vp_user        ),
        .O_last        ( S_vp_last        )
    );

    // Frame-rate readout for the OSD "FPS xx.x" line.  It counts the SAME
    // once-per-frame marker (I_video_user) that S_frame_cnt counts, so the FPS
    // line and the CNT line always describe the same stream.  The value is
    // computed entirely in this (pixel) clock domain from a 1 Hz toggle, and it
    // moves once a second, so it reaches the OSD as a slow, already-stable input
    // exactly like I_counter_bcd -- no extra pipeline stage is added.
    wire [11:0] S_fps_bcd;
    fps_meter u_fps_meter (
        .I_clk         ( I_clk          ),
        .I_rst_n       ( I_rst_n        ),
        .I_frame_start ( I_video_user   ),
        .I_tick_tog    ( I_tick_1hz_tog ),
        .O_fps_bcd     ( S_fps_bcd      )
    );

    // Six-clock OSD pipeline (three clocks more than the baseline).
    // Fixed latency applies to every mode and to both OSD on/off paths.
    // 'selected colour' thresholds for the OSD readout (follows the fill mode).
    wire [1:0] S_fill_color = (I_mode==4'd7||I_mode==4'd8)?2'd2:((I_mode==4'd9||I_mode==4'd10)?2'd1:2'd0);
    wire [7:0] S_adj_min = (S_fill_color==2'd1)?I_g_min:((S_fill_color==2'd2)?I_b_min:I_r_min);
    wire [7:0] S_adj_dom = (S_fill_color==2'd1)?I_g_dom:((S_fill_color==2'd2)?I_b_dom:I_r_dom);
    wire[23:0]S_legacy_osd_rgb,S_multi_color;wire S_multi_hit;
    shape_list_overlay #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT)) u_shape_list_overlay(
     .I_clk(I_clk),.I_rst_n(I_rst_n && I_source_rst_n),.I_de(S_vp_de),.I_enable(I_osd_en && I_mode==12),
     .I_x(S_vp_x),.I_y(S_vp_y),.I_packets(S_multi_packets),.O_hit(S_multi_hit),.O_color(S_multi_color));
    assign O_hdmi_data=S_multi_hit?S_multi_color:S_legacy_osd_rgb;
    red_object_osd_pipeline #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT), .DEBUG_MODE(DEBUG_MODE),
        .SHAPE_DIAGNOSTICS(1),.ZONE_GUARD(1),.GUARD_DIAGNOSTICS(1),.GUARD_X0(GUARD_X0),.GUARD_X1(GUARD_X1),.GUARD_Y0(GUARD_Y0),.GUARD_Y1(GUARD_Y1)) u_osd_pipeline(
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_de(S_vp_de), .I_rgb(S_display_rgb), .I_x(S_vp_x), .I_y(S_vp_y),
        .I_vsync(S_vp_vsync), .I_hsync(S_vp_hsync),
        .I_osd_en(I_osd_en), .I_mode(I_mode), .I_adj_idx(I_adj_idx),
            .I_fill_color(S_fill_color), .I_th_min(S_adj_min), .I_th_dom(S_adj_dom),
        .I_shape_value(I_adj_idx==8?I_shape_threshold:I_adj_idx==9?I_paper_threshold:I_adj_idx==10?{6'd0,I_shape_profile}:I_adj_idx==11?{3'd0,I_area_min}:{3'd0,I_area_max}),
        .I_show_temp(I_show_temp), .I_wb_temp(I_wb_temp),
        .I_wb_is_trim(I_wb_is_trim), .I_wb_trim_neg(I_wb_trim_neg),
        .I_gain_level(I_gain_level),   // composite gain control value shown by the gain item
        .I_sat_level(I_sat_level), .I_bright_level(I_bright_level),
        .I_counter_bcd({S_cnt_d5,S_cnt_d4,S_cnt_d3,S_cnt_d2,S_cnt_d1,S_cnt_d0}),
        .I_fps_bcd(S_fps_bcd),
        // L04: LAT row sources (already committed in the same clock by u_lat_calc)
        .I_lat_bcd(O_lat_bcd), .I_lat_state(O_lat_state),
        .I_debug_status(I_debug_status),
        .I_det_enable(S_overlay_packet[52] && S_overlay_mode[8]!=12),.I_det_target_valid(S_overlay_packet[51]),
        .I_det_status(S_overlay_packet[50:48]),
        .I_det_xmin(S_overlay_packet[47:36]),.I_det_xmax(S_overlay_packet[35:24]),
        .I_det_ymin(S_overlay_packet[23:12]),.I_det_ymax(S_overlay_packet[11:0]),
        .I_coords_valid(S_overlay_packet[177] && S_overlay_mode[8]!=12),
        .I_center_x_bcd(S_overlay_packet[152:137]),.I_center_y_bcd(S_overlay_packet[136:121]),
        .I_result_mode(S_overlay_packet[120:117]),.I_shape_class(S_shape_class),
        .I_shape_diag(S_shape_diag),.I_shape_diag_view(S_shape_view_delay[8]),
        .I_guard_enable(S_overlay_mode[8]==4'd13),.I_guard_sof(S_guard_sof),
        .I_guard_reason(S_guard_reason),
        .I_guard_state(S_guard_state),.I_guard_alarm(S_guard_alarm),
        .I_guard_center_valid(S_guard_center_valid),
        .I_guard_center_x(S_guard_center_x),.I_guard_center_y(S_guard_center_y),
        .O_vsync(O_hdmi_vsync), .O_hsync(O_hdmi_hsync),
        .O_de(O_hdmi_de), .O_rgb(S_legacy_osd_rgb)
    );

    // ---- L03: 终点采样与会话内延迟计算 -------------------------------------
    // T_end 定义（L01 冻结）：本帧第一个出现在 O_hdmi_de 上的有效像素，也就是该
    // 像素真正打到 hdmi_tx 数字输入的那一刻（顶层 O_hdmi_de 直连
    // hdmi_tx.I_video_in_de，中间没有额外寄存）。
    // 用显示时序 vsync 上升沿武装，在 HDMI TX 消费首个 O_hdmi_de 的同一沿
    // 采样终点。描述符先在 reader 对齐的首像素上冻结，故不另加 RGB 流水。
    // 帧级故障可在首像素之后撤销计算结果，OSD 自身保证整行快照原子性。
    reg S_lat_vs_d, S_lat_arm;
    // Sample on the SAME edge as hdmi_tx consumes its first DE pixel, not
    // one edge later through a registered pulse.
    wire S_lat_first_pix = S_lat_arm && O_hdmi_de &&
                          !(I_video_vsync && !S_lat_vs_d);
    reg S_lat_source_seen, S_lat_frame_fault;
    reg [31:0] S_lat_frame_source, S_lat_frame_start;
    reg [15:0] S_lat_frame_session;
    // Reader quality/source tags align with S_video_de_2d, just as in the
    // existing detection paths. Blank cycles are NOT invalid source pixels.
    wire S_lat_bad_pixel = !I_video_pixel_good || !I_source_valid ||
        !I_lat_valid || !I_source_rst_n ||
        (S_lat_source_seen && ((I_source_frame_id != S_lat_frame_source) ||
         (I_lat_start_tick != S_lat_frame_start) ||
         (I_lat_session != S_lat_frame_session)));
    wire S_lat_abort = S_lat_frame_fault ||
        (S_video_de_2d && S_lat_bad_pixel) ||
        (S_lat_source_seen && !I_source_rst_n);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_lat_vs_d      <= 1'b0;
            S_lat_arm       <= 1'b0;
            S_lat_source_seen <= 1'b0;
            S_lat_frame_fault <= 1'b0;
            S_lat_frame_source <= 32'd0;
            S_lat_frame_start <= 32'd0;
            S_lat_frame_session <= 16'd0;
        end else begin
            S_lat_vs_d <= I_video_vsync;
            if(I_video_vsync && !S_lat_vs_d) begin
                S_lat_arm       <= 1'b1;
                S_lat_source_seen <= 1'b0;
                S_lat_frame_fault <= 1'b0;
            end else if(S_lat_arm && O_hdmi_de) begin
                S_lat_arm       <= 1'b0;
            end
            if(!(I_video_vsync && !S_lat_vs_d)) begin
                if(S_video_de_2d) begin
                    if(!S_lat_source_seen) begin
                        S_lat_source_seen <= 1'b1;
                        S_lat_frame_source <= I_source_frame_id;
                        S_lat_frame_start <= I_lat_start_tick;
                        S_lat_frame_session <= I_lat_session;
                    end
                    if(S_lat_bad_pixel) S_lat_frame_fault <= 1'b1;
                end
                if(S_lat_source_seen && !I_source_rst_n)
                    S_lat_frame_fault <= 1'b1;
            end
        end
    end

    lat_calc_bcd #(
        .LIFETIME_TICKS (32'd100000),
        .MAX_TICKS      (32'd9999)
    ) u_lat_calc (
        .I_clk          (I_clk),
        .I_rst_n        (I_rst_n),
        .I_tick_now     (I_tick_now),
        .I_session_now  (I_session_now),
        .I_time_ready   (I_time_ready),
        .I_frame_pulse  (S_lat_first_pix),
        .I_frame_abort  (S_lat_abort),
        .I_lat_valid    (I_lat_valid && S_lat_source_seen && !S_lat_abort),
        .I_desc_session (S_lat_frame_session),
        .I_start_tick   (S_lat_frame_start),
        .O_state        (O_lat_state),
        .O_bcd          (O_lat_bcd),
        .O_delta_tick   (O_lat_delta),
        .O_snapshot_tog (O_lat_snap_tog),
        .O_desc_session ()
    );

    // Brightness statistics for the automatic gain loop, tapped BEFORE video_process
    // (centre 1024x512 window), i.e. on the camera picture itself. Tapping after it meant
    // the loop measured the effects of invert / cartoon / emboss / edge / fill modes,
    // saturation and brightness: those bias the mean, and a non-monotonic effect such as
    // invert (255-x) actually flips the sign of the loop, which drove the gain to its limit
    // and left the picture washed out. The on-screen readouts cannot bias it either, since
    // the OSD overlay is applied much later.
    luma_stats #(.IMG_WIDTH(IMG_WIDTH), .IMG_HEIGHT(IMG_HEIGHT)) u_luma_stats (
        .I_clk(I_clk), .I_rst_n(I_rst_n),
        .I_de(S_video_de_2d), .I_pixel_good(I_video_pixel_good), .I_vsync(S_video_vsync_2d),
        .I_rgb(I_video_rd_data), .I_x(S_x_2d), .I_y(S_y_2d),
        .O_y_mean(O_y_mean), .O_y_filt(O_y_filt), .O_sat_count(O_sat_count),
        // Controller trust means measured pixels, not UNCLIPPED pixels.
        // luma_stats suppresses the entire event if reader quality was bad.
        .O_pix_count(O_valid_count), .O_stats_valid(O_stats_valid),
        .O_valid_count());
endmodule
