

module design_top_wrapper (
    input wire        I_sys_clk,
    input wire        I_rst_n,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    input wire        I_sw1,
    input wire        I_sw2,
    input wire        I_sw3,
    input wire        I_sw4,
    input wire        I_key1,
    input wire        I_key2,
    input wire        I_key3,
    input wire        I_key4,

    output wire       O_alarm_led, // V1 LED1 / D10, active high; mode6 only
    output wire       O_alarm_buzzer_n, // V1 C12, active low; high=silent
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n,

    // ---- PH1P35 hard MCU (RISC-V) UART ----
    output wire       O_mcu_uart0_tx,   // V2
    input  wire       I_mcu_uart0_rx,   // V1
    output wire       O_mcu_uart1_tx,   // U2
    input  wire       I_mcu_uart1_rx    // U1
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
	reg  [17:0] 	S_cam_power_cnt;
	reg         	S_cam_pwdnb;
	reg         	S_cam_i2c_ready;
	(* keep = "true" *) wire [15:0] S_cam_sensor_id;
	(* keep = "true" *) wire        S_cam_sensor_id_ok;
	(* keep = "true" *) wire        S_cam_i2c_error;
	localparam integer CAM_PWDNB_LOW_CYCLES = 23810;  // >= 1 ms at 23.809524 MHz
	localparam integer CAM_I2C_START_CYCLES = 142857; // release + >= 5 ms
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy;
    wire [1:0]  S_display_slot;
    wire        S_frame_available;
    wire        S_detect_now;      // ¼ì²âÖÐ
    wire        S_target_now;      // ÕÒµ½Ä¿±ê
    wire [19:0] S_target_area;      // 2026-11-01 colour area from the mixer (pixel domain)
    wire [1:0]  S_shape_class;      // 2026-11-02 shape class 0 unknown 1 circle 2 tri 3 rect
    wire        S_shape_valid;      // 2026-11-02 shape class confirmed (temporal)
    wire [1:0]  S_shape_count;      // 2026-11-02 confirmed targets in the shape list
    (* keep="true" *) wire S_write_overflow, S_read_underflow; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire [23:0] S_probe_rgb;      // LSC-corrected pixel for the colour probe
    wire        S_probe_de;
    wire        S_probe_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    wire        S_video_pixel_good; // B09: reader quality, aligned to this RGB.
    wire [31:0] S_published_source_id,S_display_source_id;
    wire        S_display_source_valid;
    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire[3:0]   S_hdmi_debug_status;
    wire        S_lane_error_any;

    reg  [1:0]  S_sw1_sync;
    reg  [1:0]  S_sw2_sync;
    reg  [1:0]  S_sw3_sync;
    reg  [1:0]  S_sw4_sync;
    reg [3:0]  S_mode;
    reg [3:0]  S_mode_stable;
    reg [19:0] S_mode_stable_count;
    reg [4:0] S_sat_frame;
    reg signed [8:0] S_bright_frame;
    reg         S_osd_on;
    reg         S_osd_frame;
    reg [3:0]   S_adj_idx;
    wire [7:0] S_shape_threshold,S_shape_threshold_f,S_paper_threshold,S_paper_threshold_f;
    wire [1:0] S_shape_profile,S_shape_profile_f,S_shape_view,S_shape_view_f;
    wire [4:0] S_area_min,S_area_max,S_area_min_f,S_area_max_f;
    reg         S_wb_manual;
    reg         S_wb_manual_f;
    reg         S_gain_manual;
    reg [15:0]  S_gain_val;
    reg         S_gain_manual_f;
    reg         S_ae_manual_f;
    // exposure (AE) item state -- all owned by the AE block below (single driver)
    reg         S_ae_manual, S_flicker, S_ae_load, S_flicker_f;
  reg  [7:0]  S_lsc_edge, S_lsc_centre;   // LSC knobs: E (edge red), C (centre)
  reg  [7:0]  S_lsc_lift;                   // K (common brightness lift), item 15
    reg  [3:0]  S_ae_idx;
    reg  [15:0] S_ae_load_val, S_ae_ladder_f;
    reg  [7:0]  S_ae_disp_f;
    reg [15:0]  S_gain_val_f;
    reg signed [6:0] S_wb_trim;    // manual white-balance trim (+ = warmer), 1/128 per step
    reg signed [6:0] S_wb_trim_f;
    // The on-screen temperature is an estimate; let it follow the manual trim with a
    // simple linear approximation (~50 K of apparent temperature per trim step).
    wire [6:0]         S_wb_trim_abs = S_wb_trim_f[6] ? (~S_wb_trim_f + 7'sd1) : S_wb_trim_f;
    wire               S_wb_trim_neg = S_wb_trim_f[6];
    wire signed [14:0] S_wb_shift   = S_wb_trim_f * 7'sd50;
    wire signed [14:0] S_wb_disp_t  = $signed({2'b0, S_wb_est_f}) + S_wb_shift;
    wire [13:0]        S_wb_disp    = (S_wb_disp_t < 15'sd3000) ? 14'd3000 :
                                      ((S_wb_disp_t > 15'sd8000) ? 14'd8000 : S_wb_disp_t[13:0]);
    wire [7:0]  S_y_mean;
    wire [7:0]  S_y_filt;
    wire [23:0] S_sat_count;
    wire [23:0] S_valid_count;
    wire        S_stats_valid;
    wire [15:0] S_agc_val;
    wire        S_agc_req;
    reg         S_agc_load;
    reg [15:0]  S_agc_load_val;
    reg [12:0]  S_wb_est_f;
    wire [12:0] S_isp_wb_temp;
    reg [7:0]   S_r_min, S_r_dom, S_g_min, S_g_dom, S_b_min, S_b_dom;
    // 2026-10-24 same-stage samples: green 45/62/54; white
    // 180/179/189, 133/132/138, 121/121/128. DOM=4 leaves 4 DN
    // to the weakest green separation (G-B=8). Board calibration remains required.
    localparam [7:0] GREEN_DOM_DEFAULT=8'd64;
    reg [7:0]   S_r_min_f, S_r_dom_f, S_g_min_f, S_g_dom_f, S_b_min_f, S_b_dom_f;
    wire        S_invert_en;
    wire        S_emboss_en;
    wire        S_key1_pulse;
    wire        S_key2_pulse;
    wire        S_key3_pulse;
    wire        S_key4_pulse;
    reg  [4:0]  S_sat_level;
    reg signed [8:0] S_bright_level;

    assign S_ISP_O_tready=1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    // SW1..SW4: 2-stage sync into the HDMI pixel domain (mixer/OSD domain)
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_sw1_sync <= 2'b00;
            S_sw2_sync <= 2'b00;
            S_sw3_sync <= 2'b00;
            S_sw4_sync <= 2'b00;
        end else begin
            S_sw1_sync <= {S_sw1_sync[0], I_sw1};
            S_sw2_sync <= {S_sw2_sync[0], I_sw2};
            S_sw3_sync <= {S_sw3_sync[0], I_sw3};
            S_sw4_sync <= {S_sw4_sync[0], I_sw4};
        end
    end

    // The four DIP switches read inverted on this board: the mechanical "ON"
    // position is LOW at the pin. Invert all four so that ON = logic 1.
    // Mode = {SW4, SW3, SW2, SW1}: 0000=original, 0001=edge, 0010=invert,
    // 0011=emboss, 0100=cartoon, 0101=red fill, 0110=red object;
    // 0111=blue fill, 1001=green fill. 1000/1010 reserve blue/green object
    // detection (not implemented); 1011..1111 remain reserved pass-through.
    wire [3:0] S_code = {~S_sw4_sync[1], ~S_sw3_sync[1], ~S_sw2_sync[1], ~S_sw1_sync[1]};
    // Mechanical switches are a multi-bit asynchronous configuration.
    // Require the entire code to remain unchanged for ~10 ms, then latch
    // it during vertical blank; never apply a transient mixed code mid-frame.
    // The OSD on/off and the adjustment target are latched on the same edge.
    // ´Ó»úÊä³öµÄ¸²¸ÇÇëÇó£¨AHB Óò£©
    wire [3:0] S_ovr_mode;
    wire       S_ovr_en;
    wire       S_ovr_tog;
    wire [3:0] S_cmd_op;      // ½×¶Î 4£ºOSD ÃüÁî
    wire [3:0] S_cmd_arg;
    wire [3:0] S_cmd_seq;

    // ---------------- ½×¶Î 3£º´®¿Ú¸²¸Ç + ²¦ÂëÇÀ»Ø ----------------
    // AHB(150MHz) -> ÏñËØÓò£ºÖð±ÈÌØ 2FF£¨Âý±äÁ¿£©
    reg [1:0] S_ovr_mode_s0, S_ovr_mode_s1, S_ovr_mode_s2, S_ovr_mode_s3;
    reg [1:0] S_ovr_en_s;
    reg [2:0] S_ovr_tog_s;      // 3FF£¬ÓÃÓÚ¿çÓò±ßÑØ¼ì²â
    reg       S_ovr_tog_d;
    reg       S_sw_kill;        // 1 = ²¦ÂëÒÑÇÀ»Ø¿ØÖÆÈ¨

    // ²¦Âë"ÉÏ´Î½ÓÊÜÖµ"¸ú×Ù£ºÖ»Òª¿ª¹ØÀë¿ªÕâ¸öÖµ£¬Á¢¼´ÇÀ»Ø
    reg [3:0]  S_code_stable;
    reg [19:0] S_code_cnt;
    wire       S_sw_moving = (S_code != S_code_stable);

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_ovr_mode_s0<=2'd0;S_ovr_mode_s1<=2'd0;
            S_ovr_mode_s2<=2'd0;S_ovr_mode_s3<=2'd0;
            S_ovr_en_s<=2'd0;S_ovr_tog_s<=3'd0;S_ovr_tog_d<=1'b0;
            S_sw_kill<=1'b0;S_code_stable<=4'd0;S_code_cnt<=20'd0;
        end else begin
            S_ovr_mode_s0<={S_ovr_mode_s0[0],S_ovr_mode[0]};
            S_ovr_mode_s1<={S_ovr_mode_s1[0],S_ovr_mode[1]};
            S_ovr_mode_s2<={S_ovr_mode_s2[0],S_ovr_mode[2]};
            S_ovr_mode_s3<={S_ovr_mode_s3[0],S_ovr_mode[3]};
            S_ovr_en_s  <={S_ovr_en_s[0],  S_ovr_en};
            S_ovr_tog_s <={S_ovr_tog_s[1:0],S_ovr_tog};
            S_ovr_tog_d <= S_ovr_tog_s[2];

            // MCU Ã¿´ÎÐ´ OVR_CTL ¶¼»á·­×ª tog£º¼ì²âµ½¾ÍÖØÐÂ½Ó¹Ü
            if(S_ovr_tog_s[2]!=S_ovr_tog_d) S_sw_kill<=1'b0;
            else if(S_sw_moving)            S_sw_kill<=1'b1;   // ²¦ÂëÒ»¶¯Á¢¼´ÇÀ»Ø

            if(S_sw_moving) begin
                S_code_stable<=S_code; S_code_cnt<=20'd0;
            end else if(S_code_cnt<20'd750000) begin
                S_code_cnt<=S_code_cnt+1'b1;
            end
        end
    end

    // ---------------- ½×¶Î 4£º´®¿Ú OSD ÃüÁî ----------------
    // AHB(150MHz) -> ÏñËØÓò£ºop/arg/seq Öð±ÈÌØ 2FF£¬seq ±ä»¯¼´Ò»´ÎÊÂ¼þ
    reg [1:0] S_cseq0,S_cseq1,S_cseq2,S_cseq3;
    reg [1:0] S_cop0,S_cop1,S_cop2,S_cop3;
    reg [1:0] S_carg0,S_carg1,S_carg2,S_carg3;
    reg [3:0] S_cseq_d;
    reg [3:0] S_cop_l, S_carg_l;
    reg       S_cmd_ev;
    reg       S_cmd_pend;

    wire [3:0] S_cseq_now = {S_cseq3[1],S_cseq2[1],S_cseq1[1],S_cseq0[1]};
    wire [3:0] S_cop_now  = {S_cop3[1], S_cop2[1], S_cop1[1], S_cop0[1]};
    wire [3:0] S_carg_now = {S_carg3[1],S_carg2[1],S_carg1[1],S_carg0[1]};

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_cseq0<=2'd0;S_cseq1<=2'd0;S_cseq2<=2'd0;S_cseq3<=2'd0;
            S_cop0<=2'd0;S_cop1<=2'd0;S_cop2<=2'd0;S_cop3<=2'd0;
            S_carg0<=2'd0;S_carg1<=2'd0;S_carg2<=2'd0;S_carg3<=2'd0;
            S_cseq_d<=4'd0;S_cop_l<=4'd0;S_carg_l<=4'd0;
            S_cmd_ev<=1'b0;S_cmd_pend<=1'b0;
        end else begin
            S_cseq0<={S_cseq0[0],S_cmd_seq[0]}; S_cseq1<={S_cseq1[0],S_cmd_seq[1]};
            S_cseq2<={S_cseq2[0],S_cmd_seq[2]}; S_cseq3<={S_cseq3[0],S_cmd_seq[3]};
            S_cop0 <={S_cop0[0], S_cmd_op[0]};  S_cop1 <={S_cop1[0], S_cmd_op[1]};
            S_cop2 <={S_cop2[0], S_cmd_op[2]};  S_cop3 <={S_cop3[0], S_cmd_op[3]};
            S_carg0<={S_carg0[0],S_cmd_arg[0]}; S_carg1<={S_carg1[0],S_cmd_arg[1]};
            S_carg2<={S_carg2[0],S_cmd_arg[2]}; S_carg3<={S_carg3[0],S_cmd_arg[3]};

            // seq ±ä»¯ -> pend ´òÒ»ÅÄ -> ÔÙ´òÒ»ÅÄ²úÉú 1 ÅÄÊÂ¼þ
            // ÕâÑù²ÉÑù op/arg Ê±ËüÃÇÒÑ¾­ÎÈ¶¨£¨4 Î»¿çÓò¿ÉÄÜÓÐÎ»Æ«Ð±£©
            S_cseq_d <= S_cseq_now;
            // Every command increments seq: use its single toggling LSB as the event.
            // Multi-bit binary CDC comparisons can duplicate a command on carry/wrap.
            if(S_cseq_now[0] != S_cseq_d[0]) S_cmd_pend <= 1'b1;
            else                       S_cmd_pend <= 1'b0;

            S_cmd_ev <= S_cmd_pend;
            if(S_cmd_pend) begin
                S_cop_l  <= S_cop_now;
                S_carg_l <= S_carg_now;
            end
        end
    end

    wire S_cmd_up   = S_cmd_ev && (S_cop_l == 4'd3);
    wire S_cmd_down = S_cmd_ev && (S_cop_l == 4'd4);
    wire S_cmd_next = S_cmd_ev && (S_cop_l == 4'd2);
    wire S_cmd_jump = S_cmd_ev && (S_cop_l == 4'd1);
    wire S_cmd_k3   = S_cmd_ev && (S_cop_l == 4'd5);   // ×Ô¶¯/ÊÖ¶¯ÇÐ»»

    // Óë°´¼ü²¢ÁÐ£º´®¿ÚÃüÁîµÈÐ§ÓÚ°´ K1/K2/K4£¨²½³¤/ÏÞ·ù/Ç°ÖÃÌõ¼þÈ«²¿Ò»ÖÂ£©
    wire S_up_pulse   = S_key1_pulse | S_cmd_up;
    wire S_down_pulse = S_key2_pulse | S_cmd_down;
    wire S_next_pulse = S_key4_pulse | S_cmd_next;
    wire S_k3_pulse   = S_key3_pulse | S_cmd_k3;

    wire [3:0] S_ovr_mode_now = {S_ovr_mode_s3[1],S_ovr_mode_s2[1],
                                 S_ovr_mode_s1[1],S_ovr_mode_s0[1]};
    wire       S_ovr_active   = S_ovr_en_s[1] && !S_sw_kill;

    wire [3:0] mode_candidate = S_ovr_active ? S_ovr_mode_now : S_code;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_mode<=4'd0;S_mode_stable<=4'd0;S_mode_stable_count<=0;
            S_sat_frame<=16;S_bright_frame<=0;S_osd_frame<=1'b1;
            // Deterministic control packet before the first display vsync.
            S_gain_manual_f<=1'b1;S_gain_val_f<=16'd160;S_ae_manual_f<=1'b1;
            S_r_min_f<=8'd24;S_r_dom_f<=8'd104;S_g_min_f<=8'd24;S_g_dom_f<=GREEN_DOM_DEFAULT;S_b_min_f<=8'd24;S_b_dom_f<=8'd104;
        end else begin
            if(mode_candidate!=S_mode_stable) begin
                S_mode_stable<=mode_candidate;S_mode_stable_count<=0;
            end else if(S_mode_stable_count<20'd750000)
                S_mode_stable_count<=S_mode_stable_count+1'b1;
            if(S_hdmi_vsync) begin
                if(S_mode_stable_count==20'd750000) S_mode<=S_mode_stable;
                S_sat_frame<=S_sat_level;S_bright_frame<=S_bright_level;
                S_wb_manual_f<=S_wb_manual;S_wb_trim_f<=S_wb_trim;
        S_gain_manual_f<=S_gain_manual;S_gain_val_f<=S_gain_val;S_ae_manual_f<=S_ae_manual;
                S_wb_est_f<=S_isp_wb_temp;
            S_r_min_f<=S_r_min;S_r_dom_f<=S_r_dom;S_g_min_f<=S_g_min;S_g_dom_f<=S_g_dom;S_b_min_f<=S_b_min;S_b_dom_f<=S_b_dom;
                S_osd_frame<=S_osd_on;
            end
        end
    end
    assign S_invert_en   = (S_mode == 4'd2);
    assign S_emboss_en   = (S_mode == 4'd3);

    // KEY1 = step up, KEY2 = step down, KEY3 = toggle adjustment target,
    // KEY4 = toggle OSD. All debounced 20 ms, active-low, in the pixel domain.
    key_debounce #(
        .CLK_FREQ         ( 75_000_000 ),
        .DEBOUNCE_TIME_MS ( 20         )
    ) u_kd1 (
        .I_clk   ( S_hdmi_pixel_clk ),
        .I_rst_n ( S_hdmi_rst_n     ),
        .I_button( I_key1           ),
        .O_pulse ( S_key1_pulse     )
    );

    key_debounce #(
        .CLK_FREQ         ( 75_000_000 ),
        .DEBOUNCE_TIME_MS ( 20         )
    ) u_kd2 (
        .I_clk   ( S_hdmi_pixel_clk ),
        .I_rst_n ( S_hdmi_rst_n     ),
        .I_button( I_key2           ),
        .O_pulse ( S_key2_pulse     )
    );

    key_debounce #(
        .CLK_FREQ         ( 75_000_000 ),
        .DEBOUNCE_TIME_MS ( 20         )
    ) u_kd3 (
        .I_clk   ( S_hdmi_pixel_clk ),
        .I_rst_n ( S_hdmi_rst_n     ),
        .I_button( I_key3           ),
        .O_pulse ( S_key3_pulse     )
    );
    // ---- LSC calibration instrument (2026-10-18) ----------------------------
    // 2026-10-24: ×Ô¶¨Òå UART£¨uart_tx_115200 + ËÍÊýÖÙ²Ã£©ÒÑÕûÌåÒÆ³ý£¬
    // ±£³Ö pin.adc Àï V2 µÄÔ¼ÊøÓÐÐ§£»½ÓÈë IP Ê±°ÑÕâÒ»ÐÐÌæ»»³É IP µÄ TX Êä³ö¡£
    localparam K3_UART = 1'b0;      // 0 = KEY3 ×öÊÖ¶¯/×Ô¶¯ÇÐ»»£¨Ô­´®¿Ú´¥·¢¹¦ÄÜÒÑËæ×Ô¶¨Òå UART ÒÆ³ý£©

    key_debounce #(
        .CLK_FREQ         ( 75_000_000 ),
        .DEBOUNCE_TIME_MS ( 20         )
    ) u_kd4 (
        .I_clk   ( S_hdmi_pixel_clk ),
        .I_rst_n ( S_hdmi_rst_n     ),
        .I_button( I_key4           ),
        .O_pulse ( S_key4_pulse     )
    );

    // Items: 0=saturation 1=brightness 2=MIN 3=DOM. MIN/DOM act on the colour of the
    // current fill mode (0101=red, 0111=blue, 1001=green); they are disabled outside
    // those modes. Thresholds: default 48/20, step 4, range 0..128.
    // Adjustment follows the ACTIVE displayed mode, not an uncommitted DIP
    // candidate. A candidate can bounce or wait for vertical blank.
    wire [1:0] S_fill_color = (S_mode==4'd7||S_mode==4'd8)?2'd2:((S_mode==4'd9||S_mode==4'd10)?2'd1:2'd0);
    wire       S_fill_mode  = (S_mode==4'd5)||(S_mode==4'd7)||(S_mode==4'd9);
    wire S_color_detect=(S_mode==6)||(S_mode==8)||(S_mode==10);
    wire S_color_adjust=S_fill_mode||S_color_detect;
    shape_adjust_r05b u_shape_threshold(
        .I_clk(S_hdmi_pixel_clk),.I_rst_n(S_hdmi_rst_n),.I_vblank(S_hdmi_vsync),
        .I_mode(S_mode),.I_item(S_adj_idx),.I_up(S_up_pulse),.I_down(S_down_pulse),
        .O_gray(S_shape_threshold),.O_gray_frame(S_shape_threshold_f),
        .O_paper(S_paper_threshold),.O_paper_frame(S_paper_threshold_f),
        .O_profile(S_shape_profile),.O_profile_frame(S_shape_profile_f),
        .O_view(S_shape_view),.O_view_frame(S_shape_view_f),
        .O_area_min(S_area_min),.O_area_max(S_area_max),
        .O_area_min_frame(S_area_min_f),.O_area_max_frame(S_area_max_f));
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_sat_level    <= 5'd16;
            S_bright_level <= 9'sd0;
            S_r_min<=8'd24; S_r_dom<=8'd104;
            S_g_min<=8'd24; S_g_dom<=GREEN_DOM_DEFAULT;
            S_b_min<=8'd24; S_b_dom<=8'd104;
            S_wb_trim <= 7'sd0; S_gain_val <= 16'd160;   // this block owns both registers
        end
        else case(S_adj_idx)
            3'd0: begin
                if(S_up_pulse && (S_sat_level < 5'd31))
                    S_sat_level <= S_sat_level + 5'd1;
                else if(S_down_pulse && (S_sat_level > 5'd0))
                    S_sat_level <= S_sat_level - 5'd1;
            end
            3'd1: begin
                // white balance: K1/K2 adjust the colour temperature while in manual
                // white balance: K1/K2 nudge the warm/cool trim around the automatic result
                if(S_wb_manual) begin
                    if(S_up_pulse && (S_wb_trim < 7'sd31)) S_wb_trim <= S_wb_trim + 7'sd1;
                    else if(S_down_pulse && (S_wb_trim > -7'sd32)) S_wb_trim <= S_wb_trim - 7'sd1;
                end
            end
            3'd2: begin
                // composite gain control value (16..176), adjustable in manual mode only;
                // the demo gain table already performs the analogue + digital-fine mapping
                // automatic -> manual on this item: take over the current automatic value.
                // This MUST sit outside the if(S_gain_manual) guard below: on the transition the
                // register still holds 0 (the toggle is non-blocking), so the guarded version
                // never ran and the manual value stayed at its stale 160 -> the picture jumped.
                if(!K3_UART && S_k3_pulse && !S_gain_manual) S_gain_val <= S_agc_val;
                if(S_gain_manual) begin
                    // step of 5 control units (~1.35 dB of fine gain); saturate at 16 / 176
                    if(S_up_pulse) S_gain_val <= (S_gain_val <= 16'd171) ? (S_gain_val + 16'd5) : 16'd176;
                    else if(S_down_pulse) S_gain_val <= (S_gain_val >= 16'd21) ? (S_gain_val - 16'd5) : 16'd16;
                end
            end
            3'd5: begin
                if(S_up_pulse)
                    S_bright_level <= (S_bright_level > 9'sd9) ? 9'sd10 : S_bright_level + 9'sd1;
                else if(S_down_pulse)
                    S_bright_level <= (S_bright_level < -9'sd9) ? -9'sd10 : S_bright_level - 9'sd1;
            end
            3'd3: case(S_fill_color)
                2'd1: begin
                    if(S_up_pulse && (S_g_min <= 8'd124)) S_g_min <= S_g_min + 8'd4;
                    else if(S_down_pulse && (S_g_min >= 8'd4)) S_g_min <= S_g_min - 8'd4;
                end
                2'd2: begin
                    if(S_up_pulse && (S_b_min <= 8'd124)) S_b_min <= S_b_min + 8'd4;
                    else if(S_down_pulse && (S_b_min >= 8'd4)) S_b_min <= S_b_min - 8'd4;
                end
                default: begin
                    if(S_up_pulse && (S_r_min <= 8'd124)) S_r_min <= S_r_min + 8'd4;
                    else if(S_down_pulse && (S_r_min >= 8'd4)) S_r_min <= S_r_min - 8'd4;
                end
            endcase
            3'd4: case(S_fill_color)
                2'd1: begin
                    if(S_up_pulse && (S_g_dom <= 8'd124)) S_g_dom <= S_g_dom + 8'd4;
                    else if(S_down_pulse && (S_g_dom >= 8'd4)) S_g_dom <= S_g_dom - 8'd4;
                end
                2'd2: begin
                    if(S_up_pulse && (S_b_dom <= 8'd124)) S_b_dom <= S_b_dom + 8'd4;
                    else if(S_down_pulse && (S_b_dom >= 8'd4)) S_b_dom <= S_b_dom - 8'd4;
                end
                default: begin
                    if(S_up_pulse && (S_r_dom <= 8'd124)) S_r_dom <= S_r_dom + 8'd4;
                    else if(S_down_pulse && (S_r_dom >= 8'd4)) S_r_dom <= S_r_dom - 8'd4;
                end
            endcase
            default: begin end // AE/debug keys must not alter colour thresholds.
        endcase
    end

    // KEY4 cycles the adjustment item: 0=saturation -> 1=white balance -> 2=brightness
    // -> 3=MIN -> 4=DOM (MIN/DOM reachable only inside the three fill modes).
    // KEY3 toggles automatic/manual for the current item -- white balance for now, and the
    // same per-item pattern carries over to gain and exposure later.
    // The OSD is permanently on. Everything takes effect at the next vsync.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_adj_idx   <= 3'd7;   // power-up: show the exposure item first
            S_osd_on    <= 1'b1;
            S_wb_manual <= 1'b0;
            // 2026-10-18 (user request): power-up in MANUAL gain, value 160.
            // Switch to 1'b0 for power-up auto.
            S_gain_manual  <= 1'b1;
            S_agc_load     <= 1'b0;
            S_agc_load_val <= 16'd160;
        end else begin
            S_agc_load <= 1'b0;            // one-cycle pulse for the AGC handover
            if(S_cmd_jump) begin
                S_adj_idx <= S_carg_l;          // ´®¿ÚÖ±½ÓÌøÏî
            end else if(S_next_pulse) begin
                // user order (2026-10-14): AE -> AG -> AWB -> (MIN -> DOM, fill mode only)
                //                          -> ï¿½ï¿½ï¿½ï¿½ -> ï¿½ï¿½ï¿½ï¿½ -> AE
                case(S_adj_idx)
                    3'd6:    S_adj_idx <= 3'd2;                        // AE  -> AG
                    3'd2:    S_adj_idx <= 3'd1;                        // AG  -> AWB
                    3'd1:    S_adj_idx <= S_mode==12 ? 4'd8 : (S_color_adjust ? 4'd3 : 4'd0);
                    4'd8:    S_adj_idx <= 4'd9;
                    4'd9:    S_adj_idx <= 4'd10;
                    4'd10:   S_adj_idx <= 4'd11;
                    4'd11:   S_adj_idx <= S_mode==12 ? 4'd12 : 4'd0;
                    4'd12:   S_adj_idx <= 4'd0;                      // detector item -> saturation
                    3'd3:    S_adj_idx <= 3'd4;                        // MIN -> DOM
                    3'd4:    S_adj_idx <= S_color_detect ? 4'd11 : 4'd0; // DOM -> AMIN for detectors
                    3'd0:    S_adj_idx <= 3'd5;                        // ï¿½ï¿½ï¿½ï¿½ -> ï¿½ï¿½ï¿½ï¿½
                    3'd7:    S_adj_idx <= 3'd6;                        // TG -> AE
                    default: S_adj_idx <= 3'd7;                        // last -> TG                        // ï¿½ï¿½ï¿½ï¿½ -> AE
                endcase
            end
            if(!K3_UART && S_k3_pulse) begin
                if(S_adj_idx == 3'd1) S_wb_manual   <= ~S_wb_manual;
                if(S_adj_idx == 3'd2) begin
                    S_gain_manual <= ~S_gain_manual;
                    if(S_gain_manual) begin          // manual -> automatic: resume from the manual value
                        S_agc_load     <= 1'b1;
                        S_agc_load_val <= S_gain_val;
                    end else begin                    // automatic -> manual: take over the current value
                    end
                end
            end
            if(!S_color_adjust && ((S_adj_idx == 3'd3) || (S_adj_idx == 3'd4))) S_adj_idx <= 3'd0;
            if(S_mode!=12 && (S_adj_idx==8||S_adj_idx==9||S_adj_idx==10||S_adj_idx==12))S_adj_idx<=0;
            if(S_mode!=12 && !S_color_detect && S_adj_idx==11)S_adj_idx<=0;
            S_osd_on <= 1'b1;
        end
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };
	
	// SC520CSä¸Šç”µåŽå…ˆä¿æŒPWDNB/XSHUTDNä¸ºä½Žï¼Œå†é‡Šæ”¾å¹¶ç­‰å¾…å†…éƒ¨ä¸Šç”µå®Œæˆï¼Œ
	// é¿å…PLLåˆšé”å®šå°±å¼€å§‹I2Cåˆå§‹åŒ–è€Œé€ æˆå¯„å­˜å™¨å¶å‘å†™å…¥å¤±è´¥ã€‚
	always @(posedge S_24m_clk or negedge S_pll_lock) begin
		if (!S_pll_lock) begin
			S_cam_power_cnt <= 18'd0;
			S_cam_pwdnb <= 1'b0;
			S_cam_i2c_ready <= 1'b0;
		end else begin
			if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
				S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
			if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
				S_cam_pwdnb <= 1'b1;
			if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
				S_cam_i2c_ready <= 1'b1;
		end
	end

	assign O_cam_rst 	= S_cam_pwdnb;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
    // ---- gain value and write request crossing into the 24 MHz configuration domain ----
    // The AGC and the manual gain registers live in the HDMI pixel domain, while ae_set and
    // uicfgcs520 live in the 24 MHz domain. Feeding the value straight across broke setup
    // timing (WNS -2.5 ns into uicfgcs520/wr_data_reg) and a single-cycle pixel-domain
    // request pulse could be missed by the 24 MHz domain altogether, which showed up on
    // hardware as the brightness hunting. So: hold the effective gain with a change toggle
    // in the pixel domain, then synchronise the toggle and capture the value at 24 MHz.
  // ---- boot retry: the first AE/AG write is issued while the sensor is still settling and gets
  // lost (seen on hardware: the picture stayed at the init table's 1x gain while the OSD showed
  // 160). Re-publish the same values about every 113 ms for the first ~2.2 s; the values are
  // identical, so this is harmless.
  reg [22:0] S_rep_cnt;
  reg [4:0]  S_rep_left;
  wire       S_rep_pulse = (S_rep_cnt == 23'd0) && (S_rep_left != 5'd0);
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
      if (!S_hdmi_rst_n) begin S_rep_cnt <= 23'd0; S_rep_left <= 5'd20; end
      else begin
          S_rep_cnt <= (S_rep_cnt == 23'h7FFFFF) ? 23'd0 : (S_rep_cnt + 23'd1);
          if (S_rep_pulse) S_rep_left <= S_rep_left - 5'd1;
      end
  end

    wire [15:0] S_gain_eff = S_gain_manual_f ? S_gain_val_f : S_agc_val;
    wire [15:0] S_ag_24m;
    wire S_control_req_24m;
    wire S_agc_req_24m = S_control_req_24m;

  // ---- exposure (AE): 15-rung ladder (OSD 1..15, mapped 100..1991), 50 Hz anti-flicker pin ----
  // All of its registers live here so that no other block drives them.
  wire [15:0] S_ae_ladder;
  wire [3:0]  S_ae_idx_auto;   // live AE rung from ae_ctrl (auto => idx_r, manual => S_ae_idx)
  // target brightness (TG item): shared by AE, AGC and the arbiter; KEY1/KEY2 adjust it
  reg  [7:0]  S_target_y;
  wire        S_arb_ae_hold;
  wire        S_arb_ag_hold;
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
      if (!S_hdmi_rst_n) S_target_y <= 8'd110;
      else if ((S_adj_idx == 3'd7) && S_up_pulse && (S_target_y < 8'd200)) S_target_y <= S_target_y + 8'd5;
      else if ((S_adj_idx == 3'd7) && S_down_pulse && (S_target_y > 8'd80 )) S_target_y <= S_target_y - 8'd5;
  end
  wire        S_ae_exp_sat;    // exposure saturated flag (rung 15 or flicker-pinned)
  wire [7:0]  S_ae_disp;
  wire        S_ae_req_px;
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
      if (!S_hdmi_rst_n) begin
          S_ae_manual <= 1'b1;        // 2026-10-18: power-up MANUAL (was 1'b0 = auto)
          S_ae_idx <= 4'd12;       // 2026-10-16: power-up at OSD rung 10 (exp 1315)
          // LSC knobs: must be in the SAME always block as the key handlers below
          S_lsc_edge   <= 8'd60;   // E = 0.234 (edge red cut)
          S_lsc_centre <= 8'd50;   // C = 0.195 (centre green cut)
          S_lsc_lift   <= 8'd16;   // b = 0.062 (global blue cut; user 2026-10-23)
          S_flicker     <= 1'b0;        // 50 Hz anti-flicker off by default
          S_ae_load     <= 1'b0;
          S_ae_load_val <= 16'd1920;
          S_flicker_f   <= 1'b0;
          S_ae_ladder_f <= 16'd1920;
          S_ae_disp_f   <= 8'd10;       // power-up rung 10 (OSD now shows rung, not time)
      end else begin
          S_ae_load <= 1'b0;                          // one-cycle handover pulse
          S_flicker_f   <= S_flicker;
          S_ae_ladder_f <= S_ae_ladder;
          S_ae_disp_f   <= S_ae_disp;
          if (S_adj_idx == 3'd6) begin                // exposure item
              if (S_up_pulse && (S_ae_idx < 4'd15))     S_ae_idx <= S_ae_idx + 4'd1;
              else if (S_down_pulse && (S_ae_idx > 4'd1))  S_ae_idx <= S_ae_idx - 4'd1;
              else if (!K3_UART && S_k3_pulse) begin
                  if (S_ae_manual) begin              // manual -> auto: snap ae_ctrl rung to current manual rung
                      S_ae_load     <= 1'b1;
                      // pass the RUNG (not the ladder value): the linear search that used to run here
                      // cost 28 adders and a -19.6ns setup violation
                      S_ae_load_val <= {12'd0, S_ae_idx};
                  end else begin                       // auto -> manual: snap manual rung to live auto rung
                      S_ae_idx <= S_ae_idx_auto;
                  end
                  S_ae_manual <= ~S_ae_manual;
              end
          end else if (1'b0 /* flicker switch removed: item 7 is TG now */ && (S_adj_idx == 3'd7)) begin        // 50 Hz anti-flicker switch
              if (S_up_pulse)      S_flicker <= 1'b1;
              else if (S_down_pulse) S_flicker <= 1'b0;
          end
      end
  end
  // ---- read-back bridge: uicfgcs520 reads 0x3e01 back once (24 MHz domain); carry the value
  // into the pixel domain with the same toggle/sync/capture pattern as the other two bridges.
  // No combinational crossing is allowed (that was the earlier timing failure).
  wire [7:0] S_ae_rd;
  wire       S_ae_rd_valid;
  reg        S_aerd_tog_24m;
  reg  [7:0] S_aerd_24m;
  reg  [2:0] S_aerd_sync;
  reg  [7:0] S_aerd_px;
  always @(posedge S_24m_clk or negedge S_rst_n) begin
      if (!S_rst_n) begin
          S_aerd_tog_24m <= 1'b0;
          S_aerd_24m     <= 8'd0;
      end else if (S_ae_rd_valid) begin
          S_aerd_24m     <= S_ae_rd;
          S_aerd_tog_24m <= ~S_aerd_tog_24m;
      end
  end
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
      if (!S_hdmi_rst_n) begin
          S_aerd_sync <= 3'd0;
          S_aerd_px   <= 8'd0;
      end else begin
          S_aerd_sync <= {S_aerd_sync[1:0], S_aerd_tog_24m};
          if (S_aerd_sync[2] ^ S_aerd_sync[1]) S_aerd_px <= S_aerd_24m;
      end
  end

  // ---- one-hertz time base for the on-screen FPS readout ----------------------
  // S_24m_clk is pll_clk_24m: 42.000 ns period = 23.809524 MHz, PLL-locked to the
  // 50 MHz crystal.  Counting 23,809,524 cycles therefore gives 1.000000 s to
  // within one cycle, so the displayed frame rate is an absolute measurement and
  // does not depend on the pixel clock really being 75 MHz.  Only a TOGGLE crosses
  // into the pixel domain; fps_meter synchronises it with 3 FFs and an XOR edge
  // detect, so this is a single-bit CDC and no multi-bit bus ever crosses.
  localparam [24:0] FPS_SEC_LAST = 25'd23809523;   // 23809524 cycles @ 23.809524 MHz
  reg [24:0] S_fps_sec_cnt;
  reg        S_fps_tick_tog;
  always @(posedge S_24m_clk or negedge S_rst_n) begin
      if (!S_rst_n) begin
          S_fps_sec_cnt  <= 25'd0;
          S_fps_tick_tog <= 1'b0;
      end else if (S_fps_sec_cnt == FPS_SEC_LAST) begin
          S_fps_sec_cnt  <= 25'd0;
          S_fps_tick_tog <= ~S_fps_tick_tog;
      end else begin
          S_fps_sec_cnt <= S_fps_sec_cnt + 25'd1;
      end
  end

  // bring the 24 MHz config-done flag into the pixel domain so ae_ctrl can issue its
  // first exposure write AFTER the sensor is ready to accept it.  Before cfg_done the
  // wrapper routes I_ae_req to the init/ae_set path, so any ladder pulse then is lost.
  reg [1:0] S_ae_cfg_done_sync;
  reg       S_ae_cfg_done_px;
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
      if (!S_hdmi_rst_n) begin S_ae_cfg_done_sync <= 2'd0; S_ae_cfg_done_px <= 1'b0; end
      else begin S_ae_cfg_done_sync <= {S_ae_cfg_done_sync[0], S_ae_cfg_done};
             S_ae_cfg_done_px      <= S_ae_cfg_done_sync[1]; end
  end

  ae_ctrl u_ae_ctrl (
      .clk            ( S_hdmi_pixel_clk     ),
      .rst_n          ( S_hdmi_rst_n         ),
      .I_enable       ( ~S_ae_manual ),
      .I_hold         ( S_arb_ae_hold      ),   // Ë«»·ÖÙ²Ã R3  // AE auto when not in manual; KEY3 on OSD item 6 toggles
      .I_man_idx      ( S_ae_idx             ),
      .I_flicker      ( S_flicker ),     // 50 Hz anti-flicker (toggled on OSD item 7),
      .I_stats_valid  ( S_stats_valid        ),
      .I_y_mean       ( S_y_filt             ),
      .I_target_y     ( S_target_y           ),
      .I_valid_count  ( S_valid_count        ),
      .I_load         ( S_ae_load            ),
      .I_load_val     ( S_ae_load_val        ),
      .I_go           ( S_ae_cfg_done_px     ),
      .O_ae_val       ( S_ae_ladder          ),
      .O_disp         ( S_ae_disp            ),
      .O_req          ( S_ae_req_px          ),
      .O_idx          ( S_ae_idx_auto        ),
      .O_exp_sat      ( S_ae_exp_sat         )
  );
  // One acknowledged, stable packet replaces independent unacknowledged
  // toggles: even consecutive pixel-clock changes cannot cancel each other.
  wire [15:0] S_ae_ladder_24m;
  wire S_ae_ladder_req_24m = S_control_req_24m;
  camera_control_cdc u_control_cdc (
      .I_src_clk(S_hdmi_pixel_clk), .I_dst_clk(S_24m_clk), .I_rst_n(S_rst_n),
      .I_ae(S_ae_ladder), .I_ag(S_gain_eff),
      .I_refresh(S_rep_pulse | S_ae_req_px),
      .O_ae(S_ae_ladder_24m), .O_ag(S_ag_24m), .O_req(S_control_req_24m)
  );

  // ---- temporary debug: how many AE/AG write requests actually went out, and the done flags ----
  // Shown as a 3-digit code on item 7 (DBG): <AE cfg done><CAM cfg done><requests 0..9>
  // ALL 24 MHz inputs are 2-FF synchronised into the pixel domain and ALL arithmetic runs
  // there. Computing (done*100 + done*10 + cnt) in the 24 MHz domain and feeding it
  // combinationally into the OSD (75 MHz) was a cross-domain path: the -20.557 ns setup
  // violation TD reported. No combinational crossing is allowed (same rule as the bridges).
  wire       S_dbg_req = S_agc_req_24m | S_ae_ladder_req_24m | S_ae_req;   // 24 MHz pulses
  reg  [1:0] S_dbg_req_sync;
  reg        S_dbg_req_px, S_dbg_req_prev;
  reg  [7:0] S_dbg_cnt;
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
      if (!S_hdmi_rst_n) begin
          S_dbg_req_sync <= 2'd0; S_dbg_req_px <= 1'b0; S_dbg_req_prev <= 1'b0;
          S_dbg_cnt <= 8'd0;
      end else begin
          S_dbg_req_sync <= {S_dbg_req_sync[0], S_dbg_req};
          S_dbg_req_px   <= S_dbg_req_sync[1];
          S_dbg_req_prev <= S_dbg_req_px;
          // edge-detect: a 24 MHz pulse lasts ~3.1 pixel clocks, count it exactly once
          if (S_dbg_req_px & ~S_dbg_req_prev && (S_dbg_cnt < 8'd9))
              S_dbg_cnt <= S_dbg_cnt + 8'd1;
      end
  end
  reg  [1:0] S_cam_cfg_done_sync;
  reg        S_cam_cfg_done_px;
  always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
      if (!S_hdmi_rst_n) begin S_cam_cfg_done_sync <= 2'd0; S_cam_cfg_done_px <= 1'b0; end
      else begin S_cam_cfg_done_sync <= {S_cam_cfg_done_sync[0], S_cam_cfg_done};
                 S_cam_cfg_done_px   <= S_cam_cfg_done_sync[1]; end
  end
  wire [7:0] S_dbg_val = ({7'd0, S_ae_cfg_done_px} * 8'd100) +
                         ({7'd0, S_cam_cfg_done_px} * 8'd10) + S_dbg_cnt;   // pixel domain only

  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      // No physical exposure keys exist on this board (the former N2/J14
      // "I_button" pins are removed). Drive all four button inputs released so
      // ae_set applies the current defaults AE=1991 / AG=160 once after config.
      // once the sensor reports config done -- something the init register
      // table does NOT do for 0x3e06/0x3e07.
      .I_btn(4'b1111),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag)
  );

  // ============================================================================
  // Experiment B (2026-10-15): align every I2C AE/AG write burst to a CSI frame
  // boundary.  SC520CS latches the EXPOSURE registers (0x3e00/01/02) only at the
  // frame boundary (datasheet: "write after frame start, takes effect N+2
  // frames"); writes that land mid-active-video are silently dropped.  The GAIN
  // registers are real-time and accept writes at any time.  That exact asymmetry
  // is why, on hardware, gain changes take effect but exposure changes do
  // nothing.  Experiment A (removing the 0x3812 group-hold wrapper) did NOT fix
  // it, and the init table contains no AEC-enable register, so the culprit is
  // the WRITE TIMING, not group-hold and not an AEC override.
  //
  // Fix: latch any pending AE/AG request and release it to uicfgcs520 only on a
  // CSI frame-start pulse (synchronized into the 24 MHz domain).  Gain still
  // works because a frame-boundary write is a strict subset of "any time".
  //
  // Knobs (flip to compare against the old behaviour):
  //   EXP_FRAME_SYNC = 1'b1  -> frame-aligned writes (this experiment, default)
  //   EXP_FRAME_SYNC = 1'b0  -> original "write immediately" behaviour
  //   EXP_SYNC_VBLANK = 1'b0 -> trigger at SOF   (rising edge of csi_valid)
  //   EXP_SYNC_VBLANK = 1'b1 -> trigger at VBLANK (falling edge of csi_valid)
  // ============================================================================
  // 2026-10-16: the real exposure bug was the WRITE TIMING vs cfg_done, NOT the
  // frame alignment -- the lab1 runtime write (immediate) works on hardware.  Set
  // EXP_FRAME_SYNC = 0 so Dev writes immediately too, matching the proven lab1 path.
  localparam EXP_FRAME_SYNC  = 1'b0;
  localparam EXP_SYNC_VBLANK = 1'b0;

  // S_csi_valid (active-video level) lives in the MIPI RX clock domain and is
  // held high for the whole active frame, so a 2-FF synchronizer + edge detect
  // gives a clean, reliable one-pulse-per-frame trigger in the 24 MHz domain.
  reg [2:0] S_csi_valid_sync;
  always @(posedge S_24m_clk or negedge S_rst_n) begin
      if (!S_rst_n) S_csi_valid_sync <= 3'd0;
      else          S_csi_valid_sync <= {S_csi_valid_sync[1:0], S_csi_valid};
  end
  wire S_fs_pulse_24m = EXP_SYNC_VBLANK
                        ? (~S_csi_valid_sync[1] &  S_csi_valid_sync[2])   // falling = VBLANK
                        : ( S_csi_valid_sync[1] & ~S_csi_valid_sync[2]);  // rising  = SOF

  // Latch any request (gain or exposure) and only fire it at the next frame edge.
  // A watchdog guarantees liveness: if no frame pulse arrives within ~44 ms
  // (~2^20 cycles @24 MHz) the pending write still fires, so a missing
  // frame-valid signal can never lock the sensor at its init-table values.
  // Use the STABLE latch (not the transient O_ae_cfg_done) so a gain change that
  // arrives while uicfgcs520 is busy writing the AE table is not silently dropped.
    reg S_cfg_done_latch;
  wire S_ae_req_raw = S_cfg_done_latch ? (S_agc_req_24m | S_ae_ladder_req_24m) : S_ae_req;
  reg  S_ae_req_pend;
  reg  [19:0] S_pend_wdog;
  wire S_pend_timeout = (S_pend_wdog == 20'hFFFFF);
  always @(posedge S_24m_clk or negedge S_rst_n) begin
      if (!S_rst_n) begin
          S_ae_req_pend <= 1'b0;
          S_pend_wdog   <= 20'd0;
      end else if (S_ae_req_raw) begin
          S_ae_req_pend <= 1'b1;                                          // latch until frame
          S_pend_wdog   <= 20'd0;
      end else if (S_fs_pulse_24m || S_pend_timeout) begin
          S_ae_req_pend <= 1'b0;                                          // consumed (frame or wdog)
          S_pend_wdog   <= 20'd0;
      end else if (S_ae_req_pend) begin
          S_pend_wdog   <= S_pend_wdog + 20'd1;                           // count while waiting
      end
  end
  wire S_ae_req_to_cfg = EXP_FRAME_SYNC
                         ? ((S_fs_pulse_24m || S_pend_timeout) && S_ae_req_pend)
                         : S_ae_req_raw;

    // 2026-10-16 ROOT-CAUSE FIX for "exposure never changes":
    // uicfgcs520 transiently drops O_ae_cfg_done to 0 while it writes the run-time
    // AE table (see its ST_IDLE: on I_ae_req it sets O_ae_cfg_done<=0, then back to 1
    // when the table finishes).  The old exposure-source mux
    //   .I_ae(S_ae_cfg_done ? S_ae_ladder_24m : S_ae)
    // therefore flipped I_ae back to the ae_set default (1991) for the ENTIRE write,
    // so the ae_ctrl ladder value was never actually sent to the sensor -- exposure
    // stayed pinned at 1991 no matter what the keys/ladder did.  Lab1 has no such mux
    // (I_ae = S_ae direct), which is exactly why lab1's run-time exposure writes work
    // and Dev's did not.
    //
    // Fix: latch a STABLE "config done" (set on the FIRST completion, never cleared)
    // and use it to select the exposure source.  After init the ladder value is always
    // what gets written; the transient O_ae_cfg_done drop can no longer corrupt it.
    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if (!S_rst_n)           S_cfg_done_latch <= 1'b0;
        else if (S_ae_cfg_done) S_cfg_done_latch <= 1'b1;   // set once at init, stays high
    end

    // SC520CSåŽŸåŽ‚çœŸå®žHDæ¨¡å¼ï¼š1280x720@60ã€RAW10ã€2-Laneã€828 Mbps/Laneã€‚
    // å½“å‰å¯„å­˜å™¨è¡¨å·²å…³é—­Slave/FSINç­‰å¾…ï¼ŒSensorä»¥free-runæ–¹å¼è¿žç»­è¾“å‡ºã€‚
    uicfgcs520 #(
      .CLK_DIV(23809524 / 100000 - 1)
  ) u_uicfgcs520 (
      .I_clk(S_24m_clk),  //ç³»ç»Ÿæ—¶é’Ÿè¾“å…¥
      .I_rst_n(S_cam_i2c_ready), // SC520CS power-up delay completed
      // ae_set publishes the power-up default; after AE config completes the ladder takes over.
      // Use the STABLE latch (not the transient O_ae_cfg_done) to pick the exposure source so the
      // run-time AE-table write cannot flip I_ae back to the ae_set default.
      .I_ae_req(S_ae_req_to_cfg),
            .I_ae(S_cfg_done_latch ? S_ae_ladder_24m : S_ae),
      .I_ag(S_ag_24m),
      .O_cam_scl(O_cam_scl),  //I2Cæ€»çº¿ï¼ŒSCLæ—¶é’Ÿ
      .IO_cam_sda(IO_cam_sda),  //I2Cæ€»çº¿ï¼ŒSDAæ•°æ®
      .O_cfg_done(S_cam_cfg_done),  //æ‘„åƒå¤´å¯„å­˜å™¨åˆå§‹åŒ–å®Œæˆ
      .O_ae_cfg_done(S_ae_cfg_done),  //AEé…ç½®å®Œæˆ
      .O_ae_rd(S_ae_rd),              // 0x3e01 read back once after init
      .O_ae_rd_valid(S_ae_rd_valid),
      .O_sensor_id(S_cam_sensor_id),
      .O_sensor_id_ok(S_cam_sensor_id_ok),
      .O_iic_error(S_cam_i2c_error)
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "3dB"   ),// 828 Mbps/LaneæŽ¥æ”¶å‡è¡¡
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .clk(S_csi_rx_clk)
 //);

 //csi è§£ç ä¸ºRAWæ•°æ®
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//è§£ç ä¸ºRAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//å°†æ•°æ®è½¬ä¸ºstreamæµ
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// The sensor already produces a real 1280x720 frame.  Do not pass it through
// the legacy fixed 1024x600 image_correction crop; keep line/frame markers
// aligned with the factory sensor output.
// ---- vendor BLC in the RAW domain (2026-10-23) --------------------------------
// 4-phase black level subtraction, 40bit = 4 x 10bit per beat, AXI-Stream pass-through.
// Defaults are the vendor's own (15). Change BLC_R0..R3 to tune.
localparam [9:0] BLC_R0 = 10'd15;
localparam [9:0] BLC_R1 = 10'd15;
localparam [9:0] BLC_R2 = 10'd15;
localparam [9:0] BLC_R3 = 10'd15;

BLC #(
    .Black_level_offset_r0 ( BLC_R0 ),
    .Black_level_offset_r1 ( BLC_R1 ),
    .Black_level_offset_r2 ( BLC_R2 ),
    .Black_level_offset_r3 ( BLC_R3 )
) u_blc_raw (
    .I_clk   ( S_csi_rx_clk  ),
    .I_rst_n ( S_rst_n       ),
    .I_tlast ( S_axis_tlast  ),
    .I_tuser ( S_axis_tuser  ),
    .I_tdata ( S_axis_tdata  ),
    .I_tvalid( S_axis_tvalid ),
    .I_tdest ( 10'd0         ),
    .I_tready(               ),
    .O_tlast ( S_raw_tlast   ),
    .O_tuser ( S_raw_tuser   ),
    .O_tdata ( S_raw_tdata   ),
    .O_tvalid( S_raw_tvalid  ),
    .O_tdest (               ),
    .O_tready( 1'b1          )
);


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .clk(S_csi_rx_clk)
//  );


//ISPç®—æ³•é¡¶å±‚æ¨¡å—
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.I_edge_en       (1'b0),
.O_wb_temp       ( S_isp_wb_temp ),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        ()
  );


    // ===== L03: latency visualisation -- shared time base and descriptors =====
    // Tick generator runs on I_sys_clk (sys_clk_50m, 20 ns): 5000 cycles = 0.1 ms.
    // The 32-bit Gray snapshot is distributed to two consumer domains:
    //   S_csi_rx_clk     -> stamps T_start at SOF
    //   S_hdmi_pixel_clk -> stamps T_end at the first HDMI output pixel
    // The DDR domain only carries the descriptor and never compares time, so it
    // needs no lat_time_rx instance (saves 96 flops).
    // Session rebuild: power key only (no button), so I_rebuild is tied to 0.
     wire [31:0] S_lat_tick_gray;
     wire [15:0] S_lat_session;
     wire        S_lat_session_tog;
     wire [31:0] S_lat_tick_csi, S_lat_tick_pix;
     wire [15:0] S_lat_sess_csi, S_lat_sess_pix;
     wire        S_lat_ready_csi, S_lat_ready_pix;
     wire [31:0] S_lat_pub_tick,  S_lat_disp_tick;
     wire [15:0] S_lat_pub_sess,  S_lat_disp_sess;
     wire        S_lat_pub_valid, S_lat_disp_valid;
     wire [2:0]  S_lat_state;
     wire [15:0] S_lat_bcd;
     wire [31:0] S_lat_delta;
     wire        S_lat_snap_tog;

     lat_tick_gen #(.CLKS_PER_TICK(5000)) u_lat_tick_gen (
         .I_clk         ( I_sys_clk         ),
         .I_rst_n       ( S_rst_n           ),
         .I_rebuild     ( 1'b0              ),
         .O_tick_bin    (                   ),
         .O_tick_gray   ( S_lat_tick_gray   ),
         .O_session     ( S_lat_session     ),
         .O_session_tog ( S_lat_session_tog )
     );

     lat_time_rx #(.SYNC_STAGES(3)) u_lat_rx_csi (
         .I_clk         ( S_csi_rx_clk      ),
         .I_rst_n       ( S_rst_n           ),
         .I_tick_gray   ( S_lat_tick_gray   ),
         .I_session     ( S_lat_session     ),
         .I_session_tog ( S_lat_session_tog ),
         .O_tick_bin    ( S_lat_tick_csi    ),
         .O_session     ( S_lat_sess_csi    ),
         .O_ready       ( S_lat_ready_csi   )
     );

     lat_time_rx #(.SYNC_STAGES(3)) u_lat_rx_pix (
         .I_clk         ( S_hdmi_pixel_clk  ),
         .I_rst_n       ( S_hdmi_rst_n      ),
         .I_tick_gray   ( S_lat_tick_gray   ),
         .I_session     ( S_lat_session     ),
         .I_session_tog ( S_lat_session_tog ),
         .O_tick_bin    ( S_lat_tick_pix    ),
         .O_session     ( S_lat_sess_pix    ),
         .O_ready       ( S_lat_ready_pix   )
     );
    // ===========================================================================

    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( S_lane_error_any			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),
        .I_display_slot       ( S_display_slot        ),
        // L03: synced time base of this domain (S_csi_rx_clk)
        .I_tick_now           ( S_lat_tick_csi        ),
        .I_time_session       ( S_lat_sess_csi        ),
        .I_time_ready         ( S_lat_ready_csi       ),
        .O_frame_available    ( S_frame_available     ),
        .O_source_frame_id    ( S_published_source_id ),
        .O_fifo_overflow      ( S_write_overflow      ),
        // L03: descriptor updated on the same publish edge as source_frame_id
        .O_lat_start_tick     ( S_lat_pub_tick        ),
        .O_lat_session        ( S_lat_pub_sess        ),
        .O_lat_valid          ( S_lat_pub_valid       ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out #(.PAYLOAD_BUFFER_STAGES(2)) u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
        .I_frame_available   ( S_frame_available    ),
        .I_source_frame_id   ( S_published_source_id ),
        .I_source_rst_n      ( S_rst_n              ),
        // L03: published descriptor -> frozen on slot select
        .I_lat_start_tick    ( S_lat_pub_tick       ),
        .I_lat_session       ( S_lat_pub_sess       ),
        .I_lat_valid         ( S_lat_pub_valid      ),
        .O_display_slot      ( S_display_slot       ),
        .O_fifo_underflow    ( S_read_underflow     ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      ),
        .O_pixel_good        ( S_video_pixel_good   ),
        .O_source_frame_id   ( S_display_source_id  ),
        .O_source_valid      ( S_display_source_valid )
        ,
        // L03: pixel-domain descriptor, same frame as display source identity
        .O_lat_start_tick    ( S_lat_disp_tick      ),
        .O_lat_session       ( S_lat_disp_sess      ),
        .O_lat_valid         ( S_lat_disp_valid     )
    );



    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    hdmi_mixer #(
        .SHAPE_BUILD(1),
        .COLOUR_DIAG_ENABLE(0),
        .SHARE_GREEN_FIELD(1),
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  ),
        .DEBUG_MODE ( 0    )
    )u_hdmi_mixer(
        .I_r_min     ( S_r_min_f      ),
        .I_r_dom     ( S_r_dom_f      ),
        .I_g_min     ( S_g_min_f      ),
        .I_g_dom     ( S_g_dom_f      ),
        .I_b_min     ( S_b_min_f      ),
        .I_b_dom     ( S_b_dom_f      ),

        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_hdmi_vsync       ),
        .I_video_hsync   ( S_hdmi_hsync       ),
        .I_video_de      ( S_hdmi_de          ),
        .I_video_user    ( S_hdmi_user        ),
        .I_video_last    ( S_hdmi_last        ),
        .I_tick_1hz_tog  ( S_fps_tick_tog     ),
        .I_debug_status  ( S_hdmi_debug_status),
        .I_osd_en        ( S_osd_frame     ),
        .I_invert_en     ( S_invert_en       ),
        .I_emboss_en     ( S_emboss_en       ),
        .I_sat_level     ( S_sat_frame       ),
        .I_bright_level  ( S_bright_frame    ),
        .I_mode          ( S_mode            ),
        .I_adj_idx       ( S_adj_idx        ),
        .I_lsc_edge        ( S_lsc_edge         ),
        .I_lsc_centre      ( S_lsc_centre       ),
        .I_lsc_lift        ( S_lsc_lift         ),
        .I_shape_threshold(S_shape_threshold_f), .I_paper_threshold(S_paper_threshold_f),
        .I_shape_profile(S_shape_profile_f), .I_shape_view(S_shape_view_f),
        .I_area_min(S_area_min_f),.I_area_max(S_area_max_f),
        .I_show_temp    ( (S_adj_idx==3'd1) || (S_adj_idx==3'd2) || (S_adj_idx==3'd6) || (S_adj_idx==3'd7) ),
        .I_wb_temp      ( (S_adj_idx==3'd6) ? {9'd0, S_ae_idx_auto} :  // live rung (auto idx_r / manual S_ae_idx)
                          (S_adj_idx==3'd7) ? {5'd0, S_target_y} :
                          (S_adj_idx==3'd2) ? {5'd0, S_gain_val_f[7:0]} :
                           (S_wb_manual_f ? S_wb_trim_abs : 13'd0) ),
        .I_wb_is_trim   ( (S_adj_idx==3'd2) ? S_gain_manual_f :
                          (S_adj_idx==3'd6) ? S_ae_manual_f :   // same polarity as AG: 1 = manual (show rung), 0 = AUTO
                          (S_adj_idx==3'd7) ? 1'b1 : S_wb_manual_f ),
        .I_wb_trim_neg   ( S_wb_trim_neg                    ),
        .I_gain_level    ( S_gain_val_f[7:0]                ),
        .I_wb_trim       ( S_wb_trim_f                      ),
        .I_wb_trim_en    ( S_wb_manual_f                    ),
        .O_video_rd_en   ( S_hdmi_window_rd_en),
        .O_alarm_led     ( O_alarm_led       ),
        .O_alarm_buzzer_n ( O_alarm_buzzer_n ),
        .I_video_rd_data ( S_video_rd_data    ),
        .I_video_pixel_good(S_video_pixel_good),
        .I_source_frame_id(S_display_source_id),
        .I_source_valid(S_display_source_valid),
        .I_source_rst_n(S_rst_n),
        // L03: shared time base, local session and measurement descriptor
        .I_tick_now       ( S_lat_tick_pix    ),
        .I_session_now    ( S_lat_sess_pix    ),
        .I_time_ready     ( S_lat_ready_pix   ),
        .I_lat_start_tick ( S_lat_disp_tick   ),
        .I_lat_session    ( S_lat_disp_sess   ),
        .I_lat_valid      ( S_lat_disp_valid  ),
        .O_lat_state      ( S_lat_state       ),
        .O_lat_bcd        ( S_lat_bcd         ),
        .O_lat_delta      ( S_lat_delta       ),
        .O_lat_snap_tog   ( S_lat_snap_tog    ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_colour_diag_valid(),.O_colour_diag_data(),.I_colour_diag_ready(1'b0),
        .O_probe_rgb     ( S_probe_rgb        ),
        .O_probe_de      ( S_probe_de         ),
        .O_probe_vsync   ( S_probe_vsync      ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    ),
        .O_y_mean        ( S_y_mean           ),
        .O_y_filt        ( S_y_filt           ),
        .O_sat_count     ( S_sat_count        ),
        .O_valid_count   ( S_valid_count      ),
        .O_stats_valid   ( S_stats_valid      ),
        .O_detect_now    ( S_detect_now       ),
        .O_target_now    ( S_target_now       ),
        .O_target_area   ( S_target_area      ),
        .O_shape_class   ( S_shape_class       ),
        .O_shape_valid   ( S_shape_valid       ),
        .O_shape_count   ( S_shape_count       )
    );

    // Automatic gain control. Runs in the HDMI pixel domain, i.e. the same domain as
    // the statistics and as the manual gain registers. Disabled while the gain item is
    // in manual mode; the write request is OR-ed into the existing I2C handshake.
    agc_ctrl u_agc_ctrl (
        .clk            ( S_hdmi_pixel_clk   ),
        .rst_n          ( S_hdmi_rst_n       ),
        .I_enable       ( ~S_gain_manual_f   ),
        .I_coord        ( (~S_ae_manual_f) && (~S_gain_manual_f) ),  // both auto => coordinate (exposure-first, gain fills)
        .I_stats_valid  ( S_stats_valid      ),
        .I_y_mean       ( S_y_filt           ),
        .I_target_y     ( S_target_y         ),   // the loop consumes the IIR-filtered mean
        .I_sat_count    ( S_sat_count        ),
        .I_valid_count  ( S_valid_count      ),
        .I_load         ( S_agc_load         ),
        .I_load_val     ( S_agc_load_val     ),
        .I_exp_sat      ( S_ae_exp_sat       ),
        .I_hold         ( S_arb_ag_hold      ),   // Ë«»·ÖÙ²Ã R1/R2
        .O_gain_val     ( S_agc_val          ),
        .O_req          ( S_agc_req          )
    );

  // ---- Ë«»·ÓÅÏÈ¼¶ÖÙ²Ã£¨¾«¼ò°æ£ºR1/R2/R3£¬ÎÞÖØÐÄÇ¨ÒÆ£©¡£µ¥»·Ëã·¨Î´¸Ä£¬Ö»½Ó¹ÜÁ½¸ö I_hold ----
  dual_loop_arbiter #(
      .DEADBAND     (8'd30),
      .GAIN_FLOOR   (16'd96)
  ) u_dual_arb (
      .I_clk         ( S_hdmi_pixel_clk                       ),
      .I_rst_n       ( S_hdmi_rst_n                           ),
      .I_auto_both   ( (~S_ae_manual_f) && (~S_gain_manual_f) ),
      .I_target_y    ( S_target_y                             ),
      .I_mean        ( S_y_filt                               ),
      .I_gain        ( S_agc_val                              ),
      .I_exp_sat     ( S_ae_exp_sat                           ),
      .O_ae_hold     ( S_arb_ae_hold                          ),
      .O_ag_hold     ( S_arb_ag_hold                          )
  );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync  ),
        .I_video_in_de      ( S_hdmi_out_de     ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data   ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
    // ==================== PH1P35 Ó²ºË MCU (per guide section 4/6) ====================
    // PLL: 50MHz(C4) -> clk0=300MHz core / clk1=100MHz timer / clk2=50MHz (Ô¤Áô APB)
    wire S_mcu_clk300, S_mcu_clk100, S_mcu_clk50, S_mcu_pll_lock;

    wire [19:0] S_apb_paddr;
    wire        S_apb_psel, S_apb_penable, S_apb_pwrite;
    wire [31:0] S_apb_pwdata;
    wire [3:0]  S_apb_pstrobe;
    wire [2:0]  S_apb_pprot;
    wire [31:0] S_apb_prdata;
    wire        S_apb_pready, S_apb_pslverr;
    // No APB fabric slave is instantiated; never leave MCU responses floating.
    assign S_apb_prdata = 32'd0;
    assign S_apb_pready = 1'b1;
    assign S_apb_pslverr = 1'b0;

    // ---- AHB Ì½²â£¨µ÷ÊÔÓÃ£©----
    wire        S_ahb_clk;
    wire [1:0]  S_ahb_htrans;
    wire        S_ahb_hwrite;
    wire [30:0] S_ahb_haddr;
    wire [2:0]  S_ahb_hsize;
    wire [2:0]  S_ahb_hburst;
    wire [3:0]  S_ahb_hprot;
    wire        S_ahb_hmastlock;
    wire [31:0] S_ahb_hwdata;
    wire [31:0] S_ahb_hrdata;
    wire [1:0]  S_ahb_hresp;
    wire        S_ahb_hready;

    MCU_PLL u_mcu_pll (
        .refclk  ( I_sys_clk      ),
        .reset   ( ~S_rst_n       ),
        .clk0_out( S_mcu_clk300   ),
        .clk1_out( S_mcu_clk100   ),
        .clk2_out( S_mcu_clk50    ),
        .lock    ( S_mcu_pll_lock )
    );

    // MCU ¸´Î»£ºÏµÍ³¸´Î»ÊÍ·Å + PLL Ëø¶¨ºó²Å·Å¿ª
    wire S_mcu_rst = ~( S_rst_n & S_mcu_pll_lock );

    RISCV u_mcu (
        .core_clk   ( S_mcu_clk300 ),
        .timer_clk  ( S_mcu_clk100 ),
        .core_reset ( S_mcu_rst    ),
        .por_reset  ( S_mcu_rst    ),
        .nmi        ( 1'b0         ),
        .clic_irq   ( 23'd0        ),

        .uart0_tx   ( O_mcu_uart0_tx ),
        .uart0_rx   ( I_mcu_uart0_rx ),
        .uart1_tx   ( O_mcu_uart1_tx ),
        .uart1_rx   ( I_mcu_uart1_rx ),

        // APB Ö÷¿Ú£ºÔÝÎÞ´Ó»ú£¬´Ó»úÓ¦´ð°´ÊÖ²á tie-off
        .apb_clk    ( S_mcu_clk100 ),
        .apb_rst    ( S_mcu_rst    ),
        .paddr      ( S_apb_paddr   ),
        .psel       ( S_apb_psel    ),
        .penable    ( S_apb_penable ),
        .pwrite     ( S_apb_pwrite  ),
        .pwdata     ( S_apb_pwdata  ),
        .pstrobe    ( S_apb_pstrobe ),
        .pprot      ( S_apb_pprot   ),
        .prdata     ( S_apb_prdata  ),
        .pready     ( S_apb_pready  ),
        .pslverr    ( S_apb_pslverr ),

        // AHB Ö÷¿Ú£ºÔÝÎÞ´Ó»ú£¬´Ó»úÓ¦´ð°´ÊÖ²á tie-off
        .ahb_clk    ( S_ahb_clk     ),
        .htrans     ( S_ahb_htrans  ),
        .hwrite     ( S_ahb_hwrite  ),
        .haddr      ( S_ahb_haddr   ),
        .hsize      ( S_ahb_hsize   ),
        .hburst     ( S_ahb_hburst  ),
        .hprot      ( S_ahb_hprot   ),
        .hmastlock  ( S_ahb_hmastlock ),
        .hwdata     ( S_ahb_hwdata  ),
        .hrdata     ( S_ahb_hrdata  ),
        .hresp      ( S_ahb_hresp   ),
        .hready     ( S_ahb_hready  ),

        .core_sysrst (             )
    );

    // ==================== AHB Ì½²â´Ó»ú£¨µ÷ÊÔÓÃ£©====================
    // ==================== MCU AHB ´Ó»ú£¨fabric ¼Ä´æÆ÷×é£©====================
    mcu_ahb_slave #(
        .BASE_SEL ( 19'h80000 ),     // 0x80000000£¨¼«¼ò¹¤³ÌÊµ²â´°¿Ú£©
        .BASE_SEL2( 19'h90000 )      // 0x90000000£¨±¸ÓÃ£©
    ) u_mcu_ahb_slave (
        .I_hclk      ( S_ahb_clk       ),
        .I_return_clk( S_mcu_clk300    ),
        .I_hresetn   ( ~S_mcu_rst      ),

        .I_haddr     ( S_ahb_haddr     ),
        .I_htrans    ( S_ahb_htrans    ),
        .I_hwrite    ( S_ahb_hwrite    ),
        .I_hsize     ( S_ahb_hsize     ),
        .I_hburst    ( S_ahb_hburst    ),
        .I_hprot     ( S_ahb_hprot     ),
        .I_hmastlock ( S_ahb_hmastlock ),
        .I_hwdata    ( S_ahb_hwdata    ),
        .O_hrdata    ( S_ahb_hrdata    ),
        .O_hresp     ( S_ahb_hresp     ),
        .O_hready    ( S_ahb_hready    ),

        // Ö±½ÓÎ¹ÏñËØÓòÔ­Ê¼Öµ£¬Ä£¿éÄÚ²¿ÓÃ hclk ×ö 2FF Í¬²½
        .I_sw_code   ( S_code          ),
        .I_mode_now  ( S_mode          ),
        .I_det_active( S_detect_now    ),
        .I_has_target( S_target_now    ),
        .I_target_area( S_target_area   ),
        .I_shape_class( S_shape_class   ),
        .I_shape_valid( S_shape_valid   ),
        .I_shape_count( S_shape_count   ),
        .I_frame_cnt ( 16'd0           ),

        .O_ovr_mode  ( S_ovr_mode     ),
        .O_ovr_en    ( S_ovr_en       ),
        .O_ovr_tog   ( S_ovr_tog      ),
        .O_cmd_op    ( S_cmd_op       ),
        .O_cmd_arg   ( S_cmd_arg      ),
        .O_cmd_seq   ( S_cmd_seq      ),
        .I_ovr_active( S_ovr_active   ),
        .O_th_min    (                 ),   // ½×¶Î 4 ½ÓãÐÖµ
        .O_th_dom    (                 )
    );

endmodule
