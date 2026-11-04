// P04: legacy UI preserved; RGB detection annotations share its six stages.
module red_object_osd_pipeline #(
    parameter integer IMG_WIDTH=1280,
    parameter integer IMG_HEIGHT=720,
    parameter integer DEBUG_MODE=0,SHAPE_DIAGNOSTICS=0,
    parameter integer BORDER_WIDTH=2,
    parameter integer STATUS_ENABLE=1,STATUS_X=16,STATUS_Y=160,
    parameter integer COORD_X=STATUS_X,COORD_Y=STATUS_Y+32,
    parameter integer ZONE_GUARD=0,GUARD_DIAGNOSTICS=0,GUARD_X0=400,GUARD_X1=880,GUARD_Y0=220,GUARD_Y1=500
)(
    input wire I_clk, I_rst_n,
    input wire I_de, I_vsync, I_hsync,
    input wire [23:0] I_rgb,
    input wire [11:0] I_x, I_y,
    input wire I_osd_en,
    input wire [3:0] I_mode,
    input wire [3:0] I_adj_idx,
    input wire [7:0] I_shape_value,
    input wire [1:0] I_fill_color,
    input wire I_show_temp,          // 1 = show the 4-digit colour-temperature readout
    input wire [12:0] I_wb_temp,     // value shown: K estimate, or |trim| in manual mode
    input wire        I_wb_is_trim,   // 1 = the item is in manual trim mode
    input wire        I_wb_trim_neg,  // 1 = the trim is negative (show a minus)
    input wire [7:0]  I_gain_level,   // composite gain control value (16..176)
    input wire [7:0] I_th_min,I_th_dom,
    input wire [4:0] I_sat_level,
    input wire [8:0] I_bright_level,
    input wire [23:0] I_counter_bcd,
    input wire [11:0] I_fps_bcd,        // fps*10 as three BCD digits, for the FPS line
    // L04: latency readout -----------------------------------------------------
    input wire [15:0] I_lat_bcd,       // LAT value, 4 BCD digits {hundreds,tens,units,tenths}
    input wire [2:0]  I_lat_state,     // 0 WAIT, 1 OK, 2 OVR, 3 STALE, 4 INVALID
    input wire [3:0] I_debug_status,
    input wire I_det_enable,I_det_target_valid,
    input wire [2:0] I_det_status,
    input wire [11:0] I_det_xmin,I_det_xmax,I_det_ymin,I_det_ymax,
    output reg O_de, O_vsync, O_hsync,
    output reg [23:0] O_rgb,
    input wire I_coords_valid,
    input wire [15:0] I_center_x_bcd,I_center_y_bcd,
    input wire [3:0] I_result_mode,
    input wire [1:0] I_shape_class,
    input wire [19:0] I_shape_diag,input wire [1:0] I_shape_diag_view,
    input wire I_guard_enable,I_guard_sof,I_guard_alarm,I_guard_center_valid,
    input wire [2:0] I_guard_state,
    input wire [3:0] I_guard_reason,
    input wire [11:0] I_guard_center_x,I_guard_center_y
);
    // Purpose: the existing fixed 1280-wide OSD layout, primarily 720p.
    // S0 geometry/config -> S1 character -> S2 scale -> S3 font row ->
    // S4 bit selection -> S5 overlay. One pixel each clock, no backpressure.
    localparam [11:0] ADJ_Y=IMG_HEIGHT-12'd32;
    localparam [23:0] TEXT_COLOR=24'hfff200;      // left-side STATUS/COORD text (unchanged)
    localparam [23:0] TEXT_COLOR_TOP=24'hff0000;  // 2026-11-01 CNT/FPS/LAT words, ':' and values: red
    localparam [23:0] TEXT_COLOR_BOT=24'h0000ff;  // 2026-11-01 bottom-right ADJ row: blue
    // 2026-11-01: the mode label on the CNT row takes the colour of the colour it
    // detects, so 0110/0101 read as red, 1000/0111 as blue, 1010/1001 as green.
    localparam [23:0] LABEL_RED  =24'hff0000;
    localparam [23:0] LABEL_GREEN=24'h00ff00;
    localparam [23:0] LABEL_BLUE =24'h0000ff;
    localparam [23:0] LABEL_AMBER=24'hffb000;  // 非彩色模式的标签仍是橙黄，与数值的红色区分开
    localparam [11:0] TOP_BAND_Y=12'd88;          // any OSD pixel above this is 'top-right'
    localparam [1:0] NATIVE=0, CHINESE=1, COLON=2, DIGIT=3;

    reg [11:0] tx0,ty0,ax0,ay0,lx0,ly0;
    reg [11:0] label_x0;   // 类型标签专用坐标：原点比 TXT_X 再左 8px
    reg text_valid0,adj_valid0;
    reg [3:0] mode0;
    reg [3:0] adj_idx0;
    reg [1:0] fill_color0;
    reg show_temp0;
    reg [7:0] gain_level0;
    reg is_trim0, trim_neg0;
    reg [7:0] wb_val0;
    reg [3:0] wd3,wd2,wd1,wd0;
    reg [12:0] wrem;
    reg adj_negative0;
    reg [8:0] adj_magnitude0;
    reg [23:0] counter0;
    // FPS line: the row directly under CNT.  It rides the existing glyph path
    // (S1 select -> S2 font -> S3 row -> S4 hit -> S5 overlay), so it adds no
    // pipeline stage and RGB/DE/HS/VS stay aligned at 17 clocks.
    reg [11:0] fps0;          // fps*10 BCD, latched with the row
    reg [11:0] fy0;           // row-local Y for the FPS row
reg [11:0] tx2;           // row-local X for the FPS row (tx0 belongs to the CNT row)
reg [11:0] lx2;           // row-local label X for the FPS row (label_x0 belongs to CNT)
    reg        text2_valid0;  // FPS row geometry
    reg  [4:0] fps_dot_q;     // decimal point flag, aligned with rgb_q[4]
    // L04: LAT row. lat_bcd0 and lat_state0 are sampled in the SAME clock, so the four
    // digits and the state word always belong to one measurement result.
    reg [15:0] lat_bcd0;          // LAT value, latched with the row
    reg [2:0]  lat_state0;        // LAT state, latched together with lat_bcd0
    reg [11:0] laty0;             // row-local Y for the LAT row (y - 56)
    reg [11:0] ltx;               // row-local X for the LAT row (x - 1070)
    reg        text3_valid0;      // LAT row geometry
    reg  [4:0] lat_dot_q;         // LAT decimal point flag, aligned with rgb_q[4]
    reg [15:0] lat_row_bcd;
    reg [2:0] lat_row_state;
    reg lat_row_locked;
    // Freeze the entire 24-scanline readout at its first active pixel. A late
    // fault is still recorded by calc, but cannot tear already-emitted text.
    wire [15:0] lat_pixel_bcd = lat_row_locked ? lat_row_bcd : I_lat_bcd;
    wire [2:0] lat_pixel_state = lat_row_locked ? lat_row_state : I_lat_state;

    reg [7:0] char1;
    reg [4:0] gx1,gy1;
    reg [1:0] kind1,adj_digit_index1;
    reg valid1,adj_digit1;
    reg [3:0] hundreds1;
    reg [6:0] remainder1;

    reg [7:0] char2;
    reg [4:0] row2,col2;
    reg valid2;
    wire [23:0] row_bits3;
    reg [4:0] col3;
    reg valid3,hit4;

    reg [23:0] rgb_q[0:4];
    reg [2:0] ctl_q[0:4];
    reg [11:0] x_q[0:4];
    reg [4:0] top_band_q,bot_band_q;   // y band, pipelined with x_q
    reg [4:0] lab_sel_q;   // 2026-11-01: mode-label x window, pipelined with x_q
    reg [9:0] lab_col_q;   // 2026-11-01: 2-bit mode colour per pixel, pipelined
    reg [4:0] guard_row_q; // 2026-11-01: guard-text row flag, pipelined with x_q
    reg [9:0] guard_col_q; // 2026-11-01: 2-bit guard-text colour, pipelined
    reg [4:0] osd_q,logo_valid_q;
    reg [11:0] debug_q;
    reg [14:0] logo_addr_q;
    wire [24:0] logo_pixel;
    // The three-stage Logo ROM replaces these former two delay registers.
    integer k;
    // Parallel six-stage overlay bits. RGB/DE/HS/VS retain their old latency.
    wire guard_hit;
    wire [2:0] guard_color;
    reg [4:0] guard_hit_q;
    reg [2:0] guard_color_q[0:4];
    integer guard_stage;
    generate if(ZONE_GUARD==1)begin: guard_enabled
        zone_guard_osd #(.WIDTH(IMG_WIDTH),.HEIGHT(IMG_HEIGHT),
            .X0(GUARD_X0),.X1(GUARD_X1),.Y0(GUARD_Y0),.Y1(GUARD_Y1),.DIAGNOSTICS(GUARD_DIAGNOSTICS),.TEXT_EN(0)) observer(
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_guard_enable),.I_osd_en(I_osd_en),
            .I_de(I_de),.I_sof(I_guard_sof),.I_x(I_x),.I_y(I_y),.I_state(I_guard_state),
            .I_alarm(I_guard_alarm),.I_center_valid(I_guard_center_valid),
            .I_reason(I_guard_reason),
            .I_center_x(I_guard_center_x),.I_center_y(I_guard_center_y),
            .O_hit(guard_hit),.O_color(guard_color));
    end else begin: guard_disabled
        assign guard_hit=1'b0;assign guard_color=3'd0;
    end endgenerate
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin
            guard_hit_q<=0;
            for(guard_stage=0;guard_stage<5;guard_stage=guard_stage+1)guard_color_q[guard_stage]<=0;
        end else begin
            guard_hit_q<={guard_hit_q[3:0],guard_hit};guard_color_q[0]<=guard_color;
            for(guard_stage=1;guard_stage<5;guard_stage=guard_stage+1)
                guard_color_q[guard_stage]<=guard_color_q[guard_stage-1];
        end
    end
    reg [23:0] guard_rgb;
    always @* case(guard_color_q[4])
        0:guard_rgb=24'h00ffff;
        1:guard_rgb=24'hffff00;
        2:guard_rgb=24'hff0000;
        3:guard_rgb=24'hff00ff;
        default:guard_rgb=24'hffffff;
    endcase

    reg [7:0] next_char;
    reg [4:0] next_x,next_y;
    reg [11:0] lab_dx,lab_start;   // 2026-11-01 模式标签：去起点后的位移
    reg [2:0]  lab_cell;
    reg [1:0] next_kind,next_adj_index;
    reg next_valid,next_adj_digit;
    reg [3:0] adj_dx;
    wire [7:0] decimal_pair=F_decimal_pair(remainder1);

    // Detection-only additions. Never reuse the legacy glyph slot or ROM port.
    localparam DET_PARAM_OK=(IMG_WIDTH>=1 && IMG_WIDTH<=1280 && IMG_HEIGHT>=1 && IMG_HEIGHT<=720 &&
        BORDER_WIDTH>=1 && BORDER_WIDTH<=16 && (STATUS_ENABLE==0 || STATUS_ENABLE==1) &&
        STATUS_X>=0 && STATUS_X<=4095 && STATUS_Y>=0 && STATUS_Y<=4095 &&
        COORD_X>=0 && COORD_X<=4095 && COORD_Y>=0 && COORD_Y<=4095);
    localparam [23:0] BOX_COLOR=24'h00ff00;
    wire det_ordered=I_det_xmin<=I_det_xmax && I_det_ymin<=I_det_ymax &&
        I_det_xmin<IMG_WIDTH && I_det_ymin<IMG_HEIGHT;
    wire det_success=(I_det_status==3 && I_det_target_valid && I_coords_valid && det_ordered);
    // The delayed result's frame-fixed mode, NOT live switches, owns annotations.
    wire det_pixel=DET_PARAM_OK && I_de && I_osd_en && I_det_enable &&
        (I_result_mode==6 || I_result_mode==8 || I_result_mode==10 || I_result_mode==11 || I_result_mode==12 || I_result_mode==13) &&
        I_x<IMG_WIDTH && I_y<IMG_HEIGHT;
    wire shape_pixel=I_result_mode==12;
    wire [11:0] shape_text_x=I_x-STATUS_X;
    // Register late detection metadata separately from pixel geometry. Select
    // its glyph in S1, not on the source-ID -> S0 geometry path. No new stage.
    reg [2:0] det_status0;
    reg [1:0] shape_class0;
    reg [3:0] status_slot0;        // 2026-11-01: 4 bit，供警戒行 12 个槽使用
    reg guard_row0;                // 2026-11-01: 当前像素是否落在警戒文本行
    reg [1:0] guard_col0;          // 2026-11-01: 警戒文本颜色（0 青 1 黄 2 红 3 品红）
    reg shape_pixel0,det_success0;
    reg [63:0] shape_text;
    always @(*)begin
        if(det_status0==1 || det_status0==0)shape_text="WAIT    ";
        else if(det_status0==2)shape_text="NONE    ";
        else if(!det_success0)shape_text="ERROR   ";
        else case(shape_class0)
            1:shape_text="CIRCLE  ";
            2:shape_text="TRI     ";
            3:shape_text="RECT    ";
            default:shape_text="UNKNOWN ";
        endcase
    end
    // 槽号只有低 3 位用于 8 字符串，避免 4 bit 槽号在超出时越界取片
    wire [2:0] slot8=status_slot0[2:0];
    wire [7:0] shape_char=shape_text[63-slot8*8-:8];
    // 2026-11-01: 彩色检测模式的状态由 1/2/3 改成英文单词，与形状模式共用 8 字符槽。
    //   检测成功 -> TARGET / 本帧无目标 -> NONE / 等待首帧 -> WAIT / 其它 -> ERROR
    wire [63:0] colour_text=det_success0?"TARGET  ":
                             (det_status0==2)?"NONE    ":
                             (det_status0==1 || det_status0==0)?"WAIT    ":"ERROR   ";
    wire [7:0] colour_char=colour_text[63-slot8*8-:8];

    // ---- 2026-11-01: 区域警戒的文字并入主字库（原来那套 5x7 小字库停用）----
    // 文本内容、位置 (16,232)、配色都与原来一致，字形从 5x7 换成 12x24。
    localparam [11:0] GUARD_TX=12'd16, GUARD_TY=12'd232;
    wire [7:0] guard_reason_ch=(I_guard_reason<=4'd8)?(8'h30+{4'd0,I_guard_reason}):8'h3f;
    reg [95:0] guard_text;
    always @(*) begin
        case(I_guard_state)
            3'd0:   guard_text="OFF         ";
            3'd1:   guard_text="WAIT        ";
            3'd2:   guard_text="NO TARGET   ";
            3'd3:   guard_text="OUT         ";
            3'd4:   guard_text="PENDING     ";
            3'd5:   guard_text="ALARM       ";
            default:guard_text= I_guard_alarm ? "FAULT ALARM " : "FAULT       ";
        endcase
        if(GUARD_DIAGNOSTICS==1 && I_guard_state==3'd6)
            guard_text={"FAULT ",guard_reason_ch, I_guard_alarm?" ALRM":"     "};
    end
    wire [7:0] guard_char=guard_text[95-status_slot0*8-:8];
    wire [7:0] status_char0=guard_row0?guard_char:(shape_pixel0?shape_char:colour_char);

    // 状态字窗口：状态行 128px（8 槽）+ 警戒行 192px（12 槽）
    wire guard_row_now=(ZONE_GUARD!=0) && I_guard_enable && I_osd_en && I_de &&
        I_x>=GUARD_TX && I_x<GUARD_TX+12'd192 &&
        I_y>=GUARD_TY && I_y<GUARD_TY+12'd24;
    wire status_row_now=STATUS_ENABLE==1 && I_det_status!=0 &&
        I_x>=STATUS_X && I_x<STATUS_X+128 &&
        I_y>=STATUS_Y && I_y<STATUS_Y+24;
    // 警戒行独立于检测状态字：状态码为 0 时也要显示 OFF/WAIT
    wire in_status=status_row_now || guard_row_now;
    reg [11:0] dx0,dy0,bxlo0,bxhi0,bylo0,byhi0;
    reg box_valid0,inside1,edge2,edge3,edge4;
    // Only the inward-border predicate is consumed in S2. Store that bit
    // instead of four 13-bit distances; retain exact 13-bit subtraction.
    wire [12:0] box_left0={1'b0,dx0}-{1'b0,bxlo0};
    wire [12:0] box_right0={1'b0,bxhi0}-{1'b0,dx0};
    wire [12:0] box_top0={1'b0,dy0}-{1'b0,bylo0};
    wire [12:0] box_bottom0={1'b0,byhi0}-{1'b0,dy0};
    reg border1;
    reg [7:0] dchar1,dchar2;
    reg [4:0] dcol0,drow0,dcol1,drow1,dcol2,drow2,dcol3;
    reg dvalid0,dvalid1,dvalid2,dvalid3,dhit4;
    wire [23:0] drow_bits3;
    reg coord_valid0;
    reg [7:0] coord_dx0;
    reg [31:0] coord_bcd0;
    reg [7:0] coord_char;
    reg coord_slot_valid;
    // Both original font ports share a lossless dual-read ROM. Its outputs
    // replace the existing S3 registers; RGB/control/metadata latency stays 6.
    osd_font_rom_pair u_font_rom(.I_clk(I_clk),.I_rst_n(I_rst_n),
        .I_char_a(char2),.I_row_a(row2),.O_row_a(row_bits3),
        .I_char_b(dchar2),.I_row_b(drow2),.O_row_b(drow_bits3));

    // Fixed 13 x 16px slots: X:dddd Y:dddd, 208 x 24 at (16,192).
    // Digits use the existing 15px glyph crop, colon 8px; gaps stay background.
    // S0 bounded geometry/BCD snapshot -> S1 slot selection -> S2 register ->
    // S3 font row -> S4 bit -> S5 overlay, with no pixel-clock division.
    always @(*) begin
        coord_char=0;coord_slot_valid=1;
        case(coord_dx0[7:4])
            0:coord_char=8'h58;
            1,8:begin coord_char=8'h3a;coord_slot_valid=(coord_dx0[3:0]<8);end
            2:coord_char=8'h30+coord_bcd0[31:28];
            3:coord_char=8'h30+coord_bcd0[27:24];
            4:coord_char=8'h30+coord_bcd0[23:20];
            5:coord_char=8'h30+coord_bcd0[19:16];
            7:coord_char=8'h59;
            9:coord_char=8'h30+coord_bcd0[15:12];
            10:coord_char=8'h30+coord_bcd0[11:8];
            11:coord_char=8'h30+coord_bcd0[7:4];
            12:coord_char=8'h30+coord_bcd0[3:0];
            default:coord_slot_valid=0;
        endcase
        if((coord_dx0[7:4]>=2 && coord_dx0[7:4]<=5) ||
           (coord_dx0[7:4]>=9 && coord_dx0[7:4]<=12))
            coord_slot_valid=(coord_dx0[3:0]<15);
    end

    // S0: snapshot and clipping. Lower bounds outside the frame or reversed
    // intervals are invalid; large upper bounds clip without 12-bit wraparound.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            dx0<=0;dy0<=0;bxlo0<=0;bxhi0<=0;bylo0<=0;byhi0<=0;box_valid0<=0;
            det_status0<=0;shape_class0<=0;status_slot0<=0;
            shape_pixel0<=0;det_success0<=0;
            dcol0<=0;drow0<=0;dvalid0<=0;
            coord_valid0<=0;coord_dx0<=0;coord_bcd0<=0;
        end else begin
            dx0<=I_x;dy0<=I_y;bxlo0<=I_det_xmin;bylo0<=I_det_ymin;
            bxhi0<=I_det_xmax>=IMG_WIDTH?(IMG_WIDTH-1):I_det_xmax;
            byhi0<=I_det_ymax>=IMG_HEIGHT?(IMG_HEIGHT-1):I_det_ymax;
            box_valid0<=det_pixel;
            det_status0<=I_det_status;shape_class0<=I_shape_class;
            status_slot0<=shape_text_x[7:4];shape_pixel0<=shape_pixel;
            guard_row0<=guard_row_now;
            // 配色沿用原 5x7 字库：故障=品红、报警=红、WAIT/PENDING=黄、其它=青
            guard_col0<=(I_guard_state==3'd6)?2'd3:
                        (I_guard_alarm)?2'd2:
                        ((I_guard_state==3'd1 || I_guard_state==3'd4)?2'd1:2'd0);
            det_success0<=det_success;
            dcol0<={1'b0,shape_text_x[3:0]};
            // 2026-11-01: 警戒行自己的行原点。原先直接减 STATUS_Y，y=232 时得到 72，
            // 5 bit 截断成 8，既错位又越过 24 行读进下一个字形——就是上板看到的花屏。
            drow0<=guard_row_now?(I_y-GUARD_TY):(I_y-STATUS_Y);
            // 警戒行不受 det_pixel（检测包有效）约束，只受自身使能约束
            dvalid0<=(det_pixel && status_row_now && (shape_text_x[3:0]<15)) ||
                     (guard_row_now && (shape_text_x[3:0]<15));
            coord_dx0<=I_x-COORD_X;
            coord_bcd0<={I_center_x_bcd,I_center_y_bcd};
            coord_valid0<=det_pixel &&
                I_x>=COORD_X && I_x<COORD_X+208 && I_y>=COORD_Y && I_y<COORD_Y+24;
            // Status retains priority if a verification layout overlaps windows.
            if(!in_status) begin
                dcol0<=(I_x-COORD_X)&15;
                drow0<=I_y-COORD_Y;
            end else coord_valid0<=0;
        end
    end
    // S1 records the distance predicate; S2 gates it by registered inclusion.
    // Narrow boxes are wholly covered
    // when opposing inward borders overlap, with no out-of-frame expansion.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            inside1<=0;border1<=0;
            dchar1<=0;dcol1<=0;drow1<=0;dvalid1<=0;
            edge2<=0;dchar2<=0;dcol2<=0;drow2<=0;dvalid2<=0;
            edge3<=0;dcol3<=0;dvalid3<=0;
            edge4<=0;dhit4<=0;
        end else begin
            inside1<=box_valid0 && det_success0 && dx0>=bxlo0 && dx0<=bxhi0 && dy0>=bylo0 && dy0<=byhi0;
            border1<=box_left0<BORDER_WIDTH || box_right0<BORDER_WIDTH ||
                box_top0<BORDER_WIDTH || box_bottom0<BORDER_WIDTH;
            dchar1<=(coord_valid0 && det_success0)?coord_char:status_char0;
            dcol1<=dcol0;drow1<=drow0;
            dvalid1<=(dvalid0 && (!shape_pixel0 || shape_char!=8'h20)) ||
                (coord_valid0 && det_success0 && coord_slot_valid);
            edge2<=inside1 && border1;
            dchar2<=dchar1;dcol2<=dcol1;drow2<=drow1;dvalid2<=dvalid1;
            edge3<=edge2;dcol3<=dcol2;dvalid3<=dvalid2;
            edge4<=edge3;dhit4<=dvalid3 && drow_bits3[5'd23-dcol3];
        end
    end

    // 2026-11-01: every mode now has a full label (2 or 4 glyphs); mode 14
    // (passthrough) stays blank on purpose. cell counts from the left.
    function [7:0] F_mode_char;
        input [3:0] mode;
        input [2:0] idx;
        begin
            case(mode)
                4'd0 : F_mode_char=(idx==0)?8'h80:(idx==1)?8'h81:8'd0;                        // 原图
                4'd1 : F_mode_char=(idx==0)?8'h82:(idx==1)?8'h83:(idx==2)?8'h84:
                                   (idx==3)?8'h85:8'd0;                                        // 边缘检测
                4'd2 : F_mode_char=(idx==0)?8'h86:(idx==1)?8'h87:8'd0;                        // 反相
                4'd3 : F_mode_char=(idx==0)?8'h88:(idx==1)?8'h89:8'd0;                        // 浮雕
                4'd4 : F_mode_char=(idx==0)?8'h8a:(idx==1)?8'h8b:8'd0;                        // 卡通
                4'd5 : F_mode_char=(idx==0)?8'h8c:(idx==1)?8'h8d:(idx==2)?8'h8e:
                                   (idx==3)?8'h8f:8'd0;                                        // 红色填充
                4'd6 : F_mode_char=(idx==0)?8'h8c:(idx==1)?8'h8d:(idx==2)?8'h84:
                                   (idx==3)?8'h85:8'd0;                                        // 红色检测
                4'd7 : F_mode_char=(idx==0)?8'h91:(idx==1)?8'h8d:(idx==2)?8'h8e:
                                   (idx==3)?8'h8f:8'd0;                                        // 蓝色填充
                4'd8 : F_mode_char=(idx==0)?8'h91:(idx==1)?8'h8d:(idx==2)?8'h84:
                                   (idx==3)?8'h85:8'd0;                                        // 蓝色检测
                4'd9 : F_mode_char=(idx==0)?8'h90:(idx==1)?8'h8d:(idx==2)?8'h8e:
                                   (idx==3)?8'h8f:8'd0;                                        // 绿色填充
                4'd10: F_mode_char=(idx==0)?8'h90:(idx==1)?8'h8d:(idx==2)?8'h84:
                                   (idx==3)?8'h85:8'd0;                                        // 绿色检测
                4'd11: F_mode_char=(idx==0)?8'h92:(idx==1)?8'h93:(idx==2)?8'h84:
                                   (idx==3)?8'h85:8'd0;                                        // 运动检测
                4'd12: F_mode_char=(idx==0)?8'h94:(idx==1)?8'h95:(idx==2)?8'h96:
                                   (idx==3)?8'h97:8'd0;                                        // 形状分类
                4'd13: F_mode_char=(idx==0)?8'ha0:(idx==1)?8'ha1:(idx==2)?8'h98:
                                   (idx==3)?8'h99:8'd0;                                        // 区域警戒
                4'd15: F_mode_char=(idx==0)?8'h4c:(idx==1)?8'h53:(idx==2)?8'h43:
                                   (idx==3)?8'h9a:(idx==4)?8'h9b:8'd0;                        // LSC旁路
                default:F_mode_char=8'd0;                                                       // 14 直通: 不显示
            endcase
        end
    endfunction
    function [3:0] F_scale24_row;
        input [4:0] p;
        begin
            case(p)
                5'd0: F_scale24_row=4'd0;
                5'd1: F_scale24_row=4'd0;
                5'd2: F_scale24_row=4'd1;
                5'd3: F_scale24_row=4'd2;
                5'd4: F_scale24_row=4'd2;
                5'd5: F_scale24_row=4'd3;
                5'd6: F_scale24_row=4'd4;
                5'd7: F_scale24_row=4'd4;
                5'd8: F_scale24_row=4'd5;
                5'd9: F_scale24_row=4'd6;
                5'd10: F_scale24_row=4'd6;
                5'd11: F_scale24_row=4'd7;
                5'd12: F_scale24_row=4'd8;
                5'd13: F_scale24_row=4'd8;
                5'd14: F_scale24_row=4'd9;
                5'd15: F_scale24_row=4'd10;
                5'd16: F_scale24_row=4'd10;
                5'd17: F_scale24_row=4'd11;
                5'd18: F_scale24_row=4'd12;
                5'd19: F_scale24_row=4'd12;
                5'd20: F_scale24_row=4'd13;
                5'd21: F_scale24_row=4'd14;
                5'd22: F_scale24_row=4'd14;
                5'd23: F_scale24_row=4'd15;
                default: F_scale24_row=4'd0;
            endcase
        end
    endfunction
    function [3:0] F_scale24_column;
        input [4:0] p;
        begin
            case(p)
                5'd0: F_scale24_column=4'd0;
                5'd1: F_scale24_column=4'd1;
                5'd2: F_scale24_column=4'd1;
                5'd3: F_scale24_column=4'd2;
                5'd4: F_scale24_column=4'd3;
                5'd5: F_scale24_column=4'd3;
                5'd6: F_scale24_column=4'd4;
                5'd7: F_scale24_column=4'd5;
                5'd8: F_scale24_column=4'd5;
                5'd9: F_scale24_column=4'd6;
                5'd10: F_scale24_column=4'd7;
                5'd11: F_scale24_column=4'd7;
                5'd12: F_scale24_column=4'd8;
                5'd13: F_scale24_column=4'd9;
                5'd14: F_scale24_column=4'd9;
                5'd15: F_scale24_column=4'd10;
                5'd16: F_scale24_column=4'd11;
                5'd17: F_scale24_column=4'd11;
                5'd18: F_scale24_column=4'd12;
                5'd19: F_scale24_column=4'd13;
                5'd20: F_scale24_column=4'd13;
                5'd21: F_scale24_column=4'd14;
                5'd22: F_scale24_column=4'd15;
                5'd23: F_scale24_column=4'd15;
                default: F_scale24_column=4'd0;
            endcase
        end
    endfunction
    function [7:0] F_decimal_pair;
        input [6:0] p;
        begin
            case(p)
                7'd0: F_decimal_pair=8'h00;
                7'd1: F_decimal_pair=8'h01;
                7'd2: F_decimal_pair=8'h02;
                7'd3: F_decimal_pair=8'h03;
                7'd4: F_decimal_pair=8'h04;
                7'd5: F_decimal_pair=8'h05;
                7'd6: F_decimal_pair=8'h06;
                7'd7: F_decimal_pair=8'h07;
                7'd8: F_decimal_pair=8'h08;
                7'd9: F_decimal_pair=8'h09;
                7'd10: F_decimal_pair=8'h10;
                7'd11: F_decimal_pair=8'h11;
                7'd12: F_decimal_pair=8'h12;
                7'd13: F_decimal_pair=8'h13;
                7'd14: F_decimal_pair=8'h14;
                7'd15: F_decimal_pair=8'h15;
                7'd16: F_decimal_pair=8'h16;
                7'd17: F_decimal_pair=8'h17;
                7'd18: F_decimal_pair=8'h18;
                7'd19: F_decimal_pair=8'h19;
                7'd20: F_decimal_pair=8'h20;
                7'd21: F_decimal_pair=8'h21;
                7'd22: F_decimal_pair=8'h22;
                7'd23: F_decimal_pair=8'h23;
                7'd24: F_decimal_pair=8'h24;
                7'd25: F_decimal_pair=8'h25;
                7'd26: F_decimal_pair=8'h26;
                7'd27: F_decimal_pair=8'h27;
                7'd28: F_decimal_pair=8'h28;
                7'd29: F_decimal_pair=8'h29;
                7'd30: F_decimal_pair=8'h30;
                7'd31: F_decimal_pair=8'h31;
                7'd32: F_decimal_pair=8'h32;
                7'd33: F_decimal_pair=8'h33;
                7'd34: F_decimal_pair=8'h34;
                7'd35: F_decimal_pair=8'h35;
                7'd36: F_decimal_pair=8'h36;
                7'd37: F_decimal_pair=8'h37;
                7'd38: F_decimal_pair=8'h38;
                7'd39: F_decimal_pair=8'h39;
                7'd40: F_decimal_pair=8'h40;
                7'd41: F_decimal_pair=8'h41;
                7'd42: F_decimal_pair=8'h42;
                7'd43: F_decimal_pair=8'h43;
                7'd44: F_decimal_pair=8'h44;
                7'd45: F_decimal_pair=8'h45;
                7'd46: F_decimal_pair=8'h46;
                7'd47: F_decimal_pair=8'h47;
                7'd48: F_decimal_pair=8'h48;
                7'd49: F_decimal_pair=8'h49;
                7'd50: F_decimal_pair=8'h50;
                7'd51: F_decimal_pair=8'h51;
                7'd52: F_decimal_pair=8'h52;
                7'd53: F_decimal_pair=8'h53;
                7'd54: F_decimal_pair=8'h54;
                7'd55: F_decimal_pair=8'h55;
                7'd56: F_decimal_pair=8'h56;
                7'd57: F_decimal_pair=8'h57;
                7'd58: F_decimal_pair=8'h58;
                7'd59: F_decimal_pair=8'h59;
                7'd60: F_decimal_pair=8'h60;
                7'd61: F_decimal_pair=8'h61;
                7'd62: F_decimal_pair=8'h62;
                7'd63: F_decimal_pair=8'h63;
                7'd64: F_decimal_pair=8'h64;
                7'd65: F_decimal_pair=8'h65;
                7'd66: F_decimal_pair=8'h66;
                7'd67: F_decimal_pair=8'h67;
                7'd68: F_decimal_pair=8'h68;
                7'd69: F_decimal_pair=8'h69;
                7'd70: F_decimal_pair=8'h70;
                7'd71: F_decimal_pair=8'h71;
                7'd72: F_decimal_pair=8'h72;
                7'd73: F_decimal_pair=8'h73;
                7'd74: F_decimal_pair=8'h74;
                7'd75: F_decimal_pair=8'h75;
                7'd76: F_decimal_pair=8'h76;
                7'd77: F_decimal_pair=8'h77;
                7'd78: F_decimal_pair=8'h78;
                7'd79: F_decimal_pair=8'h79;
                7'd80: F_decimal_pair=8'h80;
                7'd81: F_decimal_pair=8'h81;
                7'd82: F_decimal_pair=8'h82;
                7'd83: F_decimal_pair=8'h83;
                7'd84: F_decimal_pair=8'h84;
                7'd85: F_decimal_pair=8'h85;
                7'd86: F_decimal_pair=8'h86;
                7'd87: F_decimal_pair=8'h87;
                7'd88: F_decimal_pair=8'h88;
                7'd89: F_decimal_pair=8'h89;
                7'd90: F_decimal_pair=8'h90;
                7'd91: F_decimal_pair=8'h91;
                7'd92: F_decimal_pair=8'h92;
                7'd93: F_decimal_pair=8'h93;
                7'd94: F_decimal_pair=8'h94;
                7'd95: F_decimal_pair=8'h95;
                7'd96: F_decimal_pair=8'h96;
                7'd97: F_decimal_pair=8'h97;
                7'd98: F_decimal_pair=8'h98;
                7'd99: F_decimal_pair=8'h99;
                default: F_decimal_pair=8'd0;
            endcase
        end
    endfunction

    anlogic_logo_rom_sync #(.PIPELINE_STAGES(3)) u_logo(
        .I_clk(I_clk), .I_addr(logo_addr_q), .O_pixel(logo_pixel));

    // S0: bounded geometry and the configuration sampled for THIS pixel.
    // Brightness is a signed nine-bit two's-complement value: abs <= 256.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            tx0<=0;ty0<=0;ax0<=0;ay0<=0;lx0<=0;ly0<=0;
            text_valid0<=0;adj_valid0<=0;mode0<=0;
            adj_idx0<=0;fill_color0<=0;adj_negative0<=0;adj_magnitude0<=0;counter0<=0;
            fps0<=0;fy0<=0;tx2<=0;lx2<=0;text2_valid0<=0;fps_dot_q<=0;
            lat_bcd0<=0;lat_state0<=0;laty0<=0;ltx<=0;text3_valid0<=0;lat_dot_q<=0;
            lat_row_bcd<=0;lat_row_state<=0;lat_row_locked<=0;
            osd_q<=0;logo_valid_q<=0;debug_q<=0;
            // Logo pipeline is masked by the unchanged resettable valid path.
            for(k=0;k<5;k=k+1) begin rgb_q[k]<=0;ctl_q[k]<=0;x_q[k]<=0;end
            top_band_q<=0;bot_band_q<=0;lab_sel_q<=0;lab_col_q<=0;
            guard_row_q<=0;guard_col_q<=0;
        end else begin
            tx0<=I_x-12'd1070;ty0<=I_y-12'd8;
            // 2026-11-01: 模式标签最长 4 字（96px），行窗口与标签原点整体左移 48px，
            // 右边界仍是 1270，CNT/数值位置完全不变。
            label_x0<=I_x-12'd1014;
            ax0<=I_x-12'd1153;ay0<=I_y-ADJ_Y;
            lx0<=I_x-12'd13;ly0<=I_y+12'd14;
            text_valid0<=I_de && I_x>=12'd1014 && I_x<12'd1270 &&
                                  I_y>=12'd8 && I_y<12'd32;
            adj_valid0<=I_de && I_x>=12'd1153 && I_x<12'd1270 &&
                                 I_y>=ADJ_Y && I_y<(ADJ_Y+12'd24);
            mode0<=I_mode;adj_idx0<=I_adj_idx;fill_color0<=I_fill_color;
            show_temp0<=I_show_temp;
            gain_level0<=I_gain_level;
            is_trim0<=I_wb_is_trim; trim_neg0<=I_wb_trim_neg;
            wb_val0<=I_wb_temp[7:0];
            // 13-bit Kelvin (2000..8000) -> 4 decimal digits, no divider
            if(I_wb_temp>=13'd8000) begin wd3<=4'd8; wrem<=I_wb_temp-13'd8000; end
            else if(I_wb_temp>=13'd7000) begin wd3<=4'd7; wrem<=I_wb_temp-13'd7000; end
            else if(I_wb_temp>=13'd6000) begin wd3<=4'd6; wrem<=I_wb_temp-13'd6000; end
            else if(I_wb_temp>=13'd5000) begin wd3<=4'd5; wrem<=I_wb_temp-13'd5000; end
            else if(I_wb_temp>=13'd4000) begin wd3<=4'd4; wrem<=I_wb_temp-13'd4000; end
            else if(I_wb_temp>=13'd3000) begin wd3<=4'd3; wrem<=I_wb_temp-13'd3000; end
            else if(I_wb_temp>=13'd2000) begin wd3<=4'd2; wrem<=I_wb_temp-13'd2000; end
            else begin wd3<=4'd0; wrem<=I_wb_temp; end
            if(wrem>=13'd900) begin wd2<=4'd9; wrem<=wrem-13'd900; end
            else if(wrem>=13'd800) begin wd2<=4'd8; wrem<=wrem-13'd800; end
            else if(wrem>=13'd700) begin wd2<=4'd7; wrem<=wrem-13'd700; end
            else if(wrem>=13'd600) begin wd2<=4'd6; wrem<=wrem-13'd600; end
            else if(wrem>=13'd500) begin wd2<=4'd5; wrem<=wrem-13'd500; end
            else if(wrem>=13'd400) begin wd2<=4'd4; wrem<=wrem-13'd400; end
            else if(wrem>=13'd300) begin wd2<=4'd3; wrem<=wrem-13'd300; end
            else if(wrem>=13'd200) begin wd2<=4'd2; wrem<=wrem-13'd200; end
            else if(wrem>=13'd100) begin wd2<=4'd1; wrem<=wrem-13'd100; end
            else begin wd2<=4'd0; end
            if(wrem>=13'd90) begin wd1<=4'd9; wrem<=wrem-13'd90; end
            else if(wrem>=13'd80) begin wd1<=4'd8; wrem<=wrem-13'd80; end
            else if(wrem>=13'd70) begin wd1<=4'd7; wrem<=wrem-13'd70; end
            else if(wrem>=13'd60) begin wd1<=4'd6; wrem<=wrem-13'd60; end
            else if(wrem>=13'd50) begin wd1<=4'd5; wrem<=wrem-13'd50; end
            else if(wrem>=13'd40) begin wd1<=4'd4; wrem<=wrem-13'd40; end
            else if(wrem>=13'd30) begin wd1<=4'd3; wrem<=wrem-13'd30; end
            else if(wrem>=13'd20) begin wd1<=4'd2; wrem<=wrem-13'd20; end
            else if(wrem>=13'd10) begin wd1<=4'd1; wrem<=wrem-13'd10; end
            else begin wd1<=4'd0; end
            wd0<=wrem[3:0];
            adj_negative0<=(adj_idx0==3'd5)&&I_bright_level[8];
            adj_magnitude0<=
                // Shape/area input was previously unused, so AMIN displayed DOM.
                (I_adj_idx>=4'd8)?{1'b0,I_shape_value}:
                (adj_idx0==3'd0)?{4'd0,I_sat_level}:
                (adj_idx0==3'd1)?{1'b0,wb_val0}:                          // white balance readout
                (adj_idx0==3'd6)?{1'b0,wb_val0}:                          // exposure (0.1 ms)
                (adj_idx0==3'd7)?{5'd0,wb_val0}:                            // TG (target brightness)
                (adj_idx0==3'd2)?{1'b0,gain_level0}:                      // gain: value only
                (adj_idx0==3'd5)?(I_bright_level[8]?(~I_bright_level+9'd1):I_bright_level):
                (adj_idx0==3'd3)?{1'b0,I_th_min}:{1'b0,I_th_dom};
            counter0<=I_counter_bcd;
            // FPS row: y 32..56, the band immediately below the CNT row (y 8..32),
            // same x origin as CNT.  Mutually exclusive with text_valid0 and
            // adj_valid0 (ADJ_Y = IMG_HEIGHT-32 = 688), so it can share the one
            // glyph selector without disturbing any existing window.
            fy0<=I_y-12'd32;
            text2_valid0<=I_de && I_x>=12'd1062 && I_x<12'd1270 &&
                                   I_y>=12'd32 && I_y<12'd56;
            tx2<=I_x-12'd1070;            // digital origin, same value as the CNT row
        lx2<=I_x-12'd1062;            // label origin, same value as the CNT row
        fps0<=I_fps_bcd;
            // Decimal point between units and tenths: a fixed 4x4 block sitting on
            // the baseline.  Drawn directly rather than fetched from the font,
            // because osd_char_lib has no '.' and new glyphs must not be added here.
            fps_dot_q<={fps_dot_q[3:0],
                I_de && I_x>=12'd1210 && I_x<12'd1214 && I_y>=12'd51 && I_y<12'd55};
            // L04: LAT row, y 56..80 -- the band below the FPS row (32..56), same x origin
            // as the CNT/FPS rows.  Mutually exclusive with text_valid0 (8..32),
            // text2_valid0 (32..56) and adj_valid0 (ADJ_Y = IMG_HEIGHT-32 = 688), so it can
            // share the single glyph selector without touching any existing window.
            laty0<=I_y-12'd56;
            text3_valid0<=I_de && I_x>=12'd1062 && I_x<12'd1270 &&
                                  I_y>=12'd56 && I_y<12'd80;
            ltx<=I_x-12'd1070;                    // digital origin, as for CNT/FPS
            if(I_vsync || (I_de && I_y<12'd56)) lat_row_locked<=0;
            else if(I_de && I_y<12'd80 && !lat_row_locked) begin
                lat_row_locked<=1;
                lat_row_bcd<=I_lat_bcd;
                lat_row_state<=I_lat_state;
            end
            lat_bcd0<=lat_pixel_bcd;
            lat_state0<=lat_pixel_state;
            // Decimal point between units and tenths: a fixed 4x4 block on the baseline.
            // Drawn directly because osd_char_lib has no '.' and no glyph may be added.
            // Gated by the live state so it only shows for a valid reading.
            lat_dot_q<={lat_dot_q[3:0],
                (lat_pixel_state==3'd1) && I_de &&
                I_x>=12'd1210 && I_x<12'd1214 && I_y>=12'd75 && I_y<12'd79};

            rgb_q[0]<=I_rgb;ctl_q[0]<={I_vsync,I_hsync,I_de};x_q[0]<=I_x;
            top_band_q<={top_band_q[3:0],(I_y<TOP_BAND_Y)};
            bot_band_q<={bot_band_q[3:0],(I_y>=ADJ_Y)};
            // 模式标签所在的一小段 x 窗口（CNT 行最左 96px），以及该模式的标签颜色
            lab_sel_q<={lab_sel_q[3:0],
                I_de && I_x>=12'd1014 && I_x<12'd1110 && I_y>=12'd8 && I_y<12'd32};
            guard_row_q<={guard_row_q[3:0],guard_row_now};
            guard_col_q<={guard_col_q[7:0],guard_col0};
            lab_col_q<={lab_col_q[7:0],
                (I_mode==4'd5 || I_mode==4'd6 )?2'd1:
                (I_mode==4'd7 || I_mode==4'd8 )?2'd2:
                (I_mode==4'd9 || I_mode==4'd10)?2'd3:2'd0};
            for(k=1;k<5;k=k+1) begin
                rgb_q[k]<=rgb_q[k-1];ctl_q[k]<=ctl_q[k-1];x_q[k]<=x_q[k-1];
            end
            osd_q<={osd_q[3:0],I_osd_en};
            logo_valid_q<={logo_valid_q[3:0],
                I_de && I_x>=12'd13 && I_x<12'd173 && I_y<12'd146};
            // Legacy DEBUG2/3 samples status at its final overlay, three clocks
            // after the post-ISP input. Delay exactly the three added clocks.
            debug_q<={debug_q[7:0],I_debug_status};
            // ROM address S1, synchronous ROM S2, then S3/S4 alignment.
            // Three ROM clocks equal old ROM plus logo_pixel3/logo_pixel4.
        end
    end
    // No asynchronous reset on a BRAM address register.
    always @(posedge I_clk)
        logo_addr_q<=logo_valid_q[0]?(ly0*15'd160+lx0):15'd0;

    // S1: region-local glyph selection. No font lookup in this stage.
    always @(*) begin
        next_char=0;next_x=0;next_y=0;next_kind=NATIVE;
        next_valid=0;next_adj_digit=0;next_adj_index=0;adj_dx=0;
        if(text_valid0) begin
            if(label_x0<12'd96) begin
                // 2026-11-01: 标签右对齐到 label_x0=96（屏幕 x=1110），左边界随字数变化。
                next_y=ty0;next_kind=CHINESE;
                if(mode0==4'd15) begin
                    // LSC旁路：L/S/C 各 12px + 旁路 各 24px，共 84px -> 起点 12
                    if(label_x0>=12'd12 && label_x0<12'd24) begin
                        next_char=8'h4c;next_x=label_x0-12'd12;
                        next_valid=(next_y<5'd24);
                    end else if(label_x0>=12'd24 && label_x0<12'd36) begin
                        next_char=8'h53;next_x=label_x0-12'd24;
                        next_valid=(next_y<5'd24);
                    end else if(label_x0>=12'd36 && label_x0<12'd48) begin
                        next_char=8'h43;next_x=label_x0-12'd36;
                        next_valid=(next_y<5'd24);
                    end else if(label_x0>=12'd48 && label_x0<12'd72) begin
                        next_char=8'h9a;next_x=label_x0-12'd48;
                        next_valid=(next_x<5'd24 && next_y<5'd24);
                    end else if(label_x0>=12'd72 && label_x0<12'd96) begin
                        next_char=8'h9b;next_x=label_x0-12'd72;
                        next_valid=(next_x<5'd24 && next_y<5'd24);
                    end
                end else begin
                    // 2 字标签从 48 起，4 字标签从 0 起，均按 24px 网格
                    lab_start=(mode0==4'd0 || mode0==4'd2 ||
                               mode0==4'd3 || mode0==4'd4) ? 12'd48 : 12'd0;
                    lab_dx=label_x0-lab_start;      // 12 位，负数回绕成大数
                    if(lab_dx<12'd24)      begin lab_cell=3'd0;next_x=lab_dx[4:0];end
                    else if(lab_dx<12'd48) begin lab_cell=3'd1;next_x=lab_dx-12'd24;end
                    else if(lab_dx<12'd72) begin lab_cell=3'd2;next_x=lab_dx-12'd48;end
                    else if(lab_dx<12'd96) begin lab_cell=3'd3;next_x=lab_dx-12'd72;end
                    else begin lab_cell=3'd0;next_x=5'd24;end
                    next_char=F_mode_char(mode0,lab_cell);
                    next_valid=(next_char!=0 && next_x<5'd24 && next_y<5'd24);
                end
            end else if(tx0>=12'd52 && tx0<12'd100 && ty0<12'd24) begin
                if(tx0<12'd68) begin next_char=8'h43;next_x=tx0-12'd52;end
                else if(tx0<12'd84) begin next_char=8'h4e;next_x=tx0-12'd68;end
                else begin next_char=8'h54;next_x=tx0-12'd84;end
                next_y=ty0;next_kind=NATIVE;next_valid=1;
            end else if(tx0>=12'd100 && tx0<12'd108 && ty0<12'd24) begin
                next_char=8'h3a;next_x=tx0-12'd100;next_y=ty0;
                next_kind=COLON;next_valid=1;
            end
            if(tx0>=12'd110 && tx0<12'd200) begin
                if(tx0<12'd125) begin next_char=8'h30+counter0[23:20];next_x=tx0-12'd110;end
                else if(tx0<12'd140) begin next_char=8'h30+counter0[19:16];next_x=tx0-12'd125;end
                else if(tx0<12'd155) begin next_char=8'h30+counter0[15:12];next_x=tx0-12'd140;end
                else if(tx0<12'd170) begin next_char=8'h30+counter0[11:8];next_x=tx0-12'd155;end
                else if(tx0<12'd185) begin next_char=8'h30+counter0[7:4];next_x=tx0-12'd170;end
                else begin next_char=8'h30+counter0[3:0];next_x=tx0-12'd185;end
                next_y=ty0;next_kind=DIGIT;next_valid=(next_x<5'd15 && next_y<5'd24);
            end
        end
        // FPS line.  Layout mirrors the CNT row: a 48px label, ':' , then three
        // 15px digit slots, with the decimal point drawn between digits 2 and 3.
        // 'F','P','S' and ':' already exist in osd_char_lib (0x46/0x50/0x53/0x3a).
        // text2_valid0 never overlaps text_valid0, so sharing the one selector is safe.
        if(text2_valid0) begin
            next_char=0;next_valid=0;
            if(fy0<12'd24) begin
                if(tx2>=12'd52 && tx2<12'd100) begin
                    if(tx2<12'd68)      begin next_char=8'h46;next_x=tx2-12'd52;end  // F
                    else if(tx2<12'd84) begin next_char=8'h50;next_x=tx2-12'd68;end  // P
                    else                begin next_char=8'h53;next_x=tx2-12'd84;end  // S
                    next_y=fy0;next_kind=NATIVE;next_valid=1;
                end else if(tx2>=12'd100 && tx2<12'd108) begin
                    next_char=8'h3a;next_x=tx2-12'd100;next_y=fy0;
                    next_kind=COLON;next_valid=1;
                end else if(tx2>=12'd110 && tx2<12'd125) begin
                    next_char=8'h30+fps0[11:8];next_x=tx2-12'd110;next_y=fy0;
                    next_kind=DIGIT;next_valid=(next_x<5'd15);
                end else if(tx2>=12'd125 && tx2<12'd140) begin
                    next_char=8'h30+fps0[7:4];next_x=tx2-12'd125;next_y=fy0;
                    next_kind=DIGIT;next_valid=(next_x<5'd15);
                end else if(tx2>=12'd145 && tx2<12'd160) begin
                    next_char=8'h30+fps0[3:0];next_x=tx2-12'd145;next_y=fy0;
                    next_kind=DIGIT;next_valid=(next_x<5'd15);
                end
            end
        end
        // L04: LAT line.  Layout mirrors the proven FPS row: a 48px label at 52..100,
        // then the value at 100..200.  Valid reading: four 15px BCD digit slots with a
        // geometric decimal point between units and tenths, plus the 'MS' unit.
        // Non-valid: a state word occupying the same 16px slots, so the row never shows
        // stale digits.  All glyphs (L,A,T,M,S,W,O,V,R,E,'-') already exist in osd_char_lib.
        if(text3_valid0) begin
            next_char=0;next_valid=0;next_y=laty0;
            if(laty0<12'd24) begin
                if(ltx>=12'd52 && ltx<12'd100) begin
                    if(ltx<12'd68)      begin next_char=8'h4c;next_x=ltx-12'd52;end   // L
                    else if(ltx<12'd84) begin next_char=8'h41;next_x=ltx-12'd68;end   // A
                    else                begin next_char=8'h54;next_x=ltx-12'd84;end   // T
                    next_kind=NATIVE;next_valid=1;
                end else if(ltx>=12'd100 && ltx<12'd108) begin
                    next_char=8'h3a;next_x=ltx-12'd100;next_y=laty0;
                    next_kind=COLON;next_valid=1;
                end else if(lat_state0==3'd1) begin
                    // 2026-11-01: ':' inserted after the label; the hundreds-of-ms slot
                    // (lat_bcd0[15:12], always 0 because latency never exceeds 30 ms) is
                    // dropped, so the value is tens.units.tenths with the dot at 140..144,
                    // exactly like the FPS row.
                    if(ltx>=12'd110 && ltx<12'd125) begin
                        next_char=8'h30+lat_bcd0[11:8];next_x=ltx-12'd110;
                        next_kind=DIGIT;next_valid=(next_x<5'd15);
                    end else if(ltx>=12'd125 && ltx<12'd140) begin
                        next_char=8'h30+lat_bcd0[7:4];next_x=ltx-12'd125;
                        next_kind=DIGIT;next_valid=(next_x<5'd15);
                    end else if(ltx>=12'd145 && ltx<12'd160) begin
                        next_char=8'h30+lat_bcd0[3:0];next_x=ltx-12'd145;
                        next_kind=DIGIT;next_valid=(next_x<5'd15);
                    end else if(ltx>=12'd168 && ltx<12'd184) begin
                        next_char=8'h4d;next_x=ltx-12'd168;                       // M
                        next_kind=NATIVE;next_valid=1;
                    end else if(ltx>=12'd184 && ltx<12'd200) begin
                        next_char=8'h53;next_x=ltx-12'd184;                       // S
                        next_kind=NATIVE;next_valid=1;
                    end
                end else begin
                    // WAIT(4) / OVR(3) / STALE(5) / ----(4).
                    // Slot boundaries follow the value field (110/125/140/155/170) so the
                    // state word lines up under the digits. The dot rect at 140..144 is
                    // gated by lat_pixel_state==1, so it never lights during a state word.
                    if(ltx>=12'd110 && ltx<12'd125) begin
                        next_char=(lat_state0==3'd0)?8'h57:      // WAIT  -> W
                                  (lat_state0==3'd2)?8'h4f:      // OVR   -> O
                                  (lat_state0==3'd3)?8'h53:8'h2d;// STALE -> S, ---- -> -
                        next_x=ltx-12'd110;next_kind=NATIVE;next_valid=(next_x<5'd15);
                    end else if(ltx>=12'd125 && ltx<12'd140) begin
                        next_char=(lat_state0==3'd0)?8'h41:      // A
                                  (lat_state0==3'd2)?8'h56:      // V
                                  (lat_state0==3'd3)?8'h54:8'h2d;// T, -
                        next_x=ltx-12'd125;next_kind=NATIVE;next_valid=(next_x<5'd15);
                    end else if(ltx>=12'd140 && ltx<12'd155) begin
                        next_char=(lat_state0==3'd0)?8'h49:      // I
                                  (lat_state0==3'd2)?8'h52:      // R
                                  (lat_state0==3'd3)?8'h41:8'h2d;// A, -
                        next_x=ltx-12'd140;next_kind=NATIVE;next_valid=(next_x<5'd15);
                    end else if(ltx>=12'd155 && ltx<12'd170) begin
                        next_char=(lat_state0==3'd0)?8'h54:
                                  (lat_state0==3'd3)?8'h4c:8'h2d;// WAIT->T, STALE->L, invalid->-
                        next_x=ltx-12'd155;next_kind=NATIVE;
                        next_valid=(lat_state0!=3'd1 && lat_state0!=3'd2) && (next_x<5'd15);
                    end else if(ltx>=12'd170 && ltx<12'd186) begin
                        next_char=8'h45;next_x=ltx-12'd170;                      // E
                        next_kind=NATIVE;next_valid=(lat_state0==3'd3);
                    end
                end
            end
        end
        if(adj_valid0) begin
            next_char=0;next_valid=0;next_adj_digit=0;
            if(show_temp0) begin
                // Item readout. Label: item 1 = AWB (white balance), item 2 = AE (gain).
                // Automatic: the value area spells AUTO across its four 16px windows.
                // Manual: only the sign is drawn here -- the digits are produced by the same
                // next_adj_digit / hundreds-remainder path that the saturation readout uses,
                // which is the mechanism known to render correctly on hardware.
                if(ax0<12'd48) begin
                    next_char=(adj_idx0==3'd1)                                    // AWB
                              ? ((ax0<12'd12)?8'h41:((ax0<12'd24)?8'h57:8'h42))
                              : (adj_idx0==3'd2) ? ((ax0<12'd12)?8'h41:8'h47)     // AG
                              : (adj_idx0==3'd7) ? ((ax0<12'd12)?8'h54:8'h47)                     // TG (target brightness)
                              : ((ax0<12'd12)?8'h41:8'h45);                       // AE
                    next_x=(ax0<12'd12)?ax0:((ax0<12'd24)?(ax0-12'd12):((ax0<12'd36)?(ax0-12'd24):(ax0-12'd36)));
                    next_y=ay0;next_kind=NATIVE;
                    next_valid=(next_x<5'd12) && ((adj_idx0==3'd1) ? (ax0<12'd36) :
                                                 (adj_idx0==3'd7) ? (ax0<12'd24) :
                                                 (ax0<12'd24));
                end else if(ax0>=12'd52 && ax0<12'd117) begin
                    if(!is_trim0) begin
                        next_char=(ax0<12'd68)?8'h41:((ax0<12'd84)?8'h55:((ax0<12'd100)?8'h54:8'h4f));
                        next_x=(ax0<12'd68)?(ax0-12'd52):((ax0<12'd84)?(ax0-12'd68):((ax0<12'd100)?(ax0-12'd84):(ax0-12'd100)));
                        next_y=ay0;next_kind=NATIVE;next_valid=(next_x<5'd16);
                    end else if(adj_idx0==3'd1 && ax0<12'd68) begin
                        next_char=(trim_neg0?8'h2d:8'h2b);
                        next_x=ax0-12'd52;next_y=ay0;next_kind=NATIVE;
                        next_valid=(next_x<5'd16 && next_y<5'd24);
                    end else if(ax0>=12'd72) begin
                        // manual digits: identical geometry and thresholds to the saturation
                        // readout (hundreds window needs >=100, tens needs >=10), and it must
                        // live inside THIS branch -- the else-if chain never reaches the legacy
                        // ADJ branch while show_temp0 is high.
                        if(ax0<12'd87) begin adj_dx=ax0-12'd72;next_adj_index=2;end
                        else if(ax0<12'd102) begin adj_dx=ax0-12'd87;next_adj_index=1;end
                        else begin adj_dx=ax0-12'd102;next_adj_index=0;end
                        next_x={1'b0,adj_dx};next_y=ay0;next_kind=DIGIT;
                        next_adj_digit=1;
                        next_valid=(adj_dx<4'd15 && next_y<5'd24) &&
                            ((next_adj_index==2)?(adj_magnitude0>=9'd100):
                             (next_adj_index==1)?(adj_magnitude0>=9'd10):1'b1);
                    end else begin
                        next_valid=1'b0;
                    end
                end else begin
                    next_valid=1'b0;
                end
            end else             if(ax0<12'd48) begin


                next_y=ay0;
                if(adj_idx0>=4'd8)begin
                    // GRAY / PAPR / FIT / AMIN / AMAX (area values are gears).
                    case(adj_idx0)
                        8:next_char=ax0<12?8'h47:ax0<24?8'h52:ax0<36?8'h41:8'h59;
                        9:next_char=ax0<12?8'h50:ax0<24?8'h41:ax0<36?8'h50:8'h52;
                        10:next_char=ax0<12?8'h46:ax0<24?8'h49:ax0<36?8'h54:8'd0;
                        11:next_char=ax0<12?8'h41:ax0<24?8'h4d:ax0<36?8'h49:8'h4e;
                        default:next_char=ax0<12?8'h41:ax0<24?8'h4d:ax0<36?8'h41:8'h58;
                    endcase
                    next_x=ax0<12?ax0:ax0<24?ax0-12:ax0<36?ax0-24:ax0-36;
                    next_kind=NATIVE;next_valid=(next_char!=0 && next_x<12);
                end else if(adj_idx0==3'd3 || adj_idx0==3'd4) begin
                    // four half-width letters: <colour>MIN / <colour>DOM (R/B/G, each 12px)
                    next_char=(ax0<12'd12)?((fill_color0==2'd1)?8'h47:((fill_color0==2'd2)?8'h42:8'h52)):
                              ((ax0<12'd24)?((adj_idx0==3'd3)?8'h4d:8'h44):
                              ((ax0<12'd36)?((adj_idx0==3'd3)?8'h49:8'h4f):((adj_idx0==3'd3)?8'h4e:8'h4d)));
                    next_x=(ax0<12'd12)?ax0:((ax0<12'd24)?(ax0-12'd12):((ax0<12'd36)?(ax0-12'd24):(ax0-12'd36)));
                    next_kind=NATIVE;
                    next_valid=(next_x<5'd12);
                end else if(adj_idx0==3'd2) begin
                    next_valid=1'b0;               // gain item: value only, no label
                end else begin
                    next_char=(ax0<12'd24)?((adj_idx0==3'd0)?8'h9e:8'h9c):((adj_idx0==3'd0)?8'h9f:8'h9d);
                    next_x=(ax0<12'd24)?ax0:(ax0-12'd24);
                    next_kind=CHINESE;
                    next_valid=(next_x<5'd24 && next_y<5'd24);
                end
            end else if(!show_temp0 && ax0>=12'd52 && ax0<12'd68 && ay0<12'd24) begin
                next_char=(adj_idx0==3'd5)?(adj_negative0?8'h2d:8'h2b):8'd0;
                next_x=ax0-12'd52;next_y=ay0;
                next_kind=NATIVE;next_valid=(next_char!=0);
            end else if(!show_temp0 && ax0>=12'd72 && ax0<12'd117) begin
                // Preserve the legacy four-bit subtraction/truncation, including
                // gap pixels at large magnitudes. Timing work is not a UI redesign.
                if(ax0<12'd87) begin adj_dx=ax0-12'd72;next_adj_index=2;end
                else if(ax0<12'd102) begin adj_dx=ax0-12'd87;next_adj_index=1;end
                else begin adj_dx=ax0-12'd102;next_adj_index=0;end
                next_x={1'b0,adj_dx};next_y=ay0;next_kind=DIGIT;
                next_adj_digit=1;
                next_valid=(adj_dx<4'd15 && next_y<5'd24) &&
                    ((next_adj_index==2)?(adj_magnitude0>=9'd100):
                     (next_adj_index==1)?(adj_magnitude0>=9'd10):1'b1);
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            char1<=0;gx1<=0;gy1<=0;kind1<=NATIVE;valid1<=0;
            adj_digit1<=0;adj_digit_index1<=0;hundreds1<=0;remainder1<=0;
        end else begin
            char1<=next_char;gx1<=next_x;gy1<=next_y;kind1<=next_kind;
            valid1<=next_valid;adj_digit1<=next_adj_digit;adj_digit_index1<=next_adj_index;
            if(adj_magnitude0>=9'd200) begin hundreds1<=2;remainder1<=adj_magnitude0-9'd200;end
            else if(adj_magnitude0>=9'd100) begin hundreds1<=1;remainder1<=adj_magnitude0-9'd100;end
            else begin hundreds1<=0;remainder1<=adj_magnitude0;end
        end
    end

    // S2: exact small-domain mappings, no division/modulo arithmetic.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin char2<=0;row2<=0;col2<=0;valid2<=0;end
        else begin
            if(adj_digit1) begin
                case(adj_digit_index1)
                    2'd2:char2<=8'h30+hundreds1;
                    2'd1:char2<=8'h30+decimal_pair[7:4];
                    default:char2<=8'h30+decimal_pair[3:0];
                endcase
            end else char2<=char1;
            valid2<=valid1;
            case(kind1)
                CHINESE:begin col2<=gx1[4:0];row2<=gy1[4:0];end
                COLON:begin col2<=gx1[4:0];row2<=gy1[4:0];end
                // For 0 <= x < 15, floor(16*x/15) == x exactly.
                DIGIT:begin col2<=gx1[4:0];row2<=gy1[4:0];end
                default:begin col2<=gx1[4:0];row2<=gy1[4:0];end
            endcase
        end
    end
    // S3: font lookup. S4: column select. They must not be collapsed together.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin col3<=0;valid3<=0;hit4<=0;end
        else begin
            col3<=col2;valid3<=valid2;
            hit4<=valid3 && row_bits3[5'd23-col3];
        end
    end
    // S5: same Logo > glyph > background priority; all metadata is aligned.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin O_de<=0;O_vsync<=0;O_hsync<=0;O_rgb<=0;end
        else begin
            O_vsync<=ctl_q[4][2];O_hsync<=ctl_q[4][1];O_de<=ctl_q[4][0];
            if(!ctl_q[4][0]) O_rgb<=0;
            else if(DEBUG_MODE==1) begin
                if(x_q[4]<12'd128) O_rgb<=24'hff0000;
                else if(x_q[4]<12'd256) O_rgb<=24'h00ff00;
                else if(x_q[4]<12'd384) O_rgb<=24'h0000ff;
                else if(x_q[4]<12'd512) O_rgb<=24'hffff00;
                else if(x_q[4]<12'd640) O_rgb<=24'h00ffff;
                else if(x_q[4]<12'd768) O_rgb<=24'hff00ff;
                else if(x_q[4]<12'd896) O_rgb<=24'hffffff;
                else O_rgb<=24'h202020;
            end else if(DEBUG_MODE==2) begin
                case(debug_q[11:8])
                    4'b0000:O_rgb<=0;
                    4'b0001:O_rgb<=24'hff0000;
                    4'b0011:O_rgb<=24'hffff00;
                    4'b0111:O_rgb<=24'h00ff00;
                    4'b1111:O_rgb<=rgb_q[4];
                    default:O_rgb<=24'h0000ff;
                endcase
            end else if(DEBUG_MODE==3) begin
                if(x_q[4]<12'd256) O_rgb<=debug_q[8]?24'hff8000:24'h201000;
                else if(x_q[4]<12'd512) O_rgb<=debug_q[9]?24'hffff00:24'h202000;
                else if(x_q[4]<12'd768) O_rgb<=debug_q[10]?24'hff0000:24'h200000;
                else O_rgb<=debug_q[11]?24'h00ffff:24'h002020;
            end else if(osd_q[4] && logo_valid_q[4] && logo_pixel[24])
                O_rgb<=logo_pixel[23:0];
            else if(osd_q[4] && hit4)
                // 模式标签按检测颜色上色（红/蓝/绿检测与其填充同名同色），
                // 其余右上文字与数值统一橙黄。
                O_rgb<=lab_sel_q[4]?
                        ((lab_col_q[9:8]==2'd1)?LABEL_RED:
                         (lab_col_q[9:8]==2'd2)?LABEL_BLUE:
                         (lab_col_q[9:8]==2'd3)?LABEL_GREEN:LABEL_AMBER):
                        (top_band_q[4]?TEXT_COLOR_TOP:(bot_band_q[4]?TEXT_COLOR_BOT:TEXT_COLOR));
            else if(osd_q[4] && fps_dot_q[4]) O_rgb<=TEXT_COLOR_TOP;
            else if(osd_q[4] && lat_dot_q[4]) O_rgb<=TEXT_COLOR_TOP;
            else if(osd_q[4] && dhit4)
                O_rgb<=guard_row_q[4]?
                        ((guard_col_q[9:8]==2'd1)?24'hffff00:
                         (guard_col_q[9:8]==2'd2)?24'hff0000:
                         (guard_col_q[9:8]==2'd3)?24'hff00ff:24'h00ffff):
                        TEXT_COLOR;
            else if(osd_q[4] && edge4) O_rgb<=BOX_COLOR;
            else if(osd_q[4] && guard_hit_q[4]) O_rgb<=guard_rgb;
            else O_rgb<=rgb_q[4];
        end
    end
endmodule
