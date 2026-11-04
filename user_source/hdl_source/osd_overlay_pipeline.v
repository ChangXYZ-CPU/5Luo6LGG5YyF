module osd_overlay_pipeline #(
    parameter integer IMG_HEIGHT=720,
    parameter integer DEBUG_MODE=0
)(
    input wire I_clk, I_rst_n,
    input wire I_de, I_vsync, I_hsync,
    input wire [23:0] I_rgb,
    input wire [11:0] I_x, I_y,
    input wire I_osd_en,
    input wire [3:0] I_mode,
    input wire I_adj_sel,
    input wire [4:0] I_sat_level,
    input wire [8:0] I_bright_level,
    input wire [23:0] I_counter_bcd,
    input wire [3:0] I_debug_status,
    output reg O_de, O_vsync, O_hsync,
    output reg [23:0] O_rgb
);
    // Purpose: the existing fixed 1280-wide OSD layout, primarily 720p.
    // S0 geometry/config -> S1 character -> S2 scale -> S3 font row ->
    // S4 bit selection -> S5 overlay. One pixel each clock, no backpressure.
    localparam [11:0] ADJ_Y=IMG_HEIGHT-12'd32;
    localparam [23:0] TEXT_COLOR=24'hfff200;
    localparam [1:0] NATIVE=0, CHINESE=1, COLON=2, DIGIT=3;

    reg [11:0] tx0,ty0,ax0,ay0,lx0,ly0;
    reg [11:0] label_x0;   // 类型标签专用坐标：原点比 TXT_X 再左 8px
    reg text_valid0,adj_valid0;
    reg [3:0] mode0;
    reg adj_sel0,adj_negative0;
    reg [8:0] adj_magnitude0;
    reg [23:0] counter0;

    reg [7:0] char1;
    reg [4:0] gx1,gy1;
    reg [1:0] kind1,adj_digit_index1;
    reg valid1,adj_digit1;
    reg [3:0] hundreds1;
    reg [6:0] remainder1;

    reg [7:0] char2;
    reg [4:0] row2,col2;
    reg valid2;
    wire [23:0] font_row;
    reg [23:0] row_bits3;
    reg [4:0] col3;
    reg valid3,hit4;

    reg [23:0] rgb_q[0:4];
    reg [2:0] ctl_q[0:4];
    reg [11:0] x_q[0:4];
    reg [4:0] osd_q,logo_valid_q;
    reg [11:0] debug_q;
    reg [14:0] logo_addr_q;
    wire [24:0] logo_pixel;
    reg [24:0] logo_pixel3,logo_pixel4;
    integer k;

    reg [7:0] next_char;
    reg [4:0] next_x,next_y;
    reg [1:0] next_kind,next_adj_index;
    reg next_valid,next_adj_digit;
    reg [3:0] adj_dx;
    wire [7:0] decimal_pair=F_decimal_pair(remainder1);

    function [7:0] F_mode_char;
        input [3:0] mode;
        input second;
        begin
            case(mode)
                4'd0:F_mode_char=second?8'h83:8'h82;
                4'd1:F_mode_char=second?8'h85:8'h84;
                4'd2:F_mode_char=second?8'h89:8'h88;
                4'd3:F_mode_char=second?8'h8b:8'h8a;
                4'd4:F_mode_char=second?8'h8d:8'h8c;
                default:F_mode_char=8'd0;
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

    anlogic_logo_rom_sync u_logo(
        .I_clk(I_clk), .I_addr(logo_addr_q), .O_pixel(logo_pixel));
    osd_char_lib u_font(
        .I_char(char2), .I_row(row2), .O_row_bits(font_row));

    // S0: bounded geometry and the configuration sampled for THIS pixel.
    // Brightness is a signed nine-bit two's-complement value: abs <= 256.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            tx0<=0;ty0<=0;ax0<=0;ay0<=0;lx0<=0;ly0<=0;
            text_valid0<=0;adj_valid0<=0;mode0<=0;
            adj_sel0<=0;adj_negative0<=0;adj_magnitude0<=0;counter0<=0;
            osd_q<=0;logo_valid_q<=0;debug_q<=0;
            logo_pixel3<=0;logo_pixel4<=0;
            for(k=0;k<5;k=k+1) begin rgb_q[k]<=0;ctl_q[k]<=0;x_q[k]<=0;end
        end else begin
            tx0<=I_x-12'd1070;ty0<=I_y-12'd8;
            label_x0<=I_x-12'd1062;
            ax0<=I_x-12'd1153;ay0<=I_y-ADJ_Y;
            lx0<=I_x-12'd13;ly0<=I_y+12'd14;
            text_valid0<=I_de && I_x>=12'd1062 && I_x<12'd1270 &&
                                  I_y>=12'd8 && I_y<12'd32;
            adj_valid0<=I_de && I_x>=12'd1153 && I_x<12'd1270 &&
                                 I_y>=ADJ_Y && I_y<(ADJ_Y+12'd24);
            mode0<=I_mode;adj_sel0<=I_adj_sel;
            adj_negative0<=I_bright_level[8];
            adj_magnitude0<=I_adj_sel?{4'd0,I_sat_level}:
                (I_bright_level[8]?(~I_bright_level+9'd1):I_bright_level);
            counter0<=I_counter_bcd;

            rgb_q[0]<=I_rgb;ctl_q[0]<={I_vsync,I_hsync,I_de};x_q[0]<=I_x;
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
            logo_pixel3<=logo_pixel;logo_pixel4<=logo_pixel3;
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
            if(label_x0<12'd48) begin
                next_char=F_mode_char(mode0,label_x0>=12'd24);
                next_x=(label_x0<12'd24)?label_x0[4:0]:(label_x0-12'd24);
                next_y=ty0;
                next_kind=CHINESE;
                next_valid=(next_char!=0 && next_x<5'd24 && next_y<5'd24);
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
        if(adj_valid0) begin
            next_char=0;next_valid=0;next_adj_digit=0;
            if(ax0<12'd48) begin
                next_char=(ax0<12'd24)?(adj_sel0?8'h90:8'h8e):(adj_sel0?8'h91:8'h8f);
                next_x=(ax0<12'd24)?ax0:(ax0-12'd24);
                next_y=ay0;
                next_kind=CHINESE;
                next_valid=(next_x<5'd24 && next_y<5'd24);
            end else if(ax0>=12'd52 && ax0<12'd68 && ay0<12'd24) begin
                next_char=adj_sel0?8'd0:(adj_negative0?8'h2d:8'h2b);
                next_x=ax0-12'd52;next_y=ay0;
                next_kind=NATIVE;next_valid=(next_char!=0);
            end else if(ax0<12'd117) begin
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
        if(!I_rst_n) begin row_bits3<=0;col3<=0;valid3<=0;hit4<=0;end
        else begin
            row_bits3<=font_row;col3<=col2;valid3<=valid2;
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
            end else if(osd_q[4] && logo_valid_q[4] && logo_pixel4[24])
                O_rgb<=logo_pixel4[23:0];
            else if(osd_q[4] && hit4) O_rgb<=TEXT_COLOR;
            else O_rgb<=rgb_q[4];
        end
    end
endmodule
