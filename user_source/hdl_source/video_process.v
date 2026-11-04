// Nine-clock, one-pixel/clock pipeline. All modes have identical geometry.
// Sobel/emboss use a causal 3x3 window, never taps from the previous row end.
module video_process #(parameter IMG_WIDTH=1280, parameter EDGE_TH=160,
 parameter IMG_HEIGHT=720,
 parameter integer EXTERNAL_COLOUR_MASK=0,
 parameter integer DISPLAY_ENHANCE=0, // opt-in picture-only grade; no mask/history changes
 parameter integer TARGET_CHANNEL=0, // 0=red, 1=green, 2=blue
 parameter [23:0] FILL_RGB=24'hffff00, OUTLINE_RGB=24'h000000,
 parameter [23:0] BLUE_FILL_RGB=24'hff8000, GREEN_FILL_RGB=24'hff00ff)(
 input wire I_clk,I_rst_n,I_de,
 input wire [23:0] I_data,
 input wire [11:0] I_x,I_y,
 input wire I_vsync,I_hsync,I_user,I_last,I_invert_en,I_emboss_en,
 input wire [3:0] I_mode,
 input wire [4:0] I_sat_level,
 input wire signed [8:0] I_bright_level,
 input wire signed [6:0] I_wb_trim,     // manual white-balance trim, +1/128 per step
 input wire I_wb_trim_en,               // 1 = apply the trim on top of the automatic AWB
 input wire [7:0] I_r_min,I_r_dom,I_g_min,I_g_dom,I_b_min,I_b_dom,
 // Shared mask arrives two clocks after I_data (aligned with ctl[1]).
 input wire I_colour_mask,
 output wire O_de,
 output reg [23:0] O_data,
 output wire [11:0] O_x,O_y,
 output wire O_vsync,O_hsync,O_user,O_last
);
 reg [4:0] ctl[0:8]; // de/vsync/hsync/user/last
 reg [11:0] px[0:8],py[0:8];
 reg [3:0] mode[0:8];
 integer j;
 // Manual white-balance trim: a relative R/B gain around whatever the automatic AWB
 // produced (the user reports the default automatic result looks normal), so only a nudge
 // is needed. +trim is warmer (R up, B down), one step = 1/128 ~ 0.78%. Applied to the
 // colour pixel only, with NO extra pipeline stage, so OSD latency alignment is untouched.
 function [23:0] wb_trim_pix;
  input [23:0] p; input signed [6:0] t; input en;
  reg signed [17:0] rr, bb;
  begin
   if (!en || (t == 7'sd0)) wb_trim_pix = p;
   else begin
    rr = $signed({1'b0,p[23:16]}) + (($signed({1'b0,p[23:16]}) * t) >>> 7);
    bb = $signed({1'b0,p[7:0]})   - (($signed({1'b0,p[7:0]})   * t) >>> 7);
    wb_trim_pix = { (rr > 18'sd255) ? 8'd255 : ((rr < 18'sd0) ? 8'd0 : rr[7:0]),
                    p[15:8],
                    (bb > 18'sd255) ? 8'd255 : ((bb < 18'sd0) ? 8'd0 : bb[7:0]) };
   end
  end
 endfunction
 wire [23:0] c7t = wb_trim_pix(display_c7, I_wb_trim, I_wb_trim_en);
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   for(j=0;j<9;j=j+1) begin ctl[j]<=0;px[j]<=0;py[j]<=0;mode[j]<=0;end
  end else begin
   ctl[0]<={I_de,I_vsync,I_hsync,I_user,I_last}; px[0]<=I_x;py[0]<=I_y;
   mode[0]<=I_mode;
   for(j=1;j<9;j=j+1) begin
    ctl[j]<=ctl[j-1];px[j]<=px[j-1];py[j]<=py[j-1];mode[j]<=mode[j-1];
   end
  end
 end
 assign {O_de,O_vsync,O_hsync,O_user,O_last}=ctl[8];
 assign O_x=px[8];assign O_y=py[8];
 function [7:0] clamp;
  input signed [15:0] n;
  begin clamp=(n<0)?8'd0:((n>255)?8'd255:n[7:0]);end
 endfunction
 function [23:0] posterize;
  input [23:0] c;
  begin posterize={{4{c[23:22]}},{4{c[15:14]}},{4{c[7:6]}}};end
 endfunction
 reg [23:0] c0,c1,c2,c4,c5,c6,c7;
 // Display only. The original c0..c7/luma/line memories/masks remain exact.
 // Reuse the existing three clock intervals c4->c7; no video/tag delay added.
 wire [23:0] display_c7;
 generate if(DISPLAY_ENHANCE)begin: display_grade
  function [23:0] green_balance;
   input [23:0] p;reg[8:0]rb_sum;reg[7:0]mean_rb,excess,green;
   begin
    rb_sum={1'b0,p[23:16]}+{1'b0,p[7:0]}+9'd1;mean_rb=rb_sum[8:1];
    excess=p[15:8]>mean_rb?p[15:8]-mean_rb:8'd0;
    green=p[15:8]-{2'b0,excess[7:2]};
    green_balance={p[23:16],green,p[7:0]};
   end
  endfunction
  function [7:0] lift_mid;
   input[7:0]v;reg[4:0]delta;reg[8:0]result;
   begin
    delta=v[7]?~v[6:2]:v[6:2];
    result={1'b0,v}+{4'b0,delta};lift_mid=result[7:0];
   end
  endfunction
  reg[23:0]balance5,tone6,picture7;
  always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin balance5<=0;tone6<=0;picture7<=0;end
   else begin
    balance5<=green_balance(c4);
    tone6<={lift_mid(balance5[23:16]),lift_mid(balance5[15:8]),lift_mid(balance5[7:0])};
    picture7<=tone6;
   end
  end
  // Keep all intentional effects and mode15 LSC A/B bypass bit-exact.
  wire picture_mode=mode[7]==0||mode[7]==5||mode[7]==6||mode[7]==7||
   mode[7]==8||mode[7]==9||mode[7]==10||mode[7]==11||mode[7]==12||mode[7]==13;
  assign display_c7=picture_mode?picture7:c7;
 end else begin: original_picture
  assign display_c7=c7;
 end endgenerate

 reg signed [8:0] bright0;
 reg [4:0] sat0,sat1,sat2;
 reg [7:0] y2,y3,y4;
 // Classify pre-adjustment RGB so brightness/saturation keys cannot erase
 // the selected target. Only mask bits are buffered, not another RGB frame.
 wire [7:0] legacy_component=(TARGET_CHANNEL==1)?I_data[15:8]:
                            ((TARGET_CHANNEL==2)?I_data[7:0]:I_data[23:16]);
 wire [7:0] legacy_other_a=(TARGET_CHANNEL==0)?I_data[15:8]:I_data[23:16];
 wire [7:0] legacy_other_b=(TARGET_CHANNEL==2)?I_data[15:8]:I_data[7:0];
 // One selected mask for all fill modes. Stage 0 selects raw RGB/thresholds;
 // stage 1 forms nine-bit sums; stage 2 compares. This reuses the existing
 // five clocks before the mask RAM, leaving RGB/control latency unchanged.
 reg [7:0] target_component0,target_other_a0,target_other_b0,target_min0,target_dom0;
 reg [7:0] target_component1,target_min1;
 reg [8:0] target_sum_a1,target_sum_b1;
 wire target_hit=(target_component1>=target_min1) &&
  ({1'b0,target_component1}>=target_sum_a1) &&
  ({1'b0,target_component1}>=target_sum_b1);
 reg [2:0] target_delay;
 // Target history shares row1's ninth bit at the identical address/edge.
 reg target_left, target5, left5;
 wire target_above_raw5;
 wire above5=(py[5]==0)?1'b0:target_above_raw5;
 reg target6,target7,outline6,outline7;
 // Causal contour at the current coordinate: entering edges are inside
 // the target; exiting edges are one background pixel outside. No RGB/DE
 // geometry shift, lookahead, extra frame buffer or added pipeline clocks.
 // Clip the right/bottom image border inside the target (no offscreen pixel).
 reg signed [14:0] pr3,pg3,pb3;
 reg [7:0] ta,tb,ma,mb,ba,bb,bc;
 // Keep the RAM output registers reset-free for block-RAM inference. The
 // single resettable valid bit reproduces the former tc/mc asynchronous
 // reset and hold behavior, including reset in the middle of a row.
 reg [8:0] row1_raw;
 reg [7:0] row2_raw,forward_data;
 reg forward_valid;
 wire [7:0] mc_raw=row1_raw[7:0];
 wire [7:0] tc_raw=forward_valid?forward_data:row2_raw;
 assign target_above_raw5=row1_raw[8];
 // This write is a memory transaction, so it completes even across reset.
 // It is NOT asynchronously reset: the old design committed one edge earlier.
 reg delayed_write;
 reg [11:0] delayed_address;
 reg history_valid;
 wire [7:0] tc=history_valid?tc_raw:8'd0;
 wire [7:0] mc=history_valid?mc_raw:8'd0;
 (* ram_style="bram" *) reg [8:0] line1[0:IMG_WIDTH-1];
 (* ram_style="bram" *) reg [7:0] line2[0:IMG_WIDTH-1];
 wire [16:0] lum={1'b0,c1[23:16]}*17'd77 +
                 {1'b0,c1[15:8]}*17'd150 + {1'b0,c1[7:0]}*17'd29;
 wire signed [9:0] dr=$signed({1'b0,c2[23:16]})-$signed({1'b0,y2});
 wire signed [9:0] dg=$signed({1'b0,c2[15:8]})-$signed({1'b0,y2});
 wire signed [9:0] db=$signed({1'b0,c2[7:0]})-$signed({1'b0,y2});
 wire signed [15:0] sr=$signed({1'b0,y3})+(pr3 >>> 4);
 wire signed [15:0] sg=$signed({1'b0,y3})+(pg3 >>> 4);
 wire signed [15:0] sb=$signed({1'b0,y3})+(pb3 >>> 4);
 // Two native row memories: row1 packs grayscale and target history.
 // Commit row2 on the following edge through its independent write port;
 // forward a matching pending write to reproduce original read-first values.
 always @(posedge I_clk) begin
  delayed_write<=ctl[4][4] && px[4]<IMG_WIDTH;
  if(delayed_write)line2[delayed_address]<=row1_raw[7:0];
  if(ctl[4][4] && px[4]<IMG_WIDTH) begin
   line1[px[4]]<={target_delay[2],y4};
   row1_raw<=line1[px[4]];
   row2_raw<=line2[px[4]];
   delayed_address<=px[4];
   forward_valid<=delayed_write && delayed_address==px[4];
   forward_data<=row1_raw[7:0];
  end
 end
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n)history_valid<=0;
  else if(ctl[4][4] && px[4]<IMG_WIDTH)history_valid<=1;
 end
 reg signed [11:0] gx6,gy6,emb6;
 reg valid6,valid7;
 reg [11:0] mag7;
 reg signed [12:0] emb7;
 wire signed [11:0] gx=
  $signed({4'b0,tc})-$signed({4'b0,ta})+
  ($signed({4'b0,mc})<<<1)-($signed({4'b0,ma})<<<1)+
  $signed({4'b0,bc})-$signed({4'b0,ba});
 wire signed [11:0] gy=
  $signed({4'b0,ta})+($signed({4'b0,tb})<<<1)+$signed({4'b0,tc})-
  $signed({4'b0,ba})-($signed({4'b0,bb})<<<1)-$signed({4'b0,bc});
 wire signed [11:0] emboss=
  $signed({4'b0,mc})+$signed({4'b0,bb})+($signed({4'b0,bc})<<<1)-
  $signed({4'b0,tb})-$signed({4'b0,ma})-($signed({4'b0,ta})<<<1);
 wire [11:0] agx=gx6[11] ? -gx6 : gx6;
 wire [11:0] agy=gy6[11] ? -gy6 : gy6;
 wire edge_hit=valid7 && mag7>=((mode[7]==4'd1)?12'd80:EDGE_TH);
 wire [7:0] emb_out=valid7 ? clamp($signed(emb7)+16'sd128) : 8'd128;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   c0<=0;c1<=0;c2<=0;c4<=0;c5<=0;c6<=0;c7<=0;bright0<=0;
   sat0<=16;sat1<=16;sat2<=16;y2<=0;y3<=0;y4<=0;
   pr3<=0;pg3<=0;pb3<=0;
   target_delay<=0;target_left<=0;target5<=0;left5<=0;
   target_component0<=0;target_other_a0<=0;target_other_b0<=0;target_min0<=0;target_dom0<=0;
   target_component1<=0;target_min1<=0;target_sum_a1<=0;target_sum_b1<=0;
   target6<=0;target7<=0;outline6<=0;outline7<=0;
   ta<=0;tb<=0;ma<=0;mb<=0;ba<=0;bb<=0;bc<=0;
   gx6<=0;gy6<=0;emb6<=0;valid6<=0;valid7<=0;mag7<=0;emb7<=0;O_data<=0;
  end else begin
   c0<=I_data;bright0<=I_bright_level;sat0<=I_sat_level;
   case(I_mode)
    4'd7:begin // 0111 blue fill; 1000 is reserved for future blue detection.
     target_component0<=I_data[7:0];target_other_a0<=I_data[23:16];target_other_b0<=I_data[15:8];
     target_min0<=I_b_min;target_dom0<=I_b_dom;
    end
    4'd9:begin // 1001 green fill; 1010 is reserved for future green detection.
     target_component0<=I_data[15:8];target_other_a0<=I_data[23:16];target_other_b0<=I_data[7:0];
     target_min0<=I_g_min;target_dom0<=I_g_dom;
    end
    default:begin // Preserve the legacy mode-5 parameter interface.
     target_component0<=legacy_component;target_other_a0<=legacy_other_a;target_other_b0<=legacy_other_b;
     target_min0<=I_r_min;target_dom0<=I_r_dom;
    end
   endcase
   target_component1<=target_component0;target_min1<=target_min0;
   target_sum_a1<={1'b0,target_other_a0}+{1'b0,target_dom0};
   target_sum_b1<={1'b0,target_other_b0}+{1'b0,target_dom0};
   target_delay<={target_delay[1:0],ctl[1][4] && (EXTERNAL_COLOUR_MASK ? I_colour_mask : target_hit)};
   c1<={clamp($signed({1'b0,c0[23:16]})+bright0),
        clamp($signed({1'b0,c0[15:8]})+bright0),
        clamp($signed({1'b0,c0[7:0]})+bright0)};
   sat1<=sat0;
   c2<=c1;y2<=lum[15:8];sat2<=sat1;
   pr3<=dr*$signed({1'b0,sat2});
   pg3<=dg*$signed({1'b0,sat2});
   pb3<=db*$signed({1'b0,sat2});y3<=y2;
   c4<={clamp(sr),clamp(sg),clamp(sb)};y4<=y3;
   c5<=c4;
   if(ctl[4][4] && px[4]<IMG_WIDTH) begin
    target5<=target_delay[2];
    left5<=(px[4]==0)?1'b0:target_left;
    target_left<=target_delay[2];
    bc<=y4;
    if(px[4]==0) begin ta<=0;tb<=0;ma<=0;mb<=0;ba<=0;bb<=0; end
    else begin ta<=tb;tb<=tc;ma<=mb;mb<=mc;ba<=bb;bb<=bc;end
   end
   c6<=c5;gx6<=gx;gy6<=gy;emb6<=emboss;
   target6<=target5;
   outline6<=ctl[5][4] && ((target5!=left5) || (target5!=above5) ||
    (target5 && ((px[5]==IMG_WIDTH-1) || (py[5]==IMG_HEIGHT-1))));
   valid6<=ctl[5][4] && px[5]>=2 && py[5]>=2;
   c7<=c6;mag7<=agx+agy;emb7<=$signed(emb6)*13'sd2;valid7<=valid6;
   target7<=target6;outline7<=outline6;
   if(!ctl[7][4]) O_data<=0;
   else case(mode[7])
    4'd1:O_data<=edge_hit ? 24'hffffff : 24'h000000;       // 0001 edge (binary)
    4'd2:O_data<=~c7;                                      // 0010 invert
    4'd3:O_data<={emb_out,emb_out,emb_out};                // 0011 emboss
    4'd4:O_data<=edge_hit ? 24'h000000 : posterize(c7t);    // 0100 cartoon
    4'd5:O_data<=outline7 ? OUTLINE_RGB : (target7 ? FILL_RGB : c7t); // 0101 target fill
    4'd7:O_data<=outline7 ? OUTLINE_RGB : (target7 ? BLUE_FILL_RGB : c7t); // 0111 blue/orange
    4'd9:O_data<=outline7 ? OUTLINE_RGB : (target7 ? GREEN_FILL_RGB : c7t); // 1001 green/magenta
    default:O_data<=c7t;                                    // 0000 & reserved
   endcase
  end
 end
endmodule
