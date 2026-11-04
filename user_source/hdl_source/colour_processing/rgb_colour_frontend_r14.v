// One shared PRE-reader mask for RGB fill and detection. No video backpressure.
// Causal 3x3 RGB mean (replicated top/left borders) -> exact hue/chroma ->
// causal 2x2 majority (>=3, replicated top/left borders). Seven registers.
// Tag (x,y) is the bottom-right/current pixel, NOT a centred-window coordinate.
// Output support and boxes can move by a few pixels; no row-end wrap or flush.
module rgb_colour_frontend_r14 #(parameter integer WIDTH=1280,HEIGHT=720,BLUE_WEAK_ENABLE=1,GREEN_WEAK_ENABLE=1,USE_EXTERNAL_FIELD=0,parameter GAIN_FILE="gain_rg_1280_720.mem")(
 input wire[17:0]I_field_word,
 input wire I_clk,I_rst_n,I_de,I_user,I_last,I_good,
 input wire [23:0] I_rgb,
 input wire [31:0] I_source_id,
 input wire [11:0] I_x,I_y,
 input wire [3:0] I_mode,
 input wire [7:0] I_r_min,I_r_sat,I_g_min,I_g_sat,I_b_min,I_b_sat,
 output wire O_de,O_good,O_mask,
 output wire [11:0] O_x,O_y,
 // Five-clock context matches the existing LSC packet, two clocks BEFORE mask.
 output wire [3:0] O_frame_mode,
 output wire [7:0] O_frame_min,O_frame_sat
);
 wire accept,frame_start,frame_end,bad_event,frame_bad,config_bad;
 reg [31:0] frame_source;
 reg source_bad;
 wire source_mismatch=I_de&&!I_user&&(I_source_id!=frame_source);
 wire source_good=I_good&&!source_mismatch&&(!source_bad||I_user);
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin frame_source<=0;source_bad<=0;end
  else if(I_de&&I_user)begin frame_source<=I_source_id;source_bad<=0;end
  else if(source_mismatch)source_bad<=1;
 end
 red_frame_guard #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) guard(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(1'b1),.I_de(I_de),.I_user(I_user),.I_last(I_last),
  .I_pixel_good(source_good),.I_x(I_x),.I_y(I_y),.O_accept(accept),.O_frame_start(frame_start),
  .O_frame_end(frame_end),.O_bad_event(bad_event),.O_frame_bad(frame_bad),.O_config_bad(config_bad));
 reg [19:0] frame_config;
 reg [19:0] config_pipe[0:4];
 wire [7:0] selected_min=(I_mode==7||I_mode==8)?I_b_min:(I_mode==9||I_mode==10)?I_g_min:I_r_min;
 wire [7:0] selected_sat=(I_mode==7||I_mode==8)?I_b_sat:(I_mode==9||I_mode==10)?I_g_sat:I_r_sat;
 wire [19:0] effective_config=(I_de&&I_user)?{I_mode,selected_min,selected_sat}:frame_config;
 integer k;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   frame_config<={4'd0,8'd24,8'd104};
   for(k=0;k<5;k=k+1)config_pipe[k]<=0;
  end else begin
   if(I_de&&I_user)frame_config<=effective_config;
   config_pipe[0]<=effective_config;
   for(k=1;k<5;k=k+1)config_pipe[k]<=config_pipe[k-1];
  end
 end
 assign {O_frame_mode,O_frame_min,O_frame_sat}=config_pipe[4];
 wire wd,wu,wl,wg;
 wire [3:0] cm=config_pipe[1][19:16];
 wire [11:0] r4,g4,b4;wire good4;
 wire [11:0] wx,wy,wr,wg_sum,wb;
 generate if(BLUE_WEAK_ENABLE==1||GREEN_WEAK_ENABLE==1)begin:blue_window
 rgb_causal_window34_r11 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(I_de),.I_user(I_user),.I_last(I_last),.I_good(accept),
  .I_x(I_x),.I_y(I_y),.I_rgb(I_rgb),.O_de(wd),.O_user(wu),.O_last(wl),.O_good(wg),
  .O_x(wx),.O_y(wy),.O_r_sum(wr),.O_g_sum(wg_sum),.O_b_sum(wb),.O_good4(good4),.O_r_sum4(r4),.O_g_sum4(g4),.O_b_sum4(b4));
 end else begin:legacy_window
 rgb_causal_window3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(I_de),.I_user(I_user),.I_last(I_last),.I_good(accept),
  .I_x(I_x),.I_y(I_y),.I_rgb(I_rgb),.O_de(wd),.O_user(wu),.O_last(wl),.O_good(wg),
  .O_x(wx),.O_y(wy),.O_r_sum(wr),.O_g_sum(wg_sum),.O_b_sum(wb));
 assign r4=0;assign g4=0;assign b4=0;assign good4=0;
 end endgenerate
 wire [1:0] colour=(cm==5||cm==6)?2'd0:(cm==9||cm==10)?2'd1:(cm==7||cm==8)?2'd2:2'd3;
 wire cd,cu,cl,cg,hit,cbad;
 wire [11:0] cx,cy;
 rgb_chroma_core core(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(wd),.I_user(wu),.I_last(wl),.I_good(wg),
  .I_x(wx),.I_y(wy),.I_r_sum(wr),.I_g_sum(wg_sum),.I_b_sum(wb),.I_colour(colour),
  .I_min(config_pipe[1][15:8]),.I_saturation(config_pipe[1][7:0]),
  .O_de(cd),.O_user(cu),.O_last(cl),.O_good(cg),.O_match(hit),.O_config_bad(cbad),.O_x(cx),.O_y(cy));
 wire strong_mask,blue_mask,green_mask;
 rgb_causal_majority2 #(.WIDTH(WIDTH)) clean(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(cd),.I_user(cu),.I_last(cl),.I_good(cg),.I_mask(hit),
  .I_x(cx),.I_y(cy),.O_de(O_de),.O_good(O_good),.O_mask(strong_mask),.O_x(O_x),.O_y(O_y));
 // Keep the original majority output and good path. The 16-sample
 // complement is ORed AFTER that majority, never before it.
 generate if(BLUE_WEAK_ENABLE==1)begin:blue_complement
  wire bv,bg,bm;wire[11:0]bx,by;
  blue_raw_core_r04 weak_core(.I_clk(I_clk),.I_rst_n(I_rst_n),
   .I_valid(wd),.I_user(wu),.I_last(wl),.I_good(good4),.I_enable(cm==7||cm==8),
   .I_sum_r(r4),.I_sum_g(g4),.I_sum_b(b4),.I_min(config_pipe[1][15:8]),.I_dom(config_pipe[1][7:0]),
   .I_x(wx),.I_y(wy),.I_source_id(32'd0),.O_valid(bv),.O_user(),.O_last(),.O_good(bg),.O_weak(bm),
   .O_x(bx),.O_y(by),.O_source_id());
  // Source validity is inherited from the original sticky frame/source
  // guard at input; all sixteen window quality bits must be accepted.
  reg[1:0]valid_q,good_q,mask_q;reg[11:0]xq[0:1],yq[0:1];
  always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin valid_q<=0;good_q<=0;mask_q<=0;xq[0]<=0;xq[1]<=0;yq[0]<=0;yq[1]<=0;end
   else begin
    valid_q<={valid_q[0],bv};good_q<={good_q[0],bg};mask_q<={mask_q[0],bm};
    xq[0]<=bx;xq[1]<=xq[0];yq[0]<=by;yq[1]<=yq[0];
   end
  end
  assign blue_mask=strong_mask||(O_de&&O_good&&valid_q[1]&&good_q[1]&&mask_q[1]&&O_x==xq[1]&&O_y==yq[1]);
 end else begin:no_complement
  assign blue_mask=strong_mask;
 end endgenerate
 generate if(GREEN_WEAK_ENABLE==1)begin:green_complement
  wire[8:0]gr,gg;wire coeff_good;
  green_gain_rom_r14 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.USE_EXTERNAL_WORD(USE_EXTERNAL_FIELD),.MEM_FILE(GAIN_FILE)) field(
   .I_word_early(I_field_word),
   .I_clk(I_clk),.I_rst_n(I_rst_n),.I_x(I_x),.I_y(I_y),.O_gain_r(gr),.O_gain_g(gg),.O_good(coeff_good));
  wire gv,gq,gm;wire[11:0]gx,gy;
  green_weak_core_r10 core(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_valid(wd),.I_user(wu),.I_last(wl),
   .I_good(good4&&coeff_good),.I_enable(cm==9||cm==10),.I_sum_r(r4),.I_sum_g(g4),.I_sum_b(b4),
   .I_min(config_pipe[1][15:8]),.I_dom(config_pipe[1][7:0]),.I_gain_r(gr),.I_gain_g(gg),
   .I_x(wx),.I_y(wy),.I_source_id(32'd0),.O_valid(gv),.O_user(),.O_last(),.O_good(gq),.O_weak(gm),
   .O_x(gx),.O_y(gy),.O_source_id());
  reg[1:0]vq,gqq,mq;reg[11:0]xx[0:1],yy[0:1];
  always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin vq<=0;gqq<=0;mq<=0;xx[0]<=0;xx[1]<=0;yy[0]<=0;yy[1]<=0;end
   else begin vq<={vq[0],gv};gqq<={gqq[0],gq};mq<={mq[0],gm};xx[0]<=gx;xx[1]<=xx[0];yy[0]<=gy;yy[1]<=yy[0];end
  end
  assign green_mask=O_de&&O_good&&vq[1]&&gqq[1]&&mq[1]&&O_x==xx[1]&&O_y==yy[1];
 end else begin:no_green
  assign green_mask=1'b0;
 end endgenerate
 assign O_mask=blue_mask||green_mask;

endmodule


// Shared two-clock 3x3 / 4x4 causal statistics.
module rgb_causal_window34_r11 #(parameter integer WIDTH=1280,HEIGHT=720)(
 input wire I_clk,I_rst_n,I_de,I_user,I_last,I_good,
 input wire [11:0] I_x,I_y,input wire [23:0] I_rgb,
 output reg O_de,O_user,O_last,O_good,
 output reg [11:0] O_x,O_y,O_r_sum,O_g_sum,O_b_sum,
 output reg O_good4,output reg[11:0]O_r_sum4,O_g_sum4,O_b_sum4
);
 (* ram_style="bram" *) reg [24:0] row1[0:WIDTH-1];
 (* ram_style="bram" *) reg [24:0] row2[0:WIDTH-1];
 (* ram_style="bram" *) reg[24:0] row3[0:WIDTH-1];
 reg[24:0]oldest_raw,oldest_forward_data;
 reg [24:0] above_raw,older_raw;
 reg older_forward;
 reg [24:0] older_forward_data;
 reg d0,u0,l0,g0;
 reg [11:0] x0,y0;
 reg [23:0] rgb0;
 wire [24:0] current={g0,rgb0};
 wire [24:0] above=y0==0?current:above_raw;
 wire [24:0] older=y0==0?current:y0==1?above_raw:older_forward?older_forward_data:older_raw;
 wire[24:0]oldest=(y0<3)?older:older_forward?oldest_forward_data:oldest_raw;
 wire [9:0] col_r={2'd0,current[23:16]}+{2'd0,above[23:16]}+{2'd0,older[23:16]};
 wire [9:0] col_g={2'd0,current[15:8]}+{2'd0,above[15:8]}+{2'd0,older[15:8]};
 wire [9:0] col_b={2'd0,current[7:0]}+{2'd0,above[7:0]}+{2'd0,older[7:0]};
 wire col_good=current[24]&&above[24]&&older[24];
 reg [9:0] left_r,left_g,left_b,left2_r,left2_g,left2_b;
 reg left_good,left2_good;
 wire [9:0] r1=x0==0?col_r:left_r,r2=x0==0?col_r:x0==1?left_r:left2_r;
 wire [9:0] g1=x0==0?col_g:left_g,g2=x0==0?col_g:x0==1?left_g:left2_g;
 wire [9:0] b1=x0==0?col_b:left_b,b2=x0==0?col_b:x0==1?left_b:left2_b;
 wire g1_ok=x0==0?col_good:left_good,g2_ok=x0==0?col_good:x0==1?left_good:left2_good;
 always @(posedge I_clk)begin
  if(I_rst_n&&I_de&&I_x<WIDTH&&I_y<HEIGHT)begin
   row1[I_x]<={I_good,I_rgb};
   above_raw<=row1[I_x];older_raw<=row2[I_x];oldest_raw<=row3[I_x];
  end
  // Write returned row1 data one clock later through row2's independent
  // write port. A same-cycle unregistered row1 -> row2 copy inferred LUT RAM.
  if(I_rst_n&&d0&&x0<WIDTH&&y0<HEIGHT)begin
   row2[x0]<=above_raw;row3[x0]<=older;
  end
 end
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   d0<=0;u0<=0;l0<=0;g0<=0;x0<=0;y0<=0;rgb0<=0;older_forward<=0;older_forward_data<=0;
   O_de<=0;O_user<=0;O_last<=0;O_good<=0;O_x<=0;O_y<=0;O_r_sum<=0;O_g_sum<=0;O_b_sum<=0;
   left_r<=0;left_g<=0;left_b<=0;left2_r<=0;left2_g<=0;left2_b<=0;left_good<=0;left2_good<=0;
  end else begin
   d0<=I_de;u0<=I_user;l0<=I_last;g0<=I_good&&I_x<WIDTH&&I_y<HEIGHT;
   x0<=I_x;y0<=I_y;rgb0<=I_rgb;
   // WIDTH=1 with adjacent rows has a read/write collision on row2.
   // Forward the new write explicitly; wider rasters use the RAM read.
   older_forward<=d0&&I_de&&x0==I_x&&x0<WIDTH&&y0<HEIGHT;
   older_forward_data<=above_raw;
   O_de<=d0;O_user<=u0;O_last<=l0;O_x<=x0;O_y<=y0;
   O_good<=d0&&col_good&&g1_ok&&g2_ok;
   O_r_sum<={2'd0,col_r}+{2'd0,r1}+{2'd0,r2};
   O_g_sum<={2'd0,col_g}+{2'd0,g1}+{2'd0,g2};
   O_b_sum<={2'd0,col_b}+{2'd0,b1}+{2'd0,b2};
   if(d0)begin
    left_r<=col_r;left_g<=col_g;left_b<=col_b;left_good<=col_good;
    left2_r<=left_r;left2_g<=left_g;left2_b<=left_b;left2_good<=left_good;
   end
  end
 end
 // The third row stores the effective (including WIDTH=1 forwarding)
 // second-oldest sample, so adjacent one-pixel rows cannot copy stale RAM.
 wire[9:0]c4r=col_r+{2'd0,oldest[23:16]},c4g=col_g+{2'd0,oldest[15:8]},c4b=col_b+{2'd0,oldest[7:0]};
 wire c4good=col_good&&oldest[24];
 reg[9:0]hr1,hr2,hr3,hg1,hg2,hg3,hb1,hb2,hb3;
 reg hq1,hq2,hq3;
 wire[9:0]ar1=x0==0?c4r:hr1,ar2=x0==0?c4r:x0==1?hr1:hr2,ar3=x0==0?c4r:x0==1?hr1:x0==2?hr2:hr3;
 wire[9:0]ag1=x0==0?c4g:hg1,ag2=x0==0?c4g:x0==1?hg1:hg2,ag3=x0==0?c4g:x0==1?hg1:x0==2?hg2:hg3;
 wire[9:0]ab1=x0==0?c4b:hb1,ab2=x0==0?c4b:x0==1?hb1:hb2,ab3=x0==0?c4b:x0==1?hb1:x0==2?hb2:hb3;
 wire aq1=x0==0?c4good:hq1,aq2=x0==0?c4good:x0==1?hq1:hq2,aq3=x0==0?c4good:x0==1?hq1:x0==2?hq2:hq3;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   oldest_forward_data<=0;O_good4<=0;O_r_sum4<=0;O_g_sum4<=0;O_b_sum4<=0;
   hr1<=0;hr2<=0;hr3<=0;hg1<=0;hg2<=0;hg3<=0;hb1<=0;hb2<=0;hb3<=0;hq1<=0;hq2<=0;hq3<=0;
  end else begin
   oldest_forward_data<=older;
   O_good4<=d0&&c4good&&aq1&&aq2&&aq3;
   O_r_sum4<={2'd0,c4r}+{2'd0,ar1}+{2'd0,ar2}+{2'd0,ar3};
   O_g_sum4<={2'd0,c4g}+{2'd0,ag1}+{2'd0,ag2}+{2'd0,ag3};
   O_b_sum4<={2'd0,c4b}+{2'd0,ab1}+{2'd0,ab2}+{2'd0,ab3};
   if(d0)begin
    hr1<=c4r;hr2<=hr1;hr3<=hr2;hg1<=c4g;hg2<=hg1;hg3<=hg2;hb1<=c4b;hb2<=hb1;hb3<=hb2;
    hq1<=c4good;hq2<=hq1;hq3<=hq2;
   end
  end
 end
endmodule

