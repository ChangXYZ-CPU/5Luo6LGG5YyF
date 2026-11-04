// One shared PRE-reader mask for RGB fill and detection. No video backpressure.
// Causal 3x3 RGB mean (replicated top/left borders) -> exact hue/chroma ->
// causal 2x2 majority (>=3, replicated top/left borders). Seven registers.
// Tag (x,y) is the bottom-right/current pixel, NOT a centred-window coordinate.
// Output support and boxes can move by a few pixels; no row-end wrap or flush.
module rgb_colour_frontend #(parameter integer WIDTH=1280,HEIGHT=720)(
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
 wire [11:0] wx,wy,wr,wg_sum,wb;
 rgb_causal_window3 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) window(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(I_de),.I_user(I_user),.I_last(I_last),.I_good(accept),
  .I_x(I_x),.I_y(I_y),.I_rgb(I_rgb),.O_de(wd),.O_user(wu),.O_last(wl),.O_good(wg),
  .O_x(wx),.O_y(wy),.O_r_sum(wr),.O_g_sum(wg_sum),.O_b_sum(wb));
 wire [3:0] cm=config_pipe[1][19:16];
 wire [1:0] colour=(cm==5||cm==6)?2'd0:(cm==9||cm==10)?2'd1:(cm==7||cm==8)?2'd2:2'd3;
 wire cd,cu,cl,cg,hit,cbad;
 wire [11:0] cx,cy;
 rgb_chroma_core core(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(wd),.I_user(wu),.I_last(wl),.I_good(wg),
  .I_x(wx),.I_y(wy),.I_r_sum(wr),.I_g_sum(wg_sum),.I_b_sum(wb),.I_colour(colour),
  .I_min(config_pipe[1][15:8]),.I_saturation(config_pipe[1][7:0]),
  .O_de(cd),.O_user(cu),.O_last(cl),.O_good(cg),.O_match(hit),.O_config_bad(cbad),.O_x(cx),.O_y(cy));
 rgb_causal_majority2 #(.WIDTH(WIDTH)) clean(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(cd),.I_user(cu),.I_last(cl),.I_good(cg),.I_mask(hit),
  .I_x(cx),.I_y(cy),.O_de(O_de),.O_good(O_good),.O_mask(O_mask),.O_x(O_x),.O_y(O_y));
endmodule

// Two synchronous read-first RGB+quality row memories; RAM contents unreset.
// Row zero reconstructs all storage before use in a subsequent row/frame.
module rgb_causal_window3 #(parameter integer WIDTH=1280,HEIGHT=720)(
 input wire I_clk,I_rst_n,I_de,I_user,I_last,I_good,
 input wire [11:0] I_x,I_y,input wire [23:0] I_rgb,
 output reg O_de,O_user,O_last,O_good,
 output reg [11:0] O_x,O_y,O_r_sum,O_g_sum,O_b_sum
);
 (* ram_style="bram" *) reg [24:0] row1[0:WIDTH-1];
 (* ram_style="bram" *) reg [24:0] row2[0:WIDTH-1];
 reg [24:0] above_raw,older_raw;
 reg older_forward;
 reg [24:0] older_forward_data;
 reg d0,u0,l0,g0;
 reg [11:0] x0,y0;
 reg [23:0] rgb0;
 wire [24:0] current={g0,rgb0};
 wire [24:0] above=y0==0?current:above_raw;
 wire [24:0] older=y0==0?current:y0==1?above_raw:older_forward?older_forward_data:older_raw;
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
   above_raw<=row1[I_x];older_raw<=row2[I_x];
  end
  // Write returned row1 data one clock later through row2's independent
  // write port. A same-cycle unregistered row1 -> row2 copy inferred LUT RAM.
  if(I_rst_n&&d0&&x0<WIDTH&&y0<HEIGHT)row2[x0]<=above_raw;
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
endmodule

// Two-register synchronous read-first majority. No extra RGB frame buffer.
module rgb_causal_majority2 #(parameter integer WIDTH=1280)(
 input wire I_clk,I_rst_n,I_de,I_user,I_last,I_good,I_mask,
 input wire [11:0] I_x,I_y,
 output reg O_de,O_good,O_mask,output reg [11:0] O_x,O_y
);
 (* ram_style="bram" *) reg previous[0:WIDTH-1];
 reg upper_raw,left,upper_left;
 reg d0,g0,m0,left0;
 reg [11:0] x0,y0;
 wire upper=y0==0?m0:upper_raw;
 wire up_left=y0==0?left0:x0==0?upper:upper_left;
 wire [2:0] votes={2'd0,m0}+{2'd0,left0}+{2'd0,upper}+{2'd0,up_left};
 always @(posedge I_clk)begin
  if(I_rst_n&&I_de&&I_x<WIDTH)begin previous[I_x]<=I_mask;upper_raw<=previous[I_x];end
 end
 // Horizontal upper history must use the RAM read for the preceding pixel,
 // available one edge later, rather than a not-yet-returned current read.
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   left<=0;upper_left<=0;d0<=0;g0<=0;m0<=0;left0<=0;x0<=0;y0<=0;
   O_de<=0;O_good<=0;O_mask<=0;O_x<=0;O_y<=0;
  end else begin
   d0<=I_de;g0<=I_good;m0<=I_mask;x0<=I_x;y0<=I_y;
   if(I_de)begin left0<=I_x==0?I_mask:left;left<=I_mask;end
   if(d0)upper_left<=upper;
   // Combinational upper-left uses the previous accepted read (below).
   O_de<=d0;O_good<=g0;O_mask<=d0&&g0&&votes>=3;O_x<=x0;O_y<=y0;
  end
 end
 // upper_left is updated on the same edge as O_mask, so its old value is
 // exactly the previous accepted column even with arbitrary DE bubbles.
endmodule
