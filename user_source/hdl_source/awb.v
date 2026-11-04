// Four RGB888 pixels, preserved MSB-first. Frame-wise gray-world AWB.
// Restoring division: 48 clocks, followed by 3 private smoothing clocks.
// Gains commit ONLY at the next frame's first valid beat, never mid-frame.
module awb #(parameter IMG_HEIGHT=1080,parameter IMG_WIDTH=1920)(
 input I_clk,I_rst_n,I_tlast,I_tuser,input [95:0] I_tdata,input I_tvalid,
 output I_tready,output reg O_tlast,O_tuser,output reg [95:0] O_tdata,
 output reg O_tvalid,input O_tready,
 output [12:0] O_wb_temp                   // estimated equivalent temperature (auto path)
);
 localparam FRAME_BEATS=IMG_HEIGHT*(IMG_WIDTH/4);
 assign I_tready=O_tready;
 reg sof_pending;
 wire first=I_tvalid&&(I_tuser||sof_pending);
 reg [19:0] count;
 reg [31:0] sum_r,sum_g,sum_b;
 reg [10:0] beat_r,beat_g,beat_b;
 integer k,j;
 reg [7:0] sr,sg,sb;
 always @* begin
  beat_r=0;beat_g=0;beat_b=0;sr=0;sg=0;sb=0;
  for(j=0;j<4;j=j+1) begin
   sr=I_tdata[j*24+16+:8];sg=I_tdata[j*24+8+:8];sb=I_tdata[j*24+:8];
   // Identical RGB sample mask; exclude clipped whites and near-black noise.
   if(({2'b0,sr}+{2'b0,sg}+{2'b0,sb}<10'd720)&&
      ({2'b0,sr}+{2'b0,sg}+{2'b0,sb}>10'd24)) begin
    beat_r=beat_r+sr;beat_g=beat_g+sg;beat_b=beat_b+sb;
   end
  end
 end
 // Statistics have their own pipeline: do not put RGB masking, four-pixel
 // addition AND the 32-bit frame accumulator on one 103.52 MHz path.
 // Pixel output latency stays unchanged; SOF follows the statistics beat.
 reg [10:0] stats_r,stats_g,stats_b;
 reg stats_valid,stats_first;
 wire [31:0] total_r=sum_r+stats_r,total_g=sum_g+stats_g,total_b=sum_b+stats_b;
 reg [17:0] gain_r,gain_b,next_r,next_b;
 reg next_ready,busy;
 reg [5:0] remaining;
 reg [31:0] den_r,den_b;
 reg [47:0] quot_r,quot_b;
 reg [32:0] rem_r,rem_b;
 wire [32:0] trial_r={rem_r[31:0],quot_r[47]};
 wire [32:0] trial_b={rem_b[31:0],quot_b[47]};
 wire ge_r=trial_r>={1'b0,den_r},ge_b=trial_b>={1'b0,den_b};
 wire [47:0] step_r={quot_r[46:0],ge_r},step_b={quot_b[46:0],ge_b};
 // Capture the final quotient and the old gain together. Splitting this
 // private scalar computation does not add a stage to the RGB/marker path.
 reg [1:0] smooth_phase;
 reg [47:0] smooth_ratio_r,smooth_ratio_b;
 reg [17:0] smooth_old_r,smooth_old_b,smooth_target_r,smooth_target_b;
 reg [20:0] smooth_blend_r,smooth_blend_b;
 wire [17:0] smooth_limited_r=smooth_blend_r[20:3];
 wire [17:0] smooth_limited_b=smooth_blend_b[20:3];
 function [17:0] clamp_ratio;
  input [47:0] ratio;
  begin
   if(ratio>48'd131072)clamp_ratio=18'd131072;
   else if(ratio<48'd32768)clamp_ratio=18'd32768;
   else clamp_ratio=ratio[17:0];
  end
 endfunction
 function [17:0] limit_gain;
  input [17:0] old_gain,limited;
  begin
   if(limited>old_gain+18'd4096)limit_gain=old_gain+18'd4096;
   else if(limited+18'd4096<old_gain)limit_gain=old_gain-18'd4096;
   else limit_gain=limited;
  end
 endfunction
 wire [17:0] apply_r=(first&&next_ready)?next_r:gain_r;
 wire [17:0] apply_b=(first&&next_ready)?next_b:gain_b;
 // ---- free-running reverse lookup: equivalent temperature of the current auto gains,
 // ---- refreshed every 26 clocks, for the on-screen readout only
 reg  [4:0]  est_idx;
 wire [12:0] est_temp=13'd3000+(est_idx*13'd200);   // 3000..8000 K, matches the table clamp
 wire [19:0] est_r,est_b;
 awb_wb_temp_table u_est_wb_table (.I_temp(est_temp),.O_gain_r(est_r),.O_gain_b(est_b));
 function [39:0] wb_diff; input [19:0] a; input [19:0] b;
  begin wb_diff=(a>b)?(a-b):(b-a); end
 endfunction
 reg [39:0] est_cost; reg [12:0] est_best,est_out;
 wire [39:0] est_c=wb_diff(est_r,{2'b0,gain_r})+wb_diff(est_b,{2'b0,gain_b});
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin est_idx<=0;est_cost<=40'hFFFFFFFFFF;est_best<=13'd5000;est_out<=13'd5000; end
  else begin
   if(est_c<est_cost) begin est_cost<=est_c;est_best<=est_temp; end
   if(est_idx==5'd25) begin   // 26 entries: 3000..8000 K
    est_idx<=0;
    est_out<=(est_c<est_cost)?est_temp:est_best;
    est_cost<=40'hFFFFFFFFFF;
   end else est_idx<=est_idx+5'd1;
  end
 end
 assign O_wb_temp=est_out;
 reg [25:0] product_r[0:3],product_b[0:3];
 reg [7:0] green[0:3];
 reg valid_d,user_d,last_d;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   sof_pending<=0;count<=0;sum_r<=1;sum_g<=1;sum_b<=1;
   stats_r<=0;stats_g<=0;stats_b<=0;stats_valid<=0;stats_first<=0;
   gain_r<=18'd65536;gain_b<=18'd65536;next_r<=18'd65536;next_b<=18'd65536;
   next_ready<=0;busy<=0;remaining<=0;den_r<=1;den_b<=1;
   quot_r<=0;quot_b<=0;rem_r<=0;rem_b<=0;
   smooth_phase<=0;smooth_ratio_r<=0;smooth_ratio_b<=0;
   smooth_old_r<=0;smooth_old_b<=0;smooth_target_r<=0;smooth_target_b<=0;
   smooth_blend_r<=0;smooth_blend_b<=0;
   valid_d<=0;user_d<=0;last_d<=0;O_tvalid<=0;O_tuser<=0;O_tlast<=0;O_tdata<=0;
   for(k=0;k<4;k=k+1) begin product_r[k]<=0;product_b[k]<=0;green[k]<=0;end
  end else begin
   if(I_tuser) sof_pending<=1;
   if(first) begin
    sof_pending<=0;
    if(next_ready) begin gain_r<=next_r;gain_b<=next_b;next_ready<=0;end
   end
   // Publish after consume: a simultaneous SOF consumes the OLD pending
   // pair while retaining the NEW pair for the following SOF.
   case(smooth_phase)
    1:begin
     smooth_target_r<=clamp_ratio(smooth_ratio_r);smooth_target_b<=clamp_ratio(smooth_ratio_b);
     smooth_phase<=2;
    end
    2:begin
     smooth_blend_r<=({3'd0,smooth_old_r}<<3)-smooth_old_r+smooth_target_r+21'd4;
     smooth_blend_b<=({3'd0,smooth_old_b}<<3)-smooth_old_b+smooth_target_b+21'd4;
     smooth_phase<=3;
    end
    3:begin
     next_r<=limit_gain(smooth_old_r,smooth_limited_r);
     next_b<=limit_gain(smooth_old_b,smooth_limited_b);
     next_ready<=1;smooth_phase<=0;
    end
    default:begin end
   endcase
   stats_valid<=I_tvalid;stats_first<=first;
   if(I_tvalid) begin stats_r<=beat_r;stats_g<=beat_g;stats_b<=beat_b;end
   if(stats_valid&&stats_first) begin
    count<=1;
    sum_r<=32'd1+stats_r;sum_g<=32'd1+stats_g;sum_b<=32'd1+stats_b;
   end else if(stats_valid&&count!=0) begin
    sum_r<=total_r;sum_g<=total_g;sum_b<=total_b;
    if(count==FRAME_BEATS-1) begin
     count<=0;busy<=1;remaining<=48;
     den_r<=total_r;den_b<=total_b;quot_r<={total_g,16'd0};quot_b<={total_g,16'd0};
     rem_r<=0;rem_b<=0;
    end else count<=count+1'b1;
   end
   if(busy) begin
    quot_r<=step_r;quot_b<=step_b;
    rem_r<=ge_r ? trial_r-{1'b0,den_r}:trial_r;
    rem_b<=ge_b ? trial_b-{1'b0,den_b}:trial_b;
    remaining<=remaining-1'b1;
    if(remaining==1) begin
     busy<=0;smooth_phase<=1;
     smooth_ratio_r<=step_r;smooth_ratio_b<=step_b;
     smooth_old_r<=gain_r;smooth_old_b<=gain_b;
    end
   end
   valid_d<=I_tvalid;user_d<=first;last_d<=I_tvalid&&I_tlast;
   O_tvalid<=valid_d;O_tuser<=user_d;O_tlast<=last_d;
   if(I_tvalid) for(k=0;k<4;k=k+1) begin
    product_r[k]<=I_tdata[k*24+16+:8]*apply_r;
    product_b[k]<=I_tdata[k*24+:8]*apply_b;
    green[k]<=I_tdata[k*24+8+:8];
   end
   if(valid_d) for(k=0;k<4;k=k+1) begin
    O_tdata[k*24+16+:8]<=(|product_r[k][25:24])?8'd255:product_r[k][23:16];
    O_tdata[k*24+8+:8]<=green[k];
    O_tdata[k*24+:8]<=(|product_b[k][25:24])?8'd255:product_b[k][23:16];
   end
   else O_tdata<=0;
  end
 end
endmodule
