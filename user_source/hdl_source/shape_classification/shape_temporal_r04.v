// R04 SINGLE-CLOCK result-packet filter. No pixels/CDC/board IO here.
// I_frame_tick is a SAFE BLANKING boundary, not a combinational SOF bypass.
// Consumer sees published packet after this clock; R05 must schedule before DE.
// Independent source and sequence IDs must both advance (32-bit half-range).
module shape_temporal_r04 #(
 parameter integer CONFIRM_OBS=3,MISS_OBS=3,STALE_FRAMES=6,
 parameter integer ASSOC_EDGE=64,DEADBAND=4,MAX_SOURCE_GAP=8,MAX_SEQ_GAP=8
)(
 input wire I_clk,I_rst_n,I_enable,I_cancel,I_fault,I_frame_tick,
 input wire [31:0] I_expected_epoch,
 input wire I_result_valid,I_paper_valid,
 input wire [2:0] I_status,
 input wire I_target_valid,
 input wire [1:0] I_class,
 input wire [11:0] I_xmin,I_ymin,I_xmax,I_ymax,
 input wire [15:0] I_area,
 input wire [31:0] I_seq,I_source,I_epoch,
 input wire [1:0] I_profile,
 input wire [3:0] I_reason,
 input wire I_retry,
 output wire O_update,O_valid,
 output wire [2:0] O_status,
 output wire [1:0] O_class,
 output wire [11:0] O_xmin,O_ymin,O_xmax,O_ymax,O_center_x,O_center_y,
 output wire [15:0] O_area,
 output wire [31:0] O_seq,O_source,O_epoch,
 output wire [1:0] O_profile,
 output wire [3:0] O_reason,
 output wire O_retry
);
 localparam PARAM_OK=CONFIRM_OBS>=1&&CONFIRM_OBS<=15&&MISS_OBS>=1&&MISS_OBS<=15&&
  STALE_FRAMES>=1&&STALE_FRAMES<=255&&ASSOC_EDGE>=1&&ASSOC_EDGE<=255&&
  DEADBAND>=0&&DEADBAND<=64&&MAX_SOURCE_GAP>=1&&MAX_SEQ_GAP>=1;
 reg [31:0] context_epoch,last_seq,last_source;
 reg seen;
 reg [1:0] last_profile;
 reg [7:0] age;
 reg [3:0] misses,c_count;
 reg c_valid;
 reg [1:0] c_class,c_profile;
 reg [11:0] c_l,c_t,c_r,c_b;
 // Latest associated raw box is distinct from smoothed display geometry.
 // Otherwise smoothing lag would incorrectly break a moving target's track.
 reg t_valid;
 reg [2:0] t_status;
 reg [1:0] t_class,t_profile;
 reg [11:0] t_l,t_t,t_r,t_b,raw_l,raw_t,raw_r,raw_b;
 reg [15:0] t_area;
 reg [31:0] t_seq,t_source,t_epoch;
 reg [3:0] t_reason;
 reg t_retry;
 reg v_valid,v_update;
 reg [2:0] v_status;
 reg [1:0] v_class,v_profile;
 reg [11:0] v_l,v_t,v_r,v_b;
 reg [15:0] v_area;
 reg [31:0] v_seq,v_source,v_epoch;
 reg [3:0] v_reason;
 reg v_retry;
 function newer;
  input [31:0] a,b;reg [31:0] d;
  begin d=a-b;newer=d!=0&&!d[31];end
 endfunction
 function near_edge;
  input [11:0] a,b;reg signed [12:0] delta;
  begin delta=$signed({1'b0,a})-$signed({1'b0,b});
   near_edge=delta<=ASSOC_EDGE&&delta>=-ASSOC_EDGE;end
 endfunction
 function associated;
  input [11:0] al,at,ar,ab,bl,bt,br,bb;
  begin associated=al<=br&&bl<=ar&&at<=bb&&bt<=ab&&
   near_edge(al,bl)&&near_edge(at,bt)&&near_edge(ar,br)&&near_edge(ab,bb);end
 endfunction
 function [11:0] smooth;
  input [11:0] prior,sample;reg signed [13:0] delta,rounded,moved;
  begin
   // Arithmetic right shift already rounds negative differences away from
   // zero; only positive differences need +3. One signed update adder replaces
   // two abs subtractions and two directional update adders. 14 bits cover
   // the FULL 0..4095 interface domain, including positive delta+3=4098.
   delta=$signed({2'b00,sample})-$signed({2'b00,prior});
   rounded=delta[13]?delta:delta+14'sd3;
   moved=$signed({2'b00,prior})+(rounded>>>2);
   if(delta<=DEADBAND&&delta>=-DEADBAND)smooth=prior;
   else smooth=moved[11:0];
  end
 endfunction
 wire live=I_rst_n&&I_enable&&!I_cancel&&!I_fault&&PARAM_OK&&context_epoch==I_expected_epoch;
 wire fresh=I_result_valid&&I_status!=0&&I_epoch==I_expected_epoch&&
  (!seen||(newer(I_seq,last_seq)&&newer(I_source,last_source)));
 wire gap=seen&&($unsigned(I_source-last_source)>MAX_SOURCE_GAP||
  $unsigned(I_seq-last_seq)>MAX_SEQ_GAP||I_profile!=last_profile);
 wire malformed=I_status>3||I_profile==3||
  (I_status==1&&(!I_target_valid||I_xmin>I_xmax||I_ymin>I_ymax||I_xmax>=1280||I_ymax>=720||I_area==0))||
  (I_status==2&&(I_target_valid||I_class!=0));
 wire recognized=I_status==1&&I_target_valid&&I_class!=0;
 wire track_match=t_valid&&I_class==t_class&&I_profile==t_profile&&
  associated(raw_l,raw_t,raw_r,raw_b,I_xmin,I_ymin,I_xmax,I_ymax);
 wire cand_match=c_valid&&I_class==c_class&&I_profile==c_profile&&
  associated(c_l,c_t,c_r,c_b,I_xmin,I_ymin,I_xmax,I_ymax);
 wire [3:0] next_count=(!gap&&cand_match)?(c_count>=CONFIRM_OBS?c_count:c_count+4'd1):4'd1;
 wire stale=I_frame_tick&&!fresh&&age>=STALE_FRAMES-1;
 wire [11:0] smooth_l=smooth(t_l,I_xmin),smooth_t=smooth(t_t,I_ymin),
  smooth_r=smooth(t_r,I_xmax),smooth_b=smooth(t_b,I_ymax);
 wire smooth_ordered=smooth_l<=smooth_r&&smooth_t<=smooth_b;
 assign O_update=live&&v_update;
 assign O_valid=live&&v_valid;
 assign O_status=!I_rst_n?3'd0:!PARAM_OK?3'd3:(!I_enable||I_cancel||context_epoch!=I_expected_epoch)?3'd0:I_fault?3'd3:v_status;
 assign O_class=O_valid?v_class:2'd0;
 assign O_xmin=O_valid?v_l:12'd0;assign O_ymin=O_valid?v_t:12'd0;
 assign O_xmax=O_valid?v_r:12'd0;assign O_ymax=O_valid?v_b:12'd0;
 assign O_center_x=({1'b0,O_xmin}+{1'b0,O_xmax})>>1;
 assign O_center_y=({1'b0,O_ymin}+{1'b0,O_ymax})>>1;
 assign O_area=O_valid?v_area:16'd0;
 assign O_seq=O_valid?v_seq:32'd0;assign O_source=O_valid?v_source:32'd0;
 assign O_epoch=O_valid?v_epoch:I_expected_epoch;
 assign O_profile=O_valid?v_profile:2'd0;assign O_reason=O_valid?v_reason:4'd0;
 assign O_retry=O_valid&&v_retry;
 task clear_candidate;
  begin c_valid<=0;c_count<=0;c_class<=0;c_profile<=0;c_l<=0;c_t<=0;c_r<=0;c_b<=0;end
 endtask
 task clear_track;
  input [2:0] status;
  begin t_valid<=0;t_status<=status;t_class<=0;t_profile<=0;t_l<=0;t_t<=0;t_r<=0;t_b<=0;
   raw_l<=0;raw_t<=0;raw_r<=0;raw_b<=0;t_area<=0;t_seq<=0;t_source<=0;t_epoch<=0;t_reason<=0;t_retry<=0;misses<=0;end
 endtask
 task clear_visible;
  input [2:0] status;
  begin v_valid<=0;v_status<=status;v_class<=0;v_profile<=0;v_l<=0;v_t<=0;v_r<=0;v_b<=0;
   v_area<=0;v_seq<=0;v_source<=0;v_epoch<=0;v_reason<=0;v_retry<=0;v_update<=1;end
 endtask
 task accept_track;
  input filtered;
  begin
   t_valid<=1;t_status<=1;t_class<=I_class;t_profile<=I_profile;
   t_l<=filtered&&smooth_ordered?smooth_l:I_xmin;t_t<=filtered&&smooth_ordered?smooth_t:I_ymin;
   t_r<=filtered&&smooth_ordered?smooth_r:I_xmax;t_b<=filtered&&smooth_ordered?smooth_b:I_ymax;
   raw_l<=I_xmin;raw_t<=I_ymin;raw_r<=I_xmax;raw_b<=I_ymax;
   t_area<=I_area;t_seq<=I_seq;t_source<=I_source;t_epoch<=I_epoch;t_reason<=I_reason;t_retry<=I_retry;
   misses<=0;clear_candidate;
  end
 endtask
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   context_epoch<=0;seen<=0;last_seq<=0;last_source<=0;last_profile<=0;age<=0;
   clear_candidate;clear_track(0);clear_visible(0);v_update<=0;
  end else begin
   v_update<=0;
   if(!PARAM_OK)begin clear_candidate;clear_track(3);clear_visible(3);age<=0;end
   else if(context_epoch!=I_expected_epoch)begin
    context_epoch<=I_expected_epoch;seen<=0;last_seq<=0;last_source<=0;last_profile<=0;age<=0;
    clear_candidate;clear_track(0);clear_visible(0);
   end else if(!I_enable||I_cancel||I_fault)begin
    clear_candidate;clear_track(I_fault?3'd3:3'd0);clear_visible(I_fault?3'd3:3'd0);age<=0;
    // Keep watermarks across local flush/disable: replaying an old packet
    // cannot count as a fresh observation. Source counter reset needs epoch.
   end else begin
    if(I_frame_tick)begin
     v_update<=1;v_valid<=t_valid;v_status<=t_status;v_class<=t_class;v_profile<=t_profile;
     v_l<=t_l;v_t<=t_t;v_r<=t_r;v_b<=t_b;v_area<=t_area;
     v_seq<=t_seq;v_source<=t_source;v_epoch<=t_epoch;v_reason<=t_reason;v_retry<=t_retry;
     if(age<STALE_FRAMES)age<=age+1'b1;
    end
    if(stale)begin clear_candidate;clear_track(2);clear_visible(2);age<=STALE_FRAMES;end
    else if(fresh)begin
     seen<=1;last_seq<=I_seq;last_source<=I_source;last_profile<=I_profile;age<=0;
     if(malformed||I_status==3)begin clear_candidate;clear_track(3);clear_visible(3);end
     else if(!I_paper_valid)begin clear_candidate;clear_track(2);clear_visible(2);end
     else if(recognized)begin
      if(!gap&&track_match)accept_track(1);
      else if(next_count>=CONFIRM_OBS)begin
       if(gap)clear_visible(0);
       accept_track(0);
      end else begin
       if(gap)begin clear_track(0);clear_visible(0);end
       else if(t_valid)begin
        if(misses>=MISS_OBS-1)clear_track(2);else misses<=misses+1'b1;
       end else t_status<=0;
       c_valid<=1;c_count<=next_count;c_class<=I_class;c_profile<=I_profile;
       c_l<=I_xmin;c_t<=I_ymin;c_r<=I_xmax;c_b<=I_ymax;
      end
     end else begin
      clear_candidate;
      if(gap)begin clear_track(2);clear_visible(2);end
      else if(t_valid)begin
       if(misses>=MISS_OBS-1)clear_track(2);else misses<=misses+1'b1;
      end else t_status<=2;
     end
    end
   end
  end
 end
endmodule
