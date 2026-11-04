// Independent low-resolution motion CCA and atomic display result.
// Stream is provisional: a final/done comparison frame is mandatory.
// Snapshot changes only on raw RGB SOF; same-edge bypass includes pixel zero.
module motion_regions #(
 parameter integer WIDTH=1280,HEIGHT=720,MIN_AREA=4,MIN_WIDTH=2,MIN_HEIGHT=2,
 parameter integer ROW_BUDGET=12000,FOLD_BUDGET=12000,TAIL_BUDGET=30000
)(
 input wire I_clk,I_rst_n,I_enable,I_raw_sof,I_reference_clear,
 input wire I_valid,I_first,I_final,I_compare_valid,I_mask,I_done,
 input wire [7:0] I_x,input wire [6:0] I_y,
 input wire [31:0] I_source_id,I_reference_id,I_epoch,
 output wire [177:0] O_snapshot,
 output wire [31:0] O_reference_id,
 output wire O_pending_ready,O_busy,
 output wire [177:0] O_candidate,output wire [19:0] O_candidate_area,
 output reg O_fault,output reg [2:0] O_error
);
 localparam LW=WIDTH/8,LH=HEIGHT/8;
 localparam OK=(WIDTH>=8 && WIDTH<=1280 && HEIGHT>=8 && HEIGHT<=720 &&
  WIDTH%8==0 && HEIGHT%8==0 && TAIL_BUDGET>=32 && TAIL_BUDGET<=65535);
 reg active,committed,invalidated,pending;
 reg [31:0] frame_source,frame_reference,frame_epoch,pending_reference,snapshot_reference;
 reg [15:0] tail;
 reg [177:0] candidate,snapshot;
 wire first=I_valid && I_first;
 wire identity_bad=I_valid && I_compare_valid && !I_first &&
  (!active || I_source_id!=frame_source || I_reference_id!=frame_reference || I_epoch!=frame_epoch);
 wire terminal_bad=I_done && !(I_valid && I_final);
 wire clear=I_reference_clear || !I_enable || !OK;
 wire fv,ff,fl,fm,filter_fault;
 wire [7:0] fx;
 wire [6:0] fy;
 wire engine_enable=I_enable && OK && active && !clear && !I_raw_sof && !first;
 motion_mask_filter #(.WIDTH(LW),.HEIGHT(LH)) filter(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable && OK),
  .I_clear(clear || I_raw_sof || identity_bad || terminal_bad),
  .I_valid(I_valid && I_compare_valid),.I_first(I_first),.I_final(I_final),.I_mask(I_mask),
  .I_x(I_x),.I_y(I_y),.O_valid(fv),.O_first(ff),.O_final(fl),.O_mask(fm),
  .O_fault(filter_fault),.O_x(fx),.O_y(fy));
 wire rv,rn,core_done,core_failed,core_frozen,root_ready,root_valid,run_ready,idle;
 wire [10:0] run_xmin,run_xmax;
 wire [9:0] run_y,run_label,labels,root,query_label;
 wire [2:0] core_error,stats_error;
 wire [3:0] root_reads;
 wire root_req,stats_ready,stats_done,stats_failed,target;
 wire [10:0] xmin,xmax;
 wire [9:0] ymin,ymax;
 wire [19:0] area;
 wire abort=identity_bad || terminal_bad || filter_fault;
 red_cca_core #(.WIDTH(LW),.HEIGHT(LH),.MAX_LABELS(255),.ROW_BUDGET(ROW_BUDGET)) core(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(engine_enable),
  .I_valid(fv),.I_red(fm),.I_user(ff),.I_last(fv && fx==LW-1),
  .I_x({4'd0,fx}),.I_y({5'd0,fy}),.I_frame_start(ff),.I_frame_end(fl),
  .I_bad_event(abort),.I_frame_bad(stats_failed && !ff),.I_config_bad(1'b0),
  .I_run_ready(run_ready),.I_features_idle(idle),.O_run_valid(rv),.O_run_new(rn),
  .O_run_xmin(run_xmin),.O_run_xmax(run_xmax),.O_run_y(run_y),.O_run_label(run_label),
  .O_done(core_done),.O_failed(core_failed),.O_error(core_error),.O_frozen(core_frozen),.O_labels(labels),
  .I_root_req(root_req),.I_root_label(query_label),.O_root_ready(root_ready),
  .O_root_valid(root_valid),.O_root(root),.O_root_reads(root_reads),.O_fault(),.O_root_rank(),
  .O_row_done(),.O_row_cycles(),.O_row_runs(),.O_row_scans(),.O_row_overlaps(),
  .O_row_find_reads(),.O_max_find_reads(),.O_max_rank(),.O_max_runs());
 motion_cca_stats #(.WIDTH(LW),.HEIGHT(LH),.MIN_AREA(MIN_AREA),.MIN_WIDTH(MIN_WIDTH),
  .MIN_HEIGHT(MIN_HEIGHT),.FOLD_BUDGET(FOLD_BUDGET)) stats(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(engine_enable),.I_frame_start(ff),
  .I_bad_event(abort),.I_frame_bad(1'b0),.I_config_bad(1'b0),
  .I_core_failed(core_failed),.I_core_error(core_error),.I_core_done(core_done),
  .I_core_frozen(core_frozen),.I_labels(labels),.I_run_valid(rv),.I_run_new(rn),
  .I_run_xmin(run_xmin),.I_run_xmax(run_xmax),.I_run_y(run_y),.I_run_label(run_label),
  .O_run_ready(run_ready),.O_features_idle(idle),.O_root_req(root_req),.O_root_label(query_label),
  .I_root_ready(root_ready),.I_root_valid(root_valid),.I_root(root),.I_root_reads(root_reads),
  .O_result_ready(stats_ready),.O_done(stats_done),.O_failed(stats_failed),.O_error(stats_error),
  .O_target_valid(target),.O_xmin(xmin),.O_xmax(xmax),.O_ymin(ymin),.O_ymax(ymax),.O_area(area),
  .O_fault(),.O_total_area(),.O_regions(),.O_qualified(),.O_fold_cycles(),.O_max_label_cycles(),.O_busy());
 // Map inclusive block extrema before computing the binary/BCD centre.
 // Full-screen centre is floor((8*xmin + 8*xmax + 7)/2), NOT 8*low-res centre.
 wire [11:0] full_xmin={xmin[8:0],3'b000},full_xmax={xmax[8:0],3'b111};
 wire [11:0] full_ymin={ymin[8:0],3'b000},full_ymax={ymax[8:0],3'b111};
 wire coords_ready,coords_valid,coords_busy;
 wire [11:0] cx,cy;
 wire [15:0] cbx,cby;
 wire [31:0] coord_source,coord_epoch;
 wire [3:0] coord_mode;
 wire timeout=active && committed && tail>=TAIL_BUDGET-1 && !pending;
 // An old failure cannot poison the fresh first sample: engines reset there.
 wire failure=!first && (abort || core_failed || stats_failed || timeout);
 wire cancel=clear || I_raw_sof || first || failure;
 wire coordinate_start=engine_enable && committed && stats_done && stats_ready && target && !cancel;
 rgb_bbox_coordinates #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) coordinates(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(cancel),.I_start(coordinate_start),
  .I_xmin(full_xmin),.I_xmax(full_xmax),.I_ymin(full_ymin),.I_ymax(full_ymax),
  .I_source_frame_id(frame_source),.I_config_epoch(frame_epoch),.I_mode(4'd11),
  .O_ready(coords_ready),.O_valid(coords_valid),.O_busy(coords_busy),
  .O_x(cx),.O_y(cy),.O_x_bcd(cbx),.O_y_bcd(cby),
  .O_source_frame_id(coord_source),.O_config_epoch(coord_epoch),.O_mode(coord_mode));
 wire coordinate_match=coords_ready && coords_valid && coord_source==frame_source &&
  coord_epoch==frame_epoch && coord_mode==11;
 // Waiting/error maps to glyph 3; status 3 is the legacy successful glyph 1.
 wire [177:0] wait_packet={1'b0,12'd0,12'd0,16'd0,16'd0,4'd11,
  32'd0,32'd0,1'b1,1'b0,3'd1,12'd0,12'd0,12'd0,12'd0};
 wire boundary_ready=pending && !clear && !failure;
 wire [177:0] boundary=boundary_ready?candidate:
  ((clear || invalidated || active)?wait_packet:snapshot);
 assign O_snapshot=(!I_rst_n || !I_enable)?178'd0:(I_raw_sof?boundary:snapshot);
 assign O_reference_id=(!I_rst_n || !I_enable)?32'd0:
  (I_raw_sof?(boundary_ready?pending_reference:((clear || invalidated || active)?32'd0:snapshot_reference)):
   snapshot_reference);
 assign O_pending_ready=pending;
 // Observation only; no feedback to raw CCA, safety events or publication.
 assign O_candidate=candidate;assign O_candidate_area=area;
 assign O_busy=active && !pending;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   active<=0;committed<=0;invalidated<=1;pending<=0;tail<=0;
   frame_source<=0;frame_reference<=0;frame_epoch<=0;pending_reference<=0;snapshot_reference<=0;
   candidate<=0;snapshot<=0;O_fault<=0;O_error<=0;
  end else begin
   O_fault<=0;
   if(!I_enable) begin
    active<=0;committed<=0;invalidated<=1;pending<=0;tail<=0;
    snapshot<=0;snapshot_reference<=0;O_error<=0;
   end else if(I_raw_sof) begin
    snapshot<=boundary;
    snapshot_reference<=boundary_ready?pending_reference:
     ((clear || invalidated || active)?32'd0:snapshot_reference);
    active<=0;committed<=0;pending<=0;tail<=0;invalidated<=clear || (active && !boundary_ready);
   end else if(clear || failure) begin
    if(failure && active) begin O_fault<=1;
     O_error<=stats_failed?stats_error:(core_failed?core_error:(timeout?3'd6:3'd5));end
    active<=0;committed<=0;pending<=0;invalidated<=1;tail<=0;
   end else if(first) begin
    // Reset the old CCA one edge BEFORE the filtered first sample arrives.
    active<=I_compare_valid;committed<=I_done && I_final && I_compare_valid;
    pending<=0;tail<=0;invalidated<=!I_compare_valid;O_error<=0;
    frame_source<=I_source_id;frame_reference<=I_reference_id;frame_epoch<=I_epoch;
   end else if(active) begin
    if(I_done && I_compare_valid) begin committed<=1;tail<=0;end
    else if(committed && !pending) tail<=tail+16'd1;
    if(committed && !pending && stats_ready && !target) begin
     candidate<={1'b0,12'd0,12'd0,16'd0,16'd0,4'd11,frame_source,frame_epoch,
      1'b1,1'b0,3'd2,12'd0,12'd0,12'd0,12'd0};
     pending<=1;pending_reference<=frame_reference;invalidated<=0;
    end else if(committed && !pending && stats_ready && target && coordinate_match) begin
     candidate<={1'b1,cx,cy,cbx,cby,4'd11,frame_source,frame_epoch,
      1'b1,1'b1,3'd3,full_xmin,full_xmax,full_ymin,full_ymax};
     pending<=1;pending_reference<=frame_reference;invalidated<=0;
    end
   end
  end
 end
endmodule

// Display-only temporal envelope. Raw mode-13 safety events bypass this module.
// Results are prepared in blanking, with original source/epoch ownership, then
// published atomically at raw SOF (including its first-pixel bypass).
module motion_display_stabilizer #(
 parameter integer WIDTH=1280,HEIGHT=720,MIN_AREA=8,CONFIRM=2,MISS_LIMIT=3,
 parameter integer ASSOC_DISTANCE=192,STALE_FRAMES=6,MAX_SOURCE_GAP=8
)(
 input wire I_clk,I_rst_n,I_enable,I_clear,I_raw_sof,I_pending,
 input wire [177:0] I_candidate,input wire [19:0] I_area,
 output wire [177:0] O_snapshot,output reg O_fault
);
 localparam OK=WIDTH>=8&&WIDTH<=1280&&HEIGHT>=8&&HEIGHT<=720&&
  WIDTH%8==0&&HEIGHT%8==0&&MIN_AREA>=1&&MIN_AREA<=14400&&
  CONFIRM>=1&&CONFIRM<=15&&MISS_LIMIT>=1&&MISS_LIMIT<=15&&
  ASSOC_DISTANCE>=1&&ASSOC_DISTANCE<=1280&&STALE_FRAMES>=1&&STALE_FRAMES<=15&&MAX_SOURCE_GAP>=1;
 // SMOOTH is the second half of the bbox pipeline: the merge decision and the
 // picked sample edges are registered first, the inward-edge easing is applied
 // on the following cycle. The extra cycle sits in blanking, far inside the
 // frame budget, and keeps the 75 MHz path clear of the 13-level chain.
 localparam [1:0] IDLE=0,LAUNCH=1,WAIT_COORDS=2,SMOOTH=3;
 reg [1:0] state;
 reg seen_pending,seen_source,previous_valid,track_valid,ready;
 reg [31:0] last_source,last_epoch;
 reg [3:0] confirmations,misses,age;
 reg [11:0] pl,pr,pt,pb;
 reg [177:0] candidate,snapshot,track;
 reg [11:0] work_l,work_r,work_t,work_b;
 reg [31:0] work_source,work_epoch;
 reg [11:0] pend_l,pend_r,pend_t,pend_b;
 reg [31:0] pend_source,pend_epoch;
 reg pend_near;
 wire [11:0] l=I_candidate[47:36],r=I_candidate[35:24],t=I_candidate[23:12],b=I_candidate[11:0];
 wire [31:0] source=I_candidate[116:85],epoch=I_candidate[84:53];
 wire [2:0] status=I_candidate[50:48];
 wire [31:0] source_delta=source-last_source;
 wire event_new=I_pending&&!seen_pending;
 wire fresh=event_new&&(!seen_source||epoch!=last_epoch||(source_delta!=0&&!source_delta[31]));
 wire discontinuity=!seen_source||epoch!=last_epoch||source_delta>MAX_SOURCE_GAP;
 wire ordered=l<=r&&t<=b&&r<WIDTH&&b<HEIGHT;
 wire format_ok=I_candidate[120:117]==11&&I_candidate[52]&&
  ((status==3&&I_candidate[51]&&I_candidate[177]&&ordered)||
   (status==2&&!I_candidate[51]&&!I_candidate[177]));
 // Reject tiny regions and nearly full-frame brightness steps, without
 // changing the physical difference threshold or raw alarm semantics.
 wire qualified=format_ok&&status==3&&I_area>=MIN_AREA&&
  I_area<=(WIDTH/8)*(HEIGHT/8)*3/4&&
  ({1'b0,r}-{1'b0,l}+13'd1)>=16&&({1'b0,b}-{1'b0,t}+13'd1)>=16;
 function nearby;
  input [11:0] al,ar,at,ab,bl,br,bt,bb;
  reg signed [13:0] dx,dy;
  begin
   dx=$signed({2'b0,al})+$signed({2'b0,ar})-$signed({2'b0,bl})-$signed({2'b0,br});
   dy=$signed({2'b0,at})+$signed({2'b0,ab})-$signed({2'b0,bt})-$signed({2'b0,bb});
   nearby=dx<=2*ASSOC_DISTANCE&&dx>=-2*ASSOC_DISTANCE&&dy<=2*ASSOC_DISTANCE&&dy>=-2*ASSOC_DISTANCE;
  end
 endfunction
 wire [11:0] ul=l<pl?l:pl,ur=r>pr?r:pr,ut=t<pt?t:pt,ub=b>pb?b:pb;
 wire merge=previous_valid&&!discontinuity&&nearby(l,r,t,b,pl,pr,pt,pb)&&
  ({1'b0,ur}-{1'b0,ul}+13'd1)<=WIDTH/2&&
  ({1'b0,ub}-{1'b0,ut}+13'd1)<=HEIGHT;
 wire [11:0] nl=merge?ul:l,nr=merge?ur:r,nt=merge?ut:t,nb=merge?ub:b;
 wire track_near=track_valid&&!discontinuity&&nearby(l,r,t,b,track[47:36],track[35:24],track[23:12],track[11:0]);
 wire [3:0] next_count=merge?(confirmations>=CONFIRM?confirmations:confirmations+4'd1):4'd1;
 // Fast expansion contains current evidence; inward edges settle by 1/4.
 function [11:0] low_edge;
  input [11:0] prior,sample;reg [12:0] delta;
  begin delta={1'b0,sample}-{1'b0,prior};low_edge=sample<=prior?sample:prior+((delta+13'd3)>>2);end
 endfunction
 function [11:0] high_edge;
  input [11:0] prior,sample;reg [12:0] delta;
  begin delta={1'b0,prior}-{1'b0,sample};high_edge=sample>=prior?sample:prior-((delta+13'd3)>>2);end
 endfunction
 // Stage 1 of the split (combinational): pick the merged/sample edges and the
 // smoothing decision. Stage 2 (SMOOTH, see the FSM below) applies the easing
 // to the registered copies, so neither half carries the full 13-level chain.
 wire [11:0] wl=pend_near?low_edge(track[47:36],pend_l):pend_l;
 wire [11:0] wr=pend_near?high_edge(track[35:24],pend_r):pend_r;
 wire [11:0] wt=pend_near?low_edge(track[23:12],pend_t):pend_t;
 wire [11:0] wb=pend_near?high_edge(track[11:0],pend_b):pend_b;
 wire cr,cv,cbusy;wire [11:0] cx,cy;wire [15:0] cbx,cby;
 wire [31:0] csource,cepoch;wire [3:0] cmode;
 wire clear=I_clear||!I_enable||!OK;
 wire coordinate_match=cr&&cv&&csource==work_source&&cepoch==work_epoch&&cmode==11;
 rgb_bbox_coordinates #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) coordinates(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(clear||I_raw_sof||(event_new&&state!=IDLE)||(fresh&&!format_ok)),.I_start(state==LAUNCH),
  .I_xmin(work_l),.I_xmax(work_r),.I_ymin(work_t),.I_ymax(work_b),
  .I_source_frame_id(work_source),.I_config_epoch(work_epoch),.I_mode(4'd11),
  .O_ready(cr),.O_valid(cv),.O_busy(cbusy),.O_x(cx),.O_y(cy),.O_x_bcd(cbx),.O_y_bcd(cby),
  .O_source_frame_id(csource),.O_config_epoch(cepoch),.O_mode(cmode));
 wire [177:0] wait_packet={1'b0,12'd0,12'd0,16'd0,16'd0,4'd11,32'd0,32'd0,1'b1,1'b0,3'd1,48'd0};
 wire expired=age>=STALE_FRAMES-1&&!ready;
 wire [177:0] boundary=(state!=IDLE||event_new||expired)?wait_packet:(ready?candidate:snapshot);
 assign O_snapshot=(!I_rst_n||clear)?178'd0:(I_raw_sof?boundary:snapshot);
 task clear_tracking;
  begin previous_valid<=0;track_valid<=0;confirmations<=0;misses<=0;pl<=0;pr<=0;pt<=0;pb<=0;track<=0;end
 endtask
 task miss_observation;
  begin
   ready<=1;
   if(!discontinuity&&misses<MISS_LIMIT-1)begin
    misses<=misses+4'd1;
    // This is a display hold of the ORIGINAL accepted source/epoch. It is
    // never forwarded to the alarm and never relabelled as a new detection.
    candidate<=track_valid?track:{1'b0,12'd0,12'd0,16'd0,16'd0,4'd11,source,epoch,1'b1,1'b0,3'd2,48'd0};
   end else begin
    clear_tracking;
    candidate<={1'b0,12'd0,12'd0,16'd0,16'd0,4'd11,source,epoch,1'b1,1'b0,3'd2,48'd0};
   end
  end
 endtask
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   state<=IDLE;seen_pending<=0;seen_source<=0;last_source<=0;last_epoch<=0;
   ready<=0;candidate<=0;snapshot<=0;age<=0;O_fault<=0;
   work_l<=0;work_r<=0;work_t<=0;work_b<=0;work_source<=0;work_epoch<=0;clear_tracking;
   pend_l<=0;pend_r<=0;pend_t<=0;pend_b<=0;pend_source<=0;pend_epoch<=0;pend_near<=0;
  end else begin
   O_fault<=0;
   if(clear)begin
    state<=IDLE;seen_pending<=0;seen_source<=0;last_source<=0;last_epoch<=0;
    ready<=0;candidate<=0;snapshot<=0;age<=0;clear_tracking;
    pend_l<=0;pend_r<=0;pend_t<=0;pend_b<=0;pend_source<=0;pend_epoch<=0;pend_near<=0;
   end else if(I_raw_sof)begin
    snapshot<=boundary;ready<=0;state<=IDLE;
    if(age<STALE_FRAMES)age<=age+4'd1;
    if(state!=IDLE||event_new||expired)begin
     clear_tracking;
     if(state!=IDLE||event_new)O_fault<=1;
    end
   end else begin
    if(!I_pending)seen_pending<=0;
    if(event_new)seen_pending<=1;
    if(fresh)begin
     seen_source<=1;last_source<=source;last_epoch<=epoch;age<=0;
     if(state!=IDLE||!format_ok)begin
      clear_tracking;candidate<=wait_packet;ready<=1;state<=IDLE;O_fault<=1;
     end else if(qualified)begin
      previous_valid<=1;pl<=l;pr<=r;pt<=t;pb<=b;confirmations<=next_count;misses<=0;
      if(discontinuity)begin track_valid<=0;track<=0;end
      if(next_count>=CONFIRM)begin
       pend_l<=nl;pend_r<=nr;pend_t<=nt;pend_b<=nb;pend_near<=track_near;
       pend_source<=source;pend_epoch<=epoch;state<=SMOOTH;
      end else begin
       ready<=1;
       if(track_valid&&!discontinuity&&misses<MISS_LIMIT-1)begin candidate<=track;misses<=misses+4'd1;end
       else begin
        track_valid<=0;track<=0;
        candidate<={1'b0,12'd0,12'd0,16'd0,16'd0,4'd11,source,epoch,1'b1,1'b0,3'd2,48'd0};
       end
      end
     end else miss_observation;
    end else if(state==SMOOTH)begin
     work_l<=wl;work_r<=wr;work_t<=wt;work_b<=wb;
     work_source<=pend_source;work_epoch<=pend_epoch;state<=LAUNCH;
    end else if(state==LAUNCH)state<=WAIT_COORDS;
    else if(state==WAIT_COORDS&&cr)begin
     state<=IDLE;ready<=1;
     if(coordinate_match)begin
      candidate<={1'b1,cx,cy,cbx,cby,4'd11,work_source,work_epoch,1'b1,1'b1,3'd3,work_l,work_r,work_t,work_b};
      track<={1'b1,cx,cy,cbx,cby,4'd11,work_source,work_epoch,1'b1,1'b1,3'd3,work_l,work_r,work_t,work_b};track_valid<=1;
     end else begin clear_tracking;candidate<=wait_packet;O_fault<=1;end
    end
   end
  end
 end
endmodule
