// Shared RGB detector: one B04/B05/B06 core, frame-fixed colour/configuration.
// Observer only: no RGB output, video backpressure, new clock or external pin.
// O_* includes a SOF bypass: a same-edge consumer sees the NEW frame snapshot.
module red_cca_stream #(
 parameter integer WIDTH=1280,HEIGHT=720,MIN_AREA=64,MIN_WIDTH=4,MIN_HEIGHT=4,
 parameter integer ROW_BUDGET=1600,FOLD_BUDGET=44006,
 parameter integer TAIL_BUDGET=45654,PUBLISH_GUARD=32,DETECT_MODE=6,
 parameter integer BLUE_DETECT_MODE=8,GREEN_DETECT_MODE=10,AREA_CONTROL=0,EXTERNAL_MASK=0
)(
 input wire I_clk,I_rst_n,I_enable,I_de,I_user,I_last,I_pixel_good,
 input wire [23:0] I_rgb,
 input wire I_mask_de,I_mask_good,I_mask,
 input wire [11:0] I_mask_x,I_mask_y,
 input wire [11:0] I_x,I_y,
 input wire [7:0] I_th_min,I_th_dom,   // latched at raw SOF with the selected mode
 input wire [3:0] I_mode,
 input wire [4:0] I_area_min,
 output wire O_detect,O_target_valid,
 output wire [2:0] O_status,
 output wire [11:0] O_xmin,O_xmax,O_ymin,O_ymax,
 output wire [19:0] O_area,
 output wire [31:0] O_source_frame_id,O_mode_epoch,
 output wire [15:0] O_tail_cycles,O_fold_cycles,O_max_row_cycles,
 output wire O_pending_ready,
 output wire O_coords_valid,
 output wire [11:0] O_center_x,O_center_y,
 output wire [15:0] O_center_x_bcd,O_center_y_bcd,
 output wire [3:0] O_result_mode,
 // Observer-only provenance/diagnostic taps; existing outputs are unchanged.
 output wire [31:0] O_capture_frame_seq,
 output wire O_frame_fault,
 output wire [31:0] O_capture_epoch,
 output reg O_diag_valid,
 output reg [31:0] O_diag_capture_seq,O_diag_epoch,
 output reg [3:0] O_diag_mode,
 output reg [2:0] O_diag_status,O_diag_error,
 output reg [7:0] O_diag_max_runs,
 output reg [9:0] O_diag_labels,
 output reg [15:0] O_diag_max_row_cycles,O_diag_fold_cycles,O_diag_tail_cycles
);
 localparam PARAM_OK=(TAIL_BUDGET>PUBLISH_GUARD+2 && TAIL_BUDGET<=65535 &&
  PUBLISH_GUARD>=1 && PUBLISH_GUARD<=255 && DETECT_MODE>=0 && DETECT_MODE<=15 &&
  BLUE_DETECT_MODE>=0 && BLUE_DETECT_MODE<=15 && GREEN_DETECT_MODE>=0 && GREEN_DETECT_MODE<=15 && AREA_CONTROL>=0 && AREA_CONTROL<=1 &&
  DETECT_MODE!=BLUE_DETECT_MODE && DETECT_MODE!=GREEN_DETECT_MODE && BLUE_DETECT_MODE!=GREEN_DETECT_MODE);
 localparam COMPLETE_LIMIT=TAIL_BUDGET-PUBLISH_GUARD;
 wire sof=I_enable && I_de && I_user;
 reg [3:0] frame_mode;
 reg [7:0] frame_th_min,frame_th_dom;
 reg [4:0] frame_area_min;
 reg [31:0] frame_id,epoch;
 reg have_frame,active;
 assign O_capture_frame_seq=sof?(frame_id+32'd1):frame_id;
 wire next_detect=(I_mode==DETECT_MODE || I_mode==BLUE_DETECT_MODE || I_mode==GREEN_DETECT_MODE);
 wire frame_detect=(frame_mode==DETECT_MODE || frame_mode==BLUE_DETECT_MODE || frame_mode==GREEN_DETECT_MODE);
 // Only the selected detector's thresholds matter. A same-mode threshold change
 // starts a new epoch and cannot publish a candidate computed under old values.
 wire config_changed=(I_mode!=frame_mode) ||
  (next_detect && (I_th_min!=frame_th_min || I_th_dom!=frame_th_dom ||
   (AREA_CONTROL && I_area_min!=frame_area_min)));
 wire [31:0] next_epoch=epoch+(config_changed?32'd1:32'd0);
 assign O_capture_epoch=sof?next_epoch:epoch;
 wire front_enable=I_enable && (sof?next_detect:(have_frame && frame_detect));
 wire [3:0] effective_mode=sof?I_mode:frame_mode;
 wire [7:0] effective_min=sof?I_th_min:frame_th_min;
 wire [7:0] effective_dom=sof?I_th_dom:frame_th_dom;
 wire [4:0] effective_area_gear=sof?I_area_min:frame_area_min;
 wire [19:0] effective_area=effective_area_gear>=16?20'd921600:(20'd16<<effective_area_gear);
 // Map selected channel into the existing classifier's R slot; keep display RGB
 // untouched and introduce no extra detector register or video-path latency.
 wire [23:0] classifier_rgb=(effective_mode==BLUE_DETECT_MODE)?
  {I_rgb[7:0],I_rgb[15:8],I_rgb[23:16]}:
  ((effective_mode==GREEN_DETECT_MODE)?{I_rgb[15:8],I_rgb[23:16],I_rgb[7:0]}:I_rgb);

 reg pending_ready,pending_fault;
 reg [2:0] pending_status;
 reg [11:0] pending_xmin,pending_xmax,pending_ymin,pending_ymax;
 reg [19:0] pending_area;
 reg [31:0] pending_id,pending_epoch;
 reg [7:0] settle;
 reg eof_seen;
 reg [15:0] tail_cycles,last_tail,max_row;
 wire tail_expired=active && eof_seen && !pending_ready && tail_cycles>=COMPLETE_LIMIT;
 reg [2:0] snapshot_status;
 reg [11:0] snapshot_xmin,snapshot_xmax,snapshot_ymin,snapshot_ymax;
 reg [19:0] snapshot_area;
 reg [31:0] snapshot_id,snapshot_epoch;
 reg [11:0] snapshot_center_x,snapshot_center_y;
 reg [15:0] snapshot_center_x_bcd,snapshot_center_y_bcd;
 wire coords_ready,coords_valid,coords_busy;
 wire [11:0] coords_x,coords_y;
 wire [15:0] coords_x_bcd,coords_y_bcd;
 wire [31:0] coords_source,coords_epoch;
 wire [3:0] coords_mode;
 wire coordinates_for_pending=coords_ready && coords_valid &&
  coords_source==pending_id && coords_epoch==pending_epoch && coords_mode==frame_mode;
 wire eligible=have_frame && frame_detect && next_detect && !config_changed &&
  // settle counts intervening edges; this SOF edge completes one more interval.
  // Including it avoids an extra off-by-one clock in the total tail budget.
  pending_ready && settle>=PUBLISH_GUARD-1 && pending_id==frame_id && pending_epoch==next_epoch &&
  (pending_status!=3'd3 || coordinates_for_pending);
 wire [2:0] boundary_status=!next_detect?3'd0:(!PARAM_OK?3'd7:
  (eligible?pending_status:((!have_frame || config_changed)?3'd1:3'd6)));
 wire boundary_valid=(boundary_status==3);
 assign O_detect=I_rst_n && I_enable && (sof?next_detect:(have_frame && frame_detect));
 assign O_status=(!I_rst_n || !I_enable)?3'd0:(sof?boundary_status:snapshot_status);
 assign O_target_valid=O_detect && O_status==3;
 assign O_xmin=O_target_valid?(sof?pending_xmin:snapshot_xmin):12'd0;
 assign O_xmax=O_target_valid?(sof?pending_xmax:snapshot_xmax):12'd0;
 assign O_ymin=O_target_valid?(sof?pending_ymin:snapshot_ymin):12'd0;
 assign O_ymax=O_target_valid?(sof?pending_ymax:snapshot_ymax):12'd0;
 assign O_area=O_target_valid?(sof?pending_area:snapshot_area):20'd0;
 // Box, binary centre and decimal digits are published on the SAME SOF edge.
 // A legal centre (0,0) is still valid; callers must use the explicit valid bit.
 assign O_coords_valid=O_target_valid;
 assign O_center_x=O_coords_valid?(sof?coords_x:snapshot_center_x):12'd0;
 assign O_center_y=O_coords_valid?(sof?coords_y:snapshot_center_y):12'd0;
 assign O_center_x_bcd=O_coords_valid?(sof?coords_x_bcd:snapshot_center_x_bcd):16'd0;
 assign O_center_y_bcd=O_coords_valid?(sof?coords_y_bcd:snapshot_center_y_bcd):16'd0;
 assign O_result_mode=(!I_rst_n || !I_enable)?4'd0:(sof?I_mode:frame_mode);
 // ID/epoch are meaningful only for published status 2..7, not OFF/WAITING.
 assign O_source_frame_id=(O_status<2)?32'd0:(sof?frame_id:snapshot_id);
 assign O_mode_epoch=(!I_rst_n || !I_enable)?32'd0:(sof?next_epoch:snapshot_epoch);
 assign O_tail_cycles=last_tail;
 assign O_max_row_cycles=max_row;
 assign O_pending_ready=pending_ready;

 wire ce,cd,cu,cl,cg,cv,cr,cs,cf,cb,cbad,cfg,front_cfg;
 assign cfg=front_cfg || (AREA_CONTROL && effective_area_gear>16);
 wire [11:0] cx,cy;
 wire rv,rn,done,failed,fault,frozen,rr,rootv,rd,ready,idle;
 wire [10:0] ra,rb;
 wire [9:0] ry,rl,labels,root;
 wire [2:0] error;
 wire [3:0] rank,qreads,maxreads,maxrank;
 wire [7:0] maxruns,max_label_cycles,capture_runs;
 wire [15:0] cycles,runs,scans,overlaps,reads;
 wire sreq,sready,sdone,sfailed,sfault,target,busy;
 wire [9:0] slabel,ymin,ymax;
 wire [10:0] xmin,xmax,regions,qualified;
 wire [19:0] area,total_area;
 wire [2:0] serror;
 assign O_frame_fault=I_rst_n && I_enable && frame_detect &&
  (pending_fault || cb || cbad || failed || sfailed || tail_expired || !PARAM_OK || cfg);
 // Raw SOF is one clock ahead of classified SOF. Clear OLD controllers here,
 // then accept the new classified first pixel next edge, including W=H=1.
 // This cancels late reductions without letting start_busy poison the new frame.
 // Guard is NOT cleared by SOF: premature SOF remains an input protocol error.
 // A pipelined mask may still carry the preceding frame's sticky fault
 // after RAW SOF. Keep controllers cleared until THIS classified SOF arrives.
 reg front_started;
 wire front_current=!EXTERNAL_MASK||front_started||cs;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)front_started<=0;
  else if(!I_enable||sof)front_started<=0;
  else if(cs)front_started<=1;
 end
 wire engine_enable=I_enable && ce && !sof && front_current && PARAM_OK && !tail_expired;
 red_mask_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.EXTERNAL_MASK(EXTERNAL_MASK)) front(
  .I_th_min(effective_min),.I_th_dom(effective_dom),
  .I_mask_de(I_mask_de),.I_mask_good(I_mask_good),.I_mask(I_mask),.I_mask_x(I_mask_x),.I_mask_y(I_mask_y),
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(front_enable),.I_de(I_de),
  .I_user(I_user),.I_last(I_last),.I_pixel_good(I_pixel_good),.I_rgb(classifier_rgb),.I_x(I_x),.I_y(I_y),
  .O_enable(ce),.O_de(cd),.O_user(cu),.O_last(cl),.O_pixel_good(cg),.O_valid(cv),.O_red(cr),
  .O_x(cx),.O_y(cy),.O_frame_start(cs),.O_frame_end(cf),.O_bad_event(cb),.O_frame_bad(cbad),.O_config_bad(front_cfg));
 red_cca_core #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.ROW_BUDGET(ROW_BUDGET)) core(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(engine_enable),.I_valid(cv),.I_red(cr),.I_user(cu),.I_last(cl),.I_x(cx),.I_y(cy),
  .I_frame_start(cs),.I_frame_end(cf),.I_bad_event(cb),.I_frame_bad(cbad || (sfailed && !cs)),.I_config_bad(cfg),
  .I_run_ready(ready),.I_features_idle(idle),.O_run_valid(rv),.O_run_xmin(ra),.O_run_xmax(rb),.O_run_y(ry),.O_run_label(rl),.O_run_new(rn),
  .O_done(done),.O_failed(failed),.O_fault(fault),.O_error(error),.O_frozen(frozen),.O_labels(labels),
  .I_root_req(sreq),.I_root_label(slabel),.O_root_ready(rr),.O_root_valid(rootv),.O_root(root),.O_root_rank(rank),.O_root_reads(qreads),
  .O_row_done(rd),.O_row_cycles(cycles),.O_row_runs(runs),.O_row_scans(scans),.O_row_overlaps(overlaps),.O_row_find_reads(reads),
  .O_max_find_reads(maxreads),.O_max_rank(maxrank),.O_max_runs(maxruns),.O_capture_runs(capture_runs));
 red_cca_stats #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.MIN_AREA(MIN_AREA),.MIN_WIDTH(MIN_WIDTH),.MIN_HEIGHT(MIN_HEIGHT),.FOLD_BUDGET(FOLD_BUDGET),.RUNTIME_MIN(AREA_CONTROL)) stats(
  .I_min_area(effective_area),
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(engine_enable),.I_frame_start(cs),.I_bad_event(cb),.I_frame_bad(cbad),.I_config_bad(cfg),
  .I_core_failed(failed),.I_core_error(error),.I_core_done(done),.I_core_frozen(frozen),.I_labels(labels),
  .I_run_valid(rv),.I_run_new(rn),.I_run_xmin(ra),.I_run_xmax(rb),.I_run_y(ry),.I_run_label(rl),
  .O_run_ready(ready),.O_features_idle(idle),.O_root_req(sreq),.O_root_label(slabel),.I_root_ready(rr),.I_root_valid(rootv),.I_root(root),.I_root_reads(qreads),
  .O_result_ready(sready),.O_done(sdone),.O_failed(sfailed),.O_fault(sfault),.O_error(serror),.O_target_valid(target),
  .O_xmin(xmin),.O_xmax(xmax),.O_ymin(ymin),.O_ymax(ymax),.O_area(area),.O_total_area(total_area),
  .O_regions(regions),.O_qualified(qualified),.O_fold_cycles(O_fold_cycles),.O_max_label_cycles(max_label_cycles),.O_busy(busy));

 // Start once per successful reduction, not once per pixel. Twelve conversion
 // clocks fit inside the existing default 32-clock publication guard. If a
 // custom short interval is insufficient, eligibility returns TIMEOUT instead
 // of publishing a box with unfinished digits. Raw SOF cancels old conversion
 // AFTER the same-edge output bypass has been sampled by the next consumer.
 wire coords_cancel=!I_enable || sof || pending_fault || failed || sfailed || cb || cbad ||
  tail_expired || !PARAM_OK || cfg;
 wire coords_start=active && !pending_fault && !pending_ready && eof_seen && sdone && sready && target && !coords_cancel;
 rgb_bbox_coordinates #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) coordinate_converter(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(coords_cancel),.I_start(coords_start),
  .I_xmin({1'b0,xmin}),.I_xmax({1'b0,xmax}),.I_ymin({2'b0,ymin}),.I_ymax({2'b0,ymax}),
  .I_source_frame_id(frame_id),.I_config_epoch(epoch),.I_mode(frame_mode),
  .O_ready(coords_ready),.O_valid(coords_valid),.O_busy(coords_busy),
  .O_x(coords_x),.O_y(coords_y),.O_x_bcd(coords_x_bcd),.O_y_bcd(coords_y_bcd),
  .O_source_frame_id(coords_source),.O_config_epoch(coords_epoch),.O_mode(coords_mode));

 // Registered observer event for the PREVIOUS frame, even on a mode change.
 // It never feeds the annotation, classifier, timing budgets or error policy.
 reg [15:0] diag_frame_row_peak;
 wire diag_complete=pending_ready&&settle>=PUBLISH_GUARD-1&&
  pending_id==frame_id&&pending_epoch==epoch&&
  (pending_status!=3'd3||coordinates_for_pending);
 wire [2:0] diag_status=!PARAM_OK?3'd7:(diag_complete?pending_status:3'd6);
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   O_diag_valid<=0;O_diag_capture_seq<=0;O_diag_epoch<=0;O_diag_mode<=0;
   O_diag_status<=0;O_diag_error<=0;O_diag_max_runs<=0;O_diag_labels<=0;
   O_diag_max_row_cycles<=0;O_diag_fold_cycles<=0;O_diag_tail_cycles<=0;diag_frame_row_peak<=0;
  end else begin
   O_diag_valid<=0;
   if(!I_enable)diag_frame_row_peak<=0;
   else if(sof)begin
    diag_frame_row_peak<=0;
    if(have_frame&&frame_detect)begin
     O_diag_valid<=1;O_diag_capture_seq<=frame_id;O_diag_epoch<=epoch;O_diag_mode<=frame_mode;
     O_diag_status<=diag_status;
     O_diag_error<=failed?error:sfailed?serror:(diag_status>=4?diag_status:3'd0);
     O_diag_max_runs<=capture_runs;O_diag_labels<=labels;
     O_diag_max_row_cycles<=diag_frame_row_peak;O_diag_fold_cycles<=O_fold_cycles;
     O_diag_tail_cycles<=diag_complete?last_tail:tail_cycles;
    end
   end else if(rd&&cycles>diag_frame_row_peak)diag_frame_row_peak<=cycles;
  end
 end

 task pending_failure;
  input [2:0] code;
  begin
   pending_ready<=1;pending_fault<=1;pending_status<=code;settle<=0;
   pending_xmin<=0;pending_xmax<=0;pending_ymin<=0;pending_ymax<=0;pending_area<=0;
   pending_id<=frame_id;pending_epoch<=epoch;last_tail<=tail_cycles;
  end
 endtask
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   frame_mode<=0;frame_th_min<=0;frame_th_dom<=0;frame_area_min<=2;frame_id<=0;epoch<=0;have_frame<=0;active<=0;
   pending_ready<=0;pending_fault<=0;pending_status<=0;settle<=0;eof_seen<=0;tail_cycles<=0;last_tail<=0;max_row<=0;
   pending_xmin<=0;pending_xmax<=0;pending_ymin<=0;pending_ymax<=0;pending_area<=0;pending_id<=0;pending_epoch<=0;
   snapshot_status<=0;snapshot_xmin<=0;snapshot_xmax<=0;snapshot_ymin<=0;snapshot_ymax<=0;snapshot_area<=0;snapshot_id<=0;snapshot_epoch<=0;
   snapshot_center_x<=0;snapshot_center_y<=0;snapshot_center_x_bcd<=0;snapshot_center_y_bcd<=0;
  end else if(!I_enable) begin
   frame_mode<=0;frame_th_min<=0;frame_th_dom<=0;have_frame<=0;active<=0;pending_ready<=0;pending_fault<=0;settle<=0;eof_seen<=0;tail_cycles<=0;
   snapshot_status<=0;snapshot_xmin<=0;snapshot_xmax<=0;snapshot_ymin<=0;snapshot_ymax<=0;snapshot_area<=0;snapshot_id<=0;snapshot_epoch<=epoch;
   snapshot_center_x<=0;snapshot_center_y<=0;snapshot_center_x_bcd<=0;snapshot_center_y_bcd<=0;
  end else if(sof) begin
   snapshot_status<=boundary_status;snapshot_id<=frame_id;snapshot_epoch<=next_epoch;
   snapshot_xmin<=boundary_valid?pending_xmin:12'd0;snapshot_xmax<=boundary_valid?pending_xmax:12'd0;
   snapshot_ymin<=boundary_valid?pending_ymin:12'd0;snapshot_ymax<=boundary_valid?pending_ymax:12'd0;
   snapshot_area<=boundary_valid?pending_area:20'd0;
   snapshot_center_x<=boundary_valid?coords_x:12'd0;snapshot_center_y<=boundary_valid?coords_y:12'd0;
   snapshot_center_x_bcd<=boundary_valid?coords_x_bcd:16'd0;snapshot_center_y_bcd<=boundary_valid?coords_y_bcd:16'd0;
   frame_mode<=I_mode;frame_th_min<=I_th_min;frame_th_dom<=I_th_dom;
   frame_area_min<=I_area_min;
   frame_id<=frame_id+32'd1;epoch<=next_epoch;have_frame<=1;active<=next_detect;
   pending_ready<=0;pending_fault<=0;settle<=0;
   eof_seen<=next_detect && I_x==WIDTH-1 && I_y==HEIGHT-1;tail_cycles<=0;
   pending_id<=frame_id+32'd1;pending_epoch<=next_epoch;
  end else if(active) begin
   if(pending_ready && settle<255) settle<=settle+8'd1;
   if(I_de && I_x==WIDTH-1 && I_y==HEIGHT-1) begin eof_seen<=1;tail_cycles<=0;end
   else if(eof_seen && !pending_ready && tail_cycles<65535) tail_cycles<=tail_cycles+16'd1;
   if(rd && cycles>max_row) max_row<=cycles;
   // A late blanking protocol fault invalidates an already completed candidate.
   // First fault is sticky, but an earlier success is never allowed to mask it.
   if(!pending_fault) begin
    if(!PARAM_OK || cfg) pending_failure(3'd7);
    else if(front_current) begin
    if(cb || cbad) pending_failure(3'd5);
    else if(sfailed) pending_failure(serror==0?3'd7:serror);
    else if(failed) pending_failure(error==0?3'd7:error);
    else if(tail_expired) pending_failure(3'd6);
    else if(pending_ready && pending_status==3'd3 && coords_ready && !coords_valid) pending_failure(3'd7);
    else if(sdone && sready && !pending_ready) begin
     if(!eof_seen) pending_failure(3'd7);
     else begin
      pending_ready<=1;pending_status<=target?3'd3:3'd2;settle<=0;
      pending_id<=frame_id;pending_epoch<=epoch;last_tail<=tail_cycles;
      pending_xmin<=target?{1'b0,xmin}:12'd0;pending_xmax<=target?{1'b0,xmax}:12'd0;
      pending_ymin<=target?{2'b0,ymin}:12'd0;pending_ymax<=target?{2'b0,ymax}:12'd0;pending_area<=target?area:20'd0;
     end
    end
    end // front_current
   end
  end
 end
endmodule

// One bounded conversion per result, double-dabble on two 12-bit coordinates.
// Kept in the already-listed stream source: no project/constraint/pin changes.
module rgb_bbox_coordinates #(
 parameter integer WIDTH=1280,HEIGHT=720
)(
 input wire I_clk,I_rst_n,I_cancel,I_start,
 input wire [11:0] I_xmin,I_xmax,I_ymin,I_ymax,
 input wire [31:0] I_source_frame_id,I_config_epoch,
 input wire [3:0] I_mode,
 output reg O_ready,O_valid,
 output wire O_busy,
 output reg [11:0] O_x,O_y,
 output reg [15:0] O_x_bcd,O_y_bcd,
 output reg [31:0] O_source_frame_id,O_config_epoch,
 output reg [3:0] O_mode
);
 localparam PARAM_OK=(WIDTH>=1 && WIDTH<=1280 && HEIGHT>=1 && HEIGHT<=720);
 wire box_ok=PARAM_OK && I_xmin<=I_xmax && I_ymin<=I_ymax && I_xmax<WIDTH && I_ymax<HEIGHT;
 wire [12:0] sum_x={1'b0,I_xmin}+{1'b0,I_xmax};
 wire [12:0] sum_y={1'b0,I_ymin}+{1'b0,I_ymax};
 reg [11:0] binary_x,binary_y;
 reg [15:0] work_x,work_y;
 reg [3:0] bits_left;
 reg [15:0] adjusted_x,adjusted_y;
 integer digit;
 assign O_busy=(bits_left!=0);
 always @* begin
  adjusted_x=work_x;adjusted_y=work_y;
  for(digit=0;digit<4;digit=digit+1) begin
   if(work_x[4*digit +: 4]>=4'd5) adjusted_x[4*digit +: 4]=work_x[4*digit +: 4]+4'd3;
   if(work_y[4*digit +: 4]>=4'd5) adjusted_y[4*digit +: 4]=work_y[4*digit +: 4]+4'd3;
  end
 end
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   bits_left<=0;binary_x<=0;binary_y<=0;work_x<=0;work_y<=0;
   O_ready<=0;O_valid<=0;O_x<=0;O_y<=0;O_x_bcd<=0;O_y_bcd<=0;
   O_source_frame_id<=0;O_config_epoch<=0;O_mode<=0;
  end else if(I_cancel) begin
   bits_left<=0;binary_x<=0;binary_y<=0;work_x<=0;work_y<=0;
   O_ready<=0;O_valid<=0;O_x<=0;O_y<=0;O_x_bcd<=0;O_y_bcd<=0;
   O_source_frame_id<=0;O_config_epoch<=0;O_mode<=0;
  end else if(I_start) begin
   bits_left<=box_ok?4'd12:4'd0;
   binary_x<=box_ok?sum_x[12:1]:12'd0;binary_y<=box_ok?sum_y[12:1]:12'd0;
   work_x<=0;work_y<=0;O_x_bcd<=0;O_y_bcd<=0;
   O_ready<=!box_ok;O_valid<=0;
   O_x<=box_ok?sum_x[12:1]:12'd0;O_y<=box_ok?sum_y[12:1]:12'd0;
   O_source_frame_id<=I_source_frame_id;O_config_epoch<=I_config_epoch;O_mode<=I_mode;
  end else if(bits_left!=0) begin
   work_x<={adjusted_x[14:0],binary_x[11]};work_y<={adjusted_y[14:0],binary_y[11]};
   binary_x<={binary_x[10:0],1'b0};binary_y<={binary_y[10:0],1'b0};
   bits_left<=bits_left-4'd1;
   if(bits_left==1) begin
    O_x_bcd<={adjusted_x[14:0],binary_x[11]};O_y_bcd<={adjusted_y[14:0],binary_y[11]};
    O_ready<=1;O_valid<=1;
   end
  end
 end
endmodule
