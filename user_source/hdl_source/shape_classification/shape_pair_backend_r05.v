// R05 paired-mask scheduler. NOT a new pixel/CDC pipeline.
// Raw SOF grants BOTH captures one common bank/source/epoch. Capture streams
// must be registered, sequential, with EOF on their final write. Complete pairs
// alone become READY. A bank remains owned from acquisition to ink completion.
// A busy two-bank queue drops a WHOLE new capture, never cancels a valid job.
// PAPER_REQUIRED=0: full-frame black-pattern jobs. Paper ports/metadata remain
// explicitly absent; the temporal input is an accepted analysis-region flag.
// No synthetic paper detection, new pixel delay, or memory read latency.
module shape_pair_backend_r05 #(parameter integer PAPER_REQUIRED=1,MULTI_TARGETS=1)(
 input wire I_clk,I_rst_n,I_enable,I_cancel,I_sof,I_frame_tick,
 input wire [31:0] I_source,
 input wire [7:0] I_gray,I_paper_threshold,
 input wire [1:0] I_profile,
 input wire [4:0] I_area_min,I_area_max,
 input wire I_ink_write,I_ink_bit,I_ink_eof,I_ink_bad,
 input wire [15:0] I_ink_addr,
 input wire I_paper_write,I_paper_bit,I_paper_eof,I_paper_bad,
 input wire [13:0] I_paper_addr,
 output wire O_capture_enable,O_fault,O_busy,
 output reg O_capture_drop,O_result_valid,O_result_paper,
 output reg [2:0] O_result_status,
 output reg O_result_target,
 output reg [1:0] O_result_class,O_result_profile,
 output reg [3:0] O_result_reason,
 output reg O_result_retry,
 output reg [8:0] O_result_l,O_result_r,
 output reg [7:0] O_result_t,O_result_b,
 output reg [15:0] O_result_area,
 output reg [31:0] O_result_seq,O_result_source,O_result_epoch,
 output wire [248:0] O_multi_packets,
 output wire O_valid,
 output wire [2:0] O_status,
 output wire [1:0] O_class,
 output wire [11:0] O_xmin,O_ymin,O_xmax,O_ymax,O_center_x,O_center_y,
 output wire [31:0] O_seq,O_source,O_epoch,
 output wire [31:0] O_context_epoch,
 output reg [20:0] O_job_cycles,
 output wire [19:0] O_diag_code,
 output wire O_diag_pvalid,O_diag_kvalid,
 output wire [33:0] O_diag_pbox,O_diag_kbox,
 output wire [31:0] O_diag_source,O_diag_epoch
);
 localparam IDLE=3'd0,P_START=3'd1,P_WAIT=3'd2,K_START=3'd3,K_WAIT=3'd4;
 reg [2:0] state;
 reg cfg_enable;
 reg [7:0] cfg_gray,cfg_paper;
 reg [1:0] cfg_profile;
 reg [4:0] cfg_min,cfg_max,job_min,job_max;
 reg [31:0] epoch,seq;
 reg cap_owned,cap_bank,ink_complete,paper_complete;
 reg [15:0] ink_count;
 reg [13:0] paper_count;
 reg [1:0] ready;
 reg [31:0] bank_seq[0:1],bank_source[0:1],bank_epoch[0:1];
 reg [1:0] bank_profile[0:1];
 reg job_bank;
 reg [31:0] job_seq,job_source,job_epoch;
 reg [1:0] job_profile;
 wire config_change=I_enable!=cfg_enable||I_gray!=cfg_gray||
  (PAPER_REQUIRED!=0 && I_paper_threshold!=cfg_paper)||I_profile!=cfg_profile||I_area_min!=cfg_min||I_area_max!=cfg_max;
 wire ink_bad=cap_owned&&((I_ink_write&&(ink_complete||I_ink_addr!=ink_count||ink_count>=57600))||
  (I_ink_eof&&(!I_ink_write||ink_count!=57599||I_ink_addr!=57599)));
 wire paper_bad=(PAPER_REQUIRED!=0)&&cap_owned&&((I_paper_write&&(paper_complete||I_paper_addr!=paper_count||paper_count>=14400))||
  (I_paper_eof&&(!I_paper_write||paper_count!=14399||I_paper_addr!=14399)));
 wire cap_fault=I_enable&&(I_ink_bad||(PAPER_REQUIRED!=0 && I_paper_bad)||ink_bad||paper_bad||(I_sof&&cap_owned));
 wire parameter_bad=MULTI_TARGETS<1||MULTI_TARGETS>3||(MULTI_TARGETS>1&&PAPER_REQUIRED!=0)||(PAPER_REQUIRED!=0 && PAPER_REQUIRED!=1)||I_profile==3||I_area_min>I_area_max||I_area_max>16;
 wire flush=!I_enable||I_cancel||config_change||cap_fault||parameter_bad;
 wire held=state!=IDLE;
 wire free0=!ready[0]&&!(held&&!job_bank)&&!(cap_owned&&!cap_bank);
 wire free1=!ready[1]&&!(held&&job_bank)&&!(cap_owned&&cap_bank);
 wire admit=I_enable&&!I_cancel&&!config_change&&!parameter_bad;
 wire grant=I_sof&&admit&&!cap_owned&&(free0||free1);
 // On SOF this combinational enable lets both true pixel captures see pixel 0.
 // A registered-only grant would lose the first pixel.
 // Do not feed capture bad_event back into its own enable combinationally.
 // Bad frame blocks RAM writes now; ownership/enable clear on this edge.
 assign O_capture_enable=admit&&(cap_owned||grant);
 assign O_fault=I_enable&&(I_cancel||cap_fault||parameter_bad);
 assign O_busy=held||cap_owned||ready!=0;
 assign O_context_epoch=epoch;
 wire older0=$unsigned(bank_seq[1]-bank_seq[0])<32'h80000000;
 wire next_bank=ready[0]&&ready[1]?!older0:!ready[0];
 wire p_re,p_we,p_bit,k_re,k_we,k_bit0,k_bit1,p_bit0,p_bit1;
 wire [15:0] p_addr,k_addr;
 wire p_busy,p_done,p_failed,p_target;
 wire [2:0] p_error;
 wire [7:0] p_top,p_bottom;
 wire [8:0] p_left,p_right;
 wire [18:0] p_span;
 wire [31:0] p_source,p_epoch;
 wire k_busy,k_done,k_failed,k_target;
 wire [2:0] k_error;
 wire [1:0] k_class,k_profile;
 wire [3:0] k_reason;
 wire k_retry;
 wire [8:0] k_l,k_r;
 wire [7:0] k_t,k_b;
 wire [15:0] k_area;
 wire [20:0] k_cycles;
 wire [8:0] request_x;
 wire [7:0] request_y;
 reg [8:0] read_x;
 reg [7:0] read_y;
 // k_addr stays stable across the synchronous RAM request. Register x/y on
 // the same edge as mask and paper-row reads; never query a live video y.
 always @(posedge I_clk)if(k_re)begin read_x<=request_x;read_y<=request_y;end
 // Explicit request coordinates, not hierarchy or a large constant divider.
 wire roi_bit;
 generate if(PAPER_REQUIRED!=0)begin: paper_region
 shape_paper_roi_gate roi(.I_paper_valid(p_target),.I_paper_busy(p_busy),
  .I_ink(job_bank?k_bit1:k_bit0),.I_span(p_span),.I_ink_x(read_x),.I_ink_y(read_y),
  .I_paper_ymin(p_top),.I_paper_ymax(p_bottom),
  .I_ink_source(job_source),.I_ink_epoch(job_epoch),.I_paper_source(p_source),.I_paper_epoch(p_epoch),.O_ink(roi_bit));
 shape_paper_locator paper(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(flush),.I_start(state==P_START),
  .O_mem_re(p_re),.O_mem_we(p_we),.O_mem_addr(p_addr),.I_mem_bit(p_bit),
  .I_source(job_source),.I_epoch(job_epoch),.I_roi_row(request_y[7:1]),
  .O_roi_span(p_span),.O_source(p_source),.O_epoch(p_epoch),
  .O_busy(p_busy),.O_done(p_done),.O_failed(p_failed),.O_target_valid(p_target),
  .O_error(p_error),.O_class(),.O_xmin(p_left),.O_xmax(p_right),.O_ymin(p_top),.O_ymax(p_bottom),
  .O_area(),.O_cycles(),.O_queue_peak(),.O_components());
 end else begin: full_frame_region
  assign roi_bit=job_bank?k_bit1:k_bit0;
  assign p_re=1'b0;assign p_we=1'b0;assign p_addr=16'd0;
  assign p_busy=1'b0;assign p_done=1'b0;assign p_failed=1'b0;
  assign p_target=1'b0;assign p_error=3'd0;
  assign p_top=8'd0;assign p_bottom=8'd0;
  assign p_left=9'd0;assign p_right=9'd0;assign p_span=19'd0;
  assign p_source=32'd0;assign p_epoch=32'd0;
 end endgenerate
 wire [2:0] k_present;wire[5:0]k_classes;wire[101:0]k_boxes;
 reg[2:0] result_present;reg[5:0]result_classes;reg[101:0]result_boxes;
 generate if(MULTI_TARGETS==1)begin: single_target
 shape_mask_engine_r05 ink_engine(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(flush),.I_start(state==K_START),
  .O_mem_re(k_re),.O_mem_we(k_we),.O_mem_addr(k_addr),.O_mem_x(request_x),.O_mem_y(request_y),.I_mem_bit(roi_bit),
  .O_busy(k_busy),.O_done(k_done),.O_failed(k_failed),.O_target_valid(k_target),
  .O_error(k_error),.O_class(k_class),.O_xmin(k_l),.O_xmax(k_r),.O_ymin(k_t),.O_ymax(k_b),
  .O_area(k_area),.O_cycles(k_cycles),.O_queue_peak(),.O_components(),.I_profile(job_profile),
  .I_area_min(job_min),.I_area_max(job_max),
  .O_profile(k_profile),.O_reason(k_reason),.O_retry(k_retry),
  .O_vertices(),.O_hull_vertices(),.O_center_area2(),.O_cell_area2(),.O_poly_area2());
 assign k_present=0;assign k_classes=0;assign k_boxes=0;assign O_multi_packets=0;
 end else begin: multi_target
 shape_mask_engine_multi #(.MAX_TARGETS(MULTI_TARGETS)) ink_engine(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(flush),.I_start(state==K_START),
  .O_mem_re(k_re),.O_mem_we(k_we),.O_mem_addr(k_addr),.O_mem_x(request_x),.O_mem_y(request_y),.I_mem_bit(roi_bit),
  .O_present(k_present),.O_classes(k_classes),.O_boxes(k_boxes),.O_areas(),.O_busy(k_busy),.O_done(k_done),.O_failed(k_failed),.O_target_valid(k_target),
  .O_error(k_error),.O_class(k_class),.O_xmin(k_l),.O_xmax(k_r),.O_ymin(k_t),.O_ymax(k_b),
  .O_area(k_area),.O_cycles(k_cycles),.O_queue_peak(),.O_components(),.I_profile(job_profile),
  .I_area_min(job_min),.I_area_max(job_max),
  .O_profile(k_profile),.O_reason(k_reason),.O_retry(k_retry),
  .O_vertices(),.O_hull_vertices(),.O_center_area2(),.O_cell_area2(),.O_poly_area2());
 shape_list_filter tracking(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable),.I_cancel(config_change),.I_fault(O_fault),
  .I_frame_tick(I_frame_tick),.I_expected_epoch(epoch),.I_result_valid(O_result_valid),
  .I_seq(O_result_seq),.I_source(O_result_source),.I_epoch(O_result_epoch),.I_status(O_result_status),
  .I_present(result_present),.I_classes(result_classes),.I_boxes(result_boxes),.O_packets(O_multi_packets),.O_sources(),.O_fault());
 end endgenerate
 wire cap_kw=cap_owned&&I_ink_write&&!flush,cap_pw=(PAPER_REQUIRED!=0)&&cap_owned&&I_paper_write&&!flush;
 wire safe_kw=k_we&&!flush,safe_pw=p_we&&!flush;
 shape_bit_bank ink0(I_clk,(cap_kw&&!cap_bank)||(safe_kw&&!job_bank),k_re&&!job_bank,
  cap_kw&&!cap_bank?I_ink_addr:k_addr,k_addr,cap_kw&&!cap_bank?I_ink_bit:1'b0,k_bit0);
 shape_bit_bank ink1(I_clk,(cap_kw&&cap_bank)||(safe_kw&&job_bank),k_re&&job_bank,
  cap_kw&&cap_bank?I_ink_addr:k_addr,k_addr,cap_kw&&cap_bank?I_ink_bit:1'b0,k_bit1);
 generate if(PAPER_REQUIRED!=0)begin: paper_storage
 shape_paper_bank_r05 paper0(I_clk,(cap_pw&&!cap_bank)||(safe_pw&&!job_bank),p_re&&!job_bank,
  cap_pw&&!cap_bank?I_paper_addr:p_addr[13:0],p_addr[13:0],cap_pw&&!cap_bank?I_paper_bit:1'b0,p_bit0);
 shape_paper_bank_r05 paper1(I_clk,(cap_pw&&cap_bank)||(safe_pw&&job_bank),p_re&&job_bank,
  cap_pw&&cap_bank?I_paper_addr:p_addr[13:0],p_addr[13:0],cap_pw&&cap_bank?I_paper_bit:1'b0,p_bit1);
 end else begin: no_paper_storage
  assign p_bit0=1'b0;assign p_bit1=1'b0;
 end endgenerate
 assign p_bit=job_bank?p_bit1:p_bit0;
 shape_temporal_r04 temporal(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable),
  .I_cancel(config_change),.I_fault(O_fault),.I_frame_tick(I_frame_tick),.I_expected_epoch(epoch),
  .I_result_valid(O_result_valid),
  // Legacy name denotes valid search region here, not evidence of paper.
  .I_paper_valid(PAPER_REQUIRED!=0?O_result_paper:1'b1),.I_status(O_result_status),
  .I_target_valid(O_result_target),.I_class(O_result_class),
  .I_xmin({1'b0,O_result_l,2'b00}),.I_xmax({1'b0,O_result_r,2'b11}),
  .I_ymin({2'd0,O_result_t,2'b00}),.I_ymax({2'd0,O_result_b,2'b11}),
  .I_area(O_result_area),.I_seq(O_result_seq),.I_source(O_result_source),.I_epoch(O_result_epoch),
  .I_profile(O_result_profile),.I_reason(O_result_reason),.I_retry(O_result_retry),
  .O_update(),.O_valid(O_valid),.O_status(O_status),.O_class(O_class),
  .O_xmin(O_xmin),.O_ymin(O_ymin),.O_xmax(O_xmax),.O_ymax(O_ymax),
  .O_center_x(O_center_x),.O_center_y(O_center_y),.O_area(),.O_seq(O_seq),.O_source(O_source),.O_epoch(O_epoch),
  .O_profile(),.O_reason(),.O_retry());

 // Observation only: no diagnostic signal feeds scheduling or recognition.
 wire [11:0] diagnostic_fault=parameter_bad?12'h401:I_cancel?12'h101:
  (I_ink_bad||ink_bad)?12'h102:(I_paper_bad||paper_bad)?12'h103:12'h104;
 shape_diagnostic_snapshot #(.PAPER_REQUIRED(PAPER_REQUIRED)) diagnostics(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable),.I_clear(config_change),
  .I_frame_tick(I_frame_tick),.I_fault(O_fault),.I_fault_code(diagnostic_fault),
  .I_source(held?job_source:I_source),.I_epoch(epoch),
  .I_pdone(state==P_WAIT&&p_done),.I_pfailed(p_failed),.I_ptarget(p_target),.I_perror(p_error),
  .I_pbox({p_left,p_right,p_top,p_bottom}),
  .I_pcontext_ok(p_source==job_source&&p_epoch==job_epoch),
  .I_kdone(state==K_WAIT&&k_done),.I_kfailed(k_failed),.I_ktarget(k_target),
  .I_kerror(k_error),.I_class(k_class),.I_reason(k_reason),.I_kbox({k_l,k_r,k_t,k_b}),
  .O_code(O_diag_code),.O_pvalid(O_diag_pvalid),.O_kvalid(O_diag_kvalid),
  .O_pbox(O_diag_pbox),.O_kbox(O_diag_kbox),.O_source(O_diag_source),.O_epoch(O_diag_epoch));
 task clear_result;
  begin result_present<=0;result_classes<=0;result_boxes<=0;O_result_valid<=0;O_result_paper<=0;O_result_status<=0;O_result_target<=0;
   O_result_class<=0;O_result_profile<=0;O_result_reason<=0;O_result_retry<=0;
   O_result_l<=0;O_result_r<=0;O_result_t<=0;O_result_b<=0;O_result_area<=0;
   O_result_seq<=0;O_result_source<=0;O_result_epoch<=0;end
 endtask
 integer n;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   state<=IDLE;cfg_enable<=0;cfg_gray<=128;cfg_paper<=160;cfg_profile<=1;epoch<=0;seq<=0;
   cfg_min<=6;cfg_max<=16;job_min<=6;job_max<=16;
   cap_owned<=0;cap_bank<=0;ink_complete<=0;paper_complete<=0;ink_count<=0;paper_count<=0;ready<=0;
   job_bank<=0;job_seq<=0;job_source<=0;job_epoch<=0;job_profile<=0;O_capture_drop<=0;O_job_cycles<=0;
   for(n=0;n<2;n=n+1)begin bank_seq[n]<=0;bank_source[n]<=0;bank_epoch[n]<=0;bank_profile[n]<=0;end
   clear_result;
  end else begin
   cfg_enable<=I_enable;cfg_gray<=I_gray;cfg_paper<=I_paper_threshold;cfg_profile<=I_profile;
   cfg_min<=I_area_min;cfg_max<=I_area_max;
   if(config_change)epoch<=epoch+1'b1;
   if(I_sof&&I_enable)seq<=seq+1'b1;
   O_capture_drop<=I_sof&&I_enable&&!flush&&!(free0||free1);
   O_result_valid<=0;
   if(flush)begin state<=IDLE;cap_owned<=0;ready<=0;ink_count<=0;paper_count<=0;
    ink_complete<=0;paper_complete<=0;O_job_cycles<=0;clear_result;end
   else begin
    if(grant)begin
     cap_owned<=1;cap_bank<=!free0;ink_count<=0;paper_count<=0;ink_complete<=0;paper_complete<=0;
     bank_seq[free0?0:1]<=seq+1'b1;bank_source[free0?0:1]<=I_source;
     bank_epoch[free0?0:1]<=epoch;bank_profile[free0?0:1]<=I_profile;
    end
    if(cap_kw)ink_count<=ink_count+1'b1;
    if(cap_pw)paper_count<=paper_count+1'b1;
    if(cap_owned&&I_ink_eof)ink_complete<=1;
    if(cap_owned&&I_paper_eof)paper_complete<=1;
    if(cap_owned&&(ink_complete||I_ink_eof)&&(PAPER_REQUIRED==0||paper_complete||I_paper_eof))begin
     cap_owned<=0;ready[cap_bank]<=1;
    end
    if(held)O_job_cycles<=O_job_cycles+1'b1;
    case(state)
     IDLE:if(ready!=0)begin
      job_bank<=next_bank;ready[next_bank]<=0;job_seq<=bank_seq[next_bank];job_source<=bank_source[next_bank];
      job_epoch<=bank_epoch[next_bank];job_profile<=bank_profile[next_bank];O_job_cycles<=0;state<=PAPER_REQUIRED!=0?P_START:K_START;
      job_min<=cfg_min;job_max<=cfg_max;
     end
     P_START:state<=P_WAIT;
     P_WAIT:if(p_done)begin
      if(!p_failed&&p_target)state<=K_START;
      else begin
       clear_result;O_result_valid<=1;O_result_status<=p_failed?3'd3:3'd2;
       O_result_reason<=p_failed?4'd15:4'd0;O_result_seq<=job_seq;O_result_source<=job_source;
       O_result_epoch<=job_epoch;O_result_profile<=job_profile;state<=IDLE;
      end
     end
     K_START:state<=K_WAIT;
     K_WAIT:if(k_done)begin
      result_present<=k_failed?3'd0:k_present;result_classes<=k_failed?6'd0:k_classes;result_boxes<=k_failed?102'd0:k_boxes;
      O_result_valid<=1;O_result_paper<=p_target;O_result_status<=k_failed?3'd3:k_target?3'd1:3'd2;
      O_result_target<=!k_failed&&k_target;O_result_class<=!k_failed&&k_target?k_class:2'd0;
      O_result_l<=k_l;O_result_r<=k_r;O_result_t<=k_t;O_result_b<=k_b;O_result_area<=k_area;
      O_result_seq<=job_seq;O_result_source<=job_source;O_result_epoch<=job_epoch;
      O_result_profile<=k_profile;O_result_reason<=k_failed?4'd15:k_reason;O_result_retry<=k_retry;state<=IDLE;
     end
     default:state<=IDLE;
    endcase
   end
  end
 end
endmodule

// Diagnostic task snapshot only. Payload reset/clear prevents stale boxes;
// all normal updates publish at a safe blanking boundary. Faults revoke now.
module shape_diagnostic_snapshot #(parameter integer PAPER_REQUIRED=1)(
 input wire I_clk,I_rst_n,I_enable,I_clear,I_frame_tick,I_fault,
 input wire [11:0] I_fault_code,input wire [31:0] I_source,I_epoch,
 input wire I_pdone,I_pfailed,I_ptarget,I_pcontext_ok,input wire [2:0] I_perror,
 input wire [33:0] I_pbox,
 input wire I_kdone,I_kfailed,I_ktarget,input wire [2:0] I_kerror,
 input wire [1:0] I_class,input wire [3:0] I_reason,input wire [33:0] I_kbox,
 output wire [19:0] O_code,output wire O_pvalid,O_kvalid,
 output wire [33:0] O_pbox,O_kbox,output wire [31:0] O_source,O_epoch
);
 reg [153:0] pending,visible;
 reg dirty;reg [2:0] age;
 // {P(4), K(4), E(stage4/detail8), pvalid, kvalid, pbox34, kbox34, source32, epoch32}
 assign {O_code,O_pvalid,O_kvalid,O_pbox,O_kbox,O_source,O_epoch}=
  (!I_rst_n||!I_enable||I_clear)?154'd0:visible;
 wire p_ok=!I_pfailed&&I_ptarget&&I_pcontext_ok;
 wire [19:0] p_code=I_pfailed?{4'd3,4'd0,4'd2,5'd0,I_perror}:
  !I_ptarget?20'h10000:!I_pcontext_ok?20'h30601:20'h20000;
 wire [3:0] k_state=I_kfailed?4'd4:!I_ktarget?4'd1:I_class==0?4'd2:4'd3;
 wire [11:0] k_reason=I_kfailed?{4'd3,5'd0,I_kerror}:
  (I_ktarget&&I_class==0)?{8'd0,I_reason}:12'd0;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin pending<=0;visible<=0;dirty<=0;age<=0;end
  else if(!I_enable||I_clear)begin pending<=0;visible<=0;dirty<=0;age<=0;end
  else if(I_fault)begin
   pending<={8'd0,I_fault_code,2'b00,68'd0,I_source,I_epoch};
   visible<={8'd0,I_fault_code,2'b00,68'd0,I_source,I_epoch};dirty<=0;age<=0;
  end else begin
   if(I_frame_tick)begin
    if(dirty)begin visible<=pending;age<=0;dirty<=0;end
    else if(age>=5)begin visible<=0;age<=6;end
    else age<=age+1'b1;
   end
   if(I_pdone)begin
    pending<={p_code,p_ok,1'b0,p_ok?I_pbox:34'd0,34'd0,I_source,I_epoch};dirty<=1;
   end
   if(I_kdone)begin
    pending<={(PAPER_REQUIRED!=0?4'd2:4'd0),k_state,k_reason,pending[133],!I_kfailed&&I_ktarget,
     pending[131:98],(!I_kfailed&&I_ktarget)?I_kbox:34'd0,I_source,I_epoch};dirty<=1;
   end
  end
 end
endmodule
