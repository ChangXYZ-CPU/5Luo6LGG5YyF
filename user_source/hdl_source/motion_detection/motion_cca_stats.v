// B06: exactly-once local features, frozen-root reduction and monotone selection.
// Packed table word: xmin[61:51], xmax[50:40], ymin[39:30], ymax[29:20], area[19:0].
// No board pins, frame IDs or display publication (those belong to B07+).
module motion_cca_stats #(
 parameter integer WIDTH=160,HEIGHT=90,MIN_AREA=4,MIN_WIDTH=2,MIN_HEIGHT=2,
 parameter integer FOLD_BUDGET=12000
)(
 input wire I_clk,I_rst_n,I_enable,I_frame_start,I_bad_event,I_frame_bad,I_config_bad,
 input wire I_core_failed,I_core_done,I_core_frozen,
 input wire [2:0] I_core_error,
 input wire [9:0] I_labels,
 input wire I_run_valid,I_run_new,
 input wire [10:0] I_run_xmin,I_run_xmax,
 input wire [9:0] I_run_y,I_run_label,
 output wire O_run_ready,O_features_idle,
 output wire O_root_req,
 output wire [9:0] O_root_label,
 input wire I_root_ready,I_root_valid,
 input wire [9:0] I_root,
 input wire [3:0] I_root_reads,
 output reg O_result_ready,O_done,O_failed,O_fault,
 output reg [2:0] O_error,
 output reg O_target_valid,
 output reg [10:0] O_xmin,O_xmax,
 output reg [9:0] O_ymin,O_ymax,
 output reg [19:0] O_area,O_total_area,
 output reg [10:0] O_regions,O_qualified,
 output reg [15:0] O_fold_cycles,
 output reg [7:0] O_max_label_cycles,
 output wire O_busy
);
 localparam PARAM_OK=(WIDTH>=1 && WIDTH<=1280 && HEIGHT>=1 && HEIGHT<=720 &&
  MIN_AREA>=1 && MIN_AREA<=921600 && MIN_WIDTH>=1 && MIN_WIDTH<=1280 &&
  MIN_HEIGHT>=1 && MIN_HEIGHT<=720 && FOLD_BUDGET>=1 && FOLD_BUDGET<=65535);
 localparam COLLECT=0,LWAIT=1,LMAKE=2,LWRITE=3,CLEAR=4,QUERY=5,ROOT_WAIT=6,
  GWAIT=7,GMAKE=8,GWRITE=9,FINISHED=10;
 reg [3:0] state;
 reg active;
 reg [9:0] allocated,label_limit,index,clear_index,local_label,aggregate_root;
 reg [10:0] run_xmin,run_xmax;
 reg [9:0] run_y;
 reg [11:0] run_length;
 reg [61:0] local_word,merged;
 reg [19:0] folded_total;
 reg [15:0] fold_cycles;
 reg [7:0] label_cycles;
 reg local_re,local_we,aggregate_re,aggregate_we;
 reg [9:0] local_raddr,local_waddr,aggregate_raddr,aggregate_waddr;
 reg [61:0] local_wdata,aggregate_wdata;
 wire [61:0] local_data,aggregate_data;
 wire start_busy=I_frame_start && !O_failed && state!=COLLECT && state!=FINISHED;
 wire fold_active=(state>=CLEAR && state<=GWRITE);
 // Budget is inclusive: a last write completing exactly on the limit is allowed.
 wire finish_edge=(state==CLEAR && clear_index==255 && label_limit==0) ||
  (state==GWRITE && index==label_limit);
 wire fold_timeout=fold_active &&
  ((fold_cycles>=FOLD_BUDGET-1 && !finish_edge) || fold_cycles>=FOLD_BUDGET);
 wire label_timeout=(state>=QUERY && state<=GWRITE) && label_cycles>=41 && state!=GWRITE;
 wire abort_now=!I_rst_n || !I_enable || !PARAM_OK || I_config_bad || I_bad_event ||
  I_frame_bad || I_core_failed || start_busy || fold_timeout || label_timeout;
 wire work_ok=!abort_now && !O_failed && !I_frame_start;
 assign O_run_ready=work_ok && active && state==COLLECT && !I_core_done;
 assign O_features_idle=work_ok && state==COLLECT;
 assign O_root_req=work_ok && state==QUERY;
 assign O_root_label=index;
 assign O_busy=active && !O_failed && !O_result_ready;
 wire take=O_run_ready && I_run_valid;
 wire [11:0] live_length={1'b0,I_run_xmax}-{1'b0,I_run_xmin}+12'd1;
 wire [20:0] total_next={1'b0,O_total_area}+{9'd0,live_length};
 wire descriptor_ok=I_run_label!=0 && I_run_xmin<=I_run_xmax && I_run_xmax<WIDTH &&
  I_run_y<HEIGHT && total_next<=WIDTH*HEIGHT &&
  ((I_run_new && allocated<255 && I_run_label==allocated+10'd1) ||
   (!I_run_new && I_run_label<=allocated));
 function word_ok;
  input [61:0] word;
  begin word_ok=word[19:0]!=0 && word[19:0]<=WIDTH*HEIGHT &&
   word[61:51]<=word[50:40] && word[50:40]<WIDTH &&
   word[39:30]<=word[29:20] && word[29:20]<HEIGHT;end
 endfunction
 function qualifies;
  input [61:0] word;
  reg [11:0] width;
  reg [10:0] height;
  begin
   width={1'b0,word[50:40]}-{1'b0,word[61:51]}+12'd1;
   height={1'b0,word[29:20]}-{1'b0,word[39:30]}+11'd1;
   qualifies=word[19:0]>=MIN_AREA && width>=MIN_WIDTH && height>=MIN_HEIGHT;
  end
 endfunction
 wire [20:0] local_sum={1'b0,local_data[19:0]}+{9'd0,run_length};
 wire [20:0] aggregate_sum={1'b0,aggregate_data[19:0]}+{1'b0,local_word[19:0]};
 wire [20:0] folded_next={1'b0,folded_total}+{1'b0,local_word[19:0]};
 wire candidate=qualifies(merged);
 wire better=!O_target_valid || merged[19:0]>O_area ||
  (merged[19:0]==O_area &&
   (merged[39:30]<O_ymin || (merged[39:30]==O_ymin &&
    (merged[61:51]<O_xmin || (merged[61:51]==O_xmin &&
     (merged[29:20]<O_ymax || (merged[29:20]==O_ymax && merged[50:40]<O_xmax)))))));
 // The two logical tables never issue simultaneous reads or writes. Share
 // a 512x62 physical RAM: bit 8 selects local (0) / aggregate (1) storage.
 // ROOT_WAIT captures the local word BEFORE requesting the aggregate read.
 // This reduces four inferred ERAMs to two without changing the FSM latency.
 wire [61:0] table_data;
 assign local_data=table_data;
 assign aggregate_data=table_data;
 red_sync_ram #(.DW(62),.AW(9)) tables(.I_clk(I_clk),
  .I_we(local_we || aggregate_we),.I_re(local_re || aggregate_re),
  .I_waddr(aggregate_we?{1'b1,aggregate_waddr[7:0]}:{1'b0,local_waddr[7:0]}),
  .I_raddr(aggregate_re?{1'b1,aggregate_raddr[7:0]}:{1'b0,local_raddr[7:0]}),
  .I_wdata(aggregate_we?aggregate_wdata:local_wdata),.O_rdata(table_data));
 always @* begin
  local_re=0;local_we=0;local_raddr=local_label;local_waddr=local_label;local_wdata=merged;
  aggregate_re=0;aggregate_we=0;aggregate_raddr=aggregate_root;
  aggregate_waddr=aggregate_root;aggregate_wdata=merged;
  if(work_ok) begin
   if(take && descriptor_ok) begin
    if(I_run_new) begin
     local_we=1;local_waddr=I_run_label;
     local_wdata={I_run_xmin,I_run_xmax,I_run_y,I_run_y,8'd0,live_length};
    end else begin local_re=1;local_raddr=I_run_label;end
   end
   if(state==LWRITE) local_we=1;
   if(state==CLEAR) begin aggregate_we=1;aggregate_waddr=clear_index;aggregate_wdata=62'd0;end
   if(state==QUERY && I_root_ready) begin local_re=1;local_raddr=index;end
   if(state==ROOT_WAIT && I_root_valid && I_root!=0 && I_root<=label_limit) begin
    aggregate_re=1;aggregate_raddr=I_root;
   end
   if(state==GWRITE && folded_next<=WIDTH*HEIGHT &&
    (index!=label_limit || folded_next==O_total_area)) aggregate_we=1;
  end
 end
 task clear_control;
  begin
   state<=COLLECT;active<=0;allocated<=0;label_limit<=0;index<=0;clear_index<=0;
   local_label<=0;aggregate_root<=0;run_xmin<=0;run_xmax<=0;run_y<=0;run_length<=0;
   local_word<=0;merged<=0;folded_total<=0;fold_cycles<=0;label_cycles<=0;
   O_failed<=0;O_error<=0;O_result_ready<=0;O_target_valid<=0;
   O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;O_area<=0;O_total_area<=0;
   O_regions<=0;O_qualified<=0;O_fold_cycles<=0;O_max_label_cycles<=0;
  end
 endtask
 task fail_frame;
  input [2:0] code;
  begin
   if(!O_failed || I_frame_start) begin O_fault<=1;O_error<=code;end
   O_failed<=1;O_result_ready<=0;O_target_valid<=0;O_area<=0;
   O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;active<=0;state<=COLLECT;
  end
 endtask
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin clear_control;O_done<=0;O_fault<=0;end
  else begin
   O_done<=0;O_fault<=0;
   if(!I_enable) clear_control;
   else if(!PARAM_OK || I_config_bad) fail_frame(3'd7);
   else if(I_bad_event || I_frame_bad) fail_frame(3'd5);
   else if(start_busy || fold_timeout || label_timeout) fail_frame(3'd6);
   // A good new SOF clears previous failures. B05 also clears on that edge.
   else if(I_frame_start) begin clear_control;active<=1;end
   else if(I_core_failed) fail_frame(I_core_error==0?3'd7:I_core_error);
   else if(!O_failed) begin
    if(fold_active) fold_cycles<=fold_cycles+16'd1;
    if(state>=QUERY && state<=GWRITE) label_cycles<=label_cycles+8'd1;
    case(state)
     COLLECT: begin
      if(I_core_done) begin
       if(!active || !I_core_frozen || I_labels>255 || I_labels!=allocated || I_run_valid) fail_frame(3'd7);
       else begin
        label_limit<=I_labels;index<=1;clear_index<=0;fold_cycles<=0;folded_total<=0;state<=CLEAR;
       end
      end else if(take) begin
       if(!descriptor_ok) fail_frame(3'd7);
       else begin
        O_total_area<=total_next[19:0];
        if(I_run_new) allocated<=I_run_label;
        else begin
         local_label<=I_run_label;run_xmin<=I_run_xmin;run_xmax<=I_run_xmax;
         run_y<=I_run_y;run_length<=live_length;state<=LWAIT;
        end
       end
      end
     end
     LWAIT: state<=LMAKE;
     LMAKE: begin
      if(!word_ok(local_data) || local_sum>WIDTH*HEIGHT) fail_frame(3'd7);
      else begin
       merged<={(run_xmin<local_data[61:51]?run_xmin:local_data[61:51]),
        (run_xmax>local_data[50:40]?run_xmax:local_data[50:40]),
        (run_y<local_data[39:30]?run_y:local_data[39:30]),
        (run_y>local_data[29:20]?run_y:local_data[29:20]),local_sum[19:0]};state<=LWRITE;
      end
     end
     LWRITE: state<=COLLECT;
     CLEAR: begin
      if(clear_index==255) begin
       label_cycles<=0;
       if(label_limit==0) begin
        if(O_total_area!=0) fail_frame(3'd7);
        else begin O_result_ready<=1;O_done<=1;O_fold_cycles<=fold_cycles+16'd1;state<=FINISHED;end
       end else state<=QUERY;
      end else clear_index<=clear_index+10'd1;
     end
     QUERY: if(I_root_ready) state<=ROOT_WAIT;
     ROOT_WAIT: if(I_root_valid) begin
      if(I_root==0 || I_root>label_limit || I_root_reads==0 || I_root_reads>11 || !word_ok(local_data)) fail_frame(3'd7);
      else begin local_word<=local_data;aggregate_root<=I_root;state<=GWAIT;end
     end
     GWAIT: state<=GMAKE;
     GMAKE: begin
      if(aggregate_data[19:0]==0) begin merged<=local_word;state<=GWRITE;end
      else if(!word_ok(aggregate_data) || aggregate_sum>WIDTH*HEIGHT) fail_frame(3'd7);
      else begin
       merged<={(local_word[61:51]<aggregate_data[61:51]?local_word[61:51]:aggregate_data[61:51]),
        (local_word[50:40]>aggregate_data[50:40]?local_word[50:40]:aggregate_data[50:40]),
        (local_word[39:30]<aggregate_data[39:30]?local_word[39:30]:aggregate_data[39:30]),
        (local_word[29:20]>aggregate_data[29:20]?local_word[29:20]:aggregate_data[29:20]),aggregate_sum[19:0]};state<=GWRITE;
      end
     end
     GWRITE: begin
      if(label_cycles+8'd1>O_max_label_cycles) O_max_label_cycles<=label_cycles+8'd1;
      if(folded_next>WIDTH*HEIGHT || (index==label_limit && folded_next!=O_total_area)) fail_frame(3'd7);
      else begin
       folded_total<=folded_next[19:0];
       if(aggregate_data[19:0]==0) O_regions<=O_regions+11'd1;
       if(candidate && (aggregate_data[19:0]==0 || !qualifies(aggregate_data))) O_qualified<=O_qualified+11'd1;
       if(candidate && better) begin
        O_target_valid<=1;O_area<=merged[19:0];O_xmin<=merged[61:51];O_xmax<=merged[50:40];
        O_ymin<=merged[39:30];O_ymax<=merged[29:20];
       end
       if(index==label_limit) begin
        O_result_ready<=1;O_done<=1;O_fold_cycles<=fold_cycles+16'd1;state<=FINISHED;
       end else begin index<=index+10'd1;label_cycles<=0;state<=QUERY;end
      end
     end
     FINISHED: begin end
     default: fail_frame(3'd7);
    endcase
   end
  end
 end
endmodule
