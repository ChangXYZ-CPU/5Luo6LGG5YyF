// B05: capture runs and build a bounded union-by-rank forest (8-connectivity).
// Consumes B04 classified/guarded pixels. Not a board top and not an overlay.
// B06 consumes each accepted run exactly once and queries the frozen forest.
module red_cca_core #(
 parameter integer WIDTH=1280,HEIGHT=720,MAX_RUNS=128,MAX_LABELS=1023,
 parameter integer ROW_BUDGET=1600,FEATURE_WAIT_BUDGET=16,
 parameter integer PACK_ROW_BANKS=1
)(
 input wire I_clk,I_rst_n,I_enable,I_valid,I_red,I_user,I_last,
 input wire [11:0] I_x,I_y,
 input wire I_frame_start,I_frame_end,I_bad_event,I_frame_bad,I_config_bad,
 input wire I_run_ready,I_features_idle,
 output wire O_run_valid,
 output wire [10:0] O_run_xmin,O_run_xmax,
 output wire [9:0] O_run_y,O_run_label,
 output wire O_run_new,
 output reg O_done,O_failed,O_fault,
 output reg [2:0] O_error,
 output reg O_frozen,
 output reg [9:0] O_labels,
 input wire I_root_req,
 input wire [9:0] I_root_label,
 output wire O_root_ready,
 output reg O_root_valid,
 output reg [9:0] O_root,
 output reg [3:0] O_root_rank,O_root_reads,
 output reg O_row_done,
 output reg [15:0] O_row_cycles,O_row_runs,O_row_scans,O_row_overlaps,O_row_find_reads,
 output reg [3:0] O_max_find_reads,O_max_rank,
 output reg [7:0] O_max_runs,
 output wire [7:0] O_capture_runs
);
 localparam PARAM_OK=(WIDTH>=1 && WIDTH<=1280 && HEIGHT>=1 && HEIGHT<=720 &&
  MAX_RUNS>=1 && MAX_RUNS<=128 && MAX_LABELS>=1 && MAX_LABELS<=1023 &&
  ROW_BUDGET>=1 && ROW_BUDGET<=65535 && FEATURE_WAIT_BUDGET>=1 && FEATURE_WAIT_BUDGET<=65535);
 localparam IDLE=0,ROW_BEGIN=1,CUR_REQ=2,CUR_WAIT=3,CUR_GET=4,
  PRE_REQ=5,PRE_WAIT=6,PRE_GET=7,ALLOC=8,EMIT=9,ROW_FINISH=10,
  FIND_REQ=11,FIND_WAIT=12,FIND_GET=13,UNION_LOSER=14,UNION_WINNER=15,
  FEATURE_WAIT=16;
 localparam RET_PRE=0,RET_CUR=1,RET_EXTERNAL=2;
 reg [4:0] state;
 reg frame_open,run_open,row_active;
 reg [1:0] c_bank,q_bank,p_bank;
 reg [7:0] cap_count,q_count,p_count,cur_index,base,cursor;
 reg [10:0] cap_start,cur_start,cur_end;
 reg [9:0] q_y,chosen;
 reg q_final,chosen_new;
 reg [15:0] row_cycles,scans,overlaps,reads,tail_cycles;
 reg [5:0] emit_wait;
 reg [9:0] find_node,root_b,union_winner,union_loser;
 reg [3:0] find_steps,find_min_rank,rank_b,union_wrank,union_lrank;
 reg [1:0] find_return;
 reg union_equal;
 wire [31:0] run_data;
 reg run_re,commit_we;
 reg [1:0] read_bank;
 reg [6:0] read_addr;
 reg parent_re,parent_we;
 reg [9:0] parent_raddr,parent_waddr;
 reg [13:0] parent_wdata;
 wire [13:0] parent_data;

 wire [10:0] pre_start=run_data[31:21],pre_end=run_data[20:10];
 wire [9:0] pre_label=run_data[9:0];
 wire [9:0] read_parent=parent_data[9:0];
 wire [3:0] read_rank=parent_data[13:10];
 wire start_busy=I_frame_start && state!=IDLE;
 wire take_pixel=I_enable && I_valid && !I_bad_event && !I_frame_bad &&
  !I_config_bad && PARAM_OK && !start_busy && (I_frame_start || (frame_open && !O_failed));
 wire open_eff=I_frame_start?1'b0:run_open;
 wire [7:0] count_eff=I_frame_start?8'd0:cap_count;
 // Telemetry only. Preserve the unfinished row's count on capacity failure.
 assign O_capture_runs=cap_count>O_max_runs?cap_count:O_max_runs;
 wire [1:0] bank_eff=I_frame_start?2'd0:c_bank;
 wire [10:0] start_eff=open_eff?cap_start:I_x[10:0];
 wire close_run=take_pixel && ((open_eff && !I_red) || (I_red && I_last));
 wire cap_overflow=take_pixel && !open_eff && I_red && count_eff>=MAX_RUNS;
 wire deadline=take_pixel && I_last && !I_frame_start && state!=IDLE;
 wire row_timeout=row_active && row_cycles>=ROW_BUDGET-1 && state!=ROW_FINISH;
 wire tail_timeout=state==FEATURE_WAIT && !I_features_idle && tail_cycles>=FEATURE_WAIT_BUDGET-1;
 // 40R design budget includes at most 31 stalled clocks per emitted run.
 wire emit_timeout=state==EMIT && !I_run_ready && emit_wait>=31;
 wire abort_now=!I_rst_n || !I_enable || I_bad_event || I_frame_bad || I_config_bad || !PARAM_OK ||
  start_busy || cap_overflow || deadline || row_timeout || tail_timeout || emit_timeout;
 wire work_ok=!abort_now && !O_failed && !I_frame_start;
 wire cap_we=close_run && !abort_now;
 wire [31:0] cap_data={start_eff,(I_red?I_x[10:0]:(I_x[10:0]-11'd1)),10'd0};
 wire [31:0] commit_data={cur_start,cur_end,chosen};
 wire [8:0] closed_count={1'b0,count_eff}+(close_run?9'd1:9'd0);
 wire [2:0] free_bank=3'd3-{1'b0,c_bank}-{1'b0,p_bank};
 assign O_run_valid=work_ok && state==EMIT;
 assign O_run_xmin=cur_start;assign O_run_xmax=cur_end;
 assign O_run_y=q_y;assign O_run_label=chosen;assign O_run_new=chosen_new;
 assign O_root_ready=work_ok && O_frozen && state==IDLE;
 // Three128x32 run banks fit in one512x32 true-dual-port ERAM.
 // PortA captures the incoming row. PortB either reads a work/previous
 // row or commits a label (the FSM never requests both). The bank field
 // is part of each address, preserving all three independent lifetimes.
 genvar bank;
 generate if(PACK_ROW_BANKS==1)begin:packed_banks
  (* ram_style="bram" *)reg[31:0] words[0:511];
  reg[31:0] read_q;
  wire[8:0] cap_addr={bank_eff,count_eff[6:0]};
  wire[8:0] work_addr=commit_we?{q_bank,cur_index[6:0]}:{read_bank,read_addr};
  always @(posedge I_clk)begin
   if(cap_we)words[cap_addr]<=cap_data;
  end
  always @(posedge I_clk)begin
   if(commit_we)words[work_addr]<=commit_data;
   if(run_re)read_q<=words[work_addr];
  end
  assign run_data=read_q;
 end else begin:separate_banks
  wire[31:0] b0_data,b1_data,b2_data;
  assign run_data=(read_bank==0)?b0_data:(read_bank==1)?b1_data:b2_data;
 for(bank=0;bank<3;bank=bank+1) begin: banks
  wire capture=cap_we && bank_eff==bank;
  wire commit=commit_we && q_bank==bank;
  wire [31:0] rdata;
  red_sync_ram #(.DW(32),.AW(7)) ram(
   .I_clk(I_clk),.I_we(capture || commit),.I_re(run_re && read_bank==bank),
   .I_waddr(capture?count_eff[6:0]:cur_index[6:0]),
   .I_raddr(read_addr),.I_wdata(capture?cap_data:commit_data),.O_rdata(rdata));
  if(bank==0) assign b0_data=rdata;
  else if(bank==1) assign b1_data=rdata;
  else assign b2_data=rdata;
 end
 end endgenerate
 red_sync_ram #(.DW(14),.AW(10)) parents(
  .I_clk(I_clk),.I_we(parent_we),.I_re(parent_re),
  .I_waddr(parent_waddr),.I_raddr(parent_raddr),.I_wdata(parent_wdata),.O_rdata(parent_data));
 always @* begin
  run_re=0;read_bank=q_bank;read_addr=cur_index[6:0];commit_we=0;
  if(state==PRE_REQ || state==PRE_WAIT || state==PRE_GET) begin read_bank=p_bank;read_addr=cursor[6:0];end
  parent_re=0;parent_we=0;parent_raddr=find_node;parent_waddr=0;parent_wdata=0;
  if(work_ok) begin
   case(state)
    CUR_REQ: begin run_re=1;read_bank=q_bank;end
    PRE_REQ: begin run_re=cursor<p_count;read_bank=p_bank;read_addr=cursor[6:0];end
    FIND_REQ: parent_re=1;
    ALLOC: if(O_labels<MAX_LABELS) begin
     parent_we=1;parent_waddr=O_labels+10'd1;parent_wdata={4'd0,(O_labels+10'd1)};
    end
    EMIT: commit_we=I_run_ready;
    UNION_LOSER: begin parent_we=1;parent_waddr=union_loser;parent_wdata={union_lrank,union_winner};end
    UNION_WINNER: begin parent_we=1;parent_waddr=union_winner;parent_wdata={union_wrank+4'd1,union_winner};end
   endcase
  end
 end
 task clear_frame;
  begin
   state<=IDLE;frame_open<=0;run_open<=0;row_active<=0;
   c_bank<=0;q_bank<=1;p_bank<=2;cap_count<=0;q_count<=0;p_count<=0;
   cur_index<=0;base<=0;cursor<=0;cap_start<=0;cur_start<=0;cur_end<=0;
   q_y<=0;chosen<=0;q_final<=0;chosen_new<=0;
   row_cycles<=0;scans<=0;overlaps<=0;reads<=0;tail_cycles<=0;emit_wait<=0;
   find_node<=0;root_b<=0;union_winner<=0;union_loser<=0;
   find_steps<=0;find_min_rank<=0;rank_b<=0;union_wrank<=0;union_lrank<=0;find_return<=0;union_equal<=0;
   O_failed<=0;O_error<=0;O_frozen<=0;O_labels<=0;
   O_max_find_reads<=0;O_max_rank<=0;O_max_runs<=0;
  end
 endtask
 task fail_frame;
  input [2:0] code;
  begin
   if(!O_failed || I_frame_start) begin O_fault<=1;O_error<=code;end
   O_failed<=1;O_frozen<=0;
   frame_open<=0;run_open<=0;row_active<=0;state<=IDLE;
  end
 endtask
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   clear_frame;
   O_done<=0;O_fault<=0;O_root_valid<=0;O_root<=0;O_root_rank<=0;O_root_reads<=0;
   O_row_done<=0;O_row_cycles<=0;O_row_runs<=0;O_row_scans<=0;O_row_overlaps<=0;O_row_find_reads<=0;
  end else begin
   O_done<=0;O_fault<=0;O_root_valid<=0;O_row_done<=0;
   if(!I_enable) clear_frame;
   else if(!PARAM_OK || I_config_bad) fail_frame(3'd7);
   else if(I_bad_event || I_frame_bad) fail_frame(3'd5);
   else if(start_busy || deadline || row_timeout || tail_timeout || emit_timeout) fail_frame(3'd6);
   else if(cap_overflow) fail_frame(3'd4);
   else begin
    if(I_frame_start) begin clear_frame;frame_open<=1;end
    if(take_pixel) begin
     if(I_red) begin
      if(!open_eff) cap_start<=I_x[10:0];
      run_open<=!I_last;
     end else run_open<=0;
     if(close_run) cap_count<=closed_count[7:0];
     if(I_last) begin
      q_bank<=bank_eff;q_count<=closed_count[7:0];q_y<=I_y[9:0];q_final<=I_frame_end;
      c_bank<=I_frame_start?2'd1:free_bank[1:0];
      cap_count<=0;run_open<=0;state<=ROW_BEGIN;row_active<=1;row_cycles<=0;
      scans<=0;overlaps<=0;reads<=0;
      if(closed_count>O_max_runs || I_frame_start) O_max_runs<=closed_count[7:0];
     end
     if(I_frame_end) frame_open<=0;
    end
    if(!I_frame_start && !O_failed) begin
     if(row_active) row_cycles<=row_cycles+16'd1;
     if(state==EMIT && !I_run_ready) emit_wait<=emit_wait+6'd1;
     else emit_wait<=0;
     case(state)
      IDLE: if(O_root_ready && I_root_req) begin
       if(I_root_label==0 || I_root_label>O_labels) fail_frame(3'd7);
       else begin find_node<=I_root_label;find_steps<=0;find_min_rank<=0;find_return<=RET_EXTERNAL;state<=FIND_REQ;end
      end
      ROW_BEGIN: begin
       cur_index<=0;base<=0;cursor<=0;
       state<=(q_count==0)?ROW_FINISH:CUR_REQ;
      end
      CUR_REQ: state<=CUR_WAIT;
      CUR_WAIT: state<=CUR_GET;
      CUR_GET: begin
       cur_start<=run_data[31:21];cur_end<=run_data[20:10];chosen<=0;chosen_new<=0;
       cursor<=base;state<=PRE_REQ;
       if(run_data[31:21]>run_data[20:10] || run_data[20:10]>=WIDTH) fail_frame(3'd7);
      end
      PRE_REQ: begin
       if(cursor>=p_count) state<=(chosen==0)?ALLOC:EMIT;
       else state<=PRE_WAIT;
      end
      PRE_WAIT: state<=PRE_GET;
      PRE_GET: begin
       scans<=scans+16'd1;
       if(pre_label==0 || pre_label>O_labels || pre_start>pre_end || pre_end>=WIDTH) fail_frame(3'd7);
       else if({1'b0,pre_end}+12'd1<{1'b0,cur_start}) begin
        base<=cursor+8'd1;cursor<=cursor+8'd1;state<=PRE_REQ;
       end else if({1'b0,pre_start}>{1'b0,cur_end}+12'd1) state<=(chosen==0)?ALLOC:EMIT;
       else begin
        // The next current run starts >=cur_end+2. A previous run ending
        // <=cur_end cannot touch it, even diagonally, so retire ONLY that run.
        // Long previous runs extending beyond cur_end remain reusable.
        if(pre_end<=cur_end) base<=cursor+8'd1;
        overlaps<=overlaps+16'd1;find_node<=pre_label;find_steps<=0;find_min_rank<=0;
        find_return<=RET_PRE;state<=FIND_REQ;
       end
      end
      ALLOC: begin
       if(O_labels>=MAX_LABELS) fail_frame(3'd4);
       else begin O_labels<=O_labels+10'd1;chosen<=O_labels+10'd1;chosen_new<=1;state<=EMIT;end
      end
      EMIT: if(I_run_ready) begin
       cur_index<=cur_index+8'd1;
       state<=(cur_index+8'd1>=q_count)?ROW_FINISH:CUR_REQ;
      end
      FIND_REQ: state<=FIND_WAIT;
      FIND_WAIT: state<=FIND_GET;
      FIND_GET: begin
       if(find_return!=RET_EXTERNAL) reads<=reads+16'd1;
       if(find_node==0 || find_node>O_labels || read_parent==0 || read_parent>O_labels || read_rank>9 || read_rank<find_min_rank) fail_frame(3'd7);
       else if(read_parent==find_node) begin
        if(find_steps+4'd1>O_max_find_reads) O_max_find_reads<=find_steps+4'd1;
        if(find_return==RET_EXTERNAL) begin
         O_root_valid<=1;O_root<=find_node;O_root_rank<=read_rank;O_root_reads<=find_steps+4'd1;state<=IDLE;
        end else if(find_return==RET_PRE) begin
         if(chosen==0) begin chosen<=find_node;cursor<=cursor+8'd1;state<=PRE_REQ;end
         else begin
          root_b<=find_node;rank_b<=read_rank;find_node<=chosen;find_steps<=0;find_min_rank<=0;find_return<=RET_CUR;state<=FIND_REQ;
         end
        end else begin
         if(find_node==root_b) begin cursor<=cursor+8'd1;state<=PRE_REQ;end
         else if(read_rank==rank_b && read_rank>=9) fail_frame(3'd7);
         else begin
          if(read_rank>rank_b || (read_rank==rank_b && find_node<root_b)) begin
           union_winner<=find_node;union_wrank<=read_rank;union_loser<=root_b;union_lrank<=rank_b;
          end else begin
           union_winner<=root_b;union_wrank<=rank_b;union_loser<=find_node;union_lrank<=read_rank;
          end
          union_equal<=read_rank==rank_b;state<=UNION_LOSER;
         end
        end
       end else if(find_steps>=10) fail_frame(3'd7);
       else begin find_steps<=find_steps+4'd1;find_min_rank<=read_rank+4'd1;find_node<=read_parent;state<=FIND_REQ;end
      end
      UNION_LOSER: begin
       if(union_equal) state<=UNION_WINNER;
       else begin cursor<=cursor+8'd1;state<=PRE_REQ;end
      end
      UNION_WINNER: begin
       if(union_wrank+4'd1>O_max_rank) O_max_rank<=union_wrank+4'd1;
       cursor<=cursor+8'd1;state<=PRE_REQ;
      end
      ROW_FINISH: begin
       row_active<=0;O_row_done<=1;O_row_cycles<=row_cycles+16'd1;
       O_row_runs<={8'd0,q_count};O_row_scans<=scans;O_row_overlaps<=overlaps;O_row_find_reads<=reads;
       p_bank<=q_bank;p_count<=q_count;state<=IDLE;
       if(q_final) begin
        if(I_features_idle) begin O_frozen<=1;O_done<=1;end
        else begin state<=FEATURE_WAIT;tail_cycles<=0;end
       end
      end
      FEATURE_WAIT: begin
       if(I_features_idle) begin O_frozen<=1;O_done<=1;state<=IDLE;end
       else tail_cycles<=tail_cycles+16'd1;
      end
      default: fail_frame(3'd7);
     endcase
    end
   end
  end
 end
endmodule
