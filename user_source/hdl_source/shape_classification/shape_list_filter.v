// Three bounded spatial tracks. One fresh backend job is processed serially;
// all visible packets publish together at safe blanking. No pixel retiming.
module shape_list_filter(
 input I_clk,I_rst_n,I_enable,I_cancel,I_fault,I_frame_tick,
 input[31:0] I_expected_epoch,I_seq,I_source,I_epoch,
 input I_result_valid,input[2:0] I_status,I_present,input[5:0] I_classes,
 input[101:0] I_boxes,
 output wire[248:0] O_packets,output wire[95:0] O_sources,output reg O_fault
);
 localparam IDLE=0,SEARCH=1,APPLY=2,ALLOC=3,BUILD=4,LAUNCH=5,WAIT_COORD=6,FINISH=7;
 reg[2:0]state;reg[1:0]ti,ci,best,pi;reg best_valid;reg[10:0]best_score;
 reg[2:0] used,mask;reg[5:0] classes;reg[101:0] boxes;
 reg[33:0] track_box[0:2];reg[1:0]track_class[0:2],count[0:2],misses[0:2];reg[31:0]track_source[0:2];
 reg[31:0] context_epoch,last_seq,last_source,job_source;reg seen;
 reg[2:0]age;reg dirty;reg[248:0]work_packets,pending,visible;reg[95:0]work_sources,pending_sources,visible_sources;
 wire clear=!I_enable||I_cancel||I_fault||I_expected_epoch!=context_epoch;
 wire[31:0] sd=I_source-last_source,qd=I_seq-last_seq;
 wire fresh=I_result_valid&&I_epoch==I_expected_epoch&&(!seen||(sd!=0&&!sd[31]&&qd!=0&&!qd[31]));
 wire gap=seen&&(sd>8||qd>8);
 wire[33:0] cb=boxes[ci*34+:34],tb=track_box[ti];
 function[9:0] difference;input[8:0]a,b;begin difference=a>=b?{1'b0,a}-{1'b0,b}:{1'b0,b}-{1'b0,a};end endfunction
 wire[9:0] dl=difference(cb[33:25],tb[33:25]),dr=difference(cb[24:16],tb[24:16]);
 wire[9:0] dt=difference({1'b0,cb[15:8]},{1'b0,tb[15:8]}),db=difference({1'b0,cb[7:0]},{1'b0,tb[7:0]});
 wire[11:0] distance={2'b0,dl}+{2'b0,dr}+{2'b0,dt}+{2'b0,db};
 wire associated=cb[33:25]<=tb[24:16]&&cb[24:16]>=tb[33:25]&&cb[15:8]<=tb[7:0]&&cb[7:0]>=tb[15:8]&&dl<=16&&dr<=16&&dt<=16&&db<=16;
 wire candidate_ok=mask[ci]&&!used[ci]&&classes[ci*2+:2]!=0;
 reg[1:0] free_slot;reg have_free;integer f;
 always @*begin have_free=0;free_slot=0;for(f=0;f<3;f=f+1)if(!have_free&&count[f]==0)begin have_free=1;free_slot=f;end end
 wire[33:0] pb=track_box[pi];
 wire cr,cv,cbusy;wire[11:0]cx,cy;wire[15:0]xbcd,ybcd;wire[31:0]cs,ce;wire[3:0]cm;
 rgb_bbox_coordinates coordinates(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(clear||O_fault),.I_start(state==LAUNCH),
 .I_xmin({1'b0,pb[33:25],2'b00}),.I_xmax({1'b0,pb[24:16],2'b11}),
 .I_ymin({2'b0,pb[15:8],2'b00}),.I_ymax({2'b0,pb[7:0],2'b11}),
 .I_source_frame_id(track_source[pi]),.I_config_epoch(context_epoch),.I_mode(4'd12),
 .O_ready(cr),.O_valid(cv),.O_busy(cbusy),.O_x(cx),.O_y(cy),.O_x_bcd(xbcd),.O_y_bcd(ybcd),
 .O_source_frame_id(cs),.O_config_epoch(ce),.O_mode(cm));
 assign O_packets=(!I_rst_n||clear||O_fault)?249'd0:visible;
 assign O_sources=(!I_rst_n||clear||O_fault)?96'd0:visible_sources;
 task revoke;
 integer j;begin
  state<=IDLE;ti<=0;ci<=0;pi<=0;best<=0;best_valid<=0;best_score<=0;used<=0;mask<=0;classes<=0;boxes<=0;
  dirty<=0;age<=0;pending<=0;visible<=0;work_packets<=0;work_sources<=0;pending_sources<=0;visible_sources<=0;
  for(j=0;j<3;j=j+1)begin track_box[j]<=0;track_class[j]<=0;count[j]<=0;misses[j]<=0;track_source[j]<=0;end
 end endtask
 task next_track;
 begin
  ci<=0;best_valid<=0;best_score<=2047;
  if(ti==2)begin ci<=0;state<=ALLOC;end else begin ti<=ti+1'b1;state<=SEARCH;end
 end endtask
 task next_coordinate;
 begin if(pi==2)state<=FINISH;else begin pi<=pi+1'b1;state<=BUILD;end end endtask
 integer n,m;reg malformed;
 always @*begin
  malformed=I_status!=1&&I_status!=2;
  for(n=0;n<3;n=n+1)if(I_present[n])begin
   if(I_boxes[n*34+25+:9]>I_boxes[n*34+16+:9]||I_boxes[n*34+8+:8]>I_boxes[n*34+:8]||I_boxes[n*34+16+:9]>=320||I_boxes[n*34+:8]>=180)malformed=1;
  end
  if(I_status==2&&I_present!=0)malformed=1;
 end
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin context_epoch<=0;last_source<=0;last_seq<=0;seen<=0;job_source<=0;O_fault<=0;revoke;end
  else begin
   O_fault<=0;
   if(clear)begin context_epoch<=I_expected_epoch;last_source<=0;last_seq<=0;seen<=0;job_source<=0;revoke;end
   else if(I_frame_tick&&!dirty&&age>=5&&!fresh)begin seen<=0;revoke;end
   else if(fresh&&(state!=IDLE||malformed))begin seen<=0;revoke;O_fault<=1;end
   else if(fresh)begin
    last_source<=I_source;last_seq<=I_seq;job_source<=I_source;seen<=1;age<=0;
    if(gap)begin
     for(m=0;m<3;m=m+1)begin count[m]<=0;misses[m]<=0;track_box[m]<=0;track_class[m]<=0;track_source[m]<=0;end
     visible<=0;visible_sources<=0;dirty<=0;pending<=0;pending_sources<=0;
    end
    mask<=I_present;classes<=I_classes;boxes<=I_boxes;used<=0;ti<=0;ci<=0;best_valid<=0;best_score<=2047;state<=SEARCH;
   end else begin
    if(I_frame_tick)begin
     if(dirty)begin visible<=pending;visible_sources<=pending_sources;dirty<=0;age<=0;end
     else age<=age+1'b1;
    end
    case(state)
     IDLE:;
     SEARCH:begin
      if(count[ti]!=0&&candidate_ok&&associated&&(!best_valid||distance<best_score))begin best<=ci;best_valid<=1;best_score<=distance[10:0];end
      if(ci==2)state<=APPLY;else ci<=ci+1'b1;
     end
     APPLY:begin
      if(count[ti]!=0)begin
       if(best_valid)begin
        used[best]<=1;track_box[ti]<=boxes[best*34+:34];track_source[ti]<=job_source;misses[ti]<=0;
        track_class[ti]<=classes[best*2+:2];
        if(track_class[ti]!=classes[best*2+:2])count[ti]<=1;else if(count[ti]<3)count[ti]<=count[ti]+1'b1;
       end else if(misses[ti]>=2)begin count[ti]<=0;track_class[ti]<=0;track_box[ti]<=0;track_source[ti]<=0;misses[ti]<=0;end
       else misses[ti]<=misses[ti]+1'b1;
      end
      next_track;
     end
     ALLOC:begin
      if(candidate_ok&&have_free)begin used[ci]<=1;track_box[free_slot]<=cb;track_class[free_slot]<=classes[ci*2+:2];track_source[free_slot]<=job_source;count[free_slot]<=1;misses[free_slot]<=0;end
      if(ci==2)begin pi<=0;work_packets<=0;work_sources<=0;state<=BUILD;end else ci<=ci+1'b1;
     end
     BUILD:if(count[pi]>=3)state<=LAUNCH;else next_coordinate;
     LAUNCH:state<=WAIT_COORD;
     WAIT_COORD:if(cr)begin
      if(!cv||cs!=track_source[pi]||ce!=context_epoch||cm!=12)begin seen<=0;revoke;O_fault<=1;end
      else begin
       work_packets[pi*83+:83]<={1'b1,track_class[pi],{1'b0,pb[33:25],2'b00},{1'b0,pb[24:16],2'b11},{2'b0,pb[15:8],2'b00},{2'b0,pb[7:0],2'b11},xbcd,ybcd};
       work_sources[pi*32+:32]<=track_source[pi];next_coordinate;
      end
     end
     FINISH:begin pending<=work_packets;pending_sources<=work_sources;dirty<=1;state<=IDLE;end
     default:begin seen<=0;revoke;O_fault<=1;end
    endcase
   end
  end
 end
endmodule
