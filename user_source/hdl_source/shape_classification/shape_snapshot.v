// Standalone black/white detector. Observer only; no display path alteration.
// Status 0 WAIT/disabled, 1 complete target (class may UNKNOWN), 2 NONE, 3 ERROR.
module shape_snapshot(
 input wire I_clk,I_rst_n,I_enable,I_cancel,
 input wire I_de,I_sof,I_eol,I_pixel_good,
 input wire [11:0] I_x,I_y,
 input wire [23:0] I_rgb,
 input wire [7:0] I_threshold,
 input wire [3:0] I_mode,
 input wire [31:0] I_source_frame_id,
 output wire O_detect,
 output wire [2:0] O_status,
 output wire O_target_valid,
 output wire [1:0] O_class,
 output wire [11:0] O_xmin,O_ymin,O_xmax,O_ymax,O_center_x,O_center_y,
 output wire [15:0] O_area_cells,
 output wire [31:0] O_frame_seq,O_source_frame_id,O_epoch,
 output wire O_busy,
 output wire [20:0] O_job_cycles
);
 wire raw_sof=I_de&&I_sof;
 reg [3:0] frame_mode;
 reg [7:0] frame_threshold;
 reg [31:0] seq,epoch;
 wire change=raw_sof&&(I_mode!=frame_mode || I_threshold!=frame_threshold);
 wire enabled=I_enable&&I_mode==12;
 reg cap_bank,cap_owned;
 reg [15:0] cap_count;
 reg [1:0] ready;
 reg [31:0] bank_seq[0:1],bank_source[0:1],bank_epoch[0:1];
 reg job_bank,engine_start;
 reg [31:0] job_seq,job_source,job_epoch;
 wire engine_busy,engine_done,engine_failed,engine_target;
 wire [2:0] engine_error;
 wire [1:0] engine_class;
 wire [8:0] engine_l,engine_r;
 wire [7:0] engine_t,engine_b;
 wire [15:0] engine_area;
 wire [10:0] engine_peak,engine_components;
 wire mem_re,mem_we;wire [15:0] mem_addr;
 wire mem_bit,mem_bit0,mem_bit1;
 wire held=engine_busy||engine_start||engine_done;
 wire free0=!ready[0]&&!(held&&job_bank==0)&&!(cap_owned&&cap_bank==0);
 wire free1=!ready[1]&&!(held&&job_bank==1)&&!(cap_owned&&cap_bank==1);
 wire conflict=enabled&&raw_sof&&!change&&!(free0||free1);
 wire cap_write,cap_bit,cap_sof,cap_eof,cap_bad,cap_active;
 wire [15:0] cap_addr;
 wire flush=I_cancel||!enabled||change||conflict||cap_bad;
 wire fault_event=conflict||(cap_bad&&!change&&!I_cancel);
 wire old0=($unsigned(bank_seq[1]-bank_seq[0])<32'h80000000);
 wire next_job_bank=ready[0]&&ready[1]?!old0:!ready[0];
 shape_capture capture(.I_clk(I_clk),.I_rst_n(I_rst_n),
  .I_enable(enabled&&(cap_owned||raw_sof)),.I_cancel(I_cancel||change||conflict),
  .I_de(I_de),.I_sof(I_sof),.I_eol(I_eol),.I_pixel_good(I_pixel_good),
  .I_x(I_x),.I_y(I_y),.I_rgb(I_rgb),.I_threshold(I_threshold),
  .O_write(cap_write),.O_bit(cap_bit),.O_sof(cap_sof),.O_eof(cap_eof),
  .O_addr(cap_addr),.O_bad_event(cap_bad),.O_active(cap_active));
 shape_mask_engine engine(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(flush),.I_start(engine_start),
  .O_mem_re(mem_re),.O_mem_we(mem_we),.O_mem_addr(mem_addr),.I_mem_bit(mem_bit),
  .O_busy(engine_busy),.O_done(engine_done),.O_failed(engine_failed),.O_target_valid(engine_target),
  .O_error(engine_error),.O_class(engine_class),.O_xmin(engine_l),.O_xmax(engine_r),
  .O_ymin(engine_t),.O_ymax(engine_b),.O_area(engine_area),.O_cycles(O_job_cycles),
  .O_queue_peak(engine_peak),.O_components(engine_components));
 wire cap_we=cap_write&&cap_owned&&!flush;
 wire we0=(cap_we&&!cap_bank)||(mem_we&&!job_bank);
 wire we1=(cap_we&&cap_bank)||(mem_we&&job_bank);
 wire [15:0] wa0=(cap_we&&!cap_bank)?cap_addr:mem_addr;
 wire [15:0] wa1=(cap_we&&cap_bank)?cap_addr:mem_addr;
 wire wd0=(cap_we&&!cap_bank)?cap_bit:1'b0;
 wire wd1=(cap_we&&cap_bank)?cap_bit:1'b0;
 shape_bit_bank bank0(I_clk,we0,mem_re&&!job_bank,wa0,mem_addr,wd0,mem_bit0);
 shape_bit_bank bank1(I_clk,we1,mem_re&&job_bank,wa1,mem_addr,wd1,mem_bit1);
 assign mem_bit=job_bank?mem_bit1:mem_bit0;
 // Complete result packet. Source camera ID and display sequence stay separate.
 reg pending;
 reg [2:0] p_status,s_status;
 reg p_target,s_target;
 reg [1:0] p_class,s_class;
 reg [8:0] p_l,p_r,s_l,s_r;
 reg [7:0] p_t,p_b,s_t,s_b;
 reg [15:0] p_area,s_area;
 reg [31:0] p_seq,p_source,p_epoch,s_seq,s_source,s_epoch;
 wire direct=raw_sof&&engine_done&&!flush&&job_epoch==epoch;
 wire published=raw_sof&&pending&&!flush&&p_epoch==epoch;
 wire bypass=direct||published;
 wire [2:0] result_status=engine_failed?3'd3:engine_target?3'd1:3'd2;
 wire [8:0] output_l=direct?engine_l:published?p_l:s_l;
 wire [8:0] output_r=direct?engine_r:published?p_r:s_r;
 wire [7:0] output_t=direct?engine_t:published?p_t:s_t;
 wire [7:0] output_b=direct?engine_b:published?p_b:s_b;
 assign O_detect=enabled;
 assign O_busy=engine_busy;
 assign O_status=flush?(fault_event?3'd3:3'd0):direct?result_status:published?p_status:raw_sof?3'd0:s_status;
 assign O_target_valid=!flush&&(direct?(!engine_failed&&engine_target):published?p_target:raw_sof?1'b0:s_target);
 assign O_class=O_target_valid?(direct?engine_class:published?p_class:s_class):2'd0;
 assign O_xmin=O_target_valid?{1'b0,output_l,2'b00}:12'd0;
 assign O_xmax=O_target_valid?{1'b0,output_r,2'b11}:12'd0;
 assign O_ymin=O_target_valid?{2'b00,output_t,2'b00}:12'd0;
 assign O_ymax=O_target_valid?{2'b00,output_b,2'b11}:12'd0;
 assign O_center_x=({1'b0,O_xmin}+{1'b0,O_xmax})>>1;
 assign O_center_y=({1'b0,O_ymin}+{1'b0,O_ymax})>>1;
 assign O_area_cells=O_target_valid?(direct?engine_area:published?p_area:s_area):16'd0;
 assign O_frame_seq=(flush||(raw_sof&&!bypass))?32'd0:direct?job_seq:published?p_seq:s_seq;
 assign O_source_frame_id=(flush||(raw_sof&&!bypass))?32'd0:direct?job_source:published?p_source:s_source;
 assign O_epoch=flush?(epoch+(change?32'd1:32'd0)):(raw_sof&&!bypass)?epoch:direct?job_epoch:published?p_epoch:s_epoch;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   frame_mode<=12;frame_threshold<=128;seq<=0;epoch<=0;
   cap_bank<=0;cap_owned<=0;cap_count<=0;ready<=0;job_bank<=0;engine_start<=0;
   job_seq<=0;job_source<=0;job_epoch<=0;
   bank_seq[0]<=0;bank_seq[1]<=0;bank_source[0]<=0;bank_source[1]<=0;bank_epoch[0]<=0;bank_epoch[1]<=0;
   pending<=0;p_status<=0;p_target<=0;p_class<=0;p_l<=0;p_r<=0;p_t<=0;p_b<=0;p_area<=0;p_seq<=0;p_source<=0;p_epoch<=0;
   s_status<=0;s_target<=0;s_class<=0;s_l<=0;s_r<=0;s_t<=0;s_b<=0;s_area<=0;s_seq<=0;s_source<=0;s_epoch<=0;
  end else begin
   engine_start<=0;
   if(raw_sof)begin
    frame_mode<=I_mode;frame_threshold<=I_threshold;seq<=seq+1'b1;if(change)epoch<=epoch+1'b1;
   end
   if(flush)begin
    ready<=0;cap_owned<=0;cap_count<=0;pending<=0;engine_start<=0;
    s_status<=fault_event?3'd3:3'd0;s_target<=0;s_class<=0;s_area<=0;
    s_l<=0;s_r<=0;s_t<=0;s_b<=0;s_seq<=0;s_source<=0;s_epoch<=epoch;
   end else begin
    if(raw_sof)begin
     cap_bank<=free0?1'b0:1'b1;cap_owned<=1;cap_count<=0;
     bank_seq[free0?0:1]<=seq+1'b1;bank_source[free0?0:1]<=I_source_frame_id;bank_epoch[free0?0:1]<=epoch;
    end
    if(cap_we)cap_count<=cap_count+1'b1;
    if(cap_eof&&cap_owned)begin
     cap_owned<=0;
     if(cap_write&&cap_count==57599&&cap_addr==57599)ready[cap_bank]<=1;
     else begin pending<=0;s_status<=3;s_target<=0;s_class<=0;end
    end
    if(!held&&ready!=0)begin
     job_bank<=next_job_bank;engine_start<=1;ready[next_job_bank]<=0;
     job_seq<=bank_seq[next_job_bank];job_source<=bank_source[next_job_bank];job_epoch<=bank_epoch[next_job_bank];
    end
    if(engine_done&&job_epoch==epoch)begin
     pending<=!raw_sof;p_status<=result_status;p_target<=!engine_failed&&engine_target;
     p_class<=engine_class;p_l<=engine_l;p_r<=engine_r;p_t<=engine_t;p_b<=engine_b;p_area<=engine_area;
     p_seq<=job_seq;p_source<=job_source;p_epoch<=job_epoch;
    end
    if(raw_sof)begin
     pending<=0;
     if(direct)begin
      s_status<=result_status;s_target<=!engine_failed&&engine_target;s_class<=engine_class;
      s_l<=engine_l;s_r<=engine_r;s_t<=engine_t;s_b<=engine_b;s_area<=engine_area;
      s_seq<=job_seq;s_source<=job_source;s_epoch<=job_epoch;
     end else if(published)begin
      s_status<=p_status;s_target<=p_target;s_class<=p_class;s_l<=p_l;s_r<=p_r;s_t<=p_t;s_b<=p_b;
      s_area<=p_area;s_seq<=p_seq;s_source<=p_source;s_epoch<=p_epoch;
     end else begin s_status<=0;s_target<=0;s_class<=0;s_area<=0;end
    end
   end
  end
 end
endmodule
