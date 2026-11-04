// Same pixel-clock domain only. Observer, not part of video/control feedback.
module colour_diag_link #(
 parameter integer WIDTH=1280,HEIGHT=720,PRE_DELAY=5,PERIOD_FRAMES=16,
 parameter integer CAPTURE_TIMEOUT=2000000,RESULT_TIMEOUT=2000000
)(
 input wire I_clk,I_rst_n,
 input wire [23:0] I_reader_rgb,I_post_rgb,
 input wire [31:0] I_reader_source_id,
 input wire I_reader_source_valid,
 input wire I_post_de,I_post_user,I_post_last,I_post_good,
 input wire [11:0] I_post_x,I_post_y,
 input wire [3:0] I_mode,
 input wire [31:0] I_capture_seq,I_capture_epoch,
 input wire [63:0] I_context,
 input wire I_diag_valid,
 input wire [31:0] I_diag_capture_seq,I_diag_epoch,
 input wire [3:0] I_diag_mode,
 input wire [2:0] I_diag_status,I_diag_error,
 input wire [7:0] I_diag_max_runs,
 input wire [9:0] I_diag_labels,
 input wire [15:0] I_diag_row_cycles,I_diag_fold_cycles,I_diag_tail_cycles,
 output wire O_valid,output wire [7:0] O_data,input wire I_ready
);
 reg [23:0] reader_pipe[0:PRE_DELAY-1];
 reg [31:0] source_pipe[0:PRE_DELAY-1];
 reg [PRE_DELAY-1:0] valid_pipe;
 integer k;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   valid_pipe<=0;
   for(k=0;k<PRE_DELAY;k=k+1)begin reader_pipe[k]<=0;source_pipe[k]<=0;end
  end else begin
   reader_pipe[0]<=I_reader_rgb;source_pipe[0]<=I_reader_source_id;valid_pipe[0]<=I_reader_source_valid;
   for(k=1;k<PRE_DELAY;k=k+1)begin
    reader_pipe[k]<=reader_pipe[k-1];source_pipe[k]<=source_pipe[k-1];valid_pipe[k]<=valid_pipe[k-1];
   end
  end
 end
 wire sof=I_post_de&&I_post_user;
 // At SOF, core telemetry registers the OLD frame while capture IDs describe
 // the NEW frame. Keep both mappings; compare OLD telemetry one clock later.
 reg current_valid,previous_valid;
 reg [31:0] current_seq,current_source,current_epoch,previous_seq,previous_source,previous_epoch;
 reg [3:0] current_mode,previous_mode;
 reg result_valid;
 reg [31:0] result_source,result_seq,result_epoch;
 reg [3:0] result_mode;
 reg [2:0] result_status,result_error;
 reg [7:0] result_runs;
 reg [9:0] result_labels;
 reg [15:0] result_row,result_fold,result_tail;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   current_valid<=0;previous_valid<=0;current_seq<=0;current_source<=0;current_epoch<=0;current_mode<=0;
   previous_seq<=0;previous_source<=0;previous_epoch<=0;previous_mode<=0;
   result_valid<=0;result_source<=0;result_seq<=0;result_epoch<=0;result_mode<=0;
   result_status<=0;result_error<=0;result_runs<=0;result_labels<=0;result_row<=0;result_fold<=0;result_tail<=0;
  end else begin
   result_valid<=0;
   if(sof)begin
    previous_valid<=current_valid;previous_seq<=current_seq;previous_source<=current_source;
    previous_epoch<=current_epoch;previous_mode<=current_mode;
    current_valid<=1;current_seq<=I_capture_seq;current_source<=source_pipe[PRE_DELAY-1];
    current_epoch<=I_capture_epoch;current_mode<=I_mode;
   end
   if(I_diag_valid&&previous_valid&&I_diag_capture_seq==previous_seq&&
      I_diag_epoch==previous_epoch&&I_diag_mode==previous_mode)begin
    result_valid<=1;result_source<=previous_source;result_seq<=previous_seq;result_epoch<=previous_epoch;result_mode<=previous_mode;
    result_status<=I_diag_status;result_error<=I_diag_error;result_runs<=I_diag_max_runs;result_labels<=I_diag_labels;
    result_row<=I_diag_row_cycles;result_fold<=I_diag_fold_cycles;result_tail<=I_diag_tail_cycles;
   end
  end
 end
 colour_diag_capture #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.PERIOD_FRAMES(PERIOD_FRAMES),
  .CAPTURE_TIMEOUT(CAPTURE_TIMEOUT),.RESULT_TIMEOUT(RESULT_TIMEOUT)) capture(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(1'b1),.I_cancel(1'b0),
  .I_de(I_post_de),.I_user(I_post_user),.I_last(I_post_last),.I_pixel_good(I_post_good),.I_source_valid(valid_pipe[PRE_DELAY-1]),
  .I_x(I_post_x),.I_y(I_post_y),.I_pre_rgb(reader_pipe[PRE_DELAY-1]),.I_post_rgb(I_post_rgb),
  .I_source_id(source_pipe[PRE_DELAY-1]),.I_epoch(I_capture_epoch),.I_capture_seq(I_capture_seq),.I_mode(I_mode),.I_context(I_context),
  .I_result_valid(result_valid),.I_result_source_id(result_source),.I_result_epoch(result_epoch),.I_result_capture_seq(result_seq),
  .I_result_mode(result_mode),.I_result_status(result_status),.I_result_error(result_error),
  .I_max_runs(result_runs),.I_labels(result_labels),.I_max_row_cycles(result_row),.I_fold_cycles(result_fold),.I_tail_cycles(result_tail),
  .O_valid(O_valid),.O_data(O_data),.O_last(),.I_ready(I_ready),.O_busy(),.O_capturing(),.O_config_bad(),.O_packet_done());
endmodule
