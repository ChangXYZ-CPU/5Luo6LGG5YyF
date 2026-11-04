// One metadata entry is sufficient: red_cca_stream publishes frame N at SOF N+1.
// I_capture_seq is the detector's input-frame sequence at this SOF, NOT its
// result's source ID. Integration must expose that wire explicitly and prove it.
module zone_result_link #(
 parameter integer WIDTH=1280,HEIGHT=720
)(
 input wire I_clk,I_rst_n,I_enable,I_source_rst_n,
 input wire I_de,I_sof,I_eol,I_pixel_good,I_source_valid,
 input wire [11:0] I_x,I_y,
 input wire [31:0] I_source_id,I_capture_seq,I_result_seq,I_result_epoch,
 input wire [20:0] I_config,
 input wire [2:0] I_status,
 input wire I_target_valid,I_coords_valid,
 input wire [3:0] I_result_mode,
 input wire [11:0] I_center_x,I_center_y,
 output reg O_result_event,O_fault_event,
 output wire O_reconfigure,
 output reg O_target_valid,O_coords_valid,
 output reg [31:0] O_source_id,
 output reg [11:0] O_center_x,O_center_y,
 output wire O_monitoring
);
 reg enabled_d,have_frame,complete,frame_bad,active,reconfigure_q;
 reg [31:0] source,frame_seq,epoch;
 reg [20:0] config_d;
 wire sof=I_de && I_sof;
 wire config_change=I_config!=config_d;
 assign O_monitoring=I_enable && I_source_rst_n && have_frame && !config_change;
 // Immediate assertion prevents a queued old-configuration event from being
 // consumed on this edge; keep a registered pulse for downstream WAIT setup.
 assign O_reconfigure=I_enable && (reconfigure_q || config_change || !enabled_d);
 // Mode/config can change halfway through an old raster. WAIT until a legal
 // first SOF before judging captured pixels; watchdog still runs meanwhile.
 wire pixel_bad=I_de && (have_frame || sof) && (!I_pixel_good || !I_source_valid ||
  (!sof && (!active || I_source_id!=source)));
 wire matching=have_frame && complete && !frame_bad &&
  I_result_seq==frame_seq && I_result_epoch==epoch && I_result_mode==6;
 wire healthy=I_status==2 || (I_status==3 && I_target_valid && I_coords_valid &&
  I_center_x<WIDTH && I_center_y<HEIGHT);
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   enabled_d<=0;config_d<=0;have_frame<=0;complete<=0;frame_bad<=0;active<=0;
   source<=0;frame_seq<=0;epoch<=0;
   O_result_event<=0;O_fault_event<=0;reconfigure_q<=0;
   O_target_valid<=0;O_coords_valid<=0;O_source_id<=0;O_center_x<=0;O_center_y<=0;
  end else begin
   enabled_d<=I_enable;config_d<=I_config;
   O_result_event<=0;O_fault_event<=0;reconfigure_q<=0;
   if(!I_enable || !I_source_rst_n) begin
    have_frame<=0;complete<=0;frame_bad<=0;active<=0;
    if(I_enable && !I_source_rst_n) O_fault_event<=1;
   end else if(!enabled_d || config_change) begin
    // Flush both old associations and pending counts, not the alarm latch.
    reconfigure_q<=1;have_frame<=0;complete<=0;frame_bad<=0;active<=0;
    // This SOF is already under the new config; capture it for the next result.
    if(sof) begin
     have_frame<=1;source<=I_source_id;frame_seq<=I_capture_seq;epoch<=I_result_epoch;
     frame_bad<=!I_pixel_good || !I_source_valid || I_x!=0 || I_y!=0;
     complete<=I_eol && I_x==WIDTH-1 && I_y==HEIGHT-1;
     active<=!(I_eol && I_x==WIDTH-1 && I_y==HEIGHT-1);
     if(!I_pixel_good || !I_source_valid || I_x!=0 || I_y!=0) O_fault_event<=1;
    end
   end else begin
    if(sof) begin
     // Result is sampled BEFORE nonblocking replacement by the new frame.
     // No association yet means this is an old session's publication. Ignore
     // it while starting the new capture; do not label it healthy or FAULT.
     if(have_frame && (I_status>=4 || (I_status>=2 && (!matching || !healthy)))) O_fault_event<=1;
     else if(have_frame && I_status>=2 && matching && healthy) begin
      O_result_event<=1;O_source_id<=source;
      O_target_valid<=I_status==3;O_coords_valid<=I_status==3;
      O_center_x<=I_status==3?I_center_x:12'd0;
      O_center_y<=I_status==3?I_center_y:12'd0;
     end
     if(have_frame && !complete) O_fault_event<=1;
     have_frame<=1;source<=I_source_id;frame_seq<=I_capture_seq;epoch<=I_result_epoch;
     frame_bad<=!I_pixel_good || !I_source_valid || I_x!=0 || I_y!=0;
     complete<=I_eol && I_x==WIDTH-1 && I_y==HEIGHT-1;
     active<=!(I_eol && I_x==WIDTH-1 && I_y==HEIGHT-1);
    end else if(I_de && have_frame) begin
     if(pixel_bad) frame_bad<=1;
     if(I_eol && I_x==WIDTH-1 && I_y==HEIGHT-1) begin complete<=1;active<=0;end
    end
    // Current first/middle/last-pixel failure cannot be hidden by old success.
    if(pixel_bad || (sof && (I_x!=0 || I_y!=0))) O_fault_event<=1;
   end
  end
 end
endmodule
