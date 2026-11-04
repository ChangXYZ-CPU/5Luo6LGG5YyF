// Consume the real motion backend's atomic SOF publication, never a held box.
// No RGB delay, secondary detector, frame store or synthetic frame counter.
module motion_guard_link #(
 parameter integer WIDTH=1280,HEIGHT=720
)(
 input wire I_clk,I_rst_n,I_enable,I_source_rst_n,
 input wire I_de,I_sof,I_pixel_good,I_source_valid,
 input wire [11:0] I_x,I_y,
 input wire I_stream_abort,I_reference_clear,I_pending,I_region_fault,
 input wire [2:0] I_region_error,
 input wire [177:0] I_snapshot,
 input wire I_control_fault,
 output wire O_result_event,O_fault_event,
 output wire [3:0] O_reason
);
 reg monitoring;
 reg [3:0] reason_q;
 wire sof=I_de && I_sof;
 // Mode entry can be mid-raster; wait for a legal first pixel. The policy
 // watchdog is already running, so missing SOF/results cannot wait forever.
 wire first_good=sof && I_x==0 && I_y==0 && I_pixel_good && I_source_valid &&
  I_source_rst_n && !I_stream_abort;
 wire watching=monitoring || first_good;
 wire pixel_bad=(watching || sof) && I_de && (!I_pixel_good || !I_source_valid);
 wire stream_bad=monitoring && I_stream_abort;
 wire publish=sof && I_pending && !I_reference_clear;
 wire packet_ok=I_snapshot[52] && I_snapshot[120:117]==4'd11 &&
  ((I_snapshot[50:48]==2 && !I_snapshot[51] && !I_snapshot[177]) ||
   (I_snapshot[50:48]==3 && I_snapshot[51] && I_snapshot[177] &&
    I_snapshot[176:165]<WIDTH && I_snapshot[164:153]<HEIGHT &&
    I_snapshot[47:36]<=I_snapshot[35:24] && I_snapshot[35:24]<WIDTH &&
    I_snapshot[23:12]<=I_snapshot[11:0] && I_snapshot[11:0]<HEIGHT &&
    I_snapshot[176:165]>=I_snapshot[47:36] && I_snapshot[176:165]<=I_snapshot[35:24] &&
    I_snapshot[164:153]>=I_snapshot[23:12] && I_snapshot[164:153]<=I_snapshot[11:0]));
 wire packet_bad=watching && publish && !packet_ok;
 wire region_bad=monitoring && I_region_fault;
 assign O_fault_event=I_rst_n && I_enable &&
  (!I_source_rst_n || pixel_bad || stream_bad || region_bad || packet_bad);
 assign O_result_event=I_rst_n && I_enable && watching && publish &&
  packet_ok && !O_fault_event;
 // Diagnostic numbers refer to observed sources, not guessed camera causes:
 // 1 source reset; 2 reader quality; 3 stream/protocol; 4 region capacity;
 // 5 other region failure; 6 region budget; 7 malformed publication; 8 timeout.
 wire [3:0] reason_now=!I_source_rst_n?4'd1:pixel_bad?4'd2:
  region_bad?(I_region_error==4?4'd4:I_region_error==6?4'd6:4'd5):
  stream_bad?4'd3:packet_bad?4'd7:4'd8;
 // A watchdog fault is registered in the policy on this edge. Its reason
 // must already be valid if the OSD snapshots on the following first pixel.
 assign O_reason=I_control_fault && reason_q==0?4'd8:reason_q;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin monitoring<=0;reason_q<=0;end
  else if(!I_enable)begin monitoring<=0;reason_q<=0;end
  else begin
   if(!I_source_rst_n)monitoring<=0;
   else if(first_good)monitoring<=1;
   // Preserve the first cause through consecutive faults, clear only after
   // the policy has recovered. A timeout has no detector error pulse.
   if(O_fault_event && !I_control_fault)reason_q<=reason_now;
   else if(I_control_fault && reason_q==0)reason_q<=8;
   else if(!I_control_fault && !O_fault_event)reason_q<=0;
  end
 end
endmodule
