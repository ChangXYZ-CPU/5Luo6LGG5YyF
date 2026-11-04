// Detector-only protocol guard. No ready/backpressure and no video modification.
// Combinational outputs are sampled by red_mask_stream at the same edge as RGB.
module red_frame_guard #(
 parameter integer WIDTH=1280, HEIGHT=720
)(
 input wire I_clk,I_rst_n,I_enable,I_de,I_user,I_last,I_pixel_good,
 input wire [11:0] I_x,I_y,
 output wire O_accept,O_frame_start,O_frame_end,O_bad_event,O_frame_bad,
 output wire O_config_bad
);
 localparam PARAM_OK=(WIDTH>=1 && WIDTH<=1280 && HEIGHT>=1 && HEIGHT<=720);
 reg active,bad;
 reg [11:0] next_x,next_y;
 wire enabled=I_rst_n && I_enable;
 wire start=I_de && I_user;
 wire final_pixel=(I_x==WIDTH-1 && I_y==HEIGHT-1);
 wire malformed=I_de ?
   (I_user ? (active || I_x!=0 || I_y!=0 || I_last!=(WIDTH==1) || !I_pixel_good) :
             (!active || I_x!=next_x || I_y!=next_y ||
              I_last!=(I_x==WIDTH-1) || !I_pixel_good)) : (I_user || I_last);
 wire violation=!PARAM_OK || malformed;
 assign O_config_bad=!PARAM_OK;
 assign O_frame_start=enabled && start;
 // After a fault, wait for SOF; never accept a good-looking mid-frame suffix.
 assign O_accept=enabled && !violation && I_de && (start || !bad);
 assign O_frame_end=O_accept && final_pixel;
 assign O_bad_event=enabled && violation && (!bad || start);
 assign O_frame_bad=enabled && (violation || (bad && !start));
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin active<=0;bad<=0;next_x<=0;next_y<=0;end
  else if(!I_enable) begin active<=0;bad<=0;next_x<=0;next_y<=0;end
  else if(violation) begin active<=0;bad<=1;next_x<=0;next_y<=0;end
  else if(O_accept) begin
   bad<=0;
   active<=!final_pixel;
   if(I_x==WIDTH-1) begin next_x<=0;next_y<=I_y+1'b1;end
   else begin next_x<=I_x+1'b1;next_y<=I_y;end
  end
 end
endmodule
