// One registered stage on the detector side branch, not on the display path.
// O_valid validates this pixel/prefix, NOT the whole frame. Any later bad event
// invalidates all earlier pixels of that frame. Only O_frame_end closes a good frame.
module red_mask_stream #(
 parameter integer WIDTH=1280, HEIGHT=720, EXTERNAL_MASK=0
)(
 input wire I_clk,I_rst_n,I_enable,I_de,I_user,I_last,I_pixel_good,
 input wire [23:0] I_rgb,
 // External shared mask has exactly two clocks more latency than raw controls.
 input wire I_mask_de,I_mask_good,I_mask,
 input wire [11:0] I_mask_x,I_mask_y,
 input wire [11:0] I_x,I_y,
 input wire [7:0] I_th_min,I_th_dom,   // runtime thresholds (were parameters)
 output reg O_enable,O_de,O_user,O_last,O_pixel_good,O_valid,O_red,
 output reg [11:0] O_x,O_y,
 output reg O_frame_start,O_frame_end,O_bad_event,O_frame_bad,
 output wire O_config_bad
);
 // thresholds are runtime ports now; keep COLOR_OK constant to preserve behaviour
 localparam COLOR_OK=1'b1;
 wire [8:0] MIN_EXT={1'b0,I_th_min}, DOM_EXT={1'b0,I_th_dom};
 wire accept,start,ending,bad_event,frame_bad,geometry_bad;
 wire [8:0] red_ext={1'b0,I_rgb[23:16]};
 wire [8:0] green_limit={1'b0,I_rgb[15:8]}+DOM_EXT;
 wire [8:0] blue_limit={1'b0,I_rgb[7:0]}+DOM_EXT;
 wire red_match=(red_ext>=MIN_EXT && red_ext>=green_limit && red_ext>=blue_limit);
 red_frame_guard #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) guard(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable),.I_de(I_de),
  .I_user(I_user),.I_last(I_last),.I_pixel_good(I_pixel_good),.I_x(I_x),.I_y(I_y),
  .O_accept(accept),.O_frame_start(start),.O_frame_end(ending),
  .O_bad_event(bad_event),.O_frame_bad(frame_bad),.O_config_bad(geometry_bad));
 assign O_config_bad=geometry_bad || !COLOR_OK || (EXTERNAL_MASK!=0 && EXTERNAL_MASK!=1);
 generate if(EXTERNAL_MASK==0)begin: legacy
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   O_enable<=0;O_de<=0;O_user<=0;O_last<=0;O_pixel_good<=0;
   O_valid<=0;O_red<=0;O_x<=0;O_y<=0;
   O_frame_start<=0;O_frame_end<=0;O_bad_event<=0;O_frame_bad<=0;
  end else begin
   O_enable<=I_enable;O_de<=I_de;O_user<=I_user;O_last<=I_last;
   O_pixel_good<=I_pixel_good;O_x<=I_x;O_y<=I_y;
   O_valid<=accept && COLOR_OK;
   O_red<=accept && COLOR_OK && red_match;
   O_frame_start<=start;
   O_frame_end<=ending && COLOR_OK;
   O_bad_event<=bad_event || (I_enable && !COLOR_OK && I_de && I_user);
   O_frame_bad<=I_enable && (frame_bad || !COLOR_OK);
  end
 end
end else begin: shared
  // Delay the ORIGINAL guard packet, not a newly reconstructed frame.
  reg [33:0] q0,q1;
  reg external_bad;
  wire qe,qd,qu,ql,qg,qv,qs,qf,qb,qbad;
  wire [11:0] qx,qy;
  assign {qe,qd,qu,ql,qg,qv,qs,qf,qb,qbad,qx,qy}=q1;
  wire mismatch=qe && ((qd!=I_mask_de) || (qd &&
      (qx!=I_mask_x || qy!=I_mask_y || (qv && !I_mask_good))));
  wire poisoned=mismatch || (external_bad && !qs);
  always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin
    q0<=0;q1<=0;external_bad<=0;
    O_enable<=0;O_de<=0;O_user<=0;O_last<=0;O_pixel_good<=0;O_valid<=0;O_red<=0;O_x<=0;O_y<=0;
    O_frame_start<=0;O_frame_end<=0;O_bad_event<=0;O_frame_bad<=0;
   end else begin
    q0<={I_enable,I_de,I_user,I_last,I_pixel_good,accept,start,ending,bad_event,frame_bad,I_x,I_y};
    q1<=q0;
    if(!qe)external_bad<=0;else if(qs)external_bad<=mismatch;else if(mismatch)external_bad<=1;
    O_enable<=qe;O_de<=qd;O_user<=qu;O_last<=ql;O_pixel_good<=qg&&I_mask_good;O_x<=qx;O_y<=qy;
    O_valid<=qv&&!poisoned;O_red<=qv&&!poisoned&&I_mask;
    O_frame_start<=qs;O_frame_end<=qf&&!poisoned;
    O_bad_event<=qb||(mismatch&&(!external_bad||qs));O_frame_bad<=qbad||(qe&&poisoned);
   end
  end
 end endgenerate
endmodule
