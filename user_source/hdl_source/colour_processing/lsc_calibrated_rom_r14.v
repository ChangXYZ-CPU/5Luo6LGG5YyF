// Private display-link candidate; controls commit at valid SOF when DEFER_CONTROLS=1.
// No production installation; saturation remains an independent reusable input.
// Retains five-clock RGB/tag latency, saturation (including bypass), and E/C/b
// manual trim directions. Defaults 60/50/16 mean the calibrated field itself.
// Runtime knob differences are exact A14 gain differences, then clamp 1..305.
module lsc_calibrated_rom_r14 #(
 parameter integer WIDTH=1280,HEIGHT=720,
 parameter integer DEFER_CONTROLS=1,
 parameter [242:0] KNOTS=243'h39a00d2803e9fef480371c5006738d99007a401bae280351eefa803ca00dc,
 parameter MEM_FILE="field_1280_720.mem"
)(
 input wire I_clk,I_rst_n,I_de,I_user,I_last,I_good,I_enable,
 input wire [11:0] I_x,I_y,
 input wire [23:0] I_rgb,input wire [31:0] I_source_id,
 input wire [7:0] I_edge,I_centre,I_lift,input wire [4:0] I_sat,
 output wire[17:0] O_raw_gain_rg,
 output wire O_de,O_user,O_last,O_good,O_config_bad,
 output wire[7:0] O_frame_edge,O_frame_centre,O_frame_lift,
 output wire [11:0] O_x,O_y,output reg [23:0] O_rgb,output wire [31:0] O_source_id
);
 reg [7:0] frame_e,frame_c,frame_b;
 reg [4:0] frame_sat;
 reg frame_enable;
 wire changed=!DEFER_CONTROLS&&I_de&&!I_user&&(I_edge!=frame_e||I_centre!=frame_c||I_lift!=frame_b||I_sat!=frame_sat);
 wire [7:0] effective_e=I_user?I_edge:frame_e,effective_c=I_user?I_centre:frame_c,effective_b=I_user?I_lift:frame_b;
 wire [4:0] effective_sat=I_user?I_sat:frame_sat;
 wire effective_enable=I_user?I_enable:frame_enable;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin frame_e<=60;frame_c<=50;frame_b<=16;frame_sat<=16;frame_enable<=0;end
  else if(I_de&&I_user)begin frame_e<=I_edge;frame_c<=I_centre;frame_b<=I_lift;frame_sat<=I_sat;frame_enable<=I_enable;end
 end
 wire [8:0] base_r,base_g,base_b;
 wire accept_early;
 spatial_colour_rom_r14 #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.KNOTS(KNOTS),.MEM_FILE(MEM_FILE)) field(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_de(I_de),.I_user(I_user),.I_last(I_last),.I_good(I_good&&!changed),
  .I_enable(DEFER_CONTROLS?effective_enable:I_enable),.I_x(I_x),.I_y(I_y),.I_rgb(24'd0),.I_source_id(I_source_id),
  .O_de(O_de),.O_user(O_user),.O_last(O_last),.O_good(O_good),.O_config_bad(O_config_bad),
  .O_x(O_x),.O_y(O_y),.O_source_id(O_source_id),.O_rgb(),.O_gain_r(),.O_gain_g(),.O_gain_b(),
  .O_gain_r_early(base_r),.O_gain_g_early(base_g),.O_gain_b_early(base_b),.O_accept_early(accept_early),.O_raw_gain_rg(O_raw_gain_rg));
 wire signed [12:0] dx=$signed({1'b0,I_x})-$signed(WIDTH/2);
 wire signed [12:0] dy=$signed({1'b0,I_y})-$signed(HEIGHT/2);
 reg [25:0] dx2,dy2;
 wire [26:0] radius_sum={1'b0,dx2}+{1'b0,dy2};
 reg [7:0] radius;
 reg [23:0] rgb0,rgb1,rgb2,rgb3;
 reg [8:0] r0,g0,b0,r1,g1,b1;
 reg [7:0] e0,c0,bb0,e1,c1,bb1;
 reg[7:0] e2,c2,bb2,e3,c3,bb3,e4,c4,bb4;
 assign O_frame_edge=e4;assign O_frame_centre=c4;assign O_frame_lift=bb4;
 reg en0,en1;
 reg [4:0] sat0,sat1,sat2,sat3;
 // Exact arithmetic reduction, not an approximation or changed gain law.
 // ceil(60*r/256) = (r>>2)-(r>>6)+(r[1:0]>r[5:4]), for every r=0..255.
 wire [15:0] e_product=e1*radius,c_product=c1*radius;
 wire [8:0] e_ceiling={1'b0,e_product[15:8]}+{8'd0,(|e_product[7:0])};
 wire [5:0] ref_e_ceiling=radius[7:2]-{4'd0,radius[7:6]}+{5'd0,(radius[1:0]>radius[5:4])};
 wire [13:0] c_reference=14'd50*radius;
 wire signed [10:0] rr=$signed({2'd0,r1})-$signed({2'd0,e_ceiling})+$signed({5'd0,ref_e_ceiling});
 wire signed [10:0] gr=$signed({2'd0,g1})+11'sd50-$signed({3'd0,c1})+$signed({3'd0,c_product[15:8]})-$signed({5'd0,c_reference[13:8]});
 wire signed [10:0] br=$signed({2'd0,b1})+11'sd16-$signed({3'd0,bb1});
 function [8:0] bound_gain;
  input signed [10:0] value;
  begin bound_gain=value<1?9'd1:value>305?9'd305:value[8:0];end
 endfunction
 reg [8:0] gain_r,gain_g,gain_b;
 wire [16:0] prod_r=rgb2[23:16]*gain_r,prod_g=rgb2[15:8]*gain_g,prod_b=rgb2[7:0]*gain_b;
 wire [7:0] clip_r=prod_r[16:8]>255?8'd255:prod_r[15:8];
 wire [7:0] clip_g=prod_g[16:8]>255?8'd255:prod_g[15:8];
 wire [7:0] clip_b=prod_b[16:8]>255?8'd255:prod_b[15:8];
 wire [15:0] lum=16'd77*rgb3[23:16]+16'd150*rgb3[15:8]+16'd29*rgb3[7:0];
 wire [7:0] grey=lum[15:8];
 wire signed [9:0] dr=$signed({2'b0,rgb3[23:16]})-$signed({2'b0,grey});
 wire signed [9:0] dg=$signed({2'b0,rgb3[15:8]})-$signed({2'b0,grey});
 wire signed [9:0] db=$signed({2'b0,rgb3[7:0]})-$signed({2'b0,grey});
 wire signed [15:0] sr=dr*$signed({1'b0,sat3}),sg=dg*$signed({1'b0,sat3}),sb=db*$signed({1'b0,sat3});
 wire signed [16:0] final_r=$signed({1'b0,grey})+(sr>>>4);
 wire signed [16:0] final_g=$signed({1'b0,grey})+(sg>>>4);
 wire signed [16:0] final_b=$signed({1'b0,grey})+(sb>>>4);
 function [7:0] clip_pixel;
  input signed [16:0] value;
  begin clip_pixel=value<0?8'd0:value>255?8'd255:value[7:0];end
 endfunction
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   dx2<=0;dy2<=0;radius<=0;rgb0<=0;rgb1<=0;rgb2<=0;rgb3<=0;
   r0<=256;g0<=256;b0<=256;r1<=256;g1<=256;b1<=256;e0<=60;c0<=50;bb0<=16;e1<=60;c1<=50;bb1<=16;
   e2<=60;c2<=50;bb2<=16;e3<=60;c3<=50;bb3<=16;e4<=60;c4<=50;bb4<=16;
   en0<=0;en1<=0;gain_r<=256;gain_g<=256;gain_b<=256;sat0<=16;sat1<=16;sat2<=16;sat3<=16;O_rgb<=0;
  end else begin
   dx2<=dx*dx;dy2<=dy*dy;rgb0<=I_rgb;r0<=base_r;g0<=base_g;b0<=base_b;
   e0<=effective_e;c0<=effective_c;bb0<=effective_b;en0<=effective_enable&&accept_early;sat0<=effective_sat;
   radius<=(radius_sum>>11)>255?8'd255:radius_sum[18:11];rgb1<=rgb0;r1<=base_r;g1<=base_g;b1<=base_b; // registered ROM belongs to rgb0
   e1<=e0;c1<=c0;bb1<=bb0;en1<=en0;sat1<=sat0;
   gain_r<=en1?bound_gain(rr):9'd256;gain_g<=en1?bound_gain(gr):9'd256;gain_b<=en1?bound_gain(br):9'd256;
   rgb2<=rgb1;sat2<=sat1;e2<=e1;c2<=c1;bb2<=bb1;
   rgb3<={clip_r,clip_g,clip_b};sat3<=sat2;e3<=e2;c3<=c2;bb3<=bb2;
   O_rgb<={clip_pixel(final_r),clip_pixel(final_g),clip_pixel(final_b)};e4<=e3;c4<=c3;bb4<=bb3;
  end
 end
endmodule
