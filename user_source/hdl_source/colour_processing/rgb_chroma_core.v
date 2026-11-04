// P03 shared colour core. Exact P02 arithmetic; external causal 3x3 RGB sums.
// Input RGB sums must describe the SAME local window and tagged pixel.
// No divider, no video backpressure. Three registered stages, one pixel/clock.
// Hue half-widths: R=30deg, G=35deg, B=40deg about 0/120/240deg.
// I_saturation is Q0.8 (64=25%); unrelated to the display SAT control.
module rgb_chroma_core #(
 parameter integer SUM_PIXELS=9
)(
 input wire I_clk,I_rst_n,I_de,I_user,I_last,I_good,
 input wire [11:0] I_x,I_y,I_r_sum,I_g_sum,I_b_sum,
 input wire [1:0] I_colour,
 input wire [7:0] I_min,I_saturation,
 output reg O_de,O_user,O_last,O_good,O_match,O_config_bad,
 output reg [11:0] O_x,O_y
);
 localparam PARAM_OK=SUM_PIXELS>=1&&SUM_PIXELS<=9;
 reg [11:0] selected,other_a,other_b;
 always @*begin
  selected=I_r_sum;other_a=I_g_sum;other_b=I_b_sum;
  case(I_colour)
   1:begin selected=I_g_sum;other_a=I_r_sum;other_b=I_b_sum;end
   2:begin selected=I_b_sum;other_a=I_r_sum;other_b=I_g_sum;end
   default:begin end
  endcase
 end
 wire [11:0] minimum_other=other_a<other_b?other_a:other_b;
 wire [11:0] difference_other=other_a>=other_b?other_a-other_b:other_b-other_a;
 wire [11:0] selected_delta=selected>=minimum_other?selected-minimum_other:12'd0;
 reg de0,user0,last0,good0,bad0,dominant0;
 reg [11:0] x0,y0,selected0,delta0,other_delta0,minimum0;
 reg [7:0] saturation0;
 reg [1:0] colour0;
 reg de1,user1,last1,good1,bad1,dominant1;
 reg [11:0] x1,y1;
 reg minimum_met1;
 reg [19:0] chroma_lhs1,chroma_rhs1;
 reg [15:0] hue_lhs1,hue_rhs1;
 wire [15:0] hue_distance={4'd0,other_delta0},hue_span={4'd0,delta0};
 function [11:0] scaled_minimum;
  input [7:0] value;
  reg [11:0] total;
  integer bit_index;
  begin
   total=0;
   for(bit_index=0;bit_index<4;bit_index=bit_index+1)
    if((SUM_PIXELS>>bit_index)&1)total=total+({4'd0,value}<<bit_index);
   scaled_minimum=total;
  end
 endfunction
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   de0<=0;user0<=0;last0<=0;good0<=0;bad0<=0;dominant0<=0;
   x0<=0;y0<=0;selected0<=0;delta0<=0;other_delta0<=0;minimum0<=0;saturation0<=0;colour0<=0;
   de1<=0;user1<=0;last1<=0;good1<=0;bad1<=0;dominant1<=0;x1<=0;y1<=0;minimum_met1<=0;
   chroma_lhs1<=0;chroma_rhs1<=0;hue_lhs1<=0;hue_rhs1<=0;
   O_de<=0;O_user<=0;O_last<=0;O_good<=0;O_match<=0;O_config_bad<=0;O_x<=0;O_y<=0;
  end else begin
   de0<=I_de;user0<=I_user;last0<=I_last;good0<=I_good;x0<=I_x;y0<=I_y;
   bad0<=!PARAM_OK||I_colour==3;
   dominant0<=selected>=other_a&&selected>=other_b&&selected_delta!=0;
   selected0<=selected;delta0<=selected_delta;other_delta0<=difference_other;
   minimum0<=scaled_minimum(I_min);saturation0<=I_saturation;colour0<=I_colour;
   de1<=de0;user1<=user0;last1<=last0;good1<=good0;bad1<=bad0;dominant1<=dominant0;x1<=x0;y1<=y0;
   minimum_met1<=selected0>=minimum0;
   chroma_lhs1<={delta0,8'd0};chroma_rhs1<=selected0*saturation0;
   case(colour0)
    1:begin hue_lhs1<=(hue_distance<<3)+(hue_distance<<2);hue_rhs1<=(hue_span<<3)-hue_span;end
    2:begin hue_lhs1<=(hue_distance<<1)+hue_distance;hue_rhs1<=hue_span<<1;end
    default:begin hue_lhs1<=hue_distance<<1;hue_rhs1<=hue_span;end
   endcase
   O_de<=de1;O_user<=user1;O_last<=last1;O_good<=good1;O_x<=x1;O_y<=y1;O_config_bad<=bad1;
   O_match<=de1&&good1&&!bad1&&dominant1&&minimum_met1&&chroma_lhs1>=chroma_rhs1&&hue_lhs1<=hue_rhs1;
  end
 end
endmodule
