// Private lower-cost blue complement: exactly sixteen RAW PRE-reader samples.
// Caller owns 4x4 support, frame guard and frozen MIN/DOM. Three-clock latency.
// At DOM104: B/R>=7/6 and B/G>=6/5; no brightness upper cutoff or spatial ROI.
module blue_raw_core_r04 (
 input wire I_clk,I_rst_n,I_valid,I_user,I_last,I_good,I_enable,
 input wire [11:0] I_sum_r,I_sum_g,I_sum_b,
 input wire [7:0] I_min,I_dom,
 input wire [11:0] I_x,I_y,input wire [31:0] I_source_id,
 output wire O_valid,O_user,O_last,O_good,
 output reg O_weak,
 output wire [11:0] O_x,O_y,output wire [31:0] O_source_id
);
 wire range_good=(I_sum_r<=12'd4080)&&(I_sum_g<=12'd4080)&&(I_sum_b<=12'd4080);
 reg [11:0] r0,g0,b0;reg[7:0]min0,dom0;reg en0;
 wire [11:0] dr=(b0>=r0) ? b0-r0 : 12'd0;
 wire [11:0] dg=(b0>=g0) ? b0-g0 : 12'd0;
 wire [7:0] min_floor=(min0<8'd32) ? 8'd32 : min0;
 wire [21:0] wr={10'd0,dr},wg={10'd0,dg};
 // 728=104*7; 624=104*6. Reuse ONE selected*DOM product for both cones.
 wire [21:0] red_lhs=(wr<<9)+(wr<<7)+(wr<<6)+(wr<<4)+(wr<<3);
 wire [21:0] green_lhs=(wg<<9)+(wg<<6)+(wg<<5)+(wg<<4);
 reg [21:0] lhs_r,lhs_g;reg[19:0]rhs;reg eligible;
 reg [2:0]valid_pipe,user_pipe,last_pipe,good_pipe;
 reg [11:0]xp[0:2],yp[0:2];reg[31:0]sp[0:2];integer n;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   r0<=0;g0<=0;b0<=0;min0<=0;dom0<=0;en0<=0;lhs_r<=0;lhs_g<=0;rhs<=0;eligible<=0;O_weak<=0;
   valid_pipe<=0;user_pipe<=0;last_pipe<=0;good_pipe<=0;
   for(n=0;n<3;n=n+1)begin xp[n]<=0;yp[n]<=0;sp[n]<=0;end
  end else begin
   r0<=I_sum_r;g0<=I_sum_g;b0<=I_sum_b;min0<=I_min;dom0<=I_dom;en0<=I_enable;
   lhs_r<=red_lhs;lhs_g<=green_lhs;rhs<=b0*dom0;
   eligible<=valid_pipe[0]&&good_pipe[0]&&en0&&(dr!=0)&&(dg!=0)&&(b0>={min_floor,4'd0});
   O_weak<=eligible&&(lhs_r>={2'd0,rhs})&&(lhs_g>={2'd0,rhs});
   valid_pipe<={valid_pipe[1:0],I_valid};user_pipe<={user_pipe[1:0],I_user};last_pipe<={last_pipe[1:0],I_last};
   good_pipe<={good_pipe[1:0],I_valid&&I_good&&range_good};
   xp[0]<=I_x;yp[0]<=I_y;sp[0]<=I_source_id;
   for(n=1;n<3;n=n+1)begin xp[n]<=xp[n-1];yp[n]<=yp[n-1];sp[n]<=sp[n-1];end
  end
 end
 assign O_valid=valid_pipe[2];assign O_user=user_pipe[2];assign O_last=last_pipe[2];assign O_good=good_pipe[2];
 assign O_x=xp[2];assign O_y=yp[2];assign O_source_id=sp[2];
endmodule
