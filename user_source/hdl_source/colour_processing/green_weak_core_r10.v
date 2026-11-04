// Private candidate, not connected to production. Three registered clocks.
// Caller must align actual Q8 grid coefficients with the 4x4 PRE sums.
module green_weak_core_r10 #(parameter DEN_SHIFT=6,NOISE_HALF_DN=0)(
 input wire I_clk,I_rst_n,I_valid,I_user,I_last,I_good,I_enable,
 input wire[11:0] I_sum_r,I_sum_g,I_sum_b,
 input wire[7:0] I_min,I_dom,input wire[8:0] I_gain_r,I_gain_g,
 input wire[11:0]I_x,I_y,input wire[31:0]I_source_id,
 output wire O_valid,O_user,O_last,O_good,output reg O_weak,
 output wire[11:0]O_x,O_y,output wire[31:0]O_source_id
);
 localparam PARAM_GOOD=(DEN_SHIFT>=5&&DEN_SHIFT<=7&&NOISE_HALF_DN>=0&&NOISE_HALF_DN<=4);
 wire range_good=PARAM_GOOD&&(I_sum_r<=4080)&&(I_sum_g<=4080)&&(I_sum_b<=4080)&&
  I_gain_r>=1&&I_gain_r<=305&&I_gain_g>=1&&I_gain_g<=305;
 reg[20:0]pr0,pg0;reg[7:0]min0,dom0;reg band0,en0;
 wire[12:0]r=pr0[20:8],g=pg0[20:8];
 wire[12:0]delta=g>=r?g-r:13'd0;
 wire[12:0]excess=g>1024?g-13'd1024:13'd0;
 wire[20:0]dom_product=g*dom0;
 wire[21:0]wide_delta={9'd0,delta},wide_excess={9'd0,excess};
 reg[21:0]lhs1,rhs1;reg eligible1;
 wire[7:0]floor_min=min0<32?8'd32:min0;
 reg[2:0]vp,up,lp,gp;reg[11:0]xp[0:2],yp[0:2];reg[31:0]sp[0:2];integer n;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   pr0<=0;pg0<=0;min0<=0;dom0<=0;band0<=0;en0<=0;lhs1<=0;rhs1<=0;eligible1<=0;O_weak<=0;
   vp<=0;up<=0;lp<=0;gp<=0;
   for(n=0;n<3;n=n+1)begin xp[n]<=0;yp[n]<=0;sp[n]<=0;end
  end else begin
   pr0<=I_sum_r*I_gain_r;pg0<=I_sum_g*I_gain_g;min0<=I_min;dom0<=I_dom;en0<=I_enable;
   band0<=({2'd0,I_sum_b}<<2)>=(({2'd0,I_sum_g}<<1)+{2'd0,I_sum_g})&&
          ({4'd0,I_sum_b}<<3)<=(({4'd0,I_sum_g}<<3)+{4'd0,I_sum_g});
   lhs1<=wide_delta<<DEN_SHIFT;
   rhs1<={7'd0,dom_product[20:6]}+(wide_excess<<DEN_SHIFT)+(NOISE_HALF_DN*8*(1<<DEN_SHIFT));
   eligible1<=vp[0]&&gp[0]&&en0&&band0&&(g>=r)&&(g>={floor_min,4'd0});
   O_weak<=eligible1&&(lhs1>=rhs1);
   vp<={vp[1:0],I_valid};up<={up[1:0],I_user};lp<={lp[1:0],I_last};gp<={gp[1:0],I_valid&&I_good&&range_good};
   xp[0]<=I_x;yp[0]<=I_y;sp[0]<=I_source_id;
   for(n=1;n<3;n=n+1)begin xp[n]<=xp[n-1];yp[n]<=yp[n-1];sp[n]<=sp[n-1];end
  end
 end
 assign O_valid=vp[2];assign O_user=up[2];assign O_last=lp[2];assign O_good=gp[2];
 assign O_x=xp[2];assign O_y=yp[2];assign O_source_id=sp[2];
endmodule
