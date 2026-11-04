// One signed multiplier pipelines six products across cross/length/dot.
// Legal coordinate widths: 2..12 (28-bit outputs cover that entire domain).
// Default=12; classifier=10, sufficient for absolute grid and centered radius.
module shape_geometry #(parameter integer COORD_BITS=12)(
 input wire I_clk,I_rst_n,I_cancel,I_start,
 input wire signed [COORD_BITS-1:0] I_ax,I_ay,I_bx,I_by,I_cx,I_cy,
 output reg O_busy,O_done,
 output reg signed [27:0] O_cross,O_dot,
 output reg [27:0] O_length2
);
 reg [2:0] phase;
 reg signed [COORD_BITS:0] bax,bay,cax,cay;
 reg signed [COORD_BITS:0] mul_a,mul_b;
 reg signed [2*COORD_BITS+1:0] product,first_product;
 wire signed [27:0] wide_first=first_product,wide_product=product;
 always @* begin
  mul_a=0;mul_b=0;
  case(phase)
   0:begin mul_a=bax;mul_b=cay;end
   1:begin mul_a=bay;mul_b=cax;end
   2:begin mul_a=cax;mul_b=cax;end
   3:begin mul_a=cay;mul_b=cay;end
   4:begin mul_a=-bax;mul_b=cax-bax;end
   5:begin mul_a=-bay;mul_b=cay-bay;end
   default:begin mul_a=0;mul_b=0;end
  endcase
 end
 // No reset on this private pipeline register: phase 0 always writes it
 // before phase 1 saves/uses it, including after cancellation or reset.
 always @(posedge I_clk)if(I_rst_n&&!I_cancel&&O_busy&&phase<6)product<=mul_a*mul_b;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   O_busy<=0;O_done<=0;O_cross<=0;O_dot<=0;O_length2<=0;phase<=0;
   bax<=0;bay<=0;cax<=0;cay<=0;first_product<=0;
  end else if(I_cancel) begin O_busy<=0;O_done<=0;phase<=0;end
  else begin
   O_done<=0;
   if(!O_busy) begin
    if(I_start) begin
     bax<=$signed({I_bx[COORD_BITS-1],I_bx})-$signed({I_ax[COORD_BITS-1],I_ax});
     bay<=$signed({I_by[COORD_BITS-1],I_by})-$signed({I_ay[COORD_BITS-1],I_ay});
     cax<=$signed({I_cx[COORD_BITS-1],I_cx})-$signed({I_ax[COORD_BITS-1],I_ax});
     cay<=$signed({I_cy[COORD_BITS-1],I_cy})-$signed({I_ay[COORD_BITS-1],I_ay});
     phase<=0;O_busy<=1;
    end
   end else case(phase)
    0:phase<=1;
    1:begin first_product<=product;phase<=2;end
    2:begin O_cross<=wide_first-wide_product;phase<=3;end
    3:begin first_product<=product;phase<=4;end
    4:begin O_length2<=wide_first+wide_product;phase<=5;end
    5:begin first_product<=product;phase<=6;end
    6:begin O_dot<=wide_first+wide_product;O_busy<=0;O_done<=1;end
    default:begin O_busy<=0;O_done<=1;end
   endcase
  end
 end
endmodule
