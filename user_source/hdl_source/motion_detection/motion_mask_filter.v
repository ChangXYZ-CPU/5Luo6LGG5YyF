// Causal 2x2 majority (>=3 of four), anchored at the current pixel.
// Missing top/left neighbours are zero. No coordinate shift or tail flush.
// Thin targets can vanish; do not call this a centred 3x3 morphological filter.
module motion_mask_filter #(parameter integer WIDTH=160,HEIGHT=90)(
 input wire I_clk,I_rst_n,I_enable,I_clear,I_valid,I_first,I_final,I_mask,
 input wire [7:0] I_x,input wire [6:0] I_y,
 output reg O_valid,O_first,O_final,O_mask,O_fault,
 output reg [7:0] O_x,output reg [6:0] O_y
);
 localparam OK=(WIDTH>=1 && WIDTH<=160 && HEIGHT>=1 && HEIGHT<=90);
 // Unpacked bit memory permits the existing asynchronous-read LUT RAM;
 // the old packed vector required a register bank and wide read mux.
 reg previous_row[0:WIDTH-1]; // synthesis ram_style=dram
 reg left,upper_left,live;
 reg [7:0] expected_x;
 reg [6:0] expected_y;
 wire geometry=I_x<WIDTH && I_y<HEIGHT &&
  (I_first?(I_x==0 && I_y==0 && !live):
   (live && I_x==expected_x && I_y==expected_y)) &&
  (I_final==((I_x==WIDTH-1)&&(I_y==HEIGHT-1)));
 wire upper=(I_y!=0 && I_x<WIDTH)?previous_row[I_x]:1'b0;
 wire [2:0] votes={2'd0,I_mask}+{2'd0,upper}+
  {2'd0,((I_x!=0)?left:1'b0)}+{2'd0,((I_x!=0 && I_y!=0)?upper_left:1'b0)};
 // Do not reset row storage: row zero reconstructs every entry before use.
 always @(posedge I_clk) if(I_rst_n && I_enable && !I_clear && OK &&
  I_valid && geometry && (!O_fault || I_first)) previous_row[I_x]<=I_mask;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   O_valid<=0;O_first<=0;O_final<=0;O_mask<=0;O_x<=0;O_y<=0;O_fault<=0;
   left<=0;upper_left<=0;live<=0;expected_x<=0;expected_y<=0;
  end else begin
   O_valid<=0;O_first<=0;O_final<=0;
   if(!I_enable || I_clear) begin
    O_fault<=0;live<=0;left<=0;upper_left<=0;expected_x<=0;expected_y<=0;O_mask<=0;
   end else if(!OK || (I_valid && !geometry)) begin O_fault<=1;live<=0;O_mask<=0;end
   else if(I_valid && (!O_fault || I_first)) begin
    O_fault<=0;O_valid<=1;O_first<=I_first;O_final<=I_final;
    O_mask<=votes>=3;O_x<=I_x;O_y<=I_y;
    left<=I_mask;upper_left<=upper;live<=!I_final;
    if(I_x==WIDTH-1) begin expected_x<=0;expected_y<=I_y+7'd1;end
    else begin expected_x<=I_x+8'd1;expected_y<=I_y;end
   end
  end
 end
endmodule
