// Four explicit 16384x1 blocks. Separate read registers are essential for TD's
// ERAM inference: a conditional read of two unrelated memories into one reg
// can fall back to distributed LUT RAM, even with a bram attribute.
module shape_bit_bank(
 input wire I_clk,I_we,I_re,
 input wire [15:0] I_waddr,I_raddr,
 input wire I_wbit,
 output wire O_rbit
);
 wire [3:0] data;
 reg [1:0] selected;
 always @(posedge I_clk)if(I_re)selected<=I_raddr[15:14];
 genvar g;
 generate for(g=0;g<4;g=g+1)begin:blocks
  (* ram_style="bram" *) reg memory[0:16383];
  reg q;
  always @(posedge I_clk)begin
   if(I_we&&I_waddr[15:14]==g)memory[I_waddr[13:0]]<=I_wbit;
   if(I_re&&I_raddr[15:14]==g)q<=memory[I_raddr[13:0]];
  end
  assign data[g]=q;
 end endgenerate
 assign O_rbit=data[selected];
endmodule
