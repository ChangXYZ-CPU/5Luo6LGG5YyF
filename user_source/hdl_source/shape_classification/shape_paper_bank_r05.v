// One 16Kbit bright mask; separate banks infer separate ERAM read registers.
module shape_paper_bank_r05(
 input wire I_clk,I_we,I_re,
 input wire [13:0] I_waddr,I_raddr,input wire I_wbit,output reg O_rbit
);
 (* ram_style="bram" *) reg memory[0:16383];
 always @(posedge I_clk)begin
  if(I_we)memory[I_waddr]<=I_wbit;
  if(I_re)O_rbit<=memory[I_raddr];
 end
endmodule
