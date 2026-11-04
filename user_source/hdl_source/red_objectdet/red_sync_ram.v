// One synchronous read port and one write port. Contents are not reset.
// The controller must not read and write the same address on the same edge.
module red_sync_ram #(parameter integer DW=32,AW=7)(
 input wire I_clk,I_we,I_re,
 input wire [AW-1:0] I_waddr,I_raddr,
 input wire [DW-1:0] I_wdata,
 output reg [DW-1:0] O_rdata
);
 (* ram_style="bram" *) reg [DW-1:0] mem[0:(1<<AW)-1];
 always @(posedge I_clk) begin
  if(I_we) mem[I_waddr]<=I_wdata;
  if(I_re) O_rdata<=mem[I_raddr];
 end
endmodule
