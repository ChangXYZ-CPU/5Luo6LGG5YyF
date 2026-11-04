// Private R/G-only projection of the exact R05 field; two-clock alignment.
module green_gain_rom_r14 #(parameter integer WIDTH=1280,HEIGHT=720,parameter integer USE_EXTERNAL_WORD=0,parameter MEM_FILE="gain_rg_1280_720.mem")(
 input wire[17:0]I_word_early,
 input wire I_clk,I_rst_n,input wire[11:0]I_x,I_y,
 output wire[8:0]O_gain_r,O_gain_g,output wire O_good
);
 localparam integer NX=(WIDTH-1+16)/32+1;
 localparam integer VALID=WIDTH>=12&&WIDTH<=1280&&HEIGHT>=12&&HEIGHT<=720&&NX*((HEIGHT-1+16)/32+1)<=1024;
 wire[12:0]qx={1'b0,I_x}+13'd16,qy={1'b0,I_y}+13'd16;
 function[12:0]row_base;input[7:0]idx;integer j;reg[12:0]a;
 begin a=0;for(j=0;j<6;j=j+1)if((NX>>j)&1)a=a+({5'd0,idx}<<j);row_base=a;end endfunction
 wire[12:0]address=row_base(qy[12:5])+{5'd0,qx[12:5]};
 wire[17:0]word;reg[17:0]aligned;reg[1:0]ok;
 // External word must equal the same-profile ROM at the PRE coordinate,
 // already registered one clock. A separate reset/alignment stage is retained.
 generate if(USE_EXTERNAL_WORD==1)begin:shared
  assign word=I_word_early;
 end else begin:dedicated
  (* ram_style="bram" *)reg[17:0]mem[0:1023];reg[17:0]read_word;
  initial $readmemh(MEM_FILE,mem);
  always @(posedge I_clk)read_word<=mem[address[9:0]];
  assign word=read_word;
 end endgenerate
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin aligned<=0;ok<=0;end
  else begin aligned<=word;ok<={ok[0],VALID&&I_x<WIDTH&&I_y<HEIGHT};end
 end
 assign O_gain_r=aligned[8:0];assign O_gain_g=aligned[17:9];
 assign O_good=ok[1]&&O_gain_r>=1&&O_gain_r<=305&&O_gain_g>=1&&O_gain_g<=305;
endmodule
