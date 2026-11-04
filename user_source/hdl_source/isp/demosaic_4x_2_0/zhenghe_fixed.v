`timescale 1ns/1ps
// Complete-row buffering. Four 512-word banks in the existing 2048x32 ERAM.
// Reader is not reset at SOF: the preceding frame may still be draining.
module zhenghe #(
 parameter [12:0] IMG_WIDTH=640,IMG_HEIGHT=480,
 parameter [8:0] DATA_WIDTH=32,data_complete_delay=50
)(
 input I_clk,I_rst_n,I_tlast,I_tuser,input [DATA_WIDTH-1:0] I_tdata,
 input I_tvalid,output reg O_tlast,O_tuser,
 output [DATA_WIDTH-1:0] O_tdata,output reg O_tvalid
);
 localparam WORDS=IMG_WIDTH/4;
 reg [1:0] write_bank,read_bank;
 reg [8:0] write_x,read_x;
 reg [3:0] ready,bank_sof;
 reg reading,sof_pending,row_sof;
 wire first=I_tvalid&&(I_tuser||sof_pending);
 wire [8:0] wx=first?9'd0:write_x;
 wire completed=I_tvalid&&(wx==WORDS-1);
 wire [1:0] next_bank=read_bank+1'b1;
 wire next_available=ready[next_bank]||(completed&&write_bank==next_bank);
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   write_bank<=0;read_bank<=0;write_x<=0;read_x<=0;
   ready<=0;bank_sof<=0;reading<=0;sof_pending<=0;row_sof<=0;
   O_tvalid<=0;O_tuser<=0;O_tlast<=0;
  end else begin
   if(I_tuser) sof_pending<=1;
   if(first) begin sof_pending<=0;row_sof<=1;end
   if(I_tvalid) begin
    if(completed) begin
     write_x<=0;write_bank<=write_bank+1'b1;ready[write_bank]<=1;
     bank_sof[write_bank]<=row_sof||first;row_sof<=0;
    end else write_x<=wx+1'b1;
   end
   if(!reading&&ready[read_bank]) begin reading<=1;read_x<=0;end
   if(reading) begin
    if(read_x==WORDS-1) begin
     ready[read_bank]<=0;read_bank<=next_bank;read_x<=0;
     reading<=next_available;
    end else read_x<=read_x+1'b1;
   end
   O_tvalid<=reading;
   O_tuser<=reading&&(read_x==0)&&bank_sof[read_bank];
   O_tlast<=reading&&(read_x==WORDS-1);
  end
 end
 blk_mem_gen_zhenghe u_rows(
  .clka(I_clk),.wea(I_tvalid),.addra({write_bank,wx}),.dia(I_tdata),.doa(),
  .clkb(I_clk),.web(1'b0),.addrb({read_bank,read_x}),.dib(32'd0),.dob(O_tdata)
 );
endmodule
