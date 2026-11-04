// M03: one pixel/clock when accepted; bubbles do not advance accumulators.
// Geometry/complete-frame validation belongs to motion_frame_control.
// Every new frame initializes each column on its first row; RAM is not reset.
module motion_gray_blocks #(parameter integer WIDTH=1280,HEIGHT=720)(
 input wire I_clk,I_rst_n,I_clear,I_accept,I_first,I_final,
 input wire [23:0] I_rgb,
 input wire [11:0] I_x,I_y,
 output reg O_valid,O_first,O_final,
 output reg [7:0] O_gray,O_x,
 output reg [6:0] O_y
);
 localparam BW=(WIDTH>=8 && WIDTH<=1280 && WIDTH%8==0)?WIDTH/8:1;
 localparam AW=(BW>1)?$clog2(BW):1;
 reg gv,gfinal;
 reg [7:0] gray;
 reg [11:0] gx,gy;
 reg [10:0] horizontal;
 reg block_seen;
 wire [9:0] weighted={2'b0,I_rgb[23:16]}+{1'b0,I_rgb[15:8],1'b0}+
                     {2'b0,I_rgb[7:0]}+10'd2;
 wire [10:0] row_total=horizontal+{3'b0,gray};
 wire [13:0] row_prior;
 wire [13:0] block_total={3'b0,row_total}+((gy[2:0]==0)?14'd0:row_prior);
 wire [13:0] rounded=block_total+14'd32;
 wire row_read=gv && !I_clear && gx[2:0]==0;
 wire row_write=gv && !I_clear && gx[2:0]==7;
 red_sync_ram #(.DW(14),.AW(AW)) u_row_sums(
  .I_clk(I_clk),.I_we(row_write),.I_re(row_read),
  .I_waddr(gx[AW+2:3]),.I_raddr(gx[AW+2:3]),
  .I_wdata(block_total),.O_rdata(row_prior)
 );
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   gv<=0;gfinal<=0;gray<=0;gx<=0;gy<=0;horizontal<=0;block_seen<=0;
   O_valid<=0;O_first<=0;O_final<=0;O_gray<=0;O_x<=0;O_y<=0;
  end else if(I_clear) begin
   gv<=0;gfinal<=0;horizontal<=0;block_seen<=0;
   O_valid<=0;O_first<=0;O_final<=0;
  end else begin
   gv<=I_accept;
   if(I_accept)begin gray<=weighted[9:2];gx<=I_x;gy<=I_y;gfinal<=I_final;end
   if(I_first && I_accept) block_seen<=0;
   O_valid<=0;O_first<=0;O_final<=0;
   if(gv) begin
    if(gx[2:0]==0) horizontal<={3'b0,gray};
    else if(gx[2:0]==7) horizontal<=0;
    else horizontal<=row_total;
    if(gx[2:0]==7 && gy[2:0]==7) begin
     O_valid<=1;O_first<=!block_seen;O_final<=gfinal;block_seen<=1;
     O_gray<=rounded[13:6];O_x<=gx[10:3];O_y<=gy[9:3];
    end
   end
  end
 end
endmodule
