// M03: one in-place reference plane. Provisional masks are NEVER a complete
// frame result until O_done. Any abort invalidates the whole reference plane.
// Read old at E0, write current at E1: no same-address same-edge dependency.
module motion_history_diff #(
 parameter integer WIDTH=1280,HEIGHT=720,DIFF_THRESHOLD=16
)(
 input wire I_clk,I_rst_n,I_clear,I_frame_start,
 input wire [31:0] I_source_id,I_epoch,
 input wire I_valid,I_first,I_final,
 input wire [7:0] I_gray,I_x,
 input wire [6:0] I_y,
 output reg O_valid,O_first,O_final,O_compare_valid,O_mask,O_done,
 output reg [7:0] O_gray,O_reference_gray,O_diff,O_x,
 output reg [6:0] O_y,
 output reg [31:0] O_source_id,O_reference_id,O_epoch,
 output reg O_reference_valid
);
 localparam BW=(WIDTH>=8 && WIDTH<=1280 && WIDTH%8==0)?WIDTH/8:1;
 localparam BH=(HEIGHT>=8 && HEIGHT<=720 && HEIGHT%8==0)?HEIGHT/8:1;
 localparam BLOCKS=BW*BH;
 localparam AW=(BLOCKS>1)?$clog2(BLOCKS):1;
 reg [AW-1:0] index,paddr;
 reg pending,pfirst,pfinal;
 reg [7:0] pg,px;
 reg [6:0] py;
 reg frame_compare;
 reg [31:0] frame_source,frame_reference,frame_epoch,reference_id,reference_epoch;
 wire [7:0] prior;
 wire [7:0] delta=(pg>=prior)?(pg-prior):(prior-pg);
 red_sync_ram #(.DW(8),.AW(AW)) u_reference_ram(
  .I_clk(I_clk),.I_we(pending && !I_clear),.I_re(I_valid && !I_clear),
  .I_waddr(paddr),.I_raddr(index),.I_wdata(pg),.O_rdata(prior)
 );
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   index<=0;paddr<=0;pending<=0;pfirst<=0;pfinal<=0;pg<=0;px<=0;py<=0;
   frame_compare<=0;frame_source<=0;frame_reference<=0;frame_epoch<=0;
   reference_id<=0;reference_epoch<=0;O_reference_valid<=0;
   O_valid<=0;O_first<=0;O_final<=0;O_compare_valid<=0;O_mask<=0;O_done<=0;
   O_gray<=0;O_reference_gray<=0;O_diff<=0;O_x<=0;O_y<=0;
   O_source_id<=0;O_reference_id<=0;O_epoch<=0;
  end else if(I_clear)begin
   index<=0;pending<=0;frame_compare<=0;O_reference_valid<=0;
   O_valid<=0;O_first<=0;O_final<=0;O_compare_valid<=0;O_mask<=0;O_done<=0;
  end else begin
   pending<=I_valid;O_valid<=pending;O_first<=pending && pfirst;
   O_final<=pending && pfinal;O_done<=pending && pfinal;
   O_compare_valid<=pending && frame_compare;O_mask<=0;
   if(I_frame_start)begin
    index<=0;frame_source<=I_source_id;frame_epoch<=I_epoch;
    frame_compare<=O_reference_valid && reference_epoch==I_epoch && reference_id!=I_source_id;
    frame_reference<=O_reference_valid?reference_id:32'd0;
   end
   if(I_valid)begin
    paddr<=index;index<=index+1'b1;pg<=I_gray;px<=I_x;py<=I_y;
    pfirst<=I_first;pfinal<=I_final;
   end
   if(pending)begin
    O_gray<=pg;O_reference_gray<=frame_compare?prior:8'd0;O_diff<=frame_compare?delta:8'd0;
    O_mask<=frame_compare && delta>DIFF_THRESHOLD;
    O_x<=px;O_y<=py;O_source_id<=frame_source;O_reference_id<=frame_reference;O_epoch<=frame_epoch;
    if(pfinal)begin
     O_reference_valid<=1;reference_id<=frame_source;reference_epoch<=frame_epoch;
    end
   end
  end
 end
endmodule
