// Counters advance at end-of-line. All outputs share two registers.
module uivtc #(
 parameter H_ActiveSize=1280,H_FrameSize=1650,H_SyncStart=1390,H_SyncEnd=1430,
 parameter V_ActiveSize=720,V_FrameSize=750,V_SyncStart=725,V_SyncEnd=730
)(
 input wire I_vtc_rstn,I_vtc_clk,
 output wire O_vtc_vs,O_vtc_hs,O_vtc_de_valid,O_vtc_user,O_vtc_last
);
 reg [11:0] hcnt,vcnt;
 reg [1:0] released;
 reg [4:0] pipe0,pipe1;
 wire active=hcnt<H_ActiveSize && vcnt<V_ActiveSize;
 wire [4:0] timing={
  vcnt>=V_SyncStart && vcnt<V_SyncEnd,
  hcnt>=H_SyncStart && hcnt<H_SyncEnd,
  active,active && hcnt==0 && vcnt==0,
  active && hcnt==H_ActiveSize-1
 };
 always @(posedge I_vtc_clk or negedge I_vtc_rstn) begin
  if(!I_vtc_rstn) begin
   hcnt<=0;vcnt<=0;released<=0;pipe0<=0;pipe1<=0;
  end else begin
   released<={released[0],1'b1};
   if(released[1]) begin
    if(hcnt==H_FrameSize-1) begin
     hcnt<=0;
     vcnt<=(vcnt==V_FrameSize-1)?12'd0:vcnt+1'b1;
    end else hcnt<=hcnt+1'b1;
    pipe0<=timing;
   end else pipe0<=0;
   pipe1<=pipe0;
  end
 end
 assign {O_vtc_vs,O_vtc_hs,O_vtc_de_valid,O_vtc_user,O_vtc_last}=pipe1;
endmodule
