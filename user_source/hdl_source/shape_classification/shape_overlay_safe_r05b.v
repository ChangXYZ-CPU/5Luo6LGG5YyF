// Nine-clock metadata-only companion of the existing ISP. No RGB/control delay.
module shape_overlay_safe_r05b(
 input wire I_clk,I_rst_n,I_enable,I_target,I_flush,
 input wire [2:0] I_status,
 input wire [1:0] I_class,
 input wire [11:0] I_xmin,I_xmax,I_ymin,I_ymax,I_center_x,I_center_y,
 input wire [31:0] I_source,I_epoch,
 output wire [177:0] O_packet,
 output wire [1:0] O_class
);
 reg [177:0] packet[0:8];
 reg [1:0] category[0:8];
 reg [15:0] xbcd[0:8],ybcd[0:8];
 integer i;
 function [15:0] step3;
  input [15:0] prior;
  input [2:0] bits;
  reg [15:0] value;
  integer n,d;
  begin
   value=prior;
   for(n=2;n>=0;n=n-1)begin
    for(d=0;d<4;d=d+1)
     if(value[d*4+:4]>=5)value[d*4+:4]=value[d*4+:4]+4'd3;
    value={value[14:0],bits[n]};
   end
   step3=value;
  end
 endfunction
 wire target=I_enable && I_status==1 && I_target;
 wire [2:0] status=I_status==1?3'd3:I_status==2?3'd2:I_status==3?3'd7:3'd1;
 // Complete raw packet sampled on exactly the same edge as ISP stage zero.
 // Data stays in an unreset pipeline; only initialized tokens carry reset/flush.
 // A token reaches output only after every field has traversed all nine stages.
 // Invalid tokens mask ALL output bits, preserving the exact reset/flush API.
 reg [8:0] initialized;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)initialized<=0;
  else if(I_flush)initialized<=0;
  else initialized<={initialized[7:0],1'b1};
 end
 always @(posedge I_clk)begin
   packet[0]<={target,target?I_center_x:12'd0,target?I_center_y:12'd0,32'd0,
    4'd12,I_source,I_epoch,I_enable,target,I_enable?status:3'd0,
    target?I_xmin:12'd0,target?I_xmax:12'd0,target?I_ymin:12'd0,target?I_ymax:12'd0};
   category[0]<=target?I_class:2'd0;
   xbcd[0]<=step3(16'd0,target?I_center_x[11:9]:3'd0);
   ybcd[0]<=step3(16'd0,target?I_center_y[11:9]:3'd0);
   for(i=1;i<9;i=i+1)begin
    packet[i]<=packet[i-1];category[i]<=category[i-1];
    if(i<4)begin
     xbcd[i]<=step3(xbcd[i-1],packet[i-1][176-i*3-:3]);
     ybcd[i]<=step3(ybcd[i-1],packet[i-1][164-i*3-:3]);
    end else begin xbcd[i]<=xbcd[i-1];ybcd[i]<=ybcd[i-1];end
   end
 end
 // Invalidate immediately on a hard fault; clear all nine buffered packets
 // at the next clock. Old class/box must never return after a one-cycle fault.
 assign O_packet=(I_flush || !initialized[8])?178'd0:{packet[8][177:153],xbcd[8],ybcd[8],packet[8][120:0]};
 assign O_class=(I_flush || !initialized[8])?2'd0:category[8];
endmodule
