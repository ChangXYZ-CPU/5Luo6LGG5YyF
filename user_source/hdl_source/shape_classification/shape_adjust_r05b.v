// Shape-only controls. IDs 0..7 retain the captain's current meanings.
// 8 GRAY, 9 PAPER, 10 FIT, 11 AMIN, 12 AMAX. No persistent VIEW item.
// Area gear 0..15 means 16 << gear full-resolution pixel equivalents;
// gear 16 means the complete 921600-pixel frame. Separate R/G/B/shape banks.
// Configuration is latched in blanking; no RGB MIN/DOM register is written.
module shape_adjust_r05b(
 input wire I_clk,I_rst_n,I_vblank,
 input wire [3:0] I_mode,I_item,
 input wire I_up,I_down,
 output reg [7:0] O_gray,O_paper,O_gray_frame,O_paper_frame,
 output reg [1:0] O_profile,O_view,O_profile_frame,O_view_frame,
 output wire [4:0] O_area_min,O_area_max,
 output reg [4:0] O_area_min_frame,O_area_max_frame
);
 reg [4:0] area_min[0:3],area_max[0:3];
 wire [1:0] bank=I_mode==12?2'd3:I_mode==10?2'd1:I_mode==8?2'd2:2'd0;
 wire area_mode=I_mode==12||I_mode==6||I_mode==8||I_mode==10;
 assign O_area_min=area_min[bank];
 assign O_area_max=area_max[bank];
 integer i;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   O_gray<=128;O_paper<=160;O_profile<=1;O_view<=0;
   O_gray_frame<=128;O_paper_frame<=160;O_profile_frame<=1;O_view_frame<=0;
   O_area_min_frame<=2;O_area_max_frame<=16;
   for(i=0;i<4;i=i+1)begin area_min[i]<=i==3?5'd6:5'd2;area_max[i]<=16;end
  end else begin
   // Live indication only; the video preview consumes the blanking-latched copy.
   O_view<=I_mode!=12?2'd0:I_item==8?2'd1:I_item==9?2'd2:2'd0;
   if(I_vblank)begin
    O_gray_frame<=O_gray;O_paper_frame<=O_paper;
    O_profile_frame<=O_profile;
    O_view_frame<=I_mode!=12?2'd0:I_item==8?2'd1:I_item==9?2'd2:2'd0;
    O_area_min_frame<=O_area_min;O_area_max_frame<=O_area_max;
   end
   if(area_mode && I_item==11)begin
    if(I_up && area_min[bank]<area_max[bank])area_min[bank]<=area_min[bank]+1'b1;
    else if(I_down && area_min[bank]!=0)area_min[bank]<=area_min[bank]-1'b1;
   end
   if(I_mode==12 && I_item==12)begin
    if(I_up && area_max[bank]<16)area_max[bank]<=area_max[bank]+1'b1;
    else if(I_down && area_max[bank]>area_min[bank])area_max[bank]<=area_max[bank]-1'b1;
   end
   if(I_mode==12)case(I_item)
    8:if(I_up)O_gray<=O_gray>251?8'd255:O_gray+8'd4;
      else if(I_down)O_gray<=O_gray<4?8'd0:O_gray-8'd4;
    9:if(I_up)O_paper<=O_paper>251?8'd255:O_paper+8'd4;
      else if(I_down)O_paper<=O_paper<4?8'd0:O_paper-8'd4;
    10:if(I_up&&O_profile<2)O_profile<=O_profile+1'b1;
       else if(I_down&&O_profile>0)O_profile<=O_profile-1'b1;
    default:begin end
   endcase
  end
 end
endmodule
