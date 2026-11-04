// Independent grayscale threshold, never reuses or writes RGB MIN/DOM.
module shape_threshold_control(
 input wire I_clk,I_rst_n,I_vblank,
 input wire [3:0] I_mode,
 input wire [1:0] I_item,
 input wire I_up,I_down,
 output reg [7:0] O_value,O_frame_value
);
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin O_value<=128;O_frame_value<=128;end
  else begin
   if(I_vblank)O_frame_value<=O_value;
   if(I_mode==12 && I_item==2)begin
    if(I_up)O_value<=O_value>251?8'd255:O_value+8'd4;
    else if(I_down)O_value<=O_value<4?8'd0:O_value-8'd4;
   end
  end
 end
endmodule
