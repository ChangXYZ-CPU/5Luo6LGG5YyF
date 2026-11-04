// R02 bright-paper observer: 1280x720 -> 160x90, 8x8 votes >=48.
// This is NOT the black-ink mask and must never be ANDed pixelwise with it.
// O_write is a registered, row-major bit stream. O_eof includes the final bit.
module shape_paper_capture(
 input wire I_clk,I_rst_n,I_enable,I_cancel,
 input wire I_de,I_sof,I_eol,I_pixel_good,
 input wire [11:0] I_x,I_y,
 input wire [23:0] I_rgb,
 input wire [7:0] I_threshold,
 output reg O_write,O_bit,O_sof,O_eof,
 output reg [13:0] O_addr,
 output wire O_bad_event,
 output reg O_active
);
 wire accept,sof,eof,bad,frame_bad,config_bad;
 red_frame_guard guard(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable&&!I_cancel),
  .I_de(I_de),.I_user(I_sof),.I_last(I_eol),.I_pixel_good(I_pixel_good),.I_x(I_x),.I_y(I_y),
  .O_accept(accept),.O_frame_start(sof),.O_frame_end(eof),.O_bad_event(bad),
  .O_frame_bad(frame_bad),.O_config_bad(config_bad));
 reg [5:0] column[0:255];
 reg [5:0] column_read,vertical;
 reg [3:0] horizontal;
 reg d_valid,d_black,d_sof,d_eof;
 reg [10:0] dx;
 reg [9:0] dy;
 reg [7:0] threshold;
 reg [13:0] address;
 wire [9:0] gray_sum={2'd0,I_rgb[23:16]}+{1'd0,I_rgb[15:8],1'b0}+{2'd0,I_rgb[7:0]}+10'd2;
 wire [7:0] effective_threshold=sof?I_threshold:threshold;
 wire [6:0] total={1'b0,vertical}+{3'd0,horizontal}+{6'd0,d_black};
 assign O_bad_event=I_enable && (bad || I_cancel);
 // RAM intentionally has no reset. Row zero overwrites every used column
 // before subsequent rows can read it; an aborted frame is never published.
 always @(posedge I_clk) begin
  if(accept) column_read<=column[I_x[10:3]];
  if(I_rst_n && I_enable && !I_cancel && !bad && d_valid && dx[2:0]==7 && dy[2:0]!=7)
   column[dx[10:3]]<=total[5:0];
 end
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   d_valid<=0;d_black<=0;d_sof<=0;d_eof<=0;dx<=0;dy<=0;
   threshold<=160;horizontal<=0;vertical<=0;address<=0;
   O_write<=0;O_bit<=0;O_addr<=0;O_sof<=0;O_eof<=0;O_active<=0;
  end else if(!I_enable || I_cancel || bad) begin
   d_valid<=0;d_sof<=0;d_eof<=0;O_write<=0;O_sof<=0;O_eof<=0;
   horizontal<=0;vertical<=0;address<=0;O_active<=0;
  end else begin
   O_write<=0;O_sof<=0;O_eof<=0;
   d_valid<=accept;d_sof<=sof&&accept;d_eof<=eof;
   if(accept) begin
    dx<=I_x[10:0];dy<=I_y[9:0];d_black<=gray_sum[9:2]>=effective_threshold;
    if(sof)begin threshold<=I_threshold;O_active<=1;end
   end
   if(d_valid) begin
    if(d_sof) begin address<=0;O_sof<=1;end
    if(dx[2:0]==0) begin
     horizontal<={3'd0,d_black};vertical<=(dy[2:0]==0)?6'd0:column_read;
    end else if(dx[2:0]!=7) horizontal<=horizontal+{3'd0,d_black};
    else if(dy[2:0]==7) begin
     O_write<=1;O_bit<=total>=48;O_addr<=address;address<=address+1'b1;
    end
    if(d_eof) begin O_eof<=1;O_active<=0;end
   end
  end
 end
endmodule
