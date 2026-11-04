// V1 sideband geometry only. No RGB retiming, RAM, generated clock or new UI.
module zone_guard_osd #(
 parameter integer WIDTH=1280,HEIGHT=720,X0=400,X1=880,Y0=220,Y1=500,
 parameter integer TEXT_X=16,TEXT_Y=232,DIAGNOSTICS=0,TEXT_EN=1
)(
 input wire I_clk,I_rst_n,I_enable,I_osd_en,I_de,I_sof,
 input wire [11:0] I_x,I_y,
 input wire [2:0] I_state,
 input wire I_alarm,I_center_valid,
 input wire [11:0] I_center_x,I_center_y,
 input wire [3:0] I_reason,
 output wire O_hit,
 output wire [2:0] O_color
);
 localparam PARAM_OK=WIDTH>=1 && WIDTH<=4096 && HEIGHT>=1 && HEIGHT<=4096 &&
  X0>=0 && X0<X1 && X1<=WIDTH && Y0>=0 && Y0<Y1 && Y1<=HEIGHT &&
  TEXT_X>=0 && TEXT_X<=4095 && TEXT_Y>=0 && TEXT_Y<=4095;
 reg have_frame,alarm_q,center_q;
 reg [2:0] state_q;
 reg [3:0] reason_q;
 reg [11:0] cx_q,cy_q;
 wire first=I_de && I_sof;
 // Same-edge bypass: first pixel and the rest of this frame share one packet.
 wire [2:0] state=first?I_state:state_q;
 wire [3:0] reason=first?I_reason:reason_q;
 wire [7:0] reason_char=reason<=8?(8'd48+{4'd0,reason}):8'd63;
 wire alarm=first?I_alarm:alarm_q;
 wire center_valid=first?I_center_valid:center_q;
 wire [11:0] cx=first?I_center_x:cx_q;
 wire [11:0] cy=first?I_center_y:cy_q;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin have_frame<=0;state_q<=1;reason_q<=0;alarm_q<=0;center_q<=0;cx_q<=0;cy_q<=0;end
  else if(!I_enable)begin have_frame<=0;state_q<=0;reason_q<=0;alarm_q<=0;center_q<=0;end
  else if(first)begin
   have_frame<=1;state_q<=I_state;reason_q<=I_reason;alarm_q<=I_alarm;
   center_q<=I_center_valid;cx_q<=I_center_x;cy_q<=I_center_y;
  end
 end
 wire visible=PARAM_OK && I_rst_n && I_enable && I_osd_en && I_de &&
  (have_frame || first) && I_x<WIDTH && I_y<HEIGHT;
 wire region=I_x>=X0 && I_x<X1 && I_y>=Y0 && I_y<Y1;
 // Two inward pixels; the last included coordinates remain X1-1/Y1-1.
 wire border=region && (I_x<X0+2 || I_x>=X1-2 || I_y<Y0+2 || I_y>=Y1-2);
 // One extended subtraction per axis. The representable difference is
 // -4095..4095, so [0,4] or [-4,-1] is exact even at coordinate 0/4095.
 wire [12:0] dx={1'b0,I_x}-{1'b0,cx};
 wire [12:0] dy={1'b0,I_y}-{1'b0,cy};
 wire near_x=dx<=13'd4 || dx>=13'd8188;
 wire near_y=dy<=13'd4 || dy>=13'd8188;
 wire cross_hit=center_valid && cx<WIDTH && cy<HEIGHT &&
  ((I_y==cy && near_x) || (I_x==cx && near_y));
 wire [11:0] tx=I_x-TEXT_X,ty=I_y-TEXT_Y;
 reg [95:0] message;
 always @* begin
  case(state)
   0:message="OFF         ";
   1:message="WAIT        ";
   2:message="NO TARGET   ";
   3:message="OUT         ";
   4:message="PENDING     ";
   5:message="ALARM       ";
   7:message=DIAGNOSTICS==1?"HELD        ":(alarm?"FAULT ALARM ":"FAULT       ");
   default:message=alarm?"FAULT ALARM ":"FAULT       ";
  endcase
  if(DIAGNOSTICS==1 && state==6)
   message={"FAULT ",reason_char,alarm?" ALRM":"     "};
 end
 wire [7:0] ch=message[95-tx[7:4]*8-:8];
 function [34:0] font;
  input [7:0] c;
  begin case(c)
   "A":font={5'b01110,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
   "D":font={5'b11110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b11110};
   "E":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b11111};
   "F":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b10000,5'b10000,5'b10000};
   "G":font={5'b01110,5'b10001,5'b10000,5'b10111,5'b10001,5'b10001,5'b01110};
   "H":font={5'b10001,5'b10001,5'b10001,5'b11111,5'b10001,5'b10001,5'b10001};
   "0":font={5'b01110,5'b10001,5'b10011,5'b10101,5'b11001,5'b10001,5'b01110};
   "1":font={5'b00100,5'b01100,5'b00100,5'b00100,5'b00100,5'b00100,5'b01110};
   "2":font={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b01000,5'b11111};
   "3":font={5'b11110,5'b00001,5'b00001,5'b01110,5'b00001,5'b00001,5'b11110};
   "4":font={5'b00010,5'b00110,5'b01010,5'b10010,5'b11111,5'b00010,5'b00010};
   "5":font={5'b11111,5'b10000,5'b10000,5'b11110,5'b00001,5'b00001,5'b11110};
   "6":font={5'b01110,5'b10000,5'b10000,5'b11110,5'b10001,5'b10001,5'b01110};
   "7":font={5'b11111,5'b00001,5'b00010,5'b00100,5'b01000,5'b01000,5'b01000};
   "8":font={5'b01110,5'b10001,5'b10001,5'b01110,5'b10001,5'b10001,5'b01110};
   "?":font={5'b01110,5'b10001,5'b00001,5'b00010,5'b00100,5'b00000,5'b00100};
   "I":font={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b11111};
   "L":font={5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b10000,5'b11111};
   "M":font={5'b10001,5'b11011,5'b10101,5'b10101,5'b10001,5'b10001,5'b10001};
   "N":font={5'b10001,5'b11001,5'b10101,5'b10011,5'b10001,5'b10001,5'b10001};
   "O":font={5'b01110,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
   "P":font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10000,5'b10000,5'b10000};
   "R":font={5'b11110,5'b10001,5'b10001,5'b11110,5'b10100,5'b10010,5'b10001};
   "T":font={5'b11111,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100,5'b00100};
   "U":font={5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b10001,5'b01110};
   "W":font={5'b10001,5'b10001,5'b10001,5'b10101,5'b10101,5'b11011,5'b10001};
   default:font=35'd0;
  endcase end
 endfunction
 wire [34:0] bits=font(ch);
 // 2026-11-01: TEXT_EN=0 时只保留边框与十字，文字改由 osd_pipeline 用主字库(12x24)渲染，
 // 位置/文案/配色保持不变，避免两套字库并存。
 wire text_hit=(TEXT_EN!=0) && I_x>=TEXT_X && I_x<TEXT_X+192 &&
  I_y>=TEXT_Y && I_y<TEXT_Y+14 &&
  tx[3:0]<10 && bits[34-ty[3:1]*5-tx[3:1]];
 // Palette: cyan normal, yellow WAIT/PENDING, red ALARM, magenta FAULT, white centre.
 wire [2:0] status_color=state==6?3'd3:(alarm?3'd2:((state==1 || state==4)?3'd1:3'd0));
 assign O_hit=visible && (text_hit || cross_hit || border);
 assign O_color=text_hit?status_color:(cross_hit?3'd4:status_color);
endmodule
