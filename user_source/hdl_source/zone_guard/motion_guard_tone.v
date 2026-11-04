// Continuous sound while the policy's finite two-second window is active.
// Board PNP drive is active-low; SW5 and passive/active assembly need checking.
module motion_guard_tone #(
 parameter integer CLK_HZ=75000000,TONE_HZ=2000,ACTIVE_BUZZER=0
)(
 input wire I_clk,I_rst_n,I_enable,I_alarm,
 output wire O_buzzer_n
);
 localparam OK=CLK_HZ>=1 && TONE_HZ>=1 && TONE_HZ<=CLK_HZ/2 &&
  (ACTIVE_BUZZER==0 || ACTIVE_BUZZER==1);
 localparam integer HALF=OK?CLK_HZ/(2*TONE_HZ):1;
 localparam integer CW=HALF<2?1:$clog2(HALF);
 reg [CW-1:0] count;
 reg phase;
 wire run=OK && I_rst_n && I_enable && I_alarm;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin count<=0;phase<=0;end
  else if(!run)begin count<=0;phase<=0;end
  else if(count==HALF-1)begin count<=0;phase<=!phase;end
  else count<=count+1'b1;
 end
 assign O_buzzer_n=!(run && (ACTIVE_BUZZER==1 || !phase));
endmodule
