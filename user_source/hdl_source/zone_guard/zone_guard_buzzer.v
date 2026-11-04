// V1 PNP high-side driver: 0=energized, 1=silent. Clock enables only.
// ACTIVE_BUZZER=0 is the passive 2 kHz candidate; verify actual fitted part.
module zone_guard_buzzer #(
 parameter integer CLK_HZ=75000000,ON_MS=200,OFF_MS=800,
 parameter integer TONE_HZ=2000,ACTIVE_BUZZER=0
)(
 input wire I_clk,I_rst_n,I_enable,I_alarm,
 output wire O_buzzer_n
);
 localparam PARAM_OK=CLK_HZ>=1000 && CLK_HZ<=1000000000 && CLK_HZ%1000==0 &&
  ON_MS>=1 && ON_MS<=65535 && OFF_MS>=1 && OFF_MS<=65535 &&
  TONE_HZ>=1 && TONE_HZ<=CLK_HZ/2 && (ACTIVE_BUZZER==0 || ACTIVE_BUZZER==1);
 localparam integer MS_CLOCKS=CLK_HZ>=1000?CLK_HZ/1000:1;
 // Division rounds down if a future clock is not an exact multiple of 2*tone.
 localparam integer HALF_CLOCKS=(TONE_HZ>=1 && TONE_HZ<=CLK_HZ/2)?CLK_HZ/(2*TONE_HZ):1;
 localparam integer PERIOD_MS=(ON_MS>=1 && ON_MS<=65535 && OFF_MS>=1 && OFF_MS<=65535)?ON_MS+OFF_MS:2;
 localparam integer MW=MS_CLOCKS>1?$clog2(MS_CLOCKS):1;
 localparam integer TW=HALF_CLOCKS>1?$clog2(HALF_CLOCKS):1;
 localparam integer PW=PERIOD_MS>1?$clog2(PERIOD_MS):1;
 reg [MW-1:0] ms_div;
 reg [TW-1:0] tone_div;
 reg [PW-1:0] phase_ms;
 reg running,tone_off;
 wire run=PARAM_OK && I_enable && I_alarm;
 wire sound_window=phase_ms<ON_MS;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   running<=0;ms_div<=0;phase_ms<=0;tone_div<=0;tone_off<=0;
  end else if(!run)begin
   running<=0;ms_div<=0;phase_ms<=0;tone_div<=0;tone_off<=0;
  end else if(!running)begin
   // Deterministic phase: the first enabled edge starts a complete ON window.
   running<=1;ms_div<=0;phase_ms<=0;tone_div<=0;tone_off<=0;
  end else begin
   if(ms_div==MS_CLOCKS-1)begin
    ms_div<=0;
    if(phase_ms==PERIOD_MS-1)phase_ms<=0;else phase_ms<=phase_ms+1'b1;
   end else ms_div<=ms_div+1'b1;
   if(!sound_window || ACTIVE_BUZZER!=0)begin tone_div<=0;tone_off<=0;end
   else if(tone_div==HALF_CLOCKS-1)begin tone_div<=0;tone_off<=!tone_off;end
   else tone_div<=tone_div+1'b1;
  end
 end
 // Direct gates silence asynchronously on reset/disarm/alarm-clear. No fault
 // input: controller's retained alarm bit alone decides sound during FAULT.
 assign O_buzzer_n=!(I_rst_n && run && running && sound_window &&
  (ACTIVE_BUZZER!=0 || !tone_off));
endmodule
