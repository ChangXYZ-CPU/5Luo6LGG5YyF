// P1 policy only: consume complete, healthy, associated motion results.
// No pixel path, clock generation, RAM, pin assignment or detector changes.
module motion_guard_oneshot #(
 parameter integer WIDTH=1280,HEIGHT=720,X0=400,X1=880,Y0=220,Y1=500,
 parameter integer ENTER_COUNT=3,EXIT_COUNT=3,RECOVER_COUNT=3,
 parameter integer CLK_HZ=75000000,TIMEOUT_MS=500,ALARM_MS=2000
)(
 input wire I_clk,I_rst_n,I_enable,I_reconfigure,I_source_reset,
 input wire I_fault_event,I_result_event,I_target_valid,I_coords_valid,
 input wire [31:0] I_source_id,
 input wire [11:0] I_center_x,I_center_y,
 output wire O_alarm,O_accept,O_trigger,
 output reg O_fault,O_locked,O_center_valid,
 output wire [2:0] O_state,
 output reg [11:0] O_center_x,O_center_y
);
 localparam TIME_OK=CLK_HZ>=1000 && CLK_HZ%1000==0 &&
  TIMEOUT_MS>=1 && ALARM_MS>=1 &&
  CLK_HZ/1000<=2147483647/(TIMEOUT_MS>0?TIMEOUT_MS:1) &&
  CLK_HZ/1000<=2147483646/(ALARM_MS>0?ALARM_MS:1);
 // Invalid timing parameters select safe widths; PARAM_OK still fails shut.
 localparam integer TIMEOUT_CYCLES=TIME_OK?(CLK_HZ/1000)*TIMEOUT_MS:1;
 localparam integer ALARM_CYCLES=TIME_OK?(CLK_HZ/1000)*ALARM_MS:1;
 localparam integer AW=TIMEOUT_CYCLES<2?1:$clog2(TIMEOUT_CYCLES);
 localparam integer TW=ALARM_CYCLES<2?1:$clog2(ALARM_CYCLES+1);
 localparam integer EW=ENTER_COUNT<2?1:$clog2(ENTER_COUNT+1);
 localparam integer XW=EXIT_COUNT<2?1:$clog2(EXIT_COUNT+1);
 localparam integer RW=RECOVER_COUNT<2?1:$clog2(RECOVER_COUNT+1);
 localparam PARAM_OK=TIME_OK && WIDTH>=1 && WIDTH<=4096 && HEIGHT>=1 && HEIGHT<=4096 &&
  X0>=0 && X0<X1 && X1<=WIDTH && Y0>=0 && Y0<Y1 && Y1<=HEIGHT &&
  ENTER_COUNT>=1 && ENTER_COUNT<=255 && EXIT_COUNT>=1 && EXIT_COUNT<=255 &&
  RECOVER_COUNT>=1 && RECOVER_COUNT<=255;
 reg enabled_d,have_id;
 reg [31:0] last_id;
 reg [AW-1:0] age;
 reg [TW-1:0] alarm_left;
 reg [EW-1:0] enter_n;
 reg [XW-1:0] exit_n;
 reg [RW-1:0] recover_n;
 reg [2:0] observation;
 wire malformed=I_result_event && I_target_valid &&
  (!I_coords_valid || I_center_x>=WIDTH || I_center_y>=HEIGHT);
 wire bad=I_fault_event || I_source_reset || malformed || !PARAM_OK;
 wire fresh=I_result_event && (!have_id || I_source_id!=last_id);
 wire in_zone=I_target_valid && I_coords_valid && I_center_x>=X0 && I_center_x<X1 &&
  I_center_y>=Y0 && I_center_y<Y1;
 assign O_accept=I_rst_n && I_enable && enabled_d && !I_reconfigure && !bad && fresh;
 // The last healthy recovery sample may itself complete a new IN streak.
 assign O_trigger=O_accept && (!O_fault || recover_n>=RECOVER_COUNT-1) &&
  in_zone && !O_locked && alarm_left==0 && enter_n>=ENTER_COUNT-1;
 assign O_alarm=I_rst_n && I_enable && alarm_left!=0;
 // 7 means HELD/already prompted, not an error. Renderer integration is P2.
 assign O_state=!I_enable?3'd0:(O_fault?3'd6:(O_alarm?3'd5:(O_locked?3'd7:observation)));
 // Independent deadline: bad input/config/source reset cannot stretch sound.
 // Trigger edge loads N; exactly N output cycles follow, including that edge.
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)alarm_left<=0;
  else if(!I_enable)alarm_left<=0;
  else if(O_trigger)alarm_left<=ALARM_CYCLES;
  else if(alarm_left!=0)alarm_left<=alarm_left-1'b1;
 end
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   enabled_d<=0;have_id<=0;last_id<=0;age<=0;
   enter_n<=0;exit_n<=0;recover_n<=0;observation<=1;
   O_fault<=0;O_locked<=0;O_center_valid<=0;O_center_x<=0;O_center_y<=0;
  end else begin
   enabled_d<=I_enable;
   if(!I_enable)begin
    have_id<=0;last_id<=0;age<=0;enter_n<=0;exit_n<=0;recover_n<=0;observation<=1;
    O_fault<=0;O_locked<=0;O_center_valid<=0;O_center_x<=0;O_center_y<=0;
   end else if(bad)begin
    O_fault<=1;enter_n<=0;exit_n<=0;recover_n<=0;O_center_valid<=0;
    // A source session reset permits ID reuse, but never unlocks an entry.
    if(I_source_reset)begin have_id<=0;last_id<=0;end
    if(age<TIMEOUT_CYCLES-1)age<=age+1'b1;
   end else if(!enabled_d || I_reconfigure)begin
    age<=0;enter_n<=0;exit_n<=0;recover_n<=0;observation<=1;O_center_valid<=0;
   end else if(fresh)begin
    have_id<=1;last_id<=I_source_id;age<=0;
    O_center_valid<=I_target_valid && I_coords_valid;
    O_center_x<=I_center_x;O_center_y<=I_center_y;
    if(recover_n<RECOVER_COUNT)recover_n<=recover_n+1'b1;
    if(recover_n>=RECOVER_COUNT-1)O_fault<=0;
    if(in_zone)begin
     exit_n<=0;observation<=4;
     if(!O_locked && alarm_left==0)begin
      if(enter_n<ENTER_COUNT)enter_n<=enter_n+1'b1;
     end else enter_n<=0;
     if(O_trigger)begin O_locked<=1;enter_n<=0;end
    end else if(I_target_valid)begin
     enter_n<=0;observation<=3;
     if(exit_n<EXIT_COUNT)exit_n<=exit_n+1'b1;
     // Never unlock on fewer than RECOVER_COUNT healthy results after fault.
     if(exit_n>=EXIT_COUNT-1 && (!O_fault || recover_n>=RECOVER_COUNT-1))O_locked<=0;
    end else begin
     // A stationary target can disappear from frame difference. Heartbeat
     // and fault recovery are valid, but absence is NOT an exit observation.
     enter_n<=0;exit_n<=0;observation<=2;
    end
   end else if(age>=TIMEOUT_CYCLES-1)begin
    O_fault<=1;enter_n<=0;exit_n<=0;recover_n<=0;O_center_valid<=0;
   end else age<=age+1'b1;
  end
 end
endmodule
