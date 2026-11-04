// V1: complete, associated source-frame results only. No generated clocks.
module zone_guard_control #(
 parameter integer WIDTH=1280,HEIGHT=720,X0=400,X1=880,Y0=220,Y1=500,
 parameter integer ENTER_COUNT=3,CLEAR_COUNT=3,RECOVER_COUNT=3,
 parameter integer CLK_HZ=75000000,TIMEOUT_MS=500
)(
 input wire I_clk,I_rst_n,I_enable,I_reconfigure,I_source_reset,
 input wire I_fault_event,I_result_event,I_target_valid,I_coords_valid,
 input wire [31:0] I_source_id,
 input wire [11:0] I_center_x,I_center_y,
 output reg O_alarm,O_fault,
 output wire [2:0] O_state,
 output wire O_accept,
 output reg O_center_valid,
 output reg [11:0] O_center_x,O_center_y
);
 localparam integer TIMEOUT_CYCLES=(CLK_HZ/1000)*TIMEOUT_MS;
 localparam integer AW=(TIMEOUT_CYCLES<2)?1:$clog2(TIMEOUT_CYCLES);
 localparam integer EW=(ENTER_COUNT<2)?1:$clog2(ENTER_COUNT+1);
 localparam integer CW=(CLEAR_COUNT<2)?1:$clog2(CLEAR_COUNT+1);
 localparam integer RW=(RECOVER_COUNT<2)?1:$clog2(RECOVER_COUNT+1);
 localparam PARAM_OK=WIDTH>=1 && WIDTH<=4096 && HEIGHT>=1 && HEIGHT<=4096 &&
  X0>=0 && X0<X1 && X1<=WIDTH && Y0>=0 && Y0<Y1 && Y1<=HEIGHT &&
  ENTER_COUNT>=1 && ENTER_COUNT<=255 && CLEAR_COUNT>=1 && CLEAR_COUNT<=255 &&
  RECOVER_COUNT>=1 && RECOVER_COUNT<=255 &&
  CLK_HZ>=1000 && CLK_HZ%1000==0 && TIMEOUT_MS>=1 &&
  CLK_HZ/1000<=2147483647/TIMEOUT_MS;
 // OFF=0 WAIT=1 NO_TARGET=2 OUT=3 PENDING=4 ALARM=5 FAULT=6.
 reg [2:0] observation;
 reg enabled_d,have_id;
 reg [31:0] last_id;
 reg [AW-1:0] age;
 reg [EW-1:0] enter_n;
 reg [CW-1:0] clear_n;
 reg [RW-1:0] recover_n;
 wire malformed=I_result_event && I_target_valid &&
  (!I_coords_valid || I_center_x>=WIDTH || I_center_y>=HEIGHT);
 wire bad=I_fault_event || I_source_reset || malformed || !PARAM_OK;
 wire fresh=I_result_event && (!have_id || I_source_id!=last_id);
 wire in_zone=I_target_valid && I_coords_valid && I_center_x>=X0 && I_center_x<X1 &&
  I_center_y>=Y0 && I_center_y<Y1;
 assign O_accept=I_rst_n && I_enable && enabled_d && !I_reconfigure && !bad && fresh;
 assign O_state=!I_enable?3'd0:(O_fault?3'd6:(O_alarm?3'd5:observation));
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   O_alarm<=0;O_fault<=0;observation<=1;enabled_d<=0;have_id<=0;last_id<=0;
   age<=0;enter_n<=0;clear_n<=0;recover_n<=0;
   O_center_valid<=0;O_center_x<=0;O_center_y<=0;
  end else begin
   enabled_d<=I_enable;
   if(!I_enable) begin
    O_alarm<=0;O_fault<=0;observation<=1;have_id<=0;last_id<=0;
    age<=0;enter_n<=0;clear_n<=0;recover_n<=0;O_center_valid<=0;
   end else if(bad) begin
    // An explicit fault outranks simultaneous healthy data/config changes.
    O_fault<=1;enter_n<=0;clear_n<=0;recover_n<=0;O_center_valid<=0;
    if(I_source_reset) begin have_id<=0;last_id<=0;end
    if(age<TIMEOUT_CYCLES-1) age<=age+1'b1;
   end else if(!enabled_d || I_reconfigure) begin
    observation<=1;age<=0;enter_n<=0;clear_n<=0;recover_n<=0;O_center_valid<=0;
    // Do not forget a consumed source simply because thresholds changed.
    // Alarm/fault are retained; only OFF/system reset clears the session.
   end else if(fresh) begin
    have_id<=1;last_id<=I_source_id;age<=0;
    O_center_valid<=I_target_valid && I_coords_valid;
    O_center_x<=I_center_x;O_center_y<=I_center_y;
    if(recover_n<RECOVER_COUNT) recover_n<=recover_n+1'b1;
    if(recover_n>=RECOVER_COUNT-1) O_fault<=0;
    if(in_zone) begin
     clear_n<=0;observation<=4;
     if(enter_n<ENTER_COUNT) enter_n<=enter_n+1'b1;
     if(enter_n>=ENTER_COUNT-1) O_alarm<=1;
    end else begin
     enter_n<=0;observation<=I_target_valid?3'd3:3'd2;
     if(clear_n<CLEAR_COUNT) clear_n<=clear_n+1'b1;
     if(clear_n>=CLEAR_COUNT-1) O_alarm<=0;
    end
   end else if(age>=TIMEOUT_CYCLES-1) begin
    O_fault<=1;enter_n<=0;clear_n<=0;recover_n<=0;O_center_valid<=0;
   end else age<=age+1'b1;
  end
 end
endmodule
