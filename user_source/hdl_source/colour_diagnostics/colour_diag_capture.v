// Observer only. PRE is reader RGB before LSC, NOT sensor RAW.
// PRE and POST must be supplied with the SAME pixel/quality/geometry packet.
// No video ready/backpressure; output congestion skips whole source frames.
module colour_diag_capture #(
 parameter integer WIDTH=1280,HEIGHT=720,PERIOD_FRAMES=16,
 parameter integer CAPTURE_TIMEOUT=2000000,RESULT_TIMEOUT=2000000
)(
 input wire I_clk,I_rst_n,I_enable,I_cancel,
 input wire I_de,I_user,I_last,I_pixel_good,I_source_valid,
 input wire [11:0] I_x,I_y,
 input wire [23:0] I_pre_rgb,I_post_rgb,
 input wire [31:0] I_source_id,I_epoch,I_capture_seq,
 input wire [3:0] I_mode,
 input wire [63:0] I_context,
 input wire I_result_valid,
 input wire [31:0] I_result_source_id,I_result_epoch,I_result_capture_seq,
 input wire [3:0] I_result_mode,
 input wire [2:0] I_result_status,I_result_error,
 input wire [7:0] I_max_runs,
 input wire [9:0] I_labels,
 input wire [15:0] I_max_row_cycles,I_fold_cycles,I_tail_cycles,
 output wire O_valid,output wire [7:0] O_data,output wire O_last,
 input wire I_ready,
 output wire O_busy,O_capturing,O_config_bad,
 output reg O_packet_done
);
 localparam PARAM_OK=WIDTH>=12&&WIDTH<=1280&&HEIGHT>=12&&HEIGHT<=720&&
  PERIOD_FRAMES>=1&&PERIOD_FRAMES<=65535&&CAPTURE_TIMEOUT>=1&&RESULT_TIMEOUT>=1;
 localparam CW=CAPTURE_TIMEOUT<2?1:$clog2(CAPTURE_TIMEOUT+1);
 localparam RW=RESULT_TIMEOUT<2?1:$clog2(RESULT_TIMEOUT+1);
 localparam PW=PERIOD_FRAMES<2?1:$clog2(PERIOD_FRAMES+1);
 localparam IDLE=0,CAPTURE=1,WAIT_RESULT=2,HEADER=3,MEM_REQ=4,MEM_WAIT=5,
  PIXELS=6,CRC_HI=7,CRC_LO=8;
 reg [3:0] state;
 reg [PW-1:0] cooldown;
 reg [CW-1:0] capture_ticks;
 reg [RW-1:0] result_ticks;
 reg [31:0] sequence_count,frame_sequence,frame_source,frame_epoch,frame_capture_seq;
 reg [3:0] frame_mode;
 reg [63:0] frame_context;
 reg [7:0] flags,captured,payload_count;
 reg result_seen;
 reg [7:0] result_status,result_error,max_runs;
 reg [9:0] labels;
 reg [15:0] row_cycles,fold_cycles,tail_cycles;
 wire start=PARAM_OK&&I_enable&&!I_cancel&&state==IDLE&&cooldown==0&&I_de&&I_user;
 wire accept,frame_start,frame_end,bad_event,frame_bad,guard_config_bad;
 red_frame_guard #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) guard(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_enable(I_enable&&!I_cancel&&(state==CAPTURE||start)),
  .I_de(I_de),.I_user(I_user),.I_last(I_last),
  .I_pixel_good(I_pixel_good&&I_source_valid),.I_x(I_x),.I_y(I_y),
  .O_accept(accept),.O_frame_start(frame_start),.O_frame_end(frame_end),
  .O_bad_event(bad_event),.O_frame_bad(frame_bad),.O_config_bad(guard_config_bad));
 assign O_config_bad=!PARAM_OK;
 assign O_busy=state!=IDLE;
 assign O_capturing=state==CAPTURE;
 wire identity_bad=state==CAPTURE&&I_de&&
  (I_source_id!=frame_source||I_epoch!=frame_epoch||I_mode!=frame_mode||I_capture_seq!=frame_capture_seq||I_context!=frame_context);
 wire [7:0] pixel_flags=(bad_event?8'h05:8'h00)|
  ((I_de&&(!I_pixel_good||!I_source_valid))?8'h03:8'h00)|
  (identity_bad?8'h41:8'h00);
 wire detect_mode=frame_mode==6||frame_mode==8||frame_mode==10;
 wire matching_result=I_result_valid&&I_result_status>=2&&
  I_result_source_id==frame_source&&I_result_epoch==frame_epoch&&I_result_mode==frame_mode&&I_result_capture_seq==frame_capture_seq;
 // Nine non-overlapping 4x4 patches: corners, edge centres, image centre.
 reg hit_x,hit_y;reg [1:0] col,row,local_x,local_y;
 reg [3:0] region_index;
 always @* begin
  hit_x=1;hit_y=1;col=0;row=0;local_x=0;local_y=0;
  if(I_x<4)begin col=0;local_x=I_x[1:0];end
  else if(I_x>=WIDTH/2-2&&I_x<WIDTH/2+2)begin col=1;local_x=I_x-(WIDTH/2-2);end
  else if(I_x>=WIDTH-4&&I_x<WIDTH)begin col=2;local_x=I_x-(WIDTH-4);end
  else hit_x=0;
  if(I_y<4)begin row=0;local_y=I_y[1:0];end
  else if(I_y>=HEIGHT/2-2&&I_y<HEIGHT/2+2)begin row=1;local_y=I_y-(HEIGHT/2-2);end
  else if(I_y>=HEIGHT-4&&I_y<HEIGHT)begin row=2;local_y=I_y-(HEIGHT-4);end
  else hit_y=0;
  region_index={2'd0,row}+({2'd0,row}<<1)+{2'd0,col};
 end
 wire sample=accept&&hit_x&&hit_y&&(state==CAPTURE||start);
 wire [7:0] write_address={region_index,local_y,local_x};
 (* ram_style="bram" *) reg [47:0] samples[0:143];
 reg [7:0] read_address;
 reg [47:0] sample_word;
 // Reset-free RAM; incomplete frames send ZERO sample records, never stale RAM.
 always @(posedge I_clk)begin
  if(sample)samples[write_address]<=
   {I_post_rgb[7:0],I_post_rgb[15:8],I_post_rgb[23:16],I_pre_rgb[7:0],I_pre_rgb[15:8],I_pre_rgb[23:16]};
  if(state==MEM_REQ)sample_word<=samples[read_address];
 end
 wire [8:0] next_captured={1'b0,captured}+(sample?9'd1:9'd0);
 wire capture_expired=capture_ticks>=CAPTURE_TIMEOUT-1&&!frame_end;
 reg [5:0] header_index;
 reg [2:0] pixel_byte;
 reg [15:0] crc;
 function [15:0] crc_next;
  input [15:0] old_crc;input [7:0] data;
  reg [15:0] c;integer i;
  begin c=old_crc^{data,8'd0};for(i=0;i<8;i=i+1)c=c[15]?(c<<1)^16'h1021:(c<<1);crc_next=c;end
 endfunction
 function [7:0] header_byte;
  input [5:0] n;
  begin case(n)
   0:header_byte=8'h43;1:header_byte=8'h44;2:header_byte=8'h47;3:header_byte=8'h32;
   4:header_byte=2;5:header_byte=flags;6:header_byte={4'd0,frame_mode};7:header_byte=payload_count;
   8:header_byte=frame_sequence[7:0];9:header_byte=frame_sequence[15:8];10:header_byte=frame_sequence[23:16];11:header_byte=frame_sequence[31:24];
   12:header_byte=frame_source[7:0];13:header_byte=frame_source[15:8];14:header_byte=frame_source[23:16];15:header_byte=frame_source[31:24];
   16:header_byte=frame_epoch[7:0];17:header_byte=frame_epoch[15:8];18:header_byte=frame_epoch[23:16];19:header_byte=frame_epoch[31:24];
   20:header_byte=frame_context[7:0];21:header_byte=frame_context[15:8];22:header_byte=frame_context[23:16];23:header_byte=frame_context[31:24];
   24:header_byte=frame_context[39:32];25:header_byte=frame_context[47:40];26:header_byte=frame_context[55:48];27:header_byte=frame_context[63:56];
   28:header_byte=result_status;29:header_byte=result_error;30:header_byte=max_runs;
   31:header_byte=labels[7:0];32:header_byte={6'd0,labels[9:8]};
   33:header_byte=row_cycles[7:0];34:header_byte=row_cycles[15:8];
   35:header_byte=fold_cycles[7:0];36:header_byte=fold_cycles[15:8];
   37:header_byte=tail_cycles[7:0];38:header_byte=tail_cycles[15:8];
   39:header_byte=captured;40:header_byte=0;
   41:header_byte=WIDTH%256;42:header_byte=WIDTH/256;43:header_byte=HEIGHT%256;44:header_byte=HEIGHT/256;
   45:header_byte=1;
   48:header_byte=frame_capture_seq[7:0];49:header_byte=frame_capture_seq[15:8];
   50:header_byte=frame_capture_seq[23:16];51:header_byte=frame_capture_seq[31:24];
   default:header_byte=0;
  endcase end
 endfunction
 assign O_valid=I_rst_n&&I_enable&&!I_cancel&&(state==HEADER||state==PIXELS||state==CRC_HI||state==CRC_LO);
 assign O_last=state==CRC_LO;
 assign O_data=state==HEADER?header_byte(header_index):state==CRC_HI?crc[15:8]:
  state==CRC_LO?crc[7:0]:(pixel_byte==0?sample_word[7:0]:pixel_byte==1?sample_word[15:8]:
  pixel_byte==2?sample_word[23:16]:pixel_byte==3?sample_word[31:24]:pixel_byte==4?sample_word[39:32]:sample_word[47:40]);
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   state<=IDLE;cooldown<=0;capture_ticks<=0;result_ticks<=0;
   sequence_count<=0;frame_sequence<=0;frame_source<=0;frame_epoch<=0;frame_capture_seq<=0;frame_mode<=0;frame_context<=0;
   flags<=0;captured<=0;payload_count<=0;result_seen<=0;
   result_status<=8'hff;result_error<=8'hff;max_runs<=0;labels<=0;row_cycles<=0;fold_cycles<=0;tail_cycles<=0;
   read_address<=0;header_index<=0;pixel_byte<=0;crc<=16'hffff;O_packet_done<=0;
  end else begin
   O_packet_done<=0;
   if(I_cancel||!I_enable)begin state<=IDLE;cooldown<=0;end
   else begin
    if(I_de&&I_user&&cooldown!=0)cooldown<=cooldown-1'b1;
    if((state==CAPTURE||state==WAIT_RESULT)&&matching_result&&!result_seen)begin
     result_seen<=1;result_status<={5'd0,I_result_status};result_error<={5'd0,I_result_error};
     max_runs<=I_max_runs;labels<=I_labels;row_cycles<=I_max_row_cycles;fold_cycles<=I_fold_cycles;tail_cycles<=I_tail_cycles;
    end
    case(state)
     IDLE:if(start)begin
      state<=CAPTURE;cooldown<=PERIOD_FRAMES-1;capture_ticks<=0;result_ticks<=0;
      sequence_count<=sequence_count+1'b1;frame_sequence<=sequence_count+1'b1;
      frame_source<=I_source_id;frame_epoch<=I_epoch;frame_capture_seq<=I_capture_seq;frame_mode<=I_mode;frame_context<=I_context;
      flags<=pixel_flags;captured<=sample?1:0;payload_count<=0;result_seen<=0;
      result_status<=8'hff;result_error<=8'hff;max_runs<=0;labels<=0;row_cycles<=0;fold_cycles<=0;tail_cycles<=0;
      if(bad_event)state<=(I_mode==6||I_mode==8||I_mode==10)?WAIT_RESULT:HEADER;
      header_index<=0;crc<=16'hffff;
     end
     CAPTURE:begin
      capture_ticks<=capture_ticks+1'b1;
      if(sample)captured<=next_captured[7:0];
      flags<=flags|pixel_flags;
      if(bad_event||identity_bad||capture_expired||frame_end)begin
       state<=detect_mode?WAIT_RESULT:HEADER;result_ticks<=0;header_index<=0;crc<=16'hffff;
       if(capture_expired)flags<=flags|pixel_flags|8'h09;
       else if(frame_end&&next_captured!=144)flags<=flags|pixel_flags|8'h11;
       if(frame_end&&next_captured==144&&(flags|pixel_flags)==0)payload_count<=144;
       else payload_count<=0;
       if(!detect_mode)begin result_status<=0;result_error<=0;end
      end
     end
     WAIT_RESULT:begin
      result_ticks<=result_ticks+1'b1;
      if(result_seen||matching_result)begin state<=HEADER;header_index<=0;crc<=16'hffff;end
      else if(result_ticks>=RESULT_TIMEOUT-1)begin flags<=flags|8'h20;state<=HEADER;header_index<=0;crc<=16'hffff;end
     end
     HEADER:if(I_ready)begin
      crc<=crc_next(crc,O_data);
      if(header_index==51)begin read_address<=0;pixel_byte<=0;state<=payload_count==0?CRC_HI:MEM_REQ;end
      else header_index<=header_index+1'b1;
     end
     MEM_REQ:state<=MEM_WAIT;
     MEM_WAIT:begin state<=PIXELS;pixel_byte<=0;end
     PIXELS:if(I_ready)begin
      crc<=crc_next(crc,O_data);
      if(pixel_byte==5)begin
       if(read_address==143)state<=CRC_HI;
       else begin read_address<=read_address+1'b1;state<=MEM_REQ;end
      end else pixel_byte<=pixel_byte+1'b1;
     end
     CRC_HI:if(I_ready)state<=CRC_LO;
     CRC_LO:if(I_ready)begin state<=IDLE;O_packet_done<=1;end
     default:state<=IDLE;
    endcase
   end
  end
 end
endmodule

// Same-clock ready/valid -> existing, real 8N1 UART. No fake UART dependency.
module colour_diag_uart_tx #(parameter integer CLK_HZ=75000000,BAUD=115200)(
 input wire I_clk,I_rst_n,I_valid,input wire[7:0] I_data,
 output wire O_ready,O_uart_tx
);
 wire busy;
 assign O_ready=!busy;
 uart_tx_115200 #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) tx(
  .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(I_valid&&!busy),.I_data(I_data),.O_busy(busy),.O_uart_tx(O_uart_tx));
endmodule
