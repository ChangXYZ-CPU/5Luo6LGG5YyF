// Bounded prefetch; drain all old DDR responses before a frame FIFO reset.
module video_out #(parameter FRAME_WORDS=172800, parameter BURST_WORDS=240, parameter PAYLOAD_BUFFER_STAGES=0)(
 input wire I_rst_n,I_ddr_clk,I_video_in_wr_busy,
 input wire [1:0] I_video_out_rp,
 input wire I_frame_available,
 input wire [31:0] I_source_frame_id,
 input wire I_source_rst_n,
 output wire O_video_out_rd_busy,
 output reg [1:0] O_display_slot,
 output wire O_ddr_user_rd_en,
 output reg [24:0] O_ddr_user_addr,
 input wire I_ddr_user_ready,I_ddr_user_rd_valid,
 input wire [127:0] I_ddr_user_rd_data,
 input wire I_dsi_clk,I_video_vsync,I_video_rd_en,
 output reg [23:0] O_vdieo_data,
 output reg O_fifo_underflow,
 // B09: live quality of the SAME registered RGB, not sticky fault history.
 output reg O_pixel_good,
 output wire [31:0] O_source_frame_id,
 output wire O_source_valid,
 // L03: 测量描述符（发布侧 -> 像素域），与 O_source_frame_id 同帧对齐
 input wire [31:0] I_lat_start_tick,
 input wire [15:0] I_lat_session,
 input wire        I_lat_valid,
 output wire [31:0] O_lat_start_tick,
 output wire [15:0] O_lat_session,
 output wire        O_lat_valid
);
 function [24:0] base;
  input [1:0] slot;
  begin case(slot)
   0:base=0; 1:base=25'd7000000; 2:base=25'd14000000; 3:base=25'd21000000;
  endcase end
 endfunction
 reg [2:0] vs_sync;
 wire new_frame=vs_sync[1] && !vs_sync[2];
 always @(posedge I_ddr_clk or negedge I_rst_n)
  if(!I_rst_n) vs_sync<=0; else vs_sync<={vs_sync[1:0],I_video_vsync};
 reg burst_active,drain,enabled;
 reg [8:0] burst_count,outstanding;
 reg [17:0] requested;
 reg [4:0] reset_count;
 wire fifo_reset=!I_rst_n || reset_count!=0;
 wire [8:0] fifo_used;
 wire [9:0] occupied={1'b0,fifo_used}+{1'b0,outstanding};
 assign O_ddr_user_rd_en=burst_active && !I_video_in_wr_busy && !new_frame && !drain &&
                          !fifo_reset && I_ddr_user_ready;
 assign O_video_out_rd_busy=burst_active;
 always @(posedge I_ddr_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   burst_active<=0; burst_count<=0; outstanding<=0; requested<=0;
   drain<=1; enabled<=0; reset_count<=0;
   O_display_slot<=3; O_ddr_user_addr<=0;
  end else begin
   case({O_ddr_user_rd_en,I_ddr_user_rd_valid})
    2'b10:outstanding<=outstanding+1'b1;
    2'b01:if(outstanding!=0) outstanding<=outstanding-1'b1;
    default: ;
   endcase
   if(reset_count!=0) reset_count<=reset_count-1'b1;
   if(new_frame) begin
    burst_active<=0; burst_count<=0; drain<=1; enabled<=0;
   end else if(drain && outstanding==0) begin
    drain<=0; reset_count<=5'd16; requested<=0;
    enabled<=I_frame_available;
    if(I_frame_available) begin
     O_display_slot<=I_video_out_rp;
     O_ddr_user_addr<=base(I_video_out_rp);
    end
   end else if(!drain && !fifo_reset && enabled) begin
    if(!burst_active && !I_video_in_wr_busy &&
       occupied <= 510-BURST_WORDS && requested < FRAME_WORDS) begin
     burst_active<=1; burst_count<=0;
    end else if(O_ddr_user_rd_en) begin
     O_ddr_user_addr<=O_ddr_user_addr+25'd8;
     requested<=requested+1'b1;
     if(burst_count==BURST_WORDS-1 || requested==FRAME_WORDS-1) begin
      burst_active<=0; burst_count<=0;
     end else burst_count<=burst_count+1'b1;
    end
   end
  end
 end
 wire fifo_empty;
 wire [127:0] fifo_data;
 reg [3:0] pixel_phase,phase_d;
 reg de_d,vs_d,frame_bad;
 reg pixel_good_d;
 reg [1:0] frame_ready_sync;
 reg [127:0] old_word;
 wire fetch=I_video_rd_en && (pixel_phase==0 || pixel_phase==5 || pixel_phase==10);
 wire fifo_read=fetch && !fifo_empty && !frame_bad && !fifo_reset;
 wire [127:0] fifo_payload;
 fifo_payload_hold_buffer #(.WIDTH(128),.STAGES(PAYLOAD_BUFFER_STAGES),.BUFFER_MASK(128'h10340090142800D0000007B0800AB808)) pixel_payload_hold(
  .I_data(I_ddr_user_rd_data),.O_data(fifo_payload));
 w128_d512_fifo u_pixel_fifo(
  .rst(fifo_reset),.clkw(I_ddr_clk),.clkr(I_dsi_clk),
  .we(I_ddr_user_rd_valid && !drain && !fifo_reset),.di(fifo_payload),
  .wrusedw(fifo_used),.full_flag(),.afull(),
  .re(fifo_read),.dout(fifo_data),.rdusedw(),.valid(),
  .empty_flag(fifo_empty),.aempty()
 );
 // Parallel metadata only. The original FIFO, requests and RGB registers
 // stay byte-for-byte unchanged. Selection and responses use the SAME slot.
 motion_source_tags #(.PAYLOAD_BUFFER_STAGES(PAYLOAD_BUFFER_STAGES)) u_source_tags(
  .I_rst_n(I_rst_n),.I_source_rst_n(I_source_rst_n),
  .I_ddr_clk(I_ddr_clk),.I_pixel_clk(I_dsi_clk),.I_fifo_reset(fifo_reset),
  .I_select(!new_frame && drain && outstanding==0),
  .I_available(I_frame_available),.I_source_id(I_source_frame_id),
  .I_write(I_ddr_user_rd_valid && !drain && !fifo_reset),.I_read(fifo_read),
  .I_pixel_boundary(I_video_vsync && !vs_d),.I_de_d(de_d),.I_phase_d(phase_d),
  .O_source_id(O_source_frame_id),.O_valid(O_source_valid),
  .I_lat_start_tick(I_lat_start_tick),.I_lat_session(I_lat_session),.I_lat_valid(I_lat_valid),
  .O_lat_start_tick(O_lat_start_tick),.O_lat_session(O_lat_session),.O_lat_valid(O_lat_valid)
 );
 // FIFO output is registered once; RGB output adds one more register.
 // Thus request controls and coordinates need TWO clocks, data needs ZERO.
 always @(posedge I_dsi_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   pixel_phase<=0;phase_d<=0;de_d<=0;vs_d<=0;frame_bad<=1;
   O_fifo_underflow<=0;old_word<=0;O_vdieo_data<=0;
   pixel_good_d<=0;O_pixel_good<=0;
   frame_ready_sync<=0;
  end else begin
   frame_ready_sync<={frame_ready_sync[0],I_frame_available};
   vs_d<=I_video_vsync; de_d<=I_video_rd_en; phase_d<=pixel_phase;
   // Match the existing request/FIFO acceptance edge. An empty FIFO only
   // matters at a fetch phase; buffered non-fetch pixels remain legitimate.
   pixel_good_d<=I_video_rd_en && !frame_bad && !fifo_reset && (!fetch || !fifo_empty);
   O_pixel_good<=de_d && !frame_bad && pixel_good_d && !fifo_reset;
   old_word<=fifo_data;
   // Before the first complete camera frame, black is intentional, not an
   // underflow fault. Arm only at a subsequent frame boundary.
   if(I_video_vsync && !vs_d) frame_bad<=!frame_ready_sync[1];
   if(fetch && fifo_empty && !frame_bad && !fifo_reset) begin
    frame_bad<=1; O_fifo_underflow<=1;
   end
   if(!I_video_rd_en) pixel_phase<=0;
   else pixel_phase<=pixel_phase+1'b1;
   if(de_d && !frame_bad) begin
    case(phase_d)
     0:O_vdieo_data<=fifo_data[127:104];
     1:O_vdieo_data<=fifo_data[103:80];
     2:O_vdieo_data<=fifo_data[79:56];
     3:O_vdieo_data<=fifo_data[55:32];
     4:O_vdieo_data<=fifo_data[31:8];
     5:O_vdieo_data<={old_word[7:0],fifo_data[127:112]};
     6:O_vdieo_data<=fifo_data[111:88];
     7:O_vdieo_data<=fifo_data[87:64];
     8:O_vdieo_data<=fifo_data[63:40];
     9:O_vdieo_data<=fifo_data[39:16];
     10:O_vdieo_data<={old_word[15:0],fifo_data[127:120]};
     11:O_vdieo_data<=fifo_data[119:96];
     12:O_vdieo_data<=fifo_data[95:72];
     13:O_vdieo_data<=fifo_data[71:48];
     14:O_vdieo_data<=fifo_data[47:24];
     15:O_vdieo_data<=fifo_data[23:0];
    endcase
   end else O_vdieo_data<=0;
  end
 end
endmodule
