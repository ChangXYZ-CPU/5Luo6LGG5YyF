// Carry SOF/error with data. Never reset the FIFO at frame start.
module video_in #(parameter FRAME_WORDS=172800, parameter BURST_WORDS=240)(
 input wire I_rst_n, I_camera_clk, I_camera_frame_start, I_camera_valid,
 input wire [127:0] I_camera_data,
 input wire I_mipi_rx_error, I_ddr_clk, I_display_pause, I_video_out_rd_busy,
 input wire [1:0] I_display_slot,
 // L03: 公共时基（本模块 I_camera_clk 域已同步好的值）
 input wire [31:0] I_tick_now,
 input wire [15:0] I_time_session,
 input wire        I_time_ready,
 output wire O_video_in_wr_busy,
 output reg [1:0] O_video_out_rp,
 output reg O_frame_available,
 // M02: valid publication generation, atomically paired with O_video_out_rp.
 output reg [31:0] O_source_frame_id,
 // L03: 测量描述符，更新边沿与 O_source_frame_id 完全相同（原子发布）
 output reg [31:0] O_lat_start_tick,
 output reg [15:0] O_lat_session,
 output reg        O_lat_valid,
 output wire O_ddr_user_wr_en,
 output wire [24:0] O_ddr_user_addr,
 output wire [127:0] O_ddr_user_wr_data,
 input wire I_ddr_user_ready,
 output reg O_fifo_overflow
);
 reg sof_pending, camera_bad;
 // L03: SOF 时刻的起始时间戳锁存（SOF 可能早于首个有效字）
 reg [31:0] sof_tick_q;
 reg [15:0] sof_sess_q;
 reg        sof_valid_q;
 wire fifo_full, fifo_empty, fifo_valid;
 wire [8:0] fifo_count;
 wire [178:0] fifo_data;
 wire sof=I_camera_frame_start | sof_pending;
 wire bad=I_camera_frame_start ? I_mipi_rx_error : camera_bad | I_mipi_rx_error;
 // SOF 与首个有效字同拍时，非阻塞赋值会拿到旧寄存值；此处组合旁路新值。
 wire [31:0] w_sof_tick = I_camera_frame_start ? I_tick_now     : sof_tick_q;
 wire [15:0] w_sof_sess = I_camera_frame_start ? I_time_session : sof_sess_q;
 wire        w_sof_val  = I_camera_frame_start ? I_time_ready   : sof_valid_q;
 always @(posedge I_camera_clk or negedge I_rst_n) begin
  if (!I_rst_n) begin sof_pending<=0; camera_bad<=0; O_fifo_overflow<=0;
                      sof_tick_q<=0; sof_sess_q<=0; sof_valid_q<=0; end
  else begin
   if (I_camera_frame_start) begin sof_pending<=1; camera_bad<=I_mipi_rx_error;
                                    sof_tick_q<=I_tick_now;
                                    sof_sess_q<=I_time_session;
                                    sof_valid_q<=I_time_ready; end
   else if (I_mipi_rx_error) camera_bad<=1;
   if (I_camera_valid && !fifo_full) sof_pending<=0;
   if (I_camera_valid && fifo_full) begin camera_bad<=1; O_fifo_overflow<=1; end
  end
 end
 reg burst_active;
 reg [8:0] burst_count;
 wire fifo_pop=burst_active && I_ddr_user_ready && !fifo_empty;
 soft_fifo_al_4057d6b76aa6 #(
  .DATA_WIDTH_W(179),.DATA_WIDTH_R(179),.ADDR_WIDTH_W(9),.ADDR_WIDTH_R(9),
  .AL_FULL_NUM(253),.AL_EMPTY_NUM(2),.SHOW_AHEAD_EN(0),.OUTREG_EN("NOREG")
 ) u_tagged_fifo (
  .rst(!I_rst_n),.clkw(I_camera_clk),.clkr(I_ddr_clk),
  // L03: 原 130 位 {sof,bad,data} 仍在低 130 位，测量描述符只追加在旁路位置
  .we(I_camera_valid),.di({w_sof_val,w_sof_sess,w_sof_tick,sof,bad,I_camera_data}),
  .re(fifo_pop),.dout(fifo_data),.valid(fifo_valid),
  .full_flag(fifo_full),.empty_flag(fifo_empty),
  .afull(),.aempty(),.wrusedw(),.rdusedw(fifo_count)
 );
 // Reserve an idle arbitration cycle BEFORE registering burst_active.
 // Otherwise both reader and writer can see the other idle on the same
 // edge and start bursts together. Writer has deterministic idle priority;
 // an already active reader burst retains ownership until it ends.
 wire reserve_write=!burst_active && !fifo_valid && !I_video_out_rd_busy &&
                    fifo_count>=BURST_WORDS;
 assign O_video_in_wr_busy=burst_active | fifo_valid | reserve_write;
 always @(posedge I_ddr_clk or negedge I_rst_n) begin
  if (!I_rst_n) begin burst_active<=0; burst_count<=0; end
  else if (reserve_write) begin
   burst_active<=1; burst_count<=0;
  end else if (fifo_pop) begin
   if (burst_count==BURST_WORDS-1) begin burst_active<=0; burst_count<=0; end
   else burst_count<=burst_count+1'b1;
  end
 end
 function [24:0] base;
  input [1:0] slot;
  begin case(slot)
   0:base=25'd0; 1:base=25'd7000000; 2:base=25'd14000000; 3:base=25'd21000000;
  endcase end
 endfunction
 function [1:0] free_slot;
  input [1:0] completed,display;
  reg [1:0] candidate;
  begin
   candidate=completed+1'b1;
   if(candidate==display) candidate=candidate+1'b1;
   free_slot=candidate;
  end
 endfunction
 reg [1:0] write_slot,next_slot;
 reg [17:0] word_count;
 reg [24:0] next_addr;
 reg frame_active,frame_bad;
 // L03: 工作描述符（在 SOF 字锁存，到发布沿才提交）
 reg [31:0] wlat_tick;
 reg [15:0] wlat_sess;
 reg        wlat_valid;
 wire [1:0] new_slot=(next_slot==I_display_slot) ? next_slot+1'b1 : next_slot;
 wire valid_word=fifo_valid && !I_display_pause && (fifo_data[129] || frame_active);
 assign O_ddr_user_wr_en=valid_word;
 assign O_ddr_user_addr=fifo_data[129] ? base(new_slot) : next_addr;
 assign O_ddr_user_wr_data=fifo_data[127:0];
 always @(posedge I_ddr_clk or negedge I_rst_n) begin
  if (!I_rst_n) begin
   write_slot<=0; next_slot<=0; word_count<=0; next_addr<=0;
   frame_active<=0; frame_bad<=0; O_video_out_rp<=0; O_frame_available<=0;
   O_source_frame_id<=0;
   O_lat_start_tick<=0; O_lat_session<=0; O_lat_valid<=0;
   wlat_tick<=0; wlat_sess<=0; wlat_valid<=0;
  end else if(valid_word) begin
   if(fifo_data[129]) begin
    write_slot<=new_slot; next_addr<=base(new_slot)+25'd8; word_count<=1;
    frame_active<=1; frame_bad<=fifo_data[128];
    // L03: SOF 字携带本帧起始时间戳与会话，锁进工作描述符
    wlat_tick<=fifo_data[161:130];
    wlat_sess<=fifo_data[177:162];
    wlat_valid<=fifo_data[178];
   end else begin
    next_addr<=next_addr+25'd8; word_count<=word_count+1'b1;
    frame_bad<=frame_bad | fifo_data[128];
    if(word_count==FRAME_WORDS-1) begin
     frame_active<=0;
     if(!frame_bad && !fifo_data[128]) begin
      O_video_out_rp<=write_slot; O_frame_available<=1;
      O_source_frame_id<=O_source_frame_id+32'd1;
      // L03: 描述符与 rp/source_id 同一沿发布，三个量属于同一快照
      O_lat_start_tick<=wlat_tick;
      O_lat_session<=wlat_sess;
      O_lat_valid<=wlat_valid;
      next_slot<=free_slot(write_slot,I_display_slot);
     end
    end
   end
  end
 end
endmodule
