// M02: atomic source identity alongside each existing 128-bit DDR response.
// The vendor async FIFO transports the complete 33-bit packet. No per-bit
// synchronizers, live publication bus sampling in the pixel domain, or DDR IO.
//
// L03 (2026-10-18): 在原有的 33 位 {valid, source_id} 之后追加 49 位测量描述符。
//   标签包 33 -> 82 位，原 33 位的**位置与语义逐位不变**：
//     [32]    selected_valid     （原样）
//     [31:0]  selected_id        （原样）
//     [81]    lat_valid          （新增）
//     [80:65] time_session       （新增）
//     [64:33] start_tick         （新增）
//   冻结时机与 selected_id 完全相同（I_select 那一沿），所以测量身份与原源身份
//   天然属于同一个发布快照。
//   帧内描述符不一致只让**测量**失效（O_lat_valid=0），绝不改变原 frame_bad/good，
//   即不会让原图像/颜色/运动/形状的源有效性变差。
module motion_source_tags #(parameter PAYLOAD_BUFFER_STAGES=0)(
 input wire I_rst_n,I_source_rst_n,I_ddr_clk,I_pixel_clk,I_fifo_reset,
 input wire I_select,I_available,
 input wire [31:0] I_source_id,
 input wire [31:0] I_lat_start_tick,   // L03: 发布侧描述符
 input wire [15:0] I_lat_session,
 input wire        I_lat_valid,
 input wire I_write,I_read,I_pixel_boundary,I_de_d,
 input wire [3:0] I_phase_d,
 output reg [31:0] O_source_id,
 output reg O_valid,
 output reg [31:0] O_lat_start_tick,   // L03: 像素域描述符（与 O_source_id 同帧）
 output reg [15:0] O_lat_session,
 output reg        O_lat_valid
);
 wire reset_n=I_rst_n && I_source_rst_n;
 reg [31:0] selected_id;
 reg selected_valid;
 reg [31:0] selected_lat_tick;
 reg [15:0] selected_lat_sess;
 reg        selected_lat_valid;
 always @(posedge I_ddr_clk or negedge reset_n) begin
  if(!reset_n) begin
   selected_id<=0;selected_valid<=0;
   selected_lat_tick<=0;selected_lat_sess<=0;selected_lat_valid<=0;
  end
  else if(I_select) begin
   selected_id<=I_source_id;selected_valid<=I_available;
   selected_lat_tick<=I_lat_start_tick;
   selected_lat_sess<=I_lat_session;
   selected_lat_valid<=I_lat_valid;
  end
 end
 wire [81:0] tag;
 wire [81:0] fifo_payload;
 fifo_payload_hold_buffer #(.WIDTH(82),.STAGES(PAYLOAD_BUFFER_STAGES),.BUFFER_MASK(82'h3FFFFFFFFFFFFFFFFFFFF)) tag_payload_hold(
  .I_data({selected_lat_valid,selected_lat_sess,selected_lat_tick,selected_valid,selected_id}),
  .O_data(fifo_payload));
 wire empty,tag_valid;
 soft_fifo_al_4057d6b76aa6 #(
  .DATA_WIDTH_W(82),.DATA_WIDTH_R(82),.ADDR_WIDTH_W(9),.ADDR_WIDTH_R(9),
  .AL_FULL_NUM(253),.AL_EMPTY_NUM(2),.SHOW_AHEAD_EN(0),.OUTREG_EN("NOREG")
 ) u_tag_fifo(
  .rst(I_fifo_reset || !reset_n),.clkw(I_ddr_clk),.clkr(I_pixel_clk),
  .we(I_write),
  .di(fifo_payload),
  .re(I_read),
  .dout(tag),.valid(tag_valid),.full_flag(),.empty_flag(empty),
  .afull(),.aempty(),.wrusedw(),.rdusedw()
 );
 reg read_good_d,first_pixel,frame_bad;
 reg [31:0] frame_id;
 // L03: 帧内描述符一致性。lat_bad 只用于测量，不进 frame_bad。
 reg [31:0] frame_lat_tick;
 reg [15:0] frame_lat_sess;
 reg        frame_lat_valid;
 reg        lat_bad;
 reg  [48:0] frame_desc;
 wire word_sample=(I_phase_d==0 || I_phase_d==5 || I_phase_d==10);
 wire [48:0] word_desc={tag[81],tag[80:65],tag[64:33]};
 wire desc_mismatch=!first_pixel && word_sample && (word_desc!=frame_desc);
 wire word_good=read_good_d && tag_valid && tag[32];
 wire mismatch=!first_pixel && word_sample && tag[31:0]!=frame_id;
 wire good=!frame_bad && (!word_sample || word_good) && !mismatch;
 always @(posedge I_pixel_clk or negedge reset_n) begin
  if(!reset_n) begin
   read_good_d<=0;first_pixel<=1;frame_bad<=1;frame_id<=0;
   O_source_id<=0;O_valid<=0;
   frame_lat_tick<=0;frame_lat_sess<=0;frame_lat_valid<=0;lat_bad<=0;frame_desc<=0;
   O_lat_start_tick<=0;O_lat_session<=0;O_lat_valid<=0;
  end else begin
   read_good_d<=I_read && !empty && !I_fifo_reset;
   O_valid<=0;
   // A reset during an established frame also revokes its measurement in
   // blanking. A prefetch reset BEFORE the first pixel must not poison the
   // next frame; the first real tag establishes that frame's eligibility.
   if(I_fifo_reset && !first_pixel) begin lat_bad<=1; O_lat_valid<=0; end
   if(I_pixel_boundary) begin
    first_pixel<=1;frame_bad<=0;frame_id<=0;O_source_id<=0;
    frame_lat_tick<=0;frame_lat_sess<=0;frame_lat_valid<=0;lat_bad<=0;frame_desc<=0;
    O_lat_start_tick<=0;O_lat_session<=0;O_lat_valid<=0;
   end else if(I_de_d) begin
    if(first_pixel) begin
     // A missing first tag makes this entire display frame ineligible.
     first_pixel<=0;frame_id<=tag[31:0];O_source_id<=tag[31:0];
     O_valid<=good && word_sample && word_good && !I_fifo_reset;
     // 描述符：本帧首标签建立身份
     frame_desc<=word_desc;
     frame_lat_tick<=tag[64:33];
     frame_lat_sess<=tag[80:65];
     frame_lat_valid<=tag[81];
     O_lat_start_tick<=tag[64:33];
     O_lat_session<=tag[80:65];
     O_lat_valid<=tag[81] && good && word_sample && word_good && !I_fifo_reset;
     if(!word_sample || !word_good || I_fifo_reset) begin frame_bad<=1; lat_bad<=1; end
     if(!tag[81]) lat_bad<=1;
    end else begin
     O_source_id<=frame_id;
     O_valid<=good && !I_fifo_reset;
     O_lat_start_tick<=frame_lat_tick;
     O_lat_session<=frame_lat_sess;
     O_lat_valid<=frame_lat_valid && !lat_bad && good && !desc_mismatch && !I_fifo_reset;
     if(!good || I_fifo_reset) frame_bad<=1;
     if(!good || I_fifo_reset || desc_mismatch) lat_bad<=1;
    end
   end
  end
 end
endmodule
