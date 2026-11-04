// Exact-coordinate Bayer windows. One-row lookahead; reflected image edges.
module raw_matrix_3x3_buffer #(
 parameter IMG_HEIGHT=1080,IMG_WIDTH=1920,data_complete_delay=50
)(
 input I_clk,I_rst_n,input [39:0] axi4s_video_tdata,
 input [9:0] axi4s_video_tdest,input axi4s_video_tlast,axi4s_video_tvalid,axi4s_video_tuser,
 output axi4s_video_tready,output reg O_tlast,O_tuser,O_tvalid,input O_tready,
 output reg bayer_ypos,output reg [95:0] matrix_last_line,matrix_cur_line,matrix_next_line
);
 localparam WORDS=IMG_WIDTH/4;
 assign axi4s_video_tready=O_tready;
 wire [31:0] raw;wire rv,ru,rl;
 mipi_to_raw_converter u_raw(.I_clk(I_clk),.I_rst_n(I_rst_n),
  .I_raw_tlast(axi4s_video_tlast),.I_raw_tuser(axi4s_video_tuser),
  .I_raw_tdata(axi4s_video_tdata),.I_raw_tdest(axi4s_video_tdest),
  .I_raw_tvalid(axi4s_video_tvalid),.I_raw_tready(),
  .O_raw_tlast(rl),.O_raw_tuser(ru),.O_raw_tdata(raw),.O_raw_tvalid(rv),.O_raw_tready(O_tready));
 wire [31:0] zd;wire zv,zu,zl;
 zhenghe #(.IMG_WIDTH(IMG_WIDTH),.IMG_HEIGHT(IMG_HEIGHT),.DATA_WIDTH(32),
  .data_complete_delay(data_complete_delay)) u_formatter(I_clk,I_rst_n,rl,ru,raw,rv,zl,zu,zd,zv);
 reg [8:0] in_x,flush_x;
 reg [12:0] in_y;
 reg [1:0] bank;
 reg [1:0] bottom_bank;
 reg bottom;
 wire [8:0] wx=zu?9'd0:in_x;
 wire [12:0] wy=zu?13'd0:in_y;
 // Keep the circular write bank across frames so a short blanking interval
 // cannot overwrite the preceding frame's bottom-row neighbourhoods.
 wire [1:0] wb=bank;
 wire [1:0] prev_bank=wb-2'd1;
 wire [1:0] top_bank=wb-2'd2;
 wire [1:0] bottom_top_bank=bottom_bank-2'd1;
 wire [10:0] cur_addr=bottom?{bottom_bank,flush_x}:{prev_bank,wx};
 wire [10:0] top_addr=bottom?{bottom_top_bank,flush_x}:{top_bank,wx};
 wire [32:0] cur_ram,top_ram;
 blk_mem_gen_demosaic u_cur(.clka(I_clk),.wea(zv),.addra({wb,wx}),.dia({1'b0,zd}),.doa(),
  .clkb(I_clk),.web(1'b0),.addrb(cur_addr),.dib(33'd0),.dob(cur_ram));
 blk_mem_gen_demosaic u_top(.clka(I_clk),.wea(zv),.addra({wb,wx}),.dia({1'b0,zd}),.doa(),
  .clkb(I_clk),.web(1'b0),.addrb(top_addr),.dib(33'd0),.dob(top_ram));
 reg st_valid,st_bottom;
 reg [8:0] st_x;
 reg [12:0] st_y;
 reg [31:0] st_data;
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   in_x<=0;in_y<=0;bank<=0;bottom_bank<=0;bottom<=0;flush_x<=0;
   st_valid<=0;st_bottom<=0;st_x<=0;st_y<=0;st_data<=0;
  end else begin
   if(zv) begin
    if(wx==WORDS-1) begin
     in_x<=0;in_y<=wy+1'b1;bank<=wb+1'b1;
     if(wy==IMG_HEIGHT-1) begin bottom<=1;bottom_bank<=wb;flush_x<=0;end
    end else begin in_x<=wx+1'b1;in_y<=wy;bank<=wb;end
   end
   if(bottom) begin
    if(flush_x==WORDS-1) begin bottom<=0;flush_x<=0;end
    else flush_x<=flush_x+1'b1;
   end
   st_valid<=bottom||(zv&&wy!=0);
   st_bottom<=bottom;st_x<=bottom?flush_x:wx;
   st_y<=bottom?(IMG_HEIGHT-1):(wy-1'b1);st_data<=zd;
  end
 end
 wire [31:0] top_word=st_bottom?top_ram[31:0]:(st_y==0)?st_data:top_ram[31:0];
 wire [31:0] cur_word=cur_ram[31:0];
 wire [31:0] bot_word=st_bottom?top_ram[31:0]:st_data;
 reg [31:0] prev_top,prev_cur,prev_bot;
 reg [7:0] left_top,left_cur,left_bot;
 reg prev_parity,flush_pending;
 function [95:0] window;
  input [7:0] left;input [31:0] centre;input [7:0] right;
  begin window={24'd0,left,centre,right,24'd0};end
 endfunction
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   prev_top<=0;prev_cur<=0;prev_bot<=0;left_top<=0;left_cur<=0;left_bot<=0;
   prev_parity<=0;flush_pending<=0;bayer_ypos<=0;
   O_tvalid<=0;O_tuser<=0;O_tlast<=0;
   matrix_last_line<=0;matrix_cur_line<=0;matrix_next_line<=0;
  end else begin
   O_tvalid<=0;O_tuser<=0;O_tlast<=0;flush_pending<=0;
   if(st_valid) begin
    prev_top<=top_word;prev_cur<=cur_word;prev_bot<=bot_word;prev_parity<=st_y[0];
    left_top<=(st_x==0)?top_word[23:16]:prev_top[7:0];
    left_cur<=(st_x==0)?cur_word[23:16]:prev_cur[7:0];
    left_bot<=(st_x==0)?bot_word[23:16]:prev_bot[7:0];
    if(st_x!=0) begin
     O_tvalid<=1;O_tuser<=(st_x==1)&&(st_y==0);bayer_ypos<=st_y[0];
     matrix_last_line<=window(left_top,prev_top,top_word[31:24]);
     matrix_cur_line<=window(left_cur,prev_cur,cur_word[31:24]);
     matrix_next_line<=window(left_bot,prev_bot,bot_word[31:24]);
    end
    if(st_x==WORDS-1) flush_pending<=1;
   end
   if(flush_pending) begin
    O_tvalid<=1;O_tuser<=0;O_tlast<=1;bayer_ypos<=prev_parity;
    matrix_last_line<=window(left_top,prev_top,prev_top[15:8]);
    matrix_cur_line<=window(left_cur,prev_cur,prev_cur[15:8]);
    matrix_next_line<=window(left_bot,prev_bot,prev_bot[15:8]);
   end
  end
 end
endmodule
