// Candidate only: exact Y-then-X bilinear Q8 gain field. No production instance.
// Quantized 32-pixel coefficient grid, nearest representative; two native ERAM expected.
// KNOTS identify/validate the generated profile; MEM_FILE must be generated from them.
// RGB + tags have five-clock latency. Valid parameters: WIDTH 12..1280,
// HEIGHT 12..720, each of 27 row-major RGB knots in 1..305 (256 = unity).
module spatial_colour_rom_r14 #(
 parameter integer WIDTH=1280,HEIGHT=720,
 parameter [242:0] KNOTS=243'h39a00d2803e9fef480371c5006738d99007a401bae280351eefa803ca00dc,
 parameter MEM_FILE="field_1280_720.mem"
)(
 input wire I_clk,I_rst_n,I_de,I_user,I_last,I_good,I_enable,
 input wire [11:0] I_x,I_y,
 input wire [23:0] I_rgb,
 input wire [31:0] I_source_id,
 output wire O_de,O_user,O_last,O_good,O_config_bad,
 output wire [11:0] O_x,O_y,
 output wire [23:0] O_rgb,
 output wire [31:0] O_source_id,
 output wire [8:0] O_gain_r,O_gain_g,O_gain_b,
 // ROM coefficients are registered ONE clock after input. Accept remains combinational input validity.
 output wire [8:0] O_gain_r_early,O_gain_g_early,O_gain_b_early,
 // Raw registered field is independent of enable, validity and display bypass.
 output wire[17:0] O_raw_gain_rg,
 output wire O_accept_early
);
 function integer knot;
  input integer n;
  begin knot=(KNOTS>>(n*9))&511;end
 endfunction
 function integer floor_div;
  input integer n,d;
  begin if(n<0)floor_div=-((-n+d-1)/d);else floor_div=n/d;end
 endfunction
 function integer valid_knots;
  input integer unused;
  integer i;
  begin valid_knots=1;for(i=0;i<27;i=i+1)if(knot(i)<1||knot(i)>305)valid_knots=0;end
 endfunction
 localparam integer XM=WIDTH/2,YM=HEIGHT/2;
 localparam integer XL=XM>0?XM:1,XR=WIDTH-1-XM>0?WIDTH-1-XM:1;
 localparam integer YL=YM>0?YM:1,YR=HEIGHT-1-YM>0?HEIGHT-1-YM:1;
 localparam integer VALID=(WIDTH>=12&&WIDTH<=1280&&HEIGHT>=12&&HEIGHT<=720&&((WIDTH-1+16)/32+1)*((HEIGHT-1+16)/32+1)<=1024&&valid_knots(0));
 assign O_config_bad=!VALID;
 reg active,frame_bad,frame_enable;
 reg [31:0] frame_source;
 reg [11:0] expected_x,expected_y;
 wire sof_good=I_de&&I_user&&VALID&&I_good&&I_x==0&&I_y==0&&!I_last;
 wire regular=I_de&&!I_user&&VALID&&active&&!frame_bad&&I_good&&I_x==expected_x&&I_y==expected_y&&
  I_source_id==frame_source&&I_enable==frame_enable&&(I_last==(I_x==WIDTH-1));
 wire accept=sof_good||regular;
 wire enabled=accept&&(I_user?I_enable:frame_enable);
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin active<=0;frame_bad<=0;frame_enable<=0;frame_source<=0;expected_x<=0;expected_y<=0;end
  else if(I_de)begin
   if(I_user)begin
    active<=sof_good;frame_bad<=!sof_good;frame_enable<=I_enable;frame_source<=I_source_id;
    expected_x<=1;expected_y<=0;
   end else if(!regular)begin active<=0;frame_bad<=1;end
   else if(I_x==WIDTH-1)begin
    expected_x<=0;expected_y<=I_y+1'b1;
    if(I_y==HEIGHT-1)active<=0;
   end else expected_x<=I_x+1'b1;
  end
 end
 localparam integer NX=(WIDTH-1+16)/32+1;
 wire [12:0] qx={1'b0,I_x}+13'd16,qy={1'b0,I_y}+13'd16;
 // Constant shifts/adds keep ROM address generation out of scarce DSP blocks.
 function [12:0] row_base;
  input [7:0] row_index;integer j;reg[12:0]acc;
  begin acc=0;for(j=0;j<6;j=j+1)if((NX>>j)&1)acc=acc+({5'd0,row_index}<<j);row_base=acc;end
 endfunction
 wire [12:0] address=row_base(qy[12:5])+{5'd0,qx[12:5]};
 reg [26:0] coeff_mem[0:1023];
 reg [26:0] coeff_word;
 initial $readmemh(MEM_FILE,coeff_mem);
 always @(posedge I_clk)coeff_word<=coeff_mem[address[9:0]];
 reg enabled_d;
 reg [23:0] rgb_d;
 wire [8:0] cr=enabled_d?coeff_word[8:0]:9'd256;
 wire [8:0] cg=enabled_d?coeff_word[17:9]:9'd256;
 wire [8:0] cb=enabled_d?coeff_word[26:18]:9'd256;
 reg [16:0] product_r,product_g,product_b;
 wire [8:0] scaled_r=product_r[16:8],scaled_g=product_g[16:8],scaled_b=product_b[16:8];
 wire [7:0] clip_r=scaled_r>255?8'd255:scaled_r[7:0];
 wire [7:0] clip_g=scaled_g>255?8'd255:scaled_g[7:0];
 wire [7:0] clip_b=scaled_b>255?8'd255:scaled_b[7:0];
 reg [23:0] rgb_pipe[0:2];
 reg [4:0] de_pipe,user_pipe,last_pipe,good_pipe;
 reg [11:0] x_pipe[0:4],y_pipe[0:4];reg[31:0]source_pipe[0:4];
 reg [26:0] gain_pipe[0:3];reg[4:0] warm;integer n;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   warm<=0;rgb_d<=0;enabled_d<=0;product_r<=0;product_g<=0;product_b<=0;
   de_pipe<=0;user_pipe<=0;last_pipe<=0;good_pipe<=0;
   for(n=0;n<3;n=n+1)rgb_pipe[n]<=0;
   for(n=0;n<4;n=n+1)gain_pipe[n]<=0;
   for(n=0;n<5;n=n+1)begin x_pipe[n]<=0;y_pipe[n]<=0;source_pipe[n]<=0;end
  end else begin
   warm<={warm[3:0],1'b1};rgb_d<=I_rgb;enabled_d<=enabled;
   product_r<=rgb_d[23:16]*cr;product_g<=rgb_d[15:8]*cg;product_b<=rgb_d[7:0]*cb;
   rgb_pipe[0]<={clip_r,clip_g,clip_b};for(n=1;n<3;n=n+1)rgb_pipe[n]<=rgb_pipe[n-1];
   de_pipe<={de_pipe[3:0],I_de};user_pipe<={user_pipe[3:0],I_user};last_pipe<={last_pipe[3:0],I_last};good_pipe<={good_pipe[3:0],accept};
   x_pipe[0]<=I_x;y_pipe[0]<=I_y;source_pipe[0]<=I_source_id;
   for(n=1;n<5;n=n+1)begin x_pipe[n]<=x_pipe[n-1];y_pipe[n]<=y_pipe[n-1];source_pipe[n]<=source_pipe[n-1];end
   gain_pipe[0]<={cr,cg,cb};for(n=1;n<4;n=n+1)gain_pipe[n]<=gain_pipe[n-1];
  end
 end
 assign O_rgb=rgb_pipe[2];assign O_de=de_pipe[4];assign O_user=user_pipe[4];assign O_last=last_pipe[4];assign O_good=good_pipe[4];
 assign O_x=x_pipe[4];assign O_y=y_pipe[4];assign O_source_id=source_pipe[4];
 assign {O_gain_r,O_gain_g,O_gain_b}=warm[4]?gain_pipe[3]:27'd0;
 assign O_gain_r_early=cr;assign O_gain_g_early=cg;assign O_gain_b_early=cb;
 assign O_raw_gain_rg=coeff_word[17:0];
 assign O_accept_early=accept;
endmodule
