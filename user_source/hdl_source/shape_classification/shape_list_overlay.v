// Six-clock overlay, in parallel with existing six-clock OSD. Frame-constant
// packets arrive in blanking. Font, marker and RGB delays are not altered.
module shape_list_overlay #(parameter IMG_WIDTH=1280,IMG_HEIGHT=720)(
 input I_clk,I_rst_n,I_de,I_enable,input[11:0]I_x,I_y,input[248:0]I_packets,
 output reg O_hit,output reg[23:0]O_color
);
 wire[82:0]p0=I_packets[82:0],p1=I_packets[165:83],p2=I_packets[248:166];
 // Anchor placement is a short blanking-time job, not 15 parallel
 // candidate-row networks on every video pixel. It preserves the same order
 // and first-free-row rule tested by the original independent pixel maps.
 // 2026-11-02: 标签宽度 192 -> 224，给 X 与 Y 之间让出两个 16px 空格
 localparam [11:0] MAX_X=IMG_WIDTH>=224?IMG_WIDTH-224:0,MAX_Y=IMG_HEIGHT>=48?IMG_HEIGHT-48:0;
 reg[11:0]ax[0:2],ay[0:2];
 wire[146:0]layout_key={p2[82],p2[79:32],p1[82],p1[79:32],p0[82],p0[79:32]};
 reg[146:0]layout_seen;reg layout_ready;reg[1:0]layout_state,li,lj;reg[2:0]try_row;
 reg[11:0]base_x,base_y;reg signed[13:0]trial_y;
 wire[48:0]layout_box=layout_seen[li*49+:49];
 wire[12:0]initial_y=layout_box[23:12]>=52?{1'b0,layout_box[23:12]}-13'd52:{1'b0,layout_box[11:0]}+13'd4;
 always @*case(try_row)
  0:trial_y=$signed({2'b0,base_y});1:trial_y=$signed({2'b0,base_y})-14'sd48;
  2:trial_y=$signed({2'b0,base_y})+14'sd48;3:trial_y=$signed({2'b0,base_y})-14'sd96;
  default:trial_y=$signed({2'b0,base_y})+14'sd96;
 endcase
 wire collision=lj<li&&layout_seen[lj*49+48]&&
  {1'b0,base_x}<{1'b0,ax[lj]}+13'd224&&{1'b0,base_x}+13'd224>{1'b0,ax[lj]}&&
  trial_y<$signed({2'b0,ay[lj]})+14'sd48&&trial_y+14'sd48>$signed({2'b0,ay[lj]});
 integer a,n2;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   layout_seen<=0;layout_ready<=0;layout_state<=0;li<=0;lj<=0;try_row<=0;base_x<=0;base_y<=0;
   for(a=0;a<3;a=a+1)begin ax[a]<=0;ay[a]<=0;end
  end else if(layout_key!=layout_seen)begin
   layout_seen<=layout_key;layout_ready<=0;layout_state<=1;li<=0;lj<=0;try_row<=0;
  end else case(layout_state)
   0:;
   1:begin
    base_x<=layout_box[47:36]>MAX_X?MAX_X:layout_box[47:36];
    base_y<=initial_y>MAX_Y?MAX_Y:initial_y[11:0];lj<=0;try_row<=0;layout_state<=2;
   end
   2:begin
    if(trial_y<0||trial_y>$signed({2'b0,MAX_Y})||collision)begin
     lj<=0;if(try_row==4)begin layout_ready<=0;layout_state<=0;end else try_row<=try_row+1'b1;
    end else if(lj>=li)begin
     ax[li]<=base_x;ay[li]<=trial_y[11:0];
     if(li==2)layout_state<=3;else begin li<=li+1'b1;layout_state<=1;end
    end else lj<=lj+1'b1;
   end
   3:begin layout_ready<=1;layout_state<=0;end
  endcase
 end
 reg label_hit,box_hit;reg[7:0]local_x;reg[5:0]local_y;reg[1:0]kind;reg[15:0]bx,by;
 reg[11:0]l,r,t,b;reg[82:0]word;reg got;
 always @*begin
  label_hit=0;box_hit=0;local_x=0;local_y=0;kind=0;bx=0;by=0;l=0;r=0;t=0;b=0;word=0;got=0;
  for(n2=0;n2<3;n2=n2+1)begin
   word=I_packets[n2*83+:83];l=word[79:68];r=word[67:56];t=word[55:44];b=word[43:32];
   if(I_enable&&I_de&&layout_ready&&layout_key==layout_seen&&word[82]&&word[81:80]!=0)begin
    if(I_x>=l&&I_x<=r&&I_y>=t&&I_y<=b&&
     (I_x<l+2||I_x+2>r||I_y<t+2||I_y+2>b))box_hit=1;
    if(!got&&I_x>=ax[n2]&&I_x<ax[n2]+224&&I_y>=ay[n2]&&I_y<ay[n2]+48)begin
     got=1;label_hit=1;local_x=I_x-ax[n2];local_y=I_y-ay[n2];kind=word[81:80];bx=word[31:16];by=word[15:0];
    end
   end
  end
 end
 reg[7:0]x0;reg[5:0]y0;reg[1:0]kind0;reg[15:0]bx0,by0;reg valid0;
 reg[7:0]character;reg[4:0]col,row;reg character_valid;reg[3:0]digit,slot;
 always @*begin
  character=8'h20;col=0;row=0;character_valid=valid0;digit=0;slot=x0[7:4];
  if(y0<24)begin
   row=y0[4:0];
   if(x0<24)begin character=kind0==1?8'h92:kind0==2?8'h94:8'h96;col=x0[4:0];end
   else if(x0<48)begin character=kind0==2?8'h95:8'h93;col=x0-24;end
   else if(x0<72&&kind0==2)begin character=8'h93;col=x0-48;end
   else character_valid=0;
  end else begin
   row=y0-24;col={1'b0,x0[3:0]};
   case(slot)
    0:character="X";1:character=":";
    2,3,4,5:begin digit=bx0[(5-slot)*4+:4];character="0"+digit;end
    6,7:character_valid=0;                               // 两个空格，隔开 X 与 Y
    8:character="Y";9:character=":";
    10,11,12,13:begin digit=by0[(13-slot)*4+:4];character="0"+digit;end
    default:character_valid=0;
   endcase
  end
 end
 reg[7:0]ch1;reg[4:0]row1,col1,col2;reg valid1,valid2;wire[23:0]font_bits;wire[23:0]bits2=font_bits;
 reg hit3,hit4;reg[4:0]box_q;
 shape_list_font_rom glyph(.I_clk(I_clk),.I_char(ch1),.I_row(row1),.O_row_bits(font_bits));
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin x0<=0;y0<=0;kind0<=0;bx0<=0;by0<=0;valid0<=0;ch1<=0;row1<=0;col1<=0;valid1<=0;col2<=0;valid2<=0;hit3<=0;hit4<=0;box_q<=0;O_hit<=0;O_color<=0;end
  else begin
   x0<=local_x;y0<=local_y;kind0<=kind;bx0<=bx;by0<=by;valid0<=label_hit;
   ch1<=character;row1<=row;col1<=col;valid1<=character_valid;
   col2<=col1;valid2<=valid1;
   hit3<=valid2&&col2<24&&bits2[23-col2];hit4<=hit3;
   // 2026-11-02: 文字、坐标和框统一用绿色（原来是文字黄、框绿）
   box_q<={box_q[3:0],box_hit};O_hit<=hit4||box_q[4];O_color<=24'h00ff00;
  end
 end
endmodule
