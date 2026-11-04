// H03 standalone implementation of frozen H02 shape_model.mjs geometry.
// No board ports, display timing, CCA, mask capture or publication are implied.
module shape_classifier #(
 parameter integer BUDGET=131072,
 // 0: preserve parallel start-snapshot API. 1: caller-owned stable point
 // table, asynchronous read at O_point_index during the 16 LOAD_POINTS steps.
 parameter integer SERIAL_SUPPORT=0,
 // Paper callers use only the rectangle predicate. Preserve all rectangle
 // tests; omit only non-rectangle classification/refinement work.
 parameter integer RECTANGLE_ONLY=0
)(
 input wire I_clk,I_rst_n,I_cancel,I_start,
 input wire [15:0] I_area,
 input wire [8:0] I_xmin,I_xmax,
 input wire [7:0] I_ymin,I_ymax,
 input wire [143:0] I_support_x,
 input wire [127:0] I_support_y,
 output reg O_busy,O_done,O_failed,
 output reg [1:0] O_class, // 0 unknown, 1 circle, 2 triangle, 3 rectangle
 output reg [4:0] O_vertices,
 output reg [17:0] O_cycles,
 input wire [16:0] I_point,
 output wire [3:0] O_point_index
);
 // Dense state/return encoding; logical steps, arithmetic waits and budget unchanged.
 localparam [5:0] IDLE=6'd0,SORT_LOAD=6'd1,SORT_STEP=6'd2,
  UNIQUE=6'd3,HULL_INIT=6'd4,HULL_TEST=6'd5,HULL_AFTER=6'd6,
  HULL_PUSH=6'd7,HULL_NEXT=6'd8,COPY_HULL=6'd9,
  AREA_PREP=6'd10,AREA_GO=6'd11,AREA_ADD=6'd12,AREA_FINISH=6'd13,
  SIM_INIT=6'd14,SIM_GO=6'd15,SIM_GOT=6'd16,SIM_SQUARE=6'd17,
  SIM_LHS=6'd18,SIM_RHS=6'd19,SIM_NEXT=6'd20,SIM_DECIDE=6'd21,
  SIM_EPS=6'd22,SIM_REMOVE=6'd23,POLY_INIT=6'd24,
  POLY_GO=6'd25,POLY_ADD=6'd26,POLY_DONE=6'd27,
  EDGE_GO=6'd28,EDGE_SAVE=6'd29,RECT_GO=6'd30,
  RECT_SQUARE=6'd31,RECT_LHS=6'd32,RECT_RHS=6'd33,
  CIRCLE_INIT=6'd34,CIRCLE_GO=6'd35,CIRCLE_SAVE=6'd36,
  CIRCLE_DONE=6'd37,FINISH=6'd38,WAIT_G=6'd39,WAIT_M=6'd40,
  REF_INIT=6'd41,REF_CURRENT=6'd42,REF_CURRENT_SAVE=6'd43,
  REF_CAND=6'd44,REF_SAVE=6'd45,REF_NEXT=6'd46,
  LOAD_POINTS=6'd47,SHIFT_POINT=6'd48,SOL_AREA=6'd49,
  SOL_LOW=6'd50,SOL_HIGH=6'd51,POLY_LHS=6'd52,
  POLY_COMPARE=6'd53,RECT_CONST=6'd54,ROUND_SIZE=6'd55,
  ROUND_ASPECT=6'd56,ROUND_AREA=6'd57,ROUND_MIN=6'd58,
  ROUND_MAX=6'd59,ROUND_MAXR=6'd60,ROUND_MINR=6'd61;
 reg [5:0] state,g_return,m_return;
 // Shared RAM: P[0:15], H[16:47], Q[48:63], edge length2[64:67].
 // Four fetch clocks precede each point-consuming logical FSM step.
 // RAM is not reset: each job writes all entries before reading them.
 (* ram_style="bram" *) reg [16:0] points[0:1023];
 reg [8:0] sx[0:15]; reg [7:0] sy[0:15];
 reg [2:0] fetch_phase;
 reg fetch_needed;
 reg [6:0] ra0,ra1,ra2,pw_addr;
 reg pw_en;
 reg [16:0] pw_data,ram_q,forward_data,p0,p1,p2;
 reg forward_hit;
 wire [6:0] read_addr=fetch_phase==0?ra0:fetch_phase==1?ra1:ra2;
 wire [16:0] point_read=forward_hit?forward_data:ram_q;
 wire [8:0] p0x=p0[16:8],p1x=p1[16:8],p2x=p2[16:8];
 wire [7:0] p0y=p0[7:0],p1y=p1[7:0],p2y=p2[7:0];
 task write_point;
  input [6:0] address;input [8:0] x;input [7:0] y;
  begin pw_en<=1;pw_addr<=address;pw_data<={x,y};end
 endtask
 reg [8:0] keyx,xmin,xmax;
 reg [7:0] keyy,ymin,ymax;
 reg [15:0] area;
 reg [8:0] width,dimension;
 reg [7:0] height;
 reg [15:0] box_area;
 reg [3:0] i,j,idx;
 reg [4:0] n,unique_n,k,hull_n,upper_limit,q_n;
 reg upper;
 reg [3:0] copy_idx,remove_idx,best_idx;
 wire [8:0] load_x=SERIAL_SUPPORT?I_point[16:8]:sx[copy_idx[3:0]];
 wire [7:0] load_y=SERIAL_SUPPORT?I_point[7:0]:sy[copy_idx[3:0]];
 assign O_point_index=copy_idx[3:0];
 // Accepted boxes are <=160x160: |cross(local)|<=159^2,
 // length2/dot magnitude<=2*159^2. Thus cc<2^30, cc*length2<2^46.
 // Polygon prefix sums use signed21 (16 absolute-coordinate crosses).
 reg signed [20:0] sum_area;
 reg [15:0] center_area2,poly_area2,cell_area2;
 reg [29:0] cc,best_cc;
 reg [47:0] lhs;
 reg [15:0] ll,best_ll,min_edge,max_edge,min_radius,max_radius;
 reg best_valid;
 reg [1:0] result;
 reg [1:0] refine_round;
 reg [3:0] candidate_idx;
 reg signed [16:0] best_corner;
 reg [8:0] refine_x;
 reg [7:0] refine_y;
 integer z;
 wire [3:0] prev_wire=(idx==0)?q_n-1:idx-1;
 wire [3:0] next_wire=(idx+1==q_n)?0:idx+1;
 wire [8:0] eps_num=(dimension<64)?9'd64:dimension;
 wire signed [27:0] raw_cross,raw_dot;
 wire signed [16:0] g_cross=raw_cross[16:0],g_dot=raw_dot[16:0];
 wire [15:0] abs_cross=g_cross[16]?(~g_cross+17'd1):g_cross;
 wire [15:0] abs_dot=g_dot[16]?(~g_dot+17'd1):g_dot;
 wire abort=I_cancel || (O_busy && (O_cycles>=BUDGET-1 || I_start));
 always @(posedge I_clk) begin
  if(pw_en && !abort) points[{3'd0,pw_addr}]<=pw_data;
  ram_q<=points[{3'd0,read_addr}];
  forward_hit<=pw_en && !abort && pw_addr==read_addr;
  forward_data<=pw_data;
 end
 always @* begin
  fetch_needed=1;ra0=0;ra1=0;ra2=0;
  case(state)
   6'd1:ra0={2'd0,i[3:0]};
   6'd2:ra0={2'd0,j[3:0]-4'd1};
   6'd3:begin ra0={2'd0,i[3:0]};ra1={2'd0,unique_n[3:0]-4'd1};end
   6'd5:begin ra0=6'd16+{1'b0,k[4:0]-5'd2};ra1=6'd16+{1'b0,k[4:0]-5'd1};ra2={2'd0,i[3:0]};end
   6'd7:ra0={2'd0,i[3:0]};
   6'd9:ra0=6'd16+{2'd0,copy_idx};
   6'd11,6'd25,6'd28:begin ra0=6'd48+{2'd0,idx[3:0]};ra1=6'd48+{2'd0,next_wire[3:0]};end
   6'd15,6'd42,6'd30:begin
    ra0=6'd48+{2'd0,prev_wire[3:0]};ra1=6'd48+{2'd0,idx[3:0]};ra2=6'd48+{2'd0,next_wire[3:0]};end
   6'd48:ra0=6'd48+{2'd0,copy_idx[3:0]+4'd1};
   6'd43:ra0=6'd48+{2'd0,idx[3:0]};
   6'd44:begin ra0=6'd48+{2'd0,prev_wire[3:0]};ra1=6'd16+{2'd0,candidate_idx};ra2=6'd48+{2'd0,next_wire[3:0]};end
   6'd45:ra0=6'd16+{2'd0,candidate_idx};
   6'd35:if(RECTANGLE_ONLY==0)ra0={2'd0,idx[3:0]};else fetch_needed=0;
   6'd54:begin ra0=7'd64+{3'd0,prev_wire};ra1=7'd64+{3'd0,idx};end
   default:fetch_needed=0;
  endcase
 end
 reg g_start,m_start;
 reg signed [9:0] ax,ay,bx,by,cx,cy;
 wire [27:0] raw_length;
 wire [15:0] g_length=raw_length[15:0];
 wire g_busy,g_done,m_busy,m_done;
 reg [31:0] ma;
 reg [15:0] mb;
 wire [47:0] product;
 shape_geometry #(.COORD_BITS(10)) geom(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(abort),.I_start(g_start),
  .I_ax(ax),.I_ay(ay),.I_bx(bx),.I_by(by),.I_cx(cx),.I_cy(cy),
  .O_busy(g_busy),.O_done(g_done),.O_cross(raw_cross),.O_dot(raw_dot),.O_length2(raw_length));
 shape_mul64 #(.WIDTH(48),.A_WIDTH(32),.B_WIDTH(16),.USE_DSP(2)) mult(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(abort),.I_start(m_start),
  .I_a(ma),.I_b(mb),.O_busy(m_busy),.O_done(m_done),.O_product(product));
 task geometry;
  input signed [11:0] a_x,a_y,b_x,b_y,c_x,c_y;
  input [5:0] ret;
  begin ax<=a_x;ay<=a_y;bx<=b_x;by<=b_y;cx<=c_x;cy<=c_y;
   g_start<=1;g_return<=ret;state<=WAIT_G;end
 endtask
 task multiply;
  input [47:0] a,b;input [5:0] ret;
  begin ma<=a;mb<=b;m_start<=1;m_return<=ret;state<=WAIT_M;end
 endtask
 task finish_unknown;
  begin result<=0;state<=FINISH;end
 endtask
 // Explicit constant shift/add: prevent TD from spending DSP on constants.
 function [31:0] c100;input [31:0] v;begin c100=(v<<6)+(v<<5)+(v<<2);end endfunction
 function [31:0] c94;input [31:0] v;begin c94=(v<<6)+(v<<5)-(v<<1);end endfunction
 function [31:0] c90;input [31:0] v;begin c90=(v<<6)+(v<<4)+(v<<3)+(v<<1);end endfunction
 function [31:0] c112;input [31:0] v;begin c112=(v<<7)-(v<<4);end endfunction
 function [31:0] c200;input [31:0] v;begin c200=(v<<7)+(v<<6)+(v<<3);end endfunction
 function [31:0] c110;input [31:0] v;begin c110=(v<<7)-(v<<4)-(v<<1);end endfunction
 function [31:0] c73;input [31:0] v;begin c73=(v<<6)+(v<<3)+v;end endfunction
 function [31:0] c84;input [31:0] v;begin c84=(v<<6)+(v<<4)+(v<<2);end endfunction
 function [31:0] c125;input [31:0] v;begin c125=(v<<7)-(v<<1)-v;end endfunction
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   state<=IDLE;O_busy<=0;O_done<=0;O_failed<=0;O_class<=0;O_vertices<=0;O_cycles<=0;
   fetch_phase<=0;pw_en<=0;pw_addr<=0;pw_data<=0;p0<=0;p1<=0;p2<=0;
   g_start<=0;m_start<=0;g_return<=0;m_return<=0;ma<=0;mb<=0;ax<=0;ay<=0;bx<=0;by<=0;cx<=0;cy<=0;
   xmin<=0;xmax<=0;ymin<=0;ymax<=0;width<=0;height<=0;dimension<=0;area<=0;box_area<=0;
   keyx<=0;keyy<=0;i<=0;j<=0;n<=0;unique_n<=0;k<=0;hull_n<=0;upper_limit<=0;
   q_n<=0;idx<=0;upper<=0;copy_idx<=0;remove_idx<=0;best_idx<=0;
   sum_area<=0;center_area2<=0;poly_area2<=0;cell_area2<=0;cc<=0;best_cc<=0;lhs<=0;
   ll<=0;best_ll<=0;min_edge<=0;max_edge<=0;min_radius<=0;max_radius<=0;best_valid<=0;result<=0;
   refine_round<=0;candidate_idx<=0;best_corner<=0;refine_x<=0;refine_y<=0;
  end else if(I_cancel) begin
   state<=IDLE;O_busy<=0;O_done<=0;O_failed<=0;O_class<=0;O_vertices<=0;O_cycles<=0;
   g_start<=0;m_start<=0;
   fetch_phase<=0;pw_en<=0;
  end else if(O_busy && (O_cycles>=BUDGET-1 || I_start)) begin
   state<=IDLE;O_busy<=0;O_done<=1;O_failed<=1;O_class<=0;O_vertices<=0;g_start<=0;m_start<=0;
   fetch_phase<=0;pw_en<=0;
  end else begin
   O_done<=0;g_start<=0;m_start<=0;pw_en<=0;
   if(O_busy) O_cycles<=O_cycles+1'b1;
   if(fetch_needed && fetch_phase!=4)begin
    fetch_phase<=fetch_phase+1'b1;
    if(fetch_phase==1)p0<=point_read;
    if(fetch_phase==2)p1<=point_read;
    if(fetch_phase==3)p2<=point_read;
   end else begin
   fetch_phase<=0;
   case(state)
    6'd0:if(I_start) begin
     O_busy<=1;O_failed<=0;O_class<=0;O_cycles<=0;result<=0;O_vertices<=0;
     area<=I_area;xmin<=I_xmin;xmax<=I_xmax;ymin<=I_ymin;ymax<=I_ymax;
     width<={1'b0,I_xmax}-{1'b0,I_xmin}+10'd1;
     height<={2'b0,I_ymax}-{2'b0,I_ymin}+10'd1;
     for(z=0;z<16;z=z+1) if(SERIAL_SUPPORT==0)begin
      sx[z]<=I_support_x[z*9+:9];sy[z]<=I_support_y[z*8+:8];
     end
     i<=1;copy_idx<=0;state<=LOAD_POINTS;
     if(BUDGET<1 || BUDGET>262143 || SERIAL_SUPPORT<0 || SERIAL_SUPPORT>1 || RECTANGLE_ONLY<0 || RECTANGLE_ONLY>1 || I_area==0 || I_area>57600 ||
      I_xmin>I_xmax || I_ymin>I_ymax || I_xmax>=320 || I_ymax>=180) begin
      O_failed<=1;state<=FINISH;
     end
    end
    6'd47:begin
     write_point({2'd0,copy_idx[3:0]},load_x,load_y);
     if(copy_idx==15)state<=SORT_LOAD;else copy_idx<=copy_idx+1'b1;
     if(load_x<xmin || load_x>xmax || load_y<ymin || load_y>ymax)begin O_failed<=1;state<=FINISH;end
    end
    6'd1:begin
     dimension<=(width>height)?width:height;
     box_area<=width[8:0]*height[7:0];
     if(xmin==0 || ymin==0 || xmax==319 || ymax==179 || width>160 || height>160) finish_unknown;
     else begin keyx<=p0x;keyy<=p0y;j<=i;state<=SORT_STEP;end
    end
    6'd2:begin
     if(j>0 && (p0x>keyx || (p0x==keyx && p0y>keyy))) begin
      write_point({2'd0,j[3:0]},p0x,p0y);j<=j-1'b1;
     end else begin
      write_point({2'd0,j[3:0]},keyx,keyy);
      if(i==15) begin i<=0;unique_n<=0;state<=UNIQUE;end
      else begin i<=i+1'b1;state<=SORT_LOAD;end
     end
    end
    6'd3:begin
     if(unique_n==0 || p0!=p1) begin
      write_point({2'd0,unique_n[3:0]},p0x,p0y);unique_n<=unique_n+1'b1;
     end
     if(i==15) state<=HULL_INIT;else i<=i+1'b1;
    end
    6'd4:begin
     n<=unique_n;i<=0;k<=0;upper<=0;state<=HULL_TEST;
     if(unique_n<3) finish_unknown;
    end
    6'd5:begin
     if((!upper && k>=2) || (upper && k>=upper_limit))
      geometry($signed({3'd0,p0x}),$signed({4'd0,p0y}),
       $signed({3'd0,p1x}),$signed({4'd0,p1y}),
       $signed({3'd0,p2x}),$signed({4'd0,p2y}),HULL_AFTER);
     else state<=HULL_PUSH;
    end
    6'd6:if(g_cross<=0) begin k<=k-1'b1;state<=HULL_TEST;end else state<=HULL_PUSH;
    6'd7:begin write_point(6'd16+{1'b0,k[4:0]},p0x,p0y);k<=k+1'b1;state<=HULL_NEXT;end
    6'd8:begin
     if(!upper && i+1==n) begin upper<=1;upper_limit<=k+1'b1;i<=n-2;state<=HULL_TEST;end
     else if(upper && i==0) begin
      if(k<4 || k>17) finish_unknown;
      else begin hull_n<=k-1'b1;q_n<=k-1'b1;copy_idx<=0;state<=COPY_HULL;end
     end else begin i<=upper?i-1'b1:i+1'b1;state<=HULL_TEST;end
    end
    6'd9:begin
     write_point(6'd48+{2'd0,copy_idx[3:0]},p0x,p0y);
     if(copy_idx+1==hull_n) state<=AREA_PREP;else copy_idx<=copy_idx+1'b1;
    end
    6'd10:begin idx<=0;sum_area<=0;state<=AREA_GO;end
    6'd11:geometry(0,0,$signed({3'd0,p0x}),$signed({4'd0,p0y}),
     $signed({3'd0,p1x}),$signed({4'd0,p1y}),AREA_ADD);
    6'd12:begin
     sum_area<=sum_area+$signed(g_cross);
     if(idx+1==q_n) state<=AREA_FINISH;else begin idx<=idx+1'b1;state<=AREA_GO;end
    end
    6'd13:begin
     center_area2<=sum_area;
     cell_area2<=sum_area+(({22'd0,width}+{22'd0,height}-32'd1)<<1);
     if(sum_area<=0)finish_unknown;else multiply({48'd0,area},64'd200,SOL_AREA);
    end
    6'd49:begin lhs<=product;multiply({32'd0,cell_area2},64'd90,SOL_LOW);end
    6'd50:if(lhs<product)finish_unknown;else multiply({32'd0,cell_area2},64'd112,SOL_HIGH);
    6'd51:if(lhs>product)finish_unknown;else state<=SIM_INIT;
    6'd14:begin idx<=0;best_valid<=0;state<=SIM_GO;end
    6'd15:geometry($signed({3'd0,p0x}),$signed({4'd0,p0y}),
     $signed({3'd0,p1x}),$signed({4'd0,p1y}),
     $signed({3'd0,p2x}),$signed({4'd0,p2y}),SIM_GOT);
    6'd16:begin ll<=g_length;multiply({36'd0,abs_cross},{36'd0,abs_cross},SIM_SQUARE);end
    6'd17:begin
     cc<=product;
     if(ll==0) state<=SIM_NEXT;
     else if(!best_valid) begin best_valid<=1;best_cc<=product;best_ll<=ll;best_idx<=idx;state<=SIM_NEXT;end
     else multiply(product,{32'd0,best_ll},SIM_LHS);
    end
    6'd18:begin lhs<=product;multiply({16'd0,best_cc},{32'd0,ll},SIM_RHS);end
    6'd19:begin
     if(lhs<product) begin best_cc<=cc;best_ll<=ll;best_idx<=idx;end
     state<=SIM_NEXT;
    end
    6'd20:begin
     if(idx+1==q_n) state<=SIM_DECIDE;else begin idx<=idx+1'b1;state<=SIM_GO;end
    end
    6'd21:begin
     if(q_n<=3 || !best_valid) state<=REF_INIT;
     else multiply({54'd0,eps_num},{54'd0,eps_num},SIM_EPS);
    end
    6'd22:begin multiply(product,{32'd0,best_ll},SIM_REMOVE);remove_idx<=best_idx;end
    6'd23:begin
     if(({16'd0,best_cc}<<10)>product) state<=REF_INIT;
     else begin copy_idx<=remove_idx;state<=SHIFT_POINT;end
    end
    6'd48:begin
     if(copy_idx+1<q_n)begin
      write_point(6'd48+{2'd0,copy_idx[3:0]},p0x,p0y);copy_idx<=copy_idx+1'b1;
     end else begin q_n<=q_n-1'b1;state<=SIM_INIT;end
    end
    6'd41:begin
     idx<=0;refine_round<=0;
     if(RECTANGLE_ONLY!=0 && q_n!=4)finish_unknown;
     else if(q_n==3 || q_n==4) state<=REF_CURRENT;else state<=POLY_INIT;
    end
    6'd42:geometry($signed({3'd0,p0x}),$signed({4'd0,p0y}),
     $signed({3'd0,p1x}),$signed({4'd0,p1y}),
     $signed({3'd0,p2x}),$signed({4'd0,p2y}),REF_CURRENT_SAVE);
    6'd43:begin
     best_corner<=g_cross;refine_x<=p0x;refine_y<=p0y;candidate_idx<=0;state<=REF_CAND;
    end
    6'd44:geometry($signed({3'd0,p0x}),$signed({4'd0,p0y}),
     $signed({3'd0,p1x}),$signed({4'd0,p1y}),
     $signed({3'd0,p2x}),$signed({4'd0,p2y}),REF_SAVE);
    6'd45:begin
     if(g_cross>best_corner)begin best_corner<=g_cross;refine_x<=p0x;refine_y<=p0y;end
     if(candidate_idx+1==hull_n)state<=REF_NEXT;
     else begin candidate_idx<=candidate_idx+1'b1;state<=REF_CAND;end
    end
    6'd46:begin
     write_point(6'd48+{2'd0,idx[3:0]},refine_x,refine_y);
     if(idx+1==q_n)begin
      if(refine_round==2)state<=POLY_INIT;
      else begin refine_round<=refine_round+1'b1;idx<=0;state<=REF_CURRENT;end
     end else begin idx<=idx+1'b1;state<=REF_CURRENT;end
    end
    6'd24:begin O_vertices<=q_n[4:0];idx<=0;sum_area<=0;state<=POLY_GO;end
    6'd25:geometry(0,0,$signed({3'd0,p0x}),$signed({4'd0,p0y}),
     $signed({3'd0,p1x}),$signed({4'd0,p1y}),POLY_ADD);
    6'd26:begin
     sum_area<=sum_area+$signed(g_cross);
     if(idx+1==q_n) state<=POLY_DONE;else begin idx<=idx+1'b1;state<=POLY_GO;end
    end
    6'd27:begin
     poly_area2<=sum_area;
     if(q_n==3 || q_n==4)multiply({27'd0,$unsigned(sum_area)},48'd100,POLY_LHS);
     else state<=CIRCLE_INIT;
    end
    6'd52:begin lhs<=product;multiply({32'd0,center_area2},64'd94,POLY_COMPARE);end
    6'd53:if(lhs>=product || poly_area2+(({22'd0,width}+{22'd0,height})<<1)>=center_area2)begin
     idx<=0;min_edge<=16'hffff;max_edge<=0;state<=EDGE_GO;
    end else state<=CIRCLE_INIT;
    6'd28:geometry($signed({3'd0,p0x}),$signed({4'd0,p0y}),0,0,
     $signed({3'd0,p1x}),$signed({4'd0,p1y}),EDGE_SAVE);
    6'd29:begin
     write_point(7'd64+{3'd0,idx},{1'b0,g_length[15:8]},g_length[7:0]);
     if(g_length<min_edge) min_edge<=g_length;if(g_length>max_edge) max_edge<=g_length;
     if(idx+1==q_n) begin idx<=0;state<=RECT_GO;end else begin idx<=idx+1'b1;state<=EDGE_GO;end
    end
    6'd30:begin
     if(q_n==3) begin
      if(RECTANGLE_ONLY!=0)finish_unknown;
      else if(({4'd0,min_edge}<<4)>={4'd0,max_edge}) begin result<=2;state<=FINISH;end
      else state<=CIRCLE_INIT;
     end else geometry($signed({3'd0,p0x}),$signed({4'd0,p0y}),
      $signed({3'd0,p1x}),$signed({4'd0,p1y}),
      $signed({3'd0,p2x}),$signed({4'd0,p2y}),RECT_SQUARE);
    end
    6'd31:multiply({36'd0,abs_dot},{36'd0,abs_dot},RECT_LHS);
    6'd32:multiply(product,64'd100,RECT_CONST);
    6'd54:begin lhs<=product;multiply({32'd0,p0[15:0]},{32'd0,p1[15:0]},RECT_RHS);end
    6'd33:begin
     if(lhs>(product<<1)) state<=CIRCLE_INIT;
     else if(idx==3) begin result<=3;state<=FINISH;end
     else begin idx<=idx+1'b1;state<=RECT_GO;end
    end
    6'd34:begin
     if(RECTANGLE_ONLY!=0 || q_n<7)finish_unknown;else multiply({54'd0,dimension},64'd100,ROUND_SIZE);
    end
    6'd55:if(RECTANGLE_ONLY==0)begin lhs<=product;multiply((width<height)?{54'd0,width}:{54'd0,height},64'd110,ROUND_ASPECT);end
    6'd56:if(RECTANGLE_ONLY==0)begin if(lhs>product)finish_unknown;else multiply({48'd0,area},64'd100,ROUND_AREA);end
    6'd57:if(RECTANGLE_ONLY==0)begin lhs<=product;multiply({44'd0,box_area},64'd73,ROUND_MIN);end
    6'd58:if(RECTANGLE_ONLY==0)begin if(lhs<product)finish_unknown;else multiply({44'd0,box_area},64'd84,ROUND_MAX);end
    6'd59:if(RECTANGLE_ONLY==0)begin if(lhs>product)finish_unknown;else begin idx<=0;min_radius<=16'hffff;max_radius<=0;state<=CIRCLE_GO;end end
    6'd35:if(RECTANGLE_ONLY==0)geometry(0,0,0,0,
     $signed({2'd0,p0x,1'b0})-$signed({3'd0,xmin})-$signed({3'd0,xmax}),
     $signed({3'd0,p0y,1'b0})-$signed({4'd0,ymin})-$signed({4'd0,ymax}),CIRCLE_SAVE);
    6'd36:if(RECTANGLE_ONLY==0)begin
     if(g_length<min_radius) min_radius<=g_length;if(g_length>max_radius) max_radius<=g_length;
     // P[0:n) is the sorted unique set of the original support points.
     // Sorting/deduplication cannot change min/max radius (no averaging).
     if(idx+1==n) state<=CIRCLE_DONE;else begin idx<=idx+1'b1;state<=CIRCLE_GO;end
    end
    6'd37:if(RECTANGLE_ONLY==0)multiply({36'd0,max_radius},64'd100,ROUND_MAXR);
    6'd60:if(RECTANGLE_ONLY==0)begin lhs<=product;multiply({36'd0,min_radius},64'd125,ROUND_MINR);end
    6'd61:if(RECTANGLE_ONLY==0)begin if(lhs<=product)result<=1;state<=FINISH;end
    6'd39:if(g_done) state<=g_return;
    6'd40:if(m_done) state<=m_return;
    6'd38:begin O_class<=result;O_done<=1;O_busy<=0;state<=IDLE;end
    default:begin O_failed<=1;result<=0;state<=FINISH;end
   endcase
   end
  end
 end
endmodule
