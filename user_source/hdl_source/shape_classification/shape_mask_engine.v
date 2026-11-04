// One owned 320x180 bit bank, destructively consumed. Memory read latency=1.
// Full-rate video capture/publication belongs to shape_snapshot (separate).
module shape_mask_engine #(
 parameter integer BUDGET=1100000,QUEUE_LIMIT=1024,COMPONENT_LIMIT=1024
)(
 input wire I_clk,I_rst_n,I_cancel,I_start,
 output wire O_mem_re,O_mem_we,
 output wire [15:0] O_mem_addr,
 input wire I_mem_bit,
 output reg O_busy,O_done,O_failed,O_target_valid,
 output reg [2:0] O_error,
 output reg [1:0] O_class,
 output reg [8:0] O_xmin,O_xmax,
 output reg [7:0] O_ymin,O_ymax,
 output reg [15:0] O_area,
 output reg [20:0] O_cycles,
 output reg [10:0] O_queue_peak,O_components
);
 localparam [27:0] IDLE=28'b1<<0,
  SCAN=28'b1<<1,
  SCAN_GOT=28'b1<<2,
  NEW_COMPONENT=28'b1<<3,
  LEFT=28'b1<<4,
  LEFT_GOT=28'b1<<5,
  RIGHT=28'b1<<6,
  RIGHT_GOT=28'b1<<7,
  CLAIM=28'b1<<8,
  CLEAR=28'b1<<9,
  ENQUEUE=28'b1<<10,
  POP=28'b1<<11,
  POP_WAIT=28'b1<<12,
  POP_GOT=28'b1<<13,
  NEIGH_INIT=28'b1<<14,
  NEIGH_SCAN=28'b1<<15,
  NEIGH_GOT=28'b1<<16,
  NEIGH_NEXT=28'b1<<17,
  COMPONENT_DONE=28'b1<<18,
  NEXT_SCAN=28'b1<<19,
  CLASS_START=28'b1<<20,
  CLASS_WAIT=28'b1<<21,
  FINISH=28'b1<<22,
  READ=28'b1<<23,
  READ_WAIT=28'b1<<24,
  SUPPORT_INIT=28'b1<<25,
  SUPPORT_UPDATE=28'b1<<26,
  WINNER_COPY=28'b1<<27;
 reg [27:0] state,read_return;
 reg [15:0] cursor;
 reg [8:0] read_x;reg [7:0] read_y;
 reg [8:0] cursor_x,l,r,clear_x,parent_l,parent_r,neighbor_x,neighbor_end;
 reg [7:0] cursor_y,row,parent_y,neighbor_y;
 reg lower_neighbor,from_neighbor;
 reg [25:0] queue[0:1023];
 reg [25:0] queue_data;
 reg [9:0] queue_rd,queue_wr;
 reg [10:0] queue_count;
 reg [15:0] current_area;
 reg [8:0] current_xmin,current_xmax;
 reg [7:0] current_ymin,current_ymax;
 // Serialized support updates share one score datapath and one small LUT RAM.
 // Keep the winner descriptor latched until the entire frame scan finishes.
 // TD auto-infers this asynchronous 32-entry table as LUT RAM. Do not
 // use a non-TD RAM_STYLE spelling: final area must confirm the mapping.
 // Current component [0:15], best descriptor [16:31]. One read port
 // is sufficient: classification starts only after all components finish.
 reg [28:0] supports[0:31];
 reg [3:0] support_idx;
 // Three bounded phases cut asynchronous RAM -> lexicographic compare ->
 // RAM write into registered paths. No RGB delay or threshold change.
 reg [1:0] support_phase;
 reg [28:0] support_old,support_new;
 reg support_score_gt,support_score_eq,support_x_lt,support_x_eq,support_y_lt;
 wire [3:0] point_index;
 wire [16:0] point_data;
 reg classifier_start;
 wire classifier_busy,classifier_done,classifier_failed;
 wire [1:0] classifier_class;
 wire [4:0] classifier_vertices;
 wire [17:0] classifier_cycles;
 wire abort=I_cancel || (O_busy && (O_cycles>=BUDGET-1 || I_start));
 assign O_mem_re=O_busy&&!abort&&state[23];
 assign O_mem_we=O_busy&&!abort&&state[9];
 wire [8:0] address_x=state[9]?clear_x:read_x;
 wire [7:0] address_y=state[9]?row:read_y;
 assign O_mem_addr=({8'd0,address_y}<<8)+({8'd0,address_y}<<6)+{7'd0,address_x};
 // Winner is written only in WINNER_COPY, never during CLASS_START/WAIT.
 // One asynchronous RAM read replaces two parallel 272-bit register copies.
 shape_classifier #(.SERIAL_SUPPORT(1)) classifier(.I_clk(I_clk),.I_rst_n(I_rst_n),.I_cancel(abort),.I_start(classifier_start),
  .I_area(O_area),.I_xmin(O_xmin),.I_xmax(O_xmax),.I_ymin(O_ymin),.I_ymax(O_ymax),
  .I_support_x(144'd0),.I_support_y(128'd0),.O_busy(classifier_busy),.O_done(classifier_done),
  .O_failed(classifier_failed),.O_class(classifier_class),.O_vertices(classifier_vertices),.O_cycles(classifier_cycles),
  .I_point(point_data),.O_point_index(point_index));
 function signed [11:0] score;
  input [3:0] d;input [8:0] x;input [7:0] y;
  reg signed [11:0] xx,yy;
  begin
   xx=$signed({3'd0,x});yy=$signed({4'd0,y});
   if(d[2:0]==1 || d[2:0]==7)xx=xx<<<1;
   if(d[2:0]==3 || d[2:0]==5)yy=yy<<<1;
   if(d==4 || d==12)xx=0;
   else if(d>=5 && d<=12)xx=-xx;
   if(d==0 || d==8)yy=0;
   else if(d>=9)yy=-yy;
   score=xx+yy;
  end
 endfunction
 wire [8:0] selected_x=endpoint(support_idx,l,r);
 wire signed [11:0] selected_score=score(support_idx,selected_x,row);
 wire [4:0] support_read_addr=(state[20]||state[21])?{1'b1,point_index}:{1'b0,support_idx};
 wire [28:0] support_entry=supports[support_read_addr];
 assign point_data=support_entry[16:0];
 wire replace_support=support_score_gt || (support_score_eq &&
  (support_x_lt || (support_x_eq && support_y_lt)));
 always @(posedge I_clk) if(O_busy && !abort)begin
  if(state[25])supports[{1'b0,support_idx}]<={-12'sd2048,17'd0};
  else if(state[26] && support_phase==2 && replace_support)supports[{1'b0,support_idx}]<=support_new;
  else if(state[27])supports[{1'b1,support_idx}]<={12'd0,support_entry[16:0]};
 end
 function [8:0] endpoint;
  input integer d;input [8:0] a,b;
  begin endpoint=(d<=3 || d>=13)?b:a;end
 endfunction
 task read_pixel;
  input [8:0] x;input [7:0] y;input [27:0] ret;
  begin read_x<=x;read_y<=y;read_return<=ret;state<=READ;end
 endtask
 task fail_job;
  input [2:0] code;
  begin O_failed<=1;O_error<=code;O_target_valid<=0;O_class<=0;O_area<=0;
   O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;state<=FINISH;end
 endtask
 integer z;
 always @(posedge I_clk) if(O_busy&&!abort&&state[11]&&queue_count!=0)queue_data<=queue[queue_rd];
 always @(posedge I_clk) if(O_busy&&!abort&&state[10]&&queue_count<QUEUE_LIMIT)
  queue[queue_wr]<={l,r,row};
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   state<=IDLE;read_return<=0;read_x<=0;read_y<=0;cursor<=0;cursor_x<=0;cursor_y<=0;
   l<=0;r<=0;clear_x<=0;row<=0;parent_l<=0;parent_r<=0;parent_y<=0;
   neighbor_x<=0;neighbor_end<=0;neighbor_y<=0;lower_neighbor<=0;from_neighbor<=0;
   queue_rd<=0;queue_wr<=0;queue_count<=0;current_area<=0;
   current_xmin<=0;current_xmax<=0;current_ymin<=0;current_ymax<=0;classifier_start<=0;
   support_idx<=0;support_phase<=0;support_old<=0;support_new<=0;
   support_score_gt<=0;support_score_eq<=0;support_x_lt<=0;support_x_eq<=0;support_y_lt<=0;
   O_busy<=0;O_done<=0;O_failed<=0;O_error<=0;O_target_valid<=0;O_class<=0;
   O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;O_area<=0;O_cycles<=0;O_queue_peak<=0;O_components<=0;
  end else if(I_cancel) begin
   state<=IDLE;O_busy<=0;O_done<=0;O_failed<=0;O_error<=0;O_target_valid<=0;
   O_class<=0;O_area<=0;O_cycles<=0;classifier_start<=0;queue_count<=0;support_phase<=0;
   O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;
  end else if(O_busy&&(O_cycles>=BUDGET-1||I_start))begin
   O_busy<=0;O_done<=1;O_failed<=1;O_error<=1;O_target_valid<=0;O_class<=0;O_area<=0;
   O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;state<=IDLE;classifier_start<=0;
  end else begin
   O_done<=0;classifier_start<=0;if(O_busy)O_cycles<=O_cycles+1'b1;
   case(1'b1)
    state[0]:if(I_start) begin
     O_busy<=1;O_failed<=0;O_error<=0;O_target_valid<=0;O_class<=0;O_area<=0;
     O_cycles<=0;O_queue_peak<=0;O_components<=0;cursor<=0;cursor_x<=0;cursor_y<=0;
     O_xmin<=0;O_xmax<=0;O_ymin<=0;O_ymax<=0;
     queue_rd<=0;queue_wr<=0;queue_count<=0;state<=SCAN;
     if(BUDGET<1||BUDGET>2097151||QUEUE_LIMIT<1||QUEUE_LIMIT>1024||COMPONENT_LIMIT<1||COMPONENT_LIMIT>1024)fail_job(4);
    end
    state[1]:read_pixel(cursor_x,cursor_y,SCAN_GOT);
    state[2]:if(I_mem_bit) begin
     if(O_components>=COMPONENT_LIMIT)fail_job(3);
     else begin O_components<=O_components+1'b1;state<=NEW_COMPONENT;end
    end else state<=NEXT_SCAN;
    state[3]:begin
     current_area<=0;current_xmin<=319;current_xmax<=0;current_ymin<=179;current_ymax<=0;
     queue_count<=0;queue_rd<=0;queue_wr<=0;l<=cursor_x;r<=cursor_x;row<=cursor_y;from_neighbor<=0;
     support_idx<=0;state<=SUPPORT_INIT;
    end
    state[25]:if(support_idx==15)state<=LEFT;else support_idx<=support_idx+1'b1;
    state[4]:if(l==0)state<=RIGHT;else read_pixel(l-1'b1,row,LEFT_GOT);
    state[5]:if(I_mem_bit)begin l<=l-1'b1;state<=LEFT;end else state<=RIGHT;
    state[6]:if(r==319)state<=CLAIM;else read_pixel(r+1'b1,row,RIGHT_GOT);
    state[7]:if(I_mem_bit)begin r<=r+1'b1;state<=RIGHT;end else state<=CLAIM;
    state[8]:begin
     if(queue_count>=QUEUE_LIMIT)fail_job(2);
     else begin clear_x<=l;state<=CLEAR;end
    end
    state[9]:if(clear_x==r)state<=ENQUEUE;else clear_x<=clear_x+1'b1;
    state[10]:begin
     queue_wr<=queue_wr+1'b1;queue_count<=queue_count+1'b1;
     if(queue_count+1'b1>O_queue_peak)O_queue_peak<=queue_count+1'b1;
     current_area<=current_area+{7'd0,r}-{7'd0,l}+16'd1;
     if(l<current_xmin)current_xmin<=l;if(r>current_xmax)current_xmax<=r;
     if(row<current_ymin)current_ymin<=row;if(row>current_ymax)current_ymax<=row;
     support_idx<=0;support_phase<=0;state<=SUPPORT_UPDATE;
    end
    state[26]:case(support_phase)
     0:begin
      support_old<=support_entry;support_new<={selected_score,selected_x,row};support_phase<=1;
     end
     1:begin
      support_score_gt<=$signed(support_new[28:17])>$signed(support_old[28:17]);
      support_score_eq<=support_new[28:17]==support_old[28:17];
      support_x_lt<=support_new[16:8]<support_old[16:8];
      support_x_eq<=support_new[16:8]==support_old[16:8];
      support_y_lt<=support_new[7:0]<support_old[7:0];
      support_phase<=2;
     end
     2:begin
      support_phase<=0;
      if(support_idx!=15)support_idx<=support_idx+1'b1;
      else if(from_neighbor)begin
       if(r>=neighbor_end)state<=NEIGH_NEXT;
       else begin neighbor_x<=r+1'b1;state<=NEIGH_SCAN;end
      end else state<=POP;
     end
     default:fail_job(4);
    endcase
    state[11]:if(queue_count==0)state<=COMPONENT_DONE;
     else begin queue_count<=queue_count-1'b1;queue_rd<=queue_rd+1'b1;state<=POP_WAIT;end
    state[12]:state<=POP_GOT;
    state[13]:begin
     parent_l<=queue_data[25:17];parent_r<=queue_data[16:8];parent_y<=queue_data[7:0];
     lower_neighbor<=0;state<=NEIGH_INIT;
    end
    state[14]:begin
     neighbor_x<=(parent_l==0)?9'd0:parent_l-1'b1;
     neighbor_end<=(parent_r==319)?9'd319:parent_r+1'b1;
     if(!lower_neighbor&&parent_y==0)state<=NEIGH_NEXT;
     else if(lower_neighbor&&parent_y==179)state<=POP;
     else begin neighbor_y<=lower_neighbor?parent_y+1'b1:parent_y-1'b1;state<=NEIGH_SCAN;end
    end
    state[15]:read_pixel(neighbor_x,neighbor_y,NEIGH_GOT);
    state[16]:if(I_mem_bit)begin
     l<=neighbor_x;r<=neighbor_x;row<=neighbor_y;from_neighbor<=1;state<=LEFT;
    end else if(neighbor_x>=neighbor_end)state<=NEIGH_NEXT;
     else begin neighbor_x<=neighbor_x+1'b1;state<=NEIGH_SCAN;end
    state[17]:if(!lower_neighbor)begin lower_neighbor<=1;state<=NEIGH_INIT;end else state<=POP;
    state[18]:begin
     if(current_area>=384 && ({1'b0,current_xmax}-{1'b0,current_xmin}+10'd1)>=32 &&
       ({2'd0,current_ymax}-{2'd0,current_ymin}+10'd1)>=32 &&
       (!O_target_valid || current_area>O_area || (current_area==O_area &&
        (current_ymin<O_ymin || (current_ymin==O_ymin&&current_xmin<O_xmin))))) begin
       O_target_valid<=1;O_area<=current_area;O_xmin<=current_xmin;O_xmax<=current_xmax;
       O_ymin<=current_ymin;O_ymax<=current_ymax;
       support_idx<=0;state<=WINNER_COPY;
     end else state<=NEXT_SCAN;
    end
    state[27]:begin
     if(support_idx==15)state<=NEXT_SCAN;
     else support_idx<=support_idx+1'b1;
    end
    state[19]:if(cursor==57599)begin
     if(O_target_valid)state<=CLASS_START;else state<=FINISH;
    end else begin
     cursor<=cursor+1'b1;
     if(cursor_x==319)begin cursor_x<=0;cursor_y<=cursor_y+1'b1;end else cursor_x<=cursor_x+1'b1;
     state<=SCAN;
    end
    state[20]:begin classifier_start<=1;state<=CLASS_WAIT;end
    state[21]:if(classifier_done)begin
     if(classifier_failed)fail_job(5);else begin O_class<=classifier_class;state<=FINISH;end
    end
    state[23]:state<=READ_WAIT;
    state[24]:state<=read_return;
    state[22]:begin O_done<=1;O_busy<=0;state<=IDLE;end
    default:fail_job(4);
   endcase
  end
 end
endmodule
