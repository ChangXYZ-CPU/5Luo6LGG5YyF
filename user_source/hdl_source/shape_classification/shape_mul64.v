// Unsigned low-WIDTH product; positive WIDTH/A_WIDTH/B_WIDTH required.
// Default API is 64-bit. Classifier uses 32x16 ->48, with a proved bound.
// USE_DSP=0 (default): shift/add, completes with remaining multiplier <=1.
// For 32x16 ->48: USE_DSP=1 shares one 16x16 tile; USE_DSP=2 uses a full
// registered multiply. Other width combinations use the shift/add fallback.
module shape_mul64 #(parameter integer WIDTH=64,A_WIDTH=WIDTH,B_WIDTH=WIDTH,USE_DSP=0)(
 input wire I_clk,I_rst_n,I_cancel,I_start,
 input wire [A_WIDTH-1:0] I_a,
 input wire [B_WIDTH-1:0] I_b,
 output reg O_busy,O_done,
 output reg [WIDTH-1:0] O_product
);
 generate if(USE_DSP==2 && WIDTH==48 && A_WIDTH==32 && B_WIDTH==16)begin:full_dsp
  reg [31:0] saved_a;reg [15:0] saved_b;
  always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin O_busy<=0;O_done<=0;O_product<=0;saved_a<=0;saved_b<=0;end
   else if(I_cancel)begin O_busy<=0;O_done<=0;O_product<=0;end
   else begin
    O_done<=0;
    if(!O_busy)begin
     if(I_start)begin saved_a<=I_a;saved_b<=I_b;O_busy<=1;end
    end else begin O_product<=saved_a*saved_b;O_busy<=0;O_done<=1;end
   end
  end
 end else if(USE_DSP==1 && WIDTH==48 && A_WIDTH==32 && B_WIDTH==16)begin:split_dsp
  // Two time-multiplexed unsigned 16x16 tiles use one DSP. The low
  // 16 bits need no carry propagation; only the upper 32 bits are added.
  reg [31:0] saved_a,partial,low_product;
  reg [15:0] saved_b;
  reg [1:0] phase;
  wire [15:0] tile_a=phase==0?saved_a[15:0]:saved_a[31:16];
  always @(posedge I_clk)if(I_rst_n && !I_cancel && O_busy && phase<2)
   partial<=tile_a*saved_b;
  always @(posedge I_clk or negedge I_rst_n)begin
   if(!I_rst_n)begin O_busy<=0;O_done<=0;O_product<=0;phase<=0;saved_a<=0;saved_b<=0;low_product<=0;end
   else if(I_cancel)begin O_busy<=0;O_done<=0;O_product<=0;phase<=0;end
   else begin
    O_done<=0;
    if(!O_busy)begin
     if(I_start)begin O_busy<=1;phase<=0;saved_a<=I_a;saved_b<=I_b;end
    end else case(phase)
     0:phase<=1;
     1:begin low_product<=partial;phase<=2;end
     2:begin O_product<={partial+{16'd0,low_product[31:16]},low_product[15:0]};O_busy<=0;O_done<=1;end
     default:begin O_busy<=0;O_done<=0;phase<=0;end
    endcase
   end
  end
 end else begin:shift_add
 reg [WIDTH-1:0] addend,accumulator;
 reg [B_WIDTH-1:0] multiplier;
 wire [WIDTH-1:0] next_acc=accumulator+(multiplier[0]?addend:{WIDTH{1'b0}});
 always @(posedge I_clk or negedge I_rst_n) begin
  if(!I_rst_n) begin
   O_busy<=0;O_done<=0;O_product<=0;addend<=0;multiplier<=0;accumulator<=0;
  end else if(I_cancel) begin
   O_busy<=0;O_done<=0;O_product<=0;addend<=0;multiplier<=0;accumulator<=0;
  end else begin
   O_done<=0;
   if(!O_busy) begin
    if(I_start) begin O_busy<=1;addend<=I_a;multiplier<=I_b;accumulator<=0;end
   end else if(multiplier<=1) begin
    O_product<=next_acc;O_busy<=0;O_done<=1;
   end else begin
    accumulator<=next_acc;addend<=addend<<1;multiplier<=multiplier>>1;
   end
  end
 end
 end endgenerate
endmodule
