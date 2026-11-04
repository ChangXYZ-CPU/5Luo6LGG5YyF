// MSB-first byte stream. Bubbles retain partial words; SOF may precede valid.
module data_96bit_to_128bit (
 input wire I_clk, I_rst_n, I_96b_frame_start, I_96b_valid,
 input wire [95:0] I_96b_data,
 output reg O_128b_frame_start, O_128b_valid,
 output reg [127:0] O_128b_data
);
 reg [95:0] prev;
 reg [1:0] phase;
 reg frame_pending;
 wire [1:0] use_phase = I_96b_frame_start ? 2'd0 : phase;
 always @(posedge I_clk or negedge I_rst_n) begin
  if (!I_rst_n) begin
   prev<=0; phase<=0; frame_pending<=0;
   O_128b_frame_start<=0; O_128b_valid<=0; O_128b_data<=0;
  end else begin
   O_128b_frame_start<=0; O_128b_valid<=0;
   if (I_96b_frame_start) begin phase<=0; frame_pending<=1; end
   if (I_96b_valid) begin
    prev<=I_96b_data; phase<=use_phase+1'b1;
    case (use_phase)
     0: ;
     1: begin
      O_128b_data<={prev,I_96b_data[95:64]}; O_128b_valid<=1;
      O_128b_frame_start<=frame_pending; frame_pending<=0;
     end
     2: begin O_128b_data<={prev[63:0],I_96b_data[95:32]}; O_128b_valid<=1; end
     3: begin O_128b_data<={prev[31:0],I_96b_data}; O_128b_valid<=1; end
    endcase
   end
  end
 end
endmodule
