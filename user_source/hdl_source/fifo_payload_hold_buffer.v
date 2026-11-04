// Two legal registered-payload values (0/1), plus simulation X, retain identity.
// Official hard PH1P_LOGIC_SLICE combinational identity; no clocked stage.
// Internal registered payload has no legal Z. Vendor hard input resolves Z high.
// Functional identity applies after vendor configuration startup, before FIFO reset release.
module fifo_payload_hold_buffer #(parameter WIDTH=1, parameter STAGES=0, parameter [WIDTH-1:0] BUFFER_MASK={WIDTH{1'b1}})(
 input wire [WIDTH-1:0] I_data, output wire [WIDTH-1:0] O_data
);
 wire [WIDTH-1:0] level[0:STAGES];
 assign level[0]=I_data;
 genvar s,b;
 generate for(s=0;s<STAGES;s=s+1)begin: stage
  for(b=0;b<WIDTH;b=b+1)begin: bit_buffer
   if(BUFFER_MASK[b])begin: buffered
    PH1P_LOGIC_SLICE #(.INIT_LUT5A(32'hAAAAAAAA),.INIT_LUT5B(32'h00000000),
     .LUTMODE("DLUT5"),.OFA_SD("LUT5A"),.GSR("DISABLE"),
     .CEAMUX("0"),.CEBMUX("0"),.CLKMUX("0")) identity(
     .ia(level[s][b]),.ib(1'b0),.ic(1'b0),.id(1'b0),.ie(1'b0),
     .imf(1'b0),.ima(1'b0),.imb(1'b0),.clk(1'b0),.cea(1'b0),.ceb(1'b0),
     .ssr(1'b1),.asr(1'b1),.fci(),
     .ofa(level[s+1][b]),.ofb(),.oqa(),.oqb(),.fco());
   end else begin: bypass
    assign level[s+1][b]=level[s][b];
   end
  end
 end endgenerate
 assign O_data=level[STAGES];
endmodule
