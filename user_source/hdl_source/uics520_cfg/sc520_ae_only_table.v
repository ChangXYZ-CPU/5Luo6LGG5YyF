`timescale 1ns / 1ps
// sc520_ae_only_table -- exposure only, its own I2C transaction.
//
// Datasheet 2.2.2: exposure = {0x3e00[3:0], 0x3e01[7:0], 0x3e02[7:4]}, unit = 1/2 line,
// min 4, max 2*VTS-9. Three registers and nothing else: no gain, no group hold.
// This exists to test whether writing the exposure inside the same transaction as the gain
// (which demonstrably takes effect) is what stops the exposure from applying.
module sc520_ae_only_table (
    input  wire [8:0]  REG_INDEX,
    output reg  [23:0] REG_DATA,
    output wire [8:0]  REG_SIZE,
    input  wire [15:0] I_AE
);
    assign REG_SIZE = 9'd3;

    always @(*) begin
        case (REG_INDEX)
            9'd0:    REG_DATA = {16'h3e00, 4'h0, I_AE[15:12]};
            9'd1:    REG_DATA = {16'h3e01, I_AE[11:4]};
            9'd2:    REG_DATA = {16'h3e02, I_AE[3:0], 4'h0};
            default: REG_DATA = {16'h3e00, 8'h00};
        endcase
    end
endmodule
