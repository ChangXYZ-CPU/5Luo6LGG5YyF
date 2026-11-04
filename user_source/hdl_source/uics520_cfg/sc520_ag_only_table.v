`timescale 1ns / 1ps
// sc520_ag_only_table -- gain only, its own I2C transaction.
//
// Datasheet 2.2.3: 0x3e09 analog gain, 0x3e06 digital coarse, 0x3e07 digital fine (0x80 = 1x).
// Three registers and nothing else: no exposure, no group hold.
module sc520_ag_only_table (
    input  wire [8:0]  REG_INDEX,
    output reg  [23:0] REG_DATA,
    output wire [8:0]  REG_SIZE,
    input  wire [23:0] I_AG
);
    assign REG_SIZE = 9'd3;

    always @(*) begin
        case (REG_INDEX)
            9'd0:    REG_DATA = {16'h3e09, I_AG[23:16]};
            9'd1:    REG_DATA = {16'h3e06, I_AG[15:8]};
            9'd2:    REG_DATA = {16'h3e07, I_AG[7:0]};
            default: REG_DATA = {16'h3e09, 8'h00};
        endcase
    end
endmodule
