`timescale 1ns / 1ps
// sc520_ae_nogrp_table -- experiment A, without touching the vendor file.
//
// The vendor's uics520regAE writes the same six registers but wraps them in a group hold
// (0x3812 = 0x00 to start, 0x30 to release). On hardware the gain registers take effect while
// the exposure registers do not, and the packing has been proven correct (tb_ae_regs), so the
// group-hold apply mechanism is the prime suspect.
//
// This table writes exactly the same six registers and nothing else:
//   0: 0x3e00 <- {0, I_AE[15:12]}      exposure = {0x3e00[3:0],0x3e01[7:0],0x3e02[7:4]}
//   1: 0x3e01 <- I_AE[11:4]            unit = 1/2 line, min 4, max 2*VTS-9
//   2: 0x3e02 <- {I_AE[3:0], 0}
//   3: 0x3e09 <- I_AG[23:16]           analog gain
//   4: 0x3e06 <- I_AG[15:8]            digital coarse
//   5: 0x3e07 <- I_AG[7:0]             digital fine (0x80 = 1x)
// REG_SIZE = 6 and there is no 0x3812 write at all.
module sc520_ae_nogrp_table (
    input  wire [8:0]  REG_INDEX,
    output reg  [23:0] REG_DATA,
    output wire [8:0]  REG_SIZE,
    input  wire [15:0] I_AE,
    input  wire [23:0] I_AG
);
    assign REG_SIZE = 9'd6;

    always @(*) begin
        case (REG_INDEX)
            9'd0:    REG_DATA = {16'h3e00, 4'h0, I_AE[15:12]};
            9'd1:    REG_DATA = {16'h3e01, I_AE[11:4]};
            9'd2:    REG_DATA = {16'h3e02, I_AE[3:0], 4'h0};
            9'd3:    REG_DATA = {16'h3e09, I_AG[23:16]};
            9'd4:    REG_DATA = {16'h3e06, I_AG[15:8]};
            9'd5:    REG_DATA = {16'h3e07, I_AG[7:0]};
            default: REG_DATA = {16'h3e00, 8'h00};
        endcase
    end
endmodule
