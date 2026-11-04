// ============================================================================
// mcu_ahb_probe.v —— v2：自报地址的 AHB-Lite 从机
//
// 目的：确定 MCU 的 AHB 接口上 haddr 的真实编码
//       （字节地址？字地址靠右移？）
//
// 行为：
//   hready 恒 1，hresp 恒 OKAY
//   任何"写"传输：抓 haddr -> dbg_addr，抓 hwdata -> dbg_wdata，计数
//   任何"读"传输：返回 {1'b0, dbg_addr}（即上次写操作时总线上的地址）
//                  以及低 8 位附带上上次写数据的低 8 位，便于交叉确认：
//                  返回值 = {dbg_addr[30:8], dbg_wdata[7:0]}
//
// 判读：
//   写 0x70000000 后读回 0x700000XX  -> haddr 是字节地址
//   写 0x70000000 后读回 0x1C0000XX  -> haddr 是字地址（>>2）
//   XX 应该等于写入数据的最低字节
// ============================================================================
`timescale 1ns / 1ps

module mcu_ahb_probe (
    input  wire        I_hclk,
    input  wire        I_hresetn,

    input  wire [30:0] I_haddr,
    input  wire [1:0]  I_htrans,
    input  wire        I_hwrite,
    input  wire [2:0]  I_hsize,
    input  wire [2:0]  I_hburst,
    input  wire [3:0]  I_hprot,
    input  wire        I_hmastlock,
    input  wire [31:0] I_hwdata,
    output wire [31:0] O_hrdata,
    output wire [1:0]  O_hresp,
    output wire        O_hready
);

    assign O_hready = 1'b1;
    assign O_hresp  = 2'b00;

    wire busy_xfer = (I_htrans[1] == 1'b1);   // NONSEQ or SEQ

    reg [30:0] dbg_addr;    // 上次"写"的 haddr
    reg [31:0] dbg_wdata;   // 上次"写"的 hwdata
    reg [15:0] dbg_cnt;

    always @(posedge I_hclk or negedge I_hresetn) begin
        if (!I_hresetn) begin
            dbg_addr  <= 31'd0;
            dbg_wdata <= 32'd0;
            dbg_cnt   <= 16'd0;
        end else if (busy_xfer) begin
            dbg_cnt <= dbg_cnt + 16'd1;
            if (I_hwrite) begin
                dbg_addr  <= I_haddr;
                dbg_wdata <= I_hwdata;
            end
        end
    end

    // 读：返回 {地址高 23 位, 写数据低 8 位}
    assign O_hrdata = {dbg_addr[30:8], dbg_wdata[7:0]};

endmodule
