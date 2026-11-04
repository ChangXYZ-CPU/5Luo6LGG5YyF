// ============================================================================
// mcu_apb_regs.v —— 调试版：无条件响应所有 psel，用来判定 psel 到底到不到 fabric
//
// 与正式版的区别：
//   1. 译码：sel = I_psel（不再限定 paddr[19:16]==4'h8）
//   2. 读：除调试偏移外，任何地址都返回 ID = 0x50483150
//   3. 新增调试寄存器（写操作时抓拍）：
//        0x2C  DBG_PADDR   上次访问时总线给出的 paddr
//        0x30  DBG_PWDATA  上次访问的写数据
//        0x34  DBG_CNT     access（sel&&penable）计数值
//        0x38  DBG_SELCNT  仅 psel 为高的拍数（判断 psel 是否真的来过）
//
// 判定：
//   读到 0x50483150        -> psel 到 fabric 了
//   DBG_PADDR == 0x00080000 -> 地址编码与假设一致
//   DBG_PADDR 是别的值      -> psel 通，但地址编码约定不同
//   什么都读不到（fault）   -> psel 没到 fabric
// ============================================================================
`timescale 1ns / 1ps

module mcu_apb_regs (
    input  wire        I_apb_clk,
    input  wire        I_apb_rst,        // 高有效

    // ---- APB 从机接口 ----
    input  wire [19:0] I_paddr,
    input  wire        I_psel,
    input  wire        I_penable,
    input  wire        I_pwrite,
    input  wire [31:0] I_pwdata,
    input  wire [3:0]  I_pstrobe,
    input  wire [2:0]  I_pprot,
    output reg  [31:0] O_prdata,
    output wire        O_pready,
    output wire        O_pslverr,

    // ---- 只读状态输入（调试期未接）----
    input  wire [3:0]  I_sw_code,
    input  wire [3:0]  I_mode_now,
    input  wire        I_det_active,
    input  wire        I_guard_active,
    input  wire        I_has_target,
    input  wire [15:0] I_frame_cnt,

    // ---- 覆盖/参数输出（调试期未接）----
    output reg  [3:0]  O_ovr_mode,
    output reg         O_ovr_en,
    output reg  [7:0]  O_th_min,
    output reg  [7:0]  O_th_dom
);

    localparam [31:0] ID_VALUE = 32'h5048_3150;   // "PH1P"

    // ---- 调试版：无条件响应 ----
    wire sel    = I_psel;
    wire access = sel && I_penable;
    wire wr     = access &&  I_pwrite;
    wire rd     = access && !I_pwrite;

    assign O_pready  = sel;
    assign O_pslverr = 1'b0;

    wire [7:0] off = I_paddr[7:0];

    // ---- 调试抓拍 ----
    reg [19:0] dbg_paddr;
    reg [31:0] dbg_pwdata;
    reg [15:0] dbg_cnt;
    reg [15:0] dbg_sel_cnt;

    always @(posedge I_apb_clk or posedge I_apb_rst) begin
        if (I_apb_rst) begin
            dbg_paddr   <= 20'd0;
            dbg_pwdata  <= 32'd0;
            dbg_cnt     <= 16'd0;
            dbg_sel_cnt <= 16'd0;
        end else begin
            if (I_psel) dbg_sel_cnt <= dbg_sel_cnt + 16'd1;
            if (access) begin
                dbg_cnt    <= dbg_cnt + 16'd1;
                dbg_paddr  <= I_paddr;
                dbg_pwdata <= I_pwdata;
            end
        end
    end

    // ---- 写寄存器（保留原功能）----
    always @(posedge I_apb_clk or posedge I_apb_rst) begin
        if (I_apb_rst) begin
            O_ovr_mode <= 4'd0;
            O_ovr_en   <= 1'b0;
            O_th_min   <= 8'd48;
            O_th_dom   <= 8'd20;
        end else if (wr) begin
            case (off)
                8'h14: O_ovr_mode <= I_pwdata[3:0];
                8'h18: begin
                    if (I_pwdata[0]) O_ovr_en <= 1'b1;
                    if (I_pwdata[1]) O_ovr_en <= 1'b0;
                end
                8'h24: O_th_min <= I_pwdata[7:0];
                8'h28: O_th_dom <= I_pwdata[7:0];
                default: ;
            endcase
        end
    end

    // ---- 读：调试偏移之外一律返回 ID ----
    always @(*) begin
        if (rd) begin
            case (off)
                8'h2C:   O_prdata = {12'd0, dbg_paddr};
                8'h30:   O_prdata = dbg_pwdata;
                8'h34:   O_prdata = {16'd0, dbg_cnt};
                8'h38:   O_prdata = {16'd0, dbg_sel_cnt};
                default: O_prdata = ID_VALUE;   // 任何地址都回 ID
            endcase
        end else begin
            O_prdata = 32'd0;
        end
    end

endmodule
