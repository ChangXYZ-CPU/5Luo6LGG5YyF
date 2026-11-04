// ============================================================================
// mcu_ahb_slave.v —— MCU AHB 主口 <-> fabric 寄存器组（正式版）
//
// 总线：AHB-Lite 从机，时钟 = MCU 宏输出的 ahb_clk（= core_clk / AHB_Div = 150MHz）
//     All active transfers: one wait. Registered address and HRDATA. HRESP remains OKAY.
//     地址：窗口1 0x80000000 / 窗口2 0x90000000（haddr[30:12]==BASE_SEL/BASE_SEL2，
//           默认 19'h80000 / 19'h90000），每窗口 4KB——官方 AHBDemo 基址即 0x80000000
//           偏移 = haddr[11:2]（字对齐，忽略 bit1:0）
//
// 寄存器表（字偏移 = 字节偏移 / 4）：
//   0x00  ID        RO  0x50483150 "PH1P"
//   0x04  VER       RO  0x0001_0000
//   0x08  SW_CODE   RO  拨码开关状态（fabric -> AHB，模块内 2FF 同步）
//   0x0C  MODE_NOW  RO  当前生效模式
//   0x10  MODE_SRC  RO  bit0 = 串口覆盖生效中
//   0x14  OVR_MODE  RW  串口想设的模式（4 位）
//   0x18  OVR_CTL   W1P bit0=使能覆盖 bit1=清除覆盖
//   0x1C  STATUS    RO  bit0 检测中 / bit1 有目标
//   0x20  FRAME_CNT RO  帧计数器低 16 位
//   0x24  TH_MIN    RW  检测 MIN 阈值
//   0x28  TH_DOM    RW  检测 DOM 阈值
//   0x2C  TARGET_AREA RO 当前模式下最大连通块面积（20 位，像素数；无目标时为 0）
//   0x34  SHAPE_CLASS RO 形状分类结果
//                     bit[1:0]=最大确定目标的类别(0 未知/1 圆/2 三角/3 方)
//                     bit2=该类别已经三帧确认
//                     bit[4:3]=画面上已确认的目标个数(0~3)
//        —— 只在模式 12（形状分类）下有值；3 帧时域确认过才置有效，丢目标后自动清零。
//           2026-11-02 新增，供固件 S_RUN 命令轮询。
//
// 跨时钟：
//   fabric 域（像素时钟）信号 -> 本模块：逐比特 2FF 同步（慢变量，可接受）
//   本模块输出 O_ovr_*/O_th_* 在 AHB 域：由顶层再做 2FF 同步到像素域
// ============================================================================
`timescale 1ns / 1ps

module mcu_ahb_slave #(
    parameter [18:0] BASE_SEL  = 19'h80000,  // 窗口1 >> 12（0x80000000）
    parameter [18:0] BASE_SEL2 = 19'h80000   // 窗口2 >> 12（默认同窗口1）
) (
    input  wire        I_hclk,
    input  wire        I_return_clk, // MCU core clock, 2x I_hclk; same PLL source
    input  wire        I_hresetn,

    // ---- AHB-Lite 从机 ----
    input  wire [30:0] I_haddr,
    input  wire [1:0]  I_htrans,
    input  wire        I_hwrite,
    input  wire [2:0]  I_hsize,
    input  wire [2:0]  I_hburst,
    input  wire [3:0]  I_hprot,
    input  wire        I_hmastlock,
    input  wire [31:0] I_hwdata,
    output reg  [31:0] O_hrdata,
    output wire [1:0]  O_hresp,
    output reg         O_hready,

    // ---- 来自 fabric（像素时钟域，异步）----
    input  wire [3:0]  I_sw_code,
    input  wire [3:0]  I_mode_now,
    input  wire        I_det_active,
    input  wire        I_has_target,
    input  wire [15:0] I_frame_cnt,
    input  wire [19:0] I_target_area,  // 2026-11-01: largest-component area, current mode (pixel domain)
    input  wire [1:0]  I_shape_class,  // largest confirmed shape class
    input  wire        I_shape_valid,  // that class confirmed by the 3-frame filter
    input  wire [1:0]  I_shape_count,  // targets in the published list, 0..3

    // ---- 输出给 fabric（AHB 域，顶层需 2FF 同步到像素域）----
    output reg  [3:0]  O_ovr_mode,
    output reg         O_ovr_en,
    output reg         O_ovr_tog,      // 每次写 OVR_CTL 翻转一次（跨域边沿检测用）
    input  wire        I_ovr_active,   // 像素域实际的覆盖生效状态（回读 MODE_SRC）
    output reg  [7:0]  O_th_min,
    output reg  [7:0]  O_th_dom,

    // ---- 阶段 4：OSD 命令通道 ----
    output reg  [3:0]  O_cmd_op,    // 1=跳项 2=下一项 3=加 4=减
    output reg  [3:0]  O_cmd_arg,   // 跳项时的目标索引
    output reg  [3:0]  O_cmd_seq    // 每次写入自增，供像素域边沿检测
);

    localparam [31:0] ID_VALUE  = 32'h5048_3150;   // "PH1P"
    localparam [31:0] VER_VALUE = 32'h0001_0000;

    // ------------------------------------------------------------------
    // 窗口译码（地址相）
    // ------------------------------------------------------------------
    // Register the raw address before comparing the complete window prefix.
    // HREADY inserts one wait for every active transfer, including foreign
    // addresses. This gives decode/response a full HCLK cycle, while keeping
    // the 300 MHz hard-MCU -> 150 MHz capture path free of window-decode LUTs.
    reg [18:0] addr_hi;
    reg [9:0] word_d;
    reg wr_d;
    reg ready_d;
    wire busy_ph = I_htrans[1];
    wire in_win;
    localparam [18:0] BASE_DIFF = BASE_SEL ^ BASE_SEL2;
    localparam [18:0] SHARED_MASK = ~BASE_DIFF;
    generate if ((BASE_DIFF & (BASE_DIFF - 19'd1)) == 0) begin: factored_window
        assign in_win = ((addr_hi & SHARED_MASK) == (BASE_SEL & SHARED_MASK));
    end else begin: independent_windows
        assign in_win = (addr_hi == BASE_SEL) || (addr_hi == BASE_SEL2);
    end endgenerate
    // The hard MCU consumes the return bus in its core-clock domain.
    // Register both data and ready on that same clock. With core=2*AHB,
    // the response settles before the next HCLK completion edge; the AHB
    // state machine retains one wait. Do not use this interface at core<=AHB.
    reg [31:0] hrdata_q;
    always @(posedge I_return_clk or negedge I_hresetn) begin
        if (!I_hresetn) begin O_hready <= 1'b1; O_hrdata <= 32'd0; end
        else begin O_hready <= ready_d; O_hrdata <= hrdata_q; end
    end
    assign O_hresp = 2'b00;
    wire selected_read = !ready_d && in_win && !wr_d;

    // Capture every HCLK without an address feedback mux. Only the raw values
    // captured at an accepted address edge are consumed on the following edge.
    // While ready_d is low the master holds the next address and current write
    // data; no additional transaction is accepted until completion.
    always @(posedge I_hclk or negedge I_hresetn) begin
        if (!I_hresetn) begin
            addr_hi <= 19'd0;
            word_d <= 10'd0;
            wr_d <= 1'b0;
            ready_d <= 1'b1;
        end else begin
            addr_hi <= I_haddr[30:12];
            word_d <= I_haddr[11:2];
            wr_d <= I_hwrite;
            ready_d <= !ready_d || !busy_ph;
        end
    end

    // ------------------------------------------------------------------
    // fabric -> AHB：逐比特 2FF 同步
    // ------------------------------------------------------------------
    reg [1:0] sw_s0, sw_s1, sw_s2, sw_s3;
    reg [1:0] md_s0, md_s1, md_s2, md_s3;
    reg [1:0] det_s, tgt_s;
    reg [1:0] act_s;
    reg [1:0] cnt_s0, cnt_s1, cnt_s2, cnt_s3;
    reg [1:0] cnt_s4, cnt_s5, cnt_s6, cnt_s7;
    reg [1:0] cnt_s8, cnt_s9, cnt_s10, cnt_s11;
    reg [1:0] cnt_s12, cnt_s13, cnt_s14, cnt_s15;

    always @(posedge I_hclk or negedge I_hresetn) begin
        if (!I_hresetn) begin
            sw_s0<=0;sw_s1<=0;sw_s2<=0;sw_s3<=0;
            md_s0<=0;md_s1<=0;md_s2<=0;md_s3<=0;
            det_s<=0;tgt_s<=0;act_s<=0;
            cnt_s0<=0;cnt_s1<=0;cnt_s2<=0;cnt_s3<=0;
            cnt_s4<=0;cnt_s5<=0;cnt_s6<=0;cnt_s7<=0;
            cnt_s8<=0;cnt_s9<=0;cnt_s10<=0;cnt_s11<=0;
            cnt_s12<=0;cnt_s13<=0;cnt_s14<=0;cnt_s15<=0;
        end else begin
            sw_s0 <= {sw_s0[0], I_sw_code[0]}; sw_s1 <= {sw_s1[0], I_sw_code[1]};
            sw_s2 <= {sw_s2[0], I_sw_code[2]}; sw_s3 <= {sw_s3[0], I_sw_code[3]};
            md_s0 <= {md_s0[0], I_mode_now[0]}; md_s1 <= {md_s1[0], I_mode_now[1]};
            md_s2 <= {md_s2[0], I_mode_now[2]}; md_s3 <= {md_s3[0], I_mode_now[3]};
            det_s <= {det_s[0], I_det_active};
            act_s <= {act_s[0], I_ovr_active};
            tgt_s <= {tgt_s[0], I_has_target};
            cnt_s0  <= {cnt_s0[0],  I_frame_cnt[0]};  cnt_s1  <= {cnt_s1[0],  I_frame_cnt[1]};
            cnt_s2  <= {cnt_s2[0],  I_frame_cnt[2]};  cnt_s3  <= {cnt_s3[0],  I_frame_cnt[3]};
            cnt_s4  <= {cnt_s4[0],  I_frame_cnt[4]};  cnt_s5  <= {cnt_s5[0],  I_frame_cnt[5]};
            cnt_s6  <= {cnt_s6[0],  I_frame_cnt[6]};  cnt_s7  <= {cnt_s7[0],  I_frame_cnt[7]};
            cnt_s8  <= {cnt_s8[0],  I_frame_cnt[8]};  cnt_s9  <= {cnt_s9[0],  I_frame_cnt[9]};
            cnt_s10 <= {cnt_s10[0], I_frame_cnt[10]}; cnt_s11 <= {cnt_s11[0], I_frame_cnt[11]};
            cnt_s12 <= {cnt_s12[0], I_frame_cnt[12]}; cnt_s13 <= {cnt_s13[0], I_frame_cnt[13]};
            cnt_s14 <= {cnt_s14[0], I_frame_cnt[14]}; cnt_s15 <= {cnt_s15[0], I_frame_cnt[15]};
        end
    end

    wire [3:0]  sw_now   = {sw_s3[1], sw_s2[1], sw_s1[1], sw_s0[1]};
    wire [3:0]  mode_now = {md_s3[1], md_s2[1], md_s1[1], md_s0[1]};
    wire        det_now  = det_s[1];
    wire        tgt_now  = tgt_s[1];
    wire [15:0] cnt_now  = {cnt_s15[1], cnt_s14[1], cnt_s13[1], cnt_s12[1],
                            cnt_s11[1], cnt_s10[1], cnt_s9[1],  cnt_s8[1],
                            cnt_s7[1],  cnt_s6[1],  cnt_s5[1],  cnt_s4[1],
                            cnt_s3[1],  cnt_s2[1],  cnt_s1[1],  cnt_s0[1]};

    // 面积：整字一起更新（每帧一次），按整字 2FF 同步；固件读之前会等若干帧，不会读到半途值
    reg [19:0] area_s0, area_s1;
    always @(posedge I_hclk or negedge I_hresetn) begin
        if (!I_hresetn) begin
            area_s0 <= 20'd0;
            area_s1 <= 20'd0;
        end else begin
            area_s0 <= I_target_area;
            area_s1 <= area_s0;
        end
    end

    // 形状类别：整字（5 位）一起 2FF 同步，避免读到"类别已变、有效位还是旧值"
    reg [4:0] shape_s0, shape_s1;
    always @(posedge I_hclk or negedge I_hresetn) begin
        if (!I_hresetn) begin
            shape_s0 <= 5'd0;
            shape_s1 <= 5'd0;
        end else begin
            shape_s0 <= {I_shape_count, I_shape_valid, I_shape_class};
            shape_s1 <= shape_s0;
        end
    end


    // ------------------------------------------------------------------
    // 写寄存器（数据相）
    // ------------------------------------------------------------------
    wire wr_en = !ready_d && in_win && wr_d;

    always @(posedge I_hclk or negedge I_hresetn) begin
        if (!I_hresetn) begin
            O_ovr_mode <= 4'd0;
            O_ovr_en   <= 1'b0;
            O_ovr_tog  <= 1'b0;
            O_cmd_op   <= 4'd0;
            O_cmd_arg  <= 4'd0;
            O_cmd_seq  <= 4'd0;
            O_th_min   <= 8'd48;
            O_th_dom   <= 8'd20;
        end else if (wr_en) begin
            case (word_d)
                10'd5:  O_ovr_mode <= I_hwdata[3:0];              // 0x14
                10'd6:  begin                                     // 0x18
                    O_ovr_tog <= ~O_ovr_tog;
                    if (I_hwdata[0]) O_ovr_en <= 1'b1;
                    if (I_hwdata[1]) O_ovr_en <= 1'b0;
                end
                10'd9:  O_th_min <= I_hwdata[7:0];                // 0x24
                10'd10: O_th_dom <= I_hwdata[7:0];                // 0x28
                10'd12: begin                                     // 0x30 OSD_CMD
                    O_cmd_op  <= I_hwdata[7:4];
                    O_cmd_arg <= I_hwdata[3:0];
                    O_cmd_seq <= O_cmd_seq + 4'd1;
                end
                default: ;
            endcase
        end
    end

    // ------------------------------------------------------------------
    // 读寄存器（数据相组合）
    // ------------------------------------------------------------------
    reg [31:0] decoded_read;
    always @(*) begin
        decoded_read = 32'd0;
            case (word_d)
                10'd0:  decoded_read = ID_VALUE;
                10'd1:  decoded_read = VER_VALUE;
                10'd2:  decoded_read = {28'd0, sw_now};
                10'd3:  decoded_read = {28'd0, mode_now};
                10'd4:  decoded_read = {31'd0, act_s[1]};
                10'd5:  decoded_read = {28'd0, O_ovr_mode};
                10'd6:  decoded_read = 32'd0;
                10'd7:  decoded_read = {30'd0, tgt_now, det_now};
                10'd8:  decoded_read = {16'd0, cnt_now};
                10'd9:  decoded_read = {24'd0, O_th_min};
                10'd10: decoded_read = {24'd0, O_th_dom};
                10'd11: decoded_read = {12'd0, area_s1};          // 0x2C TARGET_AREA
                10'd13: decoded_read = {27'd0, shape_s1};         // 0x34 SHAPE_CLASS
                default: decoded_read = 32'd0;
            endcase
    end
    // Response is prepared during the wait cycle and is stable before the
    // completion edge. Writes commit exactly once during that same wait cycle.
    always @(posedge I_hclk or negedge I_hresetn) begin
        if (!I_hresetn) hrdata_q <= 32'd0;
        else if (selected_read) hrdata_q <= decoded_read;
        else hrdata_q <= 32'd0;
    end

endmodule
