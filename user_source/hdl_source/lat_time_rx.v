// ============================================================================
// lat_time_rx -- 公共时基接收端（消费域）
//
// L02 组件之二：把一个 lat_tick_gen 的时基安全地搬进某个消费时钟域。
//
// 两路跨域，各有明确机制：
//   1) 时间：32 位 **Gray** 总线，3 级同步链，再在本域解码成二进制。
//      - 源端已寄存，且每次只变 1 位；
//      - 采样落在跳变中间时，解码结果只可能是"旧值"或"新值"，误差 ≤ 1 tick；
//      - 不得逐位独立同步普通二进制时间戳（文档 §4.2.2）。
//   2) 会话：16 位**不逐位同步**，而是同步 1 位 toggle，边沿到来时把已经稳定
//      的 I_session 快照进本域（文档 §4.2.7）。这就是既有的 AE/AG 桥模式。
//
// gray_pipe 是 96 位打包向量。2026-10-19 复核：TD get_regs 解析的是逻辑
// 寄存器输出网络，不能用综合物理 cell 的 *_reg 名称；此前命令失败不能据此
// 归因于数组形式。有效主 SDC 使用源端 S_lat_tick_gray* 别名与接收端
// u_lat_rx_{csi,pix}/gray_pipe*，并需逐位核查约束未被异步时钟组覆盖。
// 位序：[31:0]=第 1 级, [63:32]=第 2 级, [95:64]=第 3 级（供解码使用）。
//
// O_ready 在第一次捕获到会话快照后置位；在此之前测量必须处于 WAIT。
// ============================================================================
module lat_time_rx #(
    parameter integer SYNC_STAGES = 3          // 本实现固定 3 级；参数保留给测试台
)(
    input  wire        I_clk,                       // 消费域时钟
    input  wire        I_rst_n,

    input  wire [31:0] I_tick_gray,                 // 来自 lat_tick_gen（异步）
    input  wire [15:0] I_session,                   // 来自 lat_tick_gen（异步，稳定）
    input  wire        I_session_tog,               // 会话快照握手翻转

    output reg  [31:0] O_tick_bin,                  // 本域二进制时间（0.1 ms）
    output reg  [15:0] O_session,                   // 本域会话快照
    output reg         O_ready                      // 已完成首次会话捕获
);

    // ------------------------------------------------------------------ Gray 解码
    // 前缀异或折叠（5 级），不要用 32 级串行 XOR。
    function [31:0] gray2bin;
        input [31:0] g;
        reg [31:0] a, b, c, d, e;
        begin
            a = g ^ (g >> 1);
            b = a ^ (a >> 2);
            c = b ^ (b >> 4);
            d = c ^ (c >> 8);
            e = d ^ (d >> 16);
            gray2bin = e;
        end
    endfunction

    // ------------------------------------------------------------------ 时间同步链
    // 96 位打包向量 = 3 级 × 32 位。源端 Gray 到第一级有显式预算；
    // 同步链内部仍按本域时钟检查 setup/hold，不可一起切掉。
    (* async_reg = "true" *) reg [95:0] gray_pipe;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) gray_pipe <= 96'd0;
        else         gray_pipe <= {gray_pipe[63:0], I_tick_gray};
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) O_tick_bin <= 32'd0;
        else         O_tick_bin <= gray2bin(gray_pipe[95:64]);
    end

    // ------------------------------------------------------------------ 会话握手
    reg [2:0] tog_sync;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            tog_sync  <= 3'b000;
            O_session <= 16'd0;
            O_ready   <= 1'b0;
        end else begin
            tog_sync <= {tog_sync[1:0], I_session_tog};
            // 边沿 = 源端会话值自上次捕获后一直稳定，此时快照安全
            if(tog_sync[2] ^ tog_sync[1]) begin
                O_session <= I_session;
                O_ready   <= 1'b1;
            end
        end
    end

endmodule
