// ============================================================================
// lat_tick_gen -- 公共时基产生器（50 MHz 域）
//
// L02 组件之一：处理延迟可视化（LAT）的"秒表"。
//
// 规格（L01 已冻结）：
//   - 32 位自由运行计数器，每 CLKS_PER_TICK 个 50 MHz 周期 +1
//   - CLKS_PER_TICK = 5000 → 5000/50MHz = 100 us = 0.1 ms 一 tick
//   - 不随每帧、切模式或输入停顿清零；2^32 tick ≈ 4.97 天回绕
//   - 输出**寄存后的 Gray 快照**供跨域（文档 §4.2.1：Gray 只能来自寄存后的计时器）
//   - session（16 位）经 toggle 握手分发，不当作普通总线逐位同步
//
// 会话语义（L01 §8）：整机复位源只有 S_pll_lock，复位=电源键。上电复位后
// session 固定为 1，并在复位释放后翻转一次 session_tog，让各接收域把 session
// 快照进本域并置 ready。运行期若 I_rebuild 有效（本功能目前不使用，留给将来
// 只复位测量逻辑的场景），session 递增并再翻转一次；此时在途的旧描述符会被
// 接收端按 session 不一致拒绝，LAT 显示 WAIT 直到新的完整好帧。
//
// 注意：本模块**不**清零 tick_bin。计时是绝对基准，重建只换 session。
// ============================================================================
module lat_tick_gen #(
    parameter integer CLKS_PER_TICK = 5000          // 50 MHz -> 0.1 ms
)(
    input  wire        I_clk,                       // sys_clk_50m
    input  wire        I_rst_n,                     // 与 S_pll_lock 同源
    input  wire        I_rebuild,                   // 1 = 重建会话（本功能暂不使用）

    output reg  [31:0] O_tick_bin,                  // 本域二进制视图（调试/自检用）
    output reg  [31:0] O_tick_gray,                 // 供跨域的寄存 Gray 快照
    output reg  [15:0] O_session,                   // 测量会话号
    output reg         O_session_tog                // 会话快照握手翻转
);

    // ------------------------------------------------------------------ Gray
    function [31:0] bin2gray;
        input [31:0] b;
        begin
            bin2gray = b ^ (b >> 1);
        end
    endfunction

    reg [12:0] div_cnt;                              // 0..4999，13 位足够（原写 32 位是浪费）
    reg        started;

    wire step = (div_cnt == (CLKS_PER_TICK - 1));

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            div_cnt       <= 13'd0;
            O_tick_bin    <= 32'd0;
            O_tick_gray   <= 32'd0;
            O_session     <= 16'd1;      // 1 = 有效会话；0 保留给"未建立"
            O_session_tog <= 1'b0;
            started       <= 1'b0;
        end else begin
            // ---- 计时：绝对基准，永不清零 ----
            if(step) begin
                div_cnt    <= 32'd0;
                O_tick_bin <= O_tick_bin + 32'd1;
                // 寄存后的 Gray 快照：直接取"下一拍的值"的 Gray，
                // 保证快照与 O_tick_bin 同相，不落后一拍。
                O_tick_gray <= bin2gray(O_tick_bin + 32'd1);
            end else begin
                div_cnt <= div_cnt + 13'd1;
            end

            // ---- 会话：复位释放后先翻一次，让接收域拿到初始 session ----
            if(!started) begin
                started       <= 1'b1;
                O_session_tog <= ~O_session_tog;
            end else if(I_rebuild) begin
                O_session     <= O_session + 16'd1;
                O_session_tog <= ~O_session_tog;
            end
        end
    end

endmodule
