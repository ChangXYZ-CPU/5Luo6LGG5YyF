`timescale 1ns / 1ps
/*****************************************************************
 * dual_loop_arbiter.v -- 双环（AE 曝光 / AG 增益）优先级仲裁（精简版，2026-10-16）
 *
 * 设计原则：**不改 ae_ctrl / agc_ctrl 内部的任何算法**，只通过两个 I_hold 输入
 * （分别冻结对应环的"步进更新"）实现优先级。出问题只需回退本模块与两处 I_hold 端口。
 *
 * 规则（用户确认的精简版；**不含重心迁移 R4**）：
 *  R1 偏暗（mean < target-30）：曝光环自己往上爬；**冻结增益环**
 *  R2 偏暗且曝光已到顶（I_exp_sat=1）：放开增益环，允许加增益
 *  R3 偏亮（mean > target+30）：**先降增益**；增益仍高于软下限 96 时冻结曝光环
 *
 * 说明：
 *  - 纯组合判定 + 一级寄存输出，**无状态机、无节拍计数**
 *  - 只在 I_auto_both=1（AE 与 AG 都在自动）时生效；手动模式一律不干预
 *  - 全部信号均在像素时钟域，不引入跨时钟域路径
 *****************************************************************/
module dual_loop_arbiter #(
    parameter [7:0]  DEADBAND   = 8'd30,
    parameter [15:0] GAIN_FLOOR = 16'd96
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_auto_both,     // 1 = AE 与 AG 均在自动
    input  wire [7:0]  I_target_y,      // target brightness (TG item, shared by three consumers)
    input  wire [7:0]  I_mean,          // frame mean
    input  wire [15:0] I_gain,          // AGC output
    input  wire        I_exp_sat,       // exposure saturated (top rung or flicker pinned)
    output reg         O_ae_hold,       // 1 = freeze the AE step
    output reg         O_ag_hold        // 1 = freeze the AGC step
);
    // 9-bit compares to avoid 8-bit overflow
    wire [8:0] lo_lim = {1'b0, I_target_y} - {1'b0, DEADBAND};
    wire [8:0] hi_lim = {1'b0, I_target_y} + {1'b0, DEADBAND};
    wire [8:0] mean9  = {1'b0, I_mean};

    wire too_dark   = (mean9 < lo_lim);
    wire too_bright = (mean9 > hi_lim);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_ae_hold <= 1'b0;
            O_ag_hold <= 1'b0;
        end else begin
            // R3: too bright and the gain is still above its soft floor -> freeze the exposure
            O_ae_hold <= I_auto_both && too_bright && (I_gain > GAIN_FLOOR);
            // R1/R2: too dark and the exposure is not saturated yet -> freeze the gain
            O_ag_hold <= I_auto_both && too_dark && !I_exp_sat;
        end
    end
endmodule
