// ============================================================================
// lat_calc_bcd -- 延迟计算、状态判定与 4 位 BCD（像素域）
//
// L02 组件之三。纯本地逻辑，可直接用强制时间戳做单元测试（不需要图像通路）。
//
// 判定优先级（L01 §8.2 冻结）：
//   1) 时基未就绪                                    -> WAIT
//   2) 描述符无效 / 会话不一致                        -> INVAL  ("----")
//   3) 该描述符已被判超龄，或 age > LIFETIME_TICKS    -> STALE
//   4) delta > MAX_TICKS                              -> OVR
//   5) 其余                                           -> OK + 4 位 BCD
//
// 结构：首像素时间判定使用纯组合 w_dec；帧中 I_frame_abort 可撤销结果，提交只有
// 一个地方，避免"判定与提交各写一份"导致两处不一致。
//   非 OK：在 frame_pulse 当拍提交（BCD 清零，不留上一帧数字）
//   OK   ：先跑 14 拍 double-dabble，转换完成时提交（state 与 bcd 同一次更新，
//          满足 L01 §9「一行内数字必须来自同一结果」）
//
// 关键细节：
//   - delta = I_tick_now - I_start_tick（模 2^32）；在"本帧首像素"采样，所以就是
//     T_end - T_start。重复显示每帧重算，读数会增长（L01 §3.3）。
//   - 32 位计数约 4.97 天回绕；超龄必须**锁存**（stale_latched + session/start），
//     否则回绕后旧描述符会重新变有效（文档 §5.2 禁止）。
//   - 运行时新帧到来会放弃未完成的转换，绝不把半成品提交。
// ============================================================================
module lat_calc_bcd #(
    parameter [31:0] LIFETIME_TICKS = 32'd100000,   // 10 s @ 0.1 ms
    parameter [31:0] MAX_TICKS      = 32'd9999      // 显示上限 999.9 ms
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire [31:0] I_tick_now,        // 本帧首像素时刻 T_end（本域同步值）
    input  wire [15:0] I_session_now,     // 本域会话快照
    input  wire        I_time_ready,      // lat_time_rx 已完成首次会话捕获

    input  wire        I_frame_pulse,     // 本显示帧首像素（1 拍）
    input  wire        I_frame_abort,     // 帧中故障；撤销转换/已提交数字，直到下一帧
    input  wire        I_lat_valid,       // 描述符 lat_valid
    input  wire [15:0] I_desc_session,    // 描述符 time_session
    input  wire [31:0] I_start_tick,      // 描述符 start_tick

    output reg  [2:0]  O_state,           // 见 ST_*
    output reg  [15:0] O_bcd,             // {d3,d2,d1,d0}
    output reg  [31:0] O_delta_tick,      // 原始延迟（tick），供仿真/调试
    output reg         O_snapshot_tog,    // 一次结果提交（供 OSD 锁存）
    output reg  [15:0] O_desc_session     // 与结果同快照的会话号（供上屏核对）
);

    localparam [2:0] ST_WAIT  = 3'd0,
                     ST_OK    = 3'd1,
                     ST_OVR   = 3'd2,
                     ST_STALE = 3'd3,
                     ST_INVAL = 3'd4;

    // ---------------------------------------------------------------- 超龄锁存
    reg        stale_latched;
    reg [15:0] stale_session;
    reg [31:0] stale_start;

    // ---------------------------------------------------------------- 组合判定
    wire [31:0] w_age        = I_tick_now - I_start_tick;
    wire        w_stale_hit  = stale_latched &&
                               (stale_session == I_desc_session) &&
                               (stale_start   == I_start_tick);
    wire        w_bad_id     = !I_lat_valid || (I_desc_session != I_session_now);

    reg [2:0] w_dec;
    always @(*) begin
        if(!I_time_ready)          w_dec = ST_WAIT;
        else if(w_bad_id)          w_dec = ST_INVAL;
        else if(w_stale_hit)       w_dec = ST_STALE;
        else if(w_age >= LIFETIME_TICKS) w_dec = ST_STALE;
        else if(w_age > MAX_TICKS) w_dec = ST_OVR;
        else                       w_dec = ST_OK;
    end

    // ---------------------------------------------------------------- BCD 转换
    reg [13:0] dd_bin;
    reg [15:0] dd_bcd;
    reg [4:0]  dd_cnt;
    reg        dd_run;
    reg [31:0] dd_delta;
    reg [15:0] result_session;
    reg        frame_live;

    // 四位分别 +3（若 >=5），再整体左移 1 位并带入二进制最高位
    wire [15:0] dd_adj = {
        (dd_bcd[15:12] >= 4'd5) ? (dd_bcd[15:12] + 4'd3) : dd_bcd[15:12],
        (dd_bcd[11:8]  >= 4'd5) ? (dd_bcd[11:8]  + 4'd3) : dd_bcd[11:8],
        (dd_bcd[7:4]   >= 4'd5) ? (dd_bcd[7:4]   + 4'd3) : dd_bcd[7:4],
        (dd_bcd[3:0]   >= 4'd5) ? (dd_bcd[3:0]   + 4'd3) : dd_bcd[3:0] };

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            stale_latched  <= 1'b0;
            stale_session  <= 16'd0;
            stale_start    <= 32'd0;
            dd_bin         <= 14'd0;
            dd_bcd         <= 16'd0;
            dd_cnt         <= 5'd0;
            dd_run         <= 1'b0;
            dd_delta       <= 32'd0;
            result_session <= 16'd0;
            frame_live     <= 1'b0;
            O_state        <= ST_WAIT;
            O_bcd          <= 16'd0;
            O_delta_tick   <= 32'd0;
            O_snapshot_tog <= 1'b0;
            O_desc_session <= 16'd0;
        end else if(I_frame_pulse) begin
            // A new frame replaces the previous (possibly aborted) result.
            frame_live     <= 1'b1;
            result_session <= I_desc_session;
            if(w_dec == ST_OK) begin
                dd_delta <= w_age;
                dd_bin   <= w_age[13:0];      // w_age <= MAX_TICKS(9999)，14 位足够
                dd_bcd   <= 16'd0;
                dd_cnt   <= 5'd14;
                dd_run   <= 1'b1;
            end else begin
                dd_run         <= 1'b0;       // 放弃未完成转换
                O_state        <= w_dec;
                O_bcd          <= 16'd0;
                O_delta_tick   <= (w_dec == ST_OVR) ? w_age : 32'd0;
                O_desc_session <= I_desc_session;
                O_snapshot_tog <= ~O_snapshot_tog;
                if(w_dec == ST_STALE && !w_stale_hit) begin
                    // 首次判超龄：锁存，防止计数器回绕后旧描述符重新变有效
                    stale_latched <= 1'b1;
                    stale_session <= I_desc_session;
                    stale_start   <= I_start_tick;
                end
            end
        end else if(frame_live && (I_frame_abort || !I_time_ready ||
                                  I_session_now != result_session)) begin
            // Abort wins over the final double-dabble cycle, so a cancelled
            // conversion cannot publish OK later. Commit invalid only once.
            frame_live     <= 1'b0;
            dd_run         <= 1'b0;
            O_state        <= I_time_ready ? ST_INVAL : ST_WAIT;
            O_bcd          <= 16'd0;
            O_delta_tick   <= 32'd0;
            O_desc_session <= result_session;
            O_snapshot_tog <= ~O_snapshot_tog;
        end else if(dd_run) begin
            // ---------------- 14 拍 double-dabble ----------------
            dd_bcd <= {dd_adj[14:0], dd_bin[13]};
            dd_bin <= {dd_bin[12:0], 1'b0};
            dd_cnt <= dd_cnt - 5'd1;
            if(dd_cnt == 5'd1) begin
                dd_run         <= 1'b0;
                O_state        <= ST_OK;
                O_bcd          <= {dd_adj[14:0], dd_bin[13]};
                O_delta_tick   <= dd_delta;
                O_desc_session <= result_session;
                O_snapshot_tog <= ~O_snapshot_tog;
            end
        end
    end

endmodule
