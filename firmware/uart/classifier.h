/**
 * classifier.h -- 三色面积分类状态机（PH1P35 软核）
 *
 * 流程：UART0 收到 "RUN" -> 依次切到红/绿/蓝检测模式各采一次面积 ->
 *       判定 -> 调用电机动作（复用 motor_master 的 L1/M1/R1）-> 等动作结束 -> 回空状态。
 *
 * 为什么轮流切模式：fabric 里的三色判定前端（rgb_colour_frontend_r14）一次只算
 * 一种颜色（按当前 mode 选 R/G/B 通道），所以同时只能拿到一种颜色的面积。
 * 轮流切 3 个模式复用同一套 CCA，几乎不额外占逻辑资源；代价是分类期间
 * 屏幕会依次显示三种检测画面（约 300 ms）。
 *
 * 判定规则（在 MCU 里算，fabric 只提供面积）：
 *   max = 三个面积里最大的，second = 次大
 *   1) max <  CLS_AREA_MIN            -> 空状态（画面里没有足够大的色块）
 *   2) 2*max >= 3*second              -> 判为 max 对应的颜色（即高 50% 以上）
 *   3) 否则                            -> 空状态（不满足支配关系）
 *
 * 颜色 -> 动作：红 = L1（底座左转），绿 = M1（只摆顶部），蓝 = R1（底座右转）。
 *
 * 第二条路径（2026-11-02，UART0 收到 "S_RUN"）：形状分类。
 *   切到模式 12（白底黑图案：圆/三角/方）-> 等 CLS_SHAPE_SETTLE_MS ->
 *   每 CLS_SHAPE_POLL_MS 轮询一次 AHB 0x34（类别 + 有效位 + 目标个数）->
 *   判出类别 -> 同样调 L1/M1/R1 -> 等动作结束 -> 恢复原模式 -> 回空状态。
 *   形状 -> 动作：方(RECT) = L1，三角(TRI) = M1，圆(CIRCLE) = R1。
 *   也就是说三种形状和三色块共用同一套云台动作，只是判别来源不同。
 *
 *   ⚠ 硬件现在同时识别最多 3 个图案（每个都会在屏上画出框和中文标签）。
 *     0x34 的 bit[4:3] 报出“画面上有几个”，超过 CLS_SHAPE_MAX_COUNT(=1) 时
 *     一律拒绝动作并打印 cls: shape ambiguous (N targets)，因为不知道该分谁。
 *     所以 S_RUN 之前要保证画面里只有一个图案。
 *
 * 面积口径：AHB 0x2C = 当前模式下**最大连通块**面积（像素数），
 *           没有目标时为 0。由 CCA 输出，已带最小面积/宽高过滤。
 */

#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <stdint.h>

/* ---------------- 视频模式（与 RTL 一致） ---------------- */
#define CLS_MODE_RED        6u      /* 0110 红色检测 */
#define CLS_MODE_BLUE       8u      /* 1000 蓝色检测 */
#define CLS_MODE_GREEN      10u     /* 1010 绿色检测 */
#define CLS_MODE_SHAPE      12u     /* 1100 形状分类（白底黑图案：圆/三角/方） */

/* ---------------- 判定参数 ---------------- */
#define CLS_AREA_MIN        1000u   /* 三个面积都低于此值 -> 空状态 */
#define CLS_DOM_X2          3u      /* 2*max >= CLS_DOM_X2*second 即“高 50%” */

#define CLS_SETTLE_MS       100u    /* 切模式后等面积稳定（60 fps 约 6 帧） */
#define CLS_ACT_GUARD_MS    50u     /* 下发动作后先给状态机一点时间进 RUN */
#define CLS_TIMEOUT_MS      12000u  /* 单次分类总超时（含一整套动作） */

/* ---------------- 形状参数（S_RUN） ----------------
 * 形状引擎在 fabric 里做完整判定（黑掩码 -> 连通块 -> 几何分类 -> 3 帧时域确认），
 * MCU 只轮询 AHB 0x34 的整字节结果：
 *   bit[1:0] = 最大确定目标的类别
 *   bit2     = 该类别已被三帧确认
 *   bit[4:3] = 画面上已确认的目标个数（0~3）
 * 所以这里不需要面积阈值，只需要“等多久”和“多久问一次”。 */
#define CLS_SHAPE_SETTLE_MS 300u    /* 切到形状模式后先等：3 帧确认 + 9 拍流水 + 取块 */
#define CLS_SHAPE_POLL_MS   20u     /* 之后每 20 ms 问一次 0x34 */
#define CLS_SHAPE_WAIT_MS   1000u   /* 总共最多等 1 s，超时判“没有合格图案” */

/* 0x34 位域 */
#define CLS_SH_REG_VALID    0x04u   /* bit2：类别已确认 */
#define CLS_SH_REG_CLASS(v) ((uint8_t)((v) & 0x03u))
#define CLS_SH_REG_COUNT(v) ((uint8_t)(((v) >> 3) & 0x03u))

/* 单次分拣最多容忍几个目标：>1 视为“画面不干净”，拒绝动作 */
#define CLS_SHAPE_MAX_COUNT 1u

/* ---------------- 结果 ---------------- */
#define CLS_NONE            0u
#define CLS_RED             1u
#define CLS_GREEN           2u
#define CLS_BLUE            3u

/* 形状类别（与 RTL 0x34 的 bit[1:0] 同编码，直接照抄硬件值） */
#define CLS_SHAPE_NONE      0u
#define CLS_SHAPE_CIRCLE    1u
#define CLS_SHAPE_TRI       2u
#define CLS_SHAPE_RECT      3u

/* ---------------- 动作（与 motor_master.h 的 MM_SEQ_xxx 对应） ---------------- */
#define CLS_ACT_NONE        0u
#define CLS_ACT_L1          1u
#define CLS_ACT_M1          2u
#define CLS_ACT_R1          3u

/* ---------------- 状态 ---------------- */
#define CLS_ST_EMPTY        0u      /* 空状态：等 RUN / S_RUN */
#define CLS_ST_SWEEP        1u      /* 正在采面积（三色路径） */
#define CLS_ST_ACT          2u      /* 动作执行中 */
#define CLS_ST_SHAPE        3u      /* 正在等形状类别（S_RUN 路径） */

/* 端口：把硬件相关的东西留在这里，模块本身不直接碰寄存器 */
typedef struct {
    uint32_t (*now_ms)(void *ctx);
    uint8_t  (*read_mode)(void *ctx);        /* 当前视频模式 0~15 */
    void     (*set_mode)(uint8_t mode, void *ctx);
    uint32_t (*read_area)(void *ctx);        /* 当前模式下的面积，无目标返回 0 */
    void     (*post_action)(uint8_t act, void *ctx);
    uint8_t  (*busy)(void *ctx);             /* 1 = 电机动作进行中 */
    void     (*log)(const char *s, void *ctx);
    uint8_t  (*read_shape)(void *ctx);       /* 0x34 整字节原样：类别+有效+目标个数 */
} cls_port_t;

void     CLS_Init(const cls_port_t *port, void *ctx);

/* 每 1 ms 调一次 */
void     CLS_Tick(void);

/* 触发一次分类（非阻塞）。动作进行中或已在分类时会忽略 */
void     CLS_Run(void);                      /* 三色面积分类（C_RUN / 裸 RUN） */
void     CLS_RunShape(void);                 /* 形状分类（S_RUN） */

uint8_t  CLS_State(void);
uint8_t  CLS_Result(void);                   /* 最近一次判定结果 CLS_xxx */
uint8_t  CLS_Shape(void);                    /* 最近一次形状结果 CLS_SHAPE_xxx */
uint32_t CLS_Area(uint8_t colour);           /* 最近一次的三个面积，colour=CLS_xxx */

#endif /* CLASSIFIER_H */
