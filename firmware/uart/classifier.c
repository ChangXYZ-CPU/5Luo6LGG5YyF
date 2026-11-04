/**
 * classifier.c -- 三色面积分类状态机（实现见 classifier.h）
 *
 * 说明：整个判定都在 MCU 里做，fabric 只提供 AHB 0x2C 的面积读数。
 */

#include "classifier.h"
#include <stddef.h>

static const cls_port_t *s_p   = 0;
static void             *s_ctx = 0;

static uint8_t  s_st;           /* CLS_ST_xxx */
static uint8_t  s_idx;          /* 0=红 1=绿 2=蓝，当前扫到哪个 */
static uint8_t  s_run_req;      /* RUN/C_RUN 请求（由串口置位） */
static uint8_t  s_run_shape_req;/* S_RUN 请求（由串口置位） */
static uint8_t  s_result;       /* 最近一次判定 */
static uint8_t  s_shape;        /* 最近一次形状类别 CLS_SHAPE_xxx */
static uint8_t  s_orig_mode;    /* 分类前的工作模式，结束后恢复 */
static uint32_t s_t;            /* 当前阶段的起始时刻 */
static uint32_t s_poll_t;       /* 形状路径上一次轮询的时刻 */
static uint32_t s_area[3];      /* 0=红 1=绿 2=蓝 */

static const uint8_t k_mode[3] = { CLS_MODE_RED, CLS_MODE_GREEN, CLS_MODE_BLUE };

/* ------------------------------------------------------------------ */
/* 小工具：无 libc 的十进制输出（内核态环境不保证有 sprintf）           */
/* ------------------------------------------------------------------ */
static uint8_t u32_dec(char *dst, uint32_t v)
{
    char    tmp[10];
    uint8_t n = 0u, i;

    if (v == 0u)
    {
        dst[0] = '0';
        return 1u;
    }
    while (v != 0u)
    {
        tmp[n++] = (char)('0' + (uint8_t)(v % 10u));
        v /= 10u;
    }
    for (i = 0u; i < n; i++)
        dst[i] = tmp[n - 1u - i];
    return n;
}

static void cls_log(const char *s)
{
    if (s_p && s_p->log)
        s_p->log(s, s_ctx);
}

static void log_areas(void)
{
    char    b[44];
    uint8_t n = 0u;

    if (!s_p->log)
        return;

    b[n++] = 'R'; b[n++] = '='; n = (uint8_t)(n + u32_dec(&b[n], s_area[0]));
    b[n++] = ' '; b[n++] = 'G'; b[n++] = '='; n = (uint8_t)(n + u32_dec(&b[n], s_area[1]));
    b[n++] = ' '; b[n++] = 'B'; b[n++] = '='; n = (uint8_t)(n + u32_dec(&b[n], s_area[2]));
    b[n]   = '\0';
    s_p->log(b, s_ctx);
}

/* ------------------------------------------------------------------ */
/* 判定：max >= CLS_AREA_MIN 且 2*max >= 3*second                      */
/* ------------------------------------------------------------------ */
static uint8_t cls_decide(uint8_t *mi_out)
{
    uint8_t  mi = 0u, si = 1u;
    uint32_t max, sec;

    if (s_area[1] > s_area[0]) { mi = 1u; si = 0u; }
    if (s_area[2] > s_area[mi]) { si = mi; mi = 2u; }
    else if (s_area[2] > s_area[si]) { si = 2u; }

    max = s_area[mi];
    sec = s_area[si];

    if (mi_out)
        *mi_out = mi;

    if (max < CLS_AREA_MIN)
        return CLS_NONE;                        /* 三个都不够大 */

    if ((max * 2u) < (sec * CLS_DOM_X2))
        return CLS_NONE;                        /* 没有明显占优的颜色 */

    return (mi == 0u) ? CLS_RED : ((mi == 1u) ? CLS_GREEN : CLS_BLUE);
}

static uint8_t cls_act_of(uint8_t colour)
{
    if (colour == CLS_RED)   return CLS_ACT_L1;   /* 红：底座左转 */
    if (colour == CLS_GREEN) return CLS_ACT_M1;   /* 绿：只摆顶部 */
    if (colour == CLS_BLUE)  return CLS_ACT_R1;   /* 蓝：底座右转 */
    return CLS_ACT_NONE;
}

/* 形状 -> 动作（2026-11-02）：方 = L1 左，三角 = M1 中，圆 = R1 右。
 * 三种形状与三色块共用同一套动作，所以直接复用 CLS_ACT_L1/M1/R1。 */
static uint8_t cls_act_of_shape(uint8_t shape)
{
    if (shape == CLS_SHAPE_RECT)   return CLS_ACT_L1;   /* 方：底座左转 */
    if (shape == CLS_SHAPE_TRI)    return CLS_ACT_M1;   /* 三角：只摆顶部 */
    if (shape == CLS_SHAPE_CIRCLE) return CLS_ACT_R1;   /* 圆：底座右转 */
    return CLS_ACT_NONE;
}

static void cls_restore(void)
{
    if (s_p->set_mode)
        s_p->set_mode(s_orig_mode, s_ctx);
}

/* 「cls: shape ambiguous (2 targets)」——个数只有 2~3，不用 sprintf。 */
static void cls_shape_ambiguous(uint8_t n)
{
    /* 预留 40 字节，实际用 26；不用 strcpy，避免把 libc 拉进来。 */
    char b[40];
    const char *pre = "cls: shape ambiguous (";
    const char *suf = " targets)";
    uint8_t i = 0u;

    while (*pre)
        b[i++] = *pre++;
    b[i++] = (char)('0' + (n > 9u ? 9u : n));
    while (*suf)
        b[i++] = *suf++;
    b[i] = '\0';
    cls_log(b);
}

/* ------------------------------------------------------------------ */
/* 对外接口                                                            */
/* ------------------------------------------------------------------ */
void CLS_Init(const cls_port_t *port, void *ctx)
{
    s_p        = port;
    s_ctx      = ctx;
    s_st       = CLS_ST_EMPTY;
    s_idx      = 0u;
    s_run_req  = 0u;
    s_run_shape_req = 0u;
    s_result   = CLS_NONE;
    s_shape    = CLS_SHAPE_NONE;
    s_orig_mode = 0u;
    s_t        = 0u;
    s_poll_t   = 0u;
    s_area[0]  = s_area[1] = s_area[2] = 0u;
}

void CLS_Run(void)
{
    s_run_req = 1u;
}

void CLS_RunShape(void)
{
    s_run_shape_req = 1u;
}

uint8_t CLS_State(void)
{
    return s_st;
}

uint8_t CLS_Result(void)
{
    return s_result;
}

uint8_t CLS_Shape(void)
{
    return s_shape;
}

uint32_t CLS_Area(uint8_t colour)
{
    if (colour == CLS_RED)   return s_area[0];
    if (colour == CLS_GREEN) return s_area[1];
    if (colour == CLS_BLUE)  return s_area[2];
    return 0u;
}

void CLS_Tick(void)
{
    uint32_t now;

    if (!s_p)
        return;

    now = s_p->now_ms(s_ctx);

    switch (s_st)
    {
    case CLS_ST_EMPTY:
        if (!s_run_req && !s_run_shape_req)
            break;

        /* 电机正在动作时不受理：motor_master 会丢弃这条动作 */
        if (s_p->busy && s_p->busy(s_ctx))
        {
            s_run_req = 0u;
            s_run_shape_req = 0u;
            cls_log("cls: busy, RUN ignored");
            break;
        }

        s_orig_mode = s_p->read_mode ? s_p->read_mode(s_ctx) : 0u;

        /* ---- 形状路径（S_RUN）：只切一次模式，然后轮询类别寄存器 ---- */
        if (s_run_shape_req)
        {
            s_run_shape_req = 0u;
            s_run_req       = 0u;
            s_shape         = CLS_SHAPE_NONE;
            s_poll_t        = now;

            cls_log("cls: shape start");
            if (s_p->set_mode)
                s_p->set_mode(CLS_MODE_SHAPE, s_ctx);
            s_t  = now;
            s_st = CLS_ST_SHAPE;
            break;
        }

        /* ---- 三色路径（C_RUN / 裸 RUN）：轮流切三种颜色各采一次面积 ---- */
        s_run_req = 0u;
        s_result = CLS_NONE;
        s_area[0] = s_area[1] = s_area[2] = 0u;
        s_idx = 0u;

        cls_log("cls: start");
        if (s_p->set_mode)
            s_p->set_mode(k_mode[0], s_ctx);
        s_t  = now;
        s_st = CLS_ST_SWEEP;
        break;

    case CLS_ST_SHAPE:
        {
            uint32_t el = (uint32_t)(now - s_t);
            uint8_t  raw;
            uint8_t  sh;
            uint8_t  act;

            if (el >= CLS_SHAPE_WAIT_MS)
            {
                cls_log("cls: shape none");
                cls_restore();
                s_st = CLS_ST_EMPTY;
                break;
            }
            if (el < CLS_SHAPE_SETTLE_MS)
                break;
            if ((uint32_t)(now - s_poll_t) < CLS_SHAPE_POLL_MS)
                break;
            s_poll_t = now;

            raw = s_p->read_shape ? s_p->read_shape(s_ctx) : 0u;
            if ((raw & CLS_SH_REG_VALID) == 0u)
                break;                  /* 还没三帧确认（或本就无目标），20 ms 后再问 */

            /* 三目标引擎会同时报出画面里的图案个数。多于一个时不知道该分谁，
             * 直接拒绝动作并让操作者把画面清干净，避免把别的东西分走。 */
            if (CLS_SH_REG_COUNT(raw) > CLS_SHAPE_MAX_COUNT)
            {
                cls_shape_ambiguous(CLS_SH_REG_COUNT(raw));
                cls_restore();
                s_st = CLS_ST_EMPTY;
                break;
            }

            sh = CLS_SH_REG_CLASS(raw);
            s_shape = sh;
            act = cls_act_of_shape(sh);
            if (act == CLS_ACT_NONE)
            {
                cls_log("cls: shape unknown");
                cls_restore();
                s_st = CLS_ST_EMPTY;
                break;
            }

            cls_log((act == CLS_ACT_L1) ? "cls: RECT -> L1"
                    : (act == CLS_ACT_M1) ? "cls: TRI -> M1" : "cls: CIRCLE -> R1");

            if (s_p->post_action)
                s_p->post_action(act, s_ctx);

            s_t  = now;
            s_st = CLS_ST_ACT;
        }
        break;

    case CLS_ST_SWEEP:
        if ((uint32_t)(now - s_t) < CLS_SETTLE_MS)
            break;

        s_area[s_idx] = s_p->read_area ? s_p->read_area(s_ctx) : 0u;
        s_idx++;

        if (s_idx < 3u)
        {
            if (s_p->set_mode)
                s_p->set_mode(k_mode[s_idx], s_ctx);
            s_t = now;
            break;
        }

        /* 三个都采完了 -> 判定 */
        {
            uint8_t mi = 0u;
            uint8_t r  = cls_decide(&mi);

            log_areas();
            s_result = r;

            if (r == CLS_NONE)
            {
                cls_log("cls: idle (no dominant colour)");
                cls_restore();
                s_st = CLS_ST_EMPTY;
                break;
            }

            cls_log((r == CLS_RED) ? "cls: RED -> L1"
                    : (r == CLS_GREEN) ? "cls: GREEN -> M1" : "cls: BLUE -> R1");

            if (s_p->post_action)
                s_p->post_action(cls_act_of(r), s_ctx);

            s_t  = now;
            s_st = CLS_ST_ACT;
        }
        break;

    case CLS_ST_ACT:
        if ((uint32_t)(now - s_t) >= CLS_TIMEOUT_MS)
        {
            cls_log("cls: action timeout");
            cls_restore();
            s_st = CLS_ST_EMPTY;
            break;
        }
        /* 动作还没开始（post 后下一拍才进 RUN），先给它一点时间 */
        if ((uint32_t)(now - s_t) < CLS_ACT_GUARD_MS)
            break;
        if (s_p->busy && s_p->busy(s_ctx))
            break;

        cls_log("cls: done");
        cls_restore();
        s_st = CLS_ST_EMPTY;
        break;

    default:
        s_st = CLS_ST_EMPTY;
        break;
    }
}
