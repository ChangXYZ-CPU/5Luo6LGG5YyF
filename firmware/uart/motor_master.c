/**
 * motor_master.c -- 上位机侧二维转台状态机（步骤表驱动）
 *
 * 结构：
 *   1. 角度换算               mm_pulses()
 *   2. Emm_V5 命令字节构造     mm_data_en / mm_data_pos / mm_data_stop / mm_data_unclog
 *   3. AA55 帧封装与发送       mm_send_motors / mm_send_pos
 *   4. 上行解析（过滤回显/打印，识别到位码，识别 L1/M1/R1）
 *   5. 步骤表与状态机
 */

#include "motor_master.h"

/* ------------------------------------------------------------------ */
/* 1. 角度换算                                                         */
/* ------------------------------------------------------------------ */

#if !(MM_USE_SIMPLE_CMD)
/* 3200 脉冲/圈、直连：脉冲 = 3200 * deg / 360，四舍五入 */
static uint32_t mm_pulses(uint16_t deg)
{
    uint32_t v = (uint32_t)MM_PULSE_PER_REV * deg;
    return (v + 180u) / 360u;
}
#endif

/* ------------------------------------------------------------------ */
/* 2. Emm_V5 命令字节（data 段，不含地址/长度）                         */
/* ------------------------------------------------------------------ */

static uint8_t mm_data_en(uint8_t *d, uint8_t state, uint8_t snF)
{
    d[0] = 0xF3; d[1] = 0xAB; d[2] = state; d[3] = snF; d[4] = 0x6B;
    return 5u;
}

static uint8_t mm_data_stop(uint8_t *d, uint8_t snF)
{
    d[0] = 0xFE; d[1] = 0x98; d[2] = snF; d[3] = 0x6B;
    return 4u;
}

static uint8_t mm_data_unclog(uint8_t *d)
{
    d[0] = 0x0E; d[1] = 0x52; d[2] = 0x6B;      /* 解除堵转保护 */
    return 3u;
}

#if !(MM_USE_SIMPLE_CMD)
static uint8_t mm_data_pos(uint8_t *d, uint8_t dir, uint16_t vel, uint8_t acc,
                           uint32_t clk, uint8_t raF, uint8_t snF)
{
    d[0]  = 0xFD;
    d[1]  = dir;
    d[2]  = (uint8_t)(vel >> 8);
    d[3]  = (uint8_t)(vel);
    d[4]  = acc;
    d[5]  = (uint8_t)(clk >> 24);
    d[6]  = (uint8_t)(clk >> 16);
    d[7]  = (uint8_t)(clk >> 8);
    d[8]  = (uint8_t)(clk);
    d[9]  = raF;
    d[10] = snF;
    d[11] = 0x6B;
    return 12u;
}
#endif

/* ------------------------------------------------------------------ */
/* 3. 帧封装与发送                                                     */
/* ------------------------------------------------------------------ */

#define MM_MAX_MOTORS  2u
#define MM_MAX_DATA    20u

typedef struct
{
    uint8_t addr;
    uint8_t len;
    uint8_t data[MM_MAX_DATA];
} mm_motor_t;

static mm_tx_fn   s_tx;
static mm_now_fn  s_now;
static void      *s_ctx;
static mm_log_fn  s_log;

static void mm_send_motors(const mm_motor_t *m, uint8_t n)
{
    uint8_t  buf[64];
    uint16_t i = 0;
    uint8_t  k, j, sum = 0;

    if ((s_tx == 0) || (n == 0u) || (n > MM_MAX_MOTORS))
        return;

    buf[i++] = 0xAA;
    buf[i++] = 0x55;
    buf[i++] = n;

    for (k = 0u; k < n; k++)
    {
        buf[i++] = m[k].addr;
        buf[i++] = m[k].len;
        for (j = 0u; j < m[k].len; j++)
            buf[i++] = m[k].data[j];
    }

    /* sum: 从 N 累加到最后一个数据字节（不含 AA 55，不含自身） */
    for (j = 2u; j < i; j++)
        sum = (uint8_t)(sum + buf[j]);
    buf[i++] = sum;

    if (s_log) s_log("TX", buf, (uint8_t)i);
    s_tx(buf, i, s_ctx);
}

static uint8_t  s_guard_axis  = 0u;      /* 刚发过命令的轴 */
static uint32_t s_guard_until = 0u;      /* 在此之前收到的同轴到位码视为回显 */

/* 命令一条轴转 90° */
static void mm_send_move(uint8_t axis, uint8_t dir)
{
#if (MM_USE_SIMPLE_CMD)
    /* 下位机封装的 2 字节命令: [轴号] [方向 0=CW 1=CCW]，必须连续两字节 */
    uint8_t b[2];
    b[0] = axis;
    b[1] = dir;
    if (s_log) s_log("TX", b, 2u);
    s_tx(b, 2u, s_ctx);

    s_guard_axis  = axis;
    s_guard_until = (s_now ? s_now(s_ctx) : 0u) + MM_ECHO_GUARD_MS;
#else
    mm_motor_t m;
    m.addr = axis;
    m.len  = mm_data_pos(m.data, dir, MM_VEL_RPM, MM_ACC, mm_pulses(MM_ANGLE_DEG), MM_RAF, 0u);
    mm_send_motors(&m, 1u);

    /* 帧模式下回显是整帧、不会误判到位，但启动瞬间的电气干扰可能让下位机
     * 立刻报一次到位，这里用"最短运动时间"把它挡掉 */
    s_guard_axis  = axis;
    s_guard_until = (s_now ? s_now(s_ctx) : 0u) + MM_MIN_ARRIVAL_MS;
#endif
}

/* 广播：kind = MM_SEQ_STOP 为停止，MM_SEQ_UNCLOG 为解除堵转，其它为使能 */
static void mm_send_broadcast(uint8_t kind)
{
    mm_motor_t m;
    m.addr = 0u;
    if (kind == MM_SEQ_STOP)
        m.len = mm_data_stop(m.data, 0u);
    else if (kind == MM_SEQ_UNCLOG)
        m.len = mm_data_unclog(m.data);
    else
        m.len = mm_data_en(m.data, 1u, 0u);
    mm_send_motors(&m, 1u);
}

/* ------------------------------------------------------------------ */
/* 4. 上行解析                                                         */
/*                                                                     */
/* 三种上行内容混在一起，必须分开处理：                                 */
/*   a) 整帧回显 AA 55 ...：收满定长再丢（同时作为链路存活证据）          */
/*   b) ASCII 调试打印 [INIT]/[帧OK]/[CMD]：只在行首认 L1/M1/R1，其余丢弃  */
/*   c) 2 字节到位码 01 01 / 02 01                                        */
/* ------------------------------------------------------------------ */

#define RX_BUF_MAX 40u

enum { RX_NORM = 0, RX_CODE1 };

static uint8_t  s_rx_st    = RX_NORM;
static uint8_t  s_rx_motor = 0;
static uint8_t  s_rxbuf[RX_BUF_MAX];
static uint8_t  s_rxn      = 0;

static uint8_t  s_pos_h    = 0;     /* 水平到位（一次性） */
static uint8_t  s_pos_t    = 0;     /* 顶部到位（一次性） */
static uint8_t  s_link_ok  = 0;     /* 收到过完整回显 = 从机主循环在跑 */

/* 行首命令识别：只认行首的 L1/M1/R1，避免把打印里的 "M1" 当成命令 */
static uint8_t  s_line_start = 1u;
static uint8_t  s_cmd[3];
static uint8_t  s_cmdn       = 0u;

static volatile uint8_t s_post = MM_SEQ_NONE;

static uint8_t rx_expected(void)
{
    uint8_t i, k, n;

    if (s_rxn < 3u)
        return 0u;

    n = s_rxbuf[2];
    if ((n == 0u) || (n > 2u))
        return 0xFFu;

    i = 3u;
    for (k = 0u; k < n; k++)
    {
        if ((uint8_t)(i + 2u) > s_rxn)
            return 0u;
        i = (uint8_t)(i + 2u + s_rxbuf[i + 1u]);
        if (i > RX_BUF_MAX)
            return 0xFFu;
    }
    return (uint8_t)(i + 1u);
}

static uint8_t mm_cmd_to_seq(uint8_t c0, uint8_t c1)
{
    uint8_t seq = MM_SEQ_NONE;

    if (c1 != '1')
        return MM_SEQ_NONE;
    if (c0 >= 'a' && c0 <= 'z')
        c0 = (uint8_t)(c0 - 'a' + 'A');

    if (c0 == 'L')      seq = MM_SEQ_L1;
    else if (c0 == 'M') seq = MM_SEQ_M1;
    else if (c0 == 'R') seq = MM_SEQ_R1;

    return seq;
}

void MM_RxByte(uint8_t b)
{
    uint8_t exp;

    /* ---- a) 回显帧收集中 ---- */
    if (s_rxn > 0u)
    {
        if (s_rxn >= RX_BUF_MAX)
        {
            s_rxn = 0u;
            return;
        }
        s_rxbuf[s_rxn++] = b;

        exp = rx_expected();
        if (exp == 0xFFu)
        {
            s_rxn = 0u;
        }
        else if ((exp != 0u) && (s_rxn >= exp))
        {
            s_rxn        = 0u;
            s_link_ok    = 1u;
            s_line_start = 1u;
        }
        return;
    }

    /* ---- 行结束 ---- */
    if (b == '\r' || b == '\n')
    {
        s_cmdn       = 0u;
        s_line_start = 1u;
        s_rx_st      = RX_NORM;
        return;
    }

    /* ---- b) ASCII 行：只认行首的 "!L1" / "!M1" / "!R1" ---- */
    if (b >= 0x20u && b < 0x7Fu)
    {
        s_rx_st = RX_NORM;
        if (!s_line_start)
            return;                     /* 行中间的内容一律忽略 */

        s_cmd[s_cmdn++] = b;

        if (s_cmdn == 1u)
        {
            if (b != MM_CMD_PREFIX)     /* 不是命令前缀：整行忽略 */
            {
                s_cmdn       = 0u;
                s_line_start = 0u;
            }
        }
        else if (s_cmdn >= 3u)          /* '!' + 两个字符 */
        {
            uint8_t seq = mm_cmd_to_seq(s_cmd[1], s_cmd[2]);
            s_cmdn = 0u;
            if (seq != MM_SEQ_NONE)
            {
                s_post       = seq;
                s_line_start = 1u;      /* 允许连发 "!L1!R1" */
            }
            else
            {
                s_line_start = 0u;
            }
        }
        return;
    }

    /* ---- c) 二进制 2 字节到位码 ---- */
    switch (s_rx_st)
    {
        case RX_NORM:
            if (b == 0xAAu)
            {
                s_rxbuf[0] = 0xAAu;
                s_rxn      = 1u;
            }
            else if (b == 0x01u || b == 0x02u)
            {
                s_rx_motor = b;
                s_rx_st    = RX_CODE1;
            }
            break;

        case RX_CODE1:
            if (b == 0x01u)
            {
                uint32_t now = s_now ? s_now(s_ctx) : 0u;
                uint8_t  ax  = (s_rx_motor == 0x01u) ? 1u : 2u;

                /* 2 字节命令的回显与“到位”同字节：刚发出 15 ms 内收到的同轴码
                 * 一定是自己的回显（回显 1~3 ms 回来，转动到位最少 40 ms） */
                if ((ax == s_guard_axis) && ((int32_t)(now - s_guard_until) < 0))
                {
                    /* 回显或启动干扰，忽略 */
                    if (s_log) s_log("SPUR", s_rxbuf, 2u);
                }
                else
                {
                    uint8_t b2[2];
                    b2[0] = s_rx_motor;
                    b2[1] = 0x01u;
                    if (s_log) s_log("POS", b2, 2u);

                    if (ax == 1u) s_pos_h = 1u;
                    else          s_pos_t = 1u;
                }
            }
            s_rx_st = RX_NORM;
            break;

        default:
            s_rx_st = RX_NORM;
            break;
    }
}

/* ------------------------------------------------------------------ */
/* 5. 步骤表与状态机                                                   */
/* ------------------------------------------------------------------ */

enum {
    OP_MOVE_H = 1,      /* arg = dir */
    OP_WAIT_H,          /* 保留：等水平到位码（当前不用） */
    OP_MOVE_T,          /* arg = dir */
    OP_WAIT_T,          /* 保留：等顶部到位码（当前不用） */
    OP_HOLD,            /* 保留 */
    OP_DELAY,           /* 固定等待 MM_STEP_DELAY_MS */
    OP_END
};

typedef struct
{
    uint8_t op;
    uint8_t arg;
} mm_op_t;

/* L1：底座左转 -> 1 s -> 顶部右转 -> 1 s -> 顶部左转回 -> 1 s -> 底座右转归位 */
static const mm_op_t P_L1[] =
{
    { OP_MOVE_H, MM_DIR_H_LEFT  }, { OP_DELAY,  0u },
    { OP_MOVE_T, MM_DIR_T_RIGHT }, { OP_DELAY,  0u },
    { OP_MOVE_T, MM_DIR_T_LEFT  }, { OP_DELAY,  0u },
    { OP_MOVE_H, MM_DIR_H_RIGHT },
    { OP_END,    0u              }
};

/* M1：只做顶部摆动（当前未使用，保留） */
static const mm_op_t P_M1[] =
{
    { OP_MOVE_T, MM_DIR_T_RIGHT }, { OP_DELAY,  0u },
    { OP_MOVE_T, MM_DIR_T_LEFT  },
    { OP_END,    0u              }
};

/* R1：底座右转 -> 1 s -> 顶部右转 -> 1 s -> 顶部左转回 -> 1 s -> 底座左转归位 */
static const mm_op_t P_R1[] =
{
    { OP_MOVE_H, MM_DIR_H_RIGHT }, { OP_DELAY,  0u },
    { OP_MOVE_T, MM_DIR_T_RIGHT }, { OP_DELAY,  0u },
    { OP_MOVE_T, MM_DIR_T_LEFT  }, { OP_DELAY,  0u },
    { OP_MOVE_H, MM_DIR_H_LEFT  },
    { OP_END,    0u              }
};

/* IDLE：每次只让底座转 90°，左右交替。
 * 第一次左转（转过去），第二次右转（转回来），第三次又左转…… */
static const mm_op_t P_IDLE_L[] =
{
    { OP_MOVE_H, MM_DIR_H_LEFT  }, { OP_DELAY, 0u },
    { OP_END,    0u             }
};

static const mm_op_t P_IDLE_R[] =
{
    { OP_MOVE_H, MM_DIR_H_RIGHT }, { OP_DELAY, 0u },
    { OP_END,    0u             }
};

static uint8_t        s_idle_left = 1u;   /* 1 = 下一次 IDLE 左转 */

static uint8_t        s_st;
static uint32_t       s_t_enter;
static uint32_t       s_t_boot;
static uint8_t        s_fault;

static const mm_op_t *s_prog;
static uint8_t        s_pc;

static const mm_op_t *mm_prog_for(uint8_t seq)
{
    switch (seq)
    {
        case MM_SEQ_L1: return P_L1;
        case MM_SEQ_M1: return P_M1;
        case MM_SEQ_R1: return P_R1;
        default:        return 0;
    }
}

static void mm_enter(uint8_t st)
{
    s_st      = st;
    s_t_enter = s_now ? s_now(s_ctx) : 0u;
}

static uint8_t mm_elapsed(uint32_t now, uint32_t limit_ms)
{
    return ((uint32_t)(now - s_t_enter) >= limit_ms) ? 1u : 0u;
}

static void mm_fault(void)
{
    s_fault = 1u;
    mm_enter(MM_ST_FAULT);
}

void MM_Init(mm_tx_fn tx, mm_now_fn now, void *ctx)
{
    s_tx  = tx;
    s_now = now;
    s_ctx = ctx;

    s_pos_h = s_pos_t = 0u;
    s_fault = 0u;
    s_post  = MM_SEQ_NONE;
    s_rxn   = 0u;
    s_link_ok    = 0u;
    s_line_start = 1u;
    s_cmdn       = 0u;
    s_prog       = 0;
    s_pc         = 0u;
    s_t_boot     = s_now ? s_now(s_ctx) : 0u;
    s_guard_axis  = 0u;
    s_guard_until = 0u;

    mm_enter(MM_ST_INIT);
}

void MM_SetLog(mm_log_fn fn)
{
    s_log = fn;
}

void MM_PostAction(uint8_t act)
{
    s_post = act;
}

uint8_t MM_State(void) { return s_st; }

int8_t MM_Step(void) { return (s_st == MM_ST_RUN) ? (int8_t)s_pc : (int8_t)-1; }

uint8_t MM_TakeFault(void)
{
    uint8_t f = s_fault;
    s_fault = 0u;
    return f;
}

void MM_Reset(void)
{
    s_pos_h = s_pos_t = 0u;
    s_fault = 0u;
    s_post  = MM_SEQ_NONE;
    s_rxn   = 0u;
    s_link_ok    = 0u;
    s_line_start = 1u;
    s_cmdn       = 0u;
    s_prog       = 0;
    s_pc         = 0u;
    s_idle_left  = 1u;                 /* 复位后第一次 IDLE 仍是左转 */
    s_t_boot     = s_now ? s_now(s_ctx) : 0u;
    mm_enter(MM_ST_INIT);
}

static void mm_start(uint8_t act)
{
    if (act == MM_SEQ_STOP)
    {
        mm_send_broadcast(MM_SEQ_STOP);
        mm_fault();
        return;
    }
    if (act == MM_SEQ_UNCLOG)
    {
        mm_send_broadcast(MM_SEQ_UNCLOG);
        return;
    }

    if (act == MM_SEQ_IDLE)
    {
        s_prog      = s_idle_left ? P_IDLE_L : P_IDLE_R;
        s_idle_left = s_idle_left ? 0u : 1u;      /* 下一次反向 */
    }
    else
    {
        s_prog = mm_prog_for(act);
    }
    if (s_prog == 0)
        return;

    s_pc    = 0u;
    s_pos_h = s_pos_t = 0u;
    mm_enter(MM_ST_RUN);
}

/* 执行当前一步；每次调用最多推进一步 */
static void mm_run(uint32_t now)
{
    const mm_op_t *op = &s_prog[s_pc];

    switch (op->op)
    {
        case OP_MOVE_H:
            s_pos_h = 0u;
            mm_send_move(1u, op->arg);
            s_pc++;
            mm_enter(MM_ST_RUN);        /* 重置本步超时基准 */
            break;

        case OP_WAIT_H:
            if (s_pos_h)
            {
                s_pos_h = 0u;
                s_pc++;
                if (s_log) s_log("H-ARRIVED", 0, 0u);
                mm_enter(MM_ST_RUN);
            }
            else if (mm_elapsed(now, MM_T_MOVE_MS))
            {
                if (s_log) s_log("H-TIMEOUT", 0, 0u);
                if (MM_TIMEOUT_AS_DONE) { s_pc++; mm_enter(MM_ST_RUN); }
                else                    { mm_fault(); }
            }
            break;

        case OP_MOVE_T:
            s_pos_t = 0u;
            mm_send_move(2u, op->arg);
            s_pc++;
            mm_enter(MM_ST_RUN);
            break;

        case OP_WAIT_T:
            if (s_pos_t)
            {
                s_pos_t = 0u;
                s_pc++;
                if (s_log) s_log("T-ARRIVED", 0, 0u);
                mm_enter(MM_ST_RUN);
            }
            else if (mm_elapsed(now, MM_T_MOVE_MS))
            {
                if (s_log) s_log("T-TIMEOUT", 0, 0u);
                if (MM_TIMEOUT_AS_DONE) { s_pc++; mm_enter(MM_ST_RUN); }
                else                    { mm_fault(); }
            }
            break;

        case OP_HOLD:
            if (mm_elapsed(now, MM_HOLD_MS))
            {
                s_pc++;
                mm_enter(MM_ST_RUN);
            }
            break;

        case OP_DELAY:
            if (mm_elapsed(now, MM_STEP_DELAY_MS))
            {
                s_pc++;
                mm_enter(MM_ST_RUN);
            }
            break;

        case OP_END:
        default:
            s_prog = 0;
            mm_enter(MM_ST_DONE);
            break;
    }
}

void MM_Tick(void)
{
    uint32_t now = s_now ? s_now(s_ctx) : 0u;
    uint8_t  act = s_post;
    s_post = MM_SEQ_NONE;

    if (act != MM_SEQ_NONE)
    {
        /* 急停/解除堵转随时可执行；动作要等从机就绪或在空闲/完成/故障态 */
        if ((act == MM_SEQ_STOP) || (act == MM_SEQ_UNCLOG))
            mm_start(act);
        else if ((s_st == MM_ST_IDLE) || (s_st == MM_ST_DONE) || (s_st == MM_ST_FAULT))
        {
            mm_start(act);
        }
        else if (s_log)
        {
            s_log("NOTREADY", 0, 0u);       /* 从机未就绪/正在执行，本次命令被忽略 */
        }
        return;
    }

    switch (s_st)
    {
        case MM_ST_INIT:
#if !(MM_HANDSHAKE_EN)
            /* 不握手：直接进入可接收命令状态（从机上电自己会回零+使能） */
            mm_enter(MM_ST_IDLE);
            break;
#endif
            if (s_link_ok)
            {
                s_link_ok = 0u;
                mm_enter(MM_ST_IDLE);
            }
            else if ((uint32_t)(now - s_t_boot) >= MM_T_BOOT_MS)
            {
                mm_fault();                 /* 15 s 没回显：链路/从机异常 */
            }
            else if (mm_elapsed(now, MM_T_EN_RETRY_MS))
            {
                mm_send_broadcast(MM_SEQ_NONE);   /* 广播使能，兼作探测 */
                mm_enter(MM_ST_INIT);
            }
            break;

        case MM_ST_IDLE:
            break;

        case MM_ST_RUN:
            mm_run(now);
            break;

        case MM_ST_DONE:
            mm_enter(MM_ST_IDLE);
            break;

        case MM_ST_FAULT:
            break;

        default:
            mm_enter(MM_ST_IDLE);
            break;
    }
}
