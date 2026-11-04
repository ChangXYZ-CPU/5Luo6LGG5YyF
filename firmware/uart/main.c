/*
 * 阶段 4：串口调 OSD（复用 K1/K2/K4 通路）
 *
 * 命令：
 *   J<n>   跳到第 n 项（n=0..12；10~12 可写 J10/J11/J12，也可写 JA/JB/JC）
 *   N      下一项（= 按 K4）
 *   +      加一步（= 按 K1）
 *   -      减一步（= 按 K2）
 *   M<n>   设置覆盖模式（阶段 3 保留）
 *   C      清除覆盖
 *   S      状态
 *   ?      帮助
 *   C_RUN  三色面积分类（红->L1 左转 / 绿->M1 只摆顶部 / 蓝->R1 右转）
 *   S_RUN  形状分类（方->L1 左转 / 三角->M1 只摆顶部 / 圆->R1 右转）
 *   裸 RUN 等价于 C_RUN（保留给旧脚本）
 *
 * 寄存器：0x30 OSD_CMD = {op[7:4], arg[3:0]}
 *   op: 1=跳项(arg=index) 2=下一项 3=加 4=减
 *          0x2C TARGET_AREA = 当前模式下最大连通块面积（C_RUN 用）
 *          0x34 SHAPE_CLASS = 形状类别 bit[1:0] + 有效位 bit2（S_RUN 用）
 *
 * ASCII only.
 */

#include <stdio.h>
#include <string.h>
#include "al_core.h"
#include "al_usart_hal.h"
#include "al_usart_ll.h"
#include "al_hwcfg.h"
#include "al_misc_ll.h"
#include "motor_master.h"
#include "classifier.h"

#define FAB_ID   0x50483150u

/* ---- 串口互通测试（临时调试）----
 * 1 = UART0(COM9) <-> UART1(U2/U1) 双向透传：任一口收到的字节立刻从另一口发出
 * 0 = 正常业务：串口屏命令 + 电机状态机(L1/M1/R1)
 * 测完改回 0 重新编译即可，其他代码不用动。 */
#define TEST_UART_BRIDGE   0u

/* ---- UART0 上"主动/周期性"输出开关（调试用；指令响应不在此列，始终保留）----
 * UART0_HB_EN    : 1=每秒打印 HB SW/MODE/SRC 心跳
 * UART0_STATUS_EN: 1=拨码/模式发生变化时自动打印一行状态
 * 接串口屏时可全部置 0，屏上就不会被这些自动输出刷屏。 */
#define UART0_HB_EN        0u
#define UART0_STATUS_EN    0u

static AL_USART_InitStruct usart_cfg;
static volatile unsigned g_fault = 0;
static unsigned g_base = 0;

__attribute__((interrupt("machine"), aligned(64)))
void trap_handler(void)
{
    unsigned epc, insn;
    g_fault++;
    __asm__ volatile("csrr %0, mepc" : "=r"(epc));
    insn = *(volatile unsigned *)epc;
    epc += ((insn & 0x3u) == 0x3u) ? 4u : 2u;
    __asm__ volatile("csrw mepc, %0" :: "r"(epc));
}

static unsigned srd(unsigned a)
{
    unsigned b = g_fault;
    unsigned v = *(volatile unsigned *)a;
    return (g_fault != b) ? 0xFFFFFFFFu : v;
}

static unsigned fab_rd(unsigned off) { return (g_base != 0u) ? srd(g_base + off) : 0u; }
static void     fab_wr(unsigned off, unsigned v) { if (g_base) *(volatile unsigned *)(g_base + off) = v; }

static unsigned uart_rx_poll(void)
{
    if (AlUsart_ll_GetRxFifoNum((AL_REG)USART0_BASE_ADDR) == 0u) return 0u;
    return (unsigned)(AlUsart_ll_RecvData((AL_REG)USART0_BASE_ADDR) & 0xFF);
}

/* ---------------- 电机链路：UART1 -> 下位机 STM32（115200 8N1） ----------------
 * 与串口屏的 UART0 完全分开：UART0 收 M0~M15 / OSD 命令，UART1 收 L1/M1/R1
 * 并向下位机发 AA55 帧、收它的回显与到位码。
 * 毫秒时基沿用固件原有标定：4000 次主循环 ≈ 1 ms（心跳 4e6 次 ≈ 1 s）。 */

static volatile unsigned s_ms_acc  = 0;
static volatile unsigned s_ms      = 0;
static unsigned          s_ms_seen = 0;

static void mm_tick_1ms(void)
{
    if (++s_ms_acc >= 4000u)
    {
        s_ms_acc = 0;
        s_ms++;
    }
}

static uint32_t motor_now(void *ctx)
{
    (void)ctx;
    return (uint32_t)s_ms;
}

/* 发一个字节：保持 FIFO 里最多 3 个待发，连续写的字节在线上无缝
 * （下位机的 2 字节命令要求两个字节落在同一次接收里） */
static void uart_putc(AL_REG base, uint8_t b)
{
    unsigned guard = 0u;
    while ((AlUsart_ll_GetTxFifoNum(base) >= 4u) && (++guard < 200000u))
    {
    }
    AlUsart_ll_SendData(base, (AL_U16)b);
}

/* 把状态机的收发记录打成文本，从 UART0（电脑/串口屏）输出，便于肉眼核对 */
static void motor_log(const char *tag, const uint8_t *buf, uint8_t len)
{
    static const char hx[] = "0123456789ABCDEF";
    uint8_t i;

    /* 收到下位机的到位码：只回一个 ok，方便肉眼确认 */
    if ((tag[0] == 'P') && (tag[1] == 'O') && (tag[2] == 'S'))
    {
        uart_putc((AL_REG)USART0_BASE_ADDR, 'o');
        uart_putc((AL_REG)USART0_BASE_ADDR, 'k');
        uart_putc((AL_REG)USART0_BASE_ADDR, '\r');
        uart_putc((AL_REG)USART0_BASE_ADDR, '\n');
        return;
    }

    while (*tag)
        uart_putc((AL_REG)USART0_BASE_ADDR, (uint8_t)*tag++);
    uart_putc((AL_REG)USART0_BASE_ADDR, ' ');
    for (i = 0; i < len; i++)
    {
        uart_putc((AL_REG)USART0_BASE_ADDR, (uint8_t)hx[buf[i] >> 4]);
        uart_putc((AL_REG)USART0_BASE_ADDR, (uint8_t)hx[buf[i] & 0x0Fu]);
        uart_putc((AL_REG)USART0_BASE_ADDR, ' ');
    }
    uart_putc((AL_REG)USART0_BASE_ADDR, '\r');
    uart_putc((AL_REG)USART0_BASE_ADDR, '\n');
}

static void motor_tx(const uint8_t *buf, uint16_t len, void *ctx)
{
    uint16_t i;
    (void)ctx;
    for (i = 0; i < len; i++)
        uart_putc((AL_REG)USART1_BASE_ADDR, buf[i]);
}

static void motor_poll(void)
{
    while (AlUsart_ll_GetRxFifoNum((AL_REG)USART1_BASE_ADDR) != 0u)
        MM_RxByte((uint8_t)(AlUsart_ll_RecvData((AL_REG)USART1_BASE_ADDR) & 0xFFu));
}

/* ---------------- 分类器端口：寄存器与电机接口的薄封装 ----------------
 * 面积寄存器 0x2C = 当前模式下最大连通块面积（像素数），0 表示没找到目标。
 * 判定逻辑全部在 classifier.c（MCU 侧），fabric 只提供这个数。 */

static uint8_t cls_read_mode(void *ctx)
{
    (void)ctx;
    return (uint8_t)(fab_rd(0x0Cu) & 0xFu);
}

static void cls_set_mode(uint8_t mode, void *ctx)
{
    (void)ctx;
    fab_wr(0x14u, (unsigned)mode);   /* OVR_MODE */
    fab_wr(0x18u, 0x1u);             /* OVR_CTL: 使能覆盖 */
}

static uint32_t cls_read_area(void *ctx)
{
    (void)ctx;
    if ((fab_rd(0x1Cu) & 0x2u) == 0u)    /* STATUS bit1 = 当前有目标 */
        return 0u;
    return (uint32_t)(fab_rd(0x2Cu) & 0xFFFFFu);
}

/* 形状结果：AHB 0x34 整字节原样返回
 *   bit[1:0] = 最大确定目标的类别（1 圆 / 2 三角 / 3 方）
 *   bit2     = 该类别已被三帧确认
 *   bit[4:3] = 画面上已确认的目标个数（0~3，屏幕上画几个框就是几个）
 * 位域解码放在 classifier.c，避免这里和状态机各理解一套。
 * 注意不能用 0x1C.bit1 门控 —— 那是颜色 CCA 的“有目标”，形状模式下不成立。 */
static uint8_t cls_read_shape(void *ctx)
{
    (void)ctx;
    return (uint8_t)(fab_rd(0x34u) & 0x1Fu);
}

static void cls_post(uint8_t act, void *ctx)
{
    (void)ctx;
    if (act == CLS_ACT_L1)      MM_PostAction(MM_SEQ_L1);
    else if (act == CLS_ACT_M1) MM_PostAction(MM_SEQ_M1);
    else if (act == CLS_ACT_R1) MM_PostAction(MM_SEQ_R1);
}

static uint8_t cls_busy(void *ctx)
{
    (void)ctx;
    return (MM_State() == MM_ST_RUN) ? 1u : 0u;
}

static void cls_log(const char *msg, void *ctx)
{
    (void)ctx;
    while (*msg)
        uart_putc((AL_REG)USART0_BASE_ADDR, (uint8_t)*msg++);
    uart_putc((AL_REG)USART0_BASE_ADDR, '\r');
    uart_putc((AL_REG)USART0_BASE_ADDR, '\n');
}

/* OSD 命令：op 1=跳项 2=下一项 3=加 4=减 */
static void osd_cmd(unsigned op, unsigned arg)
{
    fab_wr(0x30u, ((op & 0xFu) << 4) | (arg & 0xFu));
}

/* 当前模式是否支持该项（与 RTL 的钳位规则一致） */
static int item_valid(unsigned idx, unsigned mode)
{
    int color_detect = (mode == 6) || (mode == 8) || (mode == 10);
    int color_fill   = (mode == 5) || (mode == 7) || (mode == 9);
    int color_adjust = color_detect || color_fill;

    if (idx == 3 || idx == 4) return color_adjust;
    if (idx == 8 || idx == 9 || idx == 10 || idx == 12) return (mode == 12);
    if (idx == 11) return (mode == 12) || color_detect;
    return 1;                       /* 0,1,2,5,6,7 总是有效 */
}

/* 跳到第 idx 个 OSD 项。
 * 越界必须先拦：osd_cmd 会把 arg 截成 4 bit，J99 会被截成 99&0xF=3 跳到错的项上。 */
static void jump_item(unsigned idx)
{
    unsigned mode_now;

    if (idx > 12u) {
        AL_LOG(AL_LOG_LEVEL_ERROR, "\r\nERR: item must be 0..12\r\n");
        return;
    }
    mode_now = fab_rd(0x0Cu) & 0xFu;
    if (item_valid(idx, mode_now)) {
        osd_cmd(1, idx);
        AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\njump to item %lu\r\n", (unsigned long)idx);
    } else {
        AL_LOG(AL_LOG_LEVEL_ERROR, "\r\nERR: item %lu not valid in mode %lu\r\n",
               (unsigned long)idx, (unsigned long)mode_now);
    }
}

static void print_status(void)
{
    AL_LOG(AL_LOG_LEVEL_NOTICE, "SW=0x%lx MODE=0x%lx SRC=%lu ST=0x%lx\r\n",
           (unsigned long)(fab_rd(0x08u) & 0xFu),
           (unsigned long)(fab_rd(0x0Cu) & 0xFu),
           (unsigned long)(fab_rd(0x10u) & 0x1u),
           (unsigned long)(fab_rd(0x1Cu) & 0x3u));
}

static void apply_mode(unsigned m)
{
    if (m > 15u) {
        AL_LOG(AL_LOG_LEVEL_ERROR, "\r\nERR: mode must be 0..15\r\n");
        return;
    }
    fab_wr(0x14u, m);          /* OVR_MODE */
    fab_wr(0x18u, 0x1u);       /* OVR_CTL: 使能覆盖 */
    AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\nset mode = %lu\r\n", (unsigned long)m);
    print_status();
}

static void help(void)
{
    AL_LOG(AL_LOG_LEVEL_NOTICE,
           "cmd: J<n> jump (n=0..12, also JA/JB/JC), N next, + up, - down, T auto/manual,\r\n"
           "     M<n> mode 0-15, C clear, S status,\r\n"
           "     C_RUN colour classify, S_RUN shape classify (RUN = C_RUN)\r\n");
}

AL_S32 main()
{
    AL_USART_HalStruct *h0 = AL_NULL;
    unsigned mtvec_save;
    static const unsigned cand[] = { 0x80000000u, 0x90000000u, 0x70000000u };
    unsigned i, id;
    char cmd = 0;
    unsigned mnum = 0, mdig = 0, mlink = 0;
    unsigned skip = 0;              /* '!' 之后要跳过给老解析器的字符数 */
    unsigned run_seq = 0;           /* 裸 RUN 匹配进度：0=无 1=已收R 2=已收RU */
    unsigned idle_seq = 0;          /* IDLE 匹配进度：0=无 1=I 2=ID 3=IDL */
    unsigned pend = 0;              /* C/S 挂起：1=C(清覆盖) 2=S(打状态)，等下一字符看是不是 '_' */
    unsigned pend_link = 0;         /* 挂起静默计数：超时按原命令落实 */
    unsigned jnum = 0, jdig = 0, jlink = 0;  /* J<n> 累积：项号 / 已收位数 / 静默计数 */
    unsigned run_arm = 0;           /* 已收 "X_"：1=C_RUN(三色) 2=S_RUN(形状) */
    unsigned run_step = 0;          /* "X_R" / "X_RU" 之后还要收几个字符 */
    unsigned last_sw = 0xFFu, last_mode = 0xFFu, last_src = 0xFFu;

    memset(&usart_cfg, 0, sizeof(usart_cfg));
    usart_cfg.BaudRate   = 115200;
    usart_cfg.WordLength = AL_USART_CHAR_8_BITS;
    usart_cfg.StopBits   = AL_USART_STOP_1_BIT;
    usart_cfg.Parity     = AL_USART_NO_PARITY;
    usart_cfg.TxCtl      = AL_TRUE;
    usart_cfg.RxCtl      = AL_TRUE;
    AlMisc_ll_SetClkEn(AL_MISC_USART0, 1);
    AlMisc_ll_SetReset(AL_MISC_USART0, 0);
    AlMisc_ll_SetReset(AL_MISC_USART0, 1);
    AlUsart_Hal_Init(&h0, 0, &usart_cfg, AL_NULL);

    /* UART1：电机链路（同参数 115200 8N1），并对状态机做初始化 */
    {
        AL_USART_HalStruct *h1 = AL_NULL;
        AlMisc_ll_SetClkEn(AL_MISC_USART1, 1);
        AlMisc_ll_SetReset(AL_MISC_USART1, 0);
        AlMisc_ll_SetReset(AL_MISC_USART1, 1);
        AlUsart_Hal_Init(&h1, 1, &usart_cfg, AL_NULL);
    }
    MM_Init(motor_tx, motor_now, AL_NULL);
    MM_SetLog(motor_log);

    /* 分类器：面积/模式走 AHB 寄存器，动作复用 motor_master */
    {
        static const cls_port_t cls_port = {
            motor_now, cls_read_mode, cls_set_mode, cls_read_area,
            cls_post, cls_busy, cls_log, cls_read_shape
        };
        CLS_Init(&cls_port, AL_NULL);
    }

#if TEST_UART_BRIDGE
    /* ---- 串口互通测试：UART0 <-> UART1 双向透传 ---- */
    AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\n==== UART bridge: UART0 <-> UART1 ====\r\n");
    for (;;)
    {
        while (AlUsart_ll_GetRxFifoNum((AL_REG)USART0_BASE_ADDR) != 0u)
        {
            uart_putc((AL_REG)USART1_BASE_ADDR,
                      (uint8_t)(AlUsart_ll_RecvData((AL_REG)USART0_BASE_ADDR) & 0xFFu));
        }
        while (AlUsart_ll_GetRxFifoNum((AL_REG)USART1_BASE_ADDR) != 0u)
        {
            uart_putc((AL_REG)USART0_BASE_ADDR,
                      (uint8_t)(AlUsart_ll_RecvData((AL_REG)USART1_BASE_ADDR) & 0xFFu));
        }
    }
#endif

    __asm__ volatile("csrci mstatus, 8");
    __asm__ volatile("csrr %0, mtvec" : "=r"(mtvec_save));
    __asm__ volatile("csrw mtvec, %0" :: "r"((unsigned)trap_handler));

    AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\n==== stage4: serial OSD control ====\r\n");

    for (i = 0; i < sizeof(cand) / sizeof(cand[0]); i++) {
        id = srd(cand[i]);
        if (id == FAB_ID && g_base == 0u) g_base = cand[i];
    }
    if (g_base == 0u) {
        AL_LOG(AL_LOG_LEVEL_ERROR, "fabric not reachable\r\n");
    } else {
        AL_LOG(AL_LOG_LEVEL_NOTICE, "fabric OK at 0x%08lx\r\n", (unsigned long)g_base);
        help();
        print_status();
    }

    __asm__ volatile("csrw mtvec, %0" :: "r"(mtvec_save));

    unsigned hb = 0;

    for (;;) {
        unsigned c = uart_rx_poll();

        /* UART0 既是串口屏/电脑的命令口，也把字符喂给电机状态机（只认 !L1/!M1/!R1） */
        if (c != 0u) MM_RxByte((uint8_t)c);

        /* 电机链路：1ms 时基 + 收串口 + 驱动状态机 */
        mm_tick_1ms();
        motor_poll();
        if (s_ms != s_ms_seen) {
            s_ms_seen = s_ms;
            MM_Tick();
            CLS_Tick();
            (void)MM_TakeFault();       /* 只清标志，不打印（回程未通时避免刷屏） */
        }

        if (++hb >= 4000000u) {
            hb = 0;
#if UART0_HB_EN
            AL_LOG(AL_LOG_LEVEL_NOTICE, "HB SW=0x%lx MODE=0x%lx SRC=%lu\r\n",
                   (unsigned long)(fab_rd(0x08u) & 0xFu),
                   (unsigned long)(fab_rd(0x0Cu) & 0xFu),
                   (unsigned long)(fab_rd(0x10u) & 0x1u));
#endif
        }
        unsigned sw, mode, src;

        if (c != 0u) {
            AlUsart_ll_SendData((AL_REG)USART0_BASE_ADDR, (AL_U16)c);

            /* '!' 开头的是电机命令（!L1/!M1/!R1），已经交给状态机，
             * 这里把后面 2 个字符吞掉，别再当成 M<n> 设模式 */
            if (skip != 0u) {
                skip--;
                continue;
            }
            if (c == '!') {
                skip = 2u;
                cmd  = 0;
                continue;
            }

            /* C_RUN / S_RUN 与 C（清覆盖）/ S（打状态）同首字母，先挂起一个字符，
             * 等下一个字符看是不是 '_'；不是就把原命令落实，再用当前字符继续解析。 */
            if (pend != 0u) {
                unsigned p = pend;
                pend = 0u;
                pend_link = 0u;
                cmd = 0;
                if (c == '_') {
                    run_arm  = p;
                    run_step = 0u;
                    continue;
                }
                if (p == 1u) {
                    fab_wr(0x18u, 0x2u);
                    AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\nclear override\r\n");
                } else {
                    AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\n");
                }
                print_status();
                /* 不 continue：当前字符继续走下面的普通解析，不会被吞掉 */
            }

            /* "X_RUN" 的后半段：R -> U -> N */
            if (run_arm != 0u) {
                if (run_step == 0u && (c == 'R' || c == 'r')) { run_step = 1u; continue; }
                if (run_step == 1u && (c == 'U' || c == 'u')) { run_step = 2u; continue; }
                if (run_step == 2u && (c == 'N' || c == 'n')) {
                    unsigned which = run_arm;
                    run_arm  = 0u;
                    run_step = 0u;
                    cmd = 0;
                    if (which == 1u) CLS_Run();        /* C_RUN：三色面积分类 */
                    else             CLS_RunShape();   /* S_RUN：形状分类 */
                    continue;
                }
                run_arm  = 0u;      /* 不是合法后缀，整条放弃 */
                run_step = 0u;
            }

            /* IDLE：四段匹配。每发一次，底座转 90°，左右交替（奇数次左、偶数次右）。
             * 放在 RUN 之前：'I' 不被其它解析器使用，不会抢。 */
            if (idle_seq == 1u && (c == 'D' || c == 'd')) {
                idle_seq = 2u;
                continue;
            }
            if (idle_seq == 2u && (c == 'L' || c == 'l')) {
                idle_seq = 3u;
                continue;
            }
            if (idle_seq == 3u && (c == 'E' || c == 'e')) {
                idle_seq = 0u;
                cmd = 0;
                MM_PostAction(MM_SEQ_IDLE);
                continue;
            }
            if (c == 'I' || c == 'i') {
                idle_seq = 1u;
                cmd = 0;
                continue;
            }
            idle_seq = 0u;

            /* RUN：三段匹配。关键是要吃掉 'N'，否则会被当成“下一项” */
            if (run_seq == 1u && (c == 'U' || c == 'u')) {
                run_seq = 2u;
                continue;
            }
            if (run_seq == 2u && (c == 'N' || c == 'n')) {
                run_seq = 0u;
                cmd = 0;
                CLS_Run();
                continue;
            }
            if (c == 'R' || c == 'r') {
                run_seq = 1u;
                cmd = 0;
                continue;
            }
            run_seq = 0u;

            /* J 累积到一半又来了非数字：先把跳项落实，再用当前字符继续解析（不吞字符） */
            if (cmd == 'J' && jdig > 0u && !(c >= '0' && c <= '9')) {
                jump_item(jnum);
                cmd = 0; jdig = 0; jnum = 0; jlink = 0;
            }

            if (c == 'J' || c == 'j') {
                cmd = 'J'; jnum = 0; jdig = 0; jlink = 0;
            } else if (cmd == 'J' && (c >= '0' && c <= '9')) {
                jnum = jnum * 10u + (unsigned)(c - '0');
                jdig++;
                jlink = 0;
                if (jdig >= 2) { jump_item(jnum); cmd = 0; jdig = 0; jnum = 0; jlink = 0; }
            } else if (cmd == 'J' && jdig == 0u &&
                       ((c >= 'A' && c <= 'C') || (c >= 'a' && c <= 'c'))) {
                /* JA / JB / JC = 10 / 11 / 12，旧写法保留（顺带支持小写） */
                jump_item(10u + (unsigned)((c | 0x20u) - 'a'));
                cmd = 0;
            } else if (c == '+' || c == '=') {
                osd_cmd(3, 0);
                AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\nup\r\n");
                cmd = 0;
            } else if (c == '-' || c == '_') {
                osd_cmd(4, 0);
                AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\ndown\r\n");
                cmd = 0;
            } else if (c == 'N' || c == 'n') {
                osd_cmd(2, 0);
                AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\nnext item\r\n");
                cmd = 0;
            } else if (c == 'T' || c == 't') {
                osd_cmd(5, 0);
                AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\ntoggle auto/manual\r\n");
                cmd = 0;
            } else if (c == 'M' || c == 'm') {
                cmd = 'M'; mnum = 0; mdig = 0; mlink = 0;
            } else if (cmd == 'M') {
                if (c >= '0' && c <= '9') {
                    mnum = mnum * 10u + (unsigned)(c - '0');
                    mdig++;
                    mlink = 0;
                    if (mdig >= 2) { apply_mode(mnum); cmd = 0; }   /* 两位数立即生效 */
                } else {
                    if (mdig > 0) apply_mode(mnum);                 /* 单数字后回车 */
                    cmd = 0;
                }
            } else if (c == 'C' || c == 'c') {
                pend = 1u;              /* 可能后面跟 "_RUN"，下一字符再定 */
                pend_link = 0u;
                cmd = 0;
            } else if (c == 'S' || c == 's') {
                pend = 2u;              /* 可能是 S_RUN */
                pend_link = 0u;
                cmd = 0;
            } else if (c == '?') {
                AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\n");
                help();
                cmd = 0;
            } else {
                cmd = 0;
            }
            continue;
        }

        /* bare single-digit M<n>: auto-apply after ~25ms of line silence.
         * packet senders (serial screen / send box) put characters uSeconds
         * apart, so a two-digit tail always lands inside this window and
         * M12 still applies on its second digit before this timer fires.
         * (fix 2026-10-30: M09's two-digit parser made single digits wait
         *  for CR, which broke senders that append no terminator.) */
        /* 裸 C / S：静默 ~25ms 后按原命令落实（与 M<n> 单数字同一套写法，
         * 这样 C_RUN / S_RUN 之外的旧命令时序完全不变，只是晚一个字符判定期） */
        if (pend != 0u && ++pend_link >= 100000u) {
            unsigned p = pend;
            pend = 0u;
            pend_link = 0u;
            if (p == 1u) {
                fab_wr(0x18u, 0x2u);
                AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\nclear override\r\n");
            } else {
                AL_LOG(AL_LOG_LEVEL_NOTICE, "\r\n");
            }
            print_status();
        }

        /* J<n> 单数字：静默 ~25ms 按当前累积值落实（两位数已在收到第二位时立刻生效） */
        if (cmd == 'J' && jdig > 0u && ++jlink >= 100000u) {
            jump_item(jnum);
            cmd = 0; jdig = 0; jnum = 0; jlink = 0;
        }

        if (cmd == 'M' && mdig > 0u && ++mlink >= 100000u) {
            apply_mode(mnum);
            cmd = 0;
            mlink = 0;
        }

        if (g_base != 0u) {
            sw   = fab_rd(0x08u) & 0xFu;
            mode = fab_rd(0x0Cu) & 0xFu;
            src  = fab_rd(0x10u) & 0x1u;
            if (sw != last_sw || mode != last_mode || src != last_src) {
                last_sw = sw; last_mode = mode; last_src = src;
#if UART0_STATUS_EN
                print_status();
#endif
            }
        }
    }

    return 0;
}
