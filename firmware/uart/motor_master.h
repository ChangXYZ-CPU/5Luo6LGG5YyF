/**
 * motor_master.h -- 上位机侧二维转台控制状态机（PH1P35 软核 / 任意 MCU 通用）
 *
 * 职责：接收串口 ASCII 命令 L1/M1/R1 -> 按步骤表驱动两条轴 -> 解析下位机回传的到位码。
 * 与硬件解耦：只通过 mm_tx_fn 发字节、mm_now_fn 取时间，便于先在 PC 上做单元测试。
 *
 * 下行帧（下位机 STM32 固件固化）：
 *   AA 55 | N | [ addr | len | data(len) ] x N | sum
 *   addr: 0 = 广播(两轴同命令), 1 = 水平轴, 2 = 顶部轴
 *   data: Emm_V5 命令原始字节（功能码 + 参数 + 0x6B），下位机透传到 CAN（500 kbps）
 *
 * 硬件（《二维转台_帧表与接线》2026-10-31）：
 *   ZDT X42S 1.8° 步进，16 细分 = 3200 脉冲/圈，两轴直连无减速比（3200/360 ≈ 8.89 脉冲/度）
 *   到位输入 PB7 = 水平、PB6 = 顶部，高电平 3.3V = 到位
 *
 * 动作定义（每个动作 90° = 800 脉冲）：
 *   L1: 水平左转 90 -> 顶部右转 90 -> 停 1 s -> 顶部左转回 -> 水平右转归位
 *   M1: 水平不动  -> 顶部右转 90 -> 停 1 s -> 顶部左转回
 *   R1: 水平右转 90 -> 顶部右转 90 -> 停 1 s -> 顶部左转回 -> 水平左转归位
 *
 * 启动握手（MU_DSK1 上电自动回零，阻塞约 10.1 s）：
 *   INIT 每 500 ms 重发广播使能帧，直到收到从机整帧回显才进 IDLE；
 *   超过 MM_T_BOOT_MS 无回显判 FAULT。不依赖固定等待，从机先/后上电都能对上。
 *
 * 注意：从机另有 2 字节简易命令（01 00 / 01 01 / 02 00 / 02 01），
 *   其字节与“到位回传”同形且会被原样回显，用它会误判到位。
 *   **本模块一律用 AA55 整帧，不使用简易命令。**
 */

#ifndef MOTOR_MASTER_H
#define MOTOR_MASTER_H

#include <stdint.h>

/* ---------------- 动作 ID ---------------- */
#define MM_SEQ_NONE        0u
#define MM_SEQ_L1          1u   /* 水平左转 -> 顶部右转保持 -> 回 -> 归位 */
#define MM_SEQ_M1          2u   /* 水平不动 -> 顶部右转保持 -> 回 */
#define MM_SEQ_R1          3u   /* 水平右转 -> 顶部右转保持 -> 回 -> 归位 */
#define MM_SEQ_STOP        4u   /* 急停（广播停止） */
#define MM_SEQ_UNCLOG      5u   /* 解除堵转（广播） */
#define MM_SEQ_IDLE        6u   /* 底座单步 90°：每次发送左右交替（1/3/5 左，2/4/6 右） */

/* ---------------- 命令形式 ----------------
 * 1 = 用下位机已封装的 2 字节命令（01 00 / 01 01 / 02 00 / 02 01，90°、400 RPM）
 *     —— 完全不改下位机程序，帧最短（2 字节）
 * 0 = 用 AA55 整帧（角度/速度/超时都可调，回显带帧头无歧义，但一条 18 字节）
 * 两种都由同一下位机固件支持，可随时切换。 */
#define MM_USE_SIMPLE_CMD  0u

/* 发出命令后这段时间内，收到的同轴到位码视为“自己的回显”并忽略。
 * 依据：回显在 1~3 ms 内回来；400 RPM 转 90° 最少也要 ~40 ms。 */
#define MM_ECHO_GUARD_MS   15u

/* 命令发出后这段时间内收到的同轴“到位码”一律视为假信号并丢弃。
 * 依据：30 RPM 转 90° 需 0.5 s，物理上不可能更快；电机启动瞬间的干扰
 * 常会在 PB7/PB6 上造成假边沿。设 250 ms 留足余量。 */
#define MM_MIN_ARRIVAL_MS  250u

/* ---------------- 机构参数 ---------------- */
#define MM_PULSE_PER_REV   3200u    /* ZDT X42S 1.8° + 16 细分 */
#define MM_ANGLE_DEG       90u      /* 每个动作的角度 */

#define MM_VEL_RPM         30u      /* 30 RPM：90° 约 0.5 s，看得清、不易丢步 */
#define MM_ACC             10u      /* 加速度档位 */
#define MM_RAF             0u       /* 0 = 相对上一目标（与帧表一致） */

/* ---------------- 方向（物理左右未实测，翻这里即可） ---------------- */
/* Emm_V5: 0 = CW 顺时针, 1 = CCW 逆时针 */
#ifndef MM_DIR_H_LEFT
#define MM_DIR_H_LEFT      0u       /* 水平左转（2026-10-31 实测后对调） */
#endif
#ifndef MM_DIR_H_RIGHT
#define MM_DIR_H_RIGHT     1u       /* 水平右转（2026-10-31 实测后对调） */
#endif
#ifndef MM_DIR_T_RIGHT
#define MM_DIR_T_RIGHT     0u       /* 顶部右转（出） */
#endif
#ifndef MM_DIR_T_LEFT
#define MM_DIR_T_LEFT      1u       /* 顶部左转（回） */
#endif

/* ---------------- 命令格式 ----------------
 * 命令从 UART0（串口屏/PC）进来，统一带前缀 '!'，避免和视频模式的 M0~M15 撞名：
 *     !L1  !M1  !R1        （大小写都行，不需要回车）
 * UART1 专用于下位机链路，不再兼收命令。 */
#define MM_CMD_PREFIX      '!'

/* ---------------- 时间参数 ---------------- */
/* 启动握手：1=上电重发使能帧直到收到从机回显；0=跳过，直接进入可接收命令状态。
 * 回程线（从机 TX -> 本机 RX）未接通时握手永远失败，会一直刷使能帧，所以可关。 */
#define MM_HANDSHAKE_EN    0u

#define MM_T_EN_RETRY_MS   500u     /* 使能帧重发间隔（兼作链路探测） */
#define MM_T_BOOT_MS       15000u   /* 等从机启动完成的上限（自动回零约 10.1 s） */
#define MM_T_MOVE_MS       1200u    /* 单步到位超时（90° @30 RPM 约 0.5 s + 加减速） */
#define MM_HOLD_MS         1000u    /* 保留：原"顶部到位后保持" */

/* 每步之后固定等待的时间（不依赖下位机到位码）。
 * 30 RPM 转 90° 约 0.5 s，1 s 留一倍余量。 */
#define MM_STEP_DELAY_MS   1000u

/* 1 = 超时视为到位并继续（不卡死）；0 = 超时进 FAULT */
#define MM_TIMEOUT_AS_DONE 1

/* ---------------- 状态 ---------------- */
enum {
    MM_ST_INIT = 0,     /* 等待从机就绪（握手） */
    MM_ST_IDLE,         /* 等命令 */
    MM_ST_RUN,          /* 正在执行步骤表 */
    MM_ST_DONE,         /* 动作完成 */
    MM_ST_FAULT
};

/* 调试日志回调：tag = "TX"/"POS"/"ECHO"，buf 为相关字节。可为 NULL */
typedef void     (*mm_log_fn)(const char *tag, const uint8_t *buf, uint8_t len);

/* 发送回调：把 buf[0..len-1] 写进串口 */
typedef void     (*mm_tx_fn)(const uint8_t *buf, uint16_t len, void *ctx);
/* 时间回调：返回上电后的毫秒数 */
typedef uint32_t (*mm_now_fn)(void *ctx);

/* 初始化。tx / now 必须非 NULL */
void     MM_Init(mm_tx_fn tx, mm_now_fn now, void *ctx);

/* 挂调试日志（在主机串口上打印 TX/RX 记录），不需要就传 NULL */
void     MM_SetLog(mm_log_fn fn);

/* 周期性调用（建议 1 ms 一次），驱动状态机与超时 */
void     MM_Tick(void);

/* 外部触发动作（串口屏以外的入口，如按键/视觉），act 取 MM_SEQ_xxx */
void     MM_PostAction(uint8_t act);

/* 串口每收到 1 个字节就喂进来：内部识别 "L1"/"M1"/"R1"，并过滤回显与调试打印 */
void     MM_RxByte(uint8_t b);

uint8_t  MM_State(void);
uint8_t  MM_TakeFault(void);        /* 读并清故障标志，1 = 刚发生过故障 */
void     MM_Reset(void);

/* 调试：当前执行到步骤表的第几步（0 起；不在执行时返回 -1） */
int8_t   MM_Step(void);

#endif /* MOTOR_MASTER_H */
