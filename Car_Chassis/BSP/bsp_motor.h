#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"

/* =====================================================================
 * bsp_motor.h —— 达妙 DM-S2325-1EC 电机驱动（BSP 层，MIT 定速模式）
 *
 * 文件作用：
 *   把「目标转速 rpm」翻译成 DM 的 8 字节 MIT 帧并下发，
 *   再把反馈帧解成位置/转速/力矩/温度，供上层直接用。
 *
 * 命名对照：
 *   BSP   = Board Support Package（板级支持包）
 *   DM    = Damiao（达妙）        MIT = 达妙的一种控制帧格式
 *   P / V / T   = Position / Velocity / Torque（位置 / 速度 / 力矩）
 *   KP / KD     = 位置增益 / 速度增益
 *   CMD   = Command（命令）        ERR = Error（驱动器状态码）
 *   id          = 电机 CAN ID（控制帧用）
 *   master_id   = 主机 ID，也是反馈帧的 ID
 *   t_mos       = MOS 管温度 ℃      t_rotor = 绕组温度 ℃
 *   cmd_rpm     = 目标转速 rpm      rpm     = 实测转速 rpm
 *   pos_rad     = 输出轴位置 rad    tor_nm  = 输出力矩 N·m
 *   last_tick   = 最近一次收到反馈的时刻（判掉线用）
 *
 * 驱动器内部算式：t = Kp*(p_des - p) + Kd*(v_des - v) + t_ff
 *   Kp = 0 时只剩 Kd*(v_des - v)，等价于一个速度 P 环。
 *   所以主控只给 v_des（rad/s），外侧不要再套一层速度 PID。
 *
 * CAN：经典 CAN、标准帧、8 字节，一帧一机
 *   控制帧 ID = 电机 ID（不加偏移）
 *   反馈帧 ID = 该机的 Master ID（出厂 0，两台必须用上位机改成不同值）
 *   反馈是 query-based：发一帧控制就回一帧反馈，控制 200 Hz 即反馈 200 Hz
 * ===================================================================== */

/* MIT 位域量程：把物理值线性压进 16 位 / 12 位定点。
 * ⚠ 这三个值存在驱动器内部，必须用达妙上位机「读参数」核对后改成自己电机的真实值，
 *   填错会让目标转速/力矩整体缩放（看起来像"响应很弱/很猛"） */
#define DM_P_MAX        12.5f    /* rad，位置满量程 ± */
#define DM_V_MAX        30.0f    /* rad/s，速度满量程 ± */
#define DM_T_MAX        5.0f     /* N·m，力矩满量程 ± */

/* Kp / Kd 的映射区间是固定的，不是 1:1 增益 */
#define DM_KP_MIN       0.0f
#define DM_KP_MAX       500.0f
#define DM_KD_MIN       0.0f
#define DM_KD_MAX       5.0f

/* 定速用的 Kd。它就是内部速度环的 P 增益：先从 0.3 试，太大抖、太小软 */
#define DM_KD_DEFAULT   0.3f

/* 特殊命令帧：8 字节，前 7 字节全 0xFF，D7 = 命令码，帧 ID = 电机 ID */
#define DM_CMD_ENABLE    0xFCU   /* 使能，反馈 err 变 1 才算成功 */
#define DM_CMD_DISABLE   0xFDU   /* 失能，电机松轴 */
#define DM_CMD_SET_ZERO  0xFEU   /* 把当前位置存成 0 */
#define DM_CMD_CLEAR_ERR 0xFBU   /* 清错误 */

/* 反馈帧 D0 高 4 位的状态码（只列常用的） */
#define DM_ERR_DISABLED  0x0U    /* 失能 */
#define DM_ERR_ENABLED   0x1U    /* 已使能，可以发控制帧了 */

typedef struct
{
  uint8_t  id;         /* 电机 CAN ID，控制帧用它 */
  uint8_t  master_id;  /* 反馈帧 ID，逐台改成不同值，否则分不出左右轮 */
  uint8_t  err;        /* 最近一帧的状态码，DM_ERR_ENABLED 才算正常 */
  uint8_t  t_mos;      /* MOS 温度 ℃ */
  uint8_t  t_rotor;    /* 绕组温度 ℃ */
  float    cmd_rpm;    /* 目标转速 rpm（运动学给出，下发时换成 rad/s） */
  float    rpm;        /* 实测转速 rpm（反馈 rad/s 换算） */
  float    pos_rad;    /* 输出轴位置 rad，上电清零，不要当绝对位置用 */
  float    tor_nm;     /* 输出力矩 N·m */
  uint32_t last_tick;  /* 最近一次收到反馈的时刻，用于判掉线 */
} BSP_Motor_t;

/* 作用：记录 ID 并清空状态。上电后还要调 BSP_Motor_Enable 电机才会转 */
void BSP_Motor_Init(BSP_Motor_t *motor, uint8_t id, uint8_t master_id);

/* 作用：设置目标转速 rpm，内部按 ±DM_V_MAX（换算成 rpm）限幅 */
void BSP_Motor_SetSpeed(BSP_Motor_t *motor, float rpm);

/* 作用：逐台组 MIT 帧下发（一机一帧），返回 BSP_CAN_OK 表示全部发出 */
uint8_t BSP_Motor_Send(BSP_Motor_t *motors, uint8_t count);

/* 作用：使能 / 失能（内部就是发 7 个 0xFF + 命令码） */
uint8_t BSP_Motor_Enable(BSP_Motor_t *motor, uint8_t enable);

/* 作用：发任意特殊命令：DM_CMD_SET_ZERO / DM_CMD_CLEAR_ERR 等 */
uint8_t BSP_Motor_SendCmd(BSP_Motor_t *motor, uint8_t cmd);

/* 作用：解析一帧反馈并按 master_id 找到对应电机。
 * 命中返回 1，不是本组电机的帧返回 0 */
uint8_t BSP_Motor_Decode(BSP_Motor_t *motors, uint8_t count,
                         uint32_t std_id, const uint8_t data[8]);

/* 作用：判在线。timeout_ms 内收到过反馈返回 1，否则 0 */
uint8_t BSP_Motor_IsOnline(const BSP_Motor_t *motor, uint32_t timeout_ms);

#endif
