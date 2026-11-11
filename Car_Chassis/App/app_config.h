#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* =====================================================================
 * app_config.h —— 全工程可调参数（App 层，唯一改参数的地方）
 *
 * 文件作用：
 *   把电机 ID、机械尺寸、控制周期、PID 增益集中在这里。
 *   调车时只改这个文件，不要到 .c 里改魔法数。
 *
 * 命名对照：
 *   APP        = Application（应用层）
 *   ID         = 电机自身的 CAN ID（控制帧用）
 *   MASTER     = 主机 ID（反馈帧用）
 *   DIR        = Direction（转向系数，+1 正转 / -1 反转）
 *   WHEEL_RADIUS = 车轮半径      WHEEL_BASE = 轮距
 *   GEAR_RATIO = 减速比          PERIOD_MS  = 周期（毫秒）
 *   OFFLINE_MS = 掉线判定时间    PID KP/KI/KD = 比例/积分/微分增益
 *   OUT_MAX    = 输出限幅        I_MAX      = 积分限幅
 * ===================================================================== */

/* ================= 电机（达妙 DM-S2325-1EC）================= */
/* 电机 CAN ID：控制帧 ID = 这个值 */
#define APP_MOTOR_LEFT_ID       1U
#define APP_MOTOR_RIGHT_ID      2U
/* Master ID：反馈帧 ID，出厂都是 0，必须用达妙上位机逐台改成不同值 */
#define APP_MOTOR_LEFT_MASTER   0x11U
#define APP_MOTOR_RIGHT_MASTER  0x12U
/* 转向系数：装反了（前进时反转）就把对应值改成 -1.0f */
#define APP_MOTOR_LEFT_DIR      (+1.0f)
#define APP_MOTOR_RIGHT_DIR     (+1.0f)

/* ================= 机械参数（按实车实测修改） ================= */
#define APP_WHEEL_RADIUS        0.075f   /* 车轮半径 m */
#define APP_WHEEL_BASE          0.300f   /* 左右轮中心距 m */
#define APP_GEAR_RATIO          1.0f     /* DM 的转速/力矩已在输出轴，减速比填 1 */

/* ================= 控制周期 ================= */
#define APP_CONTROL_PERIOD_MS   5U       /* 控制周期 5 ms = 200 Hz */
#define APP_MOTOR_OFFLINE_MS    100U     /* 超过该时间没反馈判为掉线 */

/* ============ 速度环 PID（DM 内部已有 Kd 速度环，暂不启用）============ */
#define APP_PID_KP              6.0f     /* 比例增益 */
#define APP_PID_KI              0.3f     /* 积分增益 */
#define APP_PID_KD              0.0f     /* 微分增益 */
#define APP_PID_OUT_MAX         3000.0f  /* 输出限幅 */
#define APP_PID_I_MAX           1500.0f  /* 积分限幅 */

#endif
