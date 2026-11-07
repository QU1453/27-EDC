#ifndef __ALGO_PID_H
#define __ALGO_PID_H

/* =====================================================================
 * algo_pid.h —— 位置式 PID（Algorithm 层，纯计算）
 *
 * 文件作用：
 *   提供一个通用位置式 PID，带积分限幅和 anti-windup，
 *   给上层外环（循迹、定距、姿态）用；不含任何硬件操作。
 *
 * 命名对照：
 *   Algo = Algorithm（算法）      PID = 比例-积分-微分
 *   kp / ki / kd       = 比例 / 积分 / 微分增益
 *   out_max            = 输出限幅（对称 ±out_max）
 *   integral_max       = 积分限幅（对称 ±integral_max）
 *   target / feedback  = 目标值 / 反馈值
 *   error / last_error = 本拍误差 / 上一拍误差
 *   integral / output  = 积分累加器 / 最近一次输出
 *   Init / SetTarget / Reset / Update = 初始化 / 设目标 / 复位状态 / 单次更新
 *
 * 三个特点：
 *   1) 位置式：输出是绝对量。丢一帧只是少调一次，下一帧立刻拉回；
 *      增量式的累加器一旦丢帧就会和内部状态脱节，静默跑偏。
 *   2) 积分限幅 + 饱和时停止积分（anti-windup），防止大误差累积后超调。
 *   3) 微分作用于「误差的变化」而非「测量值的变化」，设定值突跳会有微分尖峰；
 *      需要"微分先行"请另写变体，不要在这里改。
 *
 * 本工程当前用法：
 *   DM-S2325 的 MIT 帧内部已是速度环（Kp=0、Kd≠0），所以底盘速度环没启用它。
 *   真要串在 DM 外面就是双环，参数必须重新整定，别照抄 app_config.h。
 * ===================================================================== */

typedef struct
{
  /* ---- 参数，Init 时填 ---- */
  float kp;             /* 比例增益 */
  float ki;             /* 积分增益 */
  float kd;             /* 微分增益 */
  float out_max;        /* 输出限幅，一般填执行器物理上限 */
  float integral_max;   /* 积分限幅，经验值取 out_max 的 0.3~0.8 */

  /* ---- 运行时状态，调用方不要手改 ---- */
  float target;         /* 目标值，由 Algo_PID_SetTarget 写入 */
  float feedback;       /* 最近一次的反馈值 */
  float error;          /* 误差 = target - feedback */
  float last_error;     /* 上一拍误差，用来算微分 */
  float integral;       /* 积分累加器，已限幅 */
  float output;         /* 最近一次输出，饱和后的值 */
} Algo_PID_t;

/* 作用：填参数并把所有状态清零。
 * out_max / integral_max 按正数传，内部自己取 ± */
void  Algo_PID_Init(Algo_PID_t *pid, float kp, float ki, float kd,
                    float out_max, float integral_max);

/* 作用：只改目标值，不动积分。
 * 运行中切目标不要 Reset，否则积分清零会让输出突跳 */
void  Algo_PID_SetTarget(Algo_PID_t *pid, float target);

/* 作用：清空误差/积分/输出，保留增益和限幅。
 * 用途：模式切换、急停恢复、发现输出异常时重新起调 */
void  Algo_PID_Reset(Algo_PID_t *pid);

/* 作用：算一个控制周期，返回饱和后的输出（与 pid->output 相同）。
 * dt 单位秒，必须按固定周期传真实间隔，否则 ki / kd 的实际强度会跟着变 */
float Algo_PID_Update(Algo_PID_t *pid, float feedback, float dt);

#endif
