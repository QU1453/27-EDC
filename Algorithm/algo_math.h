#ifndef ALGO_MATH_H
#define ALGO_MATH_H

/* =====================================================================
 * algo_math.h —— 数学工具（Algorithm 层，纯头文件）
 *
 * 文件作用：
 *   提供全工程共用的限幅、取符号、死区、线性映射、一阶低通、角度卷绕、
 *   单位换算等小函数。不含任何 HAL / 硬件操作，可拿到 PC 上单独跑测试。
 *
 * 命名对照：
 *   algo      = algorithm（算法）        math  = 数学
 *   Clamp     = 限幅（压到区间内）        Clampf / ClampI32 = float 版 / 32 位整型版
 *   Abs       = absolute（绝对值）        Sign  = 符号（+1 / -1 / 0）
 *   Deadband  = 死区                      Map   = 线性映射
 *   LowPass   = 一阶低通                  WrapPi = 把角度卷绕到 ±π
 *   Deg / Rad / Rpm / RadPerSec = 度 / 弧度 / 转每分 / 弧度每秒
 *
 * 两个硬约定：
 *   1) 只用 float，不用 double —— G473 的 FPU 只有单精度，double 会掉进软件模拟。
 *   2) 全部 static inline、只有头文件没有 .c，所以不用登记进 uvprojx / eide.yml。
 * ===================================================================== */

#include <stdint.h>

/* 圆周率。必须带 f 后缀，否则是 double 字面量，会把整个表达式抬成双精度 */
#define ALGO_PI 3.14159265f

/* 作用：把 v 限幅到 [lo, hi]。lo/hi 传反了不会报错，只会得到错结果 */
static inline float Math_Clampf(float v, float lo, float hi)
{
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

/* 作用：对称限幅，把 v 压进 [-lim, +lim]，lim 必须 >= 0。
 * 为什么不直接转 float 再调 Clampf：中断里 int→float→int 来回转换开销更大 */
static inline int32_t Math_ClampI32(int32_t v, int32_t lim)
{
  if (v > lim) return lim;
  if (v < -lim) return -lim;
  return v;
}

/* 作用：取绝对值 */
static inline float Math_Absf(float v)
{
  return (v < 0.0f) ? -v : v;
}

/* 作用：取符号。正数 +1、负数 -1、0 返回 0（不是 1！） */
static inline float Math_Signf(float v)
{
  if (v > 0.0f) return 1.0f;
  if (v < 0.0f) return -1.0f;
  return 0.0f;
}

/* 作用：去死区。|v| < dead 输出 0，否则扣掉死区后保留原符号。dead 必须 >= 0。
 * 注意是「平移」不是「压缩」：输入 dead+1 输出 1，所以最大输出会少掉 dead。
 * 典型用途：摇杆中位抖动、电机低速不转 */
static inline float Math_Deadband(float v, float dead)
{
  if (v > dead) return v - dead;
  if (v < -dead) return v + dead;
  return 0.0f;
}

/* 作用：把 [in_lo, in_hi] 线性映射到 [out_lo, out_hi]。
 * 两端不夹紧——超区间的输入会得到超区间的输出，要夹紧就再套一层 Math_Clampf。
 * in_hi == in_lo 时直接返回 out_lo，避免除零出 inf/nan */
static inline float Math_Map(float in, float in_lo, float in_hi, float out_lo, float out_hi)
{
  if (in_hi == in_lo) return out_lo;
  return out_lo + (in - in_lo) * (out_hi - out_lo) / (in_hi - in_lo);
}

/* 作用：一阶低通（指数平滑），*state 会被就地更新。
 *   alpha = dt / (dt + tau)，dt = 调用周期(秒)，tau = 时间常数(秒)
 *   alpha 越接近 1 越跟手，越接近 0 越平滑。
 * 固定周期下 alpha 算一次存起来即可，别每帧现算（含除法）。
 * 用途：编码器速度、电流采样、IMU 角度去抖 */
static inline float Math_LowPass(float in, float *state, float alpha)
{
  *state += alpha * (in - *state);
  return *state;
}

/* 作用：把角度卷绕到 [-π, π]。
 * 用 while 而不用 fmodf：单周期内角度跳变远小于 2π，循环最多跑一两次，比 fmodf 快。
 * 但输入若是 1e6 弧度这种巨值，while 会空转几万次，那种场景要先自己取模 */
static inline float Math_WrapPi(float ang)
{
  while (ang > ALGO_PI) ang -= 2.0f * ALGO_PI;
  while (ang < -ALGO_PI) ang += 2.0f * ALGO_PI;
  return ang;
}

/* 作用：单位换算，全工程用得最多，成对使用：
 *   Deg（度） <-> Rad（弧度）
 *   Rpm（转每分） <-> RadPerSec（弧度每秒），1 rpm = 2π/60 rad/s ≈ 0.10472 rad/s
 * DM 电机反馈速度是 rad/s，运动学算出来是 rpm，靠这两组函数对接 */
static inline float Math_DegToRad(float deg) { return deg * ALGO_PI / 180.0f; }
static inline float Math_RadToDeg(float rad) { return rad * 180.0f / ALGO_PI; }
static inline float Math_RpmToRadPerSec(float rpm) { return rpm * 2.0f * ALGO_PI / 60.0f; }
static inline float Math_RadPerSecToRpm(float rad) { return rad * 60.0f / (2.0f * ALGO_PI); }

#endif
