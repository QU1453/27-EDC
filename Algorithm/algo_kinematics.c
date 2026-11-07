#include "algo_kinematics.h"
#include "algo_math.h"

/* =====================================================================
 * algo_kinematics.c —— 两轮差速运动学的实现
 *
 * 文件作用：
 *   实现 algo_kinematics.h 里声明的三个函数，只做浮点运算，不碰硬件。
 *
 * 命名对照：
 *   half       = 半轮距（L/2）
 *   v_left / v_right = 左 / 右轮的接地线速度 m/s
 *   k          = 线速度 → 转子 rpm 的换算系数
 *   chassis    = 底盘参数结构体
 *
 * 逆解推导（一步不跳）：
 *   1) 车体中心速度 v、绕中心角速度 w，左右轮到中心横向距离都是 L/2，
 *      绕中心转时右轮被"多推"、左轮被"少推"：
 *        v_l = v - w * (L/2)
 *        v_r = v + w * (L/2)
 *      （w > 0 逆时针，右轮快左轮慢，车往左拐，符合直觉）
 *   2) 轮子线速度 → 轮子转速：  车轮圈/秒 = v_wheel / (2πR)
 *   3) 轮子转速 → 电机转子 rpm：转子 rpm = 车轮圈/秒 * 60 * gear_ratio
 *   4) 合并 2)、3)：             k = 60 * gear_ratio / (2πR)
 *                              转子 rpm = v_wheel * k
 *
 * 算例：R = 0.075 m、gear_ratio = 1、v = 0.5 m/s
 *       k = 60 / (2π * 0.075) ≈ 127.3 → 转子 rpm ≈ 63.7（约 64 转/分）
 * ===================================================================== */

/* 作用：保存轮径、轮距、减速比，不做换算也不做合法性检查 */
void Algo_Chassis_Init(Algo_Chassis_t *chassis, float wheel_radius,
                       float wheel_base, float gear_ratio)
{
  /* 传 0 这里不会报错，要到第一次逆解才炸出 inf/nan，很难追；
   * 所以调用方（App_Chassis_Init）必须保证传的是实测正数 */
  chassis->wheel_radius = wheel_radius;
  chassis->wheel_base   = wheel_base;
  chassis->gear_ratio   = gear_ratio;
}

/* 作用：逆解。把底盘 (v, w) 拆成左右轮各自的转子目标 rpm */
void Algo_Chassis_Inverse(const Algo_Chassis_t *chassis, float v, float w,
                          float *left_rpm, float *right_rpm)
{
  /* 推导第 1 步：车体速度 → 左右轮接地线速度 m/s */
  float half = chassis->wheel_base * 0.5f;
  float v_left  = v - w * half;
  float v_right = v + w * half;

  /* 推导第 4 步：线速度 → 转子 rpm 的系数。
   * k 只跟轮径和减速比有关，理论上 Init 时算好存起来更省；
   * 这里每 5 ms 现算一次，多一次除法（FPU 十几周期），影响可忽略，
   * 换来结构体不用加缓存字段、改参数也不用重算 */
  float k = 60.0f * chassis->gear_ratio / (2.0f * ALGO_PI * chassis->wheel_radius);

  *left_rpm  = v_left * k;
  *right_rpm = v_right * k;
}

/* 作用：正解。把左右轮实测 rpm 还原成底盘线速度 v 和角速度 w */
void Algo_Chassis_Forward(const Algo_Chassis_t *chassis,
                          float left_rpm, float right_rpm,
                          float *v, float *w)
{
  /* 逆解系数 k 的倒数：转子 rpm → 接地线速度 m/s，即 2πR / (60 * gear_ratio) */
  float k = (2.0f * ALGO_PI * chassis->wheel_radius)
          / (60.0f * chassis->gear_ratio);
  float v_left  = left_rpm * k;
  float v_right = right_rpm * k;

  /* 推导第 1 步的反解（左右轮线速度 → 底盘速度）：
   *   v = (v_l + v_r) / 2
   *   w = (v_r - v_l) / L    ← 除的是整轮距，不是半轮距 */
  *v = (v_left + v_right) * 0.5f;
  *w = (v_right - v_left) / chassis->wheel_base;
}
