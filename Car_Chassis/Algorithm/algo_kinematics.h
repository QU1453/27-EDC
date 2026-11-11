#ifndef __ALGO_KINEMATICS_H
#define __ALGO_KINEMATICS_H

/* =====================================================================
 * algo_kinematics.h —— 两轮差速运动学（Algorithm 层，纯计算）
 *
 * 文件作用：
 *   负责「底盘速度 (v, w)」和「左右轮转速 (rpm)」之间的双向换算：
 *   逆解给控制用（下发给电机），正解给反馈用（里程计/上位机）。
 *
 * 命名对照：
 *   Algo        = Algorithm（算法）
 *   Kinematics  = 运动学          Chassis = 底盘
 *   wheel_radius= 车轮半径        wheel_base = 轮距（两轮接地中心距）
 *   gear_ratio  = 减速比（电机转子圈数 / 车轮圈数）
 *   Inverse     = 逆解：(v, w) → 左右 rpm
 *   Forward     = 正解：左右 rpm → (v, w)
 *   v = 线速度 m/s（+ 前进）      w = 角速度 rad/s（+ 逆时针，即左转）
 *   left_rpm / right_rpm = 左右电机转子转速（转/分）
 *
 * 车型：左右两轮各一台 DM-S2325 + 前万向轮，只能差速转向（可原地转），没有横移。
 *
 * 坐标系约定（全工程统一，改约定等于改全工程）：
 *   车头 = +x，左侧 = +y；左轮在 -y 侧，右轮在 +y 侧；L = 两轮接地中心距。
 *
 * 注意：输出的是「电机转子 rpm」，不是车轮 rpm。
 *   DM-S2325 反馈和下发都已是输出轴（减速后）的量，
 *   所以 app_config.h 里 APP_GEAR_RATIO 必须填 1.0f，别填真实减速比。
 * ===================================================================== */

typedef struct
{
  float wheel_radius;   /* 车轮半径 m（配 150 mm 轮 → 0.075f） */
  float wheel_base;     /* 轮距 m，不是车宽；贴地量两侧轮心到轮心 */
  float gear_ratio;     /* 电机转子圈 / 车轮圈；DM 输出轴量 → 填 1.0f */
} Algo_Chassis_t;

/* 作用：存三个机械参数（只存不算）。
 * 三个都必须是正数，传 0 不会立刻报错，而是在第一次逆解时才算出 inf/nan */
void Algo_Chassis_Init(Algo_Chassis_t *chassis, float wheel_radius,
                       float wheel_base, float gear_ratio);

/* 作用：逆解，(v, w) → 左右轮目标 rpm，直接喂给电机。
 * 控制链里唯一被周期调用的函数，所以用输出指针而不是返回结构体，省 ABI 开销。
 * v、w 不做限幅，需要限幅在上层（App）做完再进来 */
void Algo_Chassis_Inverse(const Algo_Chassis_t *chassis, float v, float w,
                          float *left_rpm, float *right_rpm);

/* 作用：正解，左右轮实测 rpm → (v, w)，用于里程计、上位机显示、外环反馈。
 * 与逆解互为反运算，可以用来互相验证 */
void Algo_Chassis_Forward(const Algo_Chassis_t *chassis,
                          float left_rpm, float right_rpm,
                          float *v, float *w);

#endif
