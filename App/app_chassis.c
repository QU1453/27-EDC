#include "app_chassis.h"
#include "app_config.h"
#include "stm32g4xx_hal.h"
#include "bsp_can.h"
#include "bsp_motor.h"
#include "algo_kinematics.h"

/* =====================================================================
 * app_chassis.c —— 底盘任务实现（App 层，整车控制）
 *
 * 文件作用：
 *   把上层给的「整车线速度 v / 角速度 w」翻译成左右两个电机的目标转速，
 *   并负责收反馈、掉线保护、周期门控。上层只调 4 个函数，不碰 CAN。
 *
 * 数据流：
 *   App_Chassis_Init()            上电调一次：建电机对象、建运动学对象、开 CAN、发使能
 *   App_Chassis_SetVelocity(v,w)  随时调用：只写目标，不做运算（不会阻塞）
 *   App_Chassis_Task()            while(1) 里反复调用，内部自己做 5 ms 周期门控
 *   App_Chassis_Stop()            急停：目标清零并立刻下发 0 转速
 *
 * Task 的 6 步（缺一不可）：
 *   1) 收反馈：一次把 CAN FIFO 取空，逐帧解码（不取空会丢帧、速度会跳）
 *   2) 周期门控：不到 5 ms 直接返回，保证 200 Hz 固定控制频率
 *   3) 运动学：(v, w) → 左右轮 rpm
 *   4) 乘方向系数下发目标转速
 *   5) 掉线保护：没反馈的电机给 0，并定期重发使能
 *   6) 一机一帧下发
 *
 * 为什么这里没有速度环 PID：
 *   DM-S2325 的 MIT 帧内部算式是 t = Kp*(p_des-p) + Kd*(v_des-v) + t_ff，
 *   设 Kp=0、Kd≠0 时驱动器内部就是一个速度 P 环。
 *   外层再串一层 PID 会变成双环互斗，表现为抖动、超调、回中振荡。
 *
 * 命名对照：
 *   App = Application 应用层              Chassis = 底盘
 *   Init = 初始化                         Task = 周期任务
 *   SetVelocity = 设置线速度/角速度        Stop = 急停
 *   GetLeftRpm/GetRightRpm = 读左右实测转速
 *   s_motor = 电机对象数组（左[0] 右[1]）  s_chassis = 运动学参数
 *   s_target_v = 目标线速度 m/s           s_target_w = 目标角速度 rad/s
 *   s_last_tick = 上次解算时刻            s_enable_tick = 上次重发使能时刻
 *   s_ready = 初始化完成标志              APP_MOTOR_NUM = 电机数量
 *   APP_ENABLE_RETRY_MS = 使能重发间隔    rpm_left/rpm_right = 左右目标转子转速
 * ===================================================================== */

#define APP_MOTOR_NUM      2U
#define APP_ENABLE_RETRY_MS 500U     /* 掉线电机重发使能帧的间隔 */

/* 左轮固定放 [0]、右轮固定放 [1]，后面所有代码都靠这个下标区分左右 */
static BSP_Motor_t    s_motor[APP_MOTOR_NUM];
/* 差速运动学对象，只存轮径/轮距/减速比三个参数，本身没有状态 */
static Algo_Chassis_t s_chassis;

static float    s_target_v;     /* 目标线速度 m/s，由 SetVelocity 写入 */
static float    s_target_w;     /* 目标角速度 rad/s，逆时针为正 */
static uint32_t s_last_tick;    /* 上一次控制解算的时刻，用于 5 ms 门控 */
static uint32_t s_enable_tick;  /* 上一次重发使能帧的时刻，用于 500 ms 限流 */
static uint8_t  s_ready;        /* 0 = 初始化没成功，Task 直接空转 */

/**
 * 作用：上电初始化一次。建两个电机对象、建运动学参数、开 FDCAN、给两台电机发使能。
 * 返回：0 = 成功，1 = FDCAN 初始化失败（此时 Task 会一直空转，不会碰总线）。
 */
uint8_t App_Chassis_Init(void)
{
  uint8_t i;

  /* 记录两台电机的 CAN ID（控制帧用）和 Master ID（反馈帧用）。
   * Master ID 出厂都是 0，两台必须用达妙上位机改成不同值，否则左右轮反馈分不开 */
  BSP_Motor_Init(&s_motor[0], APP_MOTOR_LEFT_ID,  APP_MOTOR_LEFT_MASTER);
  BSP_Motor_Init(&s_motor[1], APP_MOTOR_RIGHT_ID, APP_MOTOR_RIGHT_MASTER);

  /* 减速比传 1.0f：DM 的转速/力矩已经是减速后的输出轴量，不能再乘一次 */
  Algo_Chassis_Init(&s_chassis, APP_WHEEL_RADIUS, APP_WHEEL_BASE, APP_GEAR_RATIO);

  /* 上电先当「停着」处理，避免 init 期间残留的旧目标让车突然冲一下 */
  s_target_v    = 0.0f;
  s_target_w    = 0.0f;
  s_last_tick   = HAL_GetTick();
  s_enable_tick = s_last_tick;
  s_ready       = 0U;

  /* FDCAN 初始化失败就直接返回，s_ready 保持 0，Task 不会碰总线 */
  if (BSP_CAN_Init() != BSP_CAN_OK)
  {
    return 1U;
  }

  /* 使能两台电机：反馈 err 变成 1 才算真正上电使能 */
  for (i = 0U; i < APP_MOTOR_NUM; i++)
  {
    (void)BSP_Motor_Enable(&s_motor[i], 1U);
  }

  s_ready = 1U;
  return 0U;
}

/**
 * 作用：写目标速度，只存值不做运算，任何时刻调用都不会阻塞。
 * 参数：v = 整车线速度 m/s（+ 前进），w = 角速度 rad/s（+ 逆时针/左转）。
 */
void App_Chassis_SetVelocity(float v, float w)
{
  s_target_v = v;
  s_target_w = w;
}

/**
 * 作用：急停。目标清零并立刻下发一次 0 转速，不等 5 ms 周期。
 */
void App_Chassis_Stop(void)
{
  uint8_t i;

  s_target_v = 0.0f;
  s_target_w = 0.0f;
  for (i = 0U; i < APP_MOTOR_NUM; i++)
  {
    BSP_Motor_SetSpeed(&s_motor[i], 0.0f);
  }
  if (s_ready != 0U)
  {
    (void)BSP_Motor_Send(s_motor, APP_MOTOR_NUM);
  }
}

/**
 * 作用：周期任务，放在 while(1) 里反复调用。收反馈 → 5 ms 门控 → 运动学 → 下发。
 * 内部自己做周期门控，调用频率高于 200 Hz 也没关系，多余调用会立刻返回。
 */
void App_Chassis_Task(void)
{
  uint32_t id;
  uint8_t  data[8];
  uint32_t now;
  float    rpm_left;
  float    rpm_right;
  uint8_t  i;

  if (s_ready == 0U)
  {
    return;
  }

  /* 1. 收反馈：一次把 FIFO 取空 */
  while (BSP_CAN_Recv(&id, data) == BSP_CAN_OK)
  {
    (void)BSP_Motor_Decode(s_motor, APP_MOTOR_NUM, id, data);
  }

  /* 2. 固定周期控制 */
  now = HAL_GetTick();
  if ((now - s_last_tick) < APP_CONTROL_PERIOD_MS)
  {
    return;
  }
  s_last_tick = now;

  /* 3. 运动学：底盘速度 → 左右轮目标转速 rpm
   *    v 取 + 是前进，w 取 + 是逆时针（左转）。
   *    差速公式：左轮 = v/R - w*L/(2R)，右轮 = v/R + w*L/(2R)，再换算成 rpm。 */
  Algo_Chassis_Inverse(&s_chassis, s_target_v, s_target_w, &rpm_left, &rpm_right);

  /* 4. 乘方向系数：如果某个电机装反了（给正转速却往后走），只改 app_config.h 里的 DIR，
   *    不要来这里改符号，也不要交换 CAN 线 */
  BSP_Motor_SetSpeed(&s_motor[0], rpm_left  * APP_MOTOR_LEFT_DIR);
  BSP_Motor_SetSpeed(&s_motor[1], rpm_right * APP_MOTOR_RIGHT_DIR);

  /* 5. 掉线电机给 0 速度，并定期重发使能帧，避免上电顺序不对一直不动 */
  for (i = 0U; i < APP_MOTOR_NUM; i++)
  {
    if (BSP_Motor_IsOnline(&s_motor[i], APP_MOTOR_OFFLINE_MS) == 0U)
    {
      BSP_Motor_SetSpeed(&s_motor[i], 0.0f);
      if ((now - s_enable_tick) >= APP_ENABLE_RETRY_MS)
      {
        s_enable_tick = now;
        (void)BSP_Motor_Enable(&s_motor[i], 1U);
      }
    }
  }

  /* 6. 一机一帧下发：MIT 帧的 ID 就是电机 ID，两台 = 两帧，不能合并 */
  (void)BSP_Motor_Send(s_motor, APP_MOTOR_NUM);
}

/**
 * 作用：读左轮实测转速 rpm（给上层调 PID / 看波形用），未收到反馈时返回上一次的值。
 */
float App_Chassis_GetLeftRpm(void)
{
  return s_motor[0].rpm;
}

/**
 * 作用：读右轮实测转速 rpm（给上层调 PID / 看波形用），未收到反馈时返回上一次的值。
 */
float App_Chassis_GetRightRpm(void)
{
  return s_motor[1].rpm;
}
