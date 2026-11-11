#include "bsp_motor.h"
#include "algo_math.h"
#include "fdcan.h"      /* 只为了用 HAL_GetTick() 打时间戳 */

/* =====================================================================
 * bsp_motor.c —— 达妙 DM-S2325-1EC 驱动的实现（BSP 层）
 *
 * 文件作用：
 *   1) 把 rpm 等物理量压成 MIT 帧的定点位域，拼成 8 字节下发；
 *   2) 把 8 字节反馈解回物理量，写进 BSP_Motor_t。
 *   协议细节见 docs/DM_S2325_CAN手册.md，这里只实现最常用的 MIT 定速。
 *
 * 命名对照：
 *   float_to_uint / uint_to_float = 浮点压成定点 / 定点还原成浮点
 *   send_mit_frame = 组装并发送一帧 MIT 控制帧
 *   buf            = 要发出去的 8 字节缓冲区
 *   p / v / kp / kd / t = 位置 / 速度 / 位置增益 / 速度增益 / 力矩前馈（均已定点化）
 *   span           = 量程跨度（x_max - x_min）
 *   max_i          = 该位宽能表示的最大整数（2^bits - 1）
 *   raw_p / raw_v / raw_t = 从反馈帧里拆出来的原始定点值
 * ===================================================================== */

/* 作用：把物理量线性压进 bits 位无符号定点，超量程直接截断到端点 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
  float    span  = x_max - x_min;
  uint32_t max_i = (1UL << bits) - 1UL;

  if (x < x_min) { x = x_min; }
  else if (x > x_max) { x = x_max; }

  return (uint16_t)((x - x_min) * (float)max_i / span);
}

/* 作用：float_to_uint 的逆运算，把 bits 位无符号定点还原成物理量 */
static float uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
  uint32_t max_i = (1UL << bits) - 1UL;

  return (float)x * (x_max - x_min) / (float)max_i + x_min;
}

/* 作用：组一帧 MIT 控制帧并按 ID = 电机 ID 发出去。
 * 位域顺序：p_des(16) + v_des(12) + Kp(12) + Kd(12) + t_ff(12) = 64 bit */
static uint8_t send_mit_frame(const BSP_Motor_t *motor)
{
  uint8_t  buf[8];
  uint16_t p;
  uint16_t v;
  uint16_t kp;
  uint16_t kd;
  uint16_t t;

  /* 先把 5 个物理量各自压成定点整数（这一步决定了精度和量程）：
   *   p  - 目标位置 rad，16 位，定速模式下恒为 0（Kp=0 时它不参与运算）
   *   v  - 目标速度 rad/s，12 位，由 cmd_rpm 换算而来，这是唯一真正起作用的量
   *   kp - 位置增益，12 位，定速模式下必须为 0，否则会跟速度环抢控制权
   *   kd - 速度增益，12 位，就是驱动器内部速度环的 P 增益，靠它把误差压下去
   *   t  - 力矩前馈 N·m，12 位，定速模式下给 0（要额外补重力/摩擦时才填） */
  p  = float_to_uint(0.0f, -DM_P_MAX, DM_P_MAX, 16U);
  v  = float_to_uint(Math_RpmToRadPerSec(motor->cmd_rpm), -DM_V_MAX, DM_V_MAX, 12U);
  kp = float_to_uint(0.0f, DM_KP_MIN, DM_KP_MAX, 12U);
  kd = float_to_uint(DM_KD_DEFAULT, DM_KD_MIN, DM_KD_MAX, 12U);
  t  = float_to_uint(0.0f, -DM_T_MAX, DM_T_MAX, 12U);

  /* 再按大端顺序拼成 8 字节（共 64 bit）：
   *
   *   buf[0]      buf[1] | buf[2]      buf[3]        | buf[4]
   *   p[15:8]     p[7:0]   v[11:4]     v[3:0] | kp[11:8]  kp[7:0]
   *
   *   buf[5]      buf[6]                | buf[7]
   *   kd[11:4]    kd[3:0] | t[11:8]     t[7:0]
   *
   * 注意 12 位的量是「跨字节」的，高低半字节要分别塞进相邻两个字节，
   * 少写一个掩码就会把相邻字段污染，表现是转速莫名其妙的跳变 */
  buf[0] = (uint8_t)(p >> 8);                                     /* p 高 8 位 */
  buf[1] = (uint8_t)(p & 0xFFU);                                  /* p 低 8 位 */
  buf[2] = (uint8_t)(v >> 4);                                     /* v 高 8 位（v[11:4]） */
  buf[3] = (uint8_t)(((v & 0x0FU) << 4) | ((kp >> 8) & 0x0FU));    /* v 低 4 位 + kp 高 4 位 */
  buf[4] = (uint8_t)(kp & 0xFFU);                                 /* kp 低 8 位 */
  buf[5] = (uint8_t)(kd >> 4);                                    /* kd 高 8 位（kd[11:4]） */
  buf[6] = (uint8_t)(((kd & 0x0FU) << 4) | ((t >> 8) & 0x0FU));    /* kd 低 4 位 + t 高 4 位 */
  buf[7] = (uint8_t)(t & 0xFFU);                                  /* t 低 8 位 */

  /* 帧 ID 直接用电机 ID（不加 0x100/0x200 偏移，那是位置速度模式和纯速度模式的用法） */
  return BSP_CAN_Send((uint32_t)motor->id, buf);
}

/* 作用：登记 ID / Master ID，其余状态清零。注意此时电机还没使能 */
void BSP_Motor_Init(BSP_Motor_t *motor, uint8_t id, uint8_t master_id)
{
  motor->id        = id;
  motor->master_id = master_id;
  motor->err       = DM_ERR_DISABLED;
  motor->t_mos     = 0U;
  motor->t_rotor   = 0U;
  motor->cmd_rpm   = 0.0f;
  motor->rpm       = 0.0f;
  motor->pos_rad   = 0.0f;
  motor->tor_nm    = 0.0f;
  motor->last_tick = 0U;
}

/* 作用：写目标转速，按驱动器速度上限限幅。
 * 限幅放在 rpm 侧做，避免下发时再判断一次符号 */
void BSP_Motor_SetSpeed(BSP_Motor_t *motor, float rpm)
{
  float rad_max = DM_V_MAX;                          /* 驱动器速度上限 */
  float rpm_max = Math_RadPerSecToRpm(rad_max);      /* 换成 rpm 再限幅 */

  motor->cmd_rpm = Math_Clampf(rpm, -rpm_max, rpm_max);
}

/* 作用：对一组电机逐个发控制帧。单台失败不影响后面几台，最后统一返回结果 */
uint8_t BSP_Motor_Send(BSP_Motor_t *motors, uint8_t count)
{
  uint8_t ret = BSP_CAN_OK;
  uint8_t i;

  for (i = 0U; i < count; i++)
  {
    if (send_mit_frame(&motors[i]) != BSP_CAN_OK)
    {
      ret = BSP_CAN_ERROR;
    }
  }

  return ret;
}

/* 作用：使能/失能的便捷封装，enable != 0 即使能 */
uint8_t BSP_Motor_Enable(BSP_Motor_t *motor, uint8_t enable)
{
  return BSP_Motor_SendCmd(motor, (enable != 0U) ? DM_CMD_ENABLE : DM_CMD_DISABLE);
}

/* 作用：发 8 字节特殊命令帧：前 7 字节全 0xFF，最后一字节是命令码 */
uint8_t BSP_Motor_SendCmd(BSP_Motor_t *motor, uint8_t cmd)
{
  uint8_t buf[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};

  buf[7] = cmd;
  return BSP_CAN_Send((uint32_t)motor->id, buf);
}

/* 作用：解析一帧反馈，按 master_id 找到对应电机并更新它的字段。
 * 命中返回 1；不是本组电机的帧返回 0（上层可直接丢弃） */
uint8_t BSP_Motor_Decode(BSP_Motor_t *motors, uint8_t count,
                         uint32_t std_id, const uint8_t data[8])
{
  uint16_t raw_p;
  uint16_t raw_v;
  uint16_t raw_t;
  uint8_t  i;

  /* 反馈帧 ID = Master ID；驱动器只比对低 8 位 */
  for (i = 0U; i < count; i++)
  {
    if ((uint8_t)(std_id & 0xFFU) != motors[i].master_id)
    {
      continue;
    }

    /* D0：高 4 位状态码，低 4 位电机 ID */
    motors[i].err = (uint8_t)((data[0] >> 4) & 0x0FU);

    /* 反馈帧 8 字节的布局（和发送帧的位域顺序不一样，别抄错）：
     *   data[0]  = ERR[7:4] | ID[3:0]
     *   data[1]  = POS[15:8]
     *   data[2]  = POS[7:0]
     *   data[3]  = VEL[11:4]
     *   data[4]  = VEL[3:0]   | TOR[11:8]     ← 高半字节是速度，低半字节是力矩
     *   data[5]  = TOR[7:0]
     *   data[6]  = MOS 温度 ℃
     *   data[7]  = 绕组温度 ℃ */
    raw_p = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);            /* 位置 16 位 */
    raw_v = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));     /* 速度 12 位 */
    raw_t = (uint16_t)(((uint16_t)(data[4] & 0x0FU) << 8) | data[5]);  /* 力矩 12 位 */

    /* 定点还原成物理量：驱动器给的都是输出轴（减速后）的量，直接用，不再乘减速比 */
    motors[i].pos_rad = uint_to_float(raw_p, -DM_P_MAX, DM_P_MAX, 16U);
    motors[i].rpm     = Math_RadPerSecToRpm(
                          uint_to_float(raw_v, -DM_V_MAX, DM_V_MAX, 12U));
    motors[i].tor_nm  = uint_to_float(raw_t, -DM_T_MAX, DM_T_MAX, 12U);

    /* 温度是原始摄氏度，不做换算；超过 80 ℃ 应该上层限功率 */
    motors[i].t_mos     = data[6];
    motors[i].t_rotor   = data[7];
    /* 打时间戳，IsOnline 靠它判断这一路还活着没有 */
    motors[i].last_tick = HAL_GetTick();
    return 1U;
  }

  return 0U;
}

/* 作用：判在线。用「当前时刻 - 最后收帧时刻」和超时阈值比，
 * 用减法而不是绝对值比较，是为了让 HAL_GetTick() 回绕时依然正确 */
uint8_t BSP_Motor_IsOnline(const BSP_Motor_t *motor, uint32_t timeout_ms)
{
  if (motor->last_tick == 0U)
  {
    return 0U;                                        /* 一帧都没收到过 */
  }

  return ((HAL_GetTick() - motor->last_tick) <= timeout_ms) ? 1U : 0U;
}
