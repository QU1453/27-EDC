#include "algo_pid.h"
#include "algo_math.h"

/* =====================================================================
 * algo_pid.c —— 位置式 PID 的实现
 *
 * 文件作用：
 *   实现 algo_pid.h 的四个函数：填参数、设目标、清状态、单周期计算。
 *
 * 命名对照：
 *   p      = 比例项        i_next = 积分项候选值
 *   d      = 微分项        out    = 最终输出（饱和后）
 *   dt     = 本次与上次调用的时间间隔（秒）
 *   pid    = PID 实例指针
 *
 * dt 的硬性要求：
 *   ki 的量纲是「输出 / (误差·秒)」，kd 是「输出·秒 / 误差」。
 *   同一个 ki 在 5 ms 和 10 ms 周期下强度差一倍，所以控制周期一改，
 *   ki / kd 必须重新整定（本工程 APP_CONTROL_PERIOD_MS = 5）。
 * ===================================================================== */

/* 作用：填参数并把状态全部清零。
 * 这里不判 pid 是否为空，调用方保证不传 NULL（与 HAL 风格一致） */
void Algo_PID_Init(Algo_PID_t *pid, float kp, float ki, float kd,
                   float out_max, float integral_max)
{
  pid->kp           = kp;
  pid->ki           = ki;
  pid->kd           = kd;
  pid->out_max      = out_max;
  pid->integral_max = integral_max;
  pid->target       = 0.0f;
  pid->feedback     = 0.0f;
  pid->error        = 0.0f;
  pid->last_error   = 0.0f;
  pid->integral     = 0.0f;
  pid->output       = 0.0f;
}

/* 作用：只改目标值。
 * 运行中切目标不要顺手 Reset —— 积分一清零，输出会从"带积分"突然掉到"纯 P" */
void Algo_PID_SetTarget(Algo_PID_t *pid, float target)
{
  pid->target = target;
}

/* 作用：清状态、保留参数（增益和限幅是调好的成果，不能清）。
 * last_error 也必须清，否则下一拍微分项会用旧误差算出巨大尖峰 */
void Algo_PID_Reset(Algo_PID_t *pid)
{
  pid->error      = 0.0f;
  pid->last_error = 0.0f;
  pid->integral   = 0.0f;
  pid->output     = 0.0f;
}

/* 作用：算一拍控制量。分 6 步，顺序不能乱 */
float Algo_PID_Update(Algo_PID_t *pid, float feedback, float dt)
{
  float p;
  float i_next;
  float d;
  float out;

  /* ---- 1) 存反馈、算误差 ---- */
  pid->feedback = feedback;
  pid->error    = pid->target - feedback;

  /* ---- 2) 比例项：只看"现在"的误差 ---- */
  p = pid->kp * pid->error;

  /* ---- 3) 积分项：先在候选值上累加，再整体限幅 ----
   * 先算 i_next 再 Clamp，而不是先 Clamp 旧值再累加：
   * 后者累加完那一拍就已超界，输出会带着超界值出去一拍。
   * 用候选值而不是直接改 pid->integral，是为了第 5 步能干净地回滚 */
  i_next = pid->integral + pid->ki * pid->error * dt;
  i_next = Math_Clampf(i_next, -pid->integral_max, pid->integral_max);

  /* ---- 4) 微分项：误差的变化率 ----
   * dt > 0 必须判：dt=0 会除出 inf/nan，并沿 integral / last_error 传播，
   * 整条控制链再也恢复不了。
   * 首拍 last_error=0，所以目标一开始不为 0 时会看到一下尖峰，这是位置式的正常现象 */
  if (dt > 0.0f)
  {
    d = pid->kd * (pid->error - pid->last_error) / dt;
  }
  else
  {
    d = 0.0f;
  }

  /* ---- 5) 求和后判饱和 ----
   * 只在"误差与饱和同向"时回滚积分：
   *   顶到 +max 且误差仍为正 → 增量留着就是纯 windup，丢掉；
   *   顶到 +max 但误差已为负 → 增量在帮输出退出饱和，必须保留。
   * 反方向同理 */
  out = p + i_next + d;

  /* 饱和时不再往同一方向积分，避免积分饱和 */
  if (out > pid->out_max)
  {
    out = pid->out_max;
    if (pid->error > 0.0f)
    {
      i_next = pid->integral;
    }
  }
  else if (out < -pid->out_max)
  {
    out = -pid->out_max;
    if (pid->error < 0.0f)
    {
      i_next = pid->integral;
    }
  }

  /* ---- 6) 回写状态 ----
   * 必须写回 i_next（可能已限幅、也可能被回滚），
   * 不能写 pid->integral += ...，那样限幅只管输出，累加器仍会无限变长 */
  pid->integral   = i_next;
  pid->last_error = pid->error;
  pid->output     = out;
  return out;
}
