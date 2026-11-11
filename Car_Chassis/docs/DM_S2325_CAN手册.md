# DM-S2325-1EC 使用手册（精简）

> 适用：达妙 DM-S2325-1EC 减速电机 + DM3520-1EC 驱动器。
> 本工程：STM32G473VET6 / FDCAN1 / 1 Mbps Classic CAN / 2 个电机差速底盘。
> 选用模式：**MIT 模式，kp = 0、kd ≠ 0（定速）**。

---

## 1. 硬件与接线

| 项目 | 内容 |
|---|---|
| 电机 | DM-S2325-1EC，额定 1.35 Nm，峰值 5 Nm |
| 驱动器 | DM3520-1EC，与电机**分离**（4-pin 编码器线 + 三相线连接） |
| 供电 | 24 V，XT30 接口 |
| 总线 | CAN **标准帧**，出厂默认 **1 Mbps**（可用命令改）；另支持 CAN FD 最高 5 Mbps |
| 终端电阻 | 总线两端各 120 Ω，本工程另一端必须有 |
| 接口 | 电源 XT30 / 串口（调试配参）/ CAN（控制）/ 编码器 / 三相 —— 五路独立 |
| 上电位置 | 输出轴位置**上电即清零**为 0.0 rad，不做多圈保持 |

输出轴量纲已经过减速比，**位置 rad、速度 rad/s、力矩 Nm 都是减速后**，主控不需要再管减速比。

---

## 2. CAN ID 规划（最容易翻车的一步）

| 帧 | ID 规则 |
|---|---|
| MIT 控制帧 | `ID`（不加偏移） |
| 位置速度帧 | `0x100 + ID` |
| 速度帧 | `0x200 + ID` |
| 反馈帧 | `Master ID`，**出厂默认 0** |

匹配规则：驱动器只比对收到的 ID **低 8 位**，高 3 位忽略。

> ⚠ **两台电机的 Master ID 必须改成不同值**，否则反馈帧在总线上撞车，主控无法区分左右轮。
> 出厂 CAN ID 也不保证互不相同，必须先用达妙上位机逐台确认/修改并保存。

建议规划（写进 `app_config.h`）：

| | 电机 ID | Master ID |
|---|---|---|
| 左轮 | 1 | 0x11 |
| 右轮 | 2 | 0x12 |

M2006 那套「4 个电机挤一帧 0x200」的做法在这里**不适用**，DM 是一帧一机。

---

## 3. 反馈帧（驱动器 → 主机）

帧 ID = `Master ID`，8 字节。

| D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 |
|---|---|---|---|---|---|---|---|
| `ID \| ERR<<4` | `POS[15:8]` | `POS[7:0]` | `VEL[11:4]` | `VEL[3:0] \| T[11:8]` | `T[7:0]` | `T_MOS` | `T_Rotor` |

`ID` 取 CAN_ID 低 4 位；`T_MOS`、`T_Rotor` 单位 ℃。

**ERR 状态码**

| 值 | 含义 | 值 | 含义 |
|---|---|---|---|
| 0 | 失能 | 9 | 欠压 |
| 1 | 使能 | A | 过流 |
| 5 | 传感器读取错误 | B | MOS 过温 |
| 6 | 电机参数读取错误 | C | 绕组过温 |
| 8 | 过压 | D | 通信丢失 |
| | | E | 过载 |

**解码**（`pos` 16 位、`vel` / `t` 各 12 位，线性映射）：

```c
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

uint8_t  id     = rx[0] & 0x0F;                     /* D0 低 4 位 = 电机 ID */
uint8_t  err    = (rx[0] >> 4) & 0x0F;              /* D0 高 4 位 = 状态 */
uint16_t pos    = ((uint16_t)rx[1] << 8) | rx[2];
uint16_t vel    = ((uint16_t)rx[3] << 4) | (rx[4] >> 4);
uint16_t tor    = (((uint16_t)(rx[4] & 0x0F)) << 8) | rx[5];
uint8_t  t_mos  = rx[6];
uint8_t  t_rot  = rx[7];

pos_f = uint_to_float(pos, -P_MAX, P_MAX, 16);
vel_f = uint_to_float(vel, -V_MAX, V_MAX, 12);
tor_f = uint_to_float(tor, -T_MAX, T_MAX, 12);
```

> 反馈是 **query-based**：驱动器只要收到 ID 匹配自己的帧就回一帧。
> 所以「发一帧控制 → 收一帧反馈」天然同步，不用额外轮询；控制 200 Hz 就是反馈 200 Hz。

---

## 4. MIT 控制帧（本项目使用）

帧 ID = `ID`，8 字节，把 5 个参数压进 64 bit。

| D0 | D1 | D2 | D3 | D4 | D5 | D6 | D7 |
|---|---|---|---|---|---|---|---|
| `p_des[15:8]` | `p_des[7:0]` | `v_des[11:4]` | `v_des[3:0] \| Kp[11:8]` | `Kp[7:0]` | `Kd[11:4]` | `Kd[3:0] \| t_ff[11:8]` | `t_ff[7:0]` |

| 参数 | 位宽 | 量程来源 |
|---|---|---|
| `p_des` | 16 | ±P_MAX（上位机可配） |
| `v_des` | 12 | ±V_MAX（上位机可配） |
| `Kp` | 12 | [0, 500] × 1/Kp_scale |
| `Kd` | 12 | [0, 5] × 1/Kd_scale |
| `t_ff` | 12 | ±T_MAX（上位机可配） |

**P_MAX / V_MAX / T_MAX 必须从电机里读出来**，不能照抄别人的值——它们存在驱动器内部，可用上位机「读参数」看到。写死错的值会导致目标力矩整体缩放错误。

```c
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (uint16_t)((x - x_min) * ((float)((1 << bits) - 1)) / span);
}

uint16_t p  = float_to_uint(p_des, -P_MAX, P_MAX, 16);
uint16_t v  = float_to_uint(v_des, -V_MAX, V_MAX, 12);
uint16_t kp = float_to_uint(kp,     KP_MIN, KP_MAX, 12);
uint16_t kd = float_to_uint(kd,     KD_MIN, KD_MAX, 12);
uint16_t t  = float_to_uint(t_ff,   -T_MAX, T_MAX, 12);

buf[0] = (uint8_t)(p >> 8);
buf[1] = (uint8_t)(p & 0xFF);
buf[2] = (uint8_t)(v >> 4);
buf[3] = (uint8_t)(((v & 0x0F) << 4) | ((kp >> 8) & 0x0F));
buf[4] = (uint8_t)(kp & 0xFF);
buf[5] = (uint8_t)(kd >> 4);
buf[6] = (uint8_t)(((kd & 0x0F) << 4) | ((t >> 8) & 0x0F));
buf[7] = (uint8_t)(t & 0xFF);
```

### 4.1 派生用法

| 想要 | 设置 |
|---|---|
| **定速（本项目）** | `Kp = 0`、`Kd ≠ 0`、`p_des = 0`、`t_ff = 0`，只给 `v_des` |
| 纯力矩 | `Kp = 0`、`Kd = 0`，只给 `t_ff` |
| 位置 | `Kp ≠ 0`、`Kd ≠ 0`（**Kd 绝不能填 0，否则振荡甚至失控**） |

定速时驱动器内部相当于 `t = Kd × (v_des − v_实际)`，本质上是一个**速度 P 环**。
所以主控外侧再套一层 PID 时要小心串级打架——建议先用 Kd 单环跑通，再决定要不要外环。

Kd 起步建议 0.3 左右，别一上手就给大值（参考达妙调试助手的示例量级；最终以实车为准）。

---

## 5. 另外两种模式（备查，本项目暂不用）

**位置速度模式**：帧 ID = `0x100 + ID`，`p_des`(float LE) + `v_des`(float LE)，`v_des` 是运动过程中的最大速度。

**速度模式**：帧 ID = `0x200 + ID`，只有 4 字节 `v_des`(float LE)，单位 rad/s。

两者都是 **little-endian**：

```c
memcpy(&buf[0], &p_des, 4);
memcpy(&buf[4], &v_des, 4);
```

如果哪天觉得 MIT 打包麻烦，速度模式其实最省事（4 字节 float 直接发）。但要注意：**控制模式必须在电机里切换并保存**，否则发过去电机不响应。

---

## 6. 使能 / 失能 / 保存零点 / 清错

这四个是**固定 8 字节的控制帧**，帧 ID = 电机 `ID`，前 7 字节全 `0xFF`，最后 1 字节是命令码：

| 功能 | D7 | 完整数据 |
|---|---|---|
| 使能 | `0xFC` | `FF FF FF FF FF FF FF FC` |
| 失能 | `0xFD` | `FF FF FF FF FF FF FF FD` |
| 保存零点 | `0xFE` | `FF FF FF FF FF FF FF FE` |
| 清除错误 | `0xFB` | `FF FF FF FF FF FF FF FB` |

> ⚠ 以上为达妙 SDK 的通用约定，本次未能从 S2325 官方说明书正文逐字核对（PDF 本机打不开）。
> **首次接入必须用 USB2CAN + 上位机抓包实测确认**，确认后再写进 `bsp_motor.c`。

上电流程：`使能` → 等反馈 `ERR == 1` → 开始发控制帧。反馈 `ERR == 0` 说明没使能成功，别急着发速度。

---

## 7. 与现有代码的差异（换电机待改清单）

| 项 | 现在（M2006/C610） | 换 DM-S2325 后 |
|---|---|---|
| 下发帧 | `0x200` / `0x1FF`，4 机共用 | 一机一帧，ID = 电机 ID |
| 下发内容 | 电流 raw `int16` | MIT 位域（p/v/kp/kd/t） |
| 力矩量程 | ±10000 raw ≈ ±10 A | ±T_MAX Nm（读参数） |
| 位置 | ecd 0~8191，需多圈累加 | 输出轴 rad，已减速 |
| 反馈帧 | 固定 `0x201~0x208` 广播 | `Master ID`，需逐台配置 |
| 减速比 | 36:1（`APP_GEAR_RATIO`） | **删除**，量纲已在输出轴 |

待改文件（本次未动）：

- `BSP/bsp_motor.c/h` — 协议整体重写
- `App/app_config.h` — 删 `APP_GEAR_RATIO`，加 `P_MAX/V_MAX/T_MAX`、Master ID
- `Algorithm/algo_kinematics.c` — 去掉减速比，只用轮径
- `README.md` 硬件表
- `docs/M2006_C610_CAN手册.md` — 保留但标注已弃用

---

## 8. 坑

1. **FDCAN 位时序**：`Core/Src/fdcan.c` 现在是 `Prescaler=16 / Seg1=1 / Seg2=1`，算出约 3.54 Mbps，而 DM 默认 1 Mbps，**必然通信不上**。必须在 CubeMX 里改成 `10 / 13 / 3 / SJW=1`（该文件禁手改）。
2. **Master ID 撞车**：两台电机默认都是 0，不改就分不出左右轮。
3. **Kp/Kd 量程不是 1:1**：`Kp` 映射到 [0,500]、`Kd` 映射到 [0,5]，别人的代码里 `Kp=500` 表示满量程，不是增益 500。
4. **控制模式必须先在电机里设成 MIT**，设成位置速度模式时发 MIT 帧电机不响应。
5. **反馈 `ERR` 要一直看**：`D`（通信丢失）、`C`（绕组过温）出现就立刻停。
6. **上电位置清零**：不要指望绝对位置，底盘也用不到，做里程靠速度积分。
