# 27 电赛小车底盘（car_dipan）

STM32G473VET6 + 2× M2006/C610 的两轮差速底盘，代码按 `App / BSP / Algorithm / Hardware` 四层组织。

---

## 1. 背景

面向全国大学生电子设计竞赛（电赛）小车类题目。这类题目通常要求小车在限定时间内完成循迹、避障、定点停靠等动作，对底盘的 **低速可控性** 和 **位置精度** 要求高于对极速的要求。

选型上放弃了直流减速电机 + 编码器 + 自制 H 桥的方案，改用大疆 RoboMaster 生态的 **M2006 无刷减速电机 + C610 电调**：

- 电机自带 13 位磁编码器（8192 counts/rev），配合 36:1 减速比，输出端分辨率远高于普通霍尔编码器；
- 电调内置电流环，主控只需要通过 CAN 下发 **目标转矩电流**，速度环放在主控里做，参数完全可控；
- 一套 CAN 总线最多挂 8 个电调，后续加云台、机械臂不需要增加走线。

## 2. 目的

1. 底盘能按上层给定的线速度 `v` 与角速度 `ω` 运动（差速转向，前部两个万向轮随动）；
2. 代码分层，硬件相关与算法解耦：换芯片只改 BSP，调算法不动驱动；
3. 所有可调参数集中一处，现场调车不需要翻代码；
4. 保留 FOC 与后续扩展（云台、IMU、遥控）的接口位置。

当前进度：屏幕点亮并显示「我是G473」；CAN 收发、M2006 协议解析、差速运动学、速度环 PID 已打通，等待实车标定。

## 3. 硬件

### 3.1 主控与外围

| 器件 | 型号 | 关键参数 | 接口 | 引脚 | 作用 |
|---|---|---|---|---|---|
| 主控 MCU | STM32G473VET6 | Cortex-M4F，170 MHz，512 KB Flash / 128 KB SRAM，LQFP100 | — | — | 整车控制 |
| 显示屏 | ST7789 IPS 2.00" | 320×240，RGB565，3.3 V，FFC 软排线（无排针） | SPI2 半双工 | PB13 SCK / PB15 MOSI | 状态显示、调参界面 |
|  |  |  | GPIO | PA7 CS / PB14 DC / PA5 RST / PA6 BLK | 片选/数据命令/复位/背光 |
| 调试串口 | USB-TTL | 115200 8N1 | USART1 | PC4 TX / PC5 RX | 打印调试信息 |

> SPI2 配成 **Half-Duplex Master（1LINE）、Mode 3、8 bit、10.625 Mbit/s**。因为 PB14 被借用作 LCD 的 DC 脚，必须关掉 MISO，否则 CubeMX 会把 PB14 占成 SPI2_MISO。

### 3.2 动力系统

| 器件 | 型号 | 关键参数 | 接口 | 数量 | 作用 |
|---|---|---|---|---|---|
| 减速电机 | DJI M2006 P36 | 无刷，减速比 36:1，编码器 8192 counts/rev | 三相线 → 电调 | 2 | 左右动力轮 |
| 电调 | DJI C610 | 输入 24 V，CAN 1 Mbps，内置电流环，反馈角度/转速/电流 | CAN | 2 | 驱动电机 |
| 终端电阻 | 120 Ω | 总线两端各一个（C610 内部**没有**） | CAN_H / CAN_L | 2 | 抑制反射 |

CAN 接线：`CAN_H` 红、`CAN_L` 黑，两端各 120 Ω 终端电阻。**C610 的调参串口与 CAN 口不能同时接**，首次使用需在 RoboMaster Assistant 里空载校准。

### 3.3 CAN 报文速查

| 方向 | 标识符 | 内容 |
|---|---|---|
| 下发 | `0x200`（ID 1~4）/ `0x1FF`（ID 5~8） | 4 个 `int16` 目标电流，大端，槽位 `(id-1)%4` |
| 反馈 | `0x201`~`0x208` | `[0:1]`机械角 0~8191、`[2:3]`转速 rpm、`[4:5]`转矩电流、`[6]`温度（C610 无此项）、`[7]`保留 |

电流量程：M2006/C610 `±10000` ≈ `±10 A`。首测限流 `±3000`（≈3 A）。

完整协议、量程换算、调试顺序见 [docs/M2006_C610_CAN手册.md](docs/M2006_C610_CAN手册.md)。

## 4. 软件结构

### 4.1 分层

依赖方向**只能从上往下**，下层永远不 `#include` 上层。

```
App            业务层：各类 Task、状态机、指令解析        app_chassis.c
  ↑
Algorithm      纯算法：PID、运动学、滤波（不含任何 HAL）   algo_pid.c / algo_kinematics.c
  ↑
BSP            板级驱动：CAN 收发、电机协议打包解包        bsp_can.c / bsp_motor.c
  ↑
Hardware       外设驱动：屏幕时序（SPI/GPIO 搬运）        lcd.c / lcd_port.c / lcd_font.c
  ↑
HAL / CMSIS    CubeMX 生成，禁止修改
```

判断一个新文件该放哪层：**知道寄存器/引脚吗？** 知道 → BSP；只算数 → Algorithm；知道「底盘要多快」这种业务意图 → App。

### 4.2 目录

```
car_dipan/
├─ Core/                   CubeMX 生成（仅 USER CODE 段可写）
│  ├─ Inc/
│  └─ Src/  main.c  fdcan.c  spi.c  usart.c  gpio.c  ...
├─ Drivers/                CubeMX 生成，HAL + CMSIS
├─ MDK-ARM/                构建配置：car_dipan.uvprojx、startup、.eide/eide.yml
├─ Hardware/               屏幕驱动
│  ├─ lcd.c / lcd.h           ST7789 命令与绘图 API
│  ├─ lcd_port.c / .h         引脚与 SPI 搬运层
│  └─ lcd_font.c / .h         字库
├─ BSP/
│  ├─ bsp_can.c / .h          FDCAN1 启动、过滤、8 字节收发
│  └─ bsp_motor.c / .h        M2006/C610 电流下发、反馈解码、多圈累加
├─ Algorithm/
│  ├─ algo_math.h             限幅、低通、角度归一化等 inline 工具
│  ├─ algo_pid.c / .h         位置式 PID（积分限幅 + 抗饱和）
│  └─ algo_kinematics.c / .h  两轮差速正逆解
├─ App/
│  ├─ app_config.h            全部可调参数（电机 ID/方向/轮径/PID）
│  └─ app_chassis.c / .h      底盘 Task
├─ docs/
│  └─ M2006_C610_CAN手册.md
├─ car_dipan.ioc            CubeMX 工程文件
├─ modify.md                修改记录（每次改动都要追加）
└─ README.md
```

### 4.3 调用链

```c
/* main.c 仅 USER CODE 段 */
App_Chassis_Init();                      // CAN 启动 + 电机注册 + PID 参数
App_Chassis_SetVelocity(0.2f, 0.0f);     // 目标线速度 0.2 m/s，角速度 0
while (1) {
    App_Chassis_Task();                  // 非阻塞，内部 5 ms 节拍（200 Hz）
}
```

`App_Chassis_Task()` 每轮：取空 CAN FIFO → 解析反馈 → 差速逆解得左右轮目标 rpm → 速度环 PID → 打包电流下发。掉线电机（>100 ms 无反馈）会自动清零电流并复位积分。

## 5. 工具链

| 用途 | 工具 | 说明 |
|---|---|---|
| 外设配置 | STM32CubeMX + STM32G4 固件包 | 改完重新生成，**不要手改生成物** |
| 编译/下载 | Keil MDK-ARM 5（AC5） | 打开 `MDK-ARM/car_dipan.uvprojx` |
| 编译（VS Code） | EIDE 插件 | 读 `MDK-ARM/.eide/eide.yml`，改完需 `Developer: Reload Window` |
| 语法检查 | arm-none-eabi-gcc（STM32CubeCLT） | 仅 `-fsyntax-only`，用于提交前自检 |
| 电调校准 | DJI RoboMaster Assistant | 首次上电空载校准用 |
| 版本管理 | Git + GitHub | 仓库：`QU1453/27-car-chassis` |

## 6. 构建与烧录

1. **CubeMX 侧**：改了引脚/外设 → 打开 `car_dipan.ioc` → 重新 Generate Code；
2. **EIDE 侧**：`Developer: Reload Window` → Build（新加的 `.c` 必须先进 `eide.yml`，否则链接报 undefined）；
3. **Keil 侧**：直接 Build → Download，ST-Link/SWD 烧录；
4. **上电自检**：串口 115200 看打印，屏幕应显示「我是G473」。

## 7. 基础规则

> 这几条是踩过坑之后定下来的，改代码前先看一遍。

1. **不修改 CubeMX 生成的文件**：`Core/`（USER CODE 段以外）、`Drivers/`、`Core/Src/*_msp.c`、默认的 `stm32g4xx_it.c`、`car_dipan.ioc`、`car_dipan.uvprojx`。
   CubeMX 需要改配置时，在 `.ioc` 里改并重新生成；手改会在下次 Generate 时被覆盖，且不可复现。
   `main.c` 只允许写 `/* USER CODE BEGIN xxx */` 与 `/* USER CODE END xxx */` 之间。
2. **分层依赖单向**：App → Algorithm / BSP → Hardware → HAL。Algorithm 层不允许出现任何 `HAL_xxx()` 调用。
3. **参数集中**：所有现场要调的数值（轮径、轮距、PID、限流、电机 ID、安装方向）只写在 `App/app_config.h`，不要散落在 `.c` 里。
4. **命名前缀**：`bsp_` / `algo_` / `app_` 对应三层；对外函数用 `模块_动作()` 形式，模块内变量加 `s_` 前缀。
5. **每次改动都记 `modify.md`**：改了什么、为什么、怎么回滚，三段以内。
6. **新增 `.c` 必须同步进 `eide.yml`**，并确认头文件目录已加入 `incList`。
7. **安全限流**：新装或改线后的第一次上电，`MOTOR_SAFE_CURRENT_RAW` 保持 3000，轮子悬空验证方向正确后再落地。

## 8. 已知问题 / 待办

- [ ] `Core/Src/fdcan.c` 的位时序仍是 CubeMX 默认值（约 3.54 Mbps），**CAN 实际不通**。需在 CubeMX 里改为 `Prescaler=10 / TimeSeg1=13 / TimeSeg2=3 / SJW=1`、`StdFiltersNbr=1`、`AutoRetransmission=Enable`、PA11/PA12 `Very High`。
- [ ] `App/app_config.h` 的 `APP_WHEEL_RADIUS`、`APP_WHEEL_BASE` 为占位值，需实测。
- [ ] 上电缓启动（电流斜坡）未实现。
- [ ] 未接 IMU，航向角闭环待做。

## 9. 相关文档

- [M2006/C610 CAN 手册](docs/M2006_C610_CAN手册.md)
- [修改记录](modify.md)
