# 文件结构（car_dipan）

> 分层单向依赖：`App → Algorithm / BSP → Hardware → HAL`。上层可以调下层，下层不许回头调上层。
> 判断文件放哪层：**它知道寄存器/引脚吗？** 知道 → Hardware；不知道但要操作外设 → BSP；
> 纯数学/纯逻辑 → Algorithm；把上面几个串起来干活 → App。

## 1. 目录树

```
car_dipan/
├── App/                      应用层（业务编排）
│   ├── app_chassis.c/h       底盘任务：v/ω → 左右轮转速 → 下发（含使能重发）
│   └── app_config.h          全部可调参数集中地（ID/Master ID/轮径/轮距/周期）
├── Algorithm/                算法层（禁止出现任何 HAL_xxx）
│   ├── algo_math.h           数学工具，纯 inline（限幅、映射、角度归一、快开方）
│   ├── algo_pid.c/h          PID 控制器
│   └── algo_kinematics.c/h   差速运动学：v/ω ↔ 左右轮线速度
├── BSP/                      板级驱动层（会用 HAL，但不碰 CubeMX 文件）
│   ├── bsp_can.c/h           FDCAN1 收发封装：启动、过滤器、发帧、收帧回调
│   └── bsp_motor.c/h         DM-S2325 驱动：MIT 组帧、反馈解码、使能/失能
├── Hardware/                 芯片外设驱动（只依赖 HAL + 引脚宏）
│   ├── lcd.c/h               ST7789 屏驱动（绘图 API）
│   ├── lcd_font.c/h          字库
│   └── lcd_port.c/h          SPI/GPIO 底层时序与引脚映射
├── Core/                     ⚠ CubeMX 生成，禁手改
│   ├── Inc/  fdcan.h gpio.h main.h spi.h usart.h stm32g4xx_hal_conf.h stm32g4xx_it.h
│   └── Src/  fdcan.c gpio.c main.c spi.c usart.c stm32g4xx_hal_msp.c stm32g4xx_it.c system_stm32g4xx.c
├── Drivers/                  ⚠ ST HAL 库 + CMSIS，禁改
├── .vscode/tasks.json        编译/烧录任务（Ctrl+Shift+B = 编译并烧录）
├── MDK-ARM/                  Keil 工程 + .eide/eide.yml（EIDE 项目树靠它）
├── docs/
│   ├── M2006_C610_CAN手册.md    旧电机协议（已弃用，备查）
│   ├── DM_S2325_CAN手册.md      新电机协议（当前使用）
│   └── mit_frame_demo.html      MIT 帧 2D 动画演示页（浏览器双击即开）
├── car_dipan.ioc             ⚠ CubeMX 工程文件，禁手改
├── README.md                 背景/目的/硬件/工具链/规则
├── modify.md                 修改记录（唯一）
├── IMPORTANT_FIXES.md        重要问题修复（现象/原因/改动/回滚）
└── struct.md                 本文件
```

## 2. 各层唯一入口

| 层 | 对外只暴露 | 调用方 |
|---|---|---|
| App | `App_Chassis_Init` / `App_Chassis_SetVelocity` / `App_Chassis_Task` | `main.c` |
| BSP | `BSP_CAN_Init/Send/Recv`、`BSP_Motor_Init/SetSpeed/Send/Enable/Decode` | App |
| Algorithm | `Algo_Kinematics_*`、`Algo_PID_*` | App |

## 3. 新增文件放哪

| 你想写的东西 | 放这 |
|---|---|
| 新的传感器芯片驱动（I2C/SPI 时序） | `Hardware/` |
| 某个模块的通信协议（CAN/串口/UART 帧） | `BSP/` |
| 纯计算：滤波、PID、运动学、坐标变换 | `Algorithm/` |
| 一个能独立跑的「任务」（底盘、云台、上位机） | `App/` |
| 可调数字（速度、限幅、ID、周期） | `App/app_config.h` |

命名前缀跟着层走：`bsp_` / `algo_` / `app_`。

## 4. 规矩

1. **`Core/`、`Drivers/`、`*.ioc` 一律不改**；`main.c` 只写 `USER CODE` 段。
   `car_dipan.uvprojx` 只允许「加 `<Group>` / 扩 `<IncludePath>`」这一种登记性改动，编译选项一律不动，且改前要先问。
2. `Algorithm/` 里出现 `HAL_` 就是分层破了。
3. 新增 `.c` 要登记**两个地方**，缺一个就不显示/不编译：
   `MDK-ARM/car_dipan.uvprojx` 的 `<Group>`（Keil 侧，也是 EIDE 重导入的来源）
   + `MDK-ARM/.eide/eide.yml` 的 `virtualFolder`（EIDE 树侧）；新头文件目录还要加到 `<IncludePath>` 和 `incList`。
   EIDE 进程内会缓存工程配置，改完必须让 EIDE 重载，否则下次操作会把磁盘改动写回旧版。
4. 每个改动写进 `modify.md`（精简，一条一段）。
