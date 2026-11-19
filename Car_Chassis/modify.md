# 修改记录（唯一）

## 2026-11-19 · Car_Chassis · ESP32 WiFi 桥（独立 Arduino 工程）
- 新增 `ESP32_WIFI/`（不进 Keil/CubeMX 构建）：`ESP32_WIFI.ino`、`wifi_config.h`、`chassis_frame.cpp/h`、`chassis_link.cpp/h`、`wifi_link.cpp/h`。
- 帧协议 `AA 55 | TYPE | LEN | PAYLOAD | SUM`，SUM=(TYPE+LEN+Σpayload)&0xFF；`0x01` CMD=float v+float w（8B），`0x81` TLM 由 STM32 定义。
- UART1 → GPIO44(RX)/GPIO43(TX)，115200 8N1；20 ms 发一帧（50 Hz），500 ms 无指令归零。
- WiFi 默认 AP：`Chassis_AP` / `88888888`，UDP 8888；上位机 8 字节小端 v/w。
- 回滚：删除 `Car_Chassis/ESP32_WIFI/`

## 2026-11-19 · 根目录 · Resources 资料库说明
- 新增 `Resources/README.md`：说明该目录只放原始资料（开发板资料包、手册、原理图、示例），不参与编译；登记 `ESP32-S3 CAM开发板资料.zip`（135.8 MB）及其内容；约定新增资料补表。
- 回滚：删除 `Resources/README.md`

## 2026-11-13 · Car_Chassis · 姿态传感器 YbImu 串口驱动
- 新增 `BSP/bsp_imu.c/.h`：逐字节中断接收 + 512B 环形缓冲 + `7E 23 len func data sum` 帧解析；接口 `BSP_IMU_Init/Task/GetData/IsOnline/SetReportRate/SetAlgoType/RequestData`。
- 新增 `App/app_imu.c/.h`：每 250 ms 刷板载 ST7789（R/P/Y、GZ、在线状态、诊断行 `RX/F/P/B0/H`）；掉线时每 300 ms 交替请求 0x26 欧拉角 / 0x04 原始量。
- 串口固定为 **USART1：PE0=TX / PE1=RX，115200 8N1**（`APP_IMU_UART=&huart1`）。CubeMX 生成文件一律未改，`main.c` 只在 USER CODE 区调用 `App_IMU_Init/Task`。
- 其它文件：`MDK-ARM/car_dipan.uvprojx` 加入 BSP/App 源文件；`car_dipan.code-workspace` 增加 EIDE 指向本机 Keil AC5 的两行配置。
- 诊断行改为 `RX/F/P/B0/H`（B0=首个字节、H=0x7E 个数），去掉无用的 E 跳变计数与 U 悬空检测。
- 回滚：`git checkout BSP/bsp_imu.c BSP/bsp_imu.h App/app_imu.c App/app_imu.h Core/Src/main.c MDK-ARM/car_dipan.uvprojx`

## 2026-11-13 · 定位通信根因（接线未交叉）
- 根因：传感器 TX 与 MCU RX **未交叉**。正确接法：传感器 TX→PE1、传感器 RX→PE0、共地。此前"无数据"是接线问题，驱动协议层无缺陷。
- `App/app_imu.c`：删除 `app_imu_check_rx_pullup()`（运行时把 PE1 改成输入上拉读回，风险高）；诊断行去掉 `U` 字段。
- 编译 Code=22988 RO=2448 RW=68 ZI=4036，0 Error 0 Warning，已烧录。
- 回滚：`git checkout App/app_imu.c`

## 2026-11-14 · 接线交叉后重写驱动（清理排查代码）
- 接线：传感器 TX→PE1、RX→PE0、GND 共地、5V 外部供电（丝印确认模块为 YAHBOOM YB-MRA02-V1.0 / YbImu）。
- `App/app_imu.c`：重写为干净版，屏上只留 R/P/Y、GZ、`IMU OK/NO DATA`、`F/RX` 计数；删除 `raw/P/B0/H/U` 等排查字段与 `s_rx_level` 引脚采样。
- `BSP/bsp_imu.c/.h`：删除 `raw[8]`、`first_byte`、`head_count` 及中断里的 8 字节移位，缩短中断时间；协议解析/校验/请求帧逻辑未变。
- 编译 Code=22656 RO=2408 RW=68 ZI=4020，0 Error 0 Warning；hex 已在工程根目录 `car_dipan.hex`，已烧录并复位。
- 回滚：`git checkout App/app_imu.c App/app_imu.h BSP/bsp_imu.c BSP/bsp_imu.h`

## 2026-11-16 · 电机首次点动（FDCAN 1 Mbps + 开环试转）
- 根因：CubeMX 里 FDCAN1 位时序是 Prescaler=16/Seg1=1/Seg2=1，实算约 3.54 Mbps；DM-S2325 出厂 1 Mbps，故收发都不通。
- `BSP/bsp_can.c`：新增 `bsp_can_fix_timing()`，在 `BSP_CAN_Init()` 启动前按 10/(1+13+3)=1 Mbps 重初始化（PCLK1=170 MHz）；**未改 CubeMX 生成的 `fdcan.c`**。
- `App/app_chassis.c/.h`：新增 `App_Chassis_BenchSpin(rpm)` 开环试转，忽略反馈与掉线清零（Master ID 未配也能转）；屏幕 208 行显示 `M 左rpm 右rpm E左err右err`。
- `Core/Src/main.c`（仅 USER CODE 2）：`App_Chassis_BenchSpin(10.0f)` 替代 `SetVelocity(0.2f,0)`。
- 编译 Code=22960 RO=2448 RW=76 ZI=4020，0 Error 0 Warning；已烧录并复位。
- 回滚：`git checkout BSP/bsp_can.c App/app_chassis.c App/app_chassis.h Core/Src/main.c`

## 2026-11-17 · 达妙 DM3520-1EC 调试串口（USART3 + UART4）
- 手册结论：驱动器 3Pin 串口是**调参口，921600 8N1**，只能读参数/改 ID/校准/升级；**跑电机必须走 CAN 1 Mbps**。
- 新增 `BSP/bsp_dm_uart.c/.h`：在 BSP 层把 USART3/UART4 从 CubeMX 的 115200 重初始化为 921600 8N1（未改 `usart.c`），单字节中断收 → 256B 环形缓冲 + 80B 行缓存；接口 `Init/Available/Read/RxCount/GetLine/LineTick/Send/OnRx/OnError`。
- 引脚：USART3 PB10=TX / PB11=RX；UART4 PC10=TX / PC11=RX。
- `BSP/bsp_imu.c`：`HAL_UART_RxCpltCallback` / `HAL_UART_ErrorCallback` 开头先给 `BSP_DM_UART_OnRx/OnError` 认领，再走 IMU 分支（全局回调只能有一个）。
- `Core/Src/main.c`（仅 USER CODE）：`BSP_DM_UART_Init()`；`dm_uart_show()` 每 300 ms 把最新一行打到屏幕 224 行 `U3/U4 <文本>`。
- 编译 Code=24188 RO=2448 RW=960 ZI=4072，0 Error 0 Warning；本次未烧录（ST-Link 未检测到）。
- 回滚：`git checkout BSP/bsp_imu.c Core/Src/main.c MDK-ARM/car_dipan.uvprojx` 并删除 `BSP/bsp_dm_uart.c/.h`

## 2026-11-17 · CubeMX 重新 Generate Code（UART4/USART3 DMA + I2C1 + TIM1）
- 在 CubeMX 里点了 GENERATE CODE（ioc 里已有 USART3/UART4 DMA 请求、I2C1、TIM1）。提示「Code is successfully generated ... but MDK-ARM V5.32 project generation has a problem」，但 `car_dipan.uvprojx` 实际已重写且 XML 合法。
- 新增生成文件：`Core/Src/dma.c`、`i2c.c`、`tim.c` 及对应 `.h`；`usart.c` 现在带 USART3/UART4 的 DMA 句柄（DMA1 CH1~4）与 NVIC 使能。
- 影响：DMA 通道已初始化但**未启动传输**，`HAL_UART_Receive_IT` 单字节中断收不受影响，`bsp_dm_uart` 无需改动；波特率仍是 115200，运行时由 `BSP_DM_UART_Init()` 改成 921600。
- USER CODE 全部保留（`LCD_Init`/`App_Chassis_*`/`App_IMU_Init`/`BSP_DM_UART_Init`）；`uvprojx` 的自定义 Group 与 `bsp_dm_uart.c` 条目也保留。
- 同步登记：`MDK-ARM/.eide/eide.yml` 补 `bsp_dm_uart.c`、`dma.c`、`i2c.c`、`tim.c`；`struct.md` 更新目录树。
- 回滚：`git checkout Core MDK-ARM/car_dipan.uvprojx MDK-ARM/.eide/eide.yml car_dipan.ioc .mxproject`

## 2026-11-17 · 编译验证（未改动工程设置）
- 首次编译报 `L6047U: image 32840 bytes exceeds the maximum allowed for this version of the linker`（Keil MDK-Lite 32 KB 上限），因新增 i2c/tim/dma HAL 模块。
- 曾试开 MicroLIB（`useUlib=1`）降到 Code=27240，**用户要求不改原工程，已改回 `useUlib=0`**，工程设置保持原样。


## 2026-11-17 CubeMX 工程名统一为 Car_Chassis
- car_dipan.ioc -> Car_Chassis.ioc（ProjectFileName/ProjectName 同步）
- MDK-ARM/car_dipan.uvprojx -> Car_Chassis.uvprojx（TargetName/OutputDirectory/OutputName/ListingPath 同步）
- car_dipan.code-workspace -> Car_Chassis.code-workspace；输出目录 MDK-ARM/car_dipan/ -> MDK-ARM/Car_Chassis/
- 清理 uvprojx 中残留的 3 个 <File>：../Core/Src/dma.c、i2c.c、	im.c（文件已删，会报 cannot open source input file）
- .mxproject 全为相对路径无需改；.eide/eide.yml 内部 target 名保持不动
- 回滚：git checkout -- Car_Chassis 后删除新增的 Car_Chassis.* 文件
- 修复 Generate Code 报错 `FileNotFoundException: Car_Chassis.uvoptx`：从 git HEAD 取回 car_dipan.uvoptx / car_dipan.uvguix.47402，字节级替换 car_dipan->Car_Chassis 后生成同名文件（Keil 改名必须 uvprojx/uvoptx/uvguix 三件套一起改）
