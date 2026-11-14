# 修改记录（唯一）

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
