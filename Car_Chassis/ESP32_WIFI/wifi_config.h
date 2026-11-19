#pragma once

/* =====================================================================
 *  ESP32-S3（Arduino 工具链）WiFi ↔ 底盘串口链路 —— 全部可调参数
 *
 *  接线（本板 PCB）：
 *      ESP32 U0RX(GPIO44)  ←  STM32 PA2 / USART2_TX
 *      ESP32 U0TX(GPIO43)  →  STM32 PA3 / USART2_RX
 *      GND 必须共地
 *  注意：UART1 走 GPIO Matrix，可映射到 43/44；若模块板载 CH343 也挂在
 *        这两根脚上，会与 STM32 抢线，需要断开板载串口芯片或改脚。
 * ===================================================================== */

/* ---------- 与 STM32 的串口（UART1） ---------- */
#define CHASSIS_UART_BAUD       115200
#define CHASSIS_PIN_RX          44      /* ESP 接收，接 STM32 的 USART2_TX */
#define CHASSIS_PIN_TX          43      /* ESP 发送，接 STM32 的 USART2_RX */

/* ---------- 链路时序 ---------- */
#define CHASSIS_TX_PERIOD_MS    20      /* 20 ms 发一帧（50 Hz） */
#define CHASSIS_LINK_TIMEOUT_MS 500     /* 超过 500 ms 没收到新指令 → 速度归零 */

/* ---------- WiFi ---------- */
/* 1 = STA 连路由器；0 = AP 自己开热点（赛场无路由时用这个） */
#define WIFI_USE_STA            0
#define WIFI_AP_SSID            "Chassis_AP"
#define WIFI_AP_PASS            "88888888"   /* 至少 8 位 */
#define WIFI_STA_SSID           "your_router"
#define WIFI_STA_PASS           "your_password"

#define UDP_PORT                8888

/* ---------- 调试打印 ---------- */
#define DEBUG_PRINT_MS          1000    /* USB 串口状态打印周期 */
