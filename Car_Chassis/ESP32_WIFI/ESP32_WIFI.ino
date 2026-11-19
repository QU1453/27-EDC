/* =====================================================================
 * ESP32_WIFI.ino —— WiFi ↔ 底盘串口桥（Arduino 工具链，独立于 STM32 工程）
 *
 * 数据流：
 *   上位机 --UDP--> ESP32 --UART1 帧--> STM32 USART2 --CAN--> 电机
 *   电机 --CAN--> STM32 --UART1 帧--> ESP32 --UDP--> 上位机
 *
 * 三层职责：
 *   wifi_link.cpp    UDP 收发包（不知道协议细节）
 *   chassis_link.cpp 20 ms 周期发 CMD 帧、500 ms 超时归零、解析 TLM 帧
 *   chassis_frame.cpp 帧的组包/拆包（AA 55 头 + 校验和）
 *
 * 本文件只做串联：收 WiFi 指令 → 写底盘目标 → 跑底盘链路 → 打印状态。
 *
 * 烧录方式：Arduino IDE 选 "ESP32S3 Dev Module"，波特率 115200，
 *           USB CDC On Boot 关闭（Serial 走 UART0，给日志用）。
 * ===================================================================== */

#include "wifi_config.h"
#include "chassis_frame.h"
#include "chassis_link.h"
#include "wifi_link.h"

static uint32_t s_last_print_ms = 0;   /* 上次打印状态的时刻 */
static float    s_last_v = 0.0f;       /* 最近一次下发的目标，仅用于打印 */
static float    s_last_w = 0.0f;

/**
 * 作用：上电初始化。顺序不能颠倒——链路要先打开串口，WiFi 后开热点。
 */
void setup()
{
    /* 0. USB 调试串口（UART0，115200），板上 USB 转串口看日志 */
    Serial.begin(115200);
    delay(200);                        /* 等 USB 枚举，否则前几行日志会丢 */
    Serial.println();
    Serial.println("=== ESP32 WiFi-Chassis Bridge ===");

    /* 1. 先把 UART1 拉起来，保证 STM32 一旦上电就能收到 50 Hz 的 0 速度帧 */
    ChassisLink_Init();

    /* 2. 再开 WiFi/UDP */
    WifiLink_Init();

    /* 3. 把 STM32 回传的 TLM 帧接到 WiFi 发送口：链路层不直接依赖 wifi_link */
    ChassisLink_SetTelemetrySink(WifiLink_SendTelemetry);

    s_last_print_ms = millis();
    Serial.println("[INIT] done");
}

/**
 * 作用：主循环，单线程顺序执行，任何一步都不允许阻塞。
 *   1) Poll WiFi：把收到的 UDP 包解析成 v/w 并缓存
 *   2) 取指令：有新的就更新底盘目标（同时刷新 500 ms 超时计时）
 *   3) Task 底盘链路：发 CMD 帧 + 收 TLM 帧 + 超时归零
 *   4) 打印状态：DEBUG_PRINT_MS 一次，看 IP/客户端数/收发计数
 */
void loop()
{
    /* 1. WiFi 收包：内部只缓存，不直接操作底盘 */
    WifiLink_Poll();

    /* 2. 有新指令才更新目标；没收到就不动，让 ChassisLink 自己超时归零 */
    float v = 0.0f;
    float w = 0.0f;
    if (WifiLink_ReadCommand(v, w)) {
        ChassisLink_SetTarget(v, w);
        s_last_v = v;
        s_last_w = w;
    }

    /* 3. 底盘链路：固定 20 ms 发一帧，同时把 STM32 的反馈转发出去 */
    ChassisLink_Task();

    /* 4. 状态打印（限流，不占串口带宽） */
    uint32_t now = millis();
    if (now - s_last_print_ms >= DEBUG_PRINT_MS) {
        s_last_print_ms = now;
        Serial.printf("[%6lu ms] IP=%s cli=%d act=%d tx=%lu tlm=%lu cmd=%lu tgt=%.2f/%.2f\n",
                      (unsigned long)now,
                      WifiLink_Ip().toString().c_str(),
                      WifiLink_ClientCount(),
                      (int)ChassisLink_IsActive(),
                      (unsigned long)ChassisLink_TxCount(),
                      (unsigned long)ChassisLink_TelemetryCount(),
                      (unsigned long)WifiLink_CmdCount(),
                      s_last_v, s_last_w);
    }
}
