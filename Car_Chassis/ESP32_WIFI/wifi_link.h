#pragma once

#include <Arduino.h>
#include <IPAddress.h>

/* =====================================================================
 *  WiFi 链路（默认 AP 模式，UDP 服务器）
 *  上位机 → ESP32：UDP 数据报，负载 8 字节 = float v(小端) + float w(小端)
 *  ESP32 → 上位机：把 STM32 的 TLM 帧负载原样回发给最近一次通信的地址
 * ===================================================================== */

void WifiLink_Init(void);

/* 主循环里调用：处理 UDP 收包 */
void WifiLink_Poll(void);

/* 存一条新指令（WifiLink_Poll 内部调用；也可供其它指令源复用） */
void WifiLink_StoreCommand(float v, float w);

/* 有新的有效指令时返回 true，并填 v(m/s) / w(rad/s) */
bool WifiLink_ReadCommand(float &v, float &w);

/* 回传反馈（作为 ChassisLink 的 TelemetrySink 使用） */
void WifiLink_SendTelemetry(uint8_t type, const uint8_t *payload, uint8_t len);

bool WifiLink_IsReady(void);
int  WifiLink_ClientCount(void);
IPAddress WifiLink_Ip(void);

uint32_t WifiLink_CmdCount(void);
