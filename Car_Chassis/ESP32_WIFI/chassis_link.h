#pragma once

#include <Arduino.h>

/* =====================================================================
 *  底盘串口链路（UART1 → STM32G473 USART2）
 *  职责：
 *    1) 周期性把当前目标速度打包成 CMD 帧发出去（50 Hz）
 *    2) 超时（默认 500 ms）没收到新指令 → 自动把目标速度归零
 *    3) 解析 STM32 回传的 TLM 帧，交给上层回调（ESP 再转发给上位机）
 *  注意：本层不做任何限幅，安全限幅请在 STM32 侧做（它知道电机参数）。
 * ===================================================================== */

/* 反馈帧回调：type 通常为 FRAME_TYPE_TLM */
typedef void (*ChassisTelemetryCb)(uint8_t type, const uint8_t *payload, uint8_t len);

void ChassisLink_Init(void);

/* 收到一条新的上位机指令（v: m/s，w: rad/s） */
void ChassisLink_SetTarget(float v, float w);

/* 主循环里调用：负责发帧、收帧、超时归零 */
void ChassisLink_Task(void);

/* 链路是否在有效期内（最近 500 ms 收到过指令） */
bool ChassisLink_IsActive(void);

void ChassisLink_SetTelemetrySink(ChassisTelemetryCb cb);

uint32_t ChassisLink_TxCount(void);
uint32_t ChassisLink_TelemetryCount(void);
