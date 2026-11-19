#pragma once

#include <Arduino.h>

/* =====================================================================
 *  帧格式（ESP↔STM32 双向通用，小端）
 *      AA 55 | TYPE(1) | LEN(1) | PAYLOAD(LEN) | SUM(1)
 *      SUM = (TYPE + LEN + 所有 PAYLOAD 字节) & 0xFF
 *  TYPE:
 *      0x01 CMD  底盘控制，PAYLOAD = float v(m/s) + float w(rad/s)，LEN=8
 *      0x81 TLM  底盘反馈，PAYLOAD 由 STM32 侧定义，ESP 原样转发给上位机
 * ===================================================================== */

#define FRAME_HEAD0             0xAA
#define FRAME_HEAD1             0x55
#define FRAME_TYPE_CMD          0x01
#define FRAME_TYPE_TLM          0x81
#define FRAME_MAX_PAYLOAD       32
#define FRAME_OVERHEAD          5       /* 2 头 + 类型 + 长度 + 校验 */

/* 组帧：返回整帧长度（头2+类型1+长度1+负载+校验1） */
uint8_t Frame_Build(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);

/* 组一帧底盘指令（v 线速度 m/s，w 角速度 rad/s） */
uint8_t Frame_BuildCmd(float v, float w, uint8_t *out);

/* 逐字节喂入解析；凑满一帧且校验通过时返回 true，并输出 type/payload/len */
bool Frame_Parse(uint8_t byte, uint8_t *type, uint8_t *payload, uint8_t *len);
