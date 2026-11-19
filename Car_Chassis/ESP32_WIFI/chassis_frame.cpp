#include "chassis_frame.h"

/* 解析状态机 */
enum { ST_HEAD0 = 0, ST_HEAD1, ST_TYPE, ST_LEN, ST_PAYLOAD, ST_SUM };

static uint8_t s_state = ST_HEAD0;
static uint8_t s_type  = 0;
static uint8_t s_len   = 0;
static uint8_t s_idx   = 0;
static uint8_t s_sum   = 0;
static uint8_t s_buf[FRAME_MAX_PAYLOAD];

uint8_t Frame_Build(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out)
{
    if (len > FRAME_MAX_PAYLOAD) {
        return 0;
    }

    out[0] = FRAME_HEAD0;
    out[1] = FRAME_HEAD1;
    out[2] = type;
    out[3] = len;

    uint8_t sum = (uint8_t)(type + len);
    for (uint8_t i = 0; i < len; i++) {
        out[4 + i] = payload[i];
        sum = (uint8_t)(sum + payload[i]);
    }
    out[4 + len] = sum;

    return (uint8_t)(FRAME_OVERHEAD + len);
}

uint8_t Frame_BuildCmd(float v, float w, uint8_t *out)
{
    uint8_t payload[8];
    /* ESP32 与 STM32 都是小端，直接按字节拷贝即可 */
    memcpy(&payload[0], &v, 4);
    memcpy(&payload[4], &w, 4);
    return Frame_Build(FRAME_TYPE_CMD, payload, 8, out);
}

bool Frame_Parse(uint8_t byte, uint8_t *type, uint8_t *payload, uint8_t *len)
{
    switch (s_state) {
    case ST_HEAD0:
        if (byte == FRAME_HEAD0) {
            s_state = ST_HEAD1;
        }
        break;

    case ST_HEAD1:
        /* 连发两个 0xAA 时不要丢帧头 */
        if (byte == FRAME_HEAD1) {
            s_state = ST_TYPE;
        } else {
            s_state = (byte == FRAME_HEAD0) ? ST_HEAD1 : ST_HEAD0;
        }
        break;

    case ST_TYPE:
        s_type  = byte;
        s_sum   = byte;
        s_state = ST_LEN;
        break;

    case ST_LEN:
        if (byte > FRAME_MAX_PAYLOAD) {     /* 长度非法，重同步 */
            s_state = ST_HEAD0;
            break;
        }
        s_len   = byte;
        s_idx   = 0;
        s_sum   = (uint8_t)(s_sum + byte);
        s_state = (s_len == 0) ? ST_SUM : ST_PAYLOAD;
        break;

    case ST_PAYLOAD:
        s_buf[s_idx++] = byte;
        s_sum = (uint8_t)(s_sum + byte);
        if (s_idx >= s_len) {
            s_state = ST_SUM;
        }
        break;

    case ST_SUM:
        s_state = ST_HEAD0;
        if (byte != s_sum) {                /* 校验错，丢弃本帧 */
            break;
        }
        *type = s_type;
        *len  = s_len;
        memcpy(payload, s_buf, s_len);
        return true;

    default:
        s_state = ST_HEAD0;
        break;
    }

    return false;
}
