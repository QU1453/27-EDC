#include "chassis_link.h"
#include "chassis_frame.h"
#include "wifi_config.h"

static float    s_v = 0.0f;                 /* 目标线速度 m/s */
static float    s_w = 0.0f;                 /* 目标角速度 rad/s */
static uint32_t s_last_cmd_ms  = 0;         /* 最近一次收到指令的时刻 */
static uint32_t s_last_tx_ms   = 0;
static bool     s_active       = false;
static uint32_t s_tx_cnt       = 0;
static uint32_t s_tlm_cnt      = 0;

static ChassisTelemetryCb s_telemetry_cb = nullptr;

void ChassisLink_Init(void)
{
    Serial1.begin(CHASSIS_UART_BAUD, SERIAL_8N1, CHASSIS_PIN_RX, CHASSIS_PIN_TX);

    s_v = 0.0f;
    s_w = 0.0f;
    s_last_cmd_ms = millis();
    s_last_tx_ms  = millis();
    s_active      = false;
}

void ChassisLink_SetTarget(float v, float w)
{
    s_v = v;
    s_w = w;
    s_last_cmd_ms = millis();
    s_active = true;
}

bool ChassisLink_IsActive(void)
{
    return s_active;
}

void ChassisLink_SetTelemetrySink(ChassisTelemetryCb cb)
{
    s_telemetry_cb = cb;
}

uint32_t ChassisLink_TxCount(void)
{
    return s_tx_cnt;
}

uint32_t ChassisLink_TelemetryCount(void)
{
    return s_tlm_cnt;
}

/* 收 STM32 回传帧 */
static void chassis_link_rx(void)
{
    uint8_t type = 0;
    uint8_t payload[FRAME_MAX_PAYLOAD];
    uint8_t len = 0;

    while (Serial1.available() > 0) {
        uint8_t b = (uint8_t)Serial1.read();
        if (!Frame_Parse(b, &type, payload, &len)) {
            continue;
        }
        s_tlm_cnt++;
        if (s_telemetry_cb != nullptr) {
            s_telemetry_cb(type, payload, len);
        }
    }
}

void ChassisLink_Task(void)
{
    uint32_t now = millis();

    /* 1. 超时保护：上位机掉线/卡死时不让车继续跑 */
    if (s_active && (now - s_last_cmd_ms > CHASSIS_LINK_TIMEOUT_MS)) {
        s_v = 0.0f;
        s_w = 0.0f;
        s_active = false;
    }

    /* 2. 固定周期下发（即使速度为 0 也持续发，STM32 侧可做掉线判断） */
    if (now - s_last_tx_ms >= CHASSIS_TX_PERIOD_MS) {
        s_last_tx_ms = now;

        uint8_t frame[FRAME_OVERHEAD + 8];
        uint8_t n = Frame_BuildCmd(s_v, s_w, frame);
        if (n > 0) {
            Serial1.write(frame, n);
            s_tx_cnt++;
        }
    }

    /* 3. 收反馈 */
    chassis_link_rx();
}
