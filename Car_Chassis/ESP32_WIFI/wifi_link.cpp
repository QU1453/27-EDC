#include "wifi_link.h"
#include "wifi_config.h"

#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP  s_udp;
static IPAddress s_remote_ip;
static uint16_t  s_remote_port = 0;
static bool      s_ready       = false;
static uint32_t  s_cmd_cnt     = 0;

void WifiLink_Init(void)
{
#if WIFI_USE_STA
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    Serial.printf("[WiFi] STA connecting to %s ...\n", WIFI_STA_SSID);
#else
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    Serial.printf("[WiFi] AP \"%s\" ip=%s\n", WIFI_AP_SSID,
                  WiFi.softAPIP().toString().c_str());
#endif

    s_udp.begin(UDP_PORT);
}

void WifiLink_Poll(void)
{
#if WIFI_USE_STA
    s_ready = (WiFi.status() == WL_CONNECTED);
#else
    s_ready = true;
#endif

    int size = s_udp.parsePacket();
    while (size > 0) {
        uint8_t buf[64];
        int n = s_udp.read(buf, sizeof(buf));
        if (n >= 8) {
            /* 记录来源，反馈帧往这里回 */
            s_remote_ip   = s_udp.remoteIP();
            s_remote_port = s_udp.remotePort();

            float v = 0.0f;
            float w = 0.0f;
            memcpy(&v, &buf[0], 4);
            memcpy(&w, &buf[4], 4);

            /* 先缓存，由 .ino 主循环取走并下发给 ChassisLink */
            WifiLink_StoreCommand(v, w);
            s_cmd_cnt++;
        }
        size = s_udp.parsePacket();
    }
}

/* 指令缓存：UDP 收到的 v/w 先放这里，主循环再取走，两边解耦 */
static float    s_pending_v = 0.0f;
static float    s_pending_w = 0.0f;
static bool     s_pending   = false;

void WifiLink_StoreCommand(float v, float w)
{
    s_pending_v = v;
    s_pending_w = w;
    s_pending   = true;
}

bool WifiLink_ReadCommand(float &v, float &w)
{
    if (!s_pending) {
        return false;
    }
    v = s_pending_v;
    w = s_pending_w;
    s_pending = false;
    return true;
}

void WifiLink_SendTelemetry(uint8_t type, const uint8_t *payload, uint8_t len)
{
    if (s_remote_port == 0 || len == 0) {
        return;
    }

    uint8_t frame[1 + 1 + 64];
    if (len > 64) {
        len = 64;
    }
    frame[0] = type;
    frame[1] = len;
    memcpy(&frame[2], payload, len);

    s_udp.beginPacket(s_remote_ip, s_remote_port);
    s_udp.write(frame, (size_t)(2 + len));
    s_udp.endPacket();
}

bool WifiLink_IsReady(void)
{
    return s_ready;
}

int WifiLink_ClientCount(void)
{
    return (int)WiFi.softAPgetStationNum();
}

IPAddress WifiLink_Ip(void)
{
#if WIFI_USE_STA
    return WiFi.localIP();
#else
    return WiFi.softAPIP();
#endif
}

uint32_t WifiLink_CmdCount(void)
{
    return s_cmd_cnt;
}
