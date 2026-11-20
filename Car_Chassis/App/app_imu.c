/* =====================================================================
 * app_imu.c —— 姿态传感器任务（App 层，板载 LCD 显示）
 *
 * 接线（I2C1）：
 *   PB7 = I2C1_SDA → 传感器 SDA
 *   PA15 = I2C1_SCL → 传感器 SCL（原 PB8，PB8 是 BOOT0，上拉会让芯片进 Bootloader）
 *   传感器 3V3    → 板 3V3
 *   传感器 GND    → 板 GND
 *   从机地址 0x23，4.7 kΩ 上拉（模块自带则不用外加）
 *
 * 显示（320x240，左上角）：roll / pitch / yaw（度）、z 轴角速度、在线状态、
 * 成功读取组数与 I2C 错误计数。刷新用 LCD_ShowStringFast 整行批量发送，
 * 约 5 ms / 次。
 * ===================================================================== */

#include "app_imu.h"
#include "bsp_imu.h"
#include "i2c.h"
#include "lcd.h"

#include <stdio.h>

/* ---------------- 可配置项 ---------------- */
#define APP_IMU_I2C             (&hi2c1)    /* PB7=SDA / PA15=SCL */
#define APP_IMU_PERIOD_MS       250U        /* 屏幕刷新周期，4 Hz */
#define APP_IMU_ONLINE_MS       500U        /* 超过 500 ms 没读到数据算掉线 */
#define APP_IMU_RAD2DEG         57.29578f

/* 屏幕布局（像素） */
#define APP_IMU_COL_X           32U         /* 数值列 */
#define APP_IMU_ROW_R           32U
#define APP_IMU_ROW_P           60U
#define APP_IMU_ROW_Y           88U
#define APP_IMU_ROW_G           116U
#define APP_IMU_ROW_STATE       160U
#define APP_IMU_ROW_CNT         184U
/* ------------------------------------------ */

static uint32_t s_last_tick;            /* 上次刷屏时间 */
static uint8_t  s_online;               /* 上次画的状态，0xFF = 还没画过 */

static void app_imu_draw_static(void)
{
    LCD_ShowString(0U, 0U, "IMU YbImu  I2C1 PB7/PB8", CYAN, BLACK);
    LCD_ShowString(0U, APP_IMU_ROW_R, "R:", GREEN, BLACK);
    LCD_ShowString(0U, APP_IMU_ROW_P, "P:", GREEN, BLACK);
    LCD_ShowString(0U, APP_IMU_ROW_Y, "Y:", GREEN, BLACK);
    LCD_ShowString(0U, APP_IMU_ROW_G, "GZ:", GREEN, BLACK);
}

/* 定宽输出，刷新不留残影 */
static void app_imu_draw_value(uint16_t y, float value)
{
    char text[16];

    (void)snprintf(text, sizeof(text), "%+7.2f", value);
    LCD_ShowStringFast(APP_IMU_COL_X, y, text, YELLOW, BLACK);
}

static void app_imu_refresh(void)
{
    const BSP_IMU_Data_t *d = BSP_IMU_GetData();
    uint8_t online;
    char text[40];

    if (d == NULL) {
        return;
    }

    online = BSP_IMU_IsOnline(APP_IMU_ONLINE_MS);

    if (online != 0U) {
        app_imu_draw_value(APP_IMU_ROW_R, d->euler[0] * APP_IMU_RAD2DEG);
        app_imu_draw_value(APP_IMU_ROW_P, d->euler[1] * APP_IMU_RAD2DEG);
        app_imu_draw_value(APP_IMU_ROW_Y, d->euler[2] * APP_IMU_RAD2DEG);

        (void)snprintf(text, sizeof(text), "%+7.3f", d->gyro[2]);
        LCD_ShowStringFast(APP_IMU_COL_X, APP_IMU_ROW_G, text, CYAN, BLACK);
    }

    /* 成功读取组数持续增长说明 I2C 链路正常；ERR 是失败次数 */
    (void)snprintf(text, sizeof(text), "F:%-6u ERR:%-6u",
                   (unsigned)d->frame_count, (unsigned)d->err_count);
    LCD_ShowStringFast(0U, APP_IMU_ROW_CNT, text, GRAY, BLACK);

    /* 状态行只在变化时写，省 SPI 带宽；两条字符串等长，不留残影 */
    if (online != s_online) {
        s_online = online;
        if (online != 0U) {
            LCD_ShowStringFast(0U, APP_IMU_ROW_STATE, "IMU OK      ", GREEN, BLACK);
        } else {
            LCD_ShowStringFast(0U, APP_IMU_ROW_STATE, "IMU NO DATA ", RED, BLACK);
        }
    }
}

void App_IMU_Init(void)
{
    s_online    = 0xFFU;
    s_last_tick = HAL_GetTick();

    app_imu_draw_static();
    (void)BSP_IMU_Init(APP_IMU_I2C);
    app_imu_refresh();
}

void App_IMU_Task(void)
{
    uint32_t now;

    BSP_IMU_Task();

    now = HAL_GetTick();

    if ((now - s_last_tick) >= APP_IMU_PERIOD_MS) {
        s_last_tick = now;
        app_imu_refresh();
    }
}
