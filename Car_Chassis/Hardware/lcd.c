#include <stddef.h>
#include "lcd.h"
#include "lcd_port.h"
#include "lcd_font.h"

/* =====================================================================
 * lcd.c —— ST7789 屏幕驱动（Hardware 层，绘图的真正实现）
 *
 * 文件作用：
 *   实现 lcd.h 里声明的所有绘图 API：初始化、画点/线/矩形、显示字符和汉字。
 *   只调用 lcd_port 的引脚/SPI 函数，不直接碰 GPIO 和 SPI 寄存器。
 *
 * 命名对照：
 *   MADCTL = Memory Access Control 扫描方向寄存器
 *   cmd = command 命令      dat = data 数据      len = length 长度
 *   set_window = 设置显示窗口（0x2A 列 / 0x2B 行 / 0x2C 写显存）
 *   config = 配置（初始化序列）
 *   s_line_buf = 整行缓冲   w/h = width/height 宽/高
 *   dx/dy = 差值   sx/sy = 步进方向   err/e2 = Bresenham 误差项
 *   ch = character 字符     str = 字符串     cp = Unicode 码点
 *   row/col = 行/列         mask = 位掩码     data = 当前字节的点阵
 *   fc/bc = 前景色/背景色   p = 点阵指针
 * ===================================================================== */

/* 2.00 寸 ST7789 的 MADCTL：0xA0 = 横向（320x240），颜色正常 */
#define LCD_MADCTL 0xA0

/* 整行缓冲，用于快速填充（1 行 = 320 像素 = 640 字节） */
static uint16_t s_line_buf[LCD_W];

/* 作用：写一条命令 + 可选的若干数据字节（dat 为 NULL 时只发命令）。 */
static void lcd_cmd(uint8_t cmd, const uint8_t *dat, uint16_t len)
{
    LCD_WriteCmd(cmd);
    if ((dat != NULL) && (len > 0U)) {
        LCD_WriteDataBulk(dat, len);
    }
}

/* 作用：设置显示窗口（含写显存命令 0x2C），此后连续写入都落在这个矩形内。 */
static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_WriteCmd(0x2A);
    LCD_WriteDataHalfWord(x1);
    LCD_WriteDataHalfWord(x2);
    LCD_WriteCmd(0x2B);
    LCD_WriteDataHalfWord(y1);
    LCD_WriteDataHalfWord(y2);
    LCD_WriteCmd(0x2C);
}

/* 作用：下发 ST7789 初始化序列（2.00 寸，参数取自厂家例程）。 */
static void lcd_config(void)
{
    static const uint8_t porch[]   = { 0x0C, 0x0C, 0x00, 0x33, 0x33 };
    static const uint8_t pw_ctrl[] = { 0xA4, 0xA1 };
    static const uint8_t gamma_p[] = { 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x05, 0x31,
                                       0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34 };
    static const uint8_t gamma_n[] = { 0xD0, 0x08, 0x0E, 0x09, 0x09, 0x15, 0x31,
                                       0x33, 0x48, 0x17, 0x14, 0x15, 0x31, 0x34 };
    uint8_t tmp;

    lcd_cmd(0x11, NULL, 0);                       /* Sleep Out */
    LCD_Delay(120);

    tmp = LCD_MADCTL;                             /* 扫描方向 */
    lcd_cmd(0x36, &tmp, 1);

    tmp = 0x05;                                   /* 16bit/pixel */
    lcd_cmd(0x3A, &tmp, 1);

    lcd_cmd(0xB2, porch, (uint16_t)sizeof(porch)); /* Porch Setting */

    tmp = 0x35;                                   /* Gate Control */
    lcd_cmd(0xB7, &tmp, 1);

    tmp = 0x32;                                   /* VCOM Setting */
    lcd_cmd(0xBB, &tmp, 1);

    tmp = 0x01;                                   /* VDV/VRH Command Enable */
    lcd_cmd(0xC2, &tmp, 1);

    tmp = 0x15;                                   /* VRH Set */
    lcd_cmd(0xC3, &tmp, 1);

    tmp = 0x20;                                   /* VDV Set */
    lcd_cmd(0xC4, &tmp, 1);

    tmp = 0x0F;                                   /* Frame Rate Control */
    lcd_cmd(0xC6, &tmp, 1);

    lcd_cmd(0xD0, pw_ctrl, (uint16_t)sizeof(pw_ctrl));      /* Power Control 1 */

    lcd_cmd(0xE0, gamma_p, (uint16_t)sizeof(gamma_p));      /* 正极性 Gamma */

    lcd_cmd(0xE1, gamma_n, (uint16_t)sizeof(gamma_n));      /* 负极性 Gamma */

    lcd_cmd(0x21, NULL, 0);                       /* Display Inversion On */
    lcd_cmd(0x29, NULL, 0);                       /* Display On */
}

/* 作用：屏幕初始化。引脚就绪 → 硬件复位 → 开背光 → 下发初始化序列 → 清黑。 */
void LCD_Init(void)
{
    LCD_PortInit();

    /* 硬件复位：低 100ms -> 高 100ms */
    LCD_RstPin(0);
    LCD_Delay(100);
    LCD_RstPin(1);
    LCD_Delay(100);

    /* 开背光 */
    LCD_BlPin(1);
    LCD_Delay(100);

    lcd_config();

    LCD_Clear(BLACK);
}

/* 作用：整屏填充一种颜色。 */
void LCD_Clear(uint16_t color)
{
    LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, color);
}

/* 作用：画一个像素点。越界直接丢弃，不做裁剪换算。 */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    if ((x >= LCD_W) || (y >= LCD_H)) {
        return;
    }
    lcd_set_window(x, y, x, y);
    LCD_WriteDataHalfWord(color);
}

/* 作用：填实心矩形。先把颜色按行填进整行缓冲，再一行一次批量发，速度远快于逐点。 */
void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    uint16_t w, h, i, j;

    if ((x1 > x2) || (y1 > y2)) {
        return;
    }
    if (x2 >= LCD_W) {
        x2 = LCD_W - 1;
    }
    if (y2 >= LCD_H) {
        y2 = LCD_H - 1;
    }

    w = (uint16_t)(x2 - x1 + 1U);
    h = (uint16_t)(y2 - y1 + 1U);

    /* 批量发送时字节顺序必须高字节在前，故先把颜色字节颠倒 */
    color = (uint16_t)((color << 8) | (color >> 8));
    for (j = 0; j < w; j++) {
        s_line_buf[j] = color;
    }

    lcd_set_window(x1, y1, x2, y2);
    for (i = 0; i < h; i++) {
        LCD_WriteDataBulk((const uint8_t *)s_line_buf, (uint32_t)w * 2U);
    }
}

/* 作用：画任意方向直线，Bresenham 整数算法（只用加减和比较，无浮点无乘法）。 */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    int dx, dy, sx, sy, err, e2;
    int x = (int)x1;
    int y = (int)y1;

    dx = (x2 > x1) ? (int)(x2 - x1) : -(int)(x1 - x2);
    dy = (y2 > y1) ? (int)(y2 - y1) : -(int)(y1 - y2);
    sx = (x1 < x2) ? 1 : -1;
    sy = (y1 < y2) ? 1 : -1;
    err = dx - dy;

    for (;;) {
        LCD_DrawPoint((uint16_t)x, (uint16_t)y, color);
        if ((x == (int)x2) && (y == (int)y2)) {
            break;
        }
        e2 = err * 2;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}

/* 作用：画空心矩形，四条边各调一次画线。 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
    LCD_DrawLine(x1, y1, x2, y1, color);
    LCD_DrawLine(x1, y1, x1, y2, color);
    LCD_DrawLine(x1, y2, x2, y2, color);
    LCD_DrawLine(x2, y1, x2, y2, color);
}

/* 作用：显示一个 8x16 ASCII 字符。不可打印字符当空格；越界直接跳过。 */
void LCD_ShowChar(uint16_t x, uint16_t y, char ch, uint16_t fc, uint16_t bc)
{
    const unsigned char *p;
    uint8_t data;
    uint16_t row, col;
    uint8_t c = (uint8_t)ch;

    if ((c < 0x20U) || (c > 0x7EU)) {
        c = (uint8_t)' ';
    }
    if (((uint32_t)x + LCD_ASCII_W > LCD_W) || ((uint32_t)y + LCD_ASCII_H > LCD_H)) {
        return;
    }

    p = ascii_1608[c - 0x20U];
    lcd_set_window(x, y, (uint16_t)(x + LCD_ASCII_W - 1U), (uint16_t)(y + LCD_ASCII_H - 1U));

    for (row = 0; row < LCD_ASCII_H; row++) {
        data = p[row];
        for (col = 0; col < LCD_ASCII_W; col++) {
            /* ASCII 表 bit0 在最左 */
            LCD_WriteDataHalfWord((data & 0x01U) ? fc : bc);
            data = (uint8_t)(data >> 1);
        }
    }
}

/* 作用：显示一个 16x16 汉字。按 Unicode 码点查表，查不到就跳过。 */
void LCD_ShowChinese(uint16_t x, uint16_t y, uint16_t code, uint16_t fc, uint16_t bc)
{
    const unsigned char *p = NULL;
    uint16_t i, row, col;
    uint8_t data, mask;

    for (i = 0; i < hz_count; i++) {
        if (hz_code[i] == code) {
            p = hz_16x16[i];
            break;
        }
    }
    if (p == NULL) {
        return;                                   /* 字库里没有该汉字，直接跳过 */
    }
    if (((uint32_t)x + LCD_HZ_W > LCD_W) || ((uint32_t)y + LCD_HZ_H > LCD_H)) {
        return;
    }

    lcd_set_window(x, y, (uint16_t)(x + LCD_HZ_W - 1U), (uint16_t)(y + LCD_HZ_H - 1U));

    for (row = 0; row < LCD_HZ_H; row++) {
        for (col = 0; col < LCD_HZ_W; col++) {
            /* 逐行 2 字节，高位在左 */
            data = p[row * 2U + (col >> 3)];
            mask = (uint8_t)(0x80U >> (col & 0x07U));
            LCD_WriteDataHalfWord((data & mask) ? fc : bc);
        }
    }
}

/* 作用：从 UTF-8 串里取出下一个字符的码点，并把指针前移相应字节数。
 *       只支持 1~3 字节序列（BMP 内的汉字够用），非法首字节返回 '?'。 */
static uint16_t lcd_utf8_next(const uint8_t **pp)
{
    const uint8_t *p = *pp;
    uint16_t cp;

    if (p[0] < 0x80U) {
        cp = p[0];
        *pp = p + 1;
    } else if ((p[0] & 0xE0U) == 0xC0U) {
        cp = (uint16_t)(((uint16_t)(p[0] & 0x1FU) << 6) | (p[1] & 0x3FU));
        *pp = p + 2;
    } else if ((p[0] & 0xF0U) == 0xE0U) {
        cp = (uint16_t)(((uint16_t)(p[0] & 0x0FU) << 12) |
                        ((uint16_t)(p[1] & 0x3FU) << 6) | (p[2] & 0x3FU));
        *pp = p + 3;
    } else {
        cp = (uint16_t)'?';
        *pp = p + 1;
    }
    return cp;
}

/* 作用：显示 UTF-8 字符串，ASCII 走 8x16 字库、汉字走 16x16 字库；超右边界自动换行。 */
void LCD_ShowString(uint16_t x, uint16_t y, const char *str, uint16_t fc, uint16_t bc)
{
    const uint8_t *p = (const uint8_t *)str;
    uint16_t cp;

    if (str == NULL) {
        return;
    }

    while (*p != 0U) {
        cp = lcd_utf8_next(&p);
        if (cp < 0x80U) {
            if (((uint32_t)x + LCD_ASCII_W > LCD_W) || (y >= LCD_H)) {
                x = 0;
                y = (uint16_t)(y + LCD_ASCII_H);
            }
            LCD_ShowChar(x, y, (char)cp, fc, bc);
            x = (uint16_t)(x + LCD_ASCII_W);
        } else {
            if (((uint32_t)x + LCD_HZ_W > LCD_W) || (y >= LCD_H)) {
                x = 0;
                y = (uint16_t)(y + LCD_HZ_H);
            }
            LCD_ShowChinese(x, y, cp, fc, bc);
            x = (uint16_t)(x + LCD_HZ_W);
        }
    }
}
