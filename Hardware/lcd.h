#ifndef __LCD_H
#define __LCD_H

#include <stdint.h>

/* =====================================================================
 * lcd.h —— 屏幕绘图层（Hardware 层，上层只需要这一个头文件）
 *
 * 文件作用：
 *   声明 ST7789 的绘图 API（点/线/矩形/字符/汉字/字符串）和 RGB565 颜色宏。
 *   只管「画什么」，不管引脚怎么接、SPI 怎么发——那是 lcd_port 的事。
 *
 * 命名对照：
 *   LCD = Liquid Crystal Display 液晶屏    W/H = Width/Height 宽/高（像素）
 *   Init = 初始化                          Clear = 整屏清成单色
 *   DrawPoint = 画一个点                   Fill = 填一个矩形
 *   DrawLine = 画直线（Bresenham）         DrawRectangle = 画空心矩形
 *   ShowChar = 显示一个 ASCII 字符         ShowChinese = 显示一个汉字
 *   ShowString = 显示字符串（UTF-8 中英混排）
 *   fc = foreground color 前景色           bc = background color 背景色
 *   x,y = 左上角坐标（像素）               color = RGB565 颜色值
 * ===================================================================== */

/* 分辨率：2.00 寸 ST7789，横向 320x240（旋转 270°） */
#define LCD_W 320
#define LCD_H 240

/* 画笔颜色（RGB565：高 5 位红、中 6 位绿、低 5 位蓝） */
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define GRAY        0x8430
#define GBLUE       0x07FF
#define BROWN       0xBC40
#define BRRED       0xFC07
#define DARKBLUE    0x01CF
#define LIGHTBLUE   0x7D7C
#define GRAYBLUE    0x5458
#define LIGHTGREEN  0x841F
#define LGRAY       0xC618
#define LGRAYBLUE   0xA651
#define LBBLUE      0x2B12

/* 作用：屏幕初始化。走完复位时序 → 开背光 → 下发 ST7789 初始化序列 → 清黑。 */
void LCD_Init(void);

/* 作用：整屏填充一种颜色。 */
void LCD_Clear(uint16_t color);

/* 作用：画一个像素点（越界自动丢弃）。 */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);

/* 作用：填实心矩形。内部整行缓冲批量发送，比逐点画快很多。 */
void LCD_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/* 作用：画直线，Bresenham 整数算法，无浮点、无乘法。 */
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/* 作用：画空心矩形（四条边各画一条线）。 */
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);

/* 作用：显示一个 8x16 ASCII 字符，非可打印字符当空格处理。 */
void LCD_ShowChar(uint16_t x, uint16_t y, char ch, uint16_t fc, uint16_t bc);

/* 作用：显示一个 16x16 汉字，code 为 Unicode 码点（如 0x6211 = “我”）；字库没有则跳过。 */
void LCD_ShowChinese(uint16_t x, uint16_t y, uint16_t code, uint16_t fc, uint16_t bc);

/* 作用：显示字符串，支持 UTF-8 中英文混排（源码按 UTF-8 保存），超出宽度自动换行。 */
void LCD_ShowString(uint16_t x, uint16_t y, const char *str, uint16_t fc, uint16_t bc);

#endif
