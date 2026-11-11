#ifndef __LCD_FONT_H
#define __LCD_FONT_H

#include <stdint.h>

/* =====================================================================
 * lcd_font.h —— 点阵字库声明（Hardware 层）
 *
 * 文件作用：
 *   只声明 lcd_font.c 里那两张点阵表，以及中英文的字符宽高常量。
 *   绘图代码靠这三个 extern 查表取模，不做任何计算。
 *
 * 命名对照：
 *   font = 字库/字体          ascii = ASCII 表（0x20~0x7E 共 95 个）
 *   hz = HanZi 汉字           code = Unicode 码点
 *   count = 汉字个数          W/H = Width/Height 宽/高（像素）
 *   ascii_1608 = 8x16 ASCII 点阵    hz_16x16 = 16x16 汉字点阵
 * ===================================================================== */

/* 8x16 ASCII（0x20~0x7E，共 95 个字符） */
#define LCD_ASCII_W 8
#define LCD_ASCII_H 16

/* 16x16 汉字（取模方式：逐行、每行 2 字节、高位在左） */
#define LCD_HZ_W    16
#define LCD_HZ_H    16

/* 作用：8x16 ASCII 点阵。每字符 16 字节 = 16 行，1 字节 1 行，bit0 在最左。 */
extern const unsigned char ascii_1608[95][16];

/* 作用：汉字 Unicode 码点数组，与下面点阵数组下标一一对应，顺序必须一致。 */
extern const uint16_t hz_code[];

/* 作用：16x16 汉字点阵，每字 32 字节 = 16 行 × 2 字节，高位在左。 */
extern const unsigned char hz_16x16[][32];

/* 作用：字库里现有的汉字个数，查表循环的上界。 */
extern const uint16_t hz_count;

#endif
