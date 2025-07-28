#ifndef _DEVICE_LCD_H_
#define _DEVICE_LCD_H_

#include "zf_common_headfile.h"

#define DEFAULT_PEN_COLOR RGB565_MAGENTA
#define DEFAULT_BACKGROUND_COLOR RGB565_WHITE
#define CHAR_WIDTH 8
#define CHAR_HEIGTH 16

void lcd_init(void);
void lcd_clear(void);

void lcd_show_string(uint16 x, uint16 y, const char *dat);
void lcd_show_string_color(uint16 x,
                           uint16 y,
                           const char *dat,
                           uint16 pen_color,
                           uint16 background_color);

void lcd_show_uint(uint16 x, uint16 y, uint32 dat, uint8 num);
void lcd_show_uint_color(uint16 x,
                         uint16 y,
                         uint32 dat,
                         uint8 num,
                         uint16 pen_color,
                         uint16 background_color);

void lcd_show_int(uint16 x, uint16 y, int32 dat, uint8 num);
void lcd_show_int_color(uint16 x,
                        uint16 y,
                        int32 dat,
                        uint8 num,
                        uint16 pen_color,
                        uint16 background_color);

void lcd_show_float(uint16 x,
                    uint16 y,
                    const double dat,
                    uint8 num,
                    uint8 pointnum);
void lcd_show_float_color(uint16 x,
                          uint16 y,
                          const double dat,
                          uint8 num,
                          uint8 pointnum,
                          uint16 pen_color,
                          uint16 background_color);

void lcd_show_char(uint16 x, uint16 y, char dat);
void lcd_show_char_color(uint16 x,
                         uint16 y,
                         char dat,
                         uint16 pen_color,
                         uint16 background_color);

void lcd_show_image(const uint8 *img,
                    uint16 width,
                    uint16 height,
                    uint8 threshold);
void lcd_show_image_mid(const uint8 *img,
                        uint16 width,
                        uint16 height,
                        uint8 threshold);

#endif