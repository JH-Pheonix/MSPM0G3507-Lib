#include "lcd.h"

void lcd_init()
{
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_set_color(DEFAULT_PEN_COLOR, DEFAULT_BACKGROUND_COLOR);
    tft180_init();
}

void lcd_clear()
{
    tft180_full(DEFAULT_BACKGROUND_COLOR);
}

void lcd_show_string(uint16 x, uint16 y, const char *dat)
{
    tft180_show_string(x * CHAR_WIDTH, y * CHAR_HEIGTH, dat);
}

void lcd_show_string_color(uint16 x,
                           uint16 y,
                           const char *dat,
                           uint16 pen_color,
                           uint16 background_color)
{
    tft180_set_color(pen_color, background_color);
    lcd_show_string(x, y, dat);
    tft180_set_color(DEFAULT_PEN_COLOR, DEFAULT_BACKGROUND_COLOR);
}
void lcd_show_uint(uint16 x, uint16 y, uint32 dat, uint8 num)
{
    tft180_show_uint(x * CHAR_WIDTH, y * CHAR_HEIGTH, dat, num);
}

void lcd_show_uint_color(uint16 x,
                         uint16 y,
                         uint32 dat,
                         uint8 num,
                         uint16 pen_color,
                         uint16 background_color)
{
    tft180_set_color(pen_color, background_color);
    lcd_show_uint(x, y, dat, num);
    tft180_set_color(DEFAULT_PEN_COLOR, DEFAULT_BACKGROUND_COLOR);
}

void lcd_show_int(uint16 x, uint16 y, int32 dat, uint8 num)
{
    tft180_show_int(x * CHAR_WIDTH, y * CHAR_HEIGTH, dat, num);
}

void lcd_show_int_color(uint16 x,
                        uint16 y,
                        int32 dat,
                        uint8 num,
                        uint16 pen_color,
                        uint16 background_color)
{
    tft180_set_color(pen_color, background_color);
    lcd_show_int(x, y, dat, num);
    tft180_set_color(DEFAULT_PEN_COLOR, DEFAULT_BACKGROUND_COLOR);
}

void lcd_show_float(uint16 x,
                    uint16 y,
                    const double dat,
                    uint8 num,
                    uint8 pointnum)
{
    tft180_show_float(x * CHAR_WIDTH, y * CHAR_HEIGTH, dat, num, pointnum);
}

void lcd_show_float_color(uint16 x,
                          uint16 y,
                          const double dat,
                          uint8 num,
                          uint8 pointnum,
                          uint16 pen_color,
                          uint16 background_color)
{
    tft180_set_color(pen_color, background_color);
    lcd_show_float(x, y, dat, num, pointnum);
    tft180_set_color(DEFAULT_PEN_COLOR, DEFAULT_BACKGROUND_COLOR);
}

void lcd_show_char(uint16 x, uint16 y, char dat)
{
    tft180_show_char(x * CHAR_WIDTH, y * CHAR_HEIGTH, dat);
}

void lcd_show_char_color(uint16 x,
                         uint16 y,
                         char dat,
                         uint16 pen_color,
                         uint16 background_color)
{
    tft180_set_color(pen_color, background_color);
    lcd_show_char(x, y, dat);
    tft180_set_color(DEFAULT_PEN_COLOR, DEFAULT_BACKGROUND_COLOR);
}

// 居中展示图像
void lcd_show_image_mid(const uint8 *img,
                        uint16 width,
                        uint16 height,
                        uint8 threshold)
{
    float scale_w = (float)TFT180_WIDTH_MAX / width;
    float scale_h = (float)TFT180_HEIGHT_MAX / height;

    float scale = scale_w < scale_h ? scale_w : scale_h;

    uint16 display_w = (uint16)(width * scale);
    uint16 display_h = (uint16)(height * scale);

    uint16 start_x = (TFT180_WIDTH_MAX - display_w) / 2;
    uint16 start_y = (TFT180_HEIGHT_MAX - display_h) / 2;

    tft180_show_gray_image(start_x, start_y, img, width, height,
                           display_w, display_h, threshold);
}

void lcd_show_image(const uint8 *img,
                    uint16 width,
                    uint16 height,
                    uint8 threshold)
{
    float scale_w = (float)TFT180_WIDTH_MAX / width;
    float scale_h = (float)TFT180_HEIGHT_MAX / height;

    float scale = scale_w < scale_h ? scale_w : scale_h;

    uint16 display_w = (uint16)(width * scale);
    uint16 display_h = (uint16)(height * scale);

    uint16 start_x = (TFT180_WIDTH_MAX - display_w) / 2;

    tft180_show_gray_image(start_x, 0, img, width, height,
                           display_w, 112, threshold);
}