#ifndef _DEVICE_PIN_H_
#define _DEVICE_PIN_H_

// SPI 接口绝对式编码器引脚配置
#define ENCODER_SPI SPI_1
#define ENCODER_SCLK_PIN SPI1_SCK_B23
#define ENCODER_MOSI_PIN SPI1_MOSI_B22
#define ENCODER_MISO_PIN SPI1_MISO_B21
#define ENCODER_CNT 4
#define ENCODER_CS_PIN_LIST { B17, B18, B26, B27 }

// 增量式编码器AB相引脚配置
#define ENCODER_1_PIN A0, A1
#define ENCODER_2_PIN A2, A3
#define ENCODER_3_PIN A4, A5
#define ENCODER_4_PIN A6, A7

// ADIS pin
#define ADIS_SPI                         (SPI_0           )
#define ADIS_CS_PIN                      (B20             )
#define ADIS_RST_PIN                     (B10             )
#define ADIS_MOSI                        (SPI0_MOSI_B17   )
#define ADIS_MISO                        (SPI0_MISO_B19   )
#define ADIS_SCK                         (SPI0_SCK_B18    )
#define ADIS_SPEED                       (2 * 1000 * 1000 )

// tft180 pin
#define LCD_SPI                          (SPI_0           )
#define LCD_SCL_PIN                      (SPI0_SCK_A12    )
#define LCD_SDA_PIN                      (SPI0_MOSI_A9    )
#define LCD_RES_PIN                      (A7              ) // 复位引脚
#define LCD_CS_PIN                       (A8              ) // 片选引脚
#define LCD_DC_PIN                       (A15             ) // 数据/命令引脚
#define LCD_BL_PIN                       (A13             ) // 背光引脚

#endif
