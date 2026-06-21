/********************************** (C) COPYRIGHT *******************************
 * File Name          : tft180.h
 * Description        : TFT180 (ST7735) 1.8寸彩屏驱动 (CH32H417 适配版)
 *                      分辨率: 128x160 (竖屏) / 160x128 (横屏)
 *                      接口: 硬件 SPI
 *                      基于逐飞科技 CH32V307 开源库移植
 *
 * 引脚定义:
 *   SCK  = PB13 (SPI2_SCK, AF5)
 *   MOSI = PC1  (SPI2_MOSI, AF5)
 *   CS   = PD10 (软件片选)
 *   DC   = PD9  (数据/命令)
 *   RST  = PD8  (复位)
 *   BL   = PD11 (背光)
 *   VCC  = 3.3V
 *   GND  = GND
 ******************************************************************************
 */
#ifndef __TFT180_H
#define __TFT180_H

#include "ch32h417.h"

/* ==================== 引脚定义 ==================== */
/* SPI 引脚 (硬件 SPI2, AF5) */
#define TFT180_SPI              SPI2
#define TFT180_SPI_CLK          RCC_HB1Periph_SPI2
#define TFT180_SPI_GPIO_CLK     RCC_HB2Periph_GPIOB | RCC_HB2Periph_GPIOC | RCC_HB2Periph_GPIOD | RCC_HB2Periph_AFIO

/* SCK: PB13 (SPI2_SCK, AF5) */
#define TFT180_SCK_PORT         GPIOB
#define TFT180_SCK_PIN          GPIO_Pin_13
#define TFT180_SCK_AF           GPIO_AF5

/* MOSI: PC1 (SPI2_MOSI, AF5) */
#define TFT180_MOSI_PORT        GPIOC
#define TFT180_MOSI_PIN         GPIO_Pin_1
#define TFT180_MOSI_AF          GPIO_AF5

/* 控制引脚 (普通 GPIO) */
#define TFT180_CS_PORT          GPIOD
#define TFT180_CS_PIN           GPIO_Pin_10

#define TFT180_DC_PORT          GPIOD
#define TFT180_DC_PIN           GPIO_Pin_9

#define TFT180_RST_PORT         GPIOD
#define TFT180_RST_PIN          GPIO_Pin_8

#define TFT180_BL_PORT          GPIOD
#define TFT180_BL_PIN           GPIO_Pin_11

/* ==================== 控制宏 ==================== */
#define TFT180_CS(x)            (x ? (TFT180_CS_PORT->BSHR = TFT180_CS_PIN) : (TFT180_CS_PORT->BCR = TFT180_CS_PIN))
#define TFT180_DC(x)            (x ? (TFT180_DC_PORT->BSHR = TFT180_DC_PIN) : (TFT180_DC_PORT->BCR = TFT180_DC_PIN))
#define TFT180_RST(x)           (x ? (TFT180_RST_PORT->BSHR = TFT180_RST_PIN) : (TFT180_RST_PORT->BCR = TFT180_RST_PIN))
#define TFT180_BL(x)            (x ? (TFT180_BL_PORT->BSHR = TFT180_BL_PIN) : (TFT180_BL_PORT->BCR = TFT180_BL_PIN))

/* ==================== SPI 速度 ==================== */
/* CH32H417 V5F 主频 400MHz, SPI2 挂在 HB1 总线
 * HB1 时钟 = SYSCLK/2 = 200MHz
 * 分频因子 256 -> SPI 时钟约 780kHz (安全初始化速度)
 * 初始化后可切换到更高速度 */
#define TFT180_SPI_INIT_PRESCALER    SPI_BaudRatePrescaler_Mode7  /* /256, 初始化用慢速 */
#define TFT180_SPI_RUN_PRESCALER     SPI_BaudRatePrescaler_Mode3  /* /16, ~12.5MHz */

/* ==================== 显示参数 ==================== */
#define TFT180_WIDTH_MAX        160
#define TFT180_HEIGHT_MAX       128

/* ==================== 颜色定义 (RGB565) ==================== */
#define RGB565_WHITE            0xFFFF
#define RGB565_BLACK            0x0000
#define RGB565_RED              0xF800
#define RGB565_GREEN            0x07E0
#define RGB565_BLUE             0x001F
#define RGB565_YELLOW           0xFFE0
#define RGB565_CYAN             0x07FF
#define RGB565_MAGENTA          0xF81F

/* ==================== 显示方向枚举 ==================== */
typedef enum {
    TFT180_PORTAIT       = 0,    /* 竖屏模式 */
    TFT180_PORTAIT_180   = 1,    /* 竖屏模式 旋转180 */
    TFT180_CROSSWISE     = 2,    /* 横屏模式 */
    TFT180_CROSSWISE_180 = 3,    /* 横屏模式 旋转180 */
} tft180_dir_enum;

/* ==================== 字体大小枚举 ==================== */
typedef enum {
    TFT180_6X8_FONT     = 0,    /* 6x8  字体 */
    TFT180_8X16_FONT    = 1,    /* 8x16 字体 */
    TFT180_16X16_FONT   = 2,    /* 16x16 字体 (暂不支持) */
} tft180_font_size_enum;

/* ==================== 默认配置 ==================== */
#define TFT180_DEFAULT_DIR      TFT180_CROSSWISE     /* 默认横屏 */
#define TFT180_DEFAULT_PENCOLOR RGB565_WHITE          /* 默认画笔白色 */
#define TFT180_DEFAULT_BGCOLOR  RGB565_BLACK          /* 默认背景黑色 */
#define TFT180_DEFAULT_FONT     TFT180_8X16_FONT      /* 默认8x16字体 */

/* ==================== 函数声明 ==================== */
/* 初始化 */
void tft180_init(void);

/* 基础显示 */
void tft180_clear(void);
void tft180_full(const uint16_t color);
void tft180_set_dir(tft180_dir_enum dir);
void tft180_set_font(tft180_font_size_enum font);
void tft180_set_color(const uint16_t pen, const uint16_t bgcolor);

/* 绘图 */
void tft180_draw_point(uint16_t x, uint16_t y, const uint16_t color);
void tft180_draw_line(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const uint16_t color);

/* 文字显示 */
void tft180_show_char(uint16_t x, uint16_t y, const char dat);
void tft180_show_string(uint16_t x, uint16_t y, const char dat[]);
void tft180_show_int(uint16_t x, uint16_t y, const int32_t dat, uint8_t num);
void tft180_show_uint(uint16_t x, uint16_t y, const uint32_t dat, uint8_t num);
void tft180_show_float(uint16_t x, uint16_t y, const double dat, uint8_t num, uint8_t pointnum);

/* 图像显示 */
void tft180_show_binary_image(uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height);
void tft180_show_gray_image(uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t threshold);
void tft180_show_rgb565_image(uint16_t x, uint16_t y, const uint16_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t color_mode);

/* 波形显示 */
void tft180_show_wave(uint16_t x, uint16_t y, const uint16_t *wave, uint16_t width, uint16_t value_max, uint16_t dis_width, uint16_t dis_value_max);

/* 中文显示 */
void tft180_show_chinese(uint16_t x, uint16_t y, uint8_t size, const uint8_t *chinese_buffer, uint8_t number, const uint16_t color);

/* 内部函数 (供调试使用) */
void tft180_write_8bit_data(uint8_t data);
void tft180_write_16bit_data(uint16_t data);
void tft180_write_index(uint8_t dat);

#endif /* __TFT180_H */
