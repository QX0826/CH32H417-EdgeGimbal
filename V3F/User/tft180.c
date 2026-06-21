/********************************** (C) COPYRIGHT *******************************
 * File Name          : tft180.c
 * Description        : TFT180 (ST7735) 1.8寸彩屏驱动 (CH32H417 适配版)
 *                      基于逐飞科技 CH32V307 开源库移植
 *                      使用 CH32H417 标准 SPI 外设库
 ******************************************************************************
 */
#include "tft180.h"
#include "font.h"
#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ==================== 全局变量 ==================== */
static uint16_t tft180_pencolor    = TFT180_DEFAULT_PENCOLOR;
static uint16_t tft180_bgcolor     = TFT180_DEFAULT_BGCOLOR;
static tft180_dir_enum tft180_display_dir = TFT180_DEFAULT_DIR;
static tft180_font_size_enum tft180_display_font = TFT180_DEFAULT_FONT;
static uint8_t tft180_x_max       = 160;
static uint8_t tft180_y_max       = 128;

/* ==================== SPI 底层函数 ==================== */

/**
 * @brief  SPI 写 8bit 数据
 * @param  data 要发送的数据
 */
void tft180_write_8bit_data(uint8_t data)
{
    while(SPI_I2S_GetFlagStatus(TFT180_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(TFT180_SPI, data);
    while(SPI_I2S_GetFlagStatus(TFT180_SPI, SPI_I2S_FLAG_BSY) == SET);
}

/**
 * @brief  SPI 写 16bit 数据 (RGB565颜色)
 *         必须拆成两次 8bit 发送，因为 SPI 配置的是 8bit 模式
 */
void tft180_write_16bit_data(uint16_t data)
{
    /* 先发高字节 */
    while(SPI_I2S_GetFlagStatus(TFT180_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(TFT180_SPI, (data >> 8) & 0xFF);
    while(SPI_I2S_GetFlagStatus(TFT180_SPI, SPI_I2S_FLAG_BSY) == SET);
    /* 再发低字节 */
    while(SPI_I2S_GetFlagStatus(TFT180_SPI, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(TFT180_SPI, data & 0xFF);
    while(SPI_I2S_GetFlagStatus(TFT180_SPI, SPI_I2S_FLAG_BSY) == SET);
}

/**
 * @brief  写命令
 * @param  dat 命令字节
 */
void tft180_write_index(uint8_t dat)
{
    TFT180_DC(0);
    tft180_write_8bit_data(dat);
    TFT180_DC(1);
}

/* ==================== 内部函数 ==================== */

/**
 * @brief  设置显示区域
 * @param  x1 起始x坐标
 * @param  y1 起始y坐标
 * @param  x2 结束x坐标
 * @param  y2 结束y坐标
 */
static void tft180_set_region(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    switch(tft180_display_dir)
    {
        case TFT180_PORTAIT:
        case TFT180_PORTAIT_180:
        {
            tft180_write_index(0x2a);
            tft180_write_8bit_data(0x00);
            tft180_write_8bit_data(x1 + 2);
            tft180_write_8bit_data(0x00);
            tft180_write_8bit_data(x2 + 2);

            tft180_write_index(0x2b);
            tft180_write_8bit_data(0x00);
            tft180_write_8bit_data(y1 + 1);
            tft180_write_8bit_data(0x00);
            tft180_write_8bit_data(y2 + 1);
        } break;
        case TFT180_CROSSWISE:
        case TFT180_CROSSWISE_180:
        {
            tft180_write_index(0x2a);
            tft180_write_8bit_data(0x00);
            tft180_write_8bit_data(x1 + 1);
            tft180_write_8bit_data(0x00);
            tft180_write_8bit_data(x2 + 1);

            tft180_write_index(0x2b);
            tft180_write_8bit_data(0x00);
            tft180_write_8bit_data(y1 + 2);
            tft180_write_8bit_data(0x00);
            tft180_write_8bit_data(y2 + 2);
        } break;
    }
    tft180_write_index(0x2c);
}

/* ==================== ST7735 初始化序列 ==================== */

/**
 * @brief  ST7735 芯片初始化
 */
static void tft180_st7735_init(void)
{
    /* 硬件复位 */
    TFT180_RST(0);
    Delay_Ms(10);
    TFT180_RST(1);
    Delay_Ms(120);

    TFT180_CS(0);

    tft180_write_index(0x11);    /* Sleep out */
    Delay_Ms(120);

    tft180_write_index(0xB1);    /* Frame rate control */
    tft180_write_8bit_data(0x01);
    tft180_write_8bit_data(0x2C);
    tft180_write_8bit_data(0x2D);

    tft180_write_index(0xB2);
    tft180_write_8bit_data(0x01);
    tft180_write_8bit_data(0x2C);
    tft180_write_8bit_data(0x2D);

    tft180_write_index(0xB3);
    tft180_write_8bit_data(0x01);
    tft180_write_8bit_data(0x2C);
    tft180_write_8bit_data(0x2D);
    tft180_write_8bit_data(0x01);
    tft180_write_8bit_data(0x2C);
    tft180_write_8bit_data(0x2D);

    tft180_write_index(0xB4);    /* Column inversion */
    tft180_write_8bit_data(0x07);

    tft180_write_index(0xC0);    /* Power control */
    tft180_write_8bit_data(0xA2);
    tft180_write_8bit_data(0x02);
    tft180_write_8bit_data(0x84);

    tft180_write_index(0xC1);
    tft180_write_8bit_data(0xC5);

    tft180_write_index(0xC2);
    tft180_write_8bit_data(0x0A);
    tft180_write_8bit_data(0x00);

    tft180_write_index(0xC3);
    tft180_write_8bit_data(0x8A);
    tft180_write_8bit_data(0x2A);

    tft180_write_index(0xC4);
    tft180_write_8bit_data(0x8A);
    tft180_write_8bit_data(0xEE);

    tft180_write_index(0xC5);    /* VCOM */
    tft180_write_8bit_data(0x0E);

    tft180_write_index(0x36);    /* Memory access control */
    switch(tft180_display_dir)
    {
        case TFT180_PORTAIT:        tft180_write_8bit_data(1<<7 | 1<<6 | 0<<5); break;
        case TFT180_PORTAIT_180:    tft180_write_8bit_data(0<<7 | 0<<6 | 0<<5); break;
        case TFT180_CROSSWISE:      tft180_write_8bit_data(1<<7 | 0<<6 | 1<<5); break;
        case TFT180_CROSSWISE_180:  tft180_write_8bit_data(0<<7 | 1<<6 | 1<<5); break;
    }

    tft180_write_index(0xe0);    /* Gamma correction */
    tft180_write_8bit_data(0x0f);
    tft180_write_8bit_data(0x1a);
    tft180_write_8bit_data(0x0f);
    tft180_write_8bit_data(0x18);
    tft180_write_8bit_data(0x2f);
    tft180_write_8bit_data(0x28);
    tft180_write_8bit_data(0x20);
    tft180_write_8bit_data(0x22);
    tft180_write_8bit_data(0x1f);
    tft180_write_8bit_data(0x1b);
    tft180_write_8bit_data(0x23);
    tft180_write_8bit_data(0x37);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x07);
    tft180_write_8bit_data(0x02);
    tft180_write_8bit_data(0x10);

    tft180_write_index(0xe1);
    tft180_write_8bit_data(0x0f);
    tft180_write_8bit_data(0x1b);
    tft180_write_8bit_data(0x0f);
    tft180_write_8bit_data(0x17);
    tft180_write_8bit_data(0x33);
    tft180_write_8bit_data(0x2c);
    tft180_write_8bit_data(0x29);
    tft180_write_8bit_data(0x2e);
    tft180_write_8bit_data(0x30);
    tft180_write_8bit_data(0x30);
    tft180_write_8bit_data(0x39);
    tft180_write_8bit_data(0x3f);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x07);
    tft180_write_8bit_data(0x03);
    tft180_write_8bit_data(0x10);

    tft180_write_index(0x2a);    /* Column address set */
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x00 + 2);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x80 + 2);

    tft180_write_index(0x2b);    /* Row address set */
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x00 + 3);
    tft180_write_8bit_data(0x00);
    tft180_write_8bit_data(0x80 + 3);

    tft180_write_index(0xF0);    /* Enable test command */
    tft180_write_8bit_data(0x01);
    tft180_write_index(0xF6);    /* Disable ram power save mode */
    tft180_write_8bit_data(0x00);

    tft180_write_index(0x3A);    /* 16bit/pixel */
    tft180_write_8bit_data(0x05);

    tft180_write_index(0x29);    /* Display on */

    TFT180_CS(1);
}

/* ==================== 公开 API ==================== */

/**
 * @brief  清屏
 */
void tft180_clear(void)
{
    uint32_t i = tft180_x_max * tft180_y_max;

    TFT180_CS(0);
    tft180_set_region(0, 0, tft180_x_max - 1, tft180_y_max - 1);
    for(; 0 < i; i--)
    {
        tft180_write_16bit_data(tft180_bgcolor);
    }
    TFT180_CS(1);
}

/**
 * @brief  全屏填充指定颜色
 */
void tft180_full(const uint16_t color)
{
    uint32_t i = tft180_x_max * tft180_y_max;

    TFT180_CS(0);
    tft180_set_region(0, 0, tft180_x_max - 1, tft180_y_max - 1);
    for(; 0 < i; i--)
    {
        tft180_write_16bit_data(color);
    }
    TFT180_CS(1);
}

/**
 * @brief  设置显示方向 (必须在初始化前调用)
 */
void tft180_set_dir(tft180_dir_enum dir)
{
    tft180_display_dir = dir;
    switch(tft180_display_dir)
    {
        case TFT180_PORTAIT:
        case TFT180_PORTAIT_180:
        {
            tft180_x_max = 128;
            tft180_y_max = 160;
        } break;
        case TFT180_CROSSWISE:
        case TFT180_CROSSWISE_180:
        {
            tft180_x_max = 160;
            tft180_y_max = 128;
        } break;
    }
}

/**
 * @brief  设置显示字体
 */
void tft180_set_font(tft180_font_size_enum font)
{
    tft180_display_font = font;
}

/**
 * @brief  设置画笔颜色和背景颜色
 */
void tft180_set_color(const uint16_t pen, const uint16_t bgcolor)
{
    tft180_pencolor = pen;
    tft180_bgcolor  = bgcolor;
}

/**
 * @brief  画点
 */
void tft180_draw_point(uint16_t x, uint16_t y, const uint16_t color)
{
    TFT180_CS(0);
    tft180_set_region(x, y, x, y);
    tft180_write_16bit_data(color);
    TFT180_CS(1);
}

/**
 * @brief  画线
 */
void tft180_draw_line(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const uint16_t color)
{
    int16_t x_dir = (x_start < x_end ? 1 : -1);
    int16_t y_dir = (y_start < y_end ? 1 : -1);
    float temp_rate = 0;
    float temp_b = 0;

    if(x_start != x_end)
    {
        temp_rate = (float)(y_start - y_end) / (float)(x_start - x_end);
        temp_b = (float)y_start - (float)x_start * temp_rate;
    }
    else
    {
        while(y_start != y_end)
        {
            tft180_draw_point(x_start, y_start, color);
            y_start += y_dir;
        }
        return;
    }

    if(abs(y_start - y_end) > abs(x_start - x_end))
    {
        while(y_start != y_end)
        {
            tft180_draw_point(x_start, y_start, color);
            y_start += y_dir;
            x_start = (int16_t)(((float)y_start - temp_b) / temp_rate);
        }
    }
    else
    {
        while(x_start != x_end)
        {
            tft180_draw_point(x_start, y_start, color);
            x_start += x_dir;
            y_start = (int16_t)((float)x_start * temp_rate + temp_b);
        }
    }
}

/**
 * @brief  显示单个字符
 */
void tft180_show_char(uint16_t x, uint16_t y, const char dat)
{
    uint8_t i = 0, j = 0;

    TFT180_CS(0);
    switch(tft180_display_font)
    {
        case TFT180_6X8_FONT:
        {
            for(i = 0; 6 > i; i++)
            {
                tft180_set_region(x + i, y, x + i, y + 7);
                uint8_t temp = ascii_font_6x8[dat - 32][i];
                for(j = 0; 8 > j; j++)
                {
                    if(temp & 0x01)
                        tft180_write_16bit_data(tft180_pencolor);
                    else
                        tft180_write_16bit_data(tft180_bgcolor);
                    temp >>= 1;
                }
            }
        } break;
        case TFT180_8X16_FONT:
        {
            for(i = 0; 8 > i; i++)
            {
                tft180_set_region(x + i, y, x + i, y + 15);
                uint8_t temp_top = ascii_font_8x16[dat - 32][i];
                uint8_t temp_bottom = ascii_font_8x16[dat - 32][i + 8];
                for(j = 0; 8 > j; j++)
                {
                    if(temp_top & 0x01)
                        tft180_write_16bit_data(tft180_pencolor);
                    else
                        tft180_write_16bit_data(tft180_bgcolor);
                    temp_top >>= 1;
                }
                for(j = 0; 8 > j; j++)
                {
                    if(temp_bottom & 0x01)
                        tft180_write_16bit_data(tft180_pencolor);
                    else
                        tft180_write_16bit_data(tft180_bgcolor);
                    temp_bottom >>= 1;
                }
            }
        } break;
        case TFT180_16X16_FONT:
        {
            /* 暂不支持 */
        } break;
    }
    TFT180_CS(1);
}

/**
 * @brief  显示字符串
 */
void tft180_show_string(uint16_t x, uint16_t y, const char dat[])
{
    uint16_t j = 0;
    while('\0' != dat[j])
    {
        switch(tft180_display_font)
        {
            case TFT180_6X8_FONT:   tft180_show_char(x + 6 * j, y, dat[j]); break;
            case TFT180_8X16_FONT:  tft180_show_char(x + 8 * j, y, dat[j]); break;
            case TFT180_16X16_FONT: break;
        }
        j++;
    }
}

/**
 * @brief  显示32位有符号整数
 */
void tft180_show_int(uint16_t x, uint16_t y, const int32_t dat, uint8_t num)
{
    int32_t dat_temp = dat;
    int32_t offset = 1;
    char data_buffer[12];

    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num + 1);

    if(10 > num)
    {
        for(; 0 < num; num--)
            offset *= 10;
        dat_temp %= offset;
    }
    sprintf(data_buffer, "%d", (int)dat_temp);
    tft180_show_string(x, y, data_buffer);
}

/**
 * @brief  显示32位无符号整数
 */
void tft180_show_uint(uint16_t x, uint16_t y, const uint32_t dat, uint8_t num)
{
    uint32_t dat_temp = dat;
    int32_t offset = 1;
    char data_buffer[12];

    memset(data_buffer, 0, 12);
    memset(data_buffer, ' ', num);

    if(10 > num)
    {
        for(; 0 < num; num--)
            offset *= 10;
        dat_temp %= offset;
    }
    sprintf(data_buffer, "%u", (unsigned int)dat_temp);
    tft180_show_string(x, y, data_buffer);
}

/**
 * @brief  显示浮点数
 */
void tft180_show_float(uint16_t x, uint16_t y, const double dat, uint8_t num, uint8_t pointnum)
{
    double dat_temp = dat;
    double offset = 1.0;
    char data_buffer[17];

    memset(data_buffer, 0, 17);
    memset(data_buffer, ' ', num + pointnum + 2);

    for(; 0 < num; num--)
        offset *= 10;
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    sprintf(data_buffer, "%.*f", pointnum, dat_temp);
    tft180_show_string(x, y, data_buffer);
}

/**
 * @brief  显示二值图像
 */
void tft180_show_binary_image(uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height)
{
    uint32_t i = 0, j = 0;
    uint8_t temp = 0;
    uint32_t width_index = 0, height_index = 0;

    TFT180_CS(0);
    tft180_set_region(x, y, x + dis_width - 1, y + dis_height - 1);

    for(j = 0; j < dis_height; j++)
    {
        height_index = j * height / dis_height;
        for(i = 0; i < dis_width; i++)
        {
            width_index = i * width / dis_width;
            temp = *(image + height_index * width / 8 + width_index / 8);
            if(0x80 & (temp << (width_index % 8)))
                tft180_write_16bit_data(RGB565_WHITE);
            else
                tft180_write_16bit_data(RGB565_BLACK);
        }
    }
    TFT180_CS(1);
}

/**
 * @brief  显示8bit灰度图像
 */
void tft180_show_gray_image(uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t threshold)
{
    uint32_t i = 0, j = 0;
    uint16_t color = 0, temp = 0;
    uint32_t width_index = 0, height_index = 0;

    TFT180_CS(0);
    tft180_set_region(x, y, x + dis_width - 1, y + dis_height - 1);

    for(j = 0; j < dis_height; j++)
    {
        height_index = j * height / dis_height;
        for(i = 0; i < dis_width; i++)
        {
            width_index = i * width / dis_width;
            temp = *(image + height_index * width + width_index);
            if(threshold == 0)
            {
                color = (0x001f & ((temp) >> 3)) << 11;
                color = color | (((0x003f) & ((temp) >> 2)) << 5);
                color = color | (0x001f & ((temp) >> 3));
                tft180_write_16bit_data(color);
            }
            else if(temp < threshold)
                tft180_write_16bit_data(RGB565_BLACK);
            else
                tft180_write_16bit_data(RGB565_WHITE);
        }
    }
    TFT180_CS(1);
}

/**
 * @brief  显示RGB565彩色图像
 */
void tft180_show_rgb565_image(uint16_t x, uint16_t y, const uint16_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t color_mode)
{
    uint32_t i = 0, j = 0;
    uint16_t color = 0;
    uint32_t width_index = 0, height_index = 0;

    TFT180_CS(0);
    tft180_set_region(x, y, x + dis_width - 1, y + dis_height - 1);

    for(j = 0; j < dis_height; j++)
    {
        height_index = j * height / dis_height;
        for(i = 0; i < dis_width; i++)
        {
            width_index = i * width / dis_width;
            color = *(image + height_index * width + width_index);
            if(color_mode)
                color = ((color & 0xff) << 8) | (color >> 8);
            tft180_write_16bit_data(color);
        }
    }
    TFT180_CS(1);
}

/**
 * @brief  显示波形
 */
void tft180_show_wave(uint16_t x, uint16_t y, const uint16_t *wave, uint16_t width, uint16_t value_max, uint16_t dis_width, uint16_t dis_value_max)
{
    uint16_t temp = 0;
    uint16_t i = 0;

    /* 清除显示区域 */
    TFT180_CS(0);
    tft180_set_region(x, y, x + dis_width - 1, y + dis_value_max - 1);
    for(i = 0; i < dis_width * dis_value_max; i++)
        tft180_write_16bit_data(tft180_bgcolor);
    TFT180_CS(1);

    /* 绘制波形 */
    for(i = 0; i < dis_width - 1; i++)
    {
        temp = (uint16_t)(wave[i * width / dis_width] * dis_value_max / value_max);
        if(temp > dis_value_max - 1) temp = dis_value_max - 1;
        tft180_draw_point(x + i, y + dis_value_max - 1 - temp, tft180_pencolor);
    }
}

/**
 * @brief  显示中文
 */
void tft180_show_chinese(uint16_t x, uint16_t y, uint8_t size, const uint8_t *chinese_buffer, uint8_t number, const uint16_t color)
{
    int i = 0, j = 0, k = 0;
    uint8_t temp = 0, temp1 = 0, temp2 = 0;
    const uint8_t *p_data = chinese_buffer;

    temp2 = size / 8;

    TFT180_CS(0);
    tft180_set_region(x, y, number * size - 1 + x, y + size - 1);

    for(i = 0; i < size; i++)
    {
        temp1 = number;
        p_data = chinese_buffer + i * temp2;
        while(temp1--)
        {
            for(k = 0; k < temp2; k++)
            {
                for(j = 8; 0 < j; j--)
                {
                    temp = (*p_data >> (j - 1)) & 0x01;
                    if(temp)
                        tft180_write_16bit_data(color);
                    else
                        tft180_write_16bit_data(tft180_bgcolor);
                }
                p_data++;
            }
            p_data = p_data - temp2 + temp2 * size;
        }
    }
    TFT180_CS(1);
}

/**
 * @brief  TFT180 初始化
 * @note   在调用前可使用 tft180_set_dir() 设置显示方向
 */
void tft180_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef SPI_InitStructure = {0};

    /* 使能时钟 */
    RCC_HB2PeriphClockCmd(TFT180_SPI_GPIO_CLK, ENABLE);
    RCC_HB1PeriphClockCmd(TFT180_SPI_CLK, ENABLE);

    /* 配置 SPI 引脚 */
    /* SCK: PB13 (AF5) */
    GPIO_PinAFConfig(TFT180_SCK_PORT, GPIO_PinSource13, TFT180_SCK_AF);
    GPIO_InitStructure.GPIO_Pin   = TFT180_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(TFT180_SCK_PORT, &GPIO_InitStructure);

    /* MOSI: PC1 (AF5) */
    GPIO_PinAFConfig(TFT180_MOSI_PORT, GPIO_PinSource1, TFT180_MOSI_AF);
    GPIO_InitStructure.GPIO_Pin   = TFT180_MOSI_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(TFT180_MOSI_PORT, &GPIO_InitStructure);

    /* 配置控制引脚为推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = TFT180_CS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(TFT180_CS_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TFT180_DC_PIN;
    GPIO_Init(TFT180_DC_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TFT180_RST_PIN;
    GPIO_Init(TFT180_RST_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = TFT180_BL_PIN;
    GPIO_Init(TFT180_BL_PORT, &GPIO_InitStructure);

    /* 默认状态 */
    TFT180_CS(1);
    TFT180_DC(1);
    TFT180_RST(1);
    TFT180_BL(1);    /* 打开背光 */

    /* 配置 SPI */
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode      = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize  = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL      = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA      = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS       = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = TFT180_SPI_INIT_PRESCALER;
    SPI_InitStructure.SPI_FirstBit  = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(TFT180_SPI, &SPI_InitStructure);

    SPI_Cmd(TFT180_SPI, ENABLE);

    /* 设置显示方向 */
    tft180_set_dir(tft180_display_dir);

    /* 初始化 ST7735 */
    tft180_st7735_init();

    /* 切换到高速 SPI */
    SPI_Cmd(TFT180_SPI, DISABLE);
    SPI_InitStructure.SPI_BaudRatePrescaler = TFT180_SPI_RUN_PRESCALER;
    SPI_Init(TFT180_SPI, &SPI_InitStructure);
    SPI_Cmd(TFT180_SPI, ENABLE);

    /* 清屏 */
    tft180_clear();
}
