/********************************** (C) COPYRIGHT *******************************
 * File Name          : font.h
 * Description        : 字体数据 (ASCII 6x8 和 8x16)
 *                      基于逐飞科技 CH32V307 开源库
 *                      格式: 阴码、逐行式、顺向
 ******************************************************************************
 */
#ifndef __FONT_H
#define __FONT_H

#include "ch32h417.h"

/* ASCII 字体数据 */
extern const uint8_t ascii_font_6x8[][6];
extern const uint8_t ascii_font_8x16[][16];

#endif /* __FONT_H */
