/********************************** (C) COPYRIGHT ******************************
 * File Name          : beep.h
 * Description        : 蜂鸣器驱动 (CH32H417)
 *                      PA4 = 蜂鸣器
 ******************************************************************************
 */
#ifndef __BEEP_H
#define __BEEP_H

#include "ch32h417.h"

#define BEEP_PORT   GPIOA
#define BEEP_PIN    GPIO_Pin_4

void Beep_Init(void);
void Beep_On(void);
void Beep_Off(void);
void Beep_Toggle(void);
void Beep_Beep(uint16_t ms);

#endif
