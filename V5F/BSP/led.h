/********************************** (C) COPYRIGHT ******************************
 * File Name          : led.h
 * Description        : LED 指示灯驱动 (CH32H417)
 *                      PC13 = LED1, PB10 = LED2
 ******************************************************************************
 */
#ifndef __LED_H
#define __LED_H

#include "ch32h417.h"

#define LED1_PORT   GPIOC
#define LED1_PIN    GPIO_Pin_13
#define LED2_PORT   GPIOB
#define LED2_PIN    GPIO_Pin_10

void LED_Init(void);
void LED1_On(void);
void LED1_Off(void);
void LED1_Toggle(void);
void LED2_On(void);
void LED2_Off(void);
void LED2_Toggle(void);

#endif
