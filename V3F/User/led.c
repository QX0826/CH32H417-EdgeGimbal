/********************************** (C) COPYRIGHT ******************************
 * File Name          : led.c
 * Description        : LED 指示灯驱动 (CH32H417)
 *                      从 CH32V307 led.c 迁移，寄存器名改为 HB2
 ******************************************************************************
 */
#include "led.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(LED1_PORT, &GPIO_InitStructure);

    LED1_Off();
}

void LED1_On(void)   { GPIO_ResetBits(LED1_PORT, LED1_PIN); }
void LED1_Off(void)  { GPIO_SetBits(LED1_PORT, LED1_PIN); }
void LED1_Toggle(void) { GPIO_WriteBit(LED1_PORT, LED1_PIN, !GPIO_ReadOutputDataBit(LED1_PORT, LED1_PIN)); }

void LED2_On(void)   { }
void LED2_Off(void)  { }
void LED2_Toggle(void) { }
