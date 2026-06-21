/********************************** (C) COPYRIGHT ******************************
 * File Name          : beep.c
 * Description        : 蜂鸣器驱动 (CH32H417)
 *                      从 CH32V307 beep.c 迁移
 ******************************************************************************
 */
#include "beep.h"
#include "debug.h"

void Beep_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(BEEP_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(BEEP_PORT, BEEP_PIN);
}

void Beep_On(void)     { GPIO_SetBits(BEEP_PORT, BEEP_PIN); }
void Beep_Off(void)    { GPIO_ResetBits(BEEP_PORT, BEEP_PIN); }
void Beep_Toggle(void) { GPIO_WriteBit(BEEP_PORT, BEEP_PIN, !GPIO_ReadOutputDataBit(BEEP_PORT, BEEP_PIN)); }

void Beep_Beep(uint16_t ms)
{
    Beep_On();
    Delay_Ms(ms);
    Beep_Off();
}
