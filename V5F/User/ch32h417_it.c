/********************************** (C) COPYRIGHT ******************************
 * File Name          : ch32h417_it.c
 * Description        : V5F 中断服务函数
 ******************************************************************************
 */

#include "ch32h417_it.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 * @brief   NMI异常处理
 * @return  none
 */
void NMI_Handler(void)
{
    while(1);
}

/*********************************************************************
 * @fn      HardFault_Handler
 * @brief   HardFault异常处理
 * @return  none
 */
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while(1);
}
