/********************************** (C) COPYRIGHT ******************************
 * File Name          : ch32h417_it.c
 * Description        : V3F 中断服务函数 (已按实际接线校正)
 ******************************************************************************
 * 实际接线 (CH340_TX→单片机_RX方向):
 *   COM10 (人脸坐标  115200): CH340_TX→PB11(USART3_RX AF7), CH340_RX→PB10(USART3_TX AF7)
 *   COM11 (手势指令   9600 ): CH340_TX→PC7 (USART4_RX AF7), CH340_RX→PC6 (USART4_TX AF7)
 *   COM12 (MCU主通信 115200): CH340_TX→PE8 (USART8_RX AF7), CH340_RX→PE9 (USART8_TX AF7)
 ******************************************************************************
 */

#include "ch32h417_it.h"
#include "shared_gimbal.h"
#include "uart_router.h"
/* #include "bluetooth.h" */  /* 蓝牙已禁用 */

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART8_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      NMI_Handler
 */
void NMI_Handler(void)
{
    while(1);
}

/*********************************************************************
 * @fn      HardFault_Handler
 */
void HardFault_Handler(void)
{
    NVIC_SystemReset();
    while(1);
}

/*********************************************************************
 * @fn      USART3_IRQHandler
 * @brief   COM10 人脸坐标串口 (115200bps) PB11(RX)/PB10(TX) AF7
 *          接收格式: "X,Y\n" 例如 "94,120\n"
 *          也接收手势字节 (Python会同时往COM10发gesture)，非坐标格式会被忽略
 */
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USART3);
        usart3_rx_count++;          /* 诊断计数: 屏幕第8行 "3:xx" */
        usart6_parse_face_coord(c); /* 解析 "X,Y\n" 格式人脸坐标 */
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}

/*********************************************************************
 * @fn      USART4_IRQHandler
 * @brief   COM11 手势指令串口 (9600bps) PC7(RX)/PC6(TX) AF7
 *          接收格式: 单字节 '0'-'7' + '\n'
 */
void USART4_IRQHandler(void)
{
    if(USART_GetITStatus(USART4, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USART4);
        usart6_rx_count++;          /* 诊断计数: 屏幕第8行 "6:xx" (复用变量名) */
        usart8_parse_gesture(c);    /* 解析手势命令 */
        USART_ClearITPendingBit(USART4, USART_IT_RXNE);
    }
}

/*********************************************************************
 * @fn      USART2_IRQHandler
 * @brief   语音模块指令接收串口 (9600bps) PD6(RX)/PD5(TX) AF7
 */
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USART2);
        usart2_rx_count++;          /* 诊断计数: 语音模块接收字节数 */
        
        if(c >= '0' && c <= '9')
        {
            usart8_parse_gesture(c);    /* 解析手势/语音命令 */
            forward_gesture_to_pc(c);
        }
        else if(c == 'o')           /* 兼容语音模块发送的 "off" 停止指令，映射为 '0' (CMD_STOP) */
        {
            usart8_parse_gesture('0');
            forward_gesture_to_pc('0');
        }
        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/*********************************************************************
 * @fn      USART8_IRQHandler
 * @brief   COM12 MCU主通信串口 (115200bps) PE7(RX)/PE8(TX) AF7
 *          接收手势命令 + 坐标触发信号 (Python往COM12也发gesture)
 */
void USART8_IRQHandler(void)
{
    if(USART_GetITStatus(USART8, USART_IT_RXNE) != RESET)
    {
        uint8_t c = USART_ReceiveData(USART8);
        usart8_rx_count++;          /* 诊断计数: 屏幕第8行 "8:xx" */
        usart3_parse_command(c);    /* 解析手势/主控命令 */
        USART_ClearITPendingBit(USART8, USART_IT_RXNE);
    }
}
