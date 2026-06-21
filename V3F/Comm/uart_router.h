/********************************** (C) COPYRIGHT ******************************
 * File Name          : uart_router.h
 * Description        : 多串口数据路由头文件
 *                      CH32H417 引脚映射（基于数据手册确认）
 ******************************************************************************
 * 引脚分配：
 *   USART1 (PA9/PA10)   → 调试/蓝牙    19200bps  AF7
 *   USART3 (PB10/PB11)  → Python主通信  115200bps AF7
 *   USART4 (PC6/PC7)    → 语音模块      9600bps   AF7
 *   USART6 (PC10/PC11)  → 人脸坐标      115200bps AF8
 *   USART8 (PA15/PA8)   → 手势指令      9600bps   AF11
 ******************************************************************************
 */

#ifndef __UART_ROUTER_H
#define __UART_ROUTER_H

#include "ch32h417.h"

/* 串口缓冲区大小 */
#define UART_RX_BUF_SIZE    64
#define FACE_COORD_BUF_SIZE 32

/* 初始化所有UART */
void UART_Router_Init(void);

/* UART中断解析函数 — 在ch32h417_it.c中调用 */
void usart1_parse_debug(uint8_t c);      /* USART1: 调试/蓝牙 */
void usart3_parse_command(uint8_t c);    /* USART3: Python主通信 */
void usart4_parse_voice(uint8_t c);      /* USART4: 语音模块 */
void usart6_parse_face_coord(uint8_t c); /* USART6: 人脸坐标 */
void usart8_parse_gesture(uint8_t c);    /* USART8: 手势指令 */

/* 发送语音触发信号到Python */
void send_voice_command(void);
void forward_gesture_to_pc(uint8_t c);
void send_voice_feedback(uint8_t c);


/* 串口接收诊断计数器 */
extern volatile uint32_t usart3_rx_count;
extern volatile uint32_t usart6_rx_count;
extern volatile uint32_t usart2_rx_count;
extern volatile uint32_t usart8_rx_count;

#endif /* __UART_ROUTER_H */
