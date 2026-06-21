/********************************** (C) COPYRIGHT ******************************
 * File Name          : bluetooth.h
 * Description        : CH9141 蓝牙模块驱动 (CH32H417)
 *                      与调试口共享 USART1 (PA9/PA10)
 *                      支持: 命令接收、数据上报、远程调参
 ******************************************************************************
 */
#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "ch32h417.h"

/* 蓝牙使用 USART1 (与调试口共享) */
#define BT_USART            USART1

/* 缓冲区 */
#define BT_RX_BUF_SIZE      64

/* 蓝牙数据结构 */
typedef struct {
    uint8_t rx_buf[BT_RX_BUF_SIZE];
    volatile uint8_t rx_index;
    volatile uint8_t rx_complete;
    volatile uint8_t last_cmd;
    /* 协议解析 */
    char     cmd_buf[32];
    uint8_t  cmd_len;
} Bluetooth_t;

extern Bluetooth_t bt;

/* 基础功能 */
void     Bluetooth_Init(void);
void     Bluetooth_SendString(const char* str);
uint8_t  Bluetooth_GetCommand(void);
void     Bluetooth_Process(void);

/* 数据上报 (V3F调用, 读共享内存→蓝牙发送) */
void     Bluetooth_ReportData(void);

/* 接收处理 */
void     Bluetooth_ReceiveByte(uint8_t c);

#endif
