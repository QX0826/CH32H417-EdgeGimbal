/********************************** (C) COPYRIGHT ******************************
 * File Name          : bluetooth.c
 * Description        : CH9141 蓝牙模块驱动 (CH32H417)
 *                      支持: 命令接收、数据上报、远程调参
 *                      协议:
 *                        接收: "1"~"9"=命令, "S1=XXXX"=设MQ7阈值, "S2=XXXX"=MQ4, "S3=XXXX"=MQ2
 *                        发送: "D:mq7=XXXX mq4=XXXX mq2=XXXX f=X\r\n" 传感器数据
 ******************************************************************************
 */
#include "bluetooth.h"
#include "debug.h"
#include "shared_gimbal.h"
#include <string.h>
#include <stdio.h>

Bluetooth_t bt = {0};

void Bluetooth_Init(void)
{
    memset(&bt, 0, sizeof(Bluetooth_t));
}

void Bluetooth_SendString(const char* str)
{
    while(*str)
    {
        USART_SendData(BT_USART, *str++);
        volatile uint32_t timeout = 100000;
        while((USART_GetFlagStatus(BT_USART, USART_FLAG_TXE) == RESET) && (timeout > 0))
        {
            timeout--;
        }
    }
}

uint8_t Bluetooth_GetCommand(void)
{
    if(bt.rx_complete)
    {
        uint8_t cmd = bt.last_cmd;
        bt.rx_complete = 0;
        return cmd;
    }
    return 0;
}

void Bluetooth_Process(void)
{
    uint8_t cmd = Bluetooth_GetCommand();
    if(cmd > 0)
    {
        gimbal_shared.command = cmd;
        gimbal_shared.command_ready = 1;
    }
}

void Bluetooth_ReportData(void)
{
    uint16_t mq7 = 0, mq4 = 0, mq2 = 0;
    uint8_t flame = 0;
    mq7   = gimbal_shared.report_mq7;
    mq4   = gimbal_shared.report_mq4;
    mq2   = gimbal_shared.report_mq2;
    flame = gimbal_shared.report_flame;
    char buf[64];
    sprintf(buf, "D:mq7=%04d mq4=%04d mq2=%04d f=%d\r\n",
            (int)mq7, (int)mq4, (int)mq2, (int)flame);
    Bluetooth_SendString(buf);
}

void Bluetooth_ReceiveByte(uint8_t c)
{
    if(bt.cmd_len == 0 && c >= '1' && c <= '9')
    {
        bt.last_cmd = c - '0';
        bt.rx_complete = 1;
        return;
    }
    if(c == '\n' || c == '\r')
    {
        bt.cmd_buf[bt.cmd_len] = '\0';
        if(bt.cmd_len >= 4 && bt.cmd_buf[0] == 'S')
        {
            uint8_t sid = bt.cmd_buf[1] - '0';
            uint16_t val = 0;
            char *p = &bt.cmd_buf[3];
            while(*p >= '0' && *p <= '9')
            {
                val = val * 10 + (*p - '0');
                p++;
            }
            if(val > 0)
            {
                if(HSEM_FastTake(GIMBAL_HSEM_ID) == READY)
                {
                    switch(sid)
                    {
                        case 1: gimbal_shared.threshold_mq7 = val; break;
                        case 2: gimbal_shared.threshold_mq4 = val; break;
                        case 3: gimbal_shared.threshold_mq2 = val; break;
                    }
                    HSEM_ReleaseOneSem(GIMBAL_HSEM_ID, 0);
                }
                char ack[32];
                sprintf(ack, "OK S%d=%d\r\n", sid, (int)val);
                Bluetooth_SendString(ack);
            }
        }
        else if(bt.cmd_len == 1 && bt.cmd_buf[0] == 'R')
        {
            Bluetooth_ReportData();
        }
        bt.cmd_len = 0;
    }
    else
    {
        if(bt.cmd_len < sizeof(bt.cmd_buf) - 1)
            bt.cmd_buf[bt.cmd_len++] = c;
    }
}
