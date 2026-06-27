/********************************** (C) COPYRIGHT ******************************
 * File Name          : uart_router.c
 * Description        : 多串口数据路由 (WCH标准库版本)
 *                      CH32H417 引脚映射（已根据实际接线校正）
 *
 * 实际接线（以CH340模块_TX→单片机_RX方向为准）：
 *   COM10 (人脸坐标  115200bps): CH340_TX→PB8(USART6_RX AF8), CH340_RX→PB9(USART6_TX AF8)
 *   COM11 (手势指令  9600bps  ): CH340_TX→PC7(USART4_RX AF7), CH340_RX→PC6(USART4_TX AF7)
 *   COM12 (MCU主通信 115200bps): CH340_TX→PE8(USART8_RX AF7), CH340_RX→PE9(USART8_TX AF7)
 *
 * 注意: PE14/PE15 在 CH32H417 上无 UART TX/RX 功能，不能用于串口通信！
 ******************************************************************************
 */

#include "debug.h"
#include "uart_router.h"

/* 串口接收诊断计数器 */
volatile uint32_t usart3_rx_count = 0;   /* COM10 人脸坐标 & 手势接收计数 */
volatile uint32_t usart6_rx_count = 0;   /* COM11 PC手势串口 (USART4) 接收计数 */
volatile uint32_t usart2_rx_count = 0;   /* 语音模块 (USART2) 接收计数 */
volatile uint32_t usart8_rx_count = 0;   /* COM12 MCU主通信 (USART8) 接收计数 */
#include "shared_gimbal.h"
#include <string.h>
#include <stdlib.h>

/* 人脸坐标解析缓冲区 */
static char face_rx_buf[FACE_COORD_BUF_SIZE];
static uint8_t face_rx_index = 0;

/* 语音模块缓冲区 */
static uint8_t voice_cmd_buf[8];
static uint8_t voice_cmd_index = 0;

/*********************************************************************
 * @fn      USARTx_Init
 * @brief   初始化单个USART (WCH标准库版本)
 */
static void USARTx_Init(USART_TypeDef* USARTx, uint32_t baudrate,
                         GPIO_TypeDef* tx_port, uint16_t tx_pin, uint8_t tx_af,
                         GPIO_TypeDef* rx_port, uint16_t rx_pin, uint8_t rx_af,
                         uint32_t periph_clock, uint8_t is_hb2)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    /* 使能时钟 */
    if(is_hb2)
        RCC_HB2PeriphClockCmd(periph_clock, ENABLE);
    else
        RCC_HB1PeriphClockCmd(periph_clock, ENABLE);
    /* 显式使能所有GPIO端口时钟 (避免位移计算错误) */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA | RCC_HB2Periph_GPIOB |
                          RCC_HB2Periph_GPIOC | RCC_HB2Periph_GPIOD |
                          RCC_HB2Periph_GPIOE | RCC_HB2Periph_AFIO, ENABLE);

    /* TX引脚复用 */
    GPIO_PinAFConfig(tx_port, tx_pin, tx_af);
    GPIO_InitStructure.GPIO_Pin = (1 << tx_pin);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(tx_port, &GPIO_InitStructure);

    /* RX引脚复用 */
    GPIO_PinAFConfig(rx_port, rx_pin, rx_af);
    GPIO_InitStructure.GPIO_Pin = (1 << rx_pin);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(rx_port, &GPIO_InitStructure);

    /* USART配置 */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USARTx, &USART_InitStructure);

    /* 使能接收中断 */
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

    /* 使能USART */
    USART_Cmd(USARTx, ENABLE);
}

/*********************************************************************
 * @fn      UART_Router_Init
 * @brief   初始化串口（严格对应实际接线引脚）
 *
 * COM10 (人脸坐标 115200): CH340_TX→PB11(USART3_RX AF7), CH340_RX→PB10(USART3_TX AF7)
 * COM11 (手势指令  9600 ): CH340_TX→PC7 (USART4_RX AF7), CH340_RX→PC6 (USART4_TX AF7)
 * COM12 (MCU主通信 115200): CH340_TX→PE8 (USART8_RX AF7), CH340_RX→PE9 (USART8_TX AF7)
 * @return  none
 */
void UART_Router_Init(void)
{
    /* USART1: 语音播报反馈 (19200bps)
     *   TX: PA9(AF7)  → 语音模块RX2(PA6) 接这里
     *   RX: PA10(AF7) → 语音模块TX2(PA5) 接这里 */
    USARTx_Init(USART1, 19200,
                GPIOA, 9, GPIO_AF7, GPIOA, 10, GPIO_AF7,
                RCC_HB2Periph_USART1, 1);

    /* USART3: COM10 人脸坐标 (115200bps)
     *   TX: PB10(AF7) ← CH340_RX 接这里
     *   RX: PB11(AF7) ← CH340_TX 接这里
     *   注: PB8/PB9 被烧录口占用，改用 PB10/PB11 */
    USARTx_Init(USART3, 115200,
                GPIOB, 10, GPIO_AF7, GPIOB, 11, GPIO_AF7,
                RCC_HB1Periph_USART3, 0);

    /* USART4: COM11 手势指令 (9600bps)
     *   TX: PC6(AF7)  ← PC CH340_RX 接这里
     *   RX: PC7(AF7)  ← PC CH340_TX 接这里  */
    USARTx_Init(USART4, 9600,
                GPIOC, 6, GPIO_AF7, GPIOC, 7, GPIO_AF7,
                RCC_HB1Periph_USART4, 0);

    /* USART2: 语音模块指令接收 (9600bps)
     *   TX: PD5(AF7) → 语音模块RX1(PA3) 接这里
     *   RX: PD6(AF7) ← 语音模块TX1(PA2) 接这里  */
    USARTx_Init(USART2, 9600,
                GPIOD, 5, GPIO_AF7, GPIOD, 6, GPIO_AF7,
                RCC_HB1Periph_USART2, 0);

    /* USART8: COM12 MCU主通信 (115200bps)
     *   TX: PE8(AF7)  → CH340_RX 接这里
     *   RX: PE7(AF7)  ← CH340_TX 接这里  */
    USARTx_Init(USART8, 115200,
                GPIOE, 8, GPIO_AF7, GPIOE, 7, GPIO_AF7,
                RCC_HB1Periph_USART8, 0);

    /* 使使能NVIC中断 */
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
    NVIC_EnableIRQ(USART4_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_EnableIRQ(USART8_IRQn);
}

/*********************************************************************
 * @fn      usart3_parse_command
 * @brief   解析USART3收到的Python命令 (手势指令)
 *          格式: 单字节ASCII '0'-'7'
 */
void usart3_parse_command(uint8_t c)
{
    if(c >= '0' && c <= '9')
    {
        uint8_t cmd = c - '0';
        gimbal_shared.com_digit = cmd;
        if(cmd <= 7)
        {
            gimbal_shared.command = cmd;
            gimbal_shared.command_ready = 1;
        }
    }
}

/*********************************************************************
 * @fn      parse_float_manual
 * @brief   手动解析正浮点数 (避免使用 stdlib 的 atof 以免触发嵌入式运行时库的 heap/lockup 崩溃)
 */
static float parse_float_manual(const char* s)
{
    float val = 0.0f;
    while(*s == ' ' || *s == '\t') s++;
    while(*s >= '0' && *s <= '9')
    {
        val = val * 10.0f + (*s - '0');
        s++;
    }
    if(*s == '.')
    {
        s++;
        float weight = 0.1f;
        while(*s >= '0' && *s <= '9')
        {
            val += (*s - '0') * weight;
            weight *= 0.1f;
            s++;
        }
    }
    return val;
}

/*********************************************************************
 * @fn      usart6_parse_face_coord
 * @brief   解析USART6收到的人脸坐标
 *          格式: "X,Y\n" 例如 "94,120\n"
 */
void usart6_parse_face_coord(uint8_t c)
{
    if(c == '\n' || c == '\r')
    {
        face_rx_buf[face_rx_index] = '\0';

        if(face_rx_index > 0)
        {
            float x = 0.0f, y = 0.0f;
            int parsed = 0;
            char* comma = strchr(face_rx_buf, ',');
            if(comma)
            {
                *comma = '\0';
                x = parse_float_manual(face_rx_buf);
                y = parse_float_manual(comma + 1);
                *comma = ','; // restore comma
                parsed = 2;
            }
            else
            {
                x = parse_float_manual(face_rx_buf);
                parsed = 1;
            }

            if(parsed == 2)
            {
                gimbal_shared.face_x = (int16_t)(x * 10.0f);
                gimbal_shared.face_y = (int16_t)(y * 10.0f);
                gimbal_shared.face_ready = 1;
            }
            else if(parsed == 1)
            {
                /* 只有一个数字，说明是通过 COM10 (USART3) 复用通道发送过来的 PC 手势指令 */
                if(x >= 0.0f && x <= 9.0f)
                {
                    gimbal_shared.com_digit = (uint8_t)x;
                    if(x <= 7.0f)
                    {
                        gimbal_shared.command = (uint8_t)x;
                        gimbal_shared.command_ready = 1;
                    }
                }
            }
        }
        face_rx_index = 0;
    }
    else
    {
        if(face_rx_index < FACE_COORD_BUF_SIZE - 1)
        {
            face_rx_buf[face_rx_index++] = (char)c;
        }
    }
}

/*********************************************************************
 * @fn      usart4_parse_voice
 * @brief   解析USART4收到的语音模块数据
 *          LD3320帧格式: 0x0E, 0x0F, 识别码
 */
void usart4_parse_voice(uint8_t c)
{
    voice_cmd_buf[voice_cmd_index] = c;

    if(voice_cmd_index == 0 && c != 0x0E)
        return;
    if(voice_cmd_index == 1 && c != 0x0F)
    {
        voice_cmd_index = 0;
        return;
    }

    voice_cmd_index++;

    if(voice_cmd_index >= 3)
    {
        uint8_t voice_result = voice_cmd_buf[2];
        voice_cmd_index = 0;

        if(voice_result != 0)
        {
            send_voice_command();
        }
    }
}

/*********************************************************************
 * @fn      usart8_parse_gesture
 * @brief   解析USART8收到的手势模块数据
 */
void usart8_parse_gesture(uint8_t c)
{
    if(c >= '0' && c <= '9')
    {
        uint8_t cmd = c - '0';
        gimbal_shared.com_digit = cmd;
        if(cmd <= 7)
        {
            gimbal_shared.command = cmd;
            gimbal_shared.command_ready = 1;
        }
    }
}

/*********************************************************************
 * @fn      usart1_parse_debug
 * @brief   解析USART1收到的调试/蓝牙数据
 */
void usart1_parse_debug(uint8_t c)
{
    /* 不阻塞中断，直接返回 */
    (void)c;
}

/*********************************************************************
 * @fn      send_voice_command
 * @brief   转发语音触发信号到 USART8 (COM12, Python MCU通信口)
 */
void send_voice_command(void)
{
    USART_SendData(USART8, '5');
    volatile uint32_t timeout1 = 100000;
    while((USART_GetFlagStatus(USART8, USART_FLAG_TXE) == RESET) && (timeout1 > 0)) timeout1--;

    USART_SendData(USART8, '\n');
    volatile uint32_t timeout2 = 100000;
    while((USART_GetFlagStatus(USART8, USART_FLAG_TXE) == RESET) && (timeout2 > 0)) timeout2--;
}

/*********************************************************************
 * @fn      forward_gesture_to_pc
 * @brief   转发语音/手势触发信号到 USART8 (COM12, Python MCU通信口)
 */
void forward_gesture_to_pc(uint8_t c)
{
    USART_SendData(USART8, c);
    volatile uint32_t timeout1 = 100000;
    while((USART_GetFlagStatus(USART8, USART_FLAG_TXE) == RESET) && (timeout1 > 0)) timeout1--;

    USART_SendData(USART8, '\n');
    volatile uint32_t timeout2 = 100000;
    while((USART_GetFlagStatus(USART8, USART_FLAG_TXE) == RESET) && (timeout2 > 0)) timeout2--;
}

/*********************************************************************
 * @fn      send_voice_feedback
 * @brief   向语音模块发送播报指令 (USART1 PA9, 19200bps)
 */
void send_voice_feedback(uint8_t c)
{
    USART_SendData(USART1, c);
    volatile uint32_t timeout = 100000;
    while((USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) && (timeout > 0)) timeout--;
}
