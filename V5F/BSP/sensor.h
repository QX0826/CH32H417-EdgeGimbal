/********************************** (C) COPYRIGHT ******************************
 * File Name          : sensor.h
 * Description        : 传感器采集模块 (CH32H417)
 *                      ADC采集 + 数字IO
 * 注意：PA0/PA1/PA2 已被TIM2舵机占用，ADC需要重新分配引脚！
 *      建议：PB0(MQ7) PB1(MQ4) PC0(MQ2) PC1(备用)
 ******************************************************************************
 */
#ifndef __SENSOR_H
#define __SENSOR_H

#include "ch32h417.h"

/* 传感器ADC引脚 (CH32H417引脚分配，需确认) */
#define MQ7_ADC_CH      ADC_Channel_8    /* PB0 */
#define MQ4_ADC_CH      ADC_Channel_9    /* PB1 */
#define MQ2_ADC_CH      ADC_Channel_10   /* PC0 */

/* 火焰传感器数字IO */
#define FLAME_PORT      GPIOA
#define FLAME_PIN       GPIO_Pin_3

/* 传感器数据结构 */
typedef struct {
    uint16_t mq7;       /* CO浓度 */
    uint16_t mq4;       /* 天然气浓度 */
    uint16_t mq2;       /* 烟雾浓度 */
    uint8_t  flame;     /* 火焰检测 0=有火 1=无火 */
    float    temperature; /* 内部温度 */
} SensorData_t;

extern SensorData_t sensor_data;

void Sensor_Init(void);
void Sensor_ReadAll(void);
uint16_t Sensor_ReadADC(uint8_t channel);

#endif
