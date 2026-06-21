/********************************** (C) COPYRIGHT ******************************
 * File Name          : alarm.h
 * Description        : 传感器阈值报警模块 (CH32H417)
 *                      超限蜂鸣器 + LED闪烁报警
 ******************************************************************************
 */
#ifndef __ALARM_H
#define __ALARM_H

#include "ch32h417.h"

/* 默认报警阈值 (ADC 12bit: 0~4095) */
#define ALARM_DEFAULT_MQ7      2000    /* CO浓度阈值 */
#define ALARM_DEFAULT_MQ4      1800    /* 天然气阈值 */
#define ALARM_DEFAULT_MQ2      1800    /* 烟雾阈值 */

/* 报警状态 */
typedef struct {
    uint8_t  mq7_alarm;     /* CO超标 */
    uint8_t  mq4_alarm;     /* 天然气超标 */
    uint8_t  mq2_alarm;     /* 烟雾超标 */
    uint8_t  flame_alarm;   /* 火焰检测 */
    uint8_t  any_alarm;     /* 任一报警 */
} AlarmState_t;

extern AlarmState_t alarm_state;

void Alarm_Init(void);
void Alarm_Check(uint16_t mq7, uint16_t mq4, uint16_t mq2, uint8_t flame);
void Alarm_SetThresholds(uint16_t mq7, uint16_t mq4, uint16_t mq2);

#endif /* __ALARM_H */
