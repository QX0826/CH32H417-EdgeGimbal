/********************************** (C) COPYRIGHT ******************************
 * File Name          : alarm.c
 * Description        : 传感器阈值报警模块 (CH32H417)
 *                      超限蜂鸣器 + LED闪烁报警
 ******************************************************************************
 */
#include "alarm.h"
#include "beep.h"
#include "led.h"

AlarmState_t alarm_state = {0};

/* 当前阈值 */
static uint16_t threshold_mq7 = ALARM_DEFAULT_MQ7;
static uint16_t threshold_mq4 = ALARM_DEFAULT_MQ4;
static uint16_t threshold_mq2 = ALARM_DEFAULT_MQ2;

/**
 * @brief  初始化报警模块
 */
void Alarm_Init(void)
{
    alarm_state.mq7_alarm    = 0;
    alarm_state.mq4_alarm    = 0;
    alarm_state.mq2_alarm    = 0;
    alarm_state.flame_alarm  = 0;
    alarm_state.any_alarm    = 0;
}

/**
 * @brief  设置报警阈值
 */
void Alarm_SetThresholds(uint16_t mq7, uint16_t mq4, uint16_t mq2)
{
    threshold_mq7 = mq7;
    threshold_mq4 = mq4;
    threshold_mq2 = mq2;
}

/**
 * @brief  检查传感器数据并触发/解除报警
 * @param  mq7    CO传感器ADC值
 * @param  mq4    天然气传感器ADC值
 * @param  mq2    烟雾传感器ADC值
 * @param  flame  火焰检测 0=有火 1=无火
 */
void Alarm_Check(uint16_t mq7, uint16_t mq4, uint16_t mq2, uint8_t flame)
{
    /* 判断各传感器是否超标 */
    alarm_state.mq7_alarm   = (mq7 > threshold_mq7) ? 1 : 0;
    alarm_state.mq4_alarm   = (mq4 > threshold_mq4) ? 1 : 0;
    alarm_state.mq2_alarm   = (mq2 > threshold_mq2) ? 1 : 0;
    alarm_state.flame_alarm = (flame == 0) ? 1 : 0;  /* 0=有火 */

    alarm_state.any_alarm = alarm_state.mq7_alarm ||
                            alarm_state.mq4_alarm ||
                            alarm_state.mq2_alarm ||
                            alarm_state.flame_alarm;

    /* 报警动作 */
    if(alarm_state.any_alarm)
    {
        /* 蜂鸣器: 间隔响 */
        Beep_On();
        /* LED2 快闪指示 */
        LED2_On();
    }
    else
    {
        Beep_Off();
        LED2_Off();
    }
}
