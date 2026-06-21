/********************************** (C) COPYRIGHT ******************************
 * File Name          : servo.h
 * Description        : 舵机PWM驱动头文件
 ******************************************************************************
 */

#ifndef __SERVO_H
#define __SERVO_H

#include "ch32h417.h"
#include "gimbal_types.h"

/* 舵机PWM初始化 (TIM2 CH1/CH2/CH3, 50Hz) */
void Servo_PWM_Init(void);

/* 设置舵机角度 (0-180°) */
void Servo_Set_Angle(Servo_Channel ch, uint16_t angle);

/* 平滑移动舵机 */
void smooth_move_servos(void);

/* 处理双核共享数据 */
void V5F_ProcessSharedData(void);

/* 全局变量 */
extern int current_angle[SERVO_COUNT];
extern int target_angle[SERVO_COUNT];

#endif /* __SERVO_H */
