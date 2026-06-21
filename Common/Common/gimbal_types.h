/********************************** (C) COPYRIGHT ******************************
 * File Name          : gimbal_types.h
 * Description        : 通用类型定义
 ******************************************************************************
 */

#ifndef __GIMBAL_TYPES_H
#define __GIMBAL_TYPES_H

#include "ch32h417.h"

/* 舵机通道 */
typedef enum {
    SERVO_X = 0,
    SERVO_Y,
    SERVO_Z,
    SERVO_COUNT
} Servo_Channel;

/* PID控制器结构体 */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
} PID_Controller;

/* 角度约束 */
#define CONSTRAIN(val, min, max) \
    ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

/* 舵机参数 */
#define SERVO_X_MIN       0
#define SERVO_X_MAX       180
#define SERVO_Y_MIN       110
#define SERVO_Y_MAX       260
#define SERVO_Z_MIN       30
#define SERVO_Z_MAX       60

#define SERVO_ANGLE_MIN   0
#define SERVO_ANGLE_MAX   180
#define SERVO_ANGLE_INIT  90

/* 控制周期 (ms) */
#define GIMBAL_CTRL_PERIOD_MS  20

#endif /* __GIMBAL_TYPES_H */
