/********************************** (C) COPYRIGHT ******************************
 * File Name          : shared_gimbal.h
 * Description        : 双核共享数据结构定义
 *                      参考: ch32h417-main/EVT/EXAM/CPU/HSEM/HSEM_DataSharing
 ******************************************************************************
 */

#ifndef __SHARED_GIMBAL_H
#define __SHARED_GIMBAL_H

#include "ch32h417.h"

/* 双核共享数据结构 */
typedef struct {
    /* 命令通道 (V3F → V5F) */
    volatile uint8_t  command;          /* 手势命令 (1-7, 0=停止) */
    volatile uint8_t  command_ready;    /* 命令就绪标志 */

    /* 人脸坐标通道 (V3F → V5F) */
    volatile int16_t  face_x;           /* 人脸X角度 */
    volatile int16_t  face_y;           /* 人脸Y角度 */
    volatile uint8_t  face_ready;       /* 坐标就绪标志 */
    volatile uint8_t  tracking_mode;    /* 追踪模式开关 */

    /* 状态反馈 (V5F → V3F) */
    volatile int16_t  servo_angle[3];   /* 当前舵机角度 X/Y/Z */
    volatile uint16_t adc_values[4];    /* ADC采集值 */
    volatile uint8_t  system_mode;      /* 当前工作模式 */

    /* 报警阈值 (蓝牙远程可调) */
    volatile uint16_t threshold_mq7;    /* CO报警阈值 */
    volatile uint16_t threshold_mq4;    /* 天然气报警阈值 */
    volatile uint16_t threshold_mq2;    /* 烟雾报警阈值 */
    volatile uint8_t  alarm_enable;     /* 报警使能 1=开 */
    volatile uint8_t  alarm_active;     /* 当前是否有报警 */

    /* COM口识别数字 (V3F->V5F) */
    volatile uint8_t  com_digit;        /* 最近一次COM口接收的识别数字 0-9, 0xFF=无效 */

    /* V3F心跳 (V3F每10ms自增, V5F读取显示, 确认V3F活着) */
    volatile uint32_t v3f_heartbeat;

    /* 蓝牙数据上报 (V5F写 → V3F读 → 蓝牙发送) */
    volatile uint16_t report_mq7;       /* 上报CO值 */
    volatile uint16_t report_mq4;       /* 上报天然气值 */
    volatile uint16_t report_mq2;       /* 上报烟雾值 */
    volatile uint8_t  report_flame;     /* 上报火焰 */
} GimbalSharedData_t;

/* 共享变量声明 (定义在shared_gimbal.c中, 放在.shared_data段) */
extern volatile GimbalSharedData_t gimbal_shared;

/* HSEM ID 定义 */
#define GIMBAL_HSEM_ID    HSEM_ID0

/* 命令编码 */
#define CMD_STOP          0
#define CMD_X_PLUS        1
#define CMD_X_MINUS       2
#define CMD_Y_PLUS        3
#define CMD_Y_MINUS       4
#define CMD_TRACK_ON      5
#define CMD_Z_PLUS        6
#define CMD_Z_MINUS       7

#endif /* __SHARED_GIMBAL_H */
