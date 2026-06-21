/********************************** (C) COPYRIGHT ******************************
 * File Name          : servo.c
 * Description        : 舵机PWM驱动
 *                      参考: ch32h417-main/EVT/EXAM/TIM/PWM_Output/Common/hardware.c
 *                      TIM2 CH1/CH2/CH3 输出50Hz PWM控制3轴舵机
 ******************************************************************************
 */

#include "debug.h"
#include "servo.h"
#include "shared_gimbal.h"
#include "uart_router.h"
#include <stdlib.h>

/* 全局变量 */
int current_angle[SERVO_COUNT] = {70, 175, 50};
int target_angle[SERVO_COUNT]  = {70, 175, 50};

/* 各轴舵机的物理边界约束 */
static const int servo_min[SERVO_COUNT] = {SERVO_X_MIN, SERVO_Y_MIN, SERVO_Z_MIN};
static const int servo_max[SERVO_COUNT] = {SERVO_X_MAX, SERVO_Y_MAX, SERVO_Z_MAX};

/* 平滑移动阈值 */
#define STEP_LARGE_THRESHOLD   20
#define STEP_MEDIUM_THRESHOLD  5

/*********************************************************************
 * @fn      Servo_PWM_Init
 * @brief   初始化TIM2 3通道PWM输出 (50Hz)
 *          参考: ch32h417-main/EVT/EXAM/TIM/PWM_Output/Common/hardware.c
 *
 * @return  none
 */
void Servo_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};

    /* 使能时钟 */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA | RCC_HB2Periph_AFIO, ENABLE);
    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM2, ENABLE);

    /* GPIO复用: PA0/PA1/PA2 → TIM2_CH1/CH2/CH3
     * AF1 已由CH32H417数据手册确认:
     *   PA0: TIM2_CH1_ETR(AF1)
     *   PA1: TIM2_CH2(AF1)
     *   PA2: TIM2_CH3(AF1) */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 定时器基础配置
     * TIM2是16位定时器，ARR最大65535
     * 400MHz / 800 / 1000 = 50Hz
     * pulse范围: 50(0.5ms) ~ 250(2.5ms)，步进精度 10us */
    TIM_TimeBaseStructure.TIM_Period = 1000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 800 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* PWM模式配置
     * 1.5ms中位 = 75 / 1000 * 20ms = 1.5ms */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 75;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);  /* X轴 */
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);  /* Y轴 */
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);  /* Z轴 */

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    /* printf("[Servo] PWM Initialized (50Hz, TIM2 CH1/CH2/CH3)\r\n"); */
}

/*********************************************************************
 * @fn      Servo_Set_Angle
 * @brief   设置舵机角度 (0-180°)
 *          脉宽: 0.5ms(2500) ~ 2.5ms(12500) 基于100000计数值
 *
 * @param   ch - 舵机通道
 *          angle - 目标角度 (0-180)
 *
 * @return  none
 */
void Servo_Set_Angle(Servo_Channel ch, uint16_t angle)
{
    if(ch == SERVO_Y)
    {
        if(angle > SERVO_Y_MAX) angle = SERVO_Y_MAX;
    }
    else if(ch == SERVO_X)
    {
        if(angle > SERVO_X_MAX) angle = SERVO_X_MAX;
    }
    else if(ch == SERVO_Z)
    {
        if(angle > SERVO_Z_MAX) angle = SERVO_Z_MAX;
    }

    /* 角度 → 脉宽: 50(0.5ms) ~ 250(2.5ms) */
    uint32_t pulse = 50 + (uint32_t)angle * 200 / 180;

    switch(ch)
    {
        case SERVO_X: TIM_SetCompare1(TIM2, pulse); break;
        case SERVO_Y: TIM_SetCompare2(TIM2, pulse); break;
        case SERVO_Z: TIM_SetCompare3(TIM2, pulse); break;
        default: break;
    }
}

/*********************************************************************
 * @fn      smooth_move_servos
 * @brief   平滑移动舵机（分段速度控制）
 *          大误差快速追赶，小误差精细调整
 *
 * @return  none
 */
void smooth_move_servos(void)
{
    int i;
    for(i = 0; i < SERVO_COUNT; i++)
    {
        int error = target_angle[i] - current_angle[i];
        int step;

        if(abs(error) > STEP_LARGE_THRESHOLD)
            step = error / 2;       /* 快速追赶 */
        else if(abs(error) > STEP_MEDIUM_THRESHOLD)
            step = error / 3;       /* 中速 */
        else
            step = (error != 0) ? (error > 0 ? 1 : -1) : 0;  /* 微调 */

        current_angle[i] += step;
        current_angle[i] = CONSTRAIN(current_angle[i], servo_min[i], servo_max[i]);
    }

    Servo_Set_Angle(SERVO_X, current_angle[0]);
    Servo_Set_Angle(SERVO_Y, current_angle[1]);
    Servo_Set_Angle(SERVO_Z, current_angle[2]);
}

/*********************************************************************
 * @fn      V5F_ProcessSharedData
 * @brief   处理双核共享数据
 *          参考: ch32h417-main/EVT/EXAM/CPU/HSEM/HSEM_DataSharing
 *
 * @return  none
 */
void V5F_ProcessSharedData(void)
{
    /* 手势命令 */
    if(gimbal_shared.command_ready)
    {
        uint8_t cmd = gimbal_shared.command;
        switch(cmd)
        {
            case CMD_X_PLUS:    
                target_angle[0] += 5; 
                send_voice_feedback('3');
                break;
            case CMD_X_MINUS:   
                target_angle[0] -= 5; 
                send_voice_feedback('4');
                break;
            case CMD_Y_PLUS:    
                target_angle[1] += 5; 
                send_voice_feedback('5');
                break;
            case CMD_Y_MINUS:   
                target_angle[1] -= 5; 
                send_voice_feedback('6');
                break;
            case CMD_TRACK_ON:
                target_angle[0] = 70;
                target_angle[1] = 175;
                target_angle[2] = 50;
                gimbal_shared.tracking_mode = 1;
                send_voice_feedback('9');
                break;
            case CMD_Z_PLUS:    
                target_angle[2] += 5; 
                send_voice_feedback('7');
                break;
            case CMD_Z_MINUS:   
                target_angle[2] -= 5; 
                send_voice_feedback('8');
                break;
            case CMD_STOP:
                gimbal_shared.tracking_mode = 0;
                send_voice_feedback('0');
                break;
            default: break;
        }

        /* 角度约束 */
        int i;
        for(i = 0; i < SERVO_COUNT; i++)
            target_angle[i] = CONSTRAIN(target_angle[i], servo_min[i], servo_max[i]);

        gimbal_shared.command_ready = 0;
    }

    /* 人脸 */
    if(gimbal_shared.face_ready && gimbal_shared.tracking_mode)
    {
        int16_t fx = gimbal_shared.face_x;
        int16_t fy = gimbal_shared.face_y;

        target_angle[0] = CONSTRAIN(fx, SERVO_X_MIN, SERVO_X_MAX);
        target_angle[1] = CONSTRAIN(fy, SERVO_Y_MIN, SERVO_Y_MAX);

        /* Z轴协同：以实际中心位置归一化（X中心=70，Y中心=175，Z初始=50） */
        float norm_x = (fx - 70) / 40.0f;    /* X范围50~110，以70为中心，半幅=40 */
        float norm_y = (fy - 175) / 85.0f;   /* Y范围110~260，以175为中心，半幅=85 */
        int z_comp = (int)(50 + norm_x * 8.0f + norm_y * 5.0f);
        target_angle[2] = CONSTRAIN(z_comp, SERVO_Z_MIN, SERVO_Z_MAX);

        gimbal_shared.face_ready = 0;
    }
}
