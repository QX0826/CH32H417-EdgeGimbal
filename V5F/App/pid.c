/********************************** (C) COPYRIGHT ******************************
 * File Name          : pid.c
 * Description        : 增量式PID控制器
 *                      从CH32V307项目直接移植，算法逻辑不变
 ******************************************************************************
 */

#include "debug.h"
#include "pid.h"
#include <math.h>

PID_Controller pid_x, pid_y, pid_z;

/*********************************************************************
 * @fn      PID_Init_All
 * @brief   初始化三轴PID参数
 *
 * @return  none
 */
void PID_Init_All(void)
{
    /* X轴（快速响应） */
    pid_x.Kp = 1.2f;  pid_x.Ki = 0.04f;  pid_x.Kd = 0.1f;
    pid_x.integral = 0;  pid_x.prev_error = 0;
    pid_x.integral_limit = 30.0f;  pid_x.output_limit = 10.0f;

    /* Y轴（平稳移动） */
    pid_y.Kp = 0.8f;  pid_y.Ki = 0.03f;  pid_y.Kd = 0.2f;
    pid_y.integral = 0;  pid_y.prev_error = 0;
    pid_y.integral_limit = 40.0f;  pid_y.output_limit = 8.0f;

    /* Z轴（精确控制） */
    pid_z.Kp = 0.8f;  pid_z.Ki = 0.05f;  pid_z.Kd = 0.2f;
    pid_z.integral = 0;  pid_z.prev_error = 0;
    pid_z.integral_limit = 20.0f;  pid_z.output_limit = 5.0f;

    printf("[PID] Initialized: X(%.2f,%.2f,%.2f) Y(%.2f,%.2f,%.2f) Z(%.2f,%.2f,%.2f)\r\n",
           pid_x.Kp, pid_x.Ki, pid_x.Kd,
           pid_y.Kp, pid_y.Ki, pid_y.Kd,
           pid_z.Kp, pid_z.Ki, pid_z.Kd);
}

/*********************************************************************
 * @fn      PID_Calculate
 * @brief   PID计算（增量式）
 *          |error| < 10° 时累积积分（消除稳态误差）
 *          |error| >= 10° 时清零积分（防止积分饱和）
 *
 * @param   pid - PID控制器指针
 *          error - 当前误差
 *
 * @return  PID输出值
 */
float PID_Calculate(PID_Controller* pid, float error)
{
    /* 比例项 */
    float P = pid->Kp * error;

    /* 积分项（带限幅和死区） */
    if(fabs(error) < 10.0f)
    {
        pid->integral += error;
        if(pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
        if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    }
    else
    {
        pid->integral = 0;
    }
    float I = pid->Ki * pid->integral;

    /* 微分项 */
    float D = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;

    /* 输出限幅 */
    float output = P + I + D;
    if(output > pid->output_limit)  output = pid->output_limit;
    if(output < -pid->output_limit) output = -pid->output_limit;

    return output;
}
