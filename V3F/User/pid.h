/********************************** (C) COPYRIGHT ******************************
 * File Name          : pid.h
 * Description        : PID控制器头文件
 ******************************************************************************
 */

#ifndef __PID_H
#define __PID_H

#include "gimbal_types.h"

extern PID_Controller pid_x, pid_y, pid_z;

void PID_Init_All(void);
float PID_Calculate(PID_Controller* pid, float error);

#endif /* __PID_H */
