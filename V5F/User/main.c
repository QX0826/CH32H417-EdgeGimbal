/********************************** (C) COPYRIGHT ******************************
 * File Name          : main.c
 * Description        : CH32H417 稳像云台控制系统 - V5F 主核
 *                      职责: 舵机控制、PID运算、传感器采集、TFT显示、报警、蓝牙上报
 ******************************************************************************
 */

#include "debug.h"
#include "shared_gimbal.h"
#include "gimbal_types.h"
#include "servo.h"
#include "pid.h"
#include "led.h"
#include "beep.h"
#include "sensor.h"
#include "tft180.h"
#include "tft_chinese.h"
#include "alarm.h"
#include <stdio.h>
#include <string.h>

/* ==================== TFT 显示 ==================== */
static char tft_buf[22];  /* 横屏 160/8 = 20字符 + 余量 */

/* sprintf后补空格到20字符，覆盖右侧残影 */
#define CLEAR_LINE() do { \
    int _i; \
    for(_i = strlen(tft_buf); _i < 20; _i++) \
        tft_buf[_i] = ' '; \
    tft_buf[20] = '\0'; \
} while(0)

/**
 * @brief  TFT 启动画面 (中文标题)
 */
static void TFT_ShowSplash(void)
{
    tft180_clear();
    /* "欢迎" 中文 */
    tft180_set_color(RGB565_RED, RGB565_BLACK);
    tft180_show_chinese(10, 5, 16, tft_chinese[CH_HUAN_YING], 2, RGB565_RED);
    /* "云台系统" */
    tft180_show_chinese(10, 27, 16, tft_chinese[CH_YUN_TAI], 2, RGB565_RED);
    tft180_show_chinese(42, 27, 16, tft_chinese[CH_XI_TONG], 2, RGB565_RED);
    /* 分隔线 */
    tft180_draw_line(5, 50, 155, 50, RGB565_RED);
    /* "启动中" */
    tft180_set_color(RGB565_GREEN, RGB565_BLACK);
    tft180_show_chinese(10, 55, 16, tft_chinese[CH_QI_DONG], 2, RGB565_GREEN);
    tft180_show_chinese(42, 55, 16, tft_chinese[CH_ZHONG], 1, RGB565_GREEN);
}

/**
 * @brief  TFT 更新报警状态行
 * @param  y  Y坐标
 * @param  label  中文标签 (字模索引)
 * @param  label_count  中文字符数
 * @param  value  当前值
 * @param  threshold  阈值
 * @param  alarmed  是否报警
 */
static void TFT_ShowAlarmRow(uint16_t y, uint8_t label, uint8_t label_count,
                              uint16_t value, uint16_t threshold, uint8_t alarmed)
{
    if(alarmed)
        tft180_set_color(RGB565_RED, RGB565_BLACK);
    else
        tft180_set_color(RGB565_WHITE, RGB565_BLACK);

    tft180_show_chinese(0, y, 16, tft_chinese[label], label_count,
                        alarmed ? RGB565_RED : RGB565_WHITE);
    sprintf(tft_buf, ":%4d/%4d", value, threshold);
    tft180_show_string(label_count * 16, y, tft_buf);
}

/**
 * @brief  TFT 显示系统状态 (每500ms刷新)
 *         纯ASCII显示，无中文
 *         显示: 手势识别、舵机角度、命令状态
 */
static uint8_t last_cmd_val = 0xFF;
static uint32_t cmd_count = 0;
static uint32_t tick_count = 0;  /* 心跳计数器 */

static void TFT_UpdateDisplay(void)
{
    tick_count++;  /* 每次刷新+1，验证主循环在跑 */
    uint8_t cmd = gimbal_shared.command;
    if(cmd <= 7 && cmd != last_cmd_val)
    {
        cmd_count++;
        last_cmd_val = cmd;
    }
    uint8_t digit = gimbal_shared.com_digit;

    /* 清除缓冲区 */
    memset(tft_buf, ' ', 20);
    tft_buf[20] = '\0';

    /* 第1行: 追踪模式 + 命令计数 */
    tft180_set_color(RGB565_CYAN, RGB565_BLACK);
    if(gimbal_shared.tracking_mode)
        sprintf(tft_buf, "Track:ON  N:%lu", (unsigned long)cmd_count);
    else
        sprintf(tft_buf, "Track:OFF N:%lu", (unsigned long)cmd_count);
    CLEAR_LINE();
    tft180_show_string(0, 0, tft_buf);

    /* 第2行: 舵机角度 */
    tft180_set_color(RGB565_GREEN, RGB565_BLACK);
    sprintf(tft_buf, "X:%3d Y:%3d Z:%3d",
            (int)current_angle[0],
            (int)current_angle[1],
            (int)current_angle[2]);
    CLEAR_LINE();
    tft180_show_string(0, 16, tft_buf);

    /* 第3行: 手势识别数字 (大字, 黄色) */
    tft180_set_color(RGB565_YELLOW, RGB565_BLACK);
    if(digit <= 9)
        sprintf(tft_buf, "Hand: %d", (int)digit);
    else
        sprintf(tft_buf, "Hand: -");
    CLEAR_LINE();
    tft180_show_string(0, 32, tft_buf);

    /* 第4行: 命令名 */
    const char *cmd_name[] = {"STOP", "X+", "X-", "Y+", "Y-", "TRK", "Z+", "Z-"};
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    if(cmd <= 7)
        sprintf(tft_buf, "CMD: %s", cmd_name[cmd]);
    else
        sprintf(tft_buf, "CMD: ---");
    CLEAR_LINE();
    tft180_show_string(0, 48, tft_buf);

    /* 第5行: 人脸坐标 */
    tft180_set_color(RGB565_MAGENTA, RGB565_BLACK);
    sprintf(tft_buf, "Face:%4d %4d",
            (int)gimbal_shared.face_x,
            (int)gimbal_shared.face_y);
    CLEAR_LINE();
    tft180_show_string(0, 64, tft_buf);

    /* 第6行: 目标角度 */
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    sprintf(tft_buf, "Tgt:%3d %3d %3d",
            (int)target_angle[0],
            (int)target_angle[1],
            (int)target_angle[2]);
    CLEAR_LINE();
    tft180_show_string(0, 80, tft_buf);

    /* 第7行: 手势名称映射 */
    tft180_set_color(RGB565_CYAN, RGB565_BLACK);
    const char *gesture_name[] = {"STOP","X+","X-","Y+","Y-","TRK","Z+","Z-"};
    if(digit <= 7)
        sprintf(tft_buf, "Gesture: %s", gesture_name[digit]);
    else
        sprintf(tft_buf, "Gesture: -");
    CLEAR_LINE();
    tft180_show_string(0, 96, tft_buf);

    /* 第8行: V3F心跳 + V5F心跳 */
    tft180_set_color(RGB565_RED, RGB565_BLACK);
    sprintf(tft_buf, "V3:%lu V5:%lu", (unsigned long)gimbal_shared.v3f_heartbeat, (unsigned long)tick_count);
    CLEAR_LINE();
    tft180_show_string(0, 112, tft_buf);
}

/* ==================== 蓝牙数据上报 ==================== */

/**
 * @brief  将传感器数据写入共享内存 (供V3F蓝牙上报)
 */
static void BT_UpdateReportData(void)
{
    /* 直接写共享内存 */
    gimbal_shared.report_mq7   = sensor_data.mq7;
    gimbal_shared.report_mq4   = sensor_data.mq4;
    gimbal_shared.report_mq2   = sensor_data.mq2;
    gimbal_shared.report_flame = sensor_data.flame;
    gimbal_shared.adc_values[0] = sensor_data.mq7;
    gimbal_shared.adc_values[1] = sensor_data.mq4;
    gimbal_shared.adc_values[2] = sensor_data.mq2;
}

/**
 * @brief  蓝牙远程调参: 从共享内存读取阈值并应用
 */
static void BT_ApplyThresholds(void)
{
    uint16_t mq7 = (uint16_t)gimbal_shared.threshold_mq7;
    uint16_t mq4 = (uint16_t)gimbal_shared.threshold_mq4;
    uint16_t mq2 = (uint16_t)gimbal_shared.threshold_mq2;

    /* 阈值非零才应用 */
    if(mq7 > 0 && mq4 > 0 && mq2 > 0)
    {
        Alarm_SetThresholds(mq7, mq4, mq2);
    }
}

/* ==================== 主函数 ==================== */

int main(void)
{
    SystemAndCoreClockUpdate();
    Delay_Init();
    /* V5F 不初始化串口，避免与V3F冲突 */
    /* 所有输出通过TFT180屏幕显示 */

#if (Run_Core == Run_Core_V3FandV5F)
    /* 不等待V3F，直接启动 */
    Delay_Ms(200);  /* 给V3F一点启动时间 */

    /* 初始化外设 */
    Servo_PWM_Init();
    PID_Init_All();
    LED_Init();
    Beep_Init();
    Sensor_Init();
    Alarm_Init();

    /* 设置默认报警阈值 */
    /* 强制设置默认报警阈值 */
    gimbal_shared.threshold_mq7 = ALARM_DEFAULT_MQ7;
    gimbal_shared.threshold_mq4 = ALARM_DEFAULT_MQ4;
    gimbal_shared.threshold_mq2 = ALARM_DEFAULT_MQ2;
    gimbal_shared.alarm_enable = 1;
    gimbal_shared.com_digit = 0xFF;  /* 初始无识别数字 */
    gimbal_shared.command = CMD_STOP;
    gimbal_shared.command_ready = 0;

    /* 初始化 TFT180 显示屏 */
    tft180_set_dir(TFT180_CROSSWISE);  /* 横屏 160x128 */
    tft180_init();
    TFT_ShowSplash();
    Delay_Ms(800);
    tft180_clear();
    /* 不使用printf，避免串口冲突 */

    /* 初始位置 */
    Servo_Set_Angle(SERVO_X, 75);
    Servo_Set_Angle(SERVO_Y, 175);
    Servo_Set_Angle(SERVO_Z, 50);

    /* 启动提示 */
    Beep_Beep(100);
    LED1_On();

    /* 主循环 */
    while(1)
    {
        /* 处理双核共享数据 */
        V5F_ProcessSharedData();

        /* 蓝牙远程调参: 从共享内存读取阈值 */
        BT_ApplyThresholds();

        /* 平滑移动舵机 */
        smooth_move_servos();

        /* 更新状态到共享内存 (不用HSEM锁，直接写) */
        gimbal_shared.servo_angle[0] = current_angle[0];
        gimbal_shared.servo_angle[1] = current_angle[1];
        gimbal_shared.servo_angle[2] = current_angle[2];

        /* 传感器采集 (每100ms) */
        static uint32_t sensor_tick = 0;
        if(++sensor_tick >= 5)
        {
            sensor_tick = 0;
            Sensor_ReadAll();

            /* 报警检查 */
            if(gimbal_shared.alarm_enable)
            {
                Alarm_Check(sensor_data.mq7, sensor_data.mq4,
                            sensor_data.mq2, sensor_data.flame);
            }

            /* 蓝牙上报数据更新 */
            BT_UpdateReportData();
        }

        /* TFT 显示刷新 (每500ms) */
        static uint32_t tft_tick = 0;
        if(++tft_tick >= 25)
        {
            tft_tick = 0;
            TFT_UpdateDisplay();
        }

        /* LED心跳 */
        static uint32_t hb_tick = 0;
        if(++hb_tick >= 50)  /* 1s */
        {
            hb_tick = 0;
            LED2_Toggle();
        }

        Delay_Ms(GIMBAL_CTRL_PERIOD_MS);
    }

#elif (Run_Core == Run_Core_V5F)
    /* 单核模式 (调试用) */
    Servo_PWM_Init();
    PID_Init_All();
    while(1)
    {
        Delay_Ms(1000);
    }
#endif

    return 0;
}
