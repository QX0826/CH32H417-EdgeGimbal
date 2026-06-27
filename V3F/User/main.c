/********************************** (C) COPYRIGHT ******************************
 * File Name          : main.c
 * Description        : CH32H417 稳像云台控制系统 - V3F 单核版本 (驱动一切)
 *                      职责: 屏幕显示、舵机控制、PID运算、传感器、蓝牙路由
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
#include "uart_router.h"
/* #include "bluetooth.h" */  /* 蓝牙模块含HSEM调用，单核模式下暂时禁用 */
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
    tft180_set_color(RGB565_RED, RGB565_BLACK);
    tft180_show_chinese(10, 5, 16, tft_chinese[CH_HUAN_YING], 2, RGB565_RED);
    tft180_show_chinese(10, 27, 16, tft_chinese[CH_YUN_TAI], 2, RGB565_RED);
    tft180_show_chinese(42, 27, 16, tft_chinese[CH_XI_TONG], 2, RGB565_RED);
    tft180_draw_line(5, 50, 155, 50, RGB565_RED);
    tft180_set_color(RGB565_GREEN, RGB565_BLACK);
    tft180_show_chinese(10, 55, 16, tft_chinese[CH_QI_DONG], 2, RGB565_GREEN);
    tft180_show_chinese(42, 55, 16, tft_chinese[CH_ZHONG], 1, RGB565_GREEN);
}

/**
 * @brief  TFT 更新报警状态行
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
 */
static uint8_t last_cmd_val = 0xFF;
static uint32_t cmd_count = 0;
static uint32_t tick_count = 0;  /* 心跳计数器 */

static void TFT_UpdateDisplay(void)
{
    tick_count++;
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

    /* 第3行: 手势识别数字 */
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

    /* 第7行: 接收诊断数据 (F:人脸坐标, G:手势, V:语音) */
    tft180_set_color(RGB565_RED, RGB565_BLACK);
    sprintf(tft_buf, "F:%02d G:%02d V:%02d", 
            (int)(usart3_rx_count % 100),
            (int)(usart6_rx_count % 100),
            (int)(usart2_rx_count % 100));
    CLEAR_LINE();
    tft180_show_string(0, 96, tft_buf);
}

/* ==================== 蓝牙数据上报 ==================== */
static void BT_UpdateReportData(void)
{
    gimbal_shared.report_mq7   = sensor_data.mq7;
    gimbal_shared.report_mq4   = sensor_data.mq4;
    gimbal_shared.report_mq2   = sensor_data.mq2;
    gimbal_shared.report_flame = sensor_data.flame;
    gimbal_shared.adc_values[0] = sensor_data.mq7;
    gimbal_shared.adc_values[1] = sensor_data.mq4;
    gimbal_shared.adc_values[2] = sensor_data.mq2;
}

static void BT_ApplyThresholds(void)
{
    uint16_t mq7 = (uint16_t)gimbal_shared.threshold_mq7;
    uint16_t mq4 = (uint16_t)gimbal_shared.threshold_mq4;
    uint16_t mq2 = (uint16_t)gimbal_shared.threshold_mq2;

    if(mq7 > 0 && mq4 > 0 && mq2 > 0)
    {
        Alarm_SetThresholds(mq7, mq4, mq2);
    }
}

/* ==================== 主函数 ==================== */

int main(void)
{
    SystemInit();
    SystemAndCoreClockUpdate();
    Delay_Init();
    
    /* 1. 串口初始化 */
    UART_Router_Init();
    Delay_Ms(200);

    /* 2. 外设初始化 */
    Servo_PWM_Init();
    PID_Init_All();
    LED_Init();
    Beep_Init();
    /* Sensor_Init(); */   /* 暂时禁用: ADC校准疑似卡死 */
    Alarm_Init();

    /* 3. 蓝牙禁用 */
    /* Bluetooth_Init(); */

    /* 4. 设置默认共享变量与阈值 */
    gimbal_shared.threshold_mq7 = ALARM_DEFAULT_MQ7;
    gimbal_shared.threshold_mq4 = ALARM_DEFAULT_MQ4;
    gimbal_shared.threshold_mq2 = ALARM_DEFAULT_MQ2;
    gimbal_shared.alarm_enable = 1;
    gimbal_shared.com_digit = 0xFF;  /* 初始无识别数字 */
    gimbal_shared.command = CMD_STOP;
    gimbal_shared.command_ready = 0;
    gimbal_shared.tracking_mode = 0;  /* 强制清除开机随机值 */
    gimbal_shared.face_x = 0;
    gimbal_shared.face_y = 0;
    gimbal_shared.face_ready = 0;

    /* 5. TFT初始化 */
    tft180_set_dir(TFT180_CROSSWISE);
    tft180_init();
    tft180_clear();

    /* 6. 初始位置 */
    Servo_Set_Angle(SERVO_X, 75);
    Servo_Set_Angle(SERVO_Y, 175);
    Servo_Set_Angle(SERVO_Z, 50);

    /* 7. 启动提示 */
    Beep_Beep(100);
    LED1_On();

    /* 主循环 */
    while(1)
    {
        /* 心跳 */
        gimbal_shared.v3f_heartbeat++;

        /* 处理手势与人脸追踪数据 */
        V5F_ProcessSharedData();

        /* 蓝牙阈值暂不应用 */
        BT_ApplyThresholds();

        /* 平滑移动舵机 */
        smooth_move_servos();

        /* 更新状态到全局变量 (供显示) */
        gimbal_shared.servo_angle[0] = current_angle[0];
        gimbal_shared.servo_angle[1] = current_angle[1];
        gimbal_shared.servo_angle[2] = current_angle[2];

        /* 传感器/报警暂时禁用 (Sensor_Init未调用) */
        /* Sensor_ReadAll(); Alarm_Check(...); */

        /* TFT 显示刷新 (每500ms) */
        static uint32_t tft_tick = 0;
        if(++tft_tick >= 50)
        {
            tft_tick = 0;
            TFT_UpdateDisplay();
        }

        /* LED心跳 (每1s) */
        static uint32_t hb_tick = 0;
        if(++hb_tick >= 100)
        {
            hb_tick = 0;
            LED2_Toggle();
        }

        Delay_Ms(10);
    }

    return 0;
}
