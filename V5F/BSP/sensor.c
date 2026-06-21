/********************************** (C) COPYRIGHT ******************************
 * File Name          : sensor.c
 * Description        : 传感器采集模块 (CH32H417)
 *                      从 CH32V307 adc.c 迁移，使用WCH标准库
 * 注意：PA0/PA1/PA2 已被TIM2舵机占用！
 *       原代码ADC用PA0/PA1/PA2，现在改到PB0/PB1/PC0
 ******************************************************************************
 */
#include "sensor.h"
#include "debug.h"

SensorData_t sensor_data = {0};

/*********************************************************************
 * @fn      Sensor_Init
 * @brief   初始化ADC和数字IO
 */
void Sensor_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    /* 使能时钟 */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_ADC1 | RCC_HB2Periph_GPIOB | RCC_HB2Periph_GPIOC | RCC_HB2Periph_GPIOA, ENABLE);

    /* ADC时钟配置 */
    RCC_ADCCLKConfig(RCC_ADCCLKSource_HCLK);
    RCC_ADCHCLKCLKAsSourceConfig(RCC_PPRE2_DIV4, RCC_HCLK_ADCPRE_DIV8);

    /* ADC引脚配置为模拟输入: PB0, PB1, PC0 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 火焰传感器: PA3 数字输入 */
    GPIO_InitStructure.GPIO_Pin = FLAME_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(FLAME_PORT, &GPIO_InitStructure);

    /* ADC配置 */
    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    ADC_BufferCmd(ADC1, ENABLE);

    /* ADC校准 */
    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));

    printf("[Sensor] ADC Init OK (PB0/PB1/PC0)\r\n");
}

/*********************************************************************
 * @fn      Sensor_ReadADC
 * @brief   读取单个ADC通道
 */
uint16_t Sensor_ReadADC(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_CyclesMode5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    return (uint16_t)ADC_GetConversionValue(ADC1);
}

/*********************************************************************
 * @fn      Sensor_ReadAll
 * @brief   读取所有传感器数据
 */
void Sensor_ReadAll(void)
{
    sensor_data.mq7 = Sensor_ReadADC(MQ7_ADC_CH);
    Delay_Ms(5);
    sensor_data.mq4 = Sensor_ReadADC(MQ4_ADC_CH);
    Delay_Ms(5);
    sensor_data.mq2 = Sensor_ReadADC(MQ2_ADC_CH);
    Delay_Ms(5);
    sensor_data.flame = GPIO_ReadInputDataBit(FLAME_PORT, FLAME_PIN);
}
