/*
* bsp_adc.h
 * 描述: ADC 驱动头文件 (DMA 循环模式)
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "main.h" // 包含 HAL 库定义

// ==============================
// 1. 宏定义
// ==============================
// 只有 1 个通道 (电位器 PA0)
#define ADC_CHANNEL_COUNT  1

// 参考电压 (通常为 3.3V)
#define ADC_REF_VOLTAGE    3.3f
// ADC 分辨率 (12-bit = 4096)
#define ADC_RESOLUTION     4095.0f

// ==============================
// 2. 接口函数声明
// ==============================

/**
 * @brief  初始化 ADC 并启动 DMA 循环传输
 * @note   调用后，硬件将在后台自动刷新缓冲区
 */
void BSP_ADC_Init(void);

/**
 * @brief  获取 ADC 原始值 (0 - 4095)
 * @retval 12位原始数值
 */
uint16_t BSP_ADC_GetRaw(void);

/**
 * @brief  获取换算后的电压值 (0.0 - 3.3V)
 * @retval 电压浮点数
 */
float BSP_ADC_GetVoltage(void);

#endif // BSP_ADC_H