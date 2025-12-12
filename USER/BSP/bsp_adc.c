/*
* bsp_adc.c
 * 描述: ADC 驱动实现 (DMA Zero-Copy 模式)
 */

#include "bsp_adc.h"
#include "adc.h"      // 引用 CubeMX 生成的 adc.c 中的 hadc1

// ==============================
// 1. 私有变量
// ==============================
// DMA 目标缓冲区
// 因为只有 1 个通道，数组长度为 1。
// 如果开启了 Memory Increment，定义数组比定义单个变量更安全，也方便以后扩展。
static uint16_t adc_buffer[ADC_CHANNEL_COUNT] = {0};

// ==============================
// 2. 函数实现
// ==============================

/**
 * @brief  初始化 ADC 并启动 DMA
 */
void BSP_ADC_Init(void)
{
    // 启动 ADC 的 DMA 模式
    // 参数2: 目标内存地址 (强制转换为 uint32_t*)
    // 参数3: 传输的数据长度 (这里是 1 个 Half-Word)
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNEL_COUNT);
}

/**
 * @brief  获取 ADC 原始值
 * @note   直接从内存读取，无 HAL 库开销，极快
 */
uint16_t BSP_ADC_GetRaw(void)
{
    return adc_buffer[0];
}

/**
 * @brief  获取电压值
 */
float BSP_ADC_GetVoltage(void)
{
    uint16_t raw = adc_buffer[0];

    // 计算公式: Voltage = Raw * 3.3 / 4095
    return (float)raw * ADC_REF_VOLTAGE / ADC_RESOLUTION;
}