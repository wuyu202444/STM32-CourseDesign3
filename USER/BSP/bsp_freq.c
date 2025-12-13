/*
 * bsp_freq.c
 * 描述: 频率测量驱动实现 (TIM4 Channel 2 - PB7)
 */

#include "bsp_freq.h"
#include "tim.h" // 引用 CubeMX 生成的 tim.h

// ==============================
// 私有变量
// ==============================
static volatile uint32_t uwIC1Value = 0;    // 第一次捕获值
static volatile uint32_t uwIC2Value = 0;    // 第二次捕获值
static volatile uint32_t uwDiff = 0;        // 差值
static volatile uint8_t  uhCaptureIndex = 0;// 捕获状态 (0:未捕获, 1:已捕获第一次)
static volatile uint32_t uwFrequency = 0;   // 计算出的频率

// ==============================
// 接口函数实现
// ==============================

/**
 * @brief  初始化频率测量
 */
void BSP_Freq_Init(void)
{
    // 开启 TIM4 通道 2 的输入捕获中断
    // 注意：根据手册 Page 4，引脚是 PB7，对应 TIM4_CH2
    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
}

/**
 * @brief  获取当前频率
 */
uint32_t BSP_Freq_Get(void)
{
    return uwFrequency;
}

// ==============================
// 中断回调函数
// ==============================

/**
 * @brief  定时器输入捕获中断回调
 * @note   此函数会被 HAL_TIM_IRQHandler 调用
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            if (uhCaptureIndex == 0)
            {
                // 1. 获取第一次上升沿的值
                uwIC1Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
                uhCaptureIndex = 1;
            }
            else if (uhCaptureIndex == 1)
            {
                // 2. 获取第二次上升沿的值
                uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

                // 3. 计算差值 (考虑计数器溢出)
                if (uwIC2Value > uwIC1Value)
                {
                    uwDiff = uwIC2Value - uwIC1Value;
                }
                else
                {
                    // 溢出处理 (ARR = 0xFFFF)
                    uwDiff = (0xFFFF - uwIC1Value) + uwIC2Value + 1;
                }

                // 4. 计算频率
                // 计数时钟为 1MHz (Prescaler = 84-1)
                // Freq = 1,000,000 / Diff
                if (uwDiff != 0)
                {
                    uwFrequency = 1000000 / uwDiff;
                }
                else
                {
                    uwFrequency = 0;
                }

                // 5. 重置状态，准备下一次测量
                uhCaptureIndex = 0;
            }
        }
    }
}