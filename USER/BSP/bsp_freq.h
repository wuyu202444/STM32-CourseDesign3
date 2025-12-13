/*
* bsp_freq.h
 * 描述: 频率测量驱动头文件 (TIM4 IC)
 */
#ifndef __BSP_FREQ_H__
#define __BSP_FREQ_H__

#include "main.h"

// 接口定义
void BSP_Freq_Init(void);
uint32_t BSP_Freq_Get(void);

#endif