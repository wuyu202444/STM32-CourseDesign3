/* bsp_key.h */
#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

#include "main.h"

// 定义按键 ID
#define KEY_NONE  0
#define KEY_0     1
#define KEY_1     2
#define KEY_2     3
#define KEY_3     4

// 如果 main.h 中没有自动生成这些宏，请根据原理图手动补充
// 根据手册：KEY0 -> PA8
#ifndef KEY0_Pin
#define KEY0_Pin GPIO_PIN_8
#define KEY0_GPIO_Port GPIOA
#endif

#endif // __BSP_KEY_H__