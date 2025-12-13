#ifndef __BSP_BT_H
#define __BSP_BT_H

#include "main.h" // 包含 HAL 库定义
#include "cmsis_os2.h"
#include "app_types.h" // 包含 SensorData_t
#include "usart.h"

// 请确保这里是你实际使用的串口句柄，比如 &huart6
#define BT_UART_HANDLE  &huart6

void BSP_BT_Init(void);
void BSP_BT_ProcessTask(SensorData_t *pData);

// 这是一个极其重要的回调，必须确保被 HAL 库调用
void BSP_BT_RxCallback(UART_HandleTypeDef *huart);

#endif