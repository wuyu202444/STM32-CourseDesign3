//
// Created by Administrator on 2025/12/12.
//

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

// 1. 传感器数据包 (Queue传输)
// 结构体大小建议 <= 16 Bytes
typedef struct {
    float temp_val;      // 4 Bytes: 温度
    uint32_t freq_hz;    // 4 Bytes: 频率
    uint16_t adc_raw;    // 2 Bytes: 电位器电压 (0-4095)
    uint16_t reserved;   // 2 Bytes: 填充对齐
} SensorData_t;

// 2. 报警事件位 (EventGroup)
#define EVENT_TEMP_HIGH    (1 << 0)  // 温度过高
#define EVENT_ADC_HIGH     (1 << 1)  // 电压异常
#define EVENT_FREQ_ERR     (1 << 2)  // 频率异常

// 3. 全局句柄声明 (对应 CubeMX 生成的变量名)
extern osMessageQueueId_t myQueue_SensorDataHandle;
extern osMessageQueueId_t myQueue_KeyEvtHandle;
extern osEventFlagsId_t myEventGroup_AlarmHandle;

#endif