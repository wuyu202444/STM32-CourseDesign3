/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "iwdg.h"
#include "stdio.h"

#include "bsp_oled.h"
#include "bsp_seg.h"
#include "bsp_lm75.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_freq.h"

#include "app_types.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALARM_TEMP_HIGH   (1 << 0)  // 温度过高
#define ALARM_VOLT_HIGH   (1 << 1)  // 电压过高
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 全局传感器数据副本 (公告板)
// SensorTask 写，LogicTask 读
SensorData_t g_LatestSensorData = {0};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for LogicTask */
osThreadId_t LogicTaskHandle;
const osThreadAttr_t LogicTask_attributes = {
  .name = "LogicTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for AlarmTask */
osThreadId_t AlarmTaskHandle;
const osThreadAttr_t AlarmTask_attributes = {
  .name = "AlarmTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for KeepAlive */
osThreadId_t KeepAliveHandle;
const osThreadAttr_t KeepAlive_attributes = {
  .name = "KeepAlive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for q_SensorData */
osMessageQueueId_t q_SensorDataHandle;
const osMessageQueueAttr_t q_SensorData_attributes = {
  .name = "q_SensorData"
};
/* Definitions for q_KeyEvt */
osMessageQueueId_t q_KeyEvtHandle;
const osMessageQueueAttr_t q_KeyEvt_attributes = {
  .name = "q_KeyEvt"
};
/* Definitions for m_I2C */
osMutexId_t m_I2CHandle;
const osMutexAttr_t m_I2C_attributes = {
  .name = "m_I2C"
};
/* Definitions for evt_Alarm */
osEventFlagsId_t evt_AlarmHandle;
const osEventFlagsAttr_t evt_Alarm_attributes = {
  .name = "evt_Alarm"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartSensorTask(void *argument);
void StartLogicTask(void *argument);
void StartDisplayTask(void *argument);
void StartAlarmTask(void *argument);
void StartKeepAlive(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of m_I2C */
  m_I2CHandle = osMutexNew(&m_I2C_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of q_SensorData */
  q_SensorDataHandle = osMessageQueueNew (4, 16, &q_SensorData_attributes);

  /* creation of q_KeyEvt */
  q_KeyEvtHandle = osMessageQueueNew (4, sizeof(uint16_t), &q_KeyEvt_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of LogicTask */
  LogicTaskHandle = osThreadNew(StartLogicTask, NULL, &LogicTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of AlarmTask */
  AlarmTaskHandle = osThreadNew(StartAlarmTask, NULL, &AlarmTask_attributes);

  /* creation of KeepAlive */
  KeepAliveHandle = osThreadNew(StartKeepAlive, NULL, &KeepAlive_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of evt_Alarm */
  evt_AlarmHandle = osEventFlagsNew(&evt_Alarm_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  OLED_Init();
  OLED_Clear();
  OLED_Refresh();

  printf("System Starting...\r\n");

  // 1. 初始化传感器
  // 1.1 LM75 (I2C)
  if (BSP_LM75_Init() == 0) {
    printf("[OK] LM75 Init\r\n");
  } else {
    printf("[ERR] LM75 Init Failed\r\n");
  }
  // 1.2 ADC (DMA)
  BSP_ADC_Init();
  printf("[OK] ADC DMA Started\r\n");
  // [新增] 初始化频率测量
  BSP_Freq_Init();
  printf("[OK] TIM4 Freq Input Capture Started\r\n");

  printf("System Startup Success!\r\n");

  /* Infinite loop */
  for(;;)
  {
    static uint32_t count = 0;
    BSP_LEDSEG_ShowNum(1, 2);
    if (count == 200)
    {
      HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
      count = 0;
    }
    count++;
    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */

  // 局部变量定义
  SensorData_t current_data;
  osStatus_t status;

  // 任务循环
  for(;;)
  {
    // ===========================
    // 1. 获取数据 (Acquire)
    // ===========================

    // 读取 LM75 (建议加上互斥锁，虽然目前单任务访问不出错，但习惯要好)
    osMutexAcquire(m_I2CHandle, osWaitForever); // [可选优化]
    current_data.temp_celsius = BSP_LM75_ReadTemp();
    osMutexRelease(m_I2CHandle);                // [可选优化]

    // 读取 ADC
    current_data.adc_raw = BSP_ADC_GetRaw();

    // 读取频率 [修改此处]
    current_data.freq_hz = BSP_Freq_Get();

    // >>>>> [新增] 更新全局副本，供 LogicTask 使用 <<<<<
    g_LatestSensorData = current_data;
    // >>>>> [结束] <<<<<

    // ===========================
    // 2. 发送队列 (Queue Send)
    // ===========================

    // 将结构体发送到队列 q_SensorData
    // 参数说明: 句柄, 数据指针, 优先级(0), 超时时间(0 - 不等待)
    status = osMessageQueuePut(q_SensorDataHandle, &current_data, 0, 0);

    // ===========================
    // 3. 调试打印 (Debug Print)
    // ===========================

    // 调试打印 (更新一下格式，把频率也打出来)
    printf("[Sensor] T:%.1f C, ADC:%d, Freq:%lu Hz\r\n",
           current_data.temp_celsius,
           current_data.adc_raw,
           current_data.freq_hz);

    // ===========================
    // 4. 任务调度 (Delay)
    // ===========================

    // 延时 200ms，控制采样率，释放 CPU 给 DisplayTask 和其他任务
    osDelay(1000);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartLogicTask */
/**
* @brief Function implementing the LogicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogicTask */
void StartLogicTask(void *argument)
{
  /* USER CODE BEGIN StartLogicTask */
  /* Infinite loop */
  for(;;)
  {
    // ==========================================
    // 1. 温度报警逻辑 (> 30.0 度)
    // ==========================================
    if (g_LatestSensorData.temp_celsius > 31.0f)
    {
      // 触发：设置标志位
      osEventFlagsSet(evt_AlarmHandle, ALARM_TEMP_HIGH);
    }
    else
    {
      // 恢复：清除标志位
      osEventFlagsClear(evt_AlarmHandle, ALARM_TEMP_HIGH);
    }

    // ==========================================
    // 2. 电压报警逻辑 (> 2.5V, 即 ADC > 3103)
    // ==========================================
    // 3.3V * (1861/4095) ≈ 1.5V
    if (g_LatestSensorData.adc_raw > 1861)
    {
      osEventFlagsSet(evt_AlarmHandle, ALARM_VOLT_HIGH);
    }
    else
    {
      osEventFlagsClear(evt_AlarmHandle, ALARM_VOLT_HIGH);
    }

    // 逻辑处理频率不用太高，10Hz 足够
    osDelay(100);
  }
  /* USER CODE END StartLogicTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  SensorData_t recv_data;
  osStatus_t status;
  char str_buf[32]; // 用于格式化字符串
  /* Infinite loop */
  for(;;)
  {
    // ===========================
    // 1. 尝试从队列获取数据
    // ===========================
    // 参数: 队列句柄, 数据接收缓冲指针, 优先级(NULL), 等待时间(0 - 不等待，直接刷新)
    // 这里的策略是：如果有新数据就更新显示，没数据就保持原样
    status = osMessageQueueGet(q_SensorDataHandle, &recv_data, NULL, 0);
    if (status == osOK)
    {
      // ===========================
      // 2. 如果获取成功，更新 OLED
      // ===========================

      // 显示温度
      // 假设 OLED_ShowString 和 ShowNum 是您 BSP 里的函数，这里用通用逻辑演示
      // 建议使用 sprintf 格式化字符串然后一次性显示，更灵活
      sprintf(str_buf, "T: %.1f C  ", recv_data.temp_celsius);
      OLED_ShowString(0, 0, (uint8_t *)str_buf, 16, 1); // 假设字体大小16

      // 显示 ADC 电压
      // 先转换成电压值: raw * 3.3 / 4095
      float voltage = recv_data.adc_raw * 3.3f / 4095.0f;
      sprintf(str_buf, "V: %.2f V  ", voltage);
      OLED_ShowString(0, 16, (uint8_t *)str_buf, 16, 1); // 第2行

      // 调试打印：确认接收端也收到了
      printf("[Disp] Recv OK! T:%.1f\r\n", recv_data.temp_celsius);
    }
    OLED_Refresh();
    osDelay(50);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartAlarmTask */
/**
* @brief Function implementing the AlarmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAlarmTask */
void StartAlarmTask(void *argument)
{
  /* USER CODE BEGIN StartAlarmTask */

  uint32_t flags;

  BSP_Buzzer_Off();

  /* Infinite loop */
  for(;;)
  {
    // 1. 等待报警事件
    // 如果无报警，这里会阻塞等待 100ms
    // 如果有报警，这里会立即返回
    flags = osEventFlagsWait(evt_AlarmHandle,
                             ALARM_TEMP_HIGH | ALARM_VOLT_HIGH,
                             osFlagsWaitAny | osFlagsNoClear,
                             100);

    if (flags > 0 && flags != osFlagsErrorTimeout)
    {
      // === 报警状态 (必须包含 osDelay) ===

      // 阶段 1: 响
      if (flags & ALARM_TEMP_HIGH) {
        BSP_Buzzer_SetTone(2000, 2); // 高温：高频，大声
      } else {
        BSP_Buzzer_SetTone(1000, 1);  // 电压：低频，小声
      }

      // 【关键点】: 延时 100ms
      // 这段时间 AlarmTask 进入阻塞态，CPU 交给 LogicTask 和 DisplayTask
      osDelay(100);

      // 阶段 2: 停 (制造滴-滴-滴的效果)
      BSP_Buzzer_Off();

      // 【关键点】: 再次延时 100ms
      // 调节这个时间和上面的时间，可以改变报警的节奏 (急促或缓慢)
      osDelay(100);
    }
    else
    {
      // === 无报警状态 ===
      BSP_Buzzer_Off();
      // 这里不需要额外的 delay，因为开头的 osEventFlagsWait 带了 100ms 超时
      // 当没有标志位时，它自动就是阻塞的
    }
  }
  /* USER CODE END StartAlarmTask */
}

/* USER CODE BEGIN Header_StartKeepAlive */
/**
* @brief Function implementing the KeepAlive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeepAlive */
void StartKeepAlive(void *argument)
{
  /* USER CODE BEGIN StartKeepAlive */
  /* Infinite loop */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(1000);
  }
  /* USER CODE END StartKeepAlive */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

