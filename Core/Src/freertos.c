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
/* --- Standard Libraries --- */
#include "stdio.h"
#include <stdbool.h>

/* --- Hardware Drivers (BSP) --- */
#include "iwdg.h"
#include "bsp_oled.h"
#include "bsp_seg.h"
#include "bsp_lm75.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_freq.h"
#include "bsp_rgb.h"
#include "bsp_key.h"
#include "bsp_bt.h"
#include "bsp_flash_crc.h"

/* --- Application Types --- */
#include "app_types.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- 报警标志位定义 --- */
#define ALARM_TEMP_HIGH   (1 << 0)  // 温度过高
#define ALARM_VOLT_HIGH   (1 << 1)  // 电压过高

/* --- RGB 呼吸灯参数配置 --- */
// 1. 最大亮度 (0-255): 降低此值可避免太刺眼
#define RGB_BREATH_MAX_BRIGHTNESS  30

// 2. 呼吸步长: 越大呼吸越急促
#define RGB_BREATH_STEP            1

// 3. 呼吸刷新周期 (ms): 建议 20-50ms，保证动画流畅
#define RGB_BREATH_REFRESH_MS      40

/* --- RGB 报警闪烁配置 --- */
// 4. 闪烁周期 (ms): 100ms亮 -> 100ms灭 (5Hz)
#define RGB_BLINK_PERIOD_MS        100

#define BUZZER_ALARM_SOUND    2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* --- 全局共享数据 (公告板) --- */
// SensorTask 负责写，LogicTask/DisplayTask 负责读
SensorData_t g_LatestSensorData = {0};

/* --- 全局控制标志 --- */
// 显示模式: 1=详细模式(默认), 2=大字体模式
volatile uint8_t g_DisplayPage = 1; // <--- 修改此处：初始化为 1

// 报警温度阈值 (LogicTask更新, DisplayTask读取)
volatile float g_AlarmThreshold = 30.0f;
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
/* Definitions for RGBTask */
osThreadId_t RGBTaskHandle;
const osThreadAttr_t RGBTask_attributes = {
  .name = "RGBTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BluetoothTask */
osThreadId_t BluetoothTaskHandle;
const osThreadAttr_t BluetoothTask_attributes = {
  .name = "BluetoothTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
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
/* Definitions for q_RGBCmd */
osMessageQueueId_t q_RGBCmdHandle;
const osMessageQueueAttr_t q_RGBCmd_attributes = {
  .name = "q_RGBCmd"
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
void StartRGBTask(void *argument);
void StartBluetoothTask(void *argument);

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

  /* creation of q_RGBCmd */
  q_RGBCmdHandle = osMessageQueueNew (4, 16, &q_RGBCmd_attributes);

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

  /* creation of RGBTask */
  RGBTaskHandle = osThreadNew(StartRGBTask, NULL, &RGBTask_attributes);

  /* creation of BluetoothTask */
  BluetoothTaskHandle = osThreadNew(StartBluetoothTask, NULL, &BluetoothTask_attributes);

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
  /* --- 1. 系统基础初始化 --- */
  OLED_Init();
  OLED_Clear();
  OLED_Refresh();

  printf("System Starting...\r\n");

  /* --- 2. 传感器初始化 --- */
  // 2.1 LM75 (I2C)
  if (BSP_LM75_Init() == 0) {
    printf("[OK] LM75 Init\r\n");
  } else {
    printf("[ERR] LM75 Init Failed\r\n");
  }

  // 2.2 ADC (DMA)
  BSP_ADC_Init();
  printf("[OK] ADC DMA Started\r\n");

  // 2.3 频率测量 (IC)
  BSP_Freq_Init();
  printf("[OK] TIM4 Freq Input Capture Started\r\n");

  /* --- 3. 外设初始化 --- */
  BSP_RGB_Init();
  printf("[OK] RGB Init\r\n");

  printf("System Startup Success!\r\n");

  /* --- 4. 心跳循环 --- */
  for(;;)
  {
    static uint32_t count = 0;

    // 数码管扫描 (如果是动态扫描，需要高频调用)
    BSP_LEDSEG_ShowNum(0, g_DisplayPage);

    // 心跳灯 LED3
    if (count == 200) // 约 1000ms 翻转一次 (5ms * 200)
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
  SensorData_t current_data;

  /* Infinite loop */
  for(;;)
  {
    /* --- 1. 获取数据 (Acquire) --- */

    // LM75: 使用互斥锁保护 I2C 总线
    osMutexAcquire(m_I2CHandle, osWaitForever);
    current_data.temp_celsius = BSP_LM75_ReadTemp();
    osMutexRelease(m_I2CHandle);

    // ADC: 读取光敏/电位器数据
    current_data.adc_raw = BSP_ADC_GetRaw();

    // Freq: 读取 555 频率
    current_data.freq_hz = BSP_Freq_Get();

    /* --- 2. 更新全局副本 (Publish) --- */
    // 供 LogicTask 或 BluetoothTask 随时读取最新状态
    g_LatestSensorData = current_data;

    /* --- 3. 发送数据队列 (Send Queue) --- */
    // 将数据发送给 DisplayTask 进行显示 (非阻塞)
    osMessageQueuePut(q_SensorDataHandle, &current_data, 0, 0);

    /* --- 4. 调试输出 (Optional) --- */
    // printf("[Sensor] T:%.1f, ADC:%d, F:%lu\r\n",
    //        current_data.temp_celsius, current_data.adc_raw, current_data.freq_hz);

    /* --- 5. 采样周期控制 --- */
    osDelay(1000); // 1Hz 采样率
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
  uint16_t key_event = 0;
  bool is_muted = false;        // 静音标志位
  bool alarm_condition = false; // 报警总状态
  bool last_alarm_status = false; // 历史状态用于检测边沿

  SystemSettings_t sys_settings;

  /* --- 1. 读取 Flash 配置 --- */
  printf("[Logic] Reading Settings...\r\n");
  if (BSP_Settings_Read(&sys_settings) == 0) {
    printf("[Logic] Read OK. Threshold: %.1f C\r\n", sys_settings.temp_threshold);
  } else {
    printf("[Logic] Read Fail. Loading Defaults.\r\n");
    sys_settings.temp_threshold = 30.0f;
    BSP_Settings_Save(&sys_settings);
  }
  // 同步全局阈值
  g_AlarmThreshold = sys_settings.temp_threshold;

  // 上电默认效果
  BSP_RGB_SendCmd(RGB_MODE_BREATHING, 0, 255, 0, 0); // 绿色呼吸

  /* Infinite loop */
  for(;;)
  {
    /* ==========================================
     * 1. 处理按键事件 (Event Processing)
     * ========================================== */
    if (osMessageQueueGet(q_KeyEvtHandle, &key_event, NULL, 0) == osOK)
    {
        switch (key_event)
        {
          case KEY_0: // 消音
            printf("[Logic] Key0 -> Mute Alarm\r\n");
            is_muted = true;
            break;

          case KEY_1: // 切换界面
            printf("[Logic] Key1 -> Switch Page\r\n");
          // <--- 修改开始: 1和2之间循环切换 --->
          if (g_DisplayPage == 1) {
            g_DisplayPage = 2;
          } else {
            g_DisplayPage = 1;
          }
            // 切换时刷新一次屏幕防止残影 (可选，交给 DisplayTask 亦可)
            OLED_Clear();
            OLED_Refresh();
            break;

          case KEY_2: // 阈值 -1
            sys_settings.temp_threshold -= 1.0f;
            g_AlarmThreshold = sys_settings.temp_threshold;
            printf("[Logic] Key2 -> Thres -1: %.1f\r\n", sys_settings.temp_threshold);
            BSP_Settings_Save(&sys_settings);
            break;

          case KEY_3: // 阈值 +1
            sys_settings.temp_threshold += 1.0f;
            g_AlarmThreshold = sys_settings.temp_threshold;
            printf("[Logic] Key3 -> Thres +1: %.1f\r\n", sys_settings.temp_threshold);
            BSP_Settings_Save(&sys_settings);
            break;
        }
    }

    /* ==========================================
     * 2. 检查报警条件 (Alarm Check)
     * ========================================== */
    bool temp_high = (g_LatestSensorData.temp_celsius > sys_settings.temp_threshold);
    bool volt_high = (g_LatestSensorData.adc_raw > 1861); // > ~1.5V

    alarm_condition = temp_high | volt_high;

    /* ==========================================
     * 3. RGB 状态机控制 (State Machine)
     * ========================================== */
    // 仅在状态跳变时发送指令，避免堵塞队列
    if (alarm_condition != last_alarm_status)
    {
      if (alarm_condition) {
        printf("[Logic] ALARM! RGB -> Red Blink\r\n");
        BSP_RGB_SendCmd(RGB_MODE_BLINK, 30, 0, 0, 0);
      } else {
        printf("[Logic] Normal. RGB -> Green Breath\r\n");
        BSP_RGB_SendCmd(RGB_MODE_BREATHING, 0, 255, 0, 0);
      }
      last_alarm_status = alarm_condition;
    }

    /* ==========================================
     * 4. 蜂鸣器控制 (Buzzer Control)
     * ========================================== */
    if (alarm_condition)
    {
        if (is_muted) {
            // 静音状态：清除标志，停止蜂鸣器
            osEventFlagsClear(evt_AlarmHandle, ALARM_TEMP_HIGH | ALARM_VOLT_HIGH);
        } else {
            // 设置对应的报警标志，触发 AlarmTask
            if (temp_high) osEventFlagsSet(evt_AlarmHandle, ALARM_TEMP_HIGH);
            if (volt_high) osEventFlagsSet(evt_AlarmHandle, ALARM_VOLT_HIGH);
        }
    }
    else
    {
        // 恢复正常：清除所有标志
        osEventFlagsClear(evt_AlarmHandle, ALARM_TEMP_HIGH | ALARM_VOLT_HIGH);
        // 如果之前静音了，恢复正常后取消静音，以便下次报警能响
        if (is_muted) is_muted = false;
    }

    osDelay(100); // 逻辑处理周期
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
  char str_buf[32];

  /* Infinite loop */
  for(;;)
  {
    // 等待数据更新
    if (osMessageQueueGet(q_SensorDataHandle, &recv_data, NULL, 0) == osOK)
    {
      if (g_DisplayPage == 2)
      {
        /* --- Page 2: 详细模式 --- */

        // 1. Temp
        sprintf(str_buf, "T: %.1f C   ", recv_data.temp_celsius);
        OLED_ShowString(0, 0, (uint8_t *)str_buf, 16, 1);

        // 2. Voltage (Conversion: Raw * 3.3 / 4095)
        float voltage = recv_data.adc_raw * 3.3f / 4095.0f;
        sprintf(str_buf, "V: %.2f V   ", voltage);
        OLED_ShowString(0, 16, (uint8_t *)str_buf, 16, 1);

        // 3. Frequency (Auto unit kHZ/Hz)
        if(recv_data.freq_hz < 10000)
          sprintf(str_buf, "F: %lu Hz   ", recv_data.freq_hz);
        else
          sprintf(str_buf, "F: %.1f kHz ", recv_data.freq_hz / 1000.0f);
        OLED_ShowString(0, 32, (uint8_t *)str_buf, 16, 1);
      }
      else
      {
        /* --- Page 1: 简略模式 --- */
        OLED_ShowString(18, 0, "Temp Monitor", 16, 1);

        // 大字体温度显示
        sprintf(str_buf, "%.1f C", recv_data.temp_celsius);
        OLED_ShowString(32, 18, (uint8_t *)str_buf, 24, 1);
      }

      /* --- 状态栏 (Common) --- */
      sprintf(str_buf, "Set: %.1f C   ", g_AlarmThreshold);
      OLED_ShowString(0, 48, (uint8_t *)str_buf, 12, 1);

      OLED_Refresh();
    }

    osDelay(100); // 刷新率限制
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
    // 等待报警事件标志 (阻塞等待)
    flags = osEventFlagsWait(evt_AlarmHandle,
                             ALARM_TEMP_HIGH | ALARM_VOLT_HIGH,
                             osFlagsWaitAny | osFlagsNoClear, // 不自动清除，由 LogicTask 清除
                             100);

    // 检查是否有有效标志
    if (flags > 0 && flags != osFlagsErrorTimeout)
    {
      /* === 报警鸣叫逻辑 === */

      // 1. 开始鸣叫 (Sound On)
      if (flags & ALARM_TEMP_HIGH) {
        BSP_Buzzer_SetTone(2000, BUZZER_ALARM_SOUND); // 高温: 急促高音
      } else {
        BSP_Buzzer_SetTone(1000, BUZZER_ALARM_SOUND); // 电压: 低沉声音
      }
      osDelay(100);

      // 2. 停止鸣叫 (Sound Off) - 制造滴滴声
      BSP_Buzzer_Off();
      osDelay(100);
    }
    else
    {
      /* === 无报警 === */
      BSP_Buzzer_Off();
      // 此处利用 osEventFlagsWait 的超时机制自动循环，无需额外 Delay
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
    // 喂狗
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(1000);
  }
  /* USER CODE END StartKeepAlive */
}

/* USER CODE BEGIN Header_StartRGBTask */
/**
* @brief Function implementing the RGBTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRGBTask */
void StartRGBTask(void *argument)
{
  /* USER CODE BEGIN StartRGBTask */
  BSP_RGB_Init();

  // 当前执行的指令
  RGBCmd_t current_cmd = { .mode = RGB_MODE_BREATHING, .G = 255 };

  // 动画状态变量
  int16_t breath_val = 0;              // 当前呼吸亮度级
  int8_t  breath_dir = RGB_BREATH_STEP; // 呼吸方向 (+1/-1)
  uint8_t blink_state = 0;             // 闪烁状态 (0/1)

  for(;;)
  {
    RGBCmd_t new_cmd;

    /* --- 1. 接收新指令 --- */
    if (osMessageQueueGet(q_RGBCmdHandle, &new_cmd, NULL, 0) == osOK)
    {
       current_cmd = new_cmd;
       // 重置动画状态，确保切换平滑
       breath_val = 0;
       blink_state = 0;
       breath_dir = RGB_BREATH_STEP;

       // 切换瞬间清空一次
       BSP_RGB_Set(0, 0, 0, 0);
       BSP_RGB_Show();
    }

    /* --- 2. 执行模式逻辑 --- */
    switch (current_cmd.mode)
    {
      case RGB_MODE_OFF:
        BSP_RGB_Set(0, 0, 0, 0);
        BSP_RGB_Show();
        osDelay(100);
        break;

      case RGB_MODE_STATIC:
        BSP_RGB_Set(0, current_cmd.R, current_cmd.G, current_cmd.B);
        BSP_RGB_Show();
        osDelay(100);
        break;

      case RGB_MODE_BREATHING:
        /* --- 呼吸算法 --- */
        breath_val += breath_dir;

        // 边界检查：反转方向
        if (breath_val >= RGB_BREATH_MAX_BRIGHTNESS) {
            breath_val = RGB_BREATH_MAX_BRIGHTNESS;
            breath_dir = -RGB_BREATH_STEP; // 变暗
        } else if (breath_val <= 0) {
            breath_val = 0;
            breath_dir = RGB_BREATH_STEP;  // 变亮
        }

        // 颜色缩放计算: 目标颜色 * (当前呼吸级 / 255)
        // 注意：这里使用 uint16_t 避免乘法溢出
        uint8_t r = (uint16_t)current_cmd.R * breath_val / 255;
        uint8_t g = (uint16_t)current_cmd.G * breath_val / 255;
        uint8_t b = (uint16_t)current_cmd.B * breath_val / 255;

        BSP_RGB_Set(0, r, g, b);
        BSP_RGB_Show();
        osDelay(RGB_BREATH_REFRESH_MS);
        break;

      case RGB_MODE_BLINK:
        /* --- 闪烁算法 --- */
        if (blink_state == 0) {
            BSP_RGB_Set(0, current_cmd.R, current_cmd.G, current_cmd.B);
            blink_state = 1;
        } else {
            BSP_RGB_Set(0, 0, 0, 0);
            blink_state = 0;
        }
        BSP_RGB_Show();
        osDelay(RGB_BLINK_PERIOD_MS);
        break;

      default:
        osDelay(100);
        break;
    }
  }
  /* USER CODE END StartRGBTask */
}

/* USER CODE BEGIN Header_StartBluetoothTask */
/**
* @brief Function implementing the BluetoothTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetoothTask */
void StartBluetoothTask(void *argument)
{
  /* USER CODE BEGIN StartBluetoothTask */
  BSP_BT_Init();

  /* Infinite loop */
  for(;;)
  {
    // 处理蓝牙接收和发送数据 (传入全局 Sensor 数据指针)
    BSP_BT_ProcessTask(&g_LatestSensorData);
  }
  /* USER CODE END StartBluetoothTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */