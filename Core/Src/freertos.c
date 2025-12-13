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
#include <stdbool.h> // 使用 bool 类型

#include "bsp_oled.h"
#include "bsp_seg.h"
#include "bsp_lm75.h"
#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_freq.h"
#include "bsp_rgb.h"
#include "bsp_key.h"
#include "bsp_bt.h"

#include "app_types.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ALARM_TEMP_HIGH   (1 << 0)  // 温度过高
#define ALARM_VOLT_HIGH   (1 << 1)  // 电压过高


// ================= RGB 呼吸灯配置 =================

// 1. 最大亮度 (0-255)
// 如果觉得 LED 太刺眼，可以把这个改小，比如 100
#define RGB_BREATH_MAX_BRIGHTNESS  30

// 2. 呼吸步长 (每次增加的数值)
// 数值越大，呼吸越急促；数值越小，呼吸越平缓
#define RGB_BREATH_STEP            1

// 3. 呼吸刷新周期 (ms)
// 决定动画的流畅度，建议 20-50ms
#define RGB_BREATH_REFRESH_MS      40

// ================= RGB 报警闪烁配置 =================

// 4. 报警闪烁周期 (ms)
// 100ms 代表 100ms亮 -> 100ms灭 (即 5Hz 闪烁)
#define RGB_BLINK_PERIOD_MS        100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 全局传感器数据副本 (公告板)
// SensorTask 写，LogicTask 读
SensorData_t g_LatestSensorData = {0};
// [新增] 显示模式: 0=详细模式(默认), 1=大字体模式
volatile uint8_t g_DisplayPage = 0;
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
  //
  BSP_RGB_Init();
  printf("[OK] RGB Init\r\n");

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
    // printf("[Sensor] T:%.1f C, ADC:%d, Freq:%lu Hz\r\n",
    //        current_data.temp_celsius,
    //        current_data.adc_raw,
    //        current_data.freq_hz);

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

  // 局部变量
  uint16_t key_event = 0;
  bool is_muted = false;       // 静音标志位
  bool alarm_condition = false; // 报警条件汇总

  // [新增] 记录上一次的报警状态，用于检测状态跳变
  bool last_alarm_status = false;

  // 上电默认发送一次“绿色呼吸”
  BSP_RGB_SendCmd(RGB_MODE_BREATHING, 0, 255, 0, 0);

  /* Infinite loop */
  for(;;)
  {
    // ==========================================
    // 1. 处理按键事件 (从队列接收)
    // ==========================================
    // 使用 0 等待时间，非阻塞查询。如果有按键按下，处理它。
    if (osMessageQueueGet(q_KeyEvtHandle, &key_event, NULL, 0) == osOK)
    {
        if (key_event == KEY_0)
        {
            printf("[Logic] Key0 Pressed -> Mute Alarm\r\n");
            is_muted = true; // 激活静音
        }
      // [新增] KEY1 切换界面
        else if (key_event == KEY_1)
        {
          OLED_Clear();
          OLED_Refresh();
          printf("[Logic] Key1 Pressed -> Switch Page\r\n");
          g_DisplayPage++;
          if (g_DisplayPage > 1) {
            g_DisplayPage = 0; // 循环切换：0 -> 1 -> 0
          }
        }
    }

    // ==========================================
    // 2. 检查报警条件
    // ==========================================
    // 判断是否有任意一项异常
    // 温度 > 30.0 或 电压 > 2.5V (ADC > 3103) - 根据您的代码调整阈值
    // 这里为了演示，假设阈值是 30度 和 1861(约1.5V，参考您原代码)
    bool temp_high = (g_LatestSensorData.temp_celsius > 35.0f);
    bool volt_high = (g_LatestSensorData.adc_raw > 1861);

    alarm_condition = temp_high | volt_high;

    // ==========================================
    // [新增] RGB 状态机控制逻辑
    // ==========================================

    // 只有当状态发生改变时 (Edge Detection)，才发送队列指令
    // 这样避免每 100ms 都塞满队列
    if (alarm_condition != last_alarm_status)
    {
      if (alarm_condition == true)
      {
        // --- 状态变更为：报警 ---
        // 发送红色 (255, 0, 0) 闪烁
        printf("[Logic] Alarm! RGB -> Red Blink\r\n");
        BSP_RGB_SendCmd(RGB_MODE_BLINK, 30, 0, 0, 0);
      }
      else
      {
        // --- 状态变更为：正常 ---
        // 发送绿色 (0, 255, 0) 呼吸
        printf("[Logic] Normal. RGB -> Green Breath\r\n");
        BSP_RGB_SendCmd(RGB_MODE_BREATHING, 0, 255, 0, 0);
      }

      // 更新历史状态
      last_alarm_status = alarm_condition;
    }

    if (alarm_condition)
    {
        // === 有异常发生 ===

        if (is_muted)
        {
            // 如果处于静音状态，强制清除所有报警标志
            // 这样 AlarmTask 就会检测到无标志，从而关闭蜂鸣器
            osEventFlagsClear(evt_AlarmHandle, ALARM_TEMP_HIGH | ALARM_VOLT_HIGH);
        }
        else
        {
            // 如果未静音，根据具体原因设置标志位
            if (temp_high) osEventFlagsSet(evt_AlarmHandle, ALARM_TEMP_HIGH);
            if (volt_high) osEventFlagsSet(evt_AlarmHandle, ALARM_VOLT_HIGH);
        }
    }
    else
    {
        // === 一切正常 ===

        // 1. 清除报警标志 (停止蜂鸣器)
        osEventFlagsClear(evt_AlarmHandle, ALARM_TEMP_HIGH | ALARM_VOLT_HIGH);

        // 2. 复位静音状态
        // 只有当环境恢复正常后，才取消静音。
        // 这样下次再变异常时，蜂鸣器会再次响起。
        if (is_muted)
        {
            is_muted = false;
            printf("[Logic] Condition Normal -> Mute Reset\r\n");
        }
    }

    // 逻辑处理周期 (100ms)
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
  char str_buf[32];

  /* Infinite loop */
  for(;;)
  {
    // 1. 获取数据
    status = osMessageQueueGet(q_SensorDataHandle, &recv_data, NULL, 0);

    // 如果没有新数据，可以使用全局缓存 g_LatestSensorData 来刷新界面（防止切换界面时闪烁）
    // 但为了简单，这里假设数据更新够快。

    if (status == osOK)
    {
      // [关键修改] 每次刷新前建议清空缓存，或者确保覆盖了旧内容
      // OLED_Clear(); // 如果您的驱动是全屏刷新模式，可以在这里 clear

      if (g_DisplayPage == 0)
      {
        // === 界面 0: 详细数据模式 (Temp + Volt + Freq) ===

        // 1. 温度
        sprintf(str_buf, "T: %.1f C   ", recv_data.temp_celsius);
        OLED_ShowString(0, 0, (uint8_t *)str_buf, 16, 1);

        // 2. 电压
        float voltage = recv_data.adc_raw * 3.3f / 4095.0f;
        sprintf(str_buf, "V: %.2f V   ", voltage);
        OLED_ShowString(0, 16, (uint8_t *)str_buf, 16, 1);

        // 3. [新增] 频率
        // 如果频率很大，可以使用 %.1f kHz
        if(recv_data.freq_hz < 10000)
          sprintf(str_buf, "F: %lu Hz   ", recv_data.freq_hz);
        else
          sprintf(str_buf, "F: %.1f kHz ", recv_data.freq_hz / 1000.0f);

        OLED_ShowString(0, 32, (uint8_t *)str_buf, 16, 1);

        // 状态栏
        OLED_ShowString(0, 48, "Mode: Detail", 12, 1);
      }
      else
      {
        // === 界面 1: 极简模式 (大字体温度) ===
        // 假设您没有移植大字体库，这里用普通字体模拟或仅显示核心信息

        OLED_ShowString(18, 0, "Temp Monitor", 16, 1);

        // 显示大一点的数值 (如果有 24号字体可以用 24)
        sprintf(str_buf, "%.1f C", recv_data.temp_celsius);
        OLED_ShowString(32, 18, (uint8_t *)str_buf, 24, 1); // 居中一点

        // 底部提示按键功能
        OLED_ShowString(0, 48, "K1:Switch View", 12, 1);
      }
    }

    OLED_Refresh();
    osDelay(100); // 刷新率 10Hz
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

  // 默认状态变量
  RGBCmd_t current_cmd = {0};

  // 初始默认：绿色呼吸
  current_cmd.mode = RGB_MODE_BREATHING;
  current_cmd.R = 0;
  current_cmd.G = 255; // 这里设置255，实际显示亮度会由呼吸算法缩放到 0~30
  current_cmd.B = 0;

  // 动画控制变量
  int16_t breath_val = 0;
  // 初始化步长方向
  int8_t  breath_dir = RGB_BREATH_STEP;
  uint8_t blink_state = 0;

  for(;;)
  {
    RGBCmd_t new_cmd;

    // 尝试获取新指令 (0等待，非阻塞)
    if (osMessageQueueGet(q_RGBCmdHandle, &new_cmd, NULL, 0) == osOK)
    {
       current_cmd = new_cmd;

       // 重置动画状态，确保切换顺滑
       breath_val = 0;
       blink_state = 0;
       breath_dir = RGB_BREATH_STEP; // 重置为变亮方向

       // 切换瞬间先灭灯，清除残影
       BSP_RGB_Set(0, 0, 0, 0);
       BSP_RGB_Show();
    }

    switch (current_cmd.mode)
    {
      case RGB_MODE_OFF:
        BSP_RGB_Set(0, 0, 0, 0);
        BSP_RGB_Show();
        osDelay(100);
        break;

      case RGB_MODE_STATIC:
        // 常亮模式使用指令指定的原始亮度
        BSP_RGB_Set(0, current_cmd.R, current_cmd.G, current_cmd.B);
        BSP_RGB_Show();
        osDelay(100);
        break;

      case RGB_MODE_BREATHING:
        // --- 呼吸算法 ---
        breath_val += breath_dir;

        // 1. 到达最大亮度 (30)：反转方向，开始变暗
        if (breath_val >= RGB_BREATH_MAX_BRIGHTNESS) {
            breath_val = RGB_BREATH_MAX_BRIGHTNESS;
            breath_dir = -RGB_BREATH_STEP;
        }
        // 2. 到达熄灭状态 (0)：反转方向，开始变亮
        else if (breath_val <= 0) {
            breath_val = 0;
            breath_dir = RGB_BREATH_STEP;
        }

        // --- 颜色缩放计算 ---
        // 核心逻辑：TargetColor * (当前呼吸等级 / 255)
        // 例如：G=255, breath_val=30 -> 255 * 30 / 255 = 30 (实际输出亮度)
        // 例如：G=255, breath_val=15 -> 255 * 15 / 255 = 15 (半亮)
        uint8_t r = (uint16_t)current_cmd.R * breath_val / 255;
        uint8_t g = (uint16_t)current_cmd.G * breath_val / 255;
        uint8_t b = (uint16_t)current_cmd.B * breath_val / 255;

        BSP_RGB_Set(0, r, g, b);
        BSP_RGB_Show();

        // 延时 40ms，控制呼吸速度
        osDelay(RGB_BREATH_REFRESH_MS);
        break;

      case RGB_MODE_BLINK:
        // --- 闪烁算法 (保持全亮度) ---
        // 报警时我们希望它越亮越好，所以这里不进行缩放，直接用原值
        if (blink_state == 0) {
            // 亮 (使用指令中的原始颜色，通常是 255,0,0)
            BSP_RGB_Set(0, current_cmd.R, current_cmd.G, current_cmd.B);
            blink_state = 1;
        } else {
            // 灭
            BSP_RGB_Set(0, 0, 0, 0);
            blink_state = 0;
        }
        BSP_RGB_Show();

        // 延时 100ms，快速闪烁
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
    BSP_BT_ProcessTask(&g_LatestSensorData);
  }
  /* USER CODE END StartBluetoothTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

