/* bsp_key.c */
#include "bsp_key.h"
#include "cmsis_os2.h"
#include <stdio.h> // 用于调试打印

// 引用 freertos.c 中定义的按键队列句柄
extern osMessageQueueId_t q_KeyEvtHandle;

/**
 * @brief  GPIO 外部中断回调函数
 * @param  GPIO_Pin: 触发中断的引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint16_t key_id = KEY_NONE;

    // 判断是哪个引脚触发了中断
    // 简单的防抖可以通过判断两次中断间隔实现，但这里为了演示，
    // 直接发送队列，依靠 RTOS 任务的处理速度差异也能起到一定“去抖”效果
    if (GPIO_Pin == KEY0_Pin)
    {
        key_id = KEY_0;
    }
    // 可以在这里扩展 KEY1, KEY2...

    if (key_id != KEY_NONE)
    {
        // 发送按键 ID 到队列
        // 注意：在中断中优先级参数通常设为 0，超时时间必须为 0 (不允许等待)
        osMessageQueuePut(q_KeyEvtHandle, &key_id, 0, 0);
    }
}