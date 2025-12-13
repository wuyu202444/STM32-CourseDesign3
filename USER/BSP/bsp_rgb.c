#include "bsp_rgb.h"
#include "tim.h"         // 引用 CubeMX 生成的 tim.h
#include "cmsis_os2.h"   // RTOS

// 引用 freertos.c 定义的队列句柄
extern osMessageQueueId_t q_RGBCmdHandle;

// ================= 时序参数 =================
// STM32F411 典型主频 100MHz (ARR=124) 或 84MHz (ARR=104)
// 请务必检查 main.c 中的 SystemClock_Config 确认 HCLK

#if defined(HCLK_100MHZ) // 如果你是 100MHz
    #define WS_BIT_0   38  // ~0.38us
    #define WS_BIT_1   80  // ~0.80us
#else // 假设是 84MHz (ARR = 104)
    // 1 Tick = 11.9ns
    // Total 105 Ticks = 1.25us
    // 0 Code (0.35us) = ~29 Ticks
    // 1 Code (0.80us) = ~67 Ticks
    #define WS_BIT_0   29
    #define WS_BIT_1   67
#endif

// ================= 变量定义 =================
RGB_Color_TypeDef g_RGB_Data[RGB_LED_NUM];

// DMA 缓冲区: (24 bit * LED数量) + 50 Reset Signals
#define DMA_BUFFER_SIZE (RGB_LED_NUM * 24 + 50)
uint16_t g_rgb_dma_buffer[DMA_BUFFER_SIZE] = {0};

volatile uint8_t g_rgb_busy = 0;

// ================= 实现 =================

void BSP_RGB_Init(void)
{
    // 初始化时全灭
    BSP_RGB_Set(0, 0, 0, 0);
    BSP_RGB_Show();
}

void BSP_RGB_Set(uint8_t id, uint8_t r, uint8_t g, uint8_t b)
{
    if (id >= RGB_LED_NUM) return;
    g_RGB_Data[id].R = r;
    g_RGB_Data[id].G = g;
    g_RGB_Data[id].B = b;
}

void BSP_RGB_Show(void)
{
    if (g_rgb_busy) return;

    uint32_t idx = 0;

    for (int i = 0; i < RGB_LED_NUM; i++)
    {
        // WS2812 顺序 GRB
        uint32_t color = (g_RGB_Data[i].G << 16) | (g_RGB_Data[i].R << 8) | g_RGB_Data[i].B;

        for (int k = 23; k >= 0; k--)
        {
            if ((color >> k) & 0x01)
                g_rgb_dma_buffer[idx++] = WS_BIT_1;
            else
                g_rgb_dma_buffer[idx++] = WS_BIT_0;
        }
    }

    // Reset 信号 (保持低电平)
    for (int i = 0; i < 50; i++)
    {
        g_rgb_dma_buffer[idx++] = 0;
    }

    g_rgb_busy = 1;
    
    // 启动 DMA, 注意这里的 Length 是 buffer 里的元素个数 (idx)
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_4, (uint32_t *)g_rgb_dma_buffer, idx);
}

void BSP_RGB_SendCmd(RGBMode_t mode, uint8_t r, uint8_t g, uint8_t b, uint16_t duration)
{
    RGBCmd_t msg;
    msg.mode = mode;
    msg.R = r; msg.G = g; msg.B = b;
    msg.duration = duration;
    
    if (q_RGBCmdHandle != NULL) {
        osMessageQueuePut(q_RGBCmdHandle, &msg, 0, 0);
    }
}

// DMA 完成回调
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    // 必须判断定时器和通道，防止冲突
    if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
        HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_4);
        // 强制拉低，防止 idle 电平异常
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0); 
        g_rgb_busy = 0;
    }
}