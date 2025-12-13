#ifndef BSP_RGB_H
#define BSP_RGB_H

#include "main.h"

// ================= 硬件配置区 =================
// LED 数量
#define RGB_LED_NUM  1  

// ================= 数据结构 =================
// 颜色结构体
typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_Color_TypeDef;

// 灯效模式
typedef enum {
    RGB_MODE_OFF = 0,    // 关灯
    RGB_MODE_STATIC,     // 常亮
    RGB_MODE_BLINK,      // 闪烁
    RGB_MODE_BREATHING   // 呼吸
} RGBMode_t;

// 指令包 (RTOS队列用)
typedef struct {
    RGBMode_t mode;
    uint8_t R;
    uint8_t G;
    uint8_t B;
    uint16_t duration;   // 持续时间/间隔
} RGBCmd_t;

// ================= 接口声明 =================
void BSP_RGB_Init(void);
void BSP_RGB_Set(uint8_t id, uint8_t r, uint8_t g, uint8_t b);
void BSP_RGB_Show(void);

// 供 LogicTask 调用的线程安全接口
void BSP_RGB_SendCmd(RGBMode_t mode, uint8_t r, uint8_t g, uint8_t b, uint16_t duration);

#endif // BSP_RGB_H