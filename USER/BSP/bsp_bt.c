/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : bsp_bt.c
  * Description        : 带有深度调试功能的蓝牙驱动
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_bt.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// ================= 缓冲区配置 =================
#define BT_RX_BUF_SIZE   256
static uint8_t rx_byte;
static uint8_t rx_buffer[BT_RX_BUF_SIZE];
static uint16_t rx_index = 0;

// 连接状态
volatile uint8_t g_BT_IsConnected = 0;

// ================= 内部工具函数 =================

// 清空接收缓冲区
static void BT_ClearRxBuf(void) {
    memset(rx_buffer, 0, BT_RX_BUF_SIZE);
    rx_index = 0;
}

// 供 printf 使用的串口发送 (调试口，通常是 USART1 或 USART2)
// 假设你已经重定向了 printf。如果没有，请使用 HAL_UART_Transmit 打印日志。

// 供蓝牙使用的原始发送
static void BT_RawSend(const char *str) {
    HAL_UART_Transmit(BT_UART_HANDLE, (uint8_t*)str, strlen(str), 100);
}

/**
 * @brief  核心调试函数：发送指令并等待特定的回复
 * @param  cmd: 要发送的 AT 指令
 * @param  ack: 期望收到的回复 (例如 "OK")
 * @param  timeout_ms: 等待超时时间
 * @return 1=成功, 0=失败
 */
static uint8_t BT_SendCmd_CheckACK(const char *cmd, const char *ack, uint32_t timeout_ms) {
    // 1. 清空旧数据，防止干扰
    BT_ClearRxBuf();

    // 2. 打印调试日志：告诉用户我要做什么
    printf("\r\n[BT_DEBUG] >>> Step: Sending '%s'...", cmd);

    // 3. 发送指令
    BT_RawSend(cmd);

    // 4. 循环等待回复
    uint32_t start_tick = HAL_GetTick();
    while ((HAL_GetTick() - start_tick) < timeout_ms) {
        // 这里的 osDelay(10) 很重要，让出 CPU 同时也给串口接收留出时间
        osDelay(10);

        // 5. 检查缓冲区里有没有出现期待的关键词 (比如 "OK")
        if (strstr((char*)rx_buffer, ack) != NULL) {
            printf("\r\n[BT_DEBUG] <<< Success! Received valid ACK.");
            printf("\r\n[BT_DEBUG]     Full Rx Content: [%s]\r\n", rx_buffer);
            return 1; // 成功
        }
    }

    // 6. 如果代码跑到这里，说明超时了
    printf("\r\n[BT_DEBUG] !!! FAILED (Timeout).");
    if (rx_index == 0) {
        printf("\r\n[BT_DEBUG] !!! Analysis: No data received. Check Wiring (TX/RX) or Power.");
    } else {
        printf("\r\n[BT_DEBUG] !!! Analysis: Data received but wrong.");
        printf("\r\n[BT_DEBUG]     Received content: [%s]", rx_buffer);
        printf("\r\n[BT_DEBUG]     Suggestion: Check Baud Rate (try 9600/115200) or Instruction format.");
    }
    printf("\r\n");
    return 0; // 失败
}

// ================= 外部接口 =================

// 必须在 main.c 的 HAL_UART_RxCpltCallback 中调用，或者直接替换该回调
void BSP_BT_RxCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) { // 确认是蓝牙所在的串口
        // 存入缓冲区
        if (rx_index < BT_RX_BUF_SIZE - 1) {
            rx_buffer[rx_index++] = rx_byte;
            rx_buffer[rx_index] = '\0'; // 补零，方便 string 操作
        }

        // 实时打印每一个字节（极其严格的调试模式）
        // 如果数据量大建议注释掉，但在初始化阶段这非常有用
        // printf("%c", rx_byte);

        // 连接状态监测 (异步)
        if (strstr((char*)rx_buffer, "+BLECONN")) {
            g_BT_IsConnected = 1;
            // 收到连接事件不急着清空，让主逻辑也能看到
        } else if (strstr((char*)rx_buffer, "+BLEDISCONN")) {
            g_BT_IsConnected = 0;
        }

        // 继续接收下一字节
        HAL_UART_Receive_IT(BT_UART_HANDLE, &rx_byte, 1);
    }
}

void BSP_BT_Init(void) {
    printf("\r\n========================================");
    printf("\r\n[BT_DEBUG] Starting Bluetooth Init (Final Strategy)...");
    printf("\r\n========================================\r\n");

    if(HAL_UART_Receive_IT(BT_UART_HANDLE, &rx_byte, 1) != HAL_OK) return;
    osDelay(1000);

    // --- 1. 恢复出厂设置 (专治各种不服) ---
    // 这一步非常关键，清除之前的错误配置
    printf("[BT_DEBUG] Factory Resetting (AT+RESTORE)...\r\n");
    // 注意：有些固件回复 restore 后会自动重启，不需要再发 RST
    BT_RawSend("AT+RESTORE\r\n");
    osDelay(3000); // 恢复出厂需要较长时间
    BT_ClearRxBuf();

    // --- 2. 基础握手 ---
    // 刚恢复完，再次确认活著
    if (!BT_SendCmd_CheckACK("AT\r\n", "OK", 1000)) {
        BT_RawSend("+++"); osDelay(500); // 防一手透传
        BT_SendCmd_CheckACK("AT\r\n", "OK", 1000);
    }
    BT_SendCmd_CheckACK("ATE0\r\n", "OK", 500);

    // --- 3. 设置为从机 ---
    BT_SendCmd_CheckACK("AT+BLEMODE=0\r\n", "OK", 1000);

    // --- 4. 关键策略变更：查询名字 ---
    // 不改名字了，直接问它：你叫什么？
    // 发送查询指令
    printf("[BT_DEBUG] Querying Device Name...\r\n");
    BT_ClearRxBuf();
    BT_RawSend("AT+BLENAME?\r\n");
    osDelay(1000); // 等待回复
    printf("[BT_DEBUG] >>> DEVICE NAME IS: %s\r\n", rx_buffer);
    // 请在串口日志里仔细看这一行！手机就搜这个名字！

    // --- 5. 尝试开启广播 ---
    // 恢复出厂后，默认可能已经是开启广播的。
    // 我们试著发一下开启指令，如果报错 232，说明它已经在广播了，也是好事。
    printf("[BT_DEBUG] Trying to enable advertising...\r\n");
    if (BT_SendCmd_CheckACK("AT+BLEADVEN=1\r\n", "OK", 2000)) {
        printf("\r\n[BT_DEBUG] >>> SUCCESS: Advertising Started!\r\n");
    } else {
        printf("\r\n[BT_DEBUG] Note: Enable failed (maybe already advertising?).\r\n");
        printf("[BT_DEBUG] PLEASE SCAN FOR THE NAME PRINTED ABOVE!\r\n");
    }

    printf("\r\n========================================\r\n");
}

// 替换 bsp_bt.c 中的 BSP_BT_ProcessTask

void BSP_BT_ProcessTask(SensorData_t *pData) {
    static uint32_t last_send_time = 0;
    char data_buf[64]; // 存放真实数据
    char cmd_buf[32];  // 存放 AT 指令

    // 只有在连接成功后才发送数据
    // 注意：这里的 g_BT_IsConnected 依赖于中断回调里的 +BLECONN 判断
    // 如果你恢复出厂后还没做这一步，可以先手动改为 if(1) 强制发送试试
    if (1) {

        // 控制发送频率: 1000ms 一次 (不要太快，AT指令需要处理时间)
        if (HAL_GetTick() - last_send_time >= 1000) {

            // 1. 准备要发送的数据
            // 格式: $Temp,Voltage,Freq
            int len = sprintf(data_buf, "$%.1f,%.2f,%lu\r\n",
                    pData->temp_celsius,
                    (float)(pData->adc_raw * 3.3f / 4095.0f),
                    pData->freq_hz);

            // 2. 发送“发送请求”指令
            // AT+BLESEND=<link_id>,<length>
            // 0 是默认连接ID，len 是数据长度
            sprintf(cmd_buf, "AT+BLESEND=0,%d\r\n", len);
            BT_RawSend(cmd_buf);

            // 3. 稍作延时，等待模块回复 ">" 符号
            // 实测 50ms-100ms 足够
            osDelay(50);

            // 4. 发送真实数据
            BT_RawSend(data_buf);

            // 5. 调试打印 (看看发了啥)
            printf("[BT] Sent: %s", data_buf);

            // 更新时间戳
            last_send_time = HAL_GetTick();
        }

        osDelay(100);
    } else {
        // 如果未连接，检测频率放慢
        // 这里的逻辑可以优化：如果长时间未连接，可以再次检测 +BLECONN
        osDelay(500);
    }
}