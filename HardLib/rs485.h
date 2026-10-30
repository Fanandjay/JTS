/**
  ******************************************************************************
  * @file    rs485.h
  * @brief   RS485 半双工驱动（USART2 + MAX3485 + DMA）
  * @note    DE/RE 由 PC0 控制：高电平发送，低电平接收
  ******************************************************************************
  */

#ifndef __RS485_H__
#define __RS485_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 接收缓冲区大小：一帧最大字节数。DMA 一次性挂载，靠 IDLE 断帧 */
#define RS485_RX_BUF_SIZE       256U
/* 发送缓冲区大小：调用 RS485_Send 时内部拷贝，调用者的 buffer 可以立刻释放 */
#define RS485_TX_BUF_SIZE       256U
/* RS485_Printf 格式化缓冲区大小，超长内容会被截断 */
#define RS485_FMT_BUF_SIZE      128U

/* 方向控制：DE 与 /RE 并联，一起拉高为发送，一起拉低为接收 */
#define RS485_DE_TX()           HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)
#define RS485_DE_RX()           HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET)

/**
  * @brief 收帧回调（在 USART2 中断上下文中执行，务必保持简短）
  * @param data 指向驱动内部接收缓冲区，回调返回后内容会被下一帧覆盖
  * @param len  本次收到的字节数
  * @note  需要长时间处理或回发数据时，请在此处拷贝到自己的队列，交给主循环处理
  */
typedef void (*RS485_RxCallback_t)(const uint8_t *data, uint16_t len);

/**
  * @brief 初始化 RS485：方向脚切到接收，挂载 IDLE + DMA 接收
  * @param cb 收帧回调，可为 NULL（只发不收）
  * @note  必须在 MX_USART2_UART_Init() 之后调用
  */
void RS485_Init(RS485_RxCallback_t cb);

/**
  * @brief 发送一帧数据（非阻塞，拷贝后由 DMA 发送）
  * @param data 数据指针
  * @param len  字节数，不能超过 RS485_TX_BUF_SIZE
  * @retval HAL_OK / HAL_BUSY（上一帧未发完）/ HAL_ERROR（参数错误）
  * @note  发送完成后自动切回接收态，期间接收是关闭的，不会收到自己发出的数据
  */
HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t len);

/**
  * @brief 查询是否正在发送
  * @retval 0 空闲（可发下一帧），非 0 忙
  */
uint8_t RS485_IsBusy(void);

/**
  * @brief 发送一个字符串（自动计算长度，不含结尾 '\0'）
  */
HAL_StatusTypeDef RS485_SendString(const char *str);

/**
  * @brief 格式化并发送，用法同 printf
  * @note  输出长度上限 RS485_FMT_BUF_SIZE-1，超出部分截断；
  *        仅支持整数/字符串/十六进制，浮点需要在链接选项里加 -u _printf_float
  */
HAL_StatusTypeDef RS485_Printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_H__ */
