/**
  ******************************************************************************
  * @file    rs485.c
  * @brief   RS485 半双工驱动（USART2 + MAX3485 + DMA）
  *
  *  收发模型：
  *    接收 —— DMA 一次性挂载整个缓冲区，总线空闲（IDLE）时 HAL 回调告知本次
  *            实际收到的字节数，处理完立刻重新挂载，等待下一帧。
  *    发送 —— DE 拉高 -> DMA 搬运 -> 等 TC（最后一位发完）-> DE 拉低 -> 恢复接收。
  *
  *  为什么发送期间要关掉接收：
  *    DE 拉高后 MAX3485 的接收器被禁用，RO 进入高阻态，PA3 悬空会浮出随机电平，
  *    被 UART 当成起始位采进来，轻则收到垃圾帧，重则触发 FE/ORE 把接收状态机卡死。
  *    所以发送前 HAL_UART_AbortReceive，发送完成后再重新挂载。
  ******************************************************************************
  */

#include "rs485.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static uint8_t s_rx_buf[RS485_RX_BUF_SIZE];
static uint8_t s_tx_buf[RS485_TX_BUF_SIZE];

static volatile uint8_t s_tx_busy = 0U;
static RS485_RxCallback_t s_rx_cb = NULL;

/**
  * @brief 挂载 IDLE + DMA 接收
  * @note  HAL_UARTEx_ReceiveToIdle_DMA 内部会打开 DMA 半传输中断，一帧被拆成两半
  *        就会触发两次回调，这里挂完立刻把 HT 中断关掉，只保留 TC + IDLE。
  */
static void rs485_arm_rx(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, s_rx_buf, RS485_RX_BUF_SIZE) == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
}

/**
  * @brief 初始化 RS485
  */
void RS485_Init(RS485_RxCallback_t cb)
{
  RS485_DE_RX();
  s_tx_busy = 0U;
  s_rx_cb   = cb;
  rs485_arm_rx();
}

/**
  * @brief 发送一帧数据
  */
HAL_StatusTypeDef RS485_Send(const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U) || (len > RS485_TX_BUF_SIZE))
  {
    return HAL_ERROR;
  }

  if (s_tx_busy != 0U)
  {
    return HAL_BUSY;
  }

  s_tx_busy = 1U;
  memcpy(s_tx_buf, data, len);

  /* 先关闭接收，避免自发自收和 RO 悬空噪声 */
  (void)HAL_UART_AbortReceive(&huart2);

  RS485_DE_TX();

  if (HAL_UART_Transmit_DMA(&huart2, s_tx_buf, len) != HAL_OK)
  {
    RS485_DE_RX();
    s_tx_busy = 0U;
    rs485_arm_rx();
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief 查询发送状态
  */
uint8_t RS485_IsBusy(void)
{
  return s_tx_busy;
}

/**
  * @brief 发送一个字符串
  */
HAL_StatusTypeDef RS485_SendString(const char *str)
{
  if (str == NULL)
  {
    return HAL_ERROR;
  }

  return RS485_Send((const uint8_t *)str, (uint16_t)strlen(str));
}

/**
  * @brief 格式化并发送
  * @note  先格式化到独立缓冲区，再交给 RS485_Send 拷贝，避免和 DMA 争用发送缓冲
  */
HAL_StatusTypeDef RS485_Printf(const char *fmt, ...)
{
  static char s_fmt_buf[RS485_FMT_BUF_SIZE];
  va_list args;
  int len;

  if (fmt == NULL)
  {
    return HAL_ERROR;
  }

  va_start(args, fmt);
  len = vsnprintf(s_fmt_buf, sizeof(s_fmt_buf), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    return HAL_ERROR;
  }

  /* 内容超长时 vsnprintf 返回的是“本应写入的长度”，这里截断到缓冲区上限 */
  if (len >= (int)sizeof(s_fmt_buf))
  {
    len = (int)sizeof(s_fmt_buf) - 1;
  }

  return RS485_Send((const uint8_t *)s_fmt_buf, (uint16_t)len);
}

/**
  * @brief DMA 发送完成回调
  * @note  该回调由 UART_EndTransmit_IT 在 TC 置位后调用，此时最后一个停止位
  *        已经完整送出，是释放总线最早且安全的时间点。
  *        若在 TXE 或 DMA TC 时就拉低 DE，末字节会被截断。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  RS485_DE_RX();
  s_tx_busy = 0U;
  rs485_arm_rx();
}

/**
  * @brief 收帧回调（IDLE 断帧）
  * @note  IDLE 事件触发后 HAL 已停止本次 DMA 接收，必须重新挂载才能收下一帧。
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  /* 发送期间收到的事件一律丢弃，接收由 TxCplt 重新挂载 */
  if (s_tx_busy != 0U)
  {
    return;
  }

  if ((Size > 0U) && (s_rx_cb != NULL))
  {
    s_rx_cb(s_rx_buf, Size);
  }

  rs485_arm_rx();
}

/**
  * @brief 错误回调
  * @note  溢出、帧错误、噪声、DMA 传输错误都会走到这里，统一做恢复：
  *        清标志、释放方向脚、重新挂接收，避免一次干扰就让通信永久失效。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }

  if (s_tx_busy != 0U)
  {
    RS485_DE_RX();
    s_tx_busy = 0U;
  }

  __HAL_UART_CLEAR_OREFLAG(huart);
  rs485_arm_rx();
}
