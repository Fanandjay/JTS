#ifndef MODBUS_TEST_MAIN_H
#define MODBUS_TEST_MAIN_H

#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t BSRR, IDR; } GPIO_TypeDef;
typedef struct { uint32_t CTRL, CYCCNT; } TestDwt;
typedef struct { uint32_t DEMCR; } TestCoreDebug;
typedef struct { uint32_t remaining; } DMA_HandleTypeDef;
typedef struct {
  void *Instance;
  uint32_t RxState;
  uint32_t flags;
  DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;

extern TestDwt test_dwt;
extern TestCoreDebug test_core_debug;
extern GPIO_TypeDef test_rs485_gpio, test_sensor_gpio;
extern uint32_t test_primask, SystemCoreClock;

#define DWT (&test_dwt)
#define CoreDebug (&test_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define USART2 ((void *)0x40004400U)
#define HAL_UART_STATE_READY 0U
#define HAL_UART_STATE_BUSY_RX 1U
#define UART_FLAG_RXNE 1U
#define DMA_IT_HT 1U
#define RESET 0U
#define RS485_DE_GPIO_Port (&test_rs485_gpio)
#define RS485_DE_Pin 1U
#define DS18B20_GPIO_Port (&test_sensor_gpio)
#define DS18B20_Pin 8U
#define __get_PRIMASK() (test_primask)
#define __disable_irq() ((void)(test_primask = 1U))
#define __set_PRIMASK(value) ((void)(test_primask = (value)))
#define __HAL_DMA_GET_COUNTER(dma) ((dma)->remaining)
#define __HAL_DMA_DISABLE_IT(dma, interrupt) ((void)(dma), (void)(interrupt))
#define __HAL_UART_GET_FLAG(uart, flag) ((uart)->flags & (flag))
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)((uart)->flags = 0U))

uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                             uint8_t *data, uint16_t size);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                      uint8_t *data, uint16_t size);

#endif
