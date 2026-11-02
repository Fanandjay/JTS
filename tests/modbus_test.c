/* Runs the real protocol and RS485 C code with an in-memory HAL/DMA model.
   This validates byte streams and timing decisions, not electrical behavior. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "usart.h"
#include "modbus.h"
#include "rs485.h"

/* Include the sensor source to test its private shared-counter initializer
   without running 1-Wire delays on the host. */
#include "../HardLib/ds18b20.c"

TestDwt test_dwt;
TestCoreDebug test_core_debug;
GPIO_TypeDef test_rs485_gpio, test_sensor_gpio;
uint32_t test_primask;
uint32_t SystemCoreClock = 168000000U;
static DMA_HandleTypeDef test_rx_dma;
UART_HandleTypeDef huart2;
static uint8_t *rx_buffer;
static uint8_t tx_buffer[256];
static uint16_t tx_size;
static unsigned tx_count, test_count;
static GPIO_PinState direction;
static HAL_StatusTypeDef tx_result;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);

#define CHECK(condition) do { if (!(condition)) { \
  fprintf(stderr, "%s:%d: %s\n", __func__, __LINE__, #condition); exit(1); \
} } while (0)

uint32_t HAL_GetTick(void) { return 0U; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  CHECK(port == &test_rs485_gpio && pin == RS485_DE_Pin);
  direction = state;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                             uint8_t *data, uint16_t size)
{
  if (uart->RxState != HAL_UART_STATE_READY) { return HAL_BUSY; }
  uart->RxState = HAL_UART_STATE_BUSY_RX;
  uart->hdmarx->remaining = size;
  rx_buffer = data;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
  uart->RxState = HAL_UART_STATE_READY;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *uart,
                                      uint8_t *data, uint16_t size)
{
  CHECK(uart == &huart2);
  CHECK(direction == GPIO_PIN_SET);
  CHECK(uart->RxState == HAL_UART_STATE_READY);
  if (tx_result != HAL_OK) { return tx_result; }
  CHECK(size <= sizeof(tx_buffer));
  memcpy(tx_buffer, data, size);
  tx_size = size;
  tx_count++;
  return HAL_OK;
}

static uint16_t decode(const char *hex, uint8_t *out)
{
  uint16_t size = 0U;
  while (*hex != '\0') {
    char *end;
    unsigned long value = strtoul(hex, &end, 16);
    CHECK(end != hex && value <= 255U && size < 256U);
    out[size++] = (uint8_t)value;
    hex = end;
    while (*hex == ' ') { hex++; }
  }
  return size;
}

static void advance_us(uint32_t us)
{
  DWT->CYCCNT += us * (SystemCoreClock / 1000000U);
}

static void reset_fixture(void)
{
  memset(&huart2, 0, sizeof(huart2));
  memset(&test_dwt, 0, sizeof(test_dwt));
  huart2.Instance = USART2;
  huart2.hdmarx = &test_rx_dma;
  test_primask = 0U;
  tx_size = 0U;
  tx_count = 0U;
  tx_result = HAL_OK;
  MODBUS_Init();
  RS485_Init(MODBUS_OnFrame);
  MODBUS_SetInputReg(MB_IR_TEMP_X100, 2500U);
}

static void receive_bytes(const uint8_t *data, uint16_t size)
{
  CHECK(size <= RS485_RX_BUF_SIZE);
  memcpy(rx_buffer, data, size);
  huart2.RxState = HAL_UART_STATE_READY; /* normal-DMA IDLE event */
  HAL_UARTEx_RxEventCallback(&huart2, size);
}

static void receive(const char *hex)
{
  uint8_t data[256];
  uint16_t size = decode(hex, data);
  receive_bytes(data, size);
}

static void finish_tx(void)
{
  HAL_UART_TxCpltCallback(&huart2);
  CHECK(direction == GPIO_PIN_RESET && RS485_IsBusy() == 0U);
  CHECK(huart2.RxState == HAL_UART_STATE_BUSY_RX);
}

static void expect_reply(const char *hex)
{
  uint8_t expected[256];
  uint16_t size = decode(hex, expected);
  CHECK(tx_size == size);
  CHECK(memcmp(tx_buffer, expected, size) == 0);
  finish_tx();
}

static void transact(const char *request, const char *reply)
{
  unsigned before = tx_count;
  receive(request);
  advance_us(1750U);
  MODBUS_Task();
  CHECK(tx_count == before + 1U);
  expect_reply(reply);
}

static void read_temperature(void)
{
  reset_fixture();
  /* The exact request from the user's trace must return seven bytes, no echo. */
  transact("01 04 00 00 00 01 31 ca", "01 04 02 09 c4 be f3");
  MODBUS_SetInputReg(MB_IR_TEMP_X100, (uint16_t)(int16_t)-1050);
  transact("01 04 00 00 00 01 31 ca", "01 04 02 fb e6 7b 8a");
}

static void wait_full_silence(void)
{
  reset_fixture();
  receive("01 04 00 00 00 01 31 ca");
  advance_us(1749U);
  MODBUS_Task();
  CHECK(tx_count == 0U);
  advance_us(1U);
  MODBUS_Task();
  CHECK(tx_count == 1U);
  expect_reply("01 04 02 09 c4 be f3");
}

static void counter_wrap(void)
{
  reset_fixture();
  DWT->CYCCNT = UINT32_MAX - 1000U;
  receive("01 04 00 00 00 01 31 ca");
  advance_us(10U); /* crosses the hardware counter wrap, still inside t3.5 */
  MODBUS_Task();
  CHECK(tx_count == 0U);
  advance_us(1740U);
  MODBUS_Task();
  CHECK(tx_count == 1U);
  expect_reply("01 04 02 09 c4 be f3");
}

static void pending_dma_fragment(void)
{
  reset_fixture();
  receive("01 04 00 00");
  test_rx_dma.remaining = RS485_RX_BUF_SIZE - 4U;
  advance_us(2000U);
  MODBUS_Task();
  CHECK(tx_count == 0U && MODBUS_GetCrcErrorCount() == 0U);
  receive("00 01 31 ca");
  advance_us(1750U);
  MODBUS_Task();
  CHECK(tx_count == 1U);
  expect_reply("01 04 02 09 c4 be f3");
}

static void long_fragment_and_multi_register_write(void)
{
  unsigned i;
  reset_fixture();
  receive("01 10 00 00");
  /* Later IDLE callback includes a long fragment's wire time. It must not
     discard the first fragment merely because callbacks are far apart. */
  advance_us(2500U);
  receive("00 08 10 00 01 00 02 00 03 00 04 00 05 00 06 00 07 00 08 8f 0c");
  advance_us(1750U);
  MODBUS_Task();
  CHECK(tx_count == 1U);
  expect_reply("01 10 00 00 00 08 c1 cf");
  for (i = 0U; i < 8U; i++) { CHECK(MODBUS_GetHoldingReg((uint16_t)i) == i + 1U); }
}

static void preserve_interrupt_mask(void)
{
  reset_fixture();
  receive("01 04 00 00 00 01 31 ca");
  test_primask = 1U;
  MODBUS_Task();
  CHECK(test_primask == 1U && tx_count == 0U);
  advance_us(1750U);
  MODBUS_Task();
  CHECK(test_primask == 1U && tx_count == 1U);
  test_primask = 0U;
  expect_reply("01 04 02 09 c4 be f3");
}

static void crc_and_address_rejection(void)
{
  reset_fixture();
  receive("01 04 00 00 00 01 31 cb");
  advance_us(1750U);
  MODBUS_Task();
  CHECK(tx_count == 0U && MODBUS_GetCrcErrorCount() == 1U);
  receive("02 04 00 00 00 01 31 f9");
  advance_us(1750U);
  MODBUS_Task();
  CHECK(tx_count == 0U && MODBUS_GetFrameCount() == 0U);
}

static void exception_replies(void)
{
  reset_fixture();
  transact("01 04 00 08 00 01 b0 08", "01 84 02 c2 c1");
  transact("01 04 00 00 00 00 f0 0a", "01 84 03 03 01");
  transact("01 41 c0 10", "01 c1 01 b0 50");
}

static void holding_register_access(void)
{
  reset_fixture();
  transact("01 06 00 01 00 02 59 cb", "01 06 00 01 00 02 59 cb");
  transact("01 03 00 00 00 02 c4 0b", "01 03 04 01 f4 00 02 3b fc");
}

static void coil_and_discrete_access(void)
{
  reset_fixture();
  transact("01 05 00 00 ff 00 8c 3a", "01 05 00 00 ff 00 8c 3a");
  transact("01 01 00 00 00 03 7c 0b", "01 01 01 01 90 48");
  MODBUS_SetDiscreteInput(0U, 1U);
  MODBUS_SetDiscreteInput(2U, 1U);
  transact("01 02 00 00 00 03 38 0b", "01 02 01 05 61 8b");
  transact("01 0f 00 00 00 03 01 06 0f 55", "01 0f 00 00 00 03 15 ca");
  CHECK(MODBUS_GetCoil(0U) == 0U);
  CHECK(MODBUS_GetCoil(1U) == 1U && MODBUS_GetCoil(2U) == 1U);
}

static void broadcast_write(void)
{
  reset_fixture();
  receive("00 06 00 01 00 01 18 1b");
  advance_us(1750U);
  MODBUS_Task();
  CHECK(tx_count == 0U && MODBUS_GetHoldingReg(1U) == 1U);
}

static void overflow_recovery(void)
{
  uint8_t noise[200] = {0};
  reset_fixture();
  receive_bytes(noise, 200U);
  receive_bytes(noise, 100U);
  CHECK(MODBUS_GetDropCount() == 1U);
  transact("01 04 00 00 00 01 31 ca", "01 04 02 09 c4 be f3");
}

static void transport_state(void)
{
  uint8_t payload[] = {1U, 2U, 3U};
  reset_fixture();
  CHECK(RS485_IsRxActive() == 0U);
  huart2.flags = UART_FLAG_RXNE;
  CHECK(RS485_IsRxActive() == 1U);
  huart2.flags = 0U;
  test_rx_dma.remaining--;
  CHECK(RS485_IsRxActive() == 1U);
  tx_result = HAL_ERROR;
  CHECK(RS485_Send(payload, sizeof(payload)) == HAL_ERROR);
  CHECK(RS485_IsBusy() == 0U && direction == GPIO_PIN_RESET);
  CHECK(huart2.RxState == HAL_UART_STATE_BUSY_RX);
  tx_result = HAL_OK;
  CHECK(RS485_Send(payload, sizeof(payload)) == HAL_OK);
  CHECK(RS485_IsBusy() != 0U && RS485_IsRxActive() == 0U);
  CHECK(RS485_Send(payload, sizeof(payload)) == HAL_BUSY);
  finish_tx();
}

static void shared_dwt_initialization(void)
{
  reset_fixture();
  DWT->CYCCNT = 1234567U;
  ds_dwt_init();
  CHECK(DWT->CYCCNT == 1234567U);
}

#define RUN(test) do { test(); test_count++; printf("PASS %s\n", #test); } while (0)
int main(void)
{
  RUN(read_temperature);
  RUN(wait_full_silence);
  RUN(counter_wrap);
  RUN(pending_dma_fragment);
  RUN(long_fragment_and_multi_register_write);
  RUN(preserve_interrupt_mask);
  RUN(crc_and_address_rejection);
  RUN(exception_replies);
  RUN(holding_register_access);
  RUN(coil_and_discrete_access);
  RUN(broadcast_write);
  RUN(overflow_recovery);
  RUN(transport_state);
  RUN(shared_dwt_initialization);
  printf("All %u test groups passed.\n", test_count);
  return 0;
}
