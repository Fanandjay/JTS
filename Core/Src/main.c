/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rs485.h"
#include "ds18b20.h"
#include "modbus.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* LED 规律闪烁的状态 */
static uint32_t g_led_tick = 0U;
static uint8_t  g_led_step = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 板上 LED 的有效电平：高电平点亮填 1，低电平点亮填 0。
   接反了不影响通信，只是读写线圈的效果整体反相。 */
#define LED_ACTIVE_HIGH   1U

static void app_led_write(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
#if (LED_ACTIVE_HIGH != 0U)
  HAL_GPIO_WritePin(port, pin, (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(port, pin, (on != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

/**
  * @brief 把传感器状态刷进输入寄存器（3x 区），主站用功能码 04 读
  */
static void app_update_inputs(void)
{
  int32_t mdeg = DS18B20_GetMilliDeg();
  int32_t t100;

  /* 毫摄氏度 -> 0.01 ℃，四舍五入；有符号整数除法向 0 截断，负数单独处理 */
  if (mdeg >= 0)
  {
    t100 = (mdeg + 5) / 10;
  }
  else
  {
    t100 = (mdeg - 5) / 10;
  }

  MODBUS_SetInputReg(MB_IR_TEMP_X100, (uint16_t)(int16_t)t100);
  MODBUS_SetInputReg(MB_IR_PRESENT,   (uint16_t)DS18B20_IsPresent());
  MODBUS_SetInputReg(MB_IR_OK_CNT,    (uint16_t)(DS18B20_GetOkCount() & 0xFFFFU));
  MODBUS_SetInputReg(MB_IR_ERR_CNT,   (uint16_t)(DS18B20_GetErrCount() & 0xFFFFU));
  MODBUS_SetInputReg(MB_IR_LAST_ERR,  (uint16_t)DS18B20_GetLastError());
  MODBUS_SetInputReg(MB_IR_UPTIME_S,  (uint16_t)((HAL_GetTick() / 1000U) & 0xFFFFU));
}

/**
  * @brief 按保持寄存器里的模式/周期驱动 LED，并把实际状态回写到线圈
  * @note  模式 0 = 手动，线圈直接控制；1 = 同步闪烁；2 = 流水灯
  */
static void app_led_pattern(void)
{
  uint16_t period = MODBUS_GetHoldingReg(MB_HR_BLINK_MS);
  uint16_t mode   = MODBUS_GetHoldingReg(MB_HR_MODE);
  uint32_t now    = HAL_GetTick();
  uint8_t  r;
  uint8_t  g;
  uint8_t  b;

  if (period < 50U)                 /* 周期太短视觉上就是常亮，也没有意义 */
  {
    period = 50U;
  }

  if (mode == MB_MODE_BLINK)
  {
    if ((now - g_led_tick) >= period)
    {
      g_led_tick = now;
      g_led_step ^= 1U;
    }
    r = g_led_step;
    g = g_led_step;
    b = g_led_step;
  }
  else if (mode == MB_MODE_FLOW)
  {
    if ((now - g_led_tick) >= period)
    {
      g_led_tick = now;
      g_led_step = (uint8_t)((g_led_step + 1U) % 3U);
    }
    r = (g_led_step == 0U) ? 1U : 0U;
    g = (g_led_step == 1U) ? 1U : 0U;
    b = (g_led_step == 2U) ? 1U : 0U;
  }
  else                              /* MB_MODE_MANUAL */
  {
    r = MODBUS_GetCoil(MB_COIL_LEDR);
    g = MODBUS_GetCoil(MB_COIL_LEDG);
    b = MODBUS_GetCoil(MB_COIL_LEDB);
  }

  app_led_write(LEDR_GPIO_Port, LEDR_Pin, r);
  app_led_write(LEDG_GPIO_Port, LEDG_Pin, g);
  app_led_write(LEDB_GPIO_Port, LEDB_Pin, b);

  /* 自动模式下把实际状态回写，主站读线圈/离散输入时看到的是真实状态 */
  MODBUS_SetCoil(MB_COIL_LEDR, r);
  MODBUS_SetCoil(MB_COIL_LEDG, g);
  MODBUS_SetCoil(MB_COIL_LEDB, b);

  MODBUS_SetDiscreteInput(MB_DI_LEDR, r);
  MODBUS_SetDiscreteInput(MB_DI_LEDG, g);
  MODBUS_SetDiscreteInput(MB_DI_LEDB, b);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Modbus 先初始化，保证第一帧到来时数据区已经就绪 */
  MODBUS_Init();
  /* 把 MODBUS_OnFrame 作为收帧回调交给 RS485 驱动（中断里只拼字节） */
  RS485_Init(MODBUS_OnFrame);
  /* DS18B20 初始化：返回 0 说明 PE3 上没探测到器件（断线 / 没接上拉） */
  (void)DS18B20_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 1) 传感器：非阻塞状态机，1 秒一轮、每轮等 750 ms 转换 */
    DS18B20_Task();

    /* 2) 把最新温度/计数刷进输入寄存器（3x 区），主站用 04 读 */
    app_update_inputs();

    /* 3) 按保持寄存器里的模式驱动 LED，并回写线圈状态 */
    app_led_pattern();

    /* 4) Modbus 从机：攒够一帧就解析并应答（发送在这里做，不在中断里做） */
    MODBUS_Task();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
