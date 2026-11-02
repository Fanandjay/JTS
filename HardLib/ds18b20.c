/**
  ******************************************************************************
  * @file    ds18b20.c
  * @brief   DS18B20 单总线温度传感器驱动（PE3，三线供电）
  *
  *  时序实现要点：
  *    1) 单总线是双向的，DQ 必须是开漏输出。写 1 时引脚高阻、靠上拉电阻回高；
  *       写 0 时拉低到地。这样 MCU 和从机谁拉低都不会打架（推挽会短路）。
  *    2) 1-Wire 的时隙单位是微秒，HAL_Delay 的粒度是毫秒，完全不能用。
  *       这里用 DWT 的周期计数器做忙等延时，精度 ±1 个指令周期，不占定时器。
  *    3) 时隙期间关中断（每个时隙约 62 us），防止 USART2 / SysTick 中断把
  *       时隙拉长导致采样点偏移。窗口很短，不会影响 RS485 的 DMA 收发。
  *    4) 温度转换要 750 ms，绝对不能阻塞：发完 0x44 立刻返回，用状态机等到点
  *       再回来读暂存器。
  ******************************************************************************
  */

#include "ds18b20.h"
#include <stdio.h>
#include <stddef.h>

/* ---------- 单总线底层操作：直接写 BSRR/读 IDR，比 HAL 库函数少几十条指令 ---------- */
#define OW_LOW()      (DS18B20_GPIO_Port->BSRR = ((uint32_t)DS18B20_Pin << 16U))
#define OW_HIGH()     (DS18B20_GPIO_Port->BSRR =  (uint32_t)DS18B20_Pin)
#define OW_READ()     (((DS18B20_GPIO_Port->IDR & DS18B20_Pin) != 0U) ? 1U : 0U)

/* ---------- ROM 命令 ---------- */
#define DS_CMD_SKIP_ROM       0xCCU
#define DS_CMD_CONVERT_T      0x44U
#define DS_CMD_READ_SCRATCH   0xBEU

/* ---------- 内部状态 ---------- */
#define DS_ST_IDLE            0U   /* 等待下一个轮询周期 */
#define DS_ST_CONVERTING      1U   /* 已发出 0x44，等 750 ms */

static uint8_t  s_state       = DS_ST_IDLE;
static uint32_t s_cycle_tick  = 0U;   /* 上一轮启动时刻 */
static uint32_t s_conv_tick   = 0U;   /* 发出转换命令的时刻 */

static int32_t  s_mdeg        = 0;    /* 最近一次温度，单位 0.001 ℃ */
static uint8_t  s_new         = 0U;   /* 有新数据待取 */
static uint8_t  s_present     = 0U;
static uint8_t  s_last_err    = DS18B20_OK;
static uint32_t s_ok_cnt      = 0U;
static uint32_t s_err_cnt     = 0U;

/* ==========================================================================
 *  微秒级延时（DWT 周期计数器）
 * ========================================================================== */
static void ds_dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* DWT 与 Modbus 共用，不能在通信已经开始后把它清零。 */
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

/**
  * @brief 忙等延时，单位微秒
  * @note  168 MHz 下 1 us = 168 个周期；CYCCNT 是 32 位自由计数器，
  *        无符号减法天然处理回绕，不需要额外判断。
  */
static inline void ds_delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

/* ==========================================================================
 *  单总线时序
 * ========================================================================== */

/**
  * @brief 复位脉冲 + 检测存在脉冲
  * @retval 1 有从机应答，0 无应答（断线或器件损坏）
  */
static uint8_t ow_reset(void)
{
  uint8_t  present;
  uint32_t primask;

  OW_LOW();
  ds_delay_us(500U);                    /* 复位低电平 >= 480 us，放宽到 500 */

  primask = __get_PRIMASK();
  __disable_irq();
  OW_HIGH();                            /* 释放总线，由上拉拉回高 */
  ds_delay_us(70U);                     /* 从机在 15~60 us 后拉低 60~240 us */
  present = (OW_READ() == 0U) ? 1U : 0U;
  __set_PRIMASK(primask);

  ds_delay_us(410U);                    /* 补齐 480 us，让总线恢复空闲 */

  return present;
}

/**
  * @brief 写一个字节，低位先出
  */
static void ow_write_byte(uint8_t byte)
{
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    OW_LOW();                           /* 每个时隙都以拉低开始 */
    if ((byte & 0x01U) != 0U)
    {
      ds_delay_us(2U);                  /* 写 1：短低脉冲，1~15 us */
      OW_HIGH();
      ds_delay_us(60U);                 /* 补齐时隙 */
    }
    else
    {
      ds_delay_us(60U);                 /* 写 0：低电平保持整个时隙 */
      OW_HIGH();
      ds_delay_us(2U);                  /* 恢复时间 */
    }

    __set_PRIMASK(primask);
    byte >>= 1;
  }
}

/**
  * @brief 读一个字节，低位先出
  */
static uint8_t ow_read_byte(void)
{
  uint8_t byte = 0U;
  uint8_t i;

  for (i = 0U; i < 8U; i++)
  {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    OW_LOW();
    ds_delay_us(2U);                    /* 读时隙：拉低 1~2 us 启动 */
    OW_HIGH();
    ds_delay_us(10U);                   /* 15 us 内必须采样 */
    byte >>= 1;
    if (OW_READ() != 0U)
    {
      byte |= 0x80U;
    }
    ds_delay_us(50U);                   /* 补齐时隙 */

    __set_PRIMASK(primask);
  }

  return byte;
}

/**
  * @brief Dallas/Maxim CRC8，多项式 X^8+X^5+X^4+1（0x8C 反写形式）
  */
static uint8_t ds_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0U;
  uint8_t i;

  while (len-- != 0U)
  {
    uint8_t in = *data++;

    for (i = 0U; i < 8U; i++)
    {
      uint8_t mix = (uint8_t)((crc ^ in) & 0x01U);
      crc >>= 1;
      if (mix != 0U)
      {
        crc ^= 0x8CU;
      }
      in >>= 1;
    }
  }

  return crc;
}

/* ==========================================================================
 *  一次完整的会话
 * ========================================================================== */

/**
  * @brief 发出"跳过 ROM + 启动转换"
  */
static uint8_t ds_start_convert(void)
{
  if (ow_reset() == 0U)
  {
    s_present  = 0U;
    s_last_err = DS18B20_ERR_NOPRESENCE;
    return 0U;
  }

  s_present = 1U;

  ow_write_byte(DS_CMD_SKIP_ROM);       /* 总线上只有一个器件，跳过 ROM 匹配 */
  ow_write_byte(DS_CMD_CONVERT_T);      /* 启动一次温度转换 */

  return 1U;
}

/**
  * @brief 读暂存器 9 字节并换算温度，约阻塞 4.5 ms
  */
static uint8_t ds_read_scratchpad(void)
{
  uint8_t  sp[9];
  uint8_t  i;
  int16_t  raw;

  if (ow_reset() == 0U)
  {
    s_present  = 0U;
    s_last_err = DS18B20_ERR_NOPRESENCE;
    return 0U;
  }

  s_present = 1U;

  ow_write_byte(DS_CMD_SKIP_ROM);
  ow_write_byte(DS_CMD_READ_SCRATCH);

  for (i = 0U; i < 9U; i++)
  {
    sp[i] = ow_read_byte();
  }

  /* 第 9 字节是前 8 字节的 CRC8，对不上说明总线有干扰，整帧丢弃 */
  if (ds_crc8(sp, 8U) != sp[8])
  {
    s_last_err = DS18B20_ERR_CRC;
    return 0U;
  }

  raw = (int16_t)(((uint16_t)sp[1] << 8) | (uint16_t)sp[0]);

  /* 0x0550 = 85.0 ℃ 是上电默认值；-127 ℃（0xF810）是器件故障码。
     首次读数就撞上 85.0 几乎必然是转换没成功，直接判错，避免把假温度报出去。
     已经正常读过之后再出现 85.0，则认为是真实温度，不再拦截。 */
  if ((raw == (int16_t)0x0550) && (s_ok_cnt == 0U))
  {
    s_last_err = DS18B20_ERR_RANGE;
    return 0U;
  }

  if (raw == (int16_t)0xF810)
  {
    s_last_err = DS18B20_ERR_RANGE;
    return 0U;
  }

  /* 12 位分辨率，最低位 = 0.0625 ℃；换成毫摄氏度便于整数打印 */
  s_mdeg = ((int32_t)raw * 1000) / 16;

  s_new      = 1U;
  s_last_err = DS18B20_OK;
  s_ok_cnt++;

  return 1U;
}

/* ==========================================================================
 *  对外接口
 * ========================================================================== */

uint8_t DS18B20_Init(void)
{
  ds_dwt_init();

  s_state      = DS_ST_IDLE;
  /* 减一个周期，让上电后第一轮转换立刻启动，不用干等 1 秒 */
  s_cycle_tick = HAL_GetTick() - DS18B20_PERIOD_MS;
  s_mdeg       = 0;
  s_new        = 0U;
  s_ok_cnt     = 0U;
  s_err_cnt    = 0U;
  s_last_err   = DS18B20_OK;

  /* 上电先探一次，方便开机自检 */
  s_present = ow_reset();

  return s_present;
}

void DS18B20_Task(void)
{
  uint32_t now = HAL_GetTick();

  switch (s_state)
  {
    case DS_ST_IDLE:
      /* 到轮询周期就启动一次新的转换 */
      if ((now - s_cycle_tick) >= DS18B20_PERIOD_MS)
      {
        s_cycle_tick = now;

        if (ds_start_convert() != 0U)
        {
          s_conv_tick = now;
          s_state     = DS_ST_CONVERTING;
        }
        else
        {
          s_err_cnt++;                  /* 器件不在线，下个周期再试 */
        }
      }
      break;

    case DS_ST_CONVERTING:
      /* 750 ms 到了再回来读结果 */
      if ((now - s_conv_tick) >= DS18B20_CONV_TIME_MS)
      {
        s_state = DS_ST_IDLE;

        if (ds_read_scratchpad() == 0U)
        {
          s_err_cnt++;
        }
      }
      break;

    default:
      s_state = DS_ST_IDLE;
      break;
  }
}

uint8_t DS18B20_GetTemp(float *temp)
{
  if ((temp == NULL) || (s_new == 0U))
  {
    return 0U;
  }

  *temp = (float)s_mdeg / 1000.0f;
  s_new = 0U;

  return 1U;
}

int DS18B20_FormatTemp(char *buf, uint32_t size)
{
  int32_t  m = s_mdeg;
  uint32_t a;
  const char *sign = "";
  int n;

  if ((buf == NULL) || (size < 8U))
  {
    return 0;
  }

  if (m < 0)
  {
    sign = "-";
    a    = (uint32_t)(-m);
  }
  else
  {
    a = (uint32_t)m;
  }

  n = snprintf(buf, (size_t)size, "%s%lu.%03lu",
               sign,
               (unsigned long)(a / 1000U),
               (unsigned long)(a % 1000U));

  return (n > 0) ? n : 0;
}

int32_t DS18B20_GetMilliDeg(void)
{
  return s_mdeg;
}

uint8_t DS18B20_IsPresent(void)
{
  return s_present;
}

uint32_t DS18B20_GetOkCount(void)
{
  return s_ok_cnt;
}

uint32_t DS18B20_GetErrCount(void)
{
  return s_err_cnt;
}

uint8_t DS18B20_GetLastError(void)
{
  return s_last_err;
}
