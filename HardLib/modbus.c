/**
  ******************************************************************************
  * @file    modbus.c
  * @brief   Modbus RTU 从机实现
  *
  *  帧边界怎么判定（Modbus RTU 的核心难点）：
  *    HAL 的 IDLE 中断在总线空闲 1 个字符时间就触发，而 Modbus 规定帧内字节
  *    间隔最多 1.5 个字符、帧间必须静默 3.5 个字符。也就是说 IDLE 有可能在
  *    一帧中间就提前触发，把一帧拆成几片。
  *    中断只拼接片段，主循环确认 DMA 没有尚未交付的字节，并等够帧间静默后解析。
  *    时间差先用 DWT 原始周期数相减，避免计数器回绕时误判帧结束。
  *    好处是不额外占用定时器，也不用改 CubeMX 配置。
  ******************************************************************************
  */

#include "modbus.h"
#include "rs485.h"
#include <string.h>

#define MB_ASM_SIZE        256U      /* 帧组装缓冲 */
#define MB_REQ_SIZE        256U      /* 请求帧快照 */
#define MB_TX_SIZE         256U      /* 应答帧缓冲 */

/* USART2 固定为 115200 bps。高于 19200 bps 时采用推荐的固定 t3.5 = 1.75 ms。
   IDLE 回调已经比最后一个字节晚约一个字符，这里再等完整静默时间，留出余量。 */
#define MB_T35_US          1750U

/* 请求帧长度下界：地址 + 功能码 + CRC16 */
#define MB_MIN_FRAME       4U

/* ---------------- 内部状态 ---------------- */
static uint8_t  s_asm[MB_ASM_SIZE];
static volatile uint16_t s_asm_len = 0U;
static volatile uint32_t s_last_cycles = 0U;
static uint32_t s_t35_cycles;

static uint8_t  s_req[MB_REQ_SIZE];
static uint8_t  s_tx[MB_TX_SIZE];

/* ---------------- 数据区 ---------------- */
static uint8_t  s_coils[MB_COIL_COUNT];
static uint8_t  s_discrete[MB_DI_COUNT];
static uint16_t s_input[MB_IR_COUNT];
static uint16_t s_holding[MB_HR_COUNT];

/* ---------------- 统计 ---------------- */
static uint32_t s_frame_cnt    = 0U;
static uint32_t s_crc_err_cnt  = 0U;
static uint32_t s_drop_cnt     = 0U;

/* ==========================================================================
 *  时间戳
 * ========================================================================== */
static inline uint32_t mb_cycles(void)
{
  return DWT->CYCCNT;
}

/* ==========================================================================
 *  CRC16 / Modbus
 *  多项式 x^16 + x^15 + x^2 + 1，初值 0xFFFF，反写形式 0xA001，
 *  发送时低字节在前 —— 这三点写错任何一个，主站都会报 CRC 错误。
 * ========================================================================== */
static uint16_t mb_crc16(const uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0xFFFFU;

  while (len-- != 0U)
  {
    uint8_t i;

    crc ^= *buf++;

    for (i = 0U; i < 8U; i++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

/**
  * @brief 在 s_tx[len] 处补 CRC 并返回整帧长度
  */
static uint16_t mb_finish(uint16_t len)
{
  uint16_t crc = mb_crc16(s_tx, len);

  s_tx[len++] = (uint8_t)(crc & 0xFFU);        /* 低字节在前 */
  s_tx[len++] = (uint8_t)(crc >> 8);

  return len;
}

static uint16_t mb_exception(uint8_t addr, uint8_t func, uint8_t code)
{
  s_tx[0] = addr;
  s_tx[1] = (uint8_t)(func | 0x80U);           /* 功能码最高位置 1 表示异常应答 */
  s_tx[2] = code;

  return mb_finish(3U);
}

/* ==========================================================================
 *  功能码 01 / 02：读线圈、读离散输入
 * ========================================================================== */
static uint16_t mb_read_bits(const uint8_t *req, uint8_t addr, uint8_t func)
{
  uint16_t start = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
  uint16_t qty   = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);
  const uint8_t *src;
  uint16_t count;
  uint16_t nbytes;
  uint16_t i;

  if (func == MB_FC_READ_COILS)
  {
    src   = s_coils;
    count = MB_COIL_COUNT;
  }
  else
  {
    src   = s_discrete;
    count = MB_DI_COUNT;
  }

  if ((qty == 0U) || (qty > 2000U))
  {
    return mb_exception(addr, func, MB_EX_ILLEGAL_VALUE);
  }

  if (((uint32_t)start + qty) > count)
  {
    return mb_exception(addr, func, MB_EX_ILLEGAL_ADDR);
  }

  nbytes = (uint16_t)((qty + 7U) / 8U);

  s_tx[0] = addr;
  s_tx[1] = func;
  s_tx[2] = (uint8_t)nbytes;
  memset(&s_tx[3], 0, nbytes);

  /* 位打包：第一个位放字节的 bit0，低位在前 */
  for (i = 0U; i < qty; i++)
  {
    if (src[start + i] != 0U)
    {
      s_tx[3U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
    }
  }

  return mb_finish((uint16_t)(3U + nbytes));
}

/* ==========================================================================
 *  功能码 03 / 04：读保持寄存器、读输入寄存器
 * ========================================================================== */
static uint16_t mb_read_regs(const uint8_t *req, uint8_t addr, uint8_t func)
{
  uint16_t start = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
  uint16_t qty   = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);
  const uint16_t *src;
  uint16_t count;
  uint16_t i;

  if (func == MB_FC_READ_HOLDING)
  {
    src   = s_holding;
    count = MB_HR_COUNT;
  }
  else
  {
    src   = s_input;
    count = MB_IR_COUNT;
  }

  if ((qty == 0U) || (qty > 125U))             /* 规范上限：125 个寄存器 */
  {
    return mb_exception(addr, func, MB_EX_ILLEGAL_VALUE);
  }

  if (((uint32_t)start + qty) > count)
  {
    return mb_exception(addr, func, MB_EX_ILLEGAL_ADDR);
  }

  s_tx[0] = addr;
  s_tx[1] = func;
  s_tx[2] = (uint8_t)(qty * 2U);

  for (i = 0U; i < qty; i++)
  {
    s_tx[3U + (i * 2U)] = (uint8_t)(src[start + i] >> 8);      /* 高字节在前 */
    s_tx[4U + (i * 2U)] = (uint8_t)(src[start + i] & 0xFFU);
  }

  return mb_finish((uint16_t)(3U + (qty * 2U)));
}

/* ==========================================================================
 *  功能码 05：写单个线圈
 * ========================================================================== */
static uint16_t mb_write_coil(const uint8_t *req, uint8_t addr)
{
  uint16_t start = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
  uint16_t value = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);

  if ((value != 0xFF00U) && (value != 0x0000U))
  {
    return mb_exception(addr, MB_FC_WRITE_COIL, MB_EX_ILLEGAL_VALUE);
  }

  if (start >= MB_COIL_COUNT)
  {
    return mb_exception(addr, MB_FC_WRITE_COIL, MB_EX_ILLEGAL_ADDR);
  }

  s_coils[start] = (value == 0xFF00U) ? 1U : 0U;

  /* 正常应答就是把请求原样回送 */
  memcpy(s_tx, req, 6U);

  return mb_finish(6U);
}

/* ==========================================================================
 *  功能码 06：写单个保持寄存器
 * ========================================================================== */
static uint16_t mb_write_reg(const uint8_t *req, uint8_t addr)
{
  uint16_t start = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
  uint16_t value = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);

  if (start >= MB_HR_COUNT)
  {
    return mb_exception(addr, MB_FC_WRITE_REG, MB_EX_ILLEGAL_ADDR);
  }

  s_holding[start] = value;

  memcpy(s_tx, req, 6U);

  return mb_finish(6U);
}

/* ==========================================================================
 *  功能码 0F：写多个线圈
 * ========================================================================== */
static uint16_t mb_write_coils(const uint8_t *req, uint16_t len, uint8_t addr)
{
  uint16_t start     = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
  uint16_t qty       = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);
  uint8_t  bytecount = req[6];
  uint16_t i;

  if ((qty == 0U) || (qty > 1968U))
  {
    return mb_exception(addr, MB_FC_WRITE_COILS, MB_EX_ILLEGAL_VALUE);
  }

  if (((uint32_t)start + qty) > MB_COIL_COUNT)
  {
    return mb_exception(addr, MB_FC_WRITE_COILS, MB_EX_ILLEGAL_ADDR);
  }

  /* 字节数必须与位数量匹配，整帧长度也要对得上 */
  if ((bytecount != (uint8_t)((qty + 7U) / 8U)) ||
      (len != (uint16_t)(7U + bytecount + 2U)))
  {
    return mb_exception(addr, MB_FC_WRITE_COILS, MB_EX_ILLEGAL_VALUE);
  }

  for (i = 0U; i < qty; i++)
  {
    uint8_t bit = (uint8_t)((req[7U + (i / 8U)] >> (i % 8U)) & 0x01U);
    s_coils[start + i] = bit;
  }

  memcpy(s_tx, req, 6U);

  return mb_finish(6U);
}

/* ==========================================================================
 *  功能码 10：写多个保持寄存器
 * ========================================================================== */
static uint16_t mb_write_regs(const uint8_t *req, uint16_t len, uint8_t addr)
{
  uint16_t start     = (uint16_t)(((uint16_t)req[2] << 8) | req[3]);
  uint16_t qty       = (uint16_t)(((uint16_t)req[4] << 8) | req[5]);
  uint8_t  bytecount = req[6];
  uint16_t i;

  if ((qty == 0U) || (qty > 123U))
  {
    return mb_exception(addr, MB_FC_WRITE_REGS, MB_EX_ILLEGAL_VALUE);
  }

  if (((uint32_t)start + qty) > MB_HR_COUNT)
  {
    return mb_exception(addr, MB_FC_WRITE_REGS, MB_EX_ILLEGAL_ADDR);
  }

  if ((bytecount != (uint8_t)(qty * 2U)) ||
      (len != (uint16_t)(7U + bytecount + 2U)))
  {
    return mb_exception(addr, MB_FC_WRITE_REGS, MB_EX_ILLEGAL_VALUE);
  }

  for (i = 0U; i < qty; i++)
  {
    s_holding[start + i] = (uint16_t)(((uint16_t)req[7U + (i * 2U)] << 8) |
                                       (uint16_t)req[8U + (i * 2U)]);
  }

  memcpy(s_tx, req, 6U);

  return mb_finish(6U);
}

/* ==========================================================================
 *  分发
 * ========================================================================== */
static uint16_t mb_dispatch(const uint8_t *req, uint16_t len, uint8_t addr, uint8_t func)
{
  switch (func)
  {
    case MB_FC_READ_COILS:
    case MB_FC_READ_DISCRETE:
      if (len != 8U)
      {
        return mb_exception(addr, func, MB_EX_ILLEGAL_VALUE);
      }
      return mb_read_bits(req, addr, func);

    case MB_FC_READ_HOLDING:
    case MB_FC_READ_INPUT:
      if (len != 8U)
      {
        return mb_exception(addr, func, MB_EX_ILLEGAL_VALUE);
      }
      return mb_read_regs(req, addr, func);

    case MB_FC_WRITE_COIL:
      if (len != 8U)
      {
        return mb_exception(addr, func, MB_EX_ILLEGAL_VALUE);
      }
      return mb_write_coil(req, addr);

    case MB_FC_WRITE_REG:
      if (len != 8U)
      {
        return mb_exception(addr, func, MB_EX_ILLEGAL_VALUE);
      }
      return mb_write_reg(req, addr);

    case MB_FC_WRITE_COILS:
      if (len < 9U)
      {
        return mb_exception(addr, func, MB_EX_ILLEGAL_VALUE);
      }
      return mb_write_coils(req, len, addr);

    case MB_FC_WRITE_REGS:
      if (len < 9U)
      {
        return mb_exception(addr, func, MB_EX_ILLEGAL_VALUE);
      }
      return mb_write_regs(req, len, addr);

    default:
      return mb_exception(addr, func, MB_EX_ILLEGAL_FUNC);
  }
}

/**
  * @brief 解析一帧完整请求
  */
static void mb_process(const uint8_t *req, uint16_t len)
{
  uint8_t  addr;
  uint8_t  func;
  uint16_t crc_rx;
  uint16_t crc_calc;
  uint16_t resp_len;

  if (len < MB_MIN_FRAME)
  {
    s_drop_cnt++;
    return;
  }

  crc_rx   = (uint16_t)((uint16_t)req[len - 2U] | ((uint16_t)req[len - 1U] << 8));
  crc_calc = mb_crc16(req, (uint16_t)(len - 2U));

  if (crc_rx != crc_calc)
  {
    /* CRC 不对必须静默丢弃，绝不能回任何东西，否则会和别的从机撞总线 */
    s_crc_err_cnt++;
    return;
  }

  addr = req[0];
  func = req[1];

  /* 不是本机地址也不是广播：直接忽略 */
  if ((addr != MB_SLAVE_ADDR) && (addr != MB_BROADCAST_ADDR))
  {
    return;
  }

  /* 串行广播只允许写操作。 */
  if ((addr == MB_BROADCAST_ADDR) &&
      (func != MB_FC_WRITE_COIL) && (func != MB_FC_WRITE_REG) &&
      (func != MB_FC_WRITE_COILS) && (func != MB_FC_WRITE_REGS))
  {
    return;
  }

  s_frame_cnt++;

  resp_len = mb_dispatch(req, len, addr, func);

  /* 广播地址只执行不回复 */
  if ((resp_len > 0U) && (addr != MB_BROADCAST_ADDR))
  {
    (void)RS485_Send(s_tx, resp_len);
  }
}

/* ==========================================================================
 *  对外接口
 * ========================================================================== */

void MODBUS_Init(void)
{
  /* DWT 周期计数器做时间戳；ds18b20 也会打开，重复设置无副作用 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U)
  {
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  }

  memset(s_coils, 0, sizeof(s_coils));
  memset(s_discrete, 0, sizeof(s_discrete));
  memset(s_input, 0, sizeof(s_input));
  memset(s_holding, 0, sizeof(s_holding));

  s_asm_len  = 0U;
  s_t35_cycles = (SystemCoreClock / 1000000U) * MB_T35_US;
  s_last_cycles = mb_cycles();
  s_frame_cnt = 0U;
  s_crc_err_cnt = 0U;
  s_drop_cnt = 0U;

  /* 默认：手动模式，闪烁周期 500 ms */
  s_holding[MB_HR_BLINK_MS] = 500U;
  s_holding[MB_HR_MODE]     = MB_MODE_MANUAL;
}

void MODBUS_OnFrame(const uint8_t *data, uint16_t len)
{
  if ((data == NULL) || (len == 0U))
  {
    return;
  }

  /* 两次回调之间包含后一片的传输时间，不能直接把它当成总线静默时间。
     帧结束统一由 MODBUS_Task 判定，避免长片段到来时误丢掉前半帧。 */
  if (((uint32_t)s_asm_len + len) > MB_ASM_SIZE)
  {
    s_asm_len = 0U;                            /* 溢出：整帧作废，等下一帧 */
    s_drop_cnt++;
    return;
  }

  memcpy(&s_asm[s_asm_len], data, len);
  s_asm_len = (uint16_t)(s_asm_len + len);
  s_last_cycles = mb_cycles();
}

void MODBUS_Task(void)
{
  uint16_t len;
  uint32_t primask;
  uint32_t now;

  if ((s_asm_len == 0U) || (RS485_IsBusy() != 0U))
  {
    return;
  }

  /* 检查静默和取快照放在同一个临界区，防止检查后又有片段中断插入。 */
  primask = __get_PRIMASK();
  __disable_irq();
  now = mb_cycles();

  /* DMA 可能正在接收下一片，尚未产生 IDLE 回调；此时不能解析已有的前半帧。 */
  if (RS485_IsRxActive() != 0U)
  {
    s_last_cycles = now;
    __set_PRIMASK(primask);
    return;
  }

  if ((s_asm_len == 0U) || ((uint32_t)(now - s_last_cycles) < s_t35_cycles))
  {
    __set_PRIMASK(primask);
    return;
  }

  len       = s_asm_len;
  s_asm_len = 0U;
  memcpy(s_req, s_asm, len);
  __set_PRIMASK(primask);

  mb_process(s_req, len);
}

/* ---------------- 数据区访问 ---------------- */

uint8_t MODBUS_GetCoil(uint16_t addr)
{
  return (addr < MB_COIL_COUNT) ? s_coils[addr] : 0U;
}

void MODBUS_SetCoil(uint16_t addr, uint8_t on)
{
  if (addr < MB_COIL_COUNT)
  {
    s_coils[addr] = (on != 0U) ? 1U : 0U;
  }
}

uint8_t MODBUS_GetDiscreteInput(uint16_t addr)
{
  return (addr < MB_DI_COUNT) ? s_discrete[addr] : 0U;
}

void MODBUS_SetDiscreteInput(uint16_t addr, uint8_t on)
{
  if (addr < MB_DI_COUNT)
  {
    s_discrete[addr] = (on != 0U) ? 1U : 0U;
  }
}

uint16_t MODBUS_GetInputReg(uint16_t addr)
{
  return (addr < MB_IR_COUNT) ? s_input[addr] : 0U;
}

void MODBUS_SetInputReg(uint16_t addr, uint16_t val)
{
  if (addr < MB_IR_COUNT)
  {
    s_input[addr] = val;
  }
}

uint16_t MODBUS_GetHoldingReg(uint16_t addr)
{
  return (addr < MB_HR_COUNT) ? s_holding[addr] : 0U;
}

void MODBUS_SetHoldingReg(uint16_t addr, uint16_t val)
{
  if (addr < MB_HR_COUNT)
  {
    s_holding[addr] = val;
  }
}

uint32_t MODBUS_GetFrameCount(void)
{
  return s_frame_cnt;
}

uint32_t MODBUS_GetCrcErrorCount(void)
{
  return s_crc_err_cnt;
}

uint32_t MODBUS_GetDropCount(void)
{
  return s_drop_cnt;
}
