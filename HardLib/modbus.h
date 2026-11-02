/**
  ******************************************************************************
  * @file    modbus.h
  * @brief   Modbus RTU 从机（USART2 + RS485 半双工）
  * @note    支持功能码 01/02/03/04/05/06/0F/10，地址表见下方宏定义。
  *          从机严格被动应答，绝不主动往总线上发数据。
  ******************************************************************************
  */

#ifndef __MODBUS_H__
#define __MODBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ---------------- 从机地址 ---------------- */
#define MB_SLAVE_ADDR           1U      /* 本机地址，1~247 */
#define MB_BROADCAST_ADDR       0U      /* 广播地址：执行但不应答 */

/* ---------------- 数据区容量 ---------------- */
#define MB_COIL_COUNT           8U      /* 0x 线圈，读写 */
#define MB_DI_COUNT             8U      /* 1x 离散输入，只读 */
#define MB_IR_COUNT             8U      /* 3x 输入寄存器，只读 */
#define MB_HR_COUNT             8U      /* 4x 保持寄存器，读写 */

/* ---------------- 线圈映射（0x0000 起） ---------------- */
#define MB_COIL_LEDR            0U
#define MB_COIL_LEDG            1U
#define MB_COIL_LEDB            2U

/* ---------------- 离散输入映射（0x0000 起） ---------------- */
#define MB_DI_LEDR              0U
#define MB_DI_LEDG              1U
#define MB_DI_LEDB              2U

/* ---------------- 输入寄存器映射（3x0000 起） ---------------- */
#define MB_IR_TEMP_X100         0U      /* 温度 ×100，有符号：2468 = 24.68 ℃ */
#define MB_IR_PRESENT           1U      /* 1 = 器件在线 */
#define MB_IR_OK_CNT            2U      /* 成功读数次数，低 16 位 */
#define MB_IR_ERR_CNT           3U      /* 失败次数，低 16 位 */
#define MB_IR_LAST_ERR          4U      /* 最近错误码：0 正常 1 无应答 2 CRC 3 异常值 */
#define MB_IR_UPTIME_S          5U      /* 运行时间（秒），低 16 位 */

/* ---------------- 保持寄存器映射（4x0000 起） ---------------- */
#define MB_HR_BLINK_MS          0U      /* 闪烁周期（ms），最小按 50 处理 */
#define MB_HR_MODE              1U      /* 0 手动（线圈直控）/ 1 同步闪烁 / 2 流水灯 */

/* ---------------- 模式 ---------------- */
#define MB_MODE_MANUAL          0U
#define MB_MODE_BLINK           1U
#define MB_MODE_FLOW            2U

/* ---------------- 功能码 ---------------- */
#define MB_FC_READ_COILS        0x01U
#define MB_FC_READ_DISCRETE     0x02U
#define MB_FC_READ_HOLDING      0x03U
#define MB_FC_READ_INPUT        0x04U
#define MB_FC_WRITE_COIL        0x05U
#define MB_FC_WRITE_REG         0x06U
#define MB_FC_WRITE_COILS       0x0FU
#define MB_FC_WRITE_REGS        0x10U

/* ---------------- 异常码 ---------------- */
#define MB_EX_ILLEGAL_FUNC      0x01U
#define MB_EX_ILLEGAL_ADDR      0x02U
#define MB_EX_ILLEGAL_VALUE     0x03U

/**
  * @brief 初始化数据区与帧组装状态
  * @note  必须在 RS485_Init() 之前调用
  */
void MODBUS_Init(void);

/**
  * @brief 收帧回调，直接传给 RS485_Init()
  * @note  运行在 USART2 中断上下文，只拼接 DMA / IDLE 片段，不做协议解析
  */
void MODBUS_OnFrame(const uint8_t *data, uint16_t len);

/**
  * @brief 主循环任务：确认接收停止并等够帧间静默后解析并应答
  * @note  必须放在主循环里反复调用；发送动作在这里做，不能在中断里做
  */
void MODBUS_Task(void);

/* ---------------- 应用层访问数据区的接口 ---------------- */
uint8_t  MODBUS_GetCoil(uint16_t addr);
void     MODBUS_SetCoil(uint16_t addr, uint8_t on);
uint8_t  MODBUS_GetDiscreteInput(uint16_t addr);
void     MODBUS_SetDiscreteInput(uint16_t addr, uint8_t on);
uint16_t MODBUS_GetInputReg(uint16_t addr);
void     MODBUS_SetInputReg(uint16_t addr, uint16_t val);
uint16_t MODBUS_GetHoldingReg(uint16_t addr);
void     MODBUS_SetHoldingReg(uint16_t addr, uint16_t val);

/* ---------------- 统计信息，便于调试 ---------------- */
uint32_t MODBUS_GetFrameCount(void);      /* 通过 CRC 且地址匹配的帧数 */
uint32_t MODBUS_GetCrcErrorCount(void);   /* CRC 校验失败次数 */
uint32_t MODBUS_GetDropCount(void);       /* 长度非法 / 溢出丢弃次数 */

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_H__ */
