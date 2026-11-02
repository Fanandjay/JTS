/**
  ******************************************************************************
  * @file    ds18b20.h
  * @brief   DS18B20 单总线温度传感器驱动（PE3，三线供电）
  * @note    DQ 必须配置为 GPIO_MODE_OUTPUT_OD + 外部 4.7k 上拉到 3.3V；
  *          时序用 DWT 周期计数器做微秒延时，不占用任何定时器。
  ******************************************************************************
  */

#ifndef __DS18B20_H__
#define __DS18B20_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 轮询周期：每多少毫秒启动一次温度转换 */
#define DS18B20_PERIOD_MS        1000U
/* 12 位分辨率下的最大转换时间，数据手册标称 750 ms */
#define DS18B20_CONV_TIME_MS     750U

/* 状态码 */
#define DS18B20_OK               0U
#define DS18B20_ERR_NOPRESENCE   1U   /* 复位后无存在脉冲：断线 / 器件损坏 */
#define DS18B20_ERR_CRC          2U   /* CRC8 校验失败：总线干扰 */
#define DS18B20_ERR_RANGE        3U   /* 读到 85.0℃ 上电默认值或 -127℃ 错误码 */

/**
  * @brief 初始化：启动 DWT 计数器，并做一次复位检测
  * @retval 1 表示总线上检测到器件，0 表示没有
  * @note  必须在 MX_GPIO_Init() 之后调用
  */
uint8_t DS18B20_Init(void);

/**
  * @brief 周期任务：内部是"发转换 -> 等 750ms -> 读暂存器"的非阻塞状态机
  * @note  在主循环里反复调用即可，本身不会阻塞超过 5 ms（读暂存器期间约 4.5 ms）
  */
void DS18B20_Task(void);

/**
  * @brief 取一次新温度（取出后清除新数据标志）
  * @param temp 输出，单位 ℃
  * @retval 1 有新数据，0 本次没有更新
  */
uint8_t DS18B20_GetTemp(float *temp);

/**
  * @brief 把最近一次温度格式化成字符串，例如 "25.437" / "-3.062"
  * @param buf  输出缓冲区，建议 >= 12 字节
  * @param size 缓冲区大小
  * @retval 写入的字符数（不含 '\0'），失败返回 0
  * @note  专门为了避开 printf 的 %f（newlib-nano 默认不支持浮点格式化）
  */
int DS18B20_FormatTemp(char *buf, uint32_t size);

/**
  * @brief 最近一次温度，单位 0.001 ℃（毫摄氏度），没读到过时返回 0
  */
int32_t DS18B20_GetMilliDeg(void);

/**
  * @brief 器件是否在线（最近一次复位是否有存在脉冲）
  */
uint8_t DS18B20_IsPresent(void);

/**
  * @brief 累计成功读数 / 失败次数，便于现场判断链路质量
  */
uint32_t DS18B20_GetOkCount(void);
uint32_t DS18B20_GetErrCount(void);

/**
  * @brief 上一次失败的错误码（DS18B20_ERR_xxx），成功时为 DS18B20_OK
  */
uint8_t DS18B20_GetLastError(void);

#ifdef __cplusplus
}
#endif

#endif /* __DS18B20_H__ */
