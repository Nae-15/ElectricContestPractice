#ifndef TRACK_H_
#define TRACK_H_

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define Sensor_1 TRACKING_PIN_1_PIN
#define Sensor_2 TRACKING_PIN_2_PIN
#define Sensor_3 TRACKING_PIN_3_PIN
#define Sensor_4 TRACKING_PIN_4_PIN
#define Sensor_5 TRACKING_PIN_5_PIN
#define Sensor_6 TRACKING_PIN_6_PIN

/*
 * 保留当前硬件上已经使用的高电平有效逻辑。若静态标定确认模块为低电平
 * 检黑线，只改此处为 1；不要再同时改权重算法或 GPIO 反相配置。
 */
#define TRACK_ACTIVE_LOW    0U
#define TRACK_ALL_ACTIVE_BITS 0x3FU

typedef struct
{
    uint8_t RawBits;       /* bit0..bit5：GPIO 原始电平 */
    uint8_t ActiveBits;    /* bit0..bit5：1 表示该路检测到黑线 */
    uint8_t ActiveCount;
    uint8_t PositionX10;   /* 10..60，中心为 35；丢线时为 0 */
} TRACK_SAMPLE;

/* 20s无球模式：直接读取同一时刻的六路GPIO，不经过时间滤波。 */
TRACK_SAMPLE Track_Read(void);
TRACK_SAMPLE Track_ReadRaw(void);

/* 30s载球模式：1ms更新滞回滤波，20ms控制环读取稳定快照。 */
void Track_Update1ms(void);
TRACK_SAMPLE Track_ReadStable(void);
void Track_ResetStable(void);

uint8_t Track_Get(void);
uint8_t Track_GetBits(void);

#endif