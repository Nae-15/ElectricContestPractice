#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "ti_msp_dl_config.h"

typedef enum 
{
    FORWARD,  // 正向，0
    REVERSAL  // 反向，1
} ENCODER_DIR;

typedef struct 
{
    volatile int32_t TempCount; //保存实时计数值
    int Count;         			//根据定时器时间更新的计数值
    ENCODER_DIR Direction;      //旋转方向
} ENCODER;

void Encoder_Init(void);
void Encoder_Get(uint8_t Id, int *Count, ENCODER_DIR *Direction);

#endif
