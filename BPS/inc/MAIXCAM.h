#ifndef __MAIXCAM_H_
#define __MAIXCAM_H_

#include "board.h"
#include "ti_msp_dl_config.h"

/* 视觉模块接收数据（32位整数） */
extern volatile int32_t maixcam_data;
extern volatile uint8_t maixcam_rx_ready;   // 1=新数据就绪，由消费者清零

void MAIXCAM_Init(void);

#endif