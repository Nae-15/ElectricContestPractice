#ifndef __ZIGBEE_H_
#define __ZIGBEE_H_

#include "board.h"
#include "ti_msp_dl_config.h"

extern volatile float zigbee_data[3];
extern volatile uint8_t rx_debug_flag;

void Zigbee_Init(void);

#endif