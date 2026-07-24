#ifndef __KEY_H__
#define __KEY_H__

#define KEY1 KEY_PIN__1_PIN
#define KEY2 KEY_PIN__2_PIN
#define KEY3 KEY_PIN__3_PIN

#include "ti_msp_dl_config.h"
#include "../Board/board.h"

uint8_t Key_Get(void);

#endif
