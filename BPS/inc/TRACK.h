#ifndef TRACK_H_
#define TRACK_H_

#define Sensor_1 TRACKING_PIN_1_PIN
#define Sensor_2 TRACKING_PIN_2_PIN
#define Sensor_3 TRACKING_PIN_3_PIN
#define Sensor_4 TRACKING_PIN_4_PIN
#define Sensor_5 TRACKING_PIN_5_PIN
#define Sensor_6 TRACKING_PIN_6_PIN
#define Sensor_7 TRACKING_PIN_7_PIN

#include "ti_msp_dl_config.h"

uint8_t Track_Get(void);
uint8_t Track_GetBits(void);

#endif