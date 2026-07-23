#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include "board.h"
#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN)) : (DL_GPIO_clearPins(TB6612_AIN1_PORT,TB6612_AIN1_PIN)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN)) : (DL_GPIO_clearPins(TB6612_AIN2_PORT,TB6612_AIN2_PIN)) )

#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN)) : (DL_GPIO_clearPins(TB6612_BIN1_PORT,TB6612_BIN1_PIN)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN)) : (DL_GPIO_clearPins(TB6612_BIN2_PORT,TB6612_BIN2_PIN)) )


void TB6612_Motor_Stop(void);
void AO_Control(uint8_t Direction, uint32_t Speed);
void BO_Control(uint8_t Direction, uint32_t Speed);

#endif