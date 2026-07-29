#ifndef MY_STEP_MOTOR_H_
#define MY_STEP_MOTOR_H_

#define STEP_MOTOR_UART       UART_2_INST  // 步进电机使用UART2（原Zigbee）
#define STEP_MOTOR_ADDR       0x01U      // 电机地址
#define PULSES_PER_REV        3200U      // 16细分：3200脉冲/圈
#define MOTOR_RPM             500U       // 转速 RPM
#define MOTOR_ACC             50U        // 加速度档位 0~255
#define MOTOR_TIMEOUT_US      200000U    // 超时 200ms

#include "ti_msp_dl_config.h"
#include "board.h"
#include "STEP_MOTOR.h"

#ifdef __cplusplus
extern "C" {
#endif

void StepMotor_Init(void);

#ifdef __cplusplus
}
#endif

#endif