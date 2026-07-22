#ifndef __IMU_H__
#define __IMU_H__

#include "board.h"
#include "ti/driverlib/m0p/dl_core.h"

/*============================================================================
 * 数据结构定义 - 六轴传感器数据
 *===========================================================================*/

/**
 * @brief 角度结构体 (单位: 度)
 */
struct SAngle
{
    float Roll;   // 横滚角  (-180° ~ +180°)
    float Pitch;  // 俯仰角  (-180° ~ +180°)
    float Yaw;    // 航向角  (-180° ~ +180°)
};

/**
 * @brief 角速度结构体 (单位: °/s)
 */
struct SGyro
{
    float wx;     // X轴角速度  (±2000°/s)
    float wy;     // Y轴角速度  (±2000°/s)
    float wz;     // Z轴角速度  (±2000°/s)

    // 原始数据(用于调试)
    short rawWx;
    short rawWy;
    short rawWz;
};

/**
 * @brief 加速度结构体 (单位: m/s²)
 */
struct SAccel
{
    float ax;     // X轴加速度  (±16g)
    float ay;     // Y轴加速度  (±16g)
    float az;     // Z轴加速度  (±16g)

    // 原始数据(用于调试)
    short rawAx;
    short rawAy;
    short rawAz;
};

/**
 * @brief 四元数结构体 (归一化单位)
 */
struct SQuat
{
    float q0;     // 四元数 q0
    float q1;     // 四元数 q1
    float q2;     // 四元数 q2
    float q3;     // 四元数 q3
};

/*============================================================================
 * 全局变量声明
 *===========================================================================*/

extern struct SAngle  stcAngle;   // 角度数据
extern struct SGyro   stcGyro;    // 角速度数据
extern struct SAccel  stcAccel;   // 加速度数据
extern struct SQuat   stcQuat;    // 四元数数据

/*============================================================================
 * 函数声明
 *===========================================================================*/

/* 板级初始化 */
void IMU_Init(void);

/* 传感器数据获取接口 */
float Yaw(void);
float Roll(void);
float Pitch(void);

float GyroX(void);
float GyroY(void);
float GyroZ(void);

float AccelX(void);
float AccelY(void);
float AccelZ(void);

float QuatQ0(void);
float QuatQ1(void);
float QuatQ2(void);
float QuatQ3(void);

void CopeSerial2Data(unsigned char ucData);

/* 串口发送（定义在 Board/board.c） */
void uart0_send_SendByte(uint8_t* data, uint32_t len);

/* IMU 校准指令 */
void sendCaliYawCommand(void);
void performCaliBias(void);

#endif
