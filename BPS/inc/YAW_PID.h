#ifndef __YAW_PID_H__
#define __YAW_PID_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    // ── 目标与反馈 ──
    float Yaw_Target;     // 目标Yaw角 (-180° ~ +180°)
    float Yaw;            // 当前Yaw角 (从IMU读取)

    // ── 误差 ──
    float Error;          // 本次角度误差 (已处理±180°环绕)
    float LastError;      // 上次角度误差

    // ── PID三项 ──
    float Integral;       // 积分累计

    // ── 输出 ──
    float Output;         // PID输出 (差速量，叠加到速度环)

    // ── PID系数 ──
    float KP;             // 比例系数
    float KI;             // 积分系数
    float KD;             // 微分系数

    // ── 限幅参数 ──
    float Integral_Max;    // 积分上限
    float Output_Max;      // 输出上限

    // ── 控制开关 ──
    bool  Enable_Flag;        // 是否启用角度环
} Yaw_Control;

extern Yaw_Control Yaw_Circle;  // 全局角度环控制器实例

void YawPID_Init(float kp, float ki, float kd);//初始化角度环控制
void YawPID_Enable(bool Enable);//启用或关闭角度环
void YawPID_SetTarget(float target);//设置目标Yaw角
void YawPID_ResetIntegral(void);//重置积分累计
float YawPID_AngleError(float Target, float Current);//计算误差
void YawPID_Compute(void);//角度环PID计算 ，建议与速度环一致 

/**
 * @brief 将YawPID输出叠加到速度环目标值
 *
 * 典型用法：
 *   Wheel_Left.Velocity_Target  = BaseSpeed + Yaw_Circle.Output;
 *   Wheel_Right.Velocity_Target = BaseSpeed - Yaw_Circle.Output;
 *
 * 电机接线不同时可能需要调换 + / - 符号。
 *
 * @param baseSpeed   基础直线速度 (速度环目标值)
 * @param leftTarget  输出：左轮目标速度指针
 * @param rightTarget 输出：右轮目标速度指针
 */
void YawPID_Apply(float baseSpeed, float *leftTarget, float *rightTarget);

#endif