#ifndef __YAW_PID_H__
#define __YAW_PID_H__

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * 角度PID控制结构体 — 用于维持小车目标航向角(Yaw)
 *===========================================================================*/

/**
 * @brief Yaw角度PID控制器
 *
 * 通过IMU获取当前Yaw角，计算PID输出作为左右轮的差速量，
 * 叠加到速度环目标值上，实现小车航向角的闭环维持。
 */
typedef struct
{
    // ── 目标与反馈 ──
    float Target;         // 目标Yaw角 (-180° ~ +180°)
    float Current;        // 当前Yaw角 (从IMU读取)

    // ── 误差 ──
    float Error;          // 本次角度误差 (已处理±180°环绕)
    float LastError;      // 上次角度误差

    // ── PID三项 ──
    float Integral;       // 积分累计

    // ── 输出 ──
    float Output;         // PID输出 (差速量，叠加到速度环)

    // ── PID系数 ──
    float Kp;             // 比例系数
    float Ki;             // 积分系数
    float Kd;             // 微分系数

    // ── 限幅参数 ──
    float IntegralMax;    // 积分上限
    float OutputMax;      // 输出上限 (差速绝对值)

    // ── 控制开关 ──
    bool  Enabled;        // 是否启用角度环
} YawPID_t;

/*============================================================================
 * 全局变量声明
 *===========================================================================*/

extern YawPID_t gYawPID;  // 全局角度环控制器实例

/*============================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief 初始化角度环PID控制器
 *
 * @param kp  比例系数
 * @param ki  积分系数
 * @param kd  微分系数
 */
void YawPID_Init(float kp, float ki, float kd);

/**
 * @brief 设置目标Yaw角
 *
 * @param target 目标航向角，范围 -180° ~ +180°
 */
void YawPID_SetTarget(float target);

/**
 * @brief 角度环PID计算 (每个控制周期调用一次)
 *
 * 内部从 IMU Yaw() 读取当前角度，计算误差 (处理±180°环绕)，
 * 完成PID运算，Output 存入 gYawPID.Output。
 *
 * @note 调用频率建议与速度环一致 (50~100Hz)
 */
void YawPID_Compute(void);

/**
 * @brief 将YawPID输出叠加到速度环目标值
 *
 * 典型用法：
 *   Wheel_Left.Velocity_Target  = BaseSpeed + gYawPID.Output;
 *   Wheel_Right.Velocity_Target = BaseSpeed - gYawPID.Output;
 *
 * 电机接线不同时可能需要调换 + / - 符号。
 *
 * @param baseSpeed   基础直线速度 (速度环目标值)
 * @param leftTarget  输出：左轮目标速度指针
 * @param rightTarget 输出：右轮目标速度指针
 */
void YawPID_ApplyToWheels(float baseSpeed, float *leftTarget, float *rightTarget);

/**
 * @brief 启用/禁用角度环
 */
void YawPID_Enable(bool enable);

/**
 * @brief 重置积分累计 (如到达目标后或手动干预时调用)
 */
void YawPID_ResetIntegral(void);

/**
 * @brief 归一化角度到 [-180, 180) 区间
 *
 * @param angle 原始角度
 * @return      归一化后的角度
 */
float YawPID_NormalizeAngle(float angle);

/**
 * @brief 计算最短路径角度误差 (已处理±180°环绕)
 *
 * @param target  目标角度
 * @param current 当前角度
 * @return        最短路径误差 (-180° ~ +180°)
 */
float YawPID_AngleError(float target, float current);

#endif /* __YAW_PID_H__ */
