#include "YAW_PID.h"
#include "IMU.h"

/*============================================================================
 * 全局实例
 *===========================================================================*/

YawPID_t gYawPID;  // 全局角度环控制器

/*============================================================================
 * 内部辅助函数
 *===========================================================================*/

/**
 * @brief 归一化角度到 [-180.0f, 180.0f) 区间
 *
 * 对IMU输出的 ±180° 环绕进行处理，确保任意角度值落回标准区间。
 */
float YawPID_NormalizeAngle(float angle)
{
    /* 使用 while 循环处理超过一圈的极端情况 */
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 计算最短路径角度误差
 *
 * 目标角 - 当前角，归一化到 (-180°, 180°]，
 * 正值表示需要向右转（CW），负值表示需要向左转（CCW）。
 */
float YawPID_AngleError(float target, float current)
{
    /* 先将目标归一到标准区间 */
    float t = YawPID_NormalizeAngle(target);
    float c = YawPID_NormalizeAngle(current);

    /* 计算原始差值 */
    float error = t - c;

    /* 折到最短路径: -180° ~ +180° */
    if (error > 180.0f)
    {
        error -= 360.0f;
    }
    else if (error < -180.0f)
    {
        error += 360.0f;
    }

    return error;
}

/*============================================================================
 * 公开接口实现
 *===========================================================================*/

/**
 * @brief 初始化角度环PID控制器
 *
 * @param kp  比例系数 — 角度偏差 1° 产生的差速量
 * @param ki  积分系数 — 用于消除稳态误差（如地面不平导致的偏航）
 * @param kd  微分系数 — 抑制超调和振荡（利用IMU角速度可提前抑制）
 */
void YawPID_Init(float kp, float ki, float kd)
{
    gYawPID.Target       = 0.0f;
    gYawPID.Current      = 0.0f;
    gYawPID.Error        = 0.0f;
    gYawPID.LastError    = 0.0f;
    gYawPID.Integral     = 0.0f;
    gYawPID.Output       = 0.0f;

    gYawPID.Kp           = kp;
    gYawPID.Ki           = ki;
    gYawPID.Kd           = kd;

    /* 默认限幅 — 角度环输出不宜过大，避免剧烈转向导致翻车或甩尾 */
    gYawPID.IntegralMax  = 100.0f;   // 积分上限
    gYawPID.OutputMax    = 300.0f;   // 输出差速上限（叠加到速度环）

    gYawPID.Enabled      = true;
}

/**
 * @brief 更新目标Yaw角
 */
void YawPID_SetTarget(float target)
{
    gYawPID.Target = YawPID_NormalizeAngle(target);
}

/**
 * @brief 核心：角度环PID计算
 *
 * 每个控制周期调用一次，内部完成：
 *  1) 从 IMU 读取当前 Yaw
 *  2) 计算最短路径角度误差
 *  3) PID 运算 (含抗饱和与限幅)
 *  4) 输出差速值到 gYawPID.Output
 */
void YawPID_Compute(void)
{
    /* 控制器关闭时输出清零 */
    if (!gYawPID.Enabled)
    {
        gYawPID.Output   = 0.0f;
        gYawPID.Integral = 0.0f;
        return;
    }

    /* ── 1. 获取当前角度 ── */
    gYawPID.Current = Yaw();                                    // 从IMU读取

    /* ── 2. 计算角度误差 (已处理±180°环绕) ── */
    gYawPID.Error = YawPID_AngleError(gYawPID.Target, gYawPID.Current);

    /* ── 3. 积分项 (带输出限幅抗饱和) ── */
    if (gYawPID.Ki != 0.0f)
    {
        /*
         * 抗积分饱和逻辑：
         * - 输出已达上限时，不再累积同向误差
         * - 输出已达下限时，不再累积同向误差
         * 这防止了"积分Windup"引起的长时间超调。
         */
        bool out_sat_hi = (gYawPID.Output >=  gYawPID.OutputMax);
        bool out_sat_lo = (gYawPID.Output <= -gYawPID.OutputMax);

        if (!((out_sat_hi && gYawPID.Error > 0.0f) ||
              (out_sat_lo && gYawPID.Error < 0.0f)))
        {
            gYawPID.Integral += gYawPID.Error;
        }

        /* 积分限幅 */
        if (gYawPID.Integral >  gYawPID.IntegralMax)
        {
            gYawPID.Integral = gYawPID.IntegralMax;
        }
        else if (gYawPID.Integral < -gYawPID.IntegralMax)
        {
            gYawPID.Integral = -gYawPID.IntegralMax;
        }
    }
    else
    {
        gYawPID.Integral = 0.0f;
    }

    /* ── 4. PID 三项合成 ── */
    float p_term = gYawPID.Kp * gYawPID.Error;
    float i_term = gYawPID.Ki * gYawPID.Integral;
    float d_term = gYawPID.Kd * (gYawPID.Error - gYawPID.LastError);

    gYawPID.Output = p_term + i_term + d_term;

    /* ── 5. 输出限幅 ── */
    if (gYawPID.Output >  gYawPID.OutputMax)
    {
        gYawPID.Output = gYawPID.OutputMax;
    }
    else if (gYawPID.Output < -gYawPID.OutputMax)
    {
        gYawPID.Output = -gYawPID.OutputMax;
    }

    /* ── 6. 保存本次误差供下次微分 ── */
    gYawPID.LastError = gYawPID.Error;
}

/**
 * @brief 将YawPID输出叠加到左右轮速度目标
 *
 * 控制逻辑 (差速转向):
 *   - YawPID.Output > 0  → 需要右转(CW) → 左轮加速 + 右轮减速
 *   - YawPID.Output < 0  → 需要左转(CCW) → 左轮减速 + 右轮加速
 *
 * 如果小车实际转向方向相反，交换 + / - 符号即可。
 *
 * @param baseSpeed     基础直线速度值
 * @param leftTarget    左轮目标速度 (输出)
 * @param rightTarget   右轮目标速度 (输出)
 */
void YawPID_ApplyToWheels(float baseSpeed, float *leftTarget, float *rightTarget)
{
    if (!gYawPID.Enabled)
    {
        *leftTarget  = baseSpeed;
        *rightTarget = baseSpeed;
        return;
    }

    /* 差速叠加: 正值Output → 左轮加速/右轮减速 → 向右转 */
    *leftTarget  = baseSpeed + gYawPID.Output;
    *rightTarget = baseSpeed - gYawPID.Output;
}

/**
 * @brief 启用/禁用角度环
 */
void YawPID_Enable(bool enable)
{
    gYawPID.Enabled = enable;
    if (!enable)
    {
        gYawPID.Output   = 0.0f;
        gYawPID.Integral = 0.0f;
    }
}

/**
 * @brief 重置积分累计
 *
 * 以下场景应调用此函数：
 *  - 目标Yaw角发生大的跳变
 *  - 小车被手动推偏后重新启用角度环
 *  - 检测到外部干扰导致积分异常积累
 */
void YawPID_ResetIntegral(void)
{
    gYawPID.Integral = 0.0f;
}
