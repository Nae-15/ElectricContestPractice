#include "YAW_PID.h"
#include "IMU.h"

Yaw_Control Yaw_Circle;  // 全局角度环控制器

void YawPID_Init(float KP, float KI, float KD)//初始化角度环控制,默认开启
{
    Yaw_Circle.Yaw_Target   = 0.0f;
    Yaw_Circle.Yaw          = 0.0f;
    Yaw_Circle.Error        = 0.0f;
    Yaw_Circle.LastError    = 0.0f;
    Yaw_Circle.Integral     = 0.0f;
    Yaw_Circle.Output       = 0.0f;

    Yaw_Circle.KP           = KP;
    Yaw_Circle.KI           = KI;
    Yaw_Circle.KD           = KD;

    /* 默认限幅 — 角度环输出不宜过大，避免剧烈转向导致翻车或甩尾 */
    Yaw_Circle.Integral_Max  = 100.0f;   // 积分上限
    Yaw_Circle.Output_Max    = 300.0f;   // 输出差速上限（叠加到速度环）

    Yaw_Circle.Enable_Flag   = true;
}

void YawPID_Enable(bool Enable)//启用或关闭角度环
{
    Yaw_Circle.Enable_Flag = Enable;
    if (!Enable)
    {
        Yaw_Circle.Output   = 0.0f;
        Yaw_Circle.Integral = 0.0f;
    }
}

void YawPID_SetTarget(float Target)//设置航向角目标值
{
    Yaw_Circle.Yaw_Target = Target;
}

void YawPID_ResetIntegral(void)//重置积分累计,以下场景应调用此函数：1目标Yaw角发生大的跳变 2小车被手动推偏后重新启用角度环 3检测到外部干扰导致积分异常积累
{
    Yaw_Circle.Integral = 0.0f;
}

float YawPID_AngleError(float Target, float Current)//计算最短路径角度误差（陀螺仪:左+右-），正值需右转(CW)，负值需左转(CCW)
{
    float Error = Current - Target;
    if (Error > 180.0f)
    {
        Error -= 360.0f;
    }
    else if (Error < -180.0f)
    {
        Error += 360.0f;
    }

    return Error;
}

void YawPID_Compute(void)
{
    if (!Yaw_Circle.Enable_Flag)
    {
        Yaw_Circle.Output   = 0.0f;
        Yaw_Circle.Integral = 0.0f;
        return;
    }
    
    //1.获取角度值
    Yaw_Circle.Yaw = Yaw(); 
    
    //2.计算路径
    Yaw_Circle.Error = YawPID_AngleError(Yaw_Circle.Yaw_Target, Yaw_Circle.Yaw);
    
    //3.积分
    if (Yaw_Circle.KI == 0.0f)
    {
        Yaw_Circle.Integral = 0.0f;
    }
    else
    {
        bool out_sat_hi = (Yaw_Circle.Output >=  Yaw_Circle.Output_Max);
        bool out_sat_lo = (Yaw_Circle.Output <= -Yaw_Circle.Output_Max);

        if (!((out_sat_hi && Yaw_Circle.Error > 0.0f) ||
              (out_sat_lo && Yaw_Circle.Error < 0.0f)))
        {
            Yaw_Circle.Integral += Yaw_Circle.Error;
        }

        if (Yaw_Circle.Integral >  Yaw_Circle.Integral_Max)
        {
            Yaw_Circle.Integral = Yaw_Circle.Integral_Max;
        }
        else if (Yaw_Circle.Integral < -Yaw_Circle.Integral_Max)
        {
            Yaw_Circle.Integral = -Yaw_Circle.Integral_Max;
        }
    }
    
    //4.计算
    float P_temp = Yaw_Circle.KP * Yaw_Circle.Error;
    float I_temp = Yaw_Circle.KI * Yaw_Circle.Integral;
    float D_temp = Yaw_Circle.KD * (Yaw_Circle.Error - Yaw_Circle.LastError);
    Yaw_Circle.Output = P_temp + I_temp + D_temp;
    
    //5.限幅
    if (Yaw_Circle.Output >  Yaw_Circle.Output_Max)
    {
        Yaw_Circle.Output = Yaw_Circle.Output_Max;
    }
    else if (Yaw_Circle.Output < -Yaw_Circle.Output_Max)
    {
        Yaw_Circle.Output = -Yaw_Circle.Output_Max;
    }

    Yaw_Circle.LastError = Yaw_Circle.Error;
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
void YawPID_Apply(float BaseSpeed, float *LeftTarget, float *RightTarget)
{
    if (!Yaw_Circle.Enable_Flag)
    {
        *LeftTarget  = BaseSpeed;
        *RightTarget = BaseSpeed;
        return;
    }

    /* 差速叠加: 正值Output → 左轮加速/右轮减速 → 向右转 */
    *LeftTarget  = BaseSpeed + Yaw_Circle.Output;
    *RightTarget = BaseSpeed - Yaw_Circle.Output;
}