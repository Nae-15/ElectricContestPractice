#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "BPS/inc/KEY.h"
#include "BPS/inc/ENCODER.h"
#include "BPS/inc/IMU.h"
#include "BPS/lcd/lcd.h"
#include "BPS/inc/MOTOR.h"
#include "BPS/inc/TRACK.h"
#include "BPS/inc/YAW_PID.h"
#include "BPS/inc/ZIGBEE.h"

typedef struct//车轮结构体
{
    int32_t Encoder;        // 编码器当前计数值
    int32_t Encoder_Last;   // 编码器上次计数值
    float Velocity;         // 当前速度
    float Velocity_Last;    // 上次速度
    float Velocity_Target;  // 目标速度
    float Feedforward;      // 前馈系数（左右轮独立校准）
    float PID_Error;        // 输出误差
    float PID_LastError;    // 上次输出误差
    float PID_Integral;     // PID积分值
    float PID_Output;       // PID输出值
} Wheel;
Wheel Wheel_Left,Wheel_Right;

//--------------模式------------------
uint8_t Mode;
#define Mode_Stop 0
#define Mode_Track 1
#define Mode_AngleHold 2

//-----------电机编码器----------------
#define Encoder_Gear_Ratio 20 //减速比1:20
#define Wheel_Radius 24 //轮子半径，单位为mm
#define Encoder_Pulses_Per_Revolution 13 //编码器每转一圈的脉冲数
#define Pi 3.14

//------------速度-----------------
#define Velocity_Ratio 0.8f //速度修正系数
#define Velocity_Interval 50 //测速周期
#define Velocity_Scale 3000 //速度放大系数，将脉冲/tick转为整数级读数
volatile int32_t Velocity_Counter;

//-------------时间-----------------
int time;

//-------------方向-----------------
#define Forward 1
#define Rewerse 0

//-----------路程-------------------
int32_t Distance;

//-----------循迹-------------------
uint8_t Track_Value;
#define TRACK_CENTER    40.0f     // 7路传感器中心加权值（传感器4居中）
#define TRACK_KP        20.0f     // 寻线比例系数（速度差/偏差单位，降低防振荡）

#define TRACK_BASE_SPEED      1300.0f   //寻线基础速度（降速增加直角弯反应时间）
#define TRACK_LOST_SPEED      700.0f    //丢线时降速搜索
#define TRACK_MAX_SPEED       2500.0f   //寻线最大速度限幅
// 直角弯不再用阈值判断，直接按Last_Error符号决定方向（Error=0时走普通丢线）
#define TRACK_MIN_SPEED       300.0f    //寻线最小速度限幅（防堵转）

//-----------直角转弯-------------------
#define TURN_PWM              160       //转弯最大PWM（降速提高精度）
#define TURN_PWM_MIN          70        //转弯最小PWM（克服静摩擦）
#define TURN_DRIVE_FORWARD_DISTANCE  10.0f     //转弯前直走距离(mm)
#define TURN_TARGET_ANGLE            85.0f     //目标转弯角度（度）
#define TURN_DEADBAND         5.0f      //转弯死区（±5°内停转）


typedef enum {
    TURN_None  = 0,
    TURN_Left  = 1,    // 左直角弯 → 车左转90°(CCW)
    TURN_Right = 2,    // 右直角弯 → 车右转90°(CW)
} Turn_Direction;

uint8_t Turn_State;          // 转弯状态
float   Turn_Start_Yaw = 0;  // 转弯起始航向角

//-----------速度环PID--------------------
float PID_KP = 0.10f;   // 比例系数（提高以加快轮速响应）
float PID_KI = 0.02f;   // 积分系数（极慢积分，减少稳态微振）
float PID_KD = 0.0f;    // 微分系数

// 前馈系数：12V供电，左右独立校准（右比左快2.72%，左需补偿）
#define LEFT_FEEDFORWARD  0.0956f
#define RIGHT_FEEDFORWARD 0.0920f
//PID输出限幅
#define PID_MAX_OUTPUT    800.0f // 最大输出
#define PID_MIN_OUTPUT    80.0f  // 最小输出
//PID积分限幅
#define PID_Integral_Max  50.0f  // 积分最大值（小幅慢调）
#define PID_Integral_Min -50.0f  // 积分最小值（对称限幅）

//-----------角度环PID--------------------
float Yaw_Value;                    // 实测航向角
#define YAW_KP            0.5f      // 角度环比例系数（降低防过冲）
#define YAW_KI            0.05f     // 角度环积分系数（提高克服死区）
#define YAW_KD            1.0f      // 角度环微分系数（轻阻尼）
#define YAW_OUTPUT_MAX    150.0f    // 角度环输出限幅（收窄范围）
#define YAW_REVERSE       0         // 方向反转：若小车越调越偏，改为1

//-----------角度维持参数-----------
#define ANGLE_PWM_MIN     80        // 角度修正最低PWM（双轮差速，降低值）
#define ANGLE_PWM_MAX     250       // 角度修正最高PWM
#define ANGLE_DEADBAND    2.0f      // 角度死区（±2°内不调整）
float Target_Yaw;                   // 维持角度目标值

void System_Init(void);     //系统初始化
void Data_Init(void);       //数据初始化
void Show_Init(void);       //显示初始化
void ALL_Init(void);        //全局初始化
void Show_Update(void);     //显示更新
void Velocity_Get(void);    //速度获取
void Distance_Get(void);    //距离获取
static void PID_Get(Wheel *Wheel_Temp);  //PID计算算法
void PID_Contorl(void);     //PID实际调用

void Run_Stop(void);                    //停止模式
void Run_Track(void);                   //沿线循迹模式
void Run_AngleHold(float Target_Yaw);   //角度环维持模式
void Run_Turn(void);                    //原地直角转弯

int main(void)
{
    ALL_Init();
    while(1)
    {
        Show_Update();
        if(time>Velocity_Interval)
        {
            Velocity_Get();

            // Zigbee串口1发送（Track_Value由Run_Track每帧更新）
            {
                char buf[64];
                sprintf(buf, "%d,%.0f,%.0f\n",
                        Track_Value,
                        Wheel_Left.Velocity,
                        Wheel_Right.Velocity);
                uart1_sendString(buf);
            }
        }

        switch (Mode)
        {
            case Mode_Stop:
            {
                Run_Stop();
                break;
            }
            case Mode_Track:    // 循迹模式（丢线直角弯检测在Run_Track内）
            {
                if (Turn_State != TURN_None)
                {
                    Run_Turn();
                }
                else
                {
                    Run_Track();
                }
                break;
            }
            case Mode_AngleHold: // 角度维持模式
            {
                Run_AngleHold(Target_Yaw);
                break;
            }
        }
    }
}

void System_Init(void)//系统初始化
{
    __enable_irq();//MSPM0 Boot ROM 冷启动后关闭了全局中断，启动代码未重新打开，加入 __enable_irq() 确保所有外设中断可用。

    SYSCFG_DL_init();
 
    //系统计时中断
    NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);
    
    Encoder_Init();
    
    LCD_Init();
    LCD_Fill(0, 0, 300, LCD_H, BLACK);

    //IMU占用串口0
    IMU_Init();
    sendCaliYawCommand();

    //角度环初始化：锁定0°航向
    YawPID_Init(YAW_KP, YAW_KI, YAW_KD);
    YawPID_SetTarget(0.0f);
    YawPID_Enable(true);

    //Zigbee占用串口1
    //Zigbee_Init();

    Mode=Mode_Track;
    Turn_State = TURN_None;
}

void Data_Init(void)//数据初始化
{
    //速度数据初始化
    Wheel_Left.Feedforward  = LEFT_FEEDFORWARD;
    Wheel_Right.Feedforward = RIGHT_FEEDFORWARD;
    Wheel_Left.Velocity_Target=2000;
    Wheel_Right.Velocity_Target=2000;

    //编码器数据初始化
    Encoder_Get(1, &Wheel_Left.Encoder, NULL);
    Encoder_Get(2, &Wheel_Right.Encoder, NULL);
    Wheel_Left.Encoder_Last  = Wheel_Left.Encoder;
    Wheel_Right.Encoder_Last = Wheel_Right.Encoder;

    //时钟初始化
    time=0;

    //路程初始化
    Distance=0;

    //角度初始化
    Yaw_Value=0;
    Target_Yaw=0;

    //循迹值初始化
    Track_Value=0;
}

void Show_Init(void)//显示初始化
{
    LCD_ShowString(30,20,"Left_E:",RED,BLACK,16,0);
    LCD_ShowString(150,20,"Right_E:",GREEN,BLACK,16,0); 
    LCD_ShowString(30,40,"Left_V:",RED,BLACK,16,0);
    LCD_ShowString(150,40,"Right_V:",GREEN,BLACK,16,0);
    LCD_ShowString(30,60,"Left_O:",RED,BLACK,16,0);
    LCD_ShowString(150,60,"Right_O:",GREEN,BLACK,16,0); 
    LCD_ShowString(30,80,"KP:",BLUE,BLACK,16,0);
    LCD_ShowString(100,80,"KI:",BLUE,BLACK,16,0);
    LCD_ShowString(170,80,"KD:",BLUE,BLACK,16,0);
    LCD_ShowString(30,100,"TRACK:",WHITE,BLACK,16,0); 
    LCD_ShowString(120,100,"Distance:",WHITE,BLACK,16,0); 
    LCD_ShowString(30,120,"Yaw:",WHITE,BLACK,16,0); 
}

void ALL_Init(void)//全局初始化
{
    System_Init();
    Data_Init();
    Show_Init();
}

void Show_Update(void)//显示更新
{
    LCD_ShowIntNum(100, 20, Wheel_Left.Encoder, 5, RED, BLACK, 16);
    LCD_ShowIntNum(220, 20, Wheel_Right.Encoder, 5, GREEN, BLACK, 16);
    LCD_ShowIntNum(100, 40, Wheel_Left.Velocity, 5, RED, BLACK, 16);
    LCD_ShowIntNum(220, 40, Wheel_Right.Velocity, 5, GREEN, BLACK, 16);
    LCD_ShowIntNum(100, 60, Wheel_Left.PID_Output, 5, RED, BLACK, 16);
    LCD_ShowIntNum(220, 60, Wheel_Right.PID_Output, 5, GREEN, BLACK, 16); 
    LCD_ShowFloatNum(60,80,PID_KP,1,2,BLUE,BLACK,16);
    LCD_ShowFloatNum(130,80,PID_KI,1,2,BLUE,BLACK,16);
    LCD_ShowFloatNum(200,80,PID_KD,1,2,BLUE,BLACK,16);
    LCD_ShowIntNum(90, 100, Track_Value , 2, WHITE, BLACK, 16);
    LCD_ShowIntNum(200, 100, Distance, 6, WHITE, BLACK, 16);
    LCD_ShowIntNum(70, 120, Yaw_Value, 6, WHITE, BLACK, 16);
    LCD_ShowFloatNum(30,140,zigbee_data[0],1,2,BLUE,BLACK,16);
    LCD_ShowFloatNum(100,140,zigbee_data[1],1,3,BLUE,BLACK,16);
    LCD_ShowFloatNum(170,140,zigbee_data[2],1,3,BLUE,BLACK,16);
}

void Velocity_Get(void)//计算轮子速度
{
    if(Velocity_Counter >= Velocity_Interval) //根据定时器中断频率和速度计算间隔，控制速度更新频率
    {
        Encoder_Get(1,&Wheel_Left.Encoder,NULL);
        Encoder_Get(2,&Wheel_Right.Encoder,NULL);
        
        int32_t Velocity_Counter_temp = Velocity_Counter; // 快照，避免ISR竞态
        Velocity_Counter -= Velocity_Counter_temp;        // 减法而非清零，保留ISR新增的tick

        //使用有符号类型处理编码器计数变化
        // Encoder 和 Encoder_Last 已是 int32_t，直接相减即可正确处理正反转
        float Delta_Left  = (float)(Wheel_Left.Encoder - Wheel_Left.Encoder_Last);
        float Delta_Right = (float)(Wheel_Right.Encoder - Wheel_Right.Encoder_Last);

        //计算速度(脉冲/tick 放大至可读范围)
        float Temp_Velocity_Left  = Delta_Left  * Velocity_Scale / Velocity_Counter_temp;
        float Temp_Velocity_Right = Delta_Right * Velocity_Scale / Velocity_Counter_temp;

        //低通滤波
        Wheel_Left.Velocity  = Temp_Velocity_Left  * Velocity_Ratio + Wheel_Left.Velocity_Last  * (1 - Velocity_Ratio);
        Wheel_Right.Velocity = Temp_Velocity_Right * Velocity_Ratio + Wheel_Right.Velocity_Last * (1 - Velocity_Ratio);

        // 更新上次计数值
        Wheel_Left.Encoder_Last  = Wheel_Left.Encoder;
        Wheel_Right.Encoder_Last = Wheel_Right.Encoder;

        // 更新上次速度
        Wheel_Left.Velocity_Last  = Wheel_Left.Velocity;
        Wheel_Right.Velocity_Last = Wheel_Right.Velocity;
    }
}

void Distance_Get(void)//计算行驶距离
{
    float Avg_Pulses =(float)(Wheel_Left.Encoder + Wheel_Right.Encoder)/2.0;//平均脉冲数

    Distance = Avg_Pulses / Encoder_Pulses_Per_Revolution / Encoder_Gear_Ratio * 2 * Pi * Wheel_Radius; // 更新累计路程，单位为毫米
}

static void PID_Get(Wheel *Wheel_Temp)//PID计算
{
    //更新误差
    Wheel_Temp->PID_LastError = Wheel_Temp->PID_Error;
    //本次误差 =目标速度-实际速度
    Wheel_Temp->PID_Error = Wheel_Temp->Velocity_Target - Wheel_Temp->Velocity;

    // 误差限幅：限制 P 项发力，避免起步过猛
    #define PID_ERROR_MAX 200.0f
    float PID_Error_Clamped = Wheel_Temp->PID_Error;
    if (PID_Error_Clamped >  PID_ERROR_MAX) PID_Error_Clamped =  PID_ERROR_MAX;
    if (PID_Error_Clamped < -PID_ERROR_MAX) PID_Error_Clamped = -PID_ERROR_MAX;

    //PI为0时不积分
    if (PID_KI != 0.0f)
    {
        // 输出限幅抗饱和：输出封顶时不积同向误差
        bool Output_At_Max = (Wheel_Temp->PID_Output >= PID_MAX_OUTPUT);
        bool Output_At_Min = (Wheel_Temp->PID_Output <= PID_MIN_OUTPUT);

        if (!(Output_At_Max && Wheel_Temp->PID_Error > 0.0f) &&
            !(Output_At_Min && Wheel_Temp->PID_Error < 0.0f))
        {
            Wheel_Temp->PID_Integral += Wheel_Temp->PID_Error;
        }

        //积分限幅
        if (Wheel_Temp->PID_Integral > PID_Integral_Max)
            Wheel_Temp->PID_Integral = PID_Integral_Max;
        else if (Wheel_Temp->PID_Integral < PID_Integral_Min)
            Wheel_Temp->PID_Integral = PID_Integral_Min;
    }
    else
    {
        Wheel_Temp->PID_Integral = 0.0f;
    }

    // PID输出 = 前馈 + P项 + I项 + D项
    // 前馈提供基础 PWM（约125），PI 只纠偏 ±20，避免从零爬坡导致超调
    Wheel_Temp->PID_Output = Wheel_Temp->Feedforward * Wheel_Temp->Velocity_Target
                  + PID_KP * PID_Error_Clamped
                  + PID_KI * Wheel_Temp->PID_Integral
                  + PID_KD * (PID_Error_Clamped - Wheel_Temp->PID_LastError);

    // 输出限幅
    if (Wheel_Temp->PID_Output > PID_MAX_OUTPUT)
        Wheel_Temp->PID_Output = PID_MAX_OUTPUT;
    else if (Wheel_Temp->Velocity_Target == 0.0f)
        Wheel_Temp->PID_Output = 0.0f;     // 目标为0时允许完全停止
    else if (Wheel_Temp->PID_Output < PID_MIN_OUTPUT)
        Wheel_Temp->PID_Output = PID_MIN_OUTPUT;
}

void PID_Contorl(void)//PID控制
{
    PID_Get(&Wheel_Left);
    PID_Get(&Wheel_Right);
}

void Run_Stop(void)//停止运动
{
    AO_Control(FORWARD,0);
    BO_Control(FORWARD,0);
}

void Run_Track(void)//沿线循迹（含丢线直角弯检测）
{
    static float   Last_Error = 0;       // 上一拍偏差，用于丢线恢复
    static uint8_t Tracking_Flag = 0;    // 丢线前是否在循迹中
    static uint8_t Last_Track_Bits = 0;  // 丢线前最后一帧传感器原始位

    // 每帧读取最新传感器状态（不依赖Velocity_Interval）
    uint8_t Current_Bits = Track_GetBits();
    Track_Value = Track_Get();

    if (Track_Value == 0)              // 丢线：所有传感器均未检测到黑线
    {
        // 刚丢线时检查是否为直角弯：传感器计数 + Last_Error 双重校验
        if (Tracking_Flag)
        {
            Tracking_Flag = 0;

            // 统计丢线前左右两侧各有多少传感器压线
            uint8_t left_cnt  = ((Last_Track_Bits & 0x01) != 0)
                              + ((Last_Track_Bits & 0x02) != 0)
                              + ((Last_Track_Bits & 0x04) != 0);
            uint8_t right_cnt = ((Last_Track_Bits & 0x10) != 0)
                              + ((Last_Track_Bits & 0x20) != 0)
                              + ((Last_Track_Bits & 0x40) != 0);

            // 双重校验：传感器计数方向 必须与 Last_Error 符号一致
            // 任一方法不确定则不走直角弯，避免转错方向
            uint8_t bits_left  = (left_cnt > right_cnt);   // 传感器认为左转
            uint8_t bits_right = (right_cnt > left_cnt);   // 传感器认为右转
            uint8_t err_left   = (Last_Error < 0);          // 偏差认为左转
            uint8_t err_right  = (Last_Error > 0);          // 偏差认为右转

            if (bits_left && err_left)
            {
                Turn_State = TURN_Left;
                Turn_Start_Yaw = Yaw();
                Last_Error = 0;
                Last_Track_Bits = 0;
                return;
            }
            if (bits_right && err_right)
            {
                Turn_State = TURN_Right;
                Turn_Start_Yaw = Yaw();
                Last_Error = 0;
                Last_Track_Bits = 0;
                return;
            }
            // 两方法结论不一致 → 不确定，走普通丢线找回
        }

        // 非直角弯丢线：降速并以最后一次偏差方向持续转向，尝试找回线
        float Lost_Error = Last_Error;
        if (Lost_Error > TRACK_CENTER) 
        {
            Lost_Error = TRACK_CENTER;
        }
        if (Lost_Error < -TRACK_CENTER)
        {
            Lost_Error = -TRACK_CENTER;
        }

        Wheel_Left.Velocity_Target  = TRACK_LOST_SPEED + Lost_Error * TRACK_KP;
        Wheel_Right.Velocity_Target = TRACK_LOST_SPEED - Lost_Error * TRACK_KP;
    }
    else
    {
        Tracking_Flag = 1;  // 标记正在循迹
        Last_Track_Bits = Current_Bits;  // 缓存本帧传感器位，供丢线时判断方向

        //计算偏差：Track_Value - TRACK_CENTER
        //正 → 黑线偏右 → 右轮减速、左轮加速 → 车向右转
        //负 → 黑线偏左 → 左轮减速、右轮加速 → 车向左转
        float Error = (float)Track_Value - TRACK_CENTER;

        Last_Error = Error;// 记录偏差，供丢线时参考

        // 差速驱动：基础速度 ± 偏差修正
        Wheel_Left.Velocity_Target  = TRACK_BASE_SPEED + Error * TRACK_KP;
        Wheel_Right.Velocity_Target = TRACK_BASE_SPEED - Error * TRACK_KP;

        // 速度限幅，防止极端偏差导致一侧停转或超速
        if(Wheel_Left.Velocity_Target > TRACK_MAX_SPEED)
        {
            Wheel_Left.Velocity_Target=TRACK_MAX_SPEED;
        }
        if(Wheel_Left.Velocity_Target < TRACK_MIN_SPEED)
        {
            Wheel_Left.Velocity_Target = TRACK_MIN_SPEED;
        } 
        if (Wheel_Right.Velocity_Target > TRACK_MAX_SPEED) 
        {
            Wheel_Right.Velocity_Target = TRACK_MAX_SPEED;
        }
        if (Wheel_Right.Velocity_Target < TRACK_MIN_SPEED)
        {
            Wheel_Right.Velocity_Target = TRACK_MIN_SPEED;
        }
    }

    //未触发转弯则执行PID+电机驱动（丢线找回 & 正常循迹共用）
    if (Turn_State == TURN_None)
    {
        PID_Contorl();
        AO_Control(1, Wheel_Left.PID_Output);
        BO_Control(1, Wheel_Right.PID_Output);
    }
}

void Run_Turn(void)//直角转弯：先短暂直走冲过路口，再原地转弯
{
    static uint8_t  Phase = 0;                // 0=直走阶段, 1=转弯阶段
    static int32_t  Encoder_Start_L = 0;      // 直走起始左编码器
    static int32_t  Encoder_Start_R = 0;      // 直走起始右编码器
    static uint8_t  Last_Turn_State = TURN_None;

    // 检测到新转弯 → 重置阶段和编码器基准
    if (Turn_State != Last_Turn_State)
    {
        Last_Turn_State = Turn_State;
        if (Turn_State != TURN_None)
        {
            Phase = 0;
            Encoder_Start_L = 0;
            Encoder_Start_R = 0;
        }
    }

    // 阶段0: 短暂直走冲过路口
    if (Phase == 0)
    {
        if (Encoder_Start_L == 0 && Encoder_Start_R == 0)
        {
            Encoder_Get(1, &Encoder_Start_L, NULL);
            Encoder_Get(2, &Encoder_Start_R, NULL);
        }
        // 两轮同时前进
        AO_Control(Forward, TURN_PWM);
        BO_Control(Forward, TURN_PWM);

        int32_t Cur_L, Cur_R;
        Encoder_Get(1, &Cur_L, NULL);
        Encoder_Get(2, &Cur_R, NULL);

        int32_t DL = Cur_L - Encoder_Start_L;
        int32_t DR = Cur_R - Encoder_Start_R;
        if (DL < 0) DL = -DL;
        if (DR < 0) DR = -DR;

        float Avg = (float)(DL + DR) / 2.0f;
        float Dist = Avg / Encoder_Pulses_Per_Revolution / Encoder_Gear_Ratio * 2 * Pi * Wheel_Radius;

        if (Dist >= TURN_DRIVE_FORWARD_DISTANCE)
        {
            Phase = 1;
            Encoder_Start_L = 0;
            Encoder_Start_R = 0;
        }
        return;
    }

    // 阶段1: 转弯（比例减速）
    float Current_Yaw = Yaw();
    float Delta = Current_Yaw - Turn_Start_Yaw;

    if (Delta > 180.0f)      Delta -= 360.0f;
    else if (Delta < -180.0f) Delta += 360.0f;

    float Remaining = TURN_TARGET_ANGLE - ((Delta > 0) ? Delta : -Delta);

    if (Remaining <= TURN_DEADBAND)
    {
        AO_Control(Forward, 0);
        BO_Control(Forward, 0);

        Wheel_Left.PID_Integral   = 0;
        Wheel_Right.PID_Integral  = 0;
        Wheel_Left.PID_LastError  = 0;
        Wheel_Right.PID_LastError = 0;

        Turn_State = TURN_None;
        Phase = 0;
        return;
    }

    float ratio = Remaining / TURN_TARGET_ANGLE;
    if (ratio > 1.0f) ratio = 1.0f;

    uint32_t Pwm = (uint32_t)(TURN_PWM_MIN + ratio * (TURN_PWM - TURN_PWM_MIN));
    if (Turn_State == TURN_Left)
    {
        AO_Control(Forward, 0);
        BO_Control(Forward, Pwm);
    }
    else
    {
        AO_Control(Forward, Pwm);
        BO_Control(Forward, 0);
    }
}

void Run_AngleHold(float Target_Yaw)//角度环维持：双轮差速，原地保持角度
{
    Yaw_Circle.Yaw_Target=Target_Yaw;
    static float Last_Direction = 0;  // 上一拍输出方向

    Yaw_Value=Yaw();
    YawPID_Compute();
    float Output = Yaw_Circle.Output; // >0需右转，<0需左转

    #if YAW_REVERSE
    Output = -Output;                 // 方向反转
    #endif

    // 方向过零 → 清积分，防过冲
    if ((Last_Direction > 0.0f && Output < 0.0f) ||
        (Last_Direction < 0.0f && Output > 0.0f))
    {
        YawPID_ResetIntegral();
    }
    Last_Direction = Output;

    // 死区：±2°内停转
    float Abs_Output = (Output > 0.0f) ? Output : -Output;
    if (Abs_Output < ANGLE_DEADBAND)
    {
        AO_Control(Forward, 0);
        BO_Control(Forward, 0);
        return;
    }

    uint32_t Pwm = (uint32_t)(ANGLE_PWM_MIN +(Abs_Output / YAW_OUTPUT_MAX) * (ANGLE_PWM_MAX - ANGLE_PWM_MIN));
    
    if (Pwm > ANGLE_PWM_MAX) 
    {
        Pwm = ANGLE_PWM_MAX;
    }
    if (Output > 0.0f)      // 右转：左轮前进，右轮后退
    {
        AO_Control(Forward, Pwm);
        BO_Control(Rewerse, Pwm);
    }
    else                    // 左转：右轮前进，左轮后退
    {
        AO_Control(Rewerse, Pwm);
        BO_Control(Forward, Pwm);
    }
}

void TIMER_TICK_INST_IRQHandler(void)
{
    //如果产生了定时器中断
    switch( DL_TimerA_getPendingInterrupt(TIMER_TICK_INST) )
    {
        case DL_TIMERA_IIDX_ZERO:
        {
            Velocity_Counter++;
            time++;
        }
        break;
        default:break;
    }
}
   