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
#include "BPS/inc/MAIXCAM.h"
#include "BPS/inc/My_STEPMOTOR.h"

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

//-------------验收问题----------------
#define Question_2 2

//-----------电机编码器----------------
#define Encoder_Gear_Ratio 28 //减速比1:28
#define Wheel_Radius 33 //轮子半径，单位为mm
#define Encoder_Pulses_Per_Revolution 13 //编码器每转一圈的脉冲数
#define Pi 3.14

//------------速度-----------------
#define Velocity_Ratio 0.8f //速度修正系数
#define Velocity_Interval 25 //测速周期
#define Velocity_Scale 3000 //速度放大系数，将脉冲/tick转为整数级读数
volatile int32_t Velocity_Counter;

//-------------时间-----------------
volatile int time;

//-------------方向-----------------
#define Forward 1
#define Rewerse 0

//-----------路程-------------------
int32_t Distance;

//-----------循迹-------------------
uint8_t Track_Value;
#define TRACK_CENTER    35.0f     // 6路传感器中心加权值（传感器3-4之间）
#define TRACK_KP        48.0f     // 寻线比例系数
#define TRACK_KD        40.0f     // 寻线微分系数（加强阻尼）
#define TRACK_DEVIATION_THRESHOLD  6.0f   // 偏差>6切弯道（Track≥41或≤29）

#define TRACK_BASE_SPEED      2000.0f   //寻线基础速度
#define TRACK_SPEED_TURN      1500.0f   //弯道降速
#define TRACK_LOST_SPEED      800.0f    //丢线时降速搜索
#define TRACK_MAX_SPEED       4000.0f   //寻线最大速度限幅
#define TRACK_MIN_SPEED       200.0f    //寻线最小速度限幅（弯道内侧轮减速）

//-----------速度环PID--------------------
float PID_KP = 0.22f;   // 比例系数（降低过冲）
float PID_KI = 0.02f;   // 积分系数（极慢积分，减少稳态微振）
float PID_KD = 0.0f;    // 微分系数

// 前馈系数：12V供电，左右独立校准（右比左快2.72%，左需补偿）
#define LEFT_FEEDFORWARD  0.0930f
#define RIGHT_FEEDFORWARD 0.0920f
//PID输出限幅
#define PID_MAX_OUTPUT    800.0f // 最大输出
#define PID_MIN_OUTPUT    80.0f  // 最小输出
//PID积分限幅
#define PID_Integral_Max  50.0f  // 积分最大值（小幅慢调）
#define PID_Integral_Min -50.0f  // 积分最小值（对称限幅）

//-----------角度环PID--------------------
float Yaw_Value;                    // 实测航向角
#define YAW_KP            0.15f      // 角度环比例系数（降低防过冲）
#define YAW_KI            0.05f     // 角度环积分系数（提高克服死区）
#define YAW_KD            1.0f      // 角度环微分系数（轻阻尼）
#define YAW_OUTPUT_MAX    150.0f    // 角度环输出限幅（收窄范围）
#define YAW_REVERSE       0         // 方向反转：若小车越调越偏，改为1

//-----------角度维持参数-----------
#define ANGLE_PWM_MIN     80        // 角度修正最低PWM（双轮差速，降低值）
#define ANGLE_PWM_MAX     250       // 角度修正最高PWM
#define ANGLE_DEADBAND    2.0f      // 角度死区（±2°内不调整）
float Target_Yaw;                   // 维持角度目标值

// ==================== 倾角平台串级PID -- 内环（电机编码器 -> RPM） ====================
// 反馈: StepMotor_ReadAngle_x10()
// 输出: StepMotor_RunSpeed_NoReply() 速度模式
// 目标: 手动给定(后续接外环 MAIXCAM)

#define INNER_KP            1.2f       // P: RPM per deg_x10 error (略升)
#define INNER_KI            0.1f       // I: 微幅慢积
#define INNER_KD            20.0f      // D: 抑制超调
#define INNER_DEADBAND      15         // 死区 deg_x10 (=1.5度)
#define INNER_MAX_RPM       400        // 最大RPM
#define INNER_MIN_RPM       20         // 最小RPM
#define INNER_I_MAX         40.0f      // 积分限幅
#define INNER_PERIOD_MS     15         // 66Hz

float Inner_Target_x10  = 300.0f;       // 目标角度 *10
float Inner_Current_x10 = 0.0f;       // 当前角度 *10
float Inner_Error        = 0.0f;
float Inner_LastError    = 0.0f;
float Inner_Integral     = 0.0f;
float Inner_Output       = 0.0f;       // RPM(带符号)
int   Inner_LastTime     = 0;
int   Inner_Dir          = 0;          // 1=CW,-1=CCW,0=stop
int   Inner_Status       = 0;          // PID 状态
int   Inner_Sent          = 0;          // SetTarget 结果: 0=未调用 1=已发送 2=死区 -1=读失败 -2=发送失败

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

void Mode_Question_Two(void);
//ZDT_EMM_DIR_CCW 为实际负

/**/

// ==================== 内环 PID 函数（位置模式） ====================
// 反馈: StepMotor_ReadAngle_x10() 电机编码器
// 策略: 直接发相对位置命令，电机自带加减速，无需速度环 PID
//
// TiltInner_Loop():  只读编码器更新状态，不发命令
// TiltInner_SetTarget(): 计算误差，发一次性位置命令给电机执行

void TiltInner_Loop(void)
{
    if (time - Inner_LastTime < INNER_PERIOD_MS) { Inner_Status = 0; return; }
    Inner_LastTime = time;
    Inner_Status = 1;

    int32_t deg;
    if (StepMotor_ReadAngle_x10(&deg) != ZDT_EMM_RESULT_OK) { Inner_Status = -1; return; }
    Inner_Current_x10 = (float)deg;
    Inner_Error = Inner_Target_x10 - Inner_Current_x10;
    Inner_Status = 2;

    // 更新输出显示（误差 * KP 作为参考）
    Inner_Output = Inner_Error;
}

// 设置目标角度，发位置命令让电机自行到位
void TiltInner_SetTarget(float degrees)
{
    Inner_Target_x10 = degrees * 10.0f;

    // 读当前位置
    int32_t cur;
    if (StepMotor_ReadAngle_x10(&cur) != ZDT_EMM_RESULT_OK) { Inner_Sent = -1; return; }
    Inner_Current_x10 = (float)cur;
    Inner_Error = Inner_Target_x10 - Inner_Current_x10;

    float abs_err = (Inner_Error > 0.0f) ? Inner_Error : -Inner_Error;

    if (abs_err < (float)INNER_DEADBAND) { Inner_Sent = 2; Inner_Output = 0.0f; return; }

    if (abs_err > 900.0f) abs_err = 900.0f;

    zdt_emm_dir_t dir = (Inner_Error > 0.0f) ? ZDT_EMM_DIR_CW : ZDT_EMM_DIR_CCW;
    zdt_emm_result_t r = StepMotor_MoveRelativeAngle(dir, (uint32_t)abs_err);
    Inner_Sent = (r == ZDT_EMM_RESULT_OK) ? 1 : -2;
    Inner_Output = Inner_Error;
}

/*

int main(void)
{
    __enable_irq();

    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(TIMER_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_TICK_INST_INT_IRQN);

    LCD_Init();
    LCD_Fill(0, 0, 300, LCD_H, BLACK);

    StepMotor_Init();
    Delay_ms(300);
    StepMotor_ZeroPosition();
    Delay_ms(300);

    TiltInner_SetTarget(30.0f);

    //Zigbee占用串口1
    Zigbee_Init();

    //视觉占用串口3
    MAIXCAM_Init();
    
  
    while(1)
    {
        // ---- 内环 PID（33Hz 定时执行）----
        TiltInner_Loop();

        // ---- LCD 显示 ----
        LCD_ShowString( 30, 30, "Tgt:", GREEN, BLACK, 16, 0);
        LCD_ShowFloatNum(70, 30, Inner_Target_x10 / 10.0f, 4, 1, GREEN, BLACK, 16);

        LCD_ShowString( 30, 50, "Cur:", WHITE, BLACK, 16, 0);
        LCD_ShowFloatNum(70, 50, Inner_Current_x10 / 10.0f, 4, 1, WHITE, BLACK, 16);

        LCD_ShowString( 30, 70, "Err:", YELLOW, BLACK, 16, 0);
        LCD_ShowFloatNum(70, 70, Inner_Error / 10.0f, 4, 1, YELLOW, BLACK, 16);

        LCD_ShowString( 30, 90, "RPM:", MAGENTA, BLACK, 16, 0);
        LCD_ShowFloatNum(70, 90, Inner_Output, 4, 1, MAGENTA, BLACK, 16);

        LCD_ShowString( 30,110, "Snt:", CYAN, BLACK, 16, 0);
        LCD_ShowIntNum( 70,110, Inner_Sent, 2, CYAN, BLACK, 16);
        LCD_ShowIntNum(100,110, time, 6, CYAN, BLACK, 16);

        LCD_ShowString( 30,130, "CAM:", MAGENTA, BLACK, 16, 0);
        LCD_ShowIntNum( 70,130, maixcam_data, 8, MAGENTA, BLACK, 16);

        // ---- UART1 发送 PID 数据 (每300ms) ----
        {
            static int last_send = 0;
            if (time - last_send >= 300)
            {
                last_send = time;
                char buf[64];
                sprintf(buf, "%.1f,%.1f,%.1f,%.0f,%d,%ld\n",
                    (double)(Inner_Target_x10 / 10.0f),
                    (double)(Inner_Current_x10 / 10.0f),
                    (double)(Inner_Error / 10.0f),
                    (double)Inner_Output,
                    Inner_Sent,
                    (long)maixcam_data);
                uart1_sendString(buf);
            }
        }
    }

}
*/


int main(void)
{   
    ALL_Init();
    while(1)
    {
        Show_Update();

        if (maixcam_rx_ready)
        {
            maixcam_rx_ready = 0;
        }

        if(Velocity_Counter>Velocity_Interval)
        {
            Velocity_Get();
            Distance_Get();
            Yaw_Value=Yaw();

            // Zigbee串口1发送（Track_Value由Run_Track每帧更新）
            {
                char buf[64];
                sprintf(buf, "%d,%f,%f\n",
                        Track_Value,
                        Wheel_Left.Velocity,
                        Wheel_Right.Velocity);
                uart1_sendString(buf);
            }
        }

        switch (Mode)
        {
            case Question_2:
            {
                Mode_Question_Two();
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
    
    //IMU占用串口0
    IMU_Init();
    sendCaliYawCommand();

    //Zigbee占用串口1
    Zigbee_Init();

    //视觉占用串口3
    MAIXCAM_Init();
    
    
    //角度环初始化：锁定0°航向
    YawPID_Init(YAW_KP, YAW_KI, YAW_KD);
    YawPID_SetTarget(0.0f);
    YawPID_Enable(true);

    //步进电机占用串口2
    StepMotor_Init();

    Mode=Question_2;
    
    LCD_Init();
    LCD_Fill(0, 0, 300, LCD_H, BLACK);
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
    LCD_ShowString(30,140,"MAIX:",MAGENTA,BLACK,16,0);
    LCD_ShowString(30,160,"Time:",MAGENTA,BLACK,16,0);
    LCD_ShowString(30,180,"PB1:",WHITE,BLACK,16,0);
    LCD_ShowString(30,200,"TrBits:",WHITE,BLACK,16,0);
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
    LCD_ShowIntNum(90, 140, maixcam_data, 8, MAGENTA, BLACK, 16);
    LCD_ShowIntNum(90, 160, time, 8, MAGENTA, BLACK, 16);
    LCD_ShowIntNum(70, 180,
        (DL_GPIO_readPins(GPIOB, DL_GPIO_PIN_1) != 0) ? 1 : 0,
        1, WHITE, BLACK, 16);
    LCD_ShowBinNum(100, 200, Track_GetBits(), WHITE, BLACK, 16);
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

void Run_Track(void)//沿线循迹
{
    static float Last_Error = 0;       // 上一拍偏差，用于丢线恢复

    Track_Value = Track_Get();

    if (Track_Value == 0)              // 丢线：所有传感器均未检测到黑线
    {
        // 降速并以最后一次偏差方向持续转向，尝试找回线
        float Lost_Error = Last_Error;
        if (Lost_Error > TRACK_CENTER) Lost_Error = TRACK_CENTER;
        if (Lost_Error < -TRACK_CENTER) Lost_Error = -TRACK_CENTER;

        Wheel_Left.Velocity_Target  = TRACK_LOST_SPEED + Lost_Error * TRACK_KP;
        Wheel_Right.Velocity_Target = TRACK_LOST_SPEED - Lost_Error * TRACK_KP;
    }
    else
    {
        // 计算偏差：Track_Value - TRACK_CENTER
        // 正 → 黑线偏右 → 左轮加速、右轮减速 → 车向右转
        // 负 → 黑线偏左 → 左轮减速、右轮加速 → 车向左转
        float Error = (float)Track_Value - TRACK_CENTER;
        float dError = Error - Last_Error;   // 微分：误差变化率（先算再更新）
        Last_Error = Error;

        // 自动判别直道/弯道：偏差大→降速过弯
        float abs_err = (Error > 0.0f) ? Error : -Error;
        float base_speed = (abs_err < TRACK_DEVIATION_THRESHOLD)
                           ? TRACK_BASE_SPEED : TRACK_SPEED_TURN;

        // 差速驱动：P项(比例) + D项(微分阻尼)
        float Correction = Error * TRACK_KP + dError * TRACK_KD;
        Wheel_Left.Velocity_Target  = base_speed + Correction;
        Wheel_Right.Velocity_Target = base_speed - Correction;

        // 速度限幅，防止极端偏差导致一侧停转或超速
        if(Wheel_Left.Velocity_Target > TRACK_MAX_SPEED)
            Wheel_Left.Velocity_Target = TRACK_MAX_SPEED;
        if(Wheel_Left.Velocity_Target < TRACK_MIN_SPEED)
            Wheel_Left.Velocity_Target = TRACK_MIN_SPEED;
        if(Wheel_Right.Velocity_Target > TRACK_MAX_SPEED)
            Wheel_Right.Velocity_Target = TRACK_MAX_SPEED;
        if(Wheel_Right.Velocity_Target < TRACK_MIN_SPEED)
            Wheel_Right.Velocity_Target = TRACK_MIN_SPEED;
    }

    PID_Contorl();
    AO_Control(1, Wheel_Left.PID_Output);
    BO_Control(1, Wheel_Right.PID_Output);
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

void Mode_Question_Two(void)
{
    static uint8_t track_done   = 0;
    static int32_t dist_start   = 0;
    static uint8_t need_snapshot = 1;

    if (!track_done)
    {
        if (need_snapshot)
        {
            dist_start   = Distance;
            need_snapshot = 0;
        }

        Run_Track();

        if ((Distance - dist_start) >= 6140)
        {
            Run_Stop();
            track_done = 1;
        }
    }
    else
    {
        Run_Stop();
    }
}

void Mode_Question_Three(void)
{

}

void Mode_Question_Four(void)
{

}

void Mode_Question_Five(void)
{

}

void Mode_Question_Six(void)
{

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
   