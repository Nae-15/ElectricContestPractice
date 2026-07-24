#include "ti_msp_dl_config.h"
#include "KEY.h"
#include "ENCODER.h"
#include "IMU.h"
#include "BPS/lcd/lcd.h"
#include "BPS/inc/MOTOR.h"
#include "BPS/inc/TRACK.h"

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
int time=0;

//-----------路程-------------------
int32_t Distance;
//-----------循迹-------------------
uint8_t Track_Value;


//-----------PID--------------------
float PID_KP = 0.05f;   // 比例系数（12V响应太快，极保守防超调）
float PID_KI = 0.02f;   // 积分系数（极慢积分，减少稳态微振）
float PID_KD = 0.0f;    // 微分系数

// 前馈系数：12V供电，左右独立校准（右比左快2.72%，左需补偿）
#define LEFT_FEEDFORWARD  0.0976f
#define RIGHT_FEEDFORWARD 0.0920f
//PID输出限幅
#define PID_MAX_OUTPUT 800.0f // 最大输出
#define PID_MIN_OUTPUT 80.0f  // 最小输出
//PID积分限幅
#define PID_Integral_Max 50.0f  // 积分最大值（小幅慢调）
#define PID_Integral_Min -50.0f // 积分最小值（对称限幅）

/*
//单个速度单位间隔（单位：mm/s）
#define STOP_VELOCITY 0.0f
#define MIDDLE_VELOCITY 600.0f
#define VELOCITY_Step 70.0f
float VELOCITY_JianGe = 1.0f;

double Distance = 0.0;//单位mm
double Target_Distance = 3500.0;//上次累计路程，单位mm
uint8_t Target_Circle_Count = 0;
uint8_t Circle_Count = 0;
*/

void System_Init(void);     //系统初始化
void Data_Init(void);       //数据初始化
void Show_Init(void);       //显示初始化
void ALL_Init(void);        //全局初始化
void Show_Update(void);
void Velocity_Get(void);    //速度获取
void Distance_Get(void);    //距离获取
static void PID_Get(Wheel *Wheel_Temp);  //PID计算算法
void PID_Contorl(void);     //PID实际调用

int main(void)
{   
    ALL_Init();
    Wheel_Left.Velocity_Target=2000;
    Wheel_Right.Velocity_Target=2000;
    int a=0;
    while(1) 
    {   

        Show_Update();
        if(time>Velocity_Interval)
        {
            Velocity_Get();
            PID_Contorl();
            AO_Control(1,Wheel_Left.PID_Output);
            BO_Control(1,Wheel_Right.PID_Output);
            lc_printf("%f\n",Wheel_Left.Velocity);//串口0发送
        }
        a=Key_Get();
        if(a){Wheel_Left.Velocity_Target=2500;}

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
}

void Data_Init(void)//数据初始化
{
    Wheel_Left.Feedforward  = LEFT_FEEDFORWARD;
    Wheel_Right.Feedforward = RIGHT_FEEDFORWARD;
    
    //编码器数值初始化
    Encoder_Get(1, &Wheel_Left.Encoder, NULL);
    Encoder_Get(2, &Wheel_Right.Encoder, NULL);
    Wheel_Left.Encoder_Last  = Wheel_Left.Encoder;
    Wheel_Right.Encoder_Last = Wheel_Right.Encoder;
}

void Show_Init(void)//显示初始化
{
    LCD_ShowString(30,20,"Left_E:",RED,BLACK,16,0);
    LCD_ShowString(150,20,"Left_E:",GREEN,BLACK,16,0); 
    LCD_ShowString(30,40,"Left_V:",RED,BLACK,16,0);
    LCD_ShowString(150,40,"Left_R:",GREEN,BLACK,16,0);
    LCD_ShowString(30,60,"Left_O:",RED,BLACK,16,0);
    LCD_ShowString(150,60,"Left_O:",GREEN,BLACK,16,0); 
    LCD_ShowString(30,80,"KP:",BLUE,BLACK,16,0);
    LCD_ShowString(100,80,"KI:",BLUE,BLACK,16,0);
    LCD_ShowString(170,80,"KD:",BLUE,BLACK,16,0);
    LCD_ShowString(30,100,"TRACK:",WHITE,BLACK,16,0); 
    LCD_ShowString(120,100,"Distance:",WHITE,BLACK,16,0); 
}

void ALL_Init(void)//全局初始化
{
    System_Init();
    Data_Init();
    Show_Init();
}

void Show_Update(void)
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
   