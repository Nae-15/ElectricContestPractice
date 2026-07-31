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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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
#define Question_3 3

//-----------电机编码器----------------
#define Encoder_Gear_Ratio 28 //减速比1:28
#define Wheel_Radius 33 //轮子半径，单位为mm
#define Encoder_Pulses_Per_Revolution 13 //编码器每转一圈的脉冲数
#define Pi 3.1415926f

//------------速度-----------------
#define Velocity_Ratio 0.8f //速度修正系数
#define Velocity_Interval 20U //测速周期
#define Velocity_Scale 3000 //速度放大系数，将脉冲/tick转为整数级读数
static uint32_t Velocity_Last_Time;

//-------------时间-----------------
volatile uint32_t time;

//-------------方向-----------------
#define Forward 1
#define Rewerse 0

//-----------路程-------------------
int32_t Distance;

//-----------循迹-------------------
uint8_t Track_Value;
#define TRACK_CENTER    35.0f     // 6路传感器中心加权值（传感器3-4之间）
#define TRACK_KP        34.68f     // 寻线比例系数
#define TRACK_KD        50.0f     // 寻线微分系数（加强阻尼）
#define TRACK_DEVIATION_THRESHOLD  6.0f   // 偏差>6切弯道（Track≥41或≤29）

#define TRACK_BASE_SPEED      1900.0f   //寻线基础速度
#define TRACK_SPEED_TURN      1500.0f   //弯道降速
#define TRACK_LOST_SPEED      800.0f    //丢线时降速搜索
#define TRACK_MAX_SPEED       4000.0f   //寻线最大速度限幅
#define TRACK_MIN_SPEED       200.0f    //寻线最小速度限幅（弯道内侧轮减速）
//-----------20s无球循迹-------------------
#define TRACK_BASE_SPEED_DEFAULT 1900.0f //寻线基础速度
#define TRACK_SPEED_TURN      1500.0f   //弯道降速
#define TRACK_LOST_SPEED      800.0f    //丢线时降速搜索
#define TRACK_MAX_SPEED       4000.0f   //寻线最大速度限幅
#define TRACK_MIN_SPEED       200.0f    //寻线最小速度限幅（弯道内侧轮减速）
#define TRACK_CORRECTION_MAX  1200.0f   //PD差速修正限幅，抑制数字探头跳变
#define TRACK_TARGET_SLEW_PER_TICK 500.0f //每20ms单轮目标最大变化量
#define TRACK_ERROR_FILTER_ALPHA 0.75f //位置误差一阶低通系数
#define TRACK_D_FILTER_ALPHA     0.40f //差分项一阶低通系数
#define TRACK_LOST_GRACE_MS   40U       //短时丢线沿最后方向继续修正
#define TRACK_LOST_STOP_MS    300U      //持续丢线后本地停车

//-----------20s无球循迹 终点检测-----------
#define Q2_SENSOR_TO_REAR_MM             300     // 传感器到车尾距离(mm)
#define Q2_BRAKE_ROLLOUT_MM               15     // 刹车滑行量(mm)
#define Q2_AFTER_LINE_COMMAND_MM \
    (Q2_SENSOR_TO_REAR_MM - Q2_BRAKE_ROLLOUT_MM)  // 285mm
#define Q2_FINISH_SEARCH_START_MM        5350    // 开始搜索A横线
#define Q2_FINISH_ACTIVE_MIN                5U   // 至少5路有效
#define Q2_FINISH_CONFIRM_SAMPLES           2U   // 连续2次确认
#define Q2_MAX_RUN_TIME_MS              20000U   // 20s超时
#define FAST20_ENCODER_FALLBACK_COMMAND_MM 5740  // 编码器兜底距离
_Static_assert(
    Q2_SENSOR_TO_REAR_MM > Q2_BRAKE_ROLLOUT_MM,
    "Q2 rear offset must exceed braking rollout");

//-----------30s载球循迹-----------
#define STABLE30_KP                         8.0f
#define STABLE30_KD                         6.0f
#define STABLE30_BASE_SPEED              1350.0f
#define STABLE30_CURVE_BASE_SPEED        1100.0f
#define STABLE30_CURVE_FEEDFORWARD        170.0f
#define STABLE30_CURVE_FEEDBACK_MAX       100.0f
#define STABLE30_STRAIGHT_FEEDBACK_MAX    120.0f
#define STABLE30_EDGE_START_ERROR          10.0f
#define STABLE30_EDGE_BASE_SPEED          900.0f
#define STABLE30_EDGE_FEEDBACK_GAIN         1.5f
#define STABLE30_EDGE_FEEDBACK_MAX        320.0f
#define STABLE30_COMMON_SLEW_PER_TICK      40.0f
#define STABLE30_CORRECTION_SLEW_PER_TICK  40.0f
#define STABLE30_ERROR_FILTER_ALPHA        0.35f
#define STABLE30_D_FILTER_ALPHA            0.20f
#define STABLE30_CURVE_TRANSITION_MM         150
#define STABLE30_CURVE_STEER_LEAD_MM          75
#define STABLE30_B_MARK_IGNORE_START_MM     1250
#define STABLE30_B_MARK_IGNORE_END_MM       1750
#define STABLE30_GYRO_TARGET_DPS        (-27.0f)
#define STABLE30_GYRO_K                     3.0f
#define STABLE30_GYRO_MAX                 100.0f
#define STABLE30_GYRO_FILTER_ALPHA         0.40f
#define STABLE30_GYRO_STALE_TICKS_MAX         5U
#define STABLE30_LOST_GRACE_MS               60U
#define STABLE30_RECOVERY_TIGHT_MS          500U
#define STABLE30_RECOVERY_SWEEP_MS          700U
#define STABLE30_RECOVERY_REACQUIRE_COUNT     3U
#define STABLE30_RECOVERY_COMMON_SPEED    500.0f
#define STABLE30_RECOVERY_CORRECTION      300.0f
#define STABLE30_RECOVERY_SLEW_PER_TICK    60.0f
#define STABLE30_RECOVERY_REACQUIRE_SPEED 800.0f
#define STABLE30_RECOVERY_YAW_RATE_DPS     30.0f
#define STABLE30_RECOVERY_GYRO_MAX         80.0f
#define STABLE30_FINISH_APPROACH_START_MM   5520
#define STABLE30_FINISH_APPROACH_SPEED    800.0f
#define STABLE30_LAP_DISTANCE_MM            6140
#define STABLE30_FINISH_PASS_DISTANCE_MM     220
#define STABLE30_MAX_RUN_TIME_MS          30000U

//-----------速度环PID--------------------
#define PID_DEFAULT_KP  0.122123f
#define PID_DEFAULT_KI  0.016943f
#define PID_DEFAULT_KD  0.000000f
float PID_KP = PID_DEFAULT_KP;
float PID_KI = PID_DEFAULT_KI;
float PID_KD = PID_DEFAULT_KD;

// 前馈系数：12V供电，左右独立校准
#define LEFT_FEEDFORWARD   0.148278f
#define RIGHT_FEEDFORWARD  0.153903f
//PID输出限幅
#define PID_MAX_OUTPUT    800.0f // 最大输出
#define PID_MIN_OUTPUT    80.0f  // 最小输出
//PID积分限幅：通过 PID_I_OUTPUT_MAX / PID_KI 反算积分上限
#define PID_I_OUTPUT_MAX  100.0f // 允许 PI 消除实测约 25% 的持续欠速

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
bool Velocity_Get(uint32_t now_ms);    //速度获取，返回 true 表示本次更新
void Distance_Get(void);    //距离获取
static void PID_Get(Wheel *Wheel_Temp);  //PID计算算法
void PID_Contorl(void);     //PID实际调用

void Run_Stop(void);                    //停止模式
void Run_Track(uint32_t now_ms, float base_speed);  //沿线循迹模式
void Run_AngleHold(float Target_Yaw);   //角度环维持模式
void SpeedPI_Reset(void);               //速度PI状态复位

static void Track_Control_Reset(void);       //循迹状态复位
static void Q2_Finish_Fast_Sample(uint32_t); //A横线终点扫描

void Mode_Question_Two(void);
void Mode_Question_Three(void);
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
    uint32_t last_control_ms;
    uint32_t last_display_ms;
    uint32_t last_1ms_sample;

    ALL_Init();
    last_control_ms = time;
    last_display_ms = time;
    last_1ms_sample = time - 1U;

    while(1)
    {
        uint32_t now_ms = time;

        /* 1ms 阶段：传感器滤波 + 终点扫描 */
        if (now_ms != last_1ms_sample)
        {
            last_1ms_sample = now_ms;
            Track_Update1ms();
            Q2_Finish_Fast_Sample(now_ms);
        }

        /* 20ms 控制阶段（模式函数仅在测速更新时运行，与优化文件对齐） */
        if ((uint32_t)(now_ms - last_control_ms) >= Velocity_Interval)
        {
            last_control_ms = now_ms;

            if (Velocity_Get(now_ms))
            {
                Distance_Get();
                Yaw_Value = Yaw();

                // Zigbee串口1发送（Track_Value由Run_Track每帧更新）
                // {
                //     char buf[64];
                //     sprintf(buf, "%d,%f,%f\n",
                //             Track_Value,
                //             Wheel_Left.Velocity,
                //             Wheel_Right.Velocity);
                //     uart1_sendString(buf);
                // }

                switch (Mode)
                {
                    case Question_2:
                        Mode_Question_Two();
                        break;
                    case Question_3:
                        Mode_Question_Three();
                        break;
                }
            }
        }

        /* 200ms 显示阶段 */
        if ((uint32_t)(now_ms - last_display_ms) >= 200U)
        {
            last_display_ms = now_ms;
            // Show_Update();  // 运行时关闭全屏刷新，避免SPI阻塞控制路径

            if (maixcam_rx_ready)
            {
                maixcam_rx_ready = 0;
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

    Mode=Question_3;
    
    LCD_Init();
    LCD_Fill(0, 0, 300, LCD_H, BLACK);
}

void Data_Init(void)//数据初始化
{
    //速度数据初始化
    Wheel_Left.Feedforward  = LEFT_FEEDFORWARD;
    Wheel_Right.Feedforward = RIGHT_FEEDFORWARD;
    Wheel_Left.Velocity_Target = 0.0f;
    Wheel_Right.Velocity_Target = 0.0f;
    Wheel_Left.Velocity = 0.0f;
    Wheel_Right.Velocity = 0.0f;
    Wheel_Left.Velocity_Last = 0.0f;
    Wheel_Right.Velocity_Last = 0.0f;
    SpeedPI_Reset();

    //编码器数据初始化
    Encoder_Get(1, &Wheel_Left.Encoder, NULL);
    Encoder_Get(2, &Wheel_Right.Encoder, NULL);
    Wheel_Left.Encoder_Last  = Wheel_Left.Encoder;
    Wheel_Right.Encoder_Last = Wheel_Right.Encoder;

    //时钟初始化
    time = 0;
    Velocity_Last_Time = 0U;

    //路程初始化
    Distance = 0;

    //角度初始化
    Yaw_Value = 0;
    Target_Yaw = 0;

    //循迹值初始化
    Track_Value = 0;
    Track_Control_Reset();
    Track_ResetStable();
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

bool Velocity_Get(uint32_t now_ms)//计算轮子速度，返回 true 表示本次更新
{
    uint32_t dt_ms = (uint32_t)(now_ms - Velocity_Last_Time);
    float Delta_Left;
    float Delta_Right;
    float Temp_Velocity_Left;
    float Temp_Velocity_Right;

    if (dt_ms < Velocity_Interval)
        return false;

    Velocity_Last_Time = now_ms;
    Encoder_Get(1, &Wheel_Left.Encoder, NULL);
    Encoder_Get(2, &Wheel_Right.Encoder, NULL);

    Delta_Left  = (float)(Wheel_Left.Encoder - Wheel_Left.Encoder_Last);
    Delta_Right = (float)(Wheel_Right.Encoder - Wheel_Right.Encoder_Last);

    /*
     * dt 使用实际毫秒数，主循环偶发延迟不会再被误当成固定采样周期。
     */
    Temp_Velocity_Left  = Delta_Left  * Velocity_Scale / (float)dt_ms;
    Temp_Velocity_Right = Delta_Right * Velocity_Scale / (float)dt_ms;

    Wheel_Left.Velocity =
        Temp_Velocity_Left * Velocity_Ratio +
        Wheel_Left.Velocity_Last * (1.0f - Velocity_Ratio);
    Wheel_Right.Velocity =
        Temp_Velocity_Right * Velocity_Ratio +
        Wheel_Right.Velocity_Last * (1.0f - Velocity_Ratio);

    Wheel_Left.Encoder_Last  = Wheel_Left.Encoder;
    Wheel_Right.Encoder_Last = Wheel_Right.Encoder;
    Wheel_Left.Velocity_Last  = Wheel_Left.Velocity;
    Wheel_Right.Velocity_Last = Wheel_Right.Velocity;
    return true;
}

void Distance_Get(void)//计算行驶距离
{
    float Avg_Pulses =(float)(Wheel_Left.Encoder + Wheel_Right.Encoder)/2.0;//平均脉冲数

    Distance = Avg_Pulses / Encoder_Pulses_Per_Revolution / Encoder_Gear_Ratio * 2 * Pi * Wheel_Radius; // 更新累计路程，单位为毫米
}

static void PID_Get(Wheel *Wheel_Temp)//PID计算（候选输出抗饱和）
{
    float p_error;
    float derivative;
    float base_output;
    float candidate_integral;
    float candidate_output;
    float integral_limit;

    if (Wheel_Temp->Velocity_Target <= 0.0f)
    {
        Wheel_Temp->PID_Error = 0.0f;
        Wheel_Temp->PID_LastError = 0.0f;
        Wheel_Temp->PID_Integral = 0.0f;
        Wheel_Temp->PID_Output = 0.0f;
        return;
    }

    Wheel_Temp->PID_Error = Wheel_Temp->Velocity_Target - Wheel_Temp->Velocity;

    // 误差限幅只作用于P、D项；原始误差仍用于积分消除稳态误差。
    #define PID_ERROR_MAX 200.0f
    p_error = Wheel_Temp->PID_Error;
    if (p_error >  PID_ERROR_MAX) p_error =  PID_ERROR_MAX;
    if (p_error < -PID_ERROR_MAX) p_error = -PID_ERROR_MAX;
    derivative = p_error - Wheel_Temp->PID_LastError;

    base_output =
        Wheel_Temp->Feedforward * Wheel_Temp->Velocity_Target +
        PID_KP * p_error +
        PID_KD * derivative;

    if (PID_KI > 0.0f)
    {
        integral_limit = PID_I_OUTPUT_MAX / PID_KI;
        candidate_integral =
            Wheel_Temp->PID_Integral + Wheel_Temp->PID_Error;
        if (candidate_integral > integral_limit)
            candidate_integral = integral_limit;
        else if (candidate_integral < -integral_limit)
            candidate_integral = -integral_limit;

        candidate_output = base_output + PID_KI * candidate_integral;

        /*
         * 使用本拍未饱和输出做条件积分。只有误差会把已经越界的输出继续
         * 推向同一方向时才拒绝积分，反向误差仍可释放已有积分。
         */
        if (!((candidate_output > PID_MAX_OUTPUT &&
               Wheel_Temp->PID_Error > 0.0f) ||
              (candidate_output < PID_MIN_OUTPUT &&
               Wheel_Temp->PID_Error < 0.0f)))
            Wheel_Temp->PID_Integral = candidate_integral;
    }
    else
    {
        Wheel_Temp->PID_Integral = 0.0f;
    }

    Wheel_Temp->PID_Output =
        base_output + PID_KI * Wheel_Temp->PID_Integral;
    Wheel_Temp->PID_LastError = p_error;

    // 输出限幅
    if (Wheel_Temp->PID_Output > PID_MAX_OUTPUT)
        Wheel_Temp->PID_Output = PID_MAX_OUTPUT;
    else if (Wheel_Temp->PID_Output < PID_MIN_OUTPUT)
        Wheel_Temp->PID_Output = PID_MIN_OUTPUT;
}

void PID_Contorl(void)//PID控制
{
    PID_Get(&Wheel_Left);
    PID_Get(&Wheel_Right);
}

void SpeedPI_Reset(void)
{
    Wheel_Left.PID_Error = 0.0f;
    Wheel_Left.PID_LastError = 0.0f;
    Wheel_Left.PID_Integral = 0.0f;
    Wheel_Left.PID_Output = 0.0f;
    Wheel_Right.PID_Error = 0.0f;
    Wheel_Right.PID_LastError = 0.0f;
    Wheel_Right.PID_Integral = 0.0f;
    Wheel_Right.PID_Output = 0.0f;
}

// ==================== 循迹静态状态变量 ====================
static TRACK_SAMPLE Track_Sample;
static float Track_Last_Error;
static float Track_Filtered_Error;
static float Track_Filtered_Derivative;
static float Track_Left_Target_Last;
static float Track_Right_Target_Last;
static int8_t Track_Last_Direction;
static uint32_t Track_Lost_Since;
static bool Track_Lost;
static float Track_KP_Runtime = TRACK_KP;
static float Track_KD_Runtime = TRACK_KD;

// Q2 终点检测状态
static int32_t Q2_Distance_Last;
static int32_t Q2_Route_Distance;
static uint32_t Q2_Run_Start_Time;
static uint32_t Q2_Finish_Last_Sample;
static uint8_t Q2_Finish_Hit_Count;
static bool Q2_Finish_Latched;
static int32_t Q2_Finish_Latched_Route_Distance;

void Run_Stop(void)//停止运动
{
    Wheel_Left.Velocity_Target = 0.0f;
    Wheel_Right.Velocity_Target = 0.0f;
    Track_Left_Target_Last = 0.0f;
    Track_Right_Target_Last = 0.0f;
    SpeedPI_Reset();
    AO_Control(Forward, 0);
    BO_Control(Forward, 0);
}

// ==================== 循迹辅助函数 ====================

static void Track_Control_Reset(void)
{
    Track_Last_Error = 0.0f;
    Track_Last_Direction = 1;
    Track_Lost_Since = 0U;
    Track_Lost = false;
    Track_Filtered_Error = 0.0f;
    Track_Filtered_Derivative = 0.0f;
    Track_Left_Target_Last = 0.0f;
    Track_Right_Target_Last = 0.0f;
    Track_Sample.RawBits = 0U;
    Track_Sample.ActiveBits = 0U;
    Track_Sample.ActiveCount = 0U;
    Track_Sample.PositionX10 = 0U;
}

static float Track_Clamp_Correction(float correction)
{
    if (correction > TRACK_CORRECTION_MAX)
        return TRACK_CORRECTION_MAX;
    if (correction < -TRACK_CORRECTION_MAX)
        return -TRACK_CORRECTION_MAX;
    return correction;
}

static float Track_Slew_Target(float target, float previous)
{
    if (target > previous + TRACK_TARGET_SLEW_PER_TICK)
        return previous + TRACK_TARGET_SLEW_PER_TICK;
    if (target < previous - TRACK_TARGET_SLEW_PER_TICK)
        return previous - TRACK_TARGET_SLEW_PER_TICK;
    return target;
}

void Run_Track(uint32_t now_ms, float base_speed)//沿线循迹（增强版：滤波+丢线恢复+斜率限制）
{
    float left_target;
    float right_target;

    Track_Sample = Track_Read();
    Track_Value = Track_Sample.PositionX10;

    if (Track_Sample.ActiveCount == 0U)
    {
        uint32_t lost_ms;
        if (!Track_Lost)
        {
            Track_Lost = true;
            Track_Lost_Since = now_ms;
        }
        lost_ms = (uint32_t)(now_ms - Track_Lost_Since);

        if (lost_ms < TRACK_LOST_GRACE_MS)
        {
            float correction = Track_Clamp_Correction(
                Track_Last_Error * Track_KP_Runtime);
            left_target = TRACK_LOST_SPEED + correction;
            right_target = TRACK_LOST_SPEED - correction;
        }
        else if (lost_ms < TRACK_LOST_STOP_MS)
        {
            /*
             * 持续丢线后让内侧轮保持最低速度、外侧轮低速搜索。
             * 即使最后一次误差接近0，也不会继续双轮直行冲出赛道。
             */
            if (Track_Last_Direction > 0)
            {
                left_target = TRACK_LOST_SPEED;
                right_target = TRACK_MIN_SPEED;
            }
            else
            {
                left_target = TRACK_MIN_SPEED;
                right_target = TRACK_LOST_SPEED;
            }
        }
        else
        {
            Run_Stop();
            return;
        }
    }
    else
    {
        float raw_error = (float)Track_Value - TRACK_CENTER;
        float d_error;
        float abs_err;
        float turn_speed = (base_speed < TRACK_SPEED_TURN)
                           ? base_speed : TRACK_SPEED_TURN;
        float selected_base;
        float correction;

        Track_Filtered_Error += TRACK_ERROR_FILTER_ALPHA *
            (raw_error - Track_Filtered_Error);
        d_error = (Track_Sample.ActiveBits == TRACK_ALL_ACTIVE_BITS)
                  ? 0.0f
                  : (Track_Filtered_Error - Track_Last_Error);
        Track_Filtered_Derivative += TRACK_D_FILTER_ALPHA *
            (d_error - Track_Filtered_Derivative);
        abs_err = (Track_Filtered_Error > 0.0f)
                  ? Track_Filtered_Error : -Track_Filtered_Error;
        selected_base = (abs_err < TRACK_DEVIATION_THRESHOLD)
                        ? base_speed : turn_speed;
        correction = Track_Clamp_Correction(
            Track_Filtered_Error * Track_KP_Runtime +
            Track_Filtered_Derivative * Track_KD_Runtime);

        Track_Lost = false;
        Track_Last_Error = Track_Filtered_Error;
        if (Track_Filtered_Error > 1.0f)
            Track_Last_Direction = 1;
        else if (Track_Filtered_Error < -1.0f)
            Track_Last_Direction = -1;

        left_target = selected_base + correction;
        right_target = selected_base - correction;
    }

    if (left_target > TRACK_MAX_SPEED) left_target = TRACK_MAX_SPEED;
    if (left_target < TRACK_MIN_SPEED) left_target = TRACK_MIN_SPEED;
    if (right_target > TRACK_MAX_SPEED) right_target = TRACK_MAX_SPEED;
    if (right_target < TRACK_MIN_SPEED) right_target = TRACK_MIN_SPEED;
    left_target =
        Track_Slew_Target(left_target, Track_Left_Target_Last);
    right_target =
        Track_Slew_Target(right_target, Track_Right_Target_Last);
    Track_Left_Target_Last = left_target;
    Track_Right_Target_Last = right_target;
    Wheel_Left.Velocity_Target = left_target;
    Wheel_Right.Velocity_Target = right_target;

    PID_Contorl();
    AO_Control(Forward, (uint32_t)Wheel_Left.PID_Output);
    BO_Control(Forward, (uint32_t)Wheel_Right.PID_Output);
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

/*
 * 只在末段搜索A横线：连续两个1ms样本至少5/6路有效后锁存。
 * 锁存不会立即停车，20ms控制环继续运行到车尾参考点到达A。
 */
static void Q2_Finish_Fast_Sample(uint32_t now_ms)
{
    TRACK_SAMPLE finish_sample;

    if (Q2_Route_Distance < Q2_FINISH_SEARCH_START_MM ||
        Q2_Finish_Latched ||
        now_ms == Q2_Finish_Last_Sample)
    {
        if (Q2_Route_Distance < Q2_FINISH_SEARCH_START_MM)
            Q2_Finish_Hit_Count = 0U;
        return;
    }
    Q2_Finish_Last_Sample = now_ms;

    finish_sample = Track_ReadRaw();
    if (finish_sample.ActiveCount >= Q2_FINISH_ACTIVE_MIN)
    {
        Q2_Finish_Hit_Count++;
        if (Q2_Finish_Hit_Count >= Q2_FINISH_CONFIRM_SAMPLES)
        {
            Q2_Finish_Latched = true;
            Q2_Finish_Latched_Route_Distance = Q2_Route_Distance;
        }
    }
    else
    {
        Q2_Finish_Hit_Count = 0U;
    }
}

void Mode_Question_Two(void)
{
    int32_t distance_step;

    /* 首次进入时自动开始 */
    {
        static uint8_t started = 0;
        if (!started)
        {
            Run_Stop();
            Track_Control_Reset();
            Track_KP_Runtime = TRACK_KP;
            Track_KD_Runtime = TRACK_KD;
            Q2_Distance_Last = Distance;
            Q2_Route_Distance = 0;
            Q2_Run_Start_Time = time;
            Q2_Finish_Last_Sample = time;
            Q2_Finish_Hit_Count = 0U;
            Q2_Finish_Latched = false;
            Q2_Finish_Latched_Route_Distance = 0;
            started = 1;
        }
    }

    distance_step = Distance - Q2_Distance_Last;
    if (!Track_Lost && distance_step > 0 && distance_step < 100)
        Q2_Route_Distance += distance_step;
    Q2_Distance_Last = Distance;

    Run_Track(time, TRACK_BASE_SPEED_DEFAULT);

    /* A横线锁存 + 扣除刹车距离后停车 */
    if (Q2_Finish_Latched &&
        (Q2_Route_Distance - Q2_Finish_Latched_Route_Distance) >=
            Q2_AFTER_LINE_COMMAND_MM)
    {
        Run_Stop();
    }
    /* 编码器兜底 */
    else if (!Q2_Finish_Latched &&
             Q2_Route_Distance >= FAST20_ENCODER_FALLBACK_COMMAND_MM)
    {
        Run_Stop();
    }
    /* 20s 超时 */
    else if ((uint32_t)(time - Q2_Run_Start_Time) >= Q2_MAX_RUN_TIME_MS)
    {
        Run_Stop();
    }
}

// ==================== 30s 载球稳定循迹 ====================

typedef struct
{
    float last_error;
    float filtered_error;
    float filtered_derivative;
    float filtered_gyro_z;
    float common_target_last;
    float correction_target_last;
    uint32_t lost_since;
    uint32_t gyro_last_update_count;
    uint8_t gyro_stale_ticks;
    uint8_t reacquire_count;
    uint8_t last_reliable_position;
    int8_t last_direction;
    int8_t recovery_direction;
    bool lost;
} STABLE30_TRACK_STATE;

static STABLE30_TRACK_STATE Stable30_Track;
static int32_t Stable30_Distance_Last;
static int32_t Stable30_Route_Distance;
static uint32_t Stable30_Run_Start_Time;

static void Stable30_Track_Reset(void)
{
    Stable30_Track.last_error = 0.0f;
    Stable30_Track.filtered_error = 0.0f;
    Stable30_Track.filtered_derivative = 0.0f;
    Stable30_Track.filtered_gyro_z = 0.0f;
    Stable30_Track.common_target_last = 0.0f;
    Stable30_Track.correction_target_last = 0.0f;
    Stable30_Track.lost_since = 0U;
    Stable30_Track.gyro_last_update_count = GyroUpdateCount();
    Stable30_Track.gyro_stale_ticks = 0U;
    Stable30_Track.reacquire_count = 0U;
    Stable30_Track.last_reliable_position = 35U;
    Stable30_Track.last_direction = 1;
    Stable30_Track.recovery_direction = 1;
    Stable30_Track.lost = false;
    Track_ResetStable();
}

static float Stable30_Slew_Target(
    float target, float previous, float max_step)
{
    if (target > previous + max_step)
        return previous + max_step;
    if (target < previous - max_step)
        return previous - max_step;
    return target;
}

static float Stable30_Approach_Base(
    float base_speed, int32_t travelled_mm)
{
    float progress;

    if (travelled_mm < STABLE30_FINISH_APPROACH_START_MM)
        return base_speed;
    progress =
        (float)(travelled_mm - STABLE30_FINISH_APPROACH_START_MM) /
        ((float)STABLE30_LAP_DISTANCE_MM -
         (float)STABLE30_FINISH_APPROACH_START_MM);
    if (progress > 1.0f)
        progress = 1.0f;
    return base_speed +
        (STABLE30_FINISH_APPROACH_SPEED - base_speed) * progress;
}

static float Stable30_Curve_Ramp(
    int32_t travelled_mm,
    int32_t curve_start_mm,
    int32_t curve_end_mm,
    int32_t lead_mm)
{
    int32_t ramp_start = curve_start_mm - lead_mm;
    int32_t ramp_full = ramp_start + STABLE30_CURVE_TRANSITION_MM;
    int32_t ramp_end = curve_end_mm + STABLE30_CURVE_TRANSITION_MM;

    if (travelled_mm <= ramp_start || travelled_mm >= ramp_end)
        return 0.0f;
    if (travelled_mm < ramp_full)
        return (float)(travelled_mm - ramp_start) /
            (float)STABLE30_CURVE_TRANSITION_MM;
    if (travelled_mm <= curve_end_mm)
        return 1.0f;
    return (float)(ramp_end - travelled_mm) /
        (float)STABLE30_CURVE_TRANSITION_MM;
}

static float Stable30_Curve_Amount(
    int32_t travelled_mm, int32_t lead_mm)
{
    float first_curve =
        Stable30_Curve_Ramp(travelled_mm, 1500, 3071, lead_mm);
    float second_curve =
        Stable30_Curve_Ramp(travelled_mm, 4571, 6142, lead_mm);
    return (first_curve > second_curve) ? first_curve : second_curve;
}

static void Stable30_Update_Gyro(void)
{
    uint32_t update_count = GyroUpdateCount();

    if (update_count != Stable30_Track.gyro_last_update_count)
    {
        Stable30_Track.gyro_last_update_count = update_count;
        Stable30_Track.gyro_stale_ticks = 0U;
        Stable30_Track.filtered_gyro_z +=
            STABLE30_GYRO_FILTER_ALPHA *
            (GyroZ() - Stable30_Track.filtered_gyro_z);
    }
    else if (Stable30_Track.gyro_stale_ticks < 255U)
    {
        Stable30_Track.gyro_stale_ticks++;
    }
}

static float Stable30_Gyro_Correction(float curve_amount)
{
    float target_rate;
    float correction;

    Stable30_Update_Gyro();
    if (Stable30_Track.gyro_last_update_count == 0U ||
        Stable30_Track.gyro_stale_ticks >
            STABLE30_GYRO_STALE_TICKS_MAX)
    {
        return 0.0f;
    }
    target_rate = STABLE30_GYRO_TARGET_DPS * curve_amount;
    correction =
        (Stable30_Track.filtered_gyro_z - target_rate) * STABLE30_GYRO_K;
    if (correction > STABLE30_GYRO_MAX)
        correction = STABLE30_GYRO_MAX;
    else if (correction < -STABLE30_GYRO_MAX)
        correction = -STABLE30_GYRO_MAX;
    return correction;
}

static float Stable30_Recovery_Gyro_Correction(int8_t direction)
{
    float target_rate;
    float correction;

    Stable30_Update_Gyro();
    if (Stable30_Track.gyro_last_update_count == 0U ||
        Stable30_Track.gyro_stale_ticks >
            STABLE30_GYRO_STALE_TICKS_MAX)
    {
        return 0.0f;
    }
    target_rate =
        -(float)direction * STABLE30_RECOVERY_YAW_RATE_DPS;
    correction =
        (Stable30_Track.filtered_gyro_z - target_rate) * STABLE30_GYRO_K;
    if (correction > STABLE30_RECOVERY_GYRO_MAX)
        correction = STABLE30_RECOVERY_GYRO_MAX;
    else if (correction < -STABLE30_RECOVERY_GYRO_MAX)
        correction = -STABLE30_RECOVERY_GYRO_MAX;
    return correction;
}

/* 30s模式专用：分离信号优先选择连续路数最多的黑线簇。 */
static uint8_t Stable30_Resolve_Position(
    uint8_t active_bits, uint8_t previous_position)
{
    uint8_t index = 0U;
    uint8_t best_position = 0U;
    uint8_t best_count = 0U;
    uint8_t best_distance = 255U;
    uint8_t reference =
        (previous_position >= 10U && previous_position <= 60U)
        ? previous_position : 35U;

    active_bits &= TRACK_ALL_ACTIVE_BITS;
    if (active_bits == 0U)
        return 0U;
    if (active_bits == TRACK_ALL_ACTIVE_BITS)
        return 35U;

    while (index < 6U)
    {
        uint8_t count = 0U;
        uint8_t sum = 0U;
        uint8_t position;
        uint8_t distance;

        while (index < 6U &&
               (active_bits & (uint8_t)(1U << index)) == 0U)
            index++;
        while (index < 6U &&
               (active_bits & (uint8_t)(1U << index)) != 0U)
        {
            sum = (uint8_t)(sum + (index + 1U) * 10U);
            count++;
            index++;
        }
        if (count == 0U)
            break;

        position = (uint8_t)(sum / count);
        distance = (position > reference)
                   ? (uint8_t)(position - reference)
                   : (uint8_t)(reference - position);
        if (best_position == 0U || count > best_count ||
            (count == best_count && distance < best_distance))
        {
            best_position = position;
            best_count = count;
            best_distance = distance;
        }
    }

    if (previous_position >= 10U && previous_position <= 60U &&
        best_count == 1U && best_distance > 20U)
    {
        return 0U;
    }
    return best_position;
}

static void Run_Track_Stable30(
    uint32_t now_ms, float base_speed, int32_t travelled_mm)
{
    float left_target;
    float right_target;
    float common_target;
    float correction_target;
    float curve_amount = Stable30_Curve_Amount(
        travelled_mm, STABLE30_CURVE_STEER_LEAD_MM);
    float curve_speed_amount = Stable30_Curve_Amount(
        travelled_mm, STABLE30_CURVE_TRANSITION_MM);
    float curve_feedforward =
        STABLE30_CURVE_FEEDFORWARD * curve_amount;
    uint8_t control_bits;
    uint8_t observed_position;

    Track_Sample = Track_ReadStable();
    control_bits = Track_Sample.ActiveBits;
    if (travelled_mm >= STABLE30_B_MARK_IGNORE_START_MM &&
        travelled_mm <= STABLE30_B_MARK_IGNORE_END_MM)
    {
        control_bits &= (uint8_t)~(1U << 5);
    }
    observed_position = Stable30_Resolve_Position(control_bits, 0U);
    Track_Value = Stable30_Resolve_Position(
        control_bits,
        Stable30_Track.lost
        ? 0U : Stable30_Track.last_reliable_position);

    if (Track_Value == 0U)
    {
        uint32_t lost_ms;
        int8_t search_direction;
        float recovery_correction;

        if (!Stable30_Track.lost)
        {
            Stable30_Track.lost = true;
            Stable30_Track.lost_since = now_ms;
            Stable30_Track.reacquire_count = 0U;
            if (observed_position > (uint8_t)TRACK_CENTER)
                Stable30_Track.recovery_direction = 1;
            else if (observed_position < (uint8_t)TRACK_CENTER &&
                     observed_position != 0U)
                Stable30_Track.recovery_direction = -1;
            else
                Stable30_Track.recovery_direction =
                    Stable30_Track.last_direction;
            if (Stable30_Track.recovery_direction == 0)
                Stable30_Track.recovery_direction = 1;
        }
        else
        {
            Stable30_Track.reacquire_count = 0U;
        }
        lost_ms = (uint32_t)(now_ms - Stable30_Track.lost_since);

        if (lost_ms < STABLE30_LOST_GRACE_MS)
        {
            left_target =
                Stable30_Track.common_target_last +
                Stable30_Track.correction_target_last;
            right_target =
                Stable30_Track.common_target_last -
                Stable30_Track.correction_target_last;
        }
        else
        {
            search_direction = Stable30_Track.recovery_direction;
            if (lost_ms >= STABLE30_RECOVERY_TIGHT_MS)
            {
                uint32_t phase =
                    (lost_ms - STABLE30_RECOVERY_TIGHT_MS) /
                    STABLE30_RECOVERY_SWEEP_MS;
                if ((phase & 1U) == 0U)
                    search_direction = -search_direction;
            }
            recovery_correction =
                (float)search_direction *
                STABLE30_RECOVERY_CORRECTION;
            recovery_correction +=
                Stable30_Recovery_Gyro_Correction(search_direction);
            left_target =
                STABLE30_RECOVERY_COMMON_SPEED + recovery_correction;
            right_target =
                STABLE30_RECOVERY_COMMON_SPEED - recovery_correction;
        }
    }
    else
    {
        float raw_error = (float)Track_Value - TRACK_CENTER;
        float d_error;
        float abs_error;
        float selected_base = base_speed;
        float effective_feedforward = curve_feedforward;
        float edge_amount = 0.0f;
        float correction;
        float feedback_max;

        if (Stable30_Track.lost &&
            Stable30_Track.reacquire_count == 0U)
        {
            Stable30_Track.filtered_error = raw_error;
            Stable30_Track.last_error = raw_error;
            Stable30_Track.filtered_derivative = 0.0f;
        }
        else
        {
            Stable30_Track.filtered_error +=
                STABLE30_ERROR_FILTER_ALPHA *
                (raw_error - Stable30_Track.filtered_error);
        }
        d_error = (Track_Sample.ActiveBits == TRACK_ALL_ACTIVE_BITS)
                  ? 0.0f
                  : (Stable30_Track.filtered_error -
                     Stable30_Track.last_error);
        Stable30_Track.filtered_derivative +=
            STABLE30_D_FILTER_ALPHA *
            (d_error - Stable30_Track.filtered_derivative);
        abs_error = (Stable30_Track.filtered_error > 0.0f)
                    ? Stable30_Track.filtered_error
                    : -Stable30_Track.filtered_error;

        if (effective_feedforward * Stable30_Track.filtered_error < 0.0f)
        {
            float scale = 1.0f - abs_error / 25.0f;
            if (scale < 0.0f)
                scale = 0.0f;
            effective_feedforward *= scale;
        }

        if (abs_error > STABLE30_EDGE_START_ERROR)
        {
            edge_amount =
                (abs_error - STABLE30_EDGE_START_ERROR) /
                (25.0f - STABLE30_EDGE_START_ERROR);
            if (edge_amount > 1.0f)
                edge_amount = 1.0f;
            if (selected_base > STABLE30_EDGE_BASE_SPEED)
            {
                selected_base +=
                    (STABLE30_EDGE_BASE_SPEED - selected_base) *
                    edge_amount;
            }
        }
        if (Stable30_Track.lost &&
            selected_base > STABLE30_RECOVERY_REACQUIRE_SPEED)
        {
            selected_base = STABLE30_RECOVERY_REACQUIRE_SPEED;
        }

        correction =
            Stable30_Track.filtered_error * STABLE30_KP +
            Stable30_Track.filtered_derivative * STABLE30_KD;
        correction *=
            1.0f +
            (STABLE30_EDGE_FEEDBACK_GAIN - 1.0f) * edge_amount;
        feedback_max = (effective_feedforward != 0.0f)
                       ? STABLE30_CURVE_FEEDBACK_MAX
                       : STABLE30_STRAIGHT_FEEDBACK_MAX;
        feedback_max +=
            (STABLE30_EDGE_FEEDBACK_MAX - feedback_max) * edge_amount;
        if (correction > feedback_max)
            correction = feedback_max;
        else if (correction < -feedback_max)
            correction = -feedback_max;

        if (selected_base > STABLE30_CURVE_BASE_SPEED)
        {
            selected_base +=
                (STABLE30_CURVE_BASE_SPEED - selected_base) *
                curve_speed_amount;
        }
        correction += effective_feedforward;
        correction += Stable30_Gyro_Correction(curve_amount);
        correction = Track_Clamp_Correction(correction);

        Stable30_Track.last_reliable_position = Track_Value;
        Stable30_Track.last_error = Stable30_Track.filtered_error;
        if (Stable30_Track.filtered_error > 1.0f)
            Stable30_Track.last_direction = 1;
        else if (Stable30_Track.filtered_error < -1.0f)
            Stable30_Track.last_direction = -1;

        left_target = selected_base + correction;
        right_target = selected_base - correction;
        if (Stable30_Track.lost)
        {
            if (Stable30_Track.reacquire_count <
                STABLE30_RECOVERY_REACQUIRE_COUNT)
            {
                Stable30_Track.reacquire_count++;
            }
            if (Stable30_Track.reacquire_count >=
                STABLE30_RECOVERY_REACQUIRE_COUNT)
            {
                Stable30_Track.lost = false;
                Stable30_Track.lost_since = 0U;
                Stable30_Track.reacquire_count = 0U;
            }
        }
    }

    if (left_target > TRACK_MAX_SPEED) left_target = TRACK_MAX_SPEED;
    if (left_target < TRACK_MIN_SPEED) left_target = TRACK_MIN_SPEED;
    if (right_target > TRACK_MAX_SPEED) right_target = TRACK_MAX_SPEED;
    if (right_target < TRACK_MIN_SPEED) right_target = TRACK_MIN_SPEED;

    common_target = (left_target + right_target) * 0.5f;
    correction_target = (left_target - right_target) * 0.5f;
    common_target = Stable30_Slew_Target(
        common_target,
        Stable30_Track.common_target_last,
        STABLE30_COMMON_SLEW_PER_TICK);
    correction_target = Stable30_Slew_Target(
        correction_target,
        Stable30_Track.correction_target_last,
        Stable30_Track.lost
        ? STABLE30_RECOVERY_SLEW_PER_TICK
        : STABLE30_CORRECTION_SLEW_PER_TICK);
    Stable30_Track.common_target_last = common_target;
    Stable30_Track.correction_target_last = correction_target;
    Wheel_Left.Velocity_Target = common_target + correction_target;
    Wheel_Right.Velocity_Target = common_target - correction_target;

    PID_Contorl();
    AO_Control(Forward, (uint32_t)Wheel_Left.PID_Output);
    BO_Control(Forward, (uint32_t)Wheel_Right.PID_Output);
}

void Mode_Question_Three(void)
{
    int32_t distance_step;

    /* 首次进入时自动开始 */
    {
        static uint8_t started = 0;
        if (!started)
        {
            Run_Stop();
            Stable30_Track_Reset();
            Stable30_Distance_Last = Distance;
            Stable30_Route_Distance = 0;
            Stable30_Run_Start_Time = time;
            started = 1;
        }
    }

    distance_step = Distance - Stable30_Distance_Last;
    if (!Stable30_Track.lost &&
        distance_step > 0 && distance_step < 100)
    {
        Stable30_Route_Distance += distance_step;
    }
    Stable30_Distance_Last = Distance;

    Run_Track_Stable30(
        time,
        Stable30_Approach_Base(
            STABLE30_BASE_SPEED,
            Stable30_Route_Distance),
        Stable30_Route_Distance);

    /* 终点检测 */
    if (Stable30_Route_Distance >=
        (STABLE30_LAP_DISTANCE_MM +
         STABLE30_FINISH_PASS_DISTANCE_MM))
    {
        Run_Stop();
    }
    /* 30s 超时 */
    else if ((uint32_t)(time - Stable30_Run_Start_Time) >=
             STABLE30_MAX_RUN_TIME_MS)
    {
        Run_Stop();
    }
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
            time++;
        }
        break;
        default:break;
    }
}
   