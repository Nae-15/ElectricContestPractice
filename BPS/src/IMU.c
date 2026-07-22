#include "IMU.h"

volatile unsigned int delay_times = 0;
volatile unsigned char uart_data = 0;

/*============================================================================
 * 六轴传感器数据全局变量定义
 *===========================================================================*/

// 角度数据
struct SAngle stcAngle = {0};

// 角速度数据
struct SGyro stcGyro = {0};

// 加速度数据
struct SAccel stcAccel = {0};

// 四元数数据
struct SQuat stcQuat = {0};

void IMU_Init(void)
{
    // 清空上电期间IMU灌入UART RX FIFO的垃圾数据
    while (DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) {
        DL_UART_Main_receiveData(UART_0_INST);
    }
    // 清除所有UART中断标志（溢出、帧错误等），防止UART硬件锁死
    DL_UART_Main_clearInterruptStatus(UART_0_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    // 清除NVIC挂起的中断
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    // 使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    sendCaliYawCommand();
}

/******************************************************************************
 * 返回当前Yaw角（Z轴角度），单位°，范围 -180 ~ 180
******************************************************************************/
float Yaw(void)
{
   return stcAngle.Yaw;
}
/******************************************************************************
 * 返回横滚角（单位：°），范围 -180 ~ 180
******************************************************************************/
float Roll(void)
{
    return stcAngle.Roll;
}

/******************************************************************************
 * 返回俯仰角（单位：°），范围 -180 ~ 180
******************************************************************************/
float Pitch(void)
{
    return stcAngle.Pitch;
}

/******************************************************************************
 * 返回X轴角速度（单位：°/s）
******************************************************************************/
float GyroX(void)
{
    return stcGyro.wx;
}

/******************************************************************************
 * 返回Y轴角速度（单位：°/s）
******************************************************************************/
float GyroY(void)
{
    return stcGyro.wy;
}

/******************************************************************************
 * 返回Z轴角速度（单位：°/s）
******************************************************************************/
float GyroZ(void)
{
    return stcGyro.wz;
}

/******************************************************************************
 * 返回X轴加速度（单位：m/s²）
******************************************************************************/
float AccelX(void)
{
    return stcAccel.ax;
}

/******************************************************************************
 * 返回Y轴加速度（单位：m/s²）
******************************************************************************/
float AccelY(void)
{
    return stcAccel.ay;
}

/******************************************************************************
 * 返回Z轴加速度（单位：m/s²）
******************************************************************************/
float AccelZ(void)
{
    return stcAccel.az;
}

/*============================================================================
 * 四元数数据获取接口函数
 *===========================================================================*/

/******************************************************************************
 * 获取四元数 q0
 * @return 四元数 q0 值
******************************************************************************/
float QuatQ0(void)
{
    return stcQuat.q0;
}

/******************************************************************************
 * 获取四元数 q1
 * @return 四元数 q1 值
******************************************************************************/
float QuatQ1(void)
{
    return stcQuat.q1;
}

/******************************************************************************
 * 获取四元数 q2
 * @return 四元数 q2 值
******************************************************************************/
float QuatQ2(void)
{
    return stcQuat.q2;
}

/******************************************************************************
 * 获取四元数 q3
 * @return 四元数 q3 值
******************************************************************************/
float QuatQ3(void)
{
    return stcQuat.q3;
}

/******************************************************************************
 * 数据解析函数：接收0x5A开头的数据帧
 * 支持：角速度(0xAA)、角度(0xBB)、加速度(0xCC)、四元数(0xDD)
 ******************************************************************************/
void CopeSerial2Data(unsigned char ucData)
{
    static unsigned char ucRxBuffer[11];
    static unsigned char ucRxCnt = 0;
    unsigned char sum = 0;
    int i;

    // 缓存数据
    ucRxBuffer[ucRxCnt++] = ucData;

    // 帧头校验
    if (ucRxBuffer[0] != 0x5A)
    {
        ucRxCnt = 0;
        return;
    }

    // 帧类型判断，确定帧长度
    // 加速度/角速度/角度帧: 11字节 (0x5A + TYPE + 4组数据 + SUM)
    // 四元数帧: 11字节 (0x5A + TYPE + 4组数据 + SUM)
    // 寄存器读报包: 11字节
    if (ucRxCnt < 11) return;  // 等待完整帧

    // 根据TYPE计算校验和
    switch (ucRxBuffer[1])
    {
        case 0xAA:  // 角速度
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +   // WxL, WxH
                  ucRxBuffer[4] + ucRxBuffer[5] +   // WyL, WyH
                  ucRxBuffer[6] + ucRxBuffer[7] +   // WzL, WzH
                  ucRxBuffer[8] + ucRxBuffer[9];    // 0x00, 0x00
            
            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }
            
            // 解析角速度
            {
                short wx = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short wy = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short wz = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
                
                stcGyro.wx = (float)wx / 32768.0f * 2000.0f;  // °/s
                stcGyro.wy = (float)wy / 32768.0f * 2000.0f;
                stcGyro.wz = (float)wz / 32768.0f * 2000.0f;
            }
            break;
            
        case 0xBB:  // 角度
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +   // RollL, RollH
                  ucRxBuffer[4] + ucRxBuffer[5] +   // PitchL, PitchH
                  ucRxBuffer[6] + ucRxBuffer[7] +   // YawL, YawH
                  ucRxBuffer[8] + ucRxBuffer[9];    // 0x00, 0x00
            
            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }
            
            // 解析角度
            {
                short roll  = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short pitch = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short yaw   = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
                
                stcAngle.Roll  = (float)roll  / 32768.0f * 180.0f;  // °
                stcAngle.Pitch = (float)pitch / 32768.0f * 180.0f;
                stcAngle.Yaw   = (float)yaw   / 32768.0f * 180.0f;
            }
            break;
            
        case 0xCC:  // 加速度
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +   // AxL, AxH
                  ucRxBuffer[4] + ucRxBuffer[5] +   // AyL, AyH
                  ucRxBuffer[6] + ucRxBuffer[7] +   // AzL, AzH
                  ucRxBuffer[8] + ucRxBuffer[9];    // 0x00, 0x00
            
            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }
            
            // 解析加速度
            {
                short ax = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short ay = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short az = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
                
                const float G = 9.8f;  // 重力加速度
                stcAccel.ax = (float)ax / 32768.0f * 16.0f * G;  // m/s²
                stcAccel.ay = (float)ay / 32768.0f * 16.0f * G;
                stcAccel.az = (float)az / 32768.0f * 16.0f * G;
            }
            break;
            
        case 0xDD:  // 四元数
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +   // Q0L, Q0H
                  ucRxBuffer[4] + ucRxBuffer[5] +   // Q1L, Q1H
                  ucRxBuffer[6] + ucRxBuffer[7] +   // Q2L, Q2H
                  ucRxBuffer[8] + ucRxBuffer[9];    // Q3L, Q3H
            
            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }
            
            // 解析四元数
            {
                short q0 = (short)((ucRxBuffer[3] << 8) | ucRxBuffer[2]);
                short q1 = (short)((ucRxBuffer[5] << 8) | ucRxBuffer[4]);
                short q2 = (short)((ucRxBuffer[7] << 8) | ucRxBuffer[6]);
                short q3 = (short)((ucRxBuffer[9] << 8) | ucRxBuffer[8]);
                
                stcQuat.q0 = (float)q0 / 32768.0f;
                stcQuat.q1 = (float)q1 / 32768.0f;
                stcQuat.q2 = (float)q2 / 32768.0f;
                stcQuat.q3 = (float)q3 / 32768.0f;
            }
            break;
            
        case 0xEE:  // 寄存器读报包（可根据需要解析）
            // 寄存器读报包的格式与上述类似，可按需处理
            sum = ucRxBuffer[0] + ucRxBuffer[1] +
                  ucRxBuffer[2] + ucRxBuffer[3] +
                  ucRxBuffer[4] + ucRxBuffer[5] +
                  ucRxBuffer[6] + ucRxBuffer[7] +
                  ucRxBuffer[8] + ucRxBuffer[9];
            
            if (sum != ucRxBuffer[10])
            {
                ucRxCnt = 0;
                return;
            }
            // 寄存器数据读取处理
            break;
            
        default:
            // 未知类型，复位
            ucRxCnt = 0;
            return;
    }
    
    // 解析成功，复位接收计数器
    ucRxCnt = 0;
}

//串口的中断服务函数
void UART_0_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_0_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            uart_data = DL_UART_Main_receiveData(UART_0_INST);
            // 调用数据解析函数
            CopeSerial2Data(uart_data);
            break;

        default://其他的串口中断
            break;
    }
}

//解锁指令
uint8_t Key[5] = {0x55, 0xAA, 0x13, 0x8E, 0x5F};
//Z轴角度归零指令
uint8_t Yaw_Zero[5] = {0x55, 0xAA, 0x0A, 0x04, 0x00};
//保存指令
uint8_t Save[5] = {0x55, 0xAA, 0x00, 0x00, 0x00};
//获取零偏指令
uint8_t BIAS_CAL[5] = {0x55, 0xAA, 0x0A, 0x01, 0x00};

/******************************************************************************
 * 发送 Z轴角度归零命令
******************************************************************************/
void sendCaliYawCommand(void)
{
   uart0_send_SendByte(Key, 5);
	Delay_ms(100);
	uart0_send_SendByte(Yaw_Zero, 5);
	Delay_ms(100);
	uart0_send_SendByte(Save, 5);
}


/******************************************************************************
* 发送校准指令(校准过程中请勿移动，否则会校准失败或者校准效果不好)
******************************************************************************/
void performCaliBias(void)
{
    uart0_send_SendByte(Key, 5);
	Delay_ms(100);
	uart0_send_SendByte(BIAS_CAL, 5);
	Delay_ms(6000);
	uart0_send_SendByte(Save, 5);
}
