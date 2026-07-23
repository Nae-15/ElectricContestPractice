#include "TRACK.h"
uint8_t Track_Get(void)//查表法获取循迹值
{
    static uint8_t Track_Value = 0;
    uint8_t Left1=0,Left2=0,Left3=0,Middle1=0,Right1=0,Right2=0,Right3=0;
    uint8_t Track_Count = 0;
    // 读取PIN_1状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_1_PIN))
    {
        Left1 = 1;  // bit0置1，表示检测到黑线
        Track_Count++;
    }

    // 读取PIN_2状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_2_PIN))
    {
        Left2 = 1;  // bit1置1，表示检测到黑线
        Track_Count++;
    }

    // 读取PIN_3状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_3_PIN))
    {
        Left3 = 1;  // bit2置1，表示检测到黑线
        Track_Count++;
    }

    // 读取PIN_4状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_4_PIN))
    {
        Middle1 = 1;  // bit3置1，表示检测到黑线
        Track_Count++;
    }

    // 读取PIN_5状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_5_PIN))
    {
        Right1 = 1;  // bit4置1，表示检测到黑线
        Track_Count++;
    }

    // 读取PIN_6状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_6_PIN))
    {
        Right2 = 1;  // bit5置1，表示检测到黑线
        Track_Count++;
    }

    // 读取PIN_7状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_7_PIN))
    {
        Right3 = 1;  // bit6置1，表示检测到黑线
        Track_Count++;
    }

    if(Track_Count == 0)//丢线状态
    {
        Track_Value=0;
    }
    else if (Track_Count==7)//全压线状态
    {

    }
    else//普通循迹状态
    {
        Track_Value = (Left1*1+Left2*2+Left3*3+Middle1*4+Right1*5+Right2*6+Right3*7) *10 /Track_Count;
    }

    return Track_Value;
}

/* 返回7路传感器原始状态：bit0=Left1 ... bit6=Right3，1=检测到黑线 */
uint8_t Track_GetBits(void)
{
    uint8_t bits = 0;
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_1_PIN)) bits |= (1 << 0);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_2_PIN)) bits |= (1 << 1);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_3_PIN)) bits |= (1 << 2);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_4_PIN)) bits |= (1 << 3);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_5_PIN)) bits |= (1 << 4);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_6_PIN)) bits |= (1 << 5);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_7_PIN)) bits |= (1 << 6);
    return bits;
}