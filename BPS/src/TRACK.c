#include "TRACK.h"
uint8_t Track_Get(void)
{
    static uint8_t Track_Value = 0;
    uint8_t Left1=0,Left2=0,Left3=0,Right1=0,Right2=0,Right3=0;
    uint8_t Track_Count = 0;
    // 读取PIN_1状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_1_PIN))
    {
        Left1 = 1;
        Track_Count++;
    }

    // 读取PIN_2状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_2_PIN))
    {
        Left2 = 1;
        Track_Count++;
    }

    // 读取PIN_3状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_3_PIN))
    {
        Left3 = 1;
        Track_Count++;
    }

    // 读取PIN_4状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_4_PIN))
    {
        Right1 = 1;
        Track_Count++;
    }

    // 读取PIN_5状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_5_PIN))
    {
        Right2 = 1;
        Track_Count++;
    }

    // 读取PIN_6状态
    if( DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_6_PIN))
    {
        Right3 = 1;
        Track_Count++;
    }

    if(Track_Count == 0)//丢线状态
    {
        Track_Value=0;
    }
    else if (Track_Count ==6 )//全压线状态
    {

    }
    else//普通循迹状态
    {
        Track_Value = (Left1*1+Left2*2+Left3*3+Right1*4+Right2*5+Right3*6) *10 /Track_Count;
    }

    return Track_Value;
}

/* 返回6路传感器原始状态：bit0=Left1 ... bit5=Right2，1=检测到黑线 */
uint8_t Track_GetBits(void)
{
    uint8_t bits = 0;
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_1_PIN)) bits |= (1 << 0);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_2_PIN)) bits |= (1 << 1);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_3_PIN)) bits |= (1 << 2);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_4_PIN)) bits |= (1 << 3);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_5_PIN)) bits |= (1 << 4);
    if (DL_GPIO_readPins(TRACKING_PORT, TRACKING_PIN_6_PIN)) bits |= (1 << 5);
    return bits;
}