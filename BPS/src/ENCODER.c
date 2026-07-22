#include "ENCODER.h"

static ENCODER motor_encoder1;
static ENCODER motor_encoder2;

// 编码器初始化
void Encoder_Init(void)
{
    // 初始化编码器1结构体
    motor_encoder1.Count = 0;
    motor_encoder1.TempCount = 0;
    motor_encoder1.Direction = FORWARD;

    // 初始化编码器2结构体
    motor_encoder2.Count = 0;
    motor_encoder2.TempCount = 0;
    motor_encoder2.Direction = FORWARD;

    // 编码器引脚外部中断已在SysConfig中配置，这里只需使能NVIC
    NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_INT_IRQN);
}

// 获取编码器数据：id=1读编码器1，其它读编码器2
void Encoder_Get(uint8_t Id, int *Count, ENCODER_DIR *Direction)
{
    ENCODER *motor_encodertemp = (Id == 1) ? &motor_encoder1 : &motor_encoder2;

    int delta = motor_encodertemp->TempCount - motor_encodertemp->Count;
    motor_encodertemp->Direction = (delta >= 0) ? FORWARD : REVERSAL;
    motor_encodertemp->Count = motor_encodertemp->TempCount;

    if (Count != NULL)
    {
        *Count = motor_encodertemp->Count;
    }
    if (Direction != NULL)
    {
        *Direction = motor_encodertemp->Direction;
    }
}

// 外部中断处理函数
void GROUP1_IRQHandler(void)
{
    uint32_t gpio_status;

    // 获取中断信号情况
    gpio_status = DL_GPIO_getEnabledInterruptStatus(ENCODER_PORT,
        ENCODER_PIN_A_PIN | ENCODER_PIN_B_PIN | ENCODER_PIN_A2_PIN | ENCODER_PIN_B2_PIN);

    // 编码器1 - A相上升沿触发
    if ((gpio_status & ENCODER_PIN_A_PIN) == ENCODER_PIN_A_PIN)
    {
        if (!DL_GPIO_readPins(ENCODER_PORT, ENCODER_PIN_B_PIN))
        {
            motor_encoder1.TempCount--;
        }
        else
        {
            motor_encoder1.TempCount++;
        }
    }

    // 编码器2 - A2相上升沿触发
    if ((gpio_status & ENCODER_PIN_A2_PIN) == ENCODER_PIN_A2_PIN)
    {
        if (!DL_GPIO_readPins(ENCODER_PORT, ENCODER_PIN_B2_PIN))
        {
            motor_encoder2.TempCount--;
        }
        else
        {
            motor_encoder2.TempCount++;
        }
    }

    // 清除所有中断状态
    DL_GPIO_clearInterruptStatus(ENCODER_PORT,
        ENCODER_PIN_A_PIN | ENCODER_PIN_B_PIN | ENCODER_PIN_A2_PIN | ENCODER_PIN_B2_PIN);
}
