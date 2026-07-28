#include "BLINK.h"
void BUZZER_Blink(void)
{
    static uint8_t state = 0;
    static int     t0    = 0;

    if (state == 0)
    {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_PIN);  // 拉低，蜂鸣器响
        t0    = time;
        state = 1;
        return;
    }

    if (state == 1 && time - t0 >= 300)       // 300ms 到 → 拉高，停响
    {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_PIN);
        state = 2;                             // 进入空闲，不再响
    }
    // state == 2: 空闲，不再动作
}

void LED_Blink(void)
{
    static uint8_t state = 0;
    static int     t0    = 0;

    if (state == 0)
    {
        DL_GPIO_clearPins(LED_PORT, LED_Pin_PIN);
        t0    = time;
        state = 1;
        return;
    }

    if (time - t0 >= 300)
    {
        DL_GPIO_setPins(LED_PORT, LED_Pin_PIN);
        state = 2;
    }
}
