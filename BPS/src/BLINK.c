#include "BLINK.h"
void BUZZER_Blink(void)
{
    DL_GPIO_clearPins(BUZZER_PORT,BUZZER_PIN_PIN);
    Delay_ms(100);
    DL_GPIO_setPins(BUZZER_PORT,BUZZER_PIN_PIN);
    Delay_ms(100);

    DL_GPIO_clearPins(BUZZER_PORT,BUZZER_PIN_PIN);
    Delay_ms(300);
    DL_GPIO_setPins(BUZZER_PORT,BUZZER_PIN_PIN);
    Delay_ms(300);
}

void LED_Blink(void)
{
    DL_GPIO_setPins(LED_PORT,LED_Pin_PIN);
    Delay_ms(100);
    DL_GPIO_clearPins(LED_PORT,LED_Pin_PIN);
    Delay_ms(100);

    DL_GPIO_setPins(LED_PORT,LED_Pin_PIN);
    Delay_ms(300);
    DL_GPIO_clearPins(LED_PORT,LED_Pin_PIN);
    Delay_ms(300);
}