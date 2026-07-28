#include "MAGENT.h"

void Magent_On(void)
{
    DL_GPIO_setPins(MAGENT_PORT, MAGENT_PIn_PIN);  // 拉高，电磁铁吸
}

void Magent_Off(void)
{
    DL_GPIO_clearPins(MAGENT_PORT, MAGENT_PIn_PIN);  // 拉低，电磁铁放
}