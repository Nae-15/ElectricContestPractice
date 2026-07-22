#include "KEY.h"
uint8_t Key_Get(void)
{
	uint8_t i;
	const uint32_t key_pins[] = {KEY1, KEY2, KEY3, KEY4};

	for (i = 0; i < 4; i++)
	{
		if (!(DL_GPIO_readPins(KEY_PORT, key_pins[i])))
		{
			Delay_ms(20);
			if (!(DL_GPIO_readPins(KEY_PORT, key_pins[i])))
			{
				while (!(DL_GPIO_readPins(KEY_PORT, key_pins[i])));
				Delay_ms(20);
				return i + 1;
			}
		}
	}
	return 0;
}
