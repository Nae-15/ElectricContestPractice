#include "ZIGBEE.h"
#include <stdio.h>
#include "BPS/lcd/lcd.h"
volatile float zigbee_data[3] = {0};
volatile uint8_t rx_debug_flag;

void Zigbee_Init(void)
{
    UART1_Init();
}

void UART_1_INST_IRQHandler(void)
{
    static char rx_buf[32];
    static uint8_t rx_idx = 0;

    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_1_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
        {
            rx_debug_flag = 1;
            char ch = (char)DL_UART_Main_receiveData(UART_1_INST);
            if (ch == '\n')
            {
                rx_buf[rx_idx] = '\0';
                sscanf(rx_buf, "%f,%f,%f", &zigbee_data[0], &zigbee_data[1], &zigbee_data[2]);
                rx_idx = 0;
            }
            else if (ch != '\r' && rx_idx < sizeof(rx_buf) - 1)
            {
                rx_buf[rx_idx++] = ch;
            }
            break;   
        }

        default://其他的串口中断
        break;
    }
}