#include "ZIGBEE.h"

volatile unsigned char zigbee_data = 0;

void Zigbee_Init(void)
{
    UART1_Init();
}

void UART_1_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_1_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            zigbee_data = DL_UART_Main_receiveData(UART_1_INST);
            break;

        default://其他的串口中断
            break;
    }
}