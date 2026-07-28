#include "MAIXCAM.h"

volatile int32_t maixcam_data   = 0;
volatile uint8_t maixcam_rx_ready = 0;

void MAIXCAM_Init(void)
{
    UART3_Init();
}

/*
 * UART3 接收中断 — 视觉模块二进制帧解析
 * 帧格式 (9字节): 0x11 0x22 0x33 D0 D1 D2 D3 0x44 0x55 0x66
 *   0x11 0x22 0x33 = 帧头（前校验，3字节）
 *   D0..D3         = 32位整数数据（小端序，低字节在前）
 *   0x44 0x55 0x66 = 帧尾（后校验，3字节）
 *
 * 状态机:
 *   S0  等待 0x11
 *   S1  等待 0x22
 *   S2  等待 0x33
 *   S3..S6  接收 4 字节数据
 *   S7  等待 0x44
 *   S8  等待 0x55
 *   S9  等待 0x66 → 校验通过，数据就绪
 */
void UART_3_INST_IRQHandler(void)
{
    static uint8_t  state  = 0;   // 当前状态 0..9
    static uint32_t buffer = 0;   // 接收缓冲区
    static uint8_t  shift  = 0;   // 已接收字节数

    switch( DL_UART_getPendingInterrupt(UART_3_INST) )
    {
        case DL_UART_IIDX_RX:
        {
            uint8_t ch = DL_UART_Main_receiveData(UART_3_INST);

            switch (state)
            {
                case 0: // 等 0x11
                    if (ch == 0x11) state = 1;
                    break;
                case 1: // 等 0x22
                    state = (ch == 0x22) ? 2 : 0;
                    break;
                case 2: // 等 0x33
                    if (ch == 0x33)
                    {
                        state  = 3;
                        buffer = 0;
                        shift  = 0;
                    }
                    else
                    {
                        state = 0;
                    }
                    break;
                case 3: // 数据字节0 (LSB)
                case 4: // 数据字节1
                case 5: // 数据字节2
                case 6: // 数据字节3 (MSB)
                    buffer |= ((uint32_t)ch) << (shift * 8);
                    shift++;
                    state++;
                    break;
                case 7: // 等 0x44
                    state = (ch == 0x44) ? 8 : 0;
                    break;
                case 8: // 等 0x55
                    state = (ch == 0x55) ? 9 : 0;
                    break;
                case 9: // 等 0x66 → 帧完成
                    if (ch == 0x66)
                    {
                        maixcam_data    = (int32_t)buffer;
                        maixcam_rx_ready = 1;
                    }
                    state = 0;
                    break;
                default:
                    state = 0;
                    break;
            }
            break;
        }
        default: break;
    }
}