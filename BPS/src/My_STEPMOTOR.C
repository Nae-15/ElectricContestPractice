#include "My_STEPMOTOR.h"

/**
 * @brief 步进电机 UART2 初始化（轮询模式，不禁用中断的 NVIC）
 *
 * 与 UART2_Init() 的区别：不清除 NVIC 挂起位、不使能 NVIC，
 * 防止电机回复数据触发 Default_Handler 导致程序卡死。
 */
void StepMotor_Init(void)
{
    /* 清空 RX FIFO 残留数据 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) {
        (void)DL_UART_Main_receiveData(UART_2_INST);
    }

    /* 清除 UART 外设中断标志 */
    DL_UART_Main_clearInterruptStatus(UART_2_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);

    /* 使能电机（上力矩） */
    zdt_emm_enable_no_reply(STEP_MOTOR_UART, STEP_MOTOR_ADDR,
                            true, ZDT_EMM_EXEC_NOW);
    Delay_ms(200);

    /* 清空电机回复数据，避免 RX FIFO 累积溢出 */
    while (!DL_UART_Main_isRXFIFOEmpty(UART_2_INST)) {
        (void)DL_UART_Main_receiveData(UART_2_INST);
    }
}