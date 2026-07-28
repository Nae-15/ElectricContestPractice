#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "board.h"
#include "ti/driverlib/m0p/dl_core.h"

void UART0_Init(void)
{
    // 清空上电期间IMU灌入UART RX FIFO的垃圾数据
    while(DL_UART_Main_isRXFIFOEmpty(UART_0_INST) == false) 
    {
        DL_UART_Main_receiveData(UART_0_INST);
    }
    // 清除所有UART中断标志（溢出、帧错误等），防止UART硬件锁死
    DL_UART_Main_clearInterruptStatus(UART_0_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    // 清除NVIC挂起的中断
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    // 使能串口中断
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void UART2_Init(void)
{
    while(DL_UART_Main_isRXFIFOEmpty(UART_2_INST) == false) 
    {
        DL_UART_Main_receiveData(UART_2_INST);
    }
    // 清除所有UART中断标志（溢出、帧错误等），防止UART硬件锁死
    DL_UART_Main_clearInterruptStatus(UART_2_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    // 清除NVIC挂起的中断
    NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
    // 使能串口中断
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
}

void UART3_Init(void)
{
    while(DL_UART_Main_isRXFIFOEmpty(UART_3_INST) == false) 
    {
        DL_UART_Main_receiveData(UART_3_INST);
    }
    // 清除所有UART中断标志（溢出、帧错误等），防止UART硬件锁死
    DL_UART_Main_clearInterruptStatus(UART_3_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
        DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
        DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    // 清除NVIC挂起的中断
    NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
    // 使能串口中断
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
}

#ifdef UART_0_INST
static void uart0_sendChar(uint8_t dat)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_0_INST) == true );

    //发送单个字符
    DL_UART_Main_transmitData(UART_0_INST, dat);
}
#else
static void uart0_sendChar(uint8_t dat)
{
    (void)dat; // 避免未使用参数警告
}
#endif

void uart0_send_SendByte(uint8_t* data, uint32_t len)
{
    for(uint32_t i = 0; i < len; i++)
    {
        uart0_sendChar(data[i]);  // 直接发送原始字节
    }
}

void uart0_sendString(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while( *str!=0 && str!=0 )
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart0_sendChar(*str++);
    }
}

#ifdef UART_2_INST
static void uart2_sendChar(uint8_t dat)
{
    while( DL_UART_isBusy(UART_2_INST) == true );
    DL_UART_Main_transmitData(UART_2_INST, dat);
}
#else
static void uart2_sendChar(uint8_t dat)
{
    (void)dat;
}
#endif

void uart2_sendString(char* str)
{
    while( *str!=0 && str!=0 )
    {
        uart2_sendChar(*str++);
    }
}

/* 将c库的printf函数重新定位到USART */
int fputc(int ch, FILE *f)
{
    // 发送单个字符
    uart0_sendChar( (uint8_t)ch );

    return ch;
}

int LOG_Debug_Out(const char* __file, const char* __func, int __line, const char* format, ...)
{
    va_list args;
    va_start(args, format);

    // 前缀信息
    char log_buff[64] = {0};
    sprintf(log_buff, "[%s Func:%s Line:%d] ",__file,__func,__line);

    // 创建一个足够大的缓冲区来存储格式化后的字符串
    char buffer[512] = {0};
    strcpy(buffer, log_buff); // 使用strcpy来复制前缀信息
    int len = vsnprintf(buffer + strlen(buffer), sizeof(buffer) - strlen(buffer), format, args); // 追加到buffer中

    va_end(args);

    // 发送格式化后的字符串
    char temp_buff[] = "\r\n";
    strcat(buffer, temp_buff);
    uart0_sendString(buffer);

    return len;
}

int lc_printf(char* format,...)
{
    va_list args;
    va_start(args, format);

    // 创建一个足够大的缓冲区来存储格式化后的字符串
    char buffer[512] = {0};
    int len = vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    // 发送格式化后的字符串
    uart0_sendString(buffer);

    return len;
}

void Delay_us(int __us) { delay_cycles( (CPUCLK_FREQ / 1000 / 1000)*__us); }
void Delay_ms(int __ms) { delay_cycles( (CPUCLK_FREQ / 1000)*__ms); }
