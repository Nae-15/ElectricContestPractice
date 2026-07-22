#include "lcd_init.h"

/**
 * @brief LCD GPIO初始化函数
 * @details 由于使用SysConfig工具配置GPIO，此函数留空
 */

/**
 * @brief LCD SPI总线写入函数
 * @param data 要写入的数据
 * @details 通过SPI接口向LCD发送一个字节的数据
 */
void LCD_Writ_Bus(u8 data) 
{
    LCD_CS_Clr();                          // 拉低片选信号，选中LCD
    
    DL_SPI_transmitData8(SPI_LCD_INST, data); // 通过SPI发送数据
    while(DL_SPI_isBusy(SPI_LCD_INST));       // 等待发送完成

    DL_SPI_receiveData8(SPI_LCD_INST);         // 读取SPI数据（清接收缓冲）
    while(DL_SPI_isBusy(SPI_LCD_INST));       // 等待接收完成

    LCD_CS_Set();                          // 拉高片选信号，取消选中
}

/**
 * @brief 写入8位数据
 * @param data 要写入的8位数据
 */
void LCD_WR_DATA8(u8 data)
{
    LCD_Writ_Bus(data);
}

/**
 * @brief 写入16位数据（RGB565格式）
 * @param data 要写入的16位颜色数据
 * @details 先写高8位，再写低8位
 */
void LCD_WR_DATA(u16 data)
{
    LCD_Writ_Bus(data >> 8);  // 先写高字节
    LCD_Writ_Bus(data);       // 再写低字节
}

/**
 * @brief 写入LCD寄存器地址
 * @param data 寄存器地址
 * @details DC引脚拉低表示写入命令/寄存器地址
 */
void LCD_WR_REG(u8 data)
{
    LCD_DC_Clr();            // DC=0，表示写命令
    LCD_Writ_Bus(data);       // 发送寄存器地址
    LCD_DC_Set();            // DC=1，表示后续写数据
}

/**
 * @brief 设置LCD显示区域（窗口）
 * @param x1 起始X坐标
 * @param y1 起始Y坐标
 * @param x2 结束X坐标
 * @param y2 结束Y坐标
 * @details 设置列地址(0x2A)和行地址(0x2B)，然后写入内存命令(0x2C)
 */
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
    LCD_WR_REG(0x2a);                         // 设置列地址命令
    LCD_WR_DATA(x1 + TFT_COLUMN_OFFSET);      // 起始列地址（带偏移）
    LCD_WR_DATA(x2 + TFT_COLUMN_OFFSET);      // 结束列地址（带偏移）
    
    LCD_WR_REG(0x2b);                         // 设置行地址命令
    LCD_WR_DATA(y1 + TFT_LINE_OFFSET);        // 起始行地址（带偏移）
    LCD_WR_DATA(y2 + TFT_LINE_OFFSET);        // 结束行地址（带偏移）
    
    LCD_WR_REG(0x2c);                         // 写入内存命令
}

/**
 * @brief LCD初始化函数
 * @details 按照ST7789V芯片的初始化序列进行配置，适配240x280屏幕
 *          参考STM32版TFT==169配置移植
 */
void LCD_Init(void)
{
    // 硬件复位序列
    LCD_RES_Clr();    // 拉低复位引脚
    Delay_ms(100);    // 延时100ms
    LCD_RES_Set();    // 拉高复位引脚
    Delay_ms(100);    // 延时100ms
    
    LCD_BLK_Set();    // 开启背光
    Delay_ms(100);    // 延时100ms

    // 0x01: Software Reset（软件复位）
    LCD_WR_REG(0x01);
    Delay_ms(120);  // 复位后需等待稳定

    // 0x11: Sleep Out（退出睡眠模式）
    LCD_WR_REG(0x11);
    Delay_ms(120);  // 等待退出睡眠

    // 0x36: Memory Data Access Control（内存数据访问控制）
    // 设置屏幕方向：MY/MX/MV/RGB位（必须在复位和睡眠退出之后设置）
    LCD_WR_REG(0x36); 
    if(USE_HORIZONTAL == 0)      LCD_WR_DATA8(0x00);  // 竖屏正常
    else if(USE_HORIZONTAL == 1) LCD_WR_DATA8(0xc0);  // 竖屏翻转
    else if(USE_HORIZONTAL == 2) LCD_WR_DATA8(0x60);  // 横屏正常
    else                         LCD_WR_DATA8(0xa0);  // 横屏翻转

    // 0x3A: Interface Pixel Format（接口像素格式）
    // 0x05 = 16-bit/pixel (RGB565)
    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);
    
    // 0xC5: VCOM Control（VCOM控制）
    LCD_WR_REG(0xC5);
    LCD_WR_DATA8(0x1A);

    // 0xB2: Porch Setting（前后廊设置）
    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33);

    // 0xB7: Gate Control（栅极控制）
    LCD_WR_REG(0xB7);
    LCD_WR_DATA8(0x05);  // 12.2v / -10.43v

    // 0xBB: VCOM Setting（VCOM设置）
    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x3F);

    // 0xC0: Power Control 1（电源控制1）
    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x2c);

    // 0xC2: VDV and VRH Command Enable（VDV和VRH命令使能）
    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01);

    // 0xC3: VRH Set（VRH设置）
    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x0F);  // 4.3+(VCOM+VCOM offset+VDV)

    // 0xC4: VDV Set（VDV设置）
    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20);  // 0v

    // 0xC6: Frame Rate Control（帧率控制）
    LCD_WR_REG(0xC6);
    LCD_WR_DATA8(0x01);  // 111Hz

    // 0xD0: Power Control 1（电源控制1）
    LCD_WR_REG(0xD0);
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1);

    // 0xE8: Power Control（电源控制）
    LCD_WR_REG(0xE8);
    LCD_WR_DATA8(0x03);

    // 0xE9: Equalize Time Control（均衡时间控制）
    LCD_WR_REG(0xE9);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x08);

    // 0xE0: Positive Gamma Correction（正伽马校正）
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x3F);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x30);

    // 0xE1: Negative Gamma Correction（负伽马校正）
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x09);
    LCD_WR_DATA8(0x08);
    LCD_WR_DATA8(0x03);
    LCD_WR_DATA8(0x24);
    LCD_WR_DATA8(0x32);
    LCD_WR_DATA8(0x32);
    LCD_WR_DATA8(0x3B);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x2F);

    // 0x21: Display Inversion On（显示反转开启）
    LCD_WR_REG(0x21);

    // 等待稳定后开启显示
    Delay_ms(120);
    LCD_WR_REG(0x11);  // 再次退出睡眠（确保稳定）
    Delay_ms(120);
    LCD_WR_REG(0x29);  // Display On（开启显示）
    Delay_ms(120);
}