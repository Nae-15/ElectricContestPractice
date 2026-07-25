/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                              4000000
/* GPIO defines for channel 3 */
#define GPIO_PWM_0_C3_PORT                                                 GPIOA
#define GPIO_PWM_0_C3_PIN                                         DL_GPIO_PIN_12
#define GPIO_PWM_0_C3_IOMUX                                      (IOMUX_PINCM34)
#define GPIO_PWM_0_C3_IOMUX_FUNC                     IOMUX_PINCM34_PF_TIMA0_CCP3
#define GPIO_PWM_0_C3_IDX                                    DL_TIMER_CC_3_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMG0
#define PWM_1_INST_IRQHandler                                   TIMG0_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                              4000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_1_C1_PORT                                                 GPIOA
#define GPIO_PWM_1_C1_PIN                                         DL_GPIO_PIN_13
#define GPIO_PWM_1_C1_IOMUX                                      (IOMUX_PINCM35)
#define GPIO_PWM_1_C1_IOMUX_FUNC                     IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_1_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for TIMER_TICK */
#define TIMER_TICK_INST                                                  (TIMA1)
#define TIMER_TICK_INST_IRQHandler                              TIMA1_IRQHandler
#define TIMER_TICK_INST_INT_IRQN                                (TIMA1_INT_IRQn)
#define TIMER_TICK_INST_LOAD_VALUE                                         (39U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                            4000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_4_MHZ_115200_BAUD                                        (2)
#define UART_0_FBRD_4_MHZ_115200_BAUD                                       (11)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART2
#define UART_1_INST_FREQUENCY                                            4000000
#define UART_1_INST_IRQHandler                                  UART2_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                        DL_GPIO_PIN_24
#define GPIO_UART_1_TX_PIN                                        DL_GPIO_PIN_23
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM54)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM53)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM54_PF_UART2_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM53_PF_UART2_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_4_MHZ_115200_BAUD                                        (2)
#define UART_1_FBRD_4_MHZ_115200_BAUD                                       (11)




/* Defines for SPI_LCD */
#define SPI_LCD_INST                                                       SPI1
#define SPI_LCD_INST_IRQHandler                                 SPI1_IRQHandler
#define SPI_LCD_INST_INT_IRQN                                     SPI1_INT_IRQn
#define GPIO_SPI_LCD_PICO_PORT                                            GPIOB
#define GPIO_SPI_LCD_PICO_PIN                                     DL_GPIO_PIN_8
#define GPIO_SPI_LCD_IOMUX_PICO                                 (IOMUX_PINCM25)
#define GPIO_SPI_LCD_IOMUX_PICO_FUNC                 IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_LCD_POCI_PORT                                            GPIOB
#define GPIO_SPI_LCD_POCI_PIN                                     DL_GPIO_PIN_7
#define GPIO_SPI_LCD_IOMUX_POCI                                 (IOMUX_PINCM24)
#define GPIO_SPI_LCD_IOMUX_POCI_FUNC                 IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_LCD */
#define GPIO_SPI_LCD_SCLK_PORT                                            GPIOB
#define GPIO_SPI_LCD_SCLK_PIN                                     DL_GPIO_PIN_9
#define GPIO_SPI_LCD_IOMUX_SCLK                                 (IOMUX_PINCM26)
#define GPIO_SPI_LCD_IOMUX_SCLK_FUNC                 IOMUX_PINCM26_PF_SPI1_SCLK



/* Port definition for Pin Group BUZZER */
#define BUZZER_PORT                                                      (GPIOA)

/* Defines for PIN: GPIOA.7 with pinCMx 14 on package pin 49 */
#define BUZZER_PIN_PIN                                           (DL_GPIO_PIN_7)
#define BUZZER_PIN_IOMUX                                         (IOMUX_PINCM14)
/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for Pin: GPIOB.3 with pinCMx 16 on package pin 51 */
#define LED_Pin_PIN                                              (DL_GPIO_PIN_3)
#define LED_Pin_IOMUX                                            (IOMUX_PINCM16)
/* Defines for AIN1: GPIOB.19 with pinCMx 45 on package pin 16 */
#define TB6612_AIN1_PORT                                                 (GPIOB)
#define TB6612_AIN1_PIN                                         (DL_GPIO_PIN_19)
#define TB6612_AIN1_IOMUX                                        (IOMUX_PINCM45)
/* Defines for AIN2: GPIOB.17 with pinCMx 43 on package pin 14 */
#define TB6612_AIN2_PORT                                                 (GPIOB)
#define TB6612_AIN2_PIN                                         (DL_GPIO_PIN_17)
#define TB6612_AIN2_IOMUX                                        (IOMUX_PINCM43)
/* Defines for BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define TB6612_BIN1_PORT                                                 (GPIOA)
#define TB6612_BIN1_PIN                                         (DL_GPIO_PIN_16)
#define TB6612_BIN1_IOMUX                                        (IOMUX_PINCM38)
/* Defines for BIN2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define TB6612_BIN2_PORT                                                 (GPIOB)
#define TB6612_BIN2_PIN                                         (DL_GPIO_PIN_24)
#define TB6612_BIN2_IOMUX                                        (IOMUX_PINCM52)
/* Port definition for Pin Group LCD */
#define LCD_PORT                                                         (GPIOB)

/* Defines for RES: GPIOB.10 with pinCMx 27 on package pin 62 */
#define LCD_RES_PIN                                             (DL_GPIO_PIN_10)
#define LCD_RES_IOMUX                                            (IOMUX_PINCM27)
/* Defines for DC: GPIOB.11 with pinCMx 28 on package pin 63 */
#define LCD_DC_PIN                                              (DL_GPIO_PIN_11)
#define LCD_DC_IOMUX                                             (IOMUX_PINCM28)
/* Defines for CS: GPIOB.14 with pinCMx 31 on package pin 2 */
#define LCD_CS_PIN                                              (DL_GPIO_PIN_14)
#define LCD_CS_IOMUX                                             (IOMUX_PINCM31)
/* Defines for BLK: GPIOB.26 with pinCMx 57 on package pin 28 */
#define LCD_BLK_PIN                                             (DL_GPIO_PIN_26)
#define LCD_BLK_IOMUX                                            (IOMUX_PINCM57)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOA)

/* Defines for PIN_A: GPIOA.26 with pinCMx 59 on package pin 30 */
// pins affected by this interrupt request:["PIN_A","PIN_B","PIN_A2","PIN_B2"]
#define ENCODER_INT_IRQN                                        (GPIOA_INT_IRQn)
#define ENCODER_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODER_PIN_A_IIDX                                  (DL_GPIO_IIDX_DIO26)
#define ENCODER_PIN_A_PIN                                       (DL_GPIO_PIN_26)
#define ENCODER_PIN_A_IOMUX                                      (IOMUX_PINCM59)
/* Defines for PIN_B: GPIOA.27 with pinCMx 60 on package pin 31 */
#define ENCODER_PIN_B_IIDX                                  (DL_GPIO_IIDX_DIO27)
#define ENCODER_PIN_B_PIN                                       (DL_GPIO_PIN_27)
#define ENCODER_PIN_B_IOMUX                                      (IOMUX_PINCM60)
/* Defines for PIN_A2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define ENCODER_PIN_A2_IIDX                                 (DL_GPIO_IIDX_DIO14)
#define ENCODER_PIN_A2_PIN                                      (DL_GPIO_PIN_14)
#define ENCODER_PIN_A2_IOMUX                                     (IOMUX_PINCM36)
/* Defines for PIN_B2: GPIOA.25 with pinCMx 55 on package pin 26 */
#define ENCODER_PIN_B2_IIDX                                 (DL_GPIO_IIDX_DIO25)
#define ENCODER_PIN_B2_PIN                                      (DL_GPIO_PIN_25)
#define ENCODER_PIN_B2_IOMUX                                     (IOMUX_PINCM55)
/* Port definition for Pin Group TRACKING */
#define TRACKING_PORT                                                    (GPIOB)

/* Defines for PIN_1: GPIOB.5 with pinCMx 18 on package pin 53 */
#define TRACKING_PIN_1_PIN                                       (DL_GPIO_PIN_5)
#define TRACKING_PIN_1_IOMUX                                     (IOMUX_PINCM18)
/* Defines for PIN_2: GPIOB.15 with pinCMx 32 on package pin 3 */
#define TRACKING_PIN_2_PIN                                      (DL_GPIO_PIN_15)
#define TRACKING_PIN_2_IOMUX                                     (IOMUX_PINCM32)
/* Defines for PIN_3: GPIOB.25 with pinCMx 56 on package pin 27 */
#define TRACKING_PIN_3_PIN                                      (DL_GPIO_PIN_25)
#define TRACKING_PIN_3_IOMUX                                     (IOMUX_PINCM56)
/* Defines for PIN_4: GPIOB.18 with pinCMx 44 on package pin 15 */
#define TRACKING_PIN_4_PIN                                      (DL_GPIO_PIN_18)
#define TRACKING_PIN_4_IOMUX                                     (IOMUX_PINCM44)
/* Defines for PIN_5: GPIOB.21 with pinCMx 49 on package pin 20 */
#define TRACKING_PIN_5_PIN                                      (DL_GPIO_PIN_21)
#define TRACKING_PIN_5_IOMUX                                     (IOMUX_PINCM49)
/* Defines for PIN_6: GPIOB.0 with pinCMx 12 on package pin 47 */
#define TRACKING_PIN_6_PIN                                       (DL_GPIO_PIN_0)
#define TRACKING_PIN_6_IOMUX                                     (IOMUX_PINCM12)
/* Defines for PIN_7: GPIOB.1 with pinCMx 13 on package pin 48 */
#define TRACKING_PIN_7_PIN                                       (DL_GPIO_PIN_1)
#define TRACKING_PIN_7_IOMUX                                     (IOMUX_PINCM13)
/* Port definition for Pin Group IIC_Software */
#define IIC_Software_PORT                                                (GPIOA)

/* Defines for SCL: GPIOA.1 with pinCMx 2 on package pin 34 */
#define IIC_Software_SCL_PIN                                     (DL_GPIO_PIN_1)
#define IIC_Software_SCL_IOMUX                                    (IOMUX_PINCM2)
/* Defines for SDA: GPIOA.0 with pinCMx 1 on package pin 33 */
#define IIC_Software_SDA_PIN                                     (DL_GPIO_PIN_0)
#define IIC_Software_SDA_IOMUX                                    (IOMUX_PINCM1)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for PIN__1: GPIOB.12 with pinCMx 29 on package pin 64 */
#define KEY_PIN__1_PIN                                          (DL_GPIO_PIN_12)
#define KEY_PIN__1_IOMUX                                         (IOMUX_PINCM29)
/* Defines for PIN__2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define KEY_PIN__2_PIN                                          (DL_GPIO_PIN_13)
#define KEY_PIN__2_IOMUX                                         (IOMUX_PINCM30)
/* Defines for PIN__3: GPIOB.23 with pinCMx 51 on package pin 22 */
#define KEY_PIN__3_PIN                                          (DL_GPIO_PIN_23)
#define KEY_PIN__3_IOMUX                                         (IOMUX_PINCM51)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_TIMER_TICK_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_SPI_LCD_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
