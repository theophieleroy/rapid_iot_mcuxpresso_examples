/*
* The Clear BSD License
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
* All rights reserved.
*
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided
* that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file    board.h
 * @brief   Board initialization header file.
 */

/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#ifndef _BOARD_H_
#define _BOARD_H_

#include <stdint.h>
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "peripherals.h"
/**
 * @brief	The board name
 */
#define BOARD_NAME "RAPID_IOT"

#ifndef APP_SERIAL_INTERFACE_TYPE
#define APP_SERIAL_INTERFACE_TYPE (gSerialMgrUart_c)
#endif

#ifndef APP_SERIAL_INTERFACE_INSTANCE
#define APP_SERIAL_INTERFACE_INSTANCE (0)
#endif

#ifndef APP_SERIAL_INTERFACE_SPEED
#define APP_SERIAL_INTERFACE_SPEED gUARTBaudRate115200_c
#endif

/* I2C error codes. */
#define I2C_RESULT_OK          0
#define I2C_RESULT_FAIL        1

/* KW41Z */
#define KW41_RESET_WAIT_TIME_MS 10u

/* TERMINAL UART definitions*/
#define TERMINAL_UART 				TERMINAL_UART_PERIPHERAL
#define TERMINAL_UART_IRQHANDLER	UART0_RX_TX_IRQHandler
#define TERMINAL_UART_IRQn			UART0_RX_TX_IRQn
#define TERMINAL_RING_BUFFER_SIZE	1024

/* BRIDGE UART definitions*/
#define BRIDGE_UART 				BRIDGE_UART_PERIPHERAL
#define BRIDGE_RING_BUFFER_SIZE 	1024

/* I2C1 SENSORS configuration */
#define BOARD_SENSORS_I2C_BASEADDR 		I2C1
#define ENS210_I2C_SLAVE_ADDRESS 		(uint8_t)0x43
#define TSL2572_I2C_SLAVE_ADDRESS 		(uint8_t)0x39
#define CCS811B_I2C_SLAVE_ADDRESS 		(uint8_t)0x5A
#define FXOS8700_I2C_SLAVE_ADDRESS 		(uint8_t)0x1E
#define FXAS21002_I2C_SLAVE_ADDRESS 	(uint8_t)0x20
#define MPL3115_I2C_SLAVE_ADDRESS 		(uint8_t)0x60

/* I2C2 SECURITY / TOUCH configuration */
#define BOARD_SECURITY_I2C_BASEADDR		I2C2
#define NTAG_I2C_SLAVE_BASEADDR 		I2C2
#define NTAG_I2C_SLAVE_ADDRESS 			(uint8_t)0x55
#define SX9500_I2C_SLAVE_ADDRESS 		(uint8_t)0x28
#define A1006_I2C_SLAVE_ADDRESS			(uint8_t)0x50

/* External Flash */
#define FLASH_RW_DATA_SIZE    			(1024*4)
#define BOARD_SPI_FLASH_CS_PORT 		PORTD
#define BOARD_SPI_FLASH_CS_GPIO  		GPIOD
#define BOARD_SPI_FLASH_CS_GPIO_PIN 	4U

/* Buzzer */
#define BUZZER_ON 						FTM_StartTimer(BOARD_INITPINS_BUZZER_PWM_PERIPHERAL, kFTM_SystemClock)
#define BUZZER_OFF						FTM_StopTimer(BOARD_INITPINS_BUZZER_PWM_PERIPHERAL)

/* Battery Voltage measurement BAT_SENS ADC */
#define BAT_SENS_BASE 					ADC0
#define BAT_SENS_GROUP 					0U
#define BAT_SENS_CHANNEL 				21U
#define BAT_SENS_ON						GPIO_WritePinOutput(BOARD_INITPINS_BAT_SENS_EN_GPIO, BOARD_INITPINS_BAT_SENS_EN_GPIO_PIN, 1)
#define BAT_SENS_OFF					GPIO_WritePinOutput(BOARD_INITPINS_BAT_SENS_EN_GPIO, BOARD_INITPINS_BAT_SENS_EN_GPIO_PIN, 0)
#define BAT_CHARGING					!GPIO_ReadPinInput(BOARD_INITPINS_CHG_STATE_GPIO, BOARD_INITPINS_CHG_STATE_GPIO_PIN)

/* RTC */
#define DSPI_MASTER_BASEADDR 			RTC_SPI_PERIPHERAL
#define DSPI_MASTER_PCS_FOR_TRANSFER 	kDSPI_MasterPcs1

/* LEDs */
/* PWM mode = default mode */
#define FTM3_SHARED_TIMER_START			FTM_StartTimer(BOARD_INITPINS_RGB_R_PERIPHERAL, kFTM_SystemClock)
#define FTM3_SHARED_TIMER_STOP			FTM_StopTimer(BOARD_INITPINS_RGB_R_PERIPHERAL)

#define BLIGHT_ZERO						0	// 0% duty cycle <=> OFF
#define BLIGHT_LOW						25	// 25% duty cycle
#define BLIGHT_MID						50	// 50% duty cycle
#define BLIGHT_HIGH						100	// 100% duty cycle

#define LED_RGB_WHITE					40, 100, 80		// White (100, 100, 100 is not really white !)
#define LED_RGB_RED						100, 0, 0		// Red max brightness
#define LED_RGB_GREEN					0, 100, 0		// Green max brightness
#define LED_RGB_BLUE					0, 0, 100		// Blue max brightness
#define LED_RGB_YELLOW					100, 100, 0		// Yellow max brightness
#define LED_RGB_CYAN					0, 100, 100		// Cyan max brightness
#define LED_RGB_PURPLE					100, 0, 100		// Purple max brightness
#define LED_RGB_BLACK					0, 0, 0			// 0% duty cycle <=> black

/* The SDHC instance/channel used for board */
#define BOARD_SDHC_CD_GPIO_IRQ_HANDLER PORTB_IRQHandler

/* SDHC base address, clock and card detection pin */
#define BOARD_SDHC_BASEADDR 			SDHC
#define BOARD_SDHC_CLKSRC 				kCLOCK_CoreSysClk
#define BOARD_SDHC_CLK_FREQ 			CLOCK_GetFreq(kCLOCK_CoreSysClk)
#define BOARD_SDHC_IRQ 					SDHC_IRQn
#define BOARD_SDHC_CD_PORT_IRQ 			PORTA_IRQn
#define BOARD_SDHC_CD_PORT_IRQ_HANDLER 	PORTA_IRQHandler
#define BOARD_SDHC_CD_GPIO_BASE 		BOARD_INITPINS_SD_CARD_DETECT_GPIO
#define BOARD_SDHC_CD_GPIO_PIN 			BOARD_INITPINS_SD_CARD_DETECT_GPIO_PIN
#define BOARD_SDHC_CD_PORT_BASE 		BOARD_INITPINS_SD_CARD_DETECT_PORT
#define BOARD_SDHC_CD_LOGIC_RISING

extern volatile uint8_t lptmrFlag;


/**
 * @brief 	Initialize board specific settings.
 */
void MK64F_hardware_init(void);
void BOARD_InitDebugConsole(void);
void BOARD_INIT_GPIOS(void);
void BOARD_InitPeripherals(void);
void BOARD_InitSPI();
void BOARD_InitUART();
void I2C1_InitPins(void);
void I2C1_DeinitPins(void);
uint32_t I2C1_GetFreq(void);
void App_WaitUsec(uint32_t microsec);
void App_WaitMsec(uint32_t millisec);

/* Sensors/actuators driver callback functions */
/* (management of I2C switch, power switch and K64 control pins) */
void Rgb_Led_Connect(void);
void Rgb_Led_Disconnect(void);
void Rgb_Led_Set_Colors(uint8_t R, uint8_t G, uint8_t B);
#ifndef BOOTLOADER
void Backlight_Connect(void);
void Backlight_Disconnect(void);
void Backlight_Set_Level(uint8_t level);
void Display_Connect(void);
void Display_Disconnect(void);
#endif
void CCS811_Connect(void);
void CCS811_Disconnect(void);
void Touch_Controller_Connect(void);
void Touch_Controller_Disconnect(void);
void Connect_NTAG_A1006(void);
void Disconnect_NTAG_A1006(void);

/* Functions used to determine the frequency of a module's input clock. */
uint32_t BOARD_GetUartClock(uint32_t instance);
uint32_t BOARD_GetFtmClock(uint32_t instance);
uint32_t BOARD_GetSpiClock(uint32_t instance);

#endif /* _BOARD_H_ */


