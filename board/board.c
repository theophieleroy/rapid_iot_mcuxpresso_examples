/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
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
 * @file    board.c
 * @brief   Board initialization file.
 */

/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#include "fsl_gpio.h"
#include "fsl_lptmr.h"
#include "board.h"
#include "pin_mux.h"
#include "peripherals.h"
#include "clock_config.h"
#include "spi_bus_share.h"
#include "pin_mux_rpk.h"
#include "rpk_led.h"
#ifndef BOOTLOADER
#include "GUI.h"
#endif

/* lptmr variables */
volatile uint8_t lptmrFlag = 0;
volatile uint8_t lptmr_init_status = 0;

/* I2C switch & power supply status */
static uint8_t rgb_led_connect_status = 0;
static uint8_t backlight_connect_status = 0;
static uint8_t display_connect_status = 0;
static uint8_t ccs811_connect_status = 0;
static uint8_t ntag_a1006_connect_status = 0;


/* Define the init structure for the output GPIO Output High pin*/
gpio_pin_config_t gpio_output_high_config = {
    kGPIO_DigitalOutput, 1,
};

/* Define the init structure for the output GPIO Output Low pin*/
gpio_pin_config_t gpio_output_low_config = {
    kGPIO_DigitalOutput, 0,
};

/* Define the init structure for the input GPIO */
gpio_pin_config_t gpio_input_config = {
    kGPIO_DigitalInput, 1,
};


/**
 * @brief Set up and initialize all required blocks and functions related to the board hardware.
 */
void BOARD_InitDebugConsole(void) {
	/* The user initialization should be placed here */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_INIT_GPIOS
 * Description   : Configures GPIOs as input or output and set init state
 *
 *END**************************************************************************/
void BOARD_InitPeripherals(void){
	/* UARTs */
	BOARD_Init_TERMINAL_UART();		// connected to MB debug probe through 50 pins MB connector
	//BOARD_Init_BRIDGE_UART();		// connected to KW41Z <=> used for FSCI
	/* I2C */
	// I2C1: Sensors + Touch: FXOS8700, FXAS21002, MPL3115, ENS210, TSL25711, CCS811 (behind I2C switch), SX9500
	// I2C2: after I2C switch (...NTAG_I2C_EN): NT3H2211, A1006, A71
	/* Flextimers, PWMs */
	BOARD_Init_BUZZER();			// PWM Initialized (4kHz/50% duty cycle) but timer not started
	/* SPIs */
	BOARD_Init_RTC_SPI();			// Real Time Clock PCF2123: Provide 32kHz clock to K64F and KW41Z
	BOARD_Init_SPI_2();				// External Flash MT25QL128A: Shared with KW41Z !!!
	/* ADCs */
	BOARD_Init_BATSENS();			// ADC: VBat sensing. Needs to be enabled/connected to VBat (...BAT_SENS_EN)

	/* MB Peripherals: UART0, SPI0, I2C0, PWM, ADC NOT INITIALIZED */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_INIT_GPIOS
 * Description   : Configures GPIOs as input or output and initial state
 *
 *END**************************************************************************/
void BOARD_INIT_GPIOS(void){
	// RTC: PCF2123
#ifdef RAPID_IOT_HW_REV_B
    GPIO_PinInit(BOARD_INITPINS_RTC_CLKOE_GPIO, BOARD_INITPINS_RTC_CLKOE_GPIO_PIN, &gpio_output_high_config);            // RTC PCF2123 Clock output CLKOUT enabled
#else
	GPIO_PinInit(BOARD_INITPINS_RTC_CLKOE_GPIO, BOARD_INITPINS_RTC_CLKOE_GPIO_PIN, &gpio_output_low_config); 			// RTC PCF2123 Clock output CLKOUT disabled
#endif
	GPIO_PinInit(BOARD_INITPINS_RTC_INT_GPIO, BOARD_INITPINS_RTC_INT_GPIO_PIN, &gpio_input_config);						// Interrupt from PCF2123

	// KW41
	GPIO_PinInit(BOARD_INITPINS_KW41_RST_GPIO, BOARD_INITPINS_KW41_RST_GPIO_PIN, &gpio_output_high_config); 			// KW41 not Reset
	GPIO_PinInit(BOARD_INITPINS_KW41_WAKE_UP_GPIO, BOARD_INITPINS_KW41_WAKE_UP_GPIO_PIN, &gpio_output_low_config);		// KW41_WAKE_UP to Low
	GPIO_PinInit(BOARD_INITPINS_KW41_PB18_GPIO, BOARD_INITPINS_KW41_PB18_GPIO_PIN, &gpio_output_high_config);			// KW41_PB18 to High

	// Batsens
	GPIO_PinInit(BOARD_INITPINS_BAT_SENS_EN_GPIO, BOARD_INITPINS_BAT_SENS_EN_GPIO_PIN, &gpio_output_low_config);		// Bat Sense Disabled

	// User buttons
	GPIO_PinInit(BOARD_INITPINS_USER_SW1_GPIO, BOARD_INITPINS_USER_SW1_GPIO_PIN, &gpio_input_config);
	GPIO_PinInit(BOARD_INITPINS_USER_SW2_GPIO, BOARD_INITPINS_USER_SW2_GPIO_PIN, &gpio_input_config);
	GPIO_PinInit(BOARD_INITPINS_USER_SW3_GPIO, BOARD_INITPINS_USER_SW3_GPIO_PIN, &gpio_input_config);
	GPIO_PinInit(BOARD_INITPINS_USER_SW4_GPIO, BOARD_INITPINS_USER_SW4_GPIO_PIN, &gpio_input_config);

	// Display and backlight
	GPIO_PinInit(BOARD_INITPINS_DISP_EN_GPIO, BOARD_INITPINS_DISP_EN_GPIO_PIN, &gpio_output_low_config); 				// Display and backlight power supply Disabled
	GPIO_PinInit(BOARD_INITPINS_DISP_DISP_GPIO, BOARD_INITPINS_DISP_DISP_GPIO_PIN, &gpio_output_low_config); 			// Display Disabled
	GPIO_PinInit(BOARD_INITPINS_DISP_EXTMODE_GPIO, BOARD_INITPINS_DISP_EXTMODE_GPIO_PIN, &gpio_output_low_config);		// Display Extmode Low = SW

	// Touch controller: SX9500
	GPIO_PinInit(BOARD_INITPINS_TOUCH_RST_GPIO, BOARD_INITPINS_TOUCH_RST_GPIO_PIN, &gpio_output_low_config); 			// Force resetN
	GPIO_PinInit(BOARD_INITPINS_TOUCH_TXEN_GPIO, BOARD_INITPINS_TOUCH_TXEN_GPIO_PIN, &gpio_output_low_config);			// Touch disabled
	GPIO_PinInit(BOARD_INITPINS_TOUCH_INT_GPIO, BOARD_INITPINS_TOUCH_INT_GPIO_PIN, &gpio_input_config);					// Interrupt pin as input

	// Docking station: MB1, MB2, MB3
	GPIO_PinInit(BOARD_INITPINS_MB1_RST_GPIO, BOARD_INITPINS_MB1_RST_GPIO_PIN, &gpio_output_low_config);
	GPIO_PinInit(BOARD_INITPINS_MB1_INT_GPIO, BOARD_INITPINS_MB1_INT_GPIO_PIN, &gpio_input_config);						// Interrupt from MB1

	GPIO_PinInit(BOARD_INITPINS_MB2_RST_GPIO, BOARD_INITPINS_MB2_RST_GPIO_PIN, &gpio_output_low_config);
	GPIO_PinInit(BOARD_INITPINS_MB2_INT_GPIO, BOARD_INITPINS_MB2_INT_GPIO_PIN, &gpio_input_config);						// Interrupt from MB2

	GPIO_PinInit(BOARD_INITPINS_MB3_RST_GPIO, BOARD_INITPINS_MB3_RST_GPIO_PIN, &gpio_output_low_config);
	GPIO_PinInit(BOARD_INITPINS_MB3_INT_GPIO, BOARD_INITPINS_MB3_INT_GPIO_PIN, &gpio_input_config);						// Interrupt from MB3

	// Light sensor: TSL25711
	GPIO_PinInit(BOARD_INITPINS_AMB_INT_GPIO, BOARD_INITPINS_AMB_INT_GPIO_PIN, &gpio_input_config);						// Interrupt from TSL25711

	// Pressure sensor: MPL3115A2
	GPIO_PinInit(BOARD_INITPINS_PRESSURE_INT1_GPIO, BOARD_INITPINS_PRESSURE_INT1_GPIO_PIN, &gpio_input_config);			// Interrupt 1 from MPL3115
	GPIO_PinInit(BOARD_INITPINS_PRESSURE_INT2_GPIO, BOARD_INITPINS_PRESSURE_INT2_GPIO_PIN, &gpio_input_config);			// Interrupt 2 from MPL3115

	// Accelerometer/Magnetometer: FXOS8700
	GPIO_PinInit(BOARD_INITPINS_ACCEL_RST_GPIO, BOARD_INITPINS_ACCEL_RST_GPIO_PIN, &gpio_output_low_config); 			// Release reset
	GPIO_PinInit(BOARD_INITPINS_ACCEL_INT1_GPIO, BOARD_INITPINS_ACCEL_INT1_GPIO_PIN, &gpio_input_config);				// Interrupt 1 from FXOS8700
	GPIO_PinInit(BOARD_INITPINS_ACCEL_INT2_GPIO, BOARD_INITPINS_ACCEL_INT2_GPIO_PIN, &gpio_input_config);				// Interrupt 2 from FXOS8700

	// Gyroscope: FXAS21002
	GPIO_PinInit(BOARD_INITPINS_GYRO_RST_GPIO, BOARD_INITPINS_GYRO_RST_GPIO_PIN, &gpio_output_high_config); 			// Release resetN
	GPIO_PinInit(BOARD_INITPINS_GYRO_INT1_GPIO, BOARD_INITPINS_GYRO_INT1_GPIO_PIN, &gpio_input_config);					// Interrupt 1 from FXAS21002
	GPIO_PinInit(BOARD_INITPINS_GYRO_INT2_GPIO, BOARD_INITPINS_GYRO_INT2_GPIO_PIN, &gpio_input_config);					// Interrupt 2 from FXAS21002

	// Air quality gas sensor CCS811
	GPIO_PinInit(BOARD_INITPINS_AIR_QUALITY_EN_GPIO, BOARD_INITPINS_AIR_QUALITY_EN_GPIO_PIN, &gpio_output_low_config); 		// Air quality power supply OFF
	GPIO_PinInit(BOARD_INITPINS_AIR_QUALITY_I2C_EN_GPIO, BOARD_INITPINS_AIR_QUALITY_I2C_EN_GPIO_PIN, &gpio_output_low_config); // Air quality I2C Switch OFF
	GPIO_PinInit(BOARD_INITPINS_AIR_RESETN_GPIO, BOARD_INITPINS_AIR_RESETN_GPIO_PIN, &gpio_output_high_config); 				// nRESET released
	GPIO_PinInit(BOARD_INITPINS_AIR_WAKEN_GPIO, BOARD_INITPINS_AIR_WAKEN_GPIO_PIN, &gpio_output_low_config); 					// nWAKE released
	GPIO_PinInit(BOARD_INITPINS_AIR_INTN_GPIO, BOARD_INITPINS_AIR_INTN_GPIO_PIN, &gpio_input_config);							// Interrupt from CCS811

	// NTAG I2C & A1006/A71CH
	GPIO_PinInit(BOARD_INITPINS_NTAG_EN_GPIO, BOARD_INITPINS_NTAG_EN_GPIO_PIN, &gpio_output_low_config); 				// NTAG, A1006, A71CH power supply OFF
	GPIO_PinInit(BOARD_INITPINS_NTAG_I2C_EN_GPIO, BOARD_INITPINS_NTAG_I2C_EN_GPIO_PIN, &gpio_output_low_config); 		// NTAG, A1006, A71CH I2C Switch OFF
	GPIO_PinInit(BOARD_INITPINS_NTAG_FD_GPIO, BOARD_INITPINS_NTAG_FD_GPIO_PIN, &gpio_input_config);						// NTAG Field detect Input

	GPIO_PinInit(BOARD_INITPINS_AUTH_RST_GPIO, BOARD_INITPINS_AUTH_RST_GPIO_PIN, &gpio_output_high_config); 			// A71 Release resetN
}

/********************/
/* LPTMR0 Functions */
/********************/
void LPTMR0_Init(void)
{
	lptmr_config_t lptmrConfig;

    /* Configure LPTMR */
    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);

    /* Initialize the LPTMR */
    LPTMR_Init(LPTMR0, &lptmrConfig);

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

    /* Enable at the NVIC */
    EnableIRQ(LPTMR0_IRQn);

    lptmr_init_status = 1; 	// LPTMR initialized
}

#if !defined(RAPID_IOT)
void LPTMR0_IRQHandler(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
    lptmrFlag = 1;
    /*
     * Workaround for TWR-KV58: because write buffer is enabled, adding
     * memory barrier instructions to make sure clearing interrupt flag completed
     * before go out ISR
     */
    __DSB();
    __ISB();
}
#endif

/*******************************************************************************
 * App_WaitUsec: Delay in microsecond
 * basic wait implementation using LPTMR HW resource
 * Min value depends on kCLOCK_LpoClk
 ******************************************************************************/
void App_WaitUsec(uint32_t microsec){

	/* If Not, initialize the LPTMR */
	if(lptmr_init_status!=1)
		LPTMR0_Init();

     /* Set timer period. */
	if(USEC_TO_COUNT(microsec, CLOCK_GetFreq(kCLOCK_LpoClk))>0)
		LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(microsec, CLOCK_GetFreq(kCLOCK_LpoClk)));
	else
		LPTMR_SetTimerPeriod(LPTMR0, 1);

    LPTMR_StartTimer(LPTMR0);
	while(lptmrFlag == 0)
	{
	}
	LPTMR_StopTimer(LPTMR0);
	lptmrFlag = 0;
}

/*******************************************************************************
 * App_WaitMsec: Delay in millisecond
 * basic wait implementation using LPTMR HW resource
 ******************************************************************************/
void App_WaitMsec(uint32_t millisec){

	/* If Not, initialized the LPTMR */
	if(lptmr_init_status!=1)
		LPTMR0_Init();

   /* Set timer period. */
	if(MSEC_TO_COUNT(millisec, CLOCK_GetFreq(kCLOCK_LpoClk))>0)
		LPTMR_SetTimerPeriod(LPTMR0, MSEC_TO_COUNT(millisec, CLOCK_GetFreq(kCLOCK_LpoClk)));
	else
		LPTMR_SetTimerPeriod(LPTMR0, 1);

    LPTMR_StartTimer(LPTMR0);
	while(lptmrFlag == 0)
	{
	}
	LPTMR_StopTimer(LPTMR0);
	lptmrFlag = 0;
}

/* Connect the RGB LED */
void Rgb_Led_Connect(void){
	/* Check if already connected */
	if (rgb_led_connect_status == 0){
		/* Start Flextimer (PWM) */
	    if ((backlight_connect_status == 0) && (display_connect_status == 0)) /* Flextimer shared with backlight and display */
		    FTM3_SHARED_TIMER_START;

		rgb_led_connect_status = 1;
	}
}

/* Disconnect the RGB LED */
void Rgb_Led_Disconnect(void){
	/* Check if connected */
	if (rgb_led_connect_status == 1){
		/* Stop Flextimer (PWM) */
	    if ((backlight_connect_status == 0) && (display_connect_status == 0)) /* Flextimer shared with backlight and display */
		    FTM3_SHARED_TIMER_STOP;

		rgb_led_connect_status = 0;
	}
}

/* Set the color the RGB LED by changing the duty cycle of each individual LED
 * R, G and B values should be between 0 (LED off) and 100 (LED on)
 */
void Rgb_Led_Set_Colors(uint8_t R, uint8_t G, uint8_t B){
	FTM_UpdatePwmDutycycle(BOARD_INITPINS_RGB_R_PERIPHERAL, BOARD_INITPINS_RGB_R_CHANNEL, kFTM_CenterAlignedPwm, R);
	FTM_UpdatePwmDutycycle(BOARD_INITPINS_RGB_G_PERIPHERAL, BOARD_INITPINS_RGB_G_CHANNEL, kFTM_CenterAlignedPwm, G);
	FTM_UpdatePwmDutycycle(BOARD_INITPINS_RGB_B_PERIPHERAL, BOARD_INITPINS_RGB_B_CHANNEL, kFTM_CenterAlignedPwm, B);
	FTM_SetSoftwareTrigger(BOARD_INITPINS_RGB_R_PERIPHERAL, true);	// RGB LEDs on the same FTM = FTM3 (channel 2, 4, 5)
}

#ifndef BOOTLOADER
/* Connect the backlight */
void Backlight_Connect(void){
	/* Check if already connected */
	if (backlight_connect_status == 0){
		/* Enable backlight power supply (VCC_DISP) */
	    if (display_connect_status == 0){
            DSPI2_InitPins();
            GPIO_WritePinOutput(BOARD_INITPINS_DISP_EN_GPIO, BOARD_INITPINS_DISP_EN_GPIO_PIN, 1);
            App_WaitMsec(1); /* T2 = 1ms in LPM013M126C datasheet */
	    }
		/* Start Flextimer (PWM) */
		if ((rgb_led_connect_status == 0) && (display_connect_status == 0)) /* Flextimer shared with RGB LED and display */
		    FTM3_SHARED_TIMER_START;

		backlight_connect_status = 1;
	}
}

/* Disconnect the backlight */
void Backlight_Disconnect(void){
	/* Check if connected */
	if (backlight_connect_status == 1){
		/* Disable backlight power supply (VCC_DISP) */
	    if (display_connect_status == 0){
	        DSPI2_DeinitPins();
	        GPIO_WritePinOutput(BOARD_INITPINS_DISP_EN_GPIO, BOARD_INITPINS_DISP_EN_GPIO_PIN, 0);
	    }
		/* Stop Flextimer (PWM) */
		if ((rgb_led_connect_status == 0) && (display_connect_status == 0)) /* Flextimer shared with RGB LED and display */
		    FTM3_SHARED_TIMER_STOP;

		backlight_connect_status = 0;
	}
}

/* Set the Backlight level by changing the duty cycle
 * R, G and B values should be between 0 (LED off) and 100 (LED on)
 */
void Backlight_Set_Level(uint8_t level){
    FTM_UpdatePwmDutycycle(BOARD_INITPINS_DISP_BLIGHT_PERIPHERAL, BOARD_INITPINS_DISP_BLIGHT_CHANNEL, kFTM_CenterAlignedPwm, level);
    FTM_SetSoftwareTrigger(BOARD_INITPINS_DISP_BLIGHT_PERIPHERAL, true);
}

/* Connect the display */
void Display_Connect(void){
    /* Check if already connected */
    if (display_connect_status == 0){
        /* Enable display power supply (VCC_DISP) */
        if (backlight_connect_status == 0){
            DSPI2_InitPins();
            GPIO_WritePinOutput(BOARD_INITPINS_DISP_EN_GPIO, BOARD_INITPINS_DISP_EN_GPIO_PIN, 1);
            App_WaitMsec(1); /* T2 = 1ms in LPM013M126C datasheet */
        }
        GUI_Init(); /* clear memory internal data (initial data is black) */
        /* Enable display on/off switching signal (DISP_DISP) */
        GPIO_WritePinOutput(BOARD_INITPINS_DISP_DISP_GPIO, BOARD_INITPINS_DISP_DISP_GPIO_PIN, 1);
        App_WaitUsec(30); /* T3 = 30us in LPM013M126C datasheet */
        /* Enable EXTCOMIN signal (DISP_EXTMODE) */
        GPIO_WritePinOutput(BOARD_INITPINS_DISP_EXTMODE_GPIO, BOARD_INITPINS_DISP_EXTMODE_GPIO_PIN, 1);
        App_WaitUsec(30); /* T4 = 30us in LPM013M126C datasheet */
        /* Start Flextimer (PWM) */
        if ((rgb_led_connect_status == 0) && (backlight_connect_status == 0)) /* Flextimer shared with RGB LED and backlight */
            FTM3_SHARED_TIMER_START;

        display_connect_status = 1;
    }
}

/* Disconnect the display */
void Display_Disconnect(void){
    /* Check if connected */
    if (display_connect_status == 1){
        GUI_Exit();
        /* Disable display on/off switching signal (DISP_DISP) */
        GPIO_WritePinOutput(BOARD_INITPINS_DISP_DISP_GPIO, BOARD_INITPINS_DISP_DISP_GPIO_PIN, 0);
        App_WaitMsec(1); /* T5 = 1ms in LPM013M126C datasheet */
        /* Disable EXTCOMIN signal (DISP_EXTMODE) */
        GPIO_WritePinOutput(BOARD_INITPINS_DISP_EXTMODE_GPIO, BOARD_INITPINS_DISP_EXTMODE_GPIO_PIN, 0);
        /* Disable display power supply (VCC_DISP) */
        if (backlight_connect_status == 0){
            DSPI2_DeinitPins();
            GPIO_WritePinOutput(BOARD_INITPINS_DISP_EN_GPIO, BOARD_INITPINS_DISP_EN_GPIO_PIN, 0);
        }
        /* Stop Flextimer (PWM) */
        if ((rgb_led_connect_status == 0) && (backlight_connect_status == 0)) /* Flextimer shared with RGB LED and backlight */
            FTM3_SHARED_TIMER_STOP;

        display_connect_status = 0;
    }
}
#endif

/* Connect the CCS811 to the K64F & switch it ON */
void CCS811_Connect(void){
	/* Check if already connected */
	if (ccs811_connect_status == 0){
		// 1- Enable CCS811 power supply
		GPIO_WritePinOutput(BOARD_INITPINS_AIR_QUALITY_EN_GPIO, BOARD_INITPINS_AIR_QUALITY_EN_GPIO_PIN, 1);
		// 2- Wait until power is high (switch delay + capacitor charging)
		App_WaitUsec(100); // 100us
		// 3- Enable I2C switch
		GPIO_WritePinOutput(BOARD_INITPINS_AIR_QUALITY_I2C_EN_GPIO, BOARD_INITPINS_AIR_QUALITY_I2C_EN_GPIO_PIN, 1);
        // 4- Toggle nRESET
        GPIO_WritePinOutput(BOARD_INITPINS_AIR_RESETN_GPIO, BOARD_INITPINS_AIR_RESETN_GPIO_PIN, 0);
        App_WaitUsec(20); // 20us
        GPIO_WritePinOutput(BOARD_INITPINS_AIR_RESETN_GPIO, BOARD_INITPINS_AIR_RESETN_GPIO_PIN, 1);
		// 5- Wait until ready (I2C switch ON = 300us + CCS811 boot Time after Power ON = 20ms)
		App_WaitMsec(20); // 20ms

		ccs811_connect_status = 1;
	}
}

/* Disconnect the CCS811 from the K64F & switch it OFF */
void CCS811_Disconnect(void){
	/* Check if connected */
	if (ccs811_connect_status == 1){
		// 2- Disable I2C switch
		GPIO_WritePinOutput(BOARD_INITPINS_AIR_QUALITY_I2C_EN_GPIO, BOARD_INITPINS_AIR_QUALITY_I2C_EN_GPIO_PIN, 0);
		// 3- Wait until the switch is OFF (100us)
		App_WaitUsec(100);
		// 4- Disable CCS811 power supply
		GPIO_WritePinOutput(BOARD_INITPINS_AIR_QUALITY_EN_GPIO, BOARD_INITPINS_AIR_QUALITY_EN_GPIO_PIN, 0);

		ccs811_connect_status = 0;
	}
}


/* Connect the NTAG/A1006/A71 to the K64F & switch them ON */
void Connect_NTAG_A1006(void){
	/* Check if already connected */
	if (ntag_a1006_connect_status ==0){
		// 1- Enable NTAG/A1006 power supply
		GPIO_WritePinOutput(BOARD_INITPINS_NTAG_EN_GPIO, BOARD_INITPINS_NTAG_EN_GPIO_PIN, 1);
		// 2- Wait until power is high (switch delay + capacitor charging)
		App_WaitUsec(100);
		// 3- Enable I2C switch
		GPIO_WritePinOutput(BOARD_INITPINS_NTAG_I2C_EN_GPIO, BOARD_INITPINS_NTAG_I2C_EN_GPIO_PIN, 1);
		// 4- Wait until the switch is ON (300us + A1006 boot Time after Power ON = 1.5ms)
		App_WaitMsec(2);

		ntag_a1006_connect_status = 1;
	}
}

/* Disconnect the NTAG/A1006/A71 from the K64F & switch it OFF */
void Disconnect_NTAG_A1006(void){
	/* Check if connected */
	if (ntag_a1006_connect_status ==1){
		// 1- Disable I2C switch
		GPIO_WritePinOutput(BOARD_INITPINS_NTAG_I2C_EN_GPIO, BOARD_INITPINS_NTAG_I2C_EN_GPIO_PIN, 0);
		// Wait until the switch is OFF (100us)
		App_WaitUsec(100);
		// 3- Disable NTAG_A1006 power supply
		GPIO_WritePinOutput(BOARD_INITPINS_NTAG_EN_GPIO, BOARD_INITPINS_NTAG_EN_GPIO_PIN, 0);

		ntag_a1006_connect_status = 0;
	}
}

/* Connect the touch controller SX9500 */
void Touch_Controller_Connect(void){
	// Release SX9500 NRST pin
	GPIO_WritePinOutput(BOARD_INITPINS_TOUCH_RST_GPIO, BOARD_INITPINS_TOUCH_RST_GPIO_PIN, 1);
	// Wait 1ms (TPOR, typical delay)
	App_WaitMsec(1);
	// Enable SX9500 TXEN pin
	GPIO_WritePinOutput(BOARD_INITPINS_TOUCH_TXEN_GPIO, BOARD_INITPINS_TOUCH_TXEN_GPIO_PIN, 1);
}

/* Disconnect the touch controller SX9500 */
void Touch_Controller_Disconnect(void){
	// Disable SX9500 TXEN pin
	GPIO_WritePinOutput(BOARD_INITPINS_TOUCH_TXEN_GPIO, BOARD_INITPINS_TOUCH_TXEN_GPIO_PIN, 0);
	// Hold SX9500 NRST pin
	GPIO_WritePinOutput(BOARD_INITPINS_TOUCH_RST_GPIO, BOARD_INITPINS_TOUCH_RST_GPIO_PIN, 0);
	// Wait 2us (TRESETPW)
	App_WaitUsec(2);
}

uint32_t BOARD_GetUartClock(uint32_t instance)
{
    clock_name_t UART_CLKs[6] = {
        UART0_CLK_SRC,
        UART1_CLK_SRC,
        UART2_CLK_SRC,
        UART3_CLK_SRC,
        UART4_CLK_SRC,
        UART5_CLK_SRC
    };
    return CLOCK_GetFreq(UART_CLKs[instance]);
}

uint32_t BOARD_GetFtmClock(uint32_t instance)
{
    instance = instance; /* Remove compiler warnings */
    return CLOCK_GetFreq(kCLOCK_BusClk);
}

uint32_t BOARD_GetSpiClock(uint32_t instance)
{
    instance = instance; /* Remove compiler warnings */
    return CLOCK_GetFreq(kCLOCK_BusClk);
}

void MK64F_hardware_init()
{
	BOARD_InitBootPins();
	BOARD_INIT_GPIOS();
	BOARD_InitBootClocks();
	OS_App_WaitMsec(100);			// Allocate time for Power and I2C switches
#ifndef BOOTLOADER
	BOARD_InitPeripherals();	// File located in board.c
#endif
	BOARD_Init_RGB_BLIGHT(); /* FTM3 init: common to RGB LED, backlight, display */
	BOARD_Init_RPK_LEDs();
}

void BOARD_InitSPI()
{
	// SPI not initialized here
}
void BOARD_InitUART()
{
	BOARD_Init_BRIDGE_UART();		// connected to KW41Z <=> used for FSCI
}

void I2C1_InitPins(void)
{
    /* function needed by fsl_i2c_cmsis.c */
    /* TODO: should ideally contain the I2C1 config done in pin_mux.c */
}

void I2C1_DeinitPins(void)
{
    /* function needed by fsl_i2c_cmsis.c */
}

uint32_t I2C1_GetFreq(void)
{
    /* function needed by fsl_i2c_cmsis.c */
    return CLOCK_GetFreq(I2C1_CLK_SRC);
}


