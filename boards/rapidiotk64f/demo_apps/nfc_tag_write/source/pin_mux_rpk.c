#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"
#include "pin_mux.h"
#include "pin_mux_rpk.h"
#include "spi_flash_driver.h"
#if defined(BOOTLOADER)
#include "microseconds.h"
#endif
#include "rpk_led.h"
#include "spi_bus_share.h"
#include "clock_config.h"
#include "hardware_init_MK64F12.h"
#if defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
#include "port_interrupts.h"
#endif

//! @addtogroup pin_mux_rpk
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#if defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
/* Define the init structure for the output GPIO Output High pin*/
gpio_pin_config_t rpk_gpio_output_high_config = {
    kGPIO_DigitalOutput, 1,
};

/* Define the init structure for the output GPIO Output Low pin*/
gpio_pin_config_t rpk_gpio_output_low_config = {
    kGPIO_DigitalOutput, 0,
};

/* Define the init structure for the input GPIO */
gpio_pin_config_t rpk_gpio_input_config = {
    kGPIO_DigitalInput, 1,
};
#endif

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////


void BOARD_Init_RPK(void)
{

#if defined(BOOTLOADER)
	microseconds_init();
#if defined(CPU_MKW41Z512VHT4)
	microseconds_delay(1500000);
#else
	microseconds_delay(1000000);
#endif
#endif
	// Init pinmux and other hardware setup.
#if defined(CPU_MKW41Z512VHT4)
	MKW41Z_init_hardware();
	MKW41Z_hardware_init();
#elif defined(CPU_MK64FN1M0VMD12)
	MK64F_init_hardware();
	MK64F_hardware_init();
#endif

#if defined(CPU_MKW41Z512VHT4)

	BOARD_InitBootPins();

	/* Port C Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortC);

	BOARD_InitBootClocks();

	BOARD_Init_RPK_LEDs();

#endif

#ifdef CPU_MK64FN1M0VMD12
#ifdef BOOTLOADER
	// initialize it in reset for the bootloader
 	GPIO_PinInit(BOARD_INITPINS_KW41_RST_GPIO,   BOARD_INITPINS_KW41_RST_PIN, &rpk_gpio_output_low_config);
#else
 	//GPIO_PinInit(BOARD_INITPINS_KW41_RST_GPIO,   BOARD_INITPINS_KW41_RST_PIN, &rpk_gpio_output_high_config);
#endif
#endif

	//BOARD_InitSPIBusSharing();

#ifdef CPU_MK64FN1M0VMD12
	PORT_IRQ_EnablePortAIrq();
#endif
}


void BOARD_InitSPIBusSharing(void)
{
	// Initialize shared SPI Bus, SPI Bus processor communications, and SPI Flash
	BOARD_ConfigurePins_RPK_Connectivity();
	BOARD_InitPins_RPK_Connectivity();
	BOARD_ConfigurePins_RPK_SPI_Bus();
	SPI_Bus_Share_Init();
	BOARD_Init_Flash_SPI();
#ifdef CPU_MK64FN1M0VMD12
	PORT_IRQ_EnablePortEIrq();
#endif
	SPI_Bus_Share_Release_Access();
}


void BOARD_ConfigurePins_RPK_Connectivity(void)
{

#if defined(RAPID_IOT) && defined(CPU_MKW41Z512VHT4)
	// Configure the K64F connectivity pins

	PORT_SetPinMux(BOARD_INITPINS_K64F_UART_RTS_PC5_PORT, BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO_PIN, kPORT_MuxAsGpio);
	PORT_SetPinMux(BOARD_INITPINS_K64F_UART_CTS_PC4_PORT, BOARD_INITPINS_K64F_UART_CTS_PC4_GPIO_PIN, kPORT_MuxAsGpio);
	PORT_SetPinMux(BOARD_INITPINS_K64F_WAKE_UP_PB0_PORT,  BOARD_INITPINS_K64F_WAKE_UP_PB0_GPIO_PIN,  kPORT_MuxAsGpio);
	PORT_SetPinMux(BOARD_INITPINS_K64F_PB18_PORT,         BOARD_INITPINS_K64F_PB18_GPIO_PIN,         kPORT_MuxAsGpio);

#elif defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
	// Configure the KW41Z connectivity pins

	PORT_SetPinMux(BOARD_INITPINS_KW41_UART_RTS_PORT, BOARD_INITPINS_KW41_UART_RTS_PIN, kPORT_MuxAsGpio);
	PORT_SetPinMux(BOARD_INITPINS_KW41_UART_CTS_PORT, BOARD_INITPINS_KW41_UART_CTS_PIN, kPORT_MuxAsGpio);
	PORT_SetPinMux(BOARD_INITPINS_KW41_WAKE_UP_PORT, BOARD_INITPINS_KW41_WAKE_UP_PIN,   kPORT_MuxAsGpio);
	PORT_SetPinMux(BOARD_INITPINS_KW41_PB18_PORT,    BOARD_INITPINS_KW41_PB18_GPIO_PIN, kPORT_MuxAsGpio);

#endif
}

 void BOARD_InitPins_RPK_Connectivity(void)
 {

 #if defined(RAPID_IOT) && defined(CPU_MKW41Z512VHT4)
 	// Configure the K64F connectivity pins
 	GPIO_PinInit(BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO,   BOARD_INITPINS_K64F_UART_RTS_PC5_GPIO_PIN, &rpk_gpio_output_low_config);
 	GPIO_PinInit(BOARD_INITPINS_K64F_UART_CTS_PC4_GPIO,   BOARD_INITPINS_K64F_UART_CTS_PC4_GPIO_PIN, &rpk_gpio_input_config);
 	GPIO_PinInit(BOARD_INITPINS_K64F_WAKE_UP_PB0_GPIO,    BOARD_INITPINS_K64F_WAKE_UP_PB0_GPIO_PIN,  &rpk_gpio_input_config);
 	GPIO_PinInit(BOARD_INITPINS_K64F_PB18_GPIO,           BOARD_INITPINS_K64F_PB18_GPIO_PIN,         &rpk_gpio_input_config);

 #elif defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
 	// Configure the KW41Z connectivity pins
 	GPIO_PinInit(BOARD_INITPINS_KW41_UART_RTS_GPIO, BOARD_INITPINS_KW41_UART_RTS_PIN, &rpk_gpio_input_config);
 	GPIO_PinInit(BOARD_INITPINS_KW41_UART_CTS_GPIO, BOARD_INITPINS_KW41_UART_CTS_PIN, &rpk_gpio_output_low_config);
 	GPIO_PinInit(BOARD_INITPINS_KW41_WAKE_UP_GPIO,  BOARD_INITPINS_KW41_WAKE_UP_PIN,  &rpk_gpio_output_low_config);
 	GPIO_PinInit(BOARD_INITPINS_KW41_PB18_GPIO,     BOARD_INITPINS_KW41_PB18_PIN,     &rpk_gpio_output_high_config);

 #endif
}

void BOARD_ConfigurePins_RPK_SPI_Bus(void)
{
#if defined(RAPID_IOT) && defined(CPU_MKW41Z512VHT4)
 	// if running on KW41Z
     PORT_SetPinMux(BOARD_INITPINS_MT25_SPI0_SCK_PC16_PORT,  BOARD_INITPINS_MT25_SPI0_SCK_PC16_GPIO_PIN,  kPORT_MuxAlt2);
     PORT_SetPinMux(BOARD_INITPINS_MT25_SPI0_MOSI_PC17_PORT, BOARD_INITPINS_MT25_SPI0_MOSI_PC17_GPIO_PIN, kPORT_MuxAlt2);
     PORT_SetPinMux(BOARD_INITPINS_MT25_SPI0_MISO_PC18_PORT, BOARD_INITPINS_MT25_SPI0_MISO_PC18_GPIO_PIN, kPORT_MuxAlt2);
     PORT_SetPinMux(BOARD_INITPINS_MT25_SPI0_PCS0_PC19_PORT, BOARD_INITPINS_MT25_SPI0_PCS0_PC19_GPIO_PIN, kPORT_MuxAlt2);
#elif defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
 	// if running on K64F
     PORT_SetPinMux(BOARD_INITPINS_SPI1_SCK_PORT,  BOARD_INITPINS_SPI1_SCK_PIN,   kPORT_MuxAlt7);
     PORT_SetPinMux(BOARD_INITPINS_SPI1_MOSI_PORT, BOARD_INITPINS_SPI1_MOSI_PIN,  kPORT_MuxAlt7);
     PORT_SetPinMux(BOARD_INITPINS_SPI1_MISO_PORT, BOARD_INITPINS_SPI1_MISO_PIN,  kPORT_MuxAlt7);
 	 PORT_SetPinMux(BOARD_INITPINS_SPI1_PCS0_PORT, BOARD_INITPINS_SPI1_PCS0_PIN,  kPORT_MuxAlt7);
#endif
}


void BOARD_UnConfigurePins_RPK_SPI_Bus(void)
{
#if defined(RAPID_IOT) && defined(CPU_MKW41Z512VHT4)
	// if running on KW41Z, configure Flash SPI pins that connect to K64F are disabled so there is no conflict
    PORT_SetPinMux(BOARD_INITPINS_MT25_SPI0_SCK_PC16_PORT,  BOARD_INITPINS_MT25_SPI0_SCK_PC16_GPIO_PIN,  kPORT_PinDisabledOrAnalog);
    PORT_SetPinMux(BOARD_INITPINS_MT25_SPI0_MOSI_PC17_PORT, BOARD_INITPINS_MT25_SPI0_MOSI_PC17_GPIO_PIN, kPORT_PinDisabledOrAnalog);
    PORT_SetPinMux(BOARD_INITPINS_MT25_SPI0_MISO_PC18_PORT, BOARD_INITPINS_MT25_SPI0_MISO_PC18_GPIO_PIN, kPORT_PinDisabledOrAnalog);
    PORT_SetPinMux(BOARD_INITPINS_MT25_SPI0_PCS0_PC19_PORT, BOARD_INITPINS_MT25_SPI0_PCS0_PC19_GPIO_PIN, kPORT_PinDisabledOrAnalog);
#elif defined(RAPID_IOT) && defined(CPU_MK64FN1M0VMD12)
	// if running on K64F, configure Flash SPI pins that connect to KW41Z as inputs so there is no conflict
    PORT_SetPinMux(BOARD_INITPINS_SPI1_SCK_PORT,  BOARD_INITPINS_SPI1_SCK_PIN,  kPORT_PinDisabledOrAnalog);
    PORT_SetPinMux(BOARD_INITPINS_SPI1_MOSI_PORT, BOARD_INITPINS_SPI1_MOSI_PIN, kPORT_PinDisabledOrAnalog);
    PORT_SetPinMux(BOARD_INITPINS_SPI1_MISO_PORT, BOARD_INITPINS_SPI1_MISO_PIN, kPORT_PinDisabledOrAnalog);
    PORT_SetPinMux(BOARD_INITPINS_SPI1_PCS0_PORT, BOARD_INITPINS_SPI1_PCS0_PIN, kPORT_PinDisabledOrAnalog);

#endif
}

void BOARD_Init_Flash_SPI(void)
{
  /* Initialize components */
	FLASH_SPI_init();
}

#ifdef CPU_MK64FN1M0VMD12

void BOARD_Put_K41_InReset()
{
	GPIO_WritePinOutput(BOARD_INITPINS_KW41_RST_GPIO, BOARD_INITPINS_KW41_RST_PIN, 0);
}

void BOARD_Release_K41_Reset()
{
	GPIO_WritePinOutput(BOARD_INITPINS_KW41_RST_GPIO, BOARD_INITPINS_KW41_RST_PIN, 1);
}

#endif





/*******************************************************************************
 * EOF
 ******************************************************************************/
