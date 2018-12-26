#ifndef _PIN_MUX_RPK_H_
#define _PIN_MUX_RPK_H_


//! @addtogroup pin_mux_rpk
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Externals
////////////////////////////////////////////////////////////////////////////////

extern gpio_pin_config_t rpk_gpio_output_low_config;
extern gpio_pin_config_t rpk_gpio_output_high_config;
extern gpio_pin_config_t rpk_gpio_output_low_config;
extern gpio_pin_config_t rpk_gpio_input_config;


#if defined(__cplusplus)
extern "C" {
#endif

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

void BOARD_Init_RPK(void);
void BOARD_InitSPIBusSharing(void);
void BOARD_ConfigurePins_RPK_Connectivity(void);
void BOARD_InitPins_RPK_Connectivity(void);
void BOARD_ConfigurePins_RPK_SPI_Bus(void);
void BOARD_UnConfigurePins_RPK_SPI_Bus(void);
void BOARD_Init_Flash_SPI(void);
#ifdef CPU_MK64FN1M0VMD12
void BOARD_Put_K41_InReset();
void BOARD_Release_K41_Reset();
#endif

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_RPK_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
