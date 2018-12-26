/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
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

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
#include "bootloader_common.h"
#include "bootloader/bl_context.h"
#include "fsl_device_registers.h"
#include "lpc_iocon/fsl_iocon.h"
#include "lpc_gpio/fsl_gpio.h"
#include "gint/fsl_gint.h"
#include "inputmux/fsl_inputmux.h"
#include "pint/fsl_pint.h"
#include "peripherals_pinmux.h"

#if (BL_CONFIG_FLEXCOMM_USART)

#if BL_ENABLE_PINMUX_UART0
#define BL_ENABLED_MAX_UART_INSTANCE (0)
#endif

//! UART autobaud port irq configurations
#define GPIO_IRQC_INTERRUPT_ENABLED_PRIORITY 1
#define GPIO_IRQC_INTERRUPT_RESTORED_PRIORITY 0

//! this is to store the function pointer for calling back to the function that wants
//! the UART RX instance pin that triggered the interrupt. This only supports 1 pin
//! for UART0 because UART1 is on PORTC which does not support interrupts :(

static pin_irq_callback_t s_pin_irq_func[BL_ENABLED_MAX_UART_INSTANCE + 1] = { 0 };

#endif // BL_CONFIG_FLEXCOMM_USART

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

static inline void IOCON_RestoreDefault(IOCON_Type *base, uint32_t port, uint32_t pin)
{
    base->PIO[port][pin] = ((~IOCON_PIO_FUNC_MASK) & base->PIO[port][pin]) | IOCON_PIO_FUNC(IOCON_FUNC0);
}

#if BL_CONFIG_FLEXCOMM_USART
//! @brief Configure the GPIO mode for auto baud detection.
static inline void IOCON_SetUartAutoBaudPinMode(IOCON_Type *ioconBase, GPIO_Type *gpioBase, uint32_t port, uint32_t pin)
{
    IOCON_PinMuxSet(ioconBase, port, pin, IOCON_FUNC0 | IOCON_GPIO_MODE | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);

    // Configure UART RX pin to digital input mode.
    gpioBase->DIR[port] &= ~(1U << pin);
}

static inline void IOCON_SetUartPinMode(IOCON_Type *base, uint32_t port, uint32_t pin, uint32_t mux)
{
    IOCON_PinMuxSet(base, port, pin, (mux | IOCON_MODE_INACT | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF));
}
#endif

#if BL_CONFIG_FLEXCOMM_I2C
static inline void IOCON_SetI2cPinMode(IOCON_Type *base, uint32_t port, uint32_t pin, uint32_t mux)
{
    IOCON_PinMuxSet(base, port, pin, IOCON_MODE_INACT | mux | IOCON_DIGITAL_EN | IOCON_INPFILT_OFF);
}
#endif

#if BL_CONFIG_FLEXCOMM_SPI
static inline void IOCON_SetSpiPinMode(IOCON_Type *base, uint32_t port, uint32_t pin, uint32_t mux)
{
    IOCON_PinMuxSet(base, port, pin, (mux | IOCON_MODE_PULLUP | IOCON_DIGITAL_EN));
}
#endif

/*!
 * @brief Configure pinmux for uart module.
 *
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module)
 */
void uart_pinmux_config(uint32_t instance, pinmux_type_t pinmux)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_UART0
        case 0:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(UART0_TX_IOCON_BASE, UART0_RX_GPIO_PIN_GROUP, UART0_RX_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(UART0_RX_IOCON_BASE, UART0_TX_GPIO_PIN_GROUP, UART0_TX_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_PollForActivity:
                    IOCON_SetUartAutoBaudPinMode(UART0_RX_IOCON_BASE, UART0_RX_GPIO_BASE, UART0_RX_GPIO_PIN_GROUP, UART0_RX_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for UART0.
                    IOCON_SetUartPinMode(UART0_RX_IOCON_BASE, UART0_RX_GPIO_PIN_GROUP, UART0_RX_GPIO_PIN_NUM, UART0_RX_FUNC_ALT_MODE); // Set UART0_RX pin to UART0_RX functionality
                    IOCON_SetUartPinMode(UART0_TX_IOCON_BASE, UART0_TX_GPIO_PIN_GROUP, UART0_TX_GPIO_PIN_NUM, UART0_TX_FUNC_ALT_MODE); // Set UART0_TX pin to UART0_TX functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_UART0

        default:
            break;
    }
}

/*!
 * @brief Configure pinmux for i2c module.
 *
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module)
 */
void i2c_pinmux_config(uint32_t instance, pinmux_type_t pinmux)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_I2C2
        case 2:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(I2C2_SDA_IOCON_BASE, I2C2_SDA_GPIO_PIN_GROUP, I2C2_SDA_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(I2C2_SCL_IOCON_BASE, I2C2_SCL_GPIO_PIN_GROUP, I2C2_SCL_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C2.
                    IOCON_SetI2cPinMode(I2C2_SDA_IOCON_BASE, I2C2_SDA_GPIO_PIN_GROUP, I2C2_SDA_GPIO_PIN_NUM,
                                       I2C2_SDA_FUNC_ALT_MODE); // Set I2C2_SDA pin to I2C2_SDA functionality
                    IOCON_SetI2cPinMode(I2C2_SCL_IOCON_BASE, I2C2_SCL_GPIO_PIN_GROUP, I2C2_SCL_GPIO_PIN_NUM,
                                       I2C2_SCL_FUNC_ALT_MODE); // Set I2C2_SCL pin to I2C2_SCL functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_I2C2

        default:
            break;
    }
}

/*!
 * @brief Configure pinmux for SPI module.
 *
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module)
 */
void spi_pinmux_config(uint32_t instance, pinmux_type_t pinmux)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_SPI3
        case 3:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(SPI3_SSEL_IOCON_BASE, SPI3_SSEL_GPIO_PIN_GROUP, SPI3_SSEL_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI3_SCK_IOCON_BASE, SPI3_SCK_GPIO_PIN_GROUP, SPI3_SCK_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI3_MISO_IOCON_BASE, SPI3_MISO_GPIO_PIN_GROUP, SPI3_MISO_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI3_MOSI_IOCON_BASE, SPI3_MOSI_GPIO_PIN_GROUP, SPI3_MOSI_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for SPI3
                    IOCON_SetSpiPinMode(SPI3_MISO_IOCON_BASE, SPI3_MISO_GPIO_PIN_GROUP, SPI3_MISO_GPIO_PIN_NUM,
                                       SPI3_MISO_FUNC_ALT_MODE); // Set SPI3_MISO pin to SPI3_MISO functionality
                    IOCON_SetSpiPinMode(SPI3_SSEL_IOCON_BASE, SPI3_SSEL_GPIO_PIN_GROUP, SPI3_SSEL_GPIO_PIN_NUM,
                                       SPI3_SSEL_FUNC_ALT_MODE);  // Set SPI3_SSEL pin to SPI3_SSEL functionality
                    IOCON_SetSpiPinMode(SPI3_SCK_IOCON_BASE, SPI3_SCK_GPIO_PIN_GROUP, SPI3_SCK_GPIO_PIN_NUM,
                                       SPI3_SCK_FUNC_ALT_MODE);  // Set SPI3_SCK pin to SPI3_SCK functionality
                    IOCON_SetSpiPinMode(SPI3_MOSI_IOCON_BASE, SPI3_MOSI_GPIO_PIN_GROUP, SPI3_MOSI_GPIO_PIN_NUM,
                                       SPI3_MOSI_FUNC_ALT_MODE);  // Set SPI3_MOSI pin to SPI3_MOSI functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_SPI3

#if BL_ENABLE_PINMUX_SPI9
        case 9:
            switch (pinmux)
            {
                case kPinmuxType_Default:
                    IOCON_RestoreDefault(SPI9_SSEL_IOCON_BASE, SPI9_SSEL_GPIO_PIN_GROUP, SPI9_SSEL_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI9_SCK_IOCON_BASE, SPI9_SCK_GPIO_PIN_GROUP, SPI9_SCK_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI9_MISO_IOCON_BASE, SPI9_MISO_GPIO_PIN_GROUP, SPI9_MISO_GPIO_PIN_NUM);
                    IOCON_RestoreDefault(SPI9_MOSI_IOCON_BASE, SPI9_MOSI_GPIO_PIN_GROUP, SPI9_MOSI_GPIO_PIN_NUM);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for SPI9
                    IOCON_SetSpiPinMode(SPI9_MISO_IOCON_BASE, SPI9_MISO_GPIO_PIN_GROUP, SPI9_MISO_GPIO_PIN_NUM,
                                       SPI9_MISO_FUNC_ALT_MODE); // Set SPI9_MISO pin to SPI9_MISO functionality
                    IOCON_SetSpiPinMode(SPI9_SSEL_IOCON_BASE, SPI9_SSEL_GPIO_PIN_GROUP, SPI9_SSEL_GPIO_PIN_NUM,
                                       SPI9_SSEL_FUNC_ALT_MODE);  // Set SPI9_SSEL pin to SPI9_SSEL functionality
                    IOCON_SetSpiPinMode(SPI9_SCK_IOCON_BASE, SPI9_SCK_GPIO_PIN_GROUP, SPI9_SCK_GPIO_PIN_NUM,
                                       SPI9_SCK_FUNC_ALT_MODE);  // Set SPI9_SCK pin to SPI9_SCK functionality
                    IOCON_SetSpiPinMode(SPI9_MOSI_IOCON_BASE, SPI9_MOSI_GPIO_PIN_GROUP, SPI9_MOSI_GPIO_PIN_NUM,
                                       SPI9_MOSI_FUNC_ALT_MODE);  // Set SPI9_MOSI pin to SPI9_MOSI functionality
                    break;
                default:
                    break;
            }
            break;
#endif // #if BL_ENABLE_PINMUX_SPI9

        default:
            break;
    }
}

//! @brief this is going to be used for autobaud IRQ handling for UART0
#if BL_ENABLE_PINMUX_UART0
void UART0_RX_GPIO_IRQHandler(void)
{
    // Clear interrupt before callback
#if UART0_ENABLE_GINT
    UART0_RX_GINT_BASE->CTRL |= GINT_CTRL_INT_MASK;
#elif UART0_ENABLE_PINT
    UART0_RX_PINT_BASE->FALL |= PINT_FALL_FDET_MASK;
#endif

    // Check if the pin for UART0 is what triggered the RX PORT interrupt
    if (s_pin_irq_func[0])
    {
        s_pin_irq_func[0](0);
    }
}
#endif // #if BL_ENABLE_PINMUX_UART0

void enable_autobaud_pin_irq(uint32_t instance, pin_irq_callback_t func)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_UART0
        case 0:
            NVIC_SetPriority(UART0_RX_GPIO_IRQn, GPIO_IRQC_INTERRUPT_ENABLED_PRIORITY);
            NVIC_EnableIRQ(UART0_RX_GPIO_IRQn);
#if UART0_ENABLE_GINT
            // Only look for a falling edge for our interrupts
            // Initialize GINT
            GINT_Init(UART0_RX_GINT_BASE);
            // Setup GINT for edge trigger, "OR" mode
            GINT_SetCtrl(UART0_RX_GINT_BASE, kGINT_CombineOr, kGINT_TrigEdge, NULL);
            // Select pins & polarity for GINT
            GINT_ConfigPins(UART0_RX_GINT_BASE, UART0_RX_GINT_GROUP, ~(1U << UART0_RX_GPIO_PIN_NUM), (1U << UART0_RX_GPIO_PIN_NUM));
#elif UART0_ENABLE_PINT
            // Connect trigger sources to PINT
            INPUTMUX_Init(INPUTMUX);
            INPUTMUX_AttachSignal(INPUTMUX, UART0_RX_PINT_INT_TYPE, UART0_RX_PINT_INT_SRC);
            // Turnoff clock to inputmux to save power. Clock is only needed to make changes
            INPUTMUX_Deinit(INPUTMUX);
            // Initialize PINT
            PINT_Init(UART0_RX_PINT_BASE);
            // Setup Pin Interrupt x for falling edge
            PINT_PinInterruptConfig(UART0_RX_PINT_BASE, UART0_RX_PINT_INT_TYPE, kPINT_PinIntEnableFallEdge, NULL);
#endif
            s_pin_irq_func[0] = func;
            break;
#endif // #if BL_ENABLE_PINMUX_UART0

        default:
            break;
    }
}

void disable_autobaud_pin_irq(uint32_t instance)
{
    switch (instance)
    {
#if BL_ENABLE_PINMUX_UART0
        case 0:
            NVIC_DisableIRQ(UART0_RX_GPIO_IRQn);
            NVIC_SetPriority(UART0_RX_GPIO_IRQn, GPIO_IRQC_INTERRUPT_RESTORED_PRIORITY);
#if UART0_ENABLE_GINT
            // De-initialize GINT
            GINT_Deinit(UART0_RX_GINT_BASE);
#elif UART0_ENABLE_PINT
            // De-initialize PINT
            PINT_Deinit(UART0_RX_PINT_BASE);
#endif
            s_pin_irq_func[0] = 0;
            break;
#endif // #if BL_ENABLE_PINMUX_UART0

        default:
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
