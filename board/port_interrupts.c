/*
 * Copyright (c) 2018 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
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

/*!
 * @file port_interrupts.c
 * This is the source file for the Port Interrupts module
 */

#include "fsl_port.h"
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
#include "fsl_os_abstraction.h"
#include "FreeRTOS.h"
#endif

#include "board.h"
#include "port_interrupts.h"

#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
/* UI Management */
#include "ui_manager.h"
#endif

#include "pin_mux.h"
#include "pin_mux_rpk.h"
#include "spi_bus_share.h"

/*!
 * @brief PORTA_IRQHandler IRQ handler for all PORT A interrupts, place necessary callbacks where needed
 *        User switch SW1       (hw label: USER_SW1)
 *        Touch interrupt       (hw label: TOUCH_INT)
 *        Air quality interrupt (hw label: AIR_INTN)
 *        NTAG field detect     (hw label: NTAG_FD)
 *        Charging state        (hw label: CHG_STATE)
 *
 * @note Callbacks should be non-blocking
 */
void PORTA_IRQHandler(void)
{
    uint32_t pin_nb;
    pin_nb = PORT_GetPinsInterruptFlags(PORTA);

    if (pin_nb & (1 << BOARD_INITPINS_USER_SW1_GPIO_PIN))
    {
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
        UiManager_SetEvent(kUiSwOne);
#endif
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW1_GPIO, 1U << BOARD_INITPINS_USER_SW1_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_TOUCH_INT_GPIO_PIN))
    {
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    	UiManager_SetEvent(kUiTouchEv);
#endif
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_TOUCH_INT_GPIO, 1U << BOARD_INITPINS_TOUCH_INT_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_AIR_INTN_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_AIR_INTN_GPIO, 1U << BOARD_INITPINS_AIR_INTN_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_NTAG_FD_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_NTAG_FD_GPIO, 1U << BOARD_INITPINS_NTAG_FD_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_CHG_STATE_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_CHG_STATE_GPIO, 1U << BOARD_INITPINS_CHG_STATE_GPIO_PIN);
    }

}

/*!
 * @brief PORTB_IRQHandler IRQ handler for all PORT B interrupts, place necessary callbacks where needed
 *        External module 1 interrupt (hw label: MB1_INT)
 *        External module 2 interrupt (hw label: MB2_INT)
 *        External module 3 interrupt (hw label: MB3_INT)
 *
 * @note Callbacks should be non-blocking
 */
void PORTB_IRQHandler(void)
{
    uint32_t pin_nb;
    pin_nb = PORT_GetPinsInterruptFlags(PORTB);

    if (pin_nb & (1 << BOARD_INITPINS_MB1_INT_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_MB1_INT_GPIO, 1U << BOARD_INITPINS_MB1_INT_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_MB2_INT_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_MB2_INT_GPIO, 1U << BOARD_INITPINS_MB2_INT_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_MB3_INT_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_MB3_INT_GPIO, 1U << BOARD_INITPINS_MB3_INT_GPIO_PIN);
    }
}

/*!
 * @brief PORTC_IRQHandler IRQ handler for all PORT C interrupts, place necessary callbacks where needed
 *        Ambient light interrupt      (hw label: AMB_INT)
 *        Accelerometer interrupt INT1 (hw label: ACCEL_INT1)
 *        Gyroscope interrupt INT2     (hw label: GYRO_INT2)
 *
 * @note Callbacks should be non-blocking
 */
void PORTC_IRQHandler(void)
{
    uint32_t pin_nb;
    pin_nb = PORT_GetPinsInterruptFlags(PORTC);

    if (pin_nb & (1 << BOARD_INITPINS_AMB_INT_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_AMB_INT_GPIO, 1U << BOARD_INITPINS_AMB_INT_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_ACCEL_INT1_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_ACCEL_INT1_GPIO, 1U << BOARD_INITPINS_ACCEL_INT1_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_GYRO_INT2_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_GYRO_INT2_GPIO, 1U << BOARD_INITPINS_GYRO_INT2_GPIO_PIN);
    }
}

/*!
 * @brief PORTD_IRQHandler IRQ handler for all PORT D interrupts, place necessary callbacks where needed
 *        Real Time Clock interrupt     (hw label: RTC_INT)
 *        Gyroscope interrupt INT1      (hw label: GYRO_INT1)
 *        Pressure interrupt INT2       (hw label: PRESSURE_INT2)
 *        Pressure interrupt INT1       (hw label: PRESSURE_INT1)
 *        Accelerometer interrupt INT2  (hw label: ACCEL_INT2)
 *
 * @note Callbacks should be non-blocking
 */
void PORTD_IRQHandler(void)
{
    uint32_t pin_nb;
    pin_nb = PORT_GetPinsInterruptFlags(PORTD);

    if (pin_nb & (1 << BOARD_INITPINS_RTC_INT_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_RTC_INT_GPIO, 1U << BOARD_INITPINS_RTC_INT_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_GYRO_INT1_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_GYRO_INT1_GPIO, 1U << BOARD_INITPINS_GYRO_INT1_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_PRESSURE_INT2_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_PRESSURE_INT2_GPIO, 1U << BOARD_INITPINS_PRESSURE_INT2_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_PRESSURE_INT1_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_PRESSURE_INT1_GPIO, 1U << BOARD_INITPINS_PRESSURE_INT1_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_ACCEL_INT2_GPIO_PIN))
    {
        /* TODO: Trigger event */
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_ACCEL_INT2_GPIO, 1U << BOARD_INITPINS_ACCEL_INT2_GPIO_PIN);
    }
}

/*!
 * @brief PORTE_IRQHandler IRQ handler for all PORT E interrupts, place necessary callbacks where needed
 *        User switch SW2 (hw label: USER_SW2)
 *        User switch SW3 (hw label: USER_SW3)
 *        User switch SW4 (hw label: USER_SW4)
 *
 * @note Callbacks should be non-blocking
 */
void PORTE_IRQHandler(void)
{
    uint32_t pin_nb;
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    uint32_t uiEvents = 0U;
#endif
    pin_nb = PORT_GetPinsInterruptFlags(PORTE);

    if (pin_nb & (1 << BOARD_INITPINS_USER_SW2_GPIO_PIN))
    {
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
        uiEvents |= kUiSwTwo;
#endif
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW2_GPIO, 1U << BOARD_INITPINS_USER_SW2_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_USER_SW3_GPIO_PIN))
    {
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
        uiEvents |= kUiSwThree;
#endif
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW3_GPIO, 1U << BOARD_INITPINS_USER_SW3_GPIO_PIN);
    }

    if (pin_nb & (1 << BOARD_INITPINS_USER_SW4_GPIO_PIN))
    {
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
        uiEvents |= kUiSwFour;
#endif
        GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW4_GPIO, 1U << BOARD_INITPINS_USER_SW4_GPIO_PIN);
    }

#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    if (0 != uiEvents)
    {
        UiManager_SetEvent(uiEvents);
    }
#endif

    if (pin_nb & (1 << BOARD_INITPINS_KW41_UART_RTS_PIN))
    {
        BOARD_K41Z_RTS_IRQ_HANDLER(pin_nb);
    }
}

void PORT_IRQ_EnablePortAIrq(void)
{
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Set interrupt priority below RTOS system interrupts */
    NVIC_SetPriority(PORTA_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY-1);
#else
    NVIC_SetPriority(PORTA_IRQn, 7);
#endif

#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Register the IRQ handler with the RTOS */
    OSA_InstallIntHandler(PORTA_IRQn, PORTA_IRQHandler);
#endif

    /* Set interrupt configuration for inputs */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW1_PORT, BOARD_INITPINS_USER_SW1_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_TOUCH_INT_PORT, BOARD_INITPINS_TOUCH_INT_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_AIR_INTN_PORT, BOARD_INITPINS_AIR_INTN_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_NTAG_FD_PORT, BOARD_INITPINS_NTAG_FD_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_CHG_STATE_PORT, BOARD_INITPINS_CHG_STATE_GPIO_PIN, kPORT_InterruptEitherEdge);

    /* Enable interrupts on this port */
    EnableIRQ(PORTA_IRQn);

    /* Clear pending interrupts */
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW1_GPIO, 1U << BOARD_INITPINS_USER_SW1_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_TOUCH_INT_GPIO, 1U << BOARD_INITPINS_TOUCH_INT_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_AIR_INTN_GPIO, 1U << BOARD_INITPINS_AIR_INTN_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_NTAG_FD_GPIO, 1U << BOARD_INITPINS_NTAG_FD_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_CHG_STATE_GPIO, 1U << BOARD_INITPINS_CHG_STATE_GPIO_PIN);
}

void PORT_IRQ_EnablePortBIrq(void)
{
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Set interrupt priority below RTOS system interrupts */
    NVIC_SetPriority(PORTB_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY-1);
#else
    NVIC_SetPriority(PORTB_IRQn, 7);
#endif

#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Register the IRQ handler with the RTOS */
    OSA_InstallIntHandler(PORTB_IRQn, PORTB_IRQHandler);
#endif

    /* Set interrupt configuration for MBn_INT inputs */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_MB1_INT_PORT, BOARD_INITPINS_MB1_INT_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_MB2_INT_PORT, BOARD_INITPINS_MB2_INT_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_MB3_INT_PORT, BOARD_INITPINS_MB3_INT_GPIO_PIN, kPORT_InterruptFallingEdge);

    /* Enable interrupts on this port */
    EnableIRQ(PORTB_IRQn);

    /* Clear pending interrupts */
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_MB1_INT_GPIO, 1U << BOARD_INITPINS_MB1_INT_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_MB2_INT_GPIO, 1U << BOARD_INITPINS_MB2_INT_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_MB3_INT_GPIO, 1U << BOARD_INITPINS_MB3_INT_GPIO_PIN);
}

void PORT_IRQ_EnablePortCIrq(void)
{
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Set interrupt priority below RTOS system interrupts */
    NVIC_SetPriority(PORTC_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY-1);
#else
    NVIC_SetPriority(PORTC_IRQn, 7);
#endif

#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Register the IRQ handler with the RTOS */
    OSA_InstallIntHandler(PORTC_IRQn, PORTC_IRQHandler);
#endif

    /* Set interrupt configuration for sensor inputs */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_AMB_INT_PORT, BOARD_INITPINS_AMB_INT_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_ACCEL_INT1_PORT, BOARD_INITPINS_ACCEL_INT1_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_GYRO_INT2_PORT, BOARD_INITPINS_GYRO_INT2_GPIO_PIN, kPORT_InterruptFallingEdge);

    /* Enable interrupts on this port */
    EnableIRQ(PORTC_IRQn);

    /* Clear pending interrupts */
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_AMB_INT_GPIO, 1U << BOARD_INITPINS_AMB_INT_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_ACCEL_INT1_GPIO, 1U << BOARD_INITPINS_ACCEL_INT1_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_GYRO_INT2_GPIO, 1U << BOARD_INITPINS_GYRO_INT2_GPIO_PIN);
}

void PORT_IRQ_EnablePortDIrq(void)
{
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Set interrupt priority below RTOS system interrupts */
    NVIC_SetPriority(PORTD_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY-1);
#else
    NVIC_SetPriority(PORTD_IRQn, 7);
#endif

#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Register the IRQ handler with the RTOS */
    OSA_InstallIntHandler(PORTD_IRQn, PORTD_IRQHandler);
#endif

    /* Set interrupt configuration for sensor inputs */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_RTC_INT_PORT, BOARD_INITPINS_RTC_INT_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_GYRO_INT1_PORT, BOARD_INITPINS_GYRO_INT1_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_PRESSURE_INT2_PORT, BOARD_INITPINS_PRESSURE_INT2_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_PRESSURE_INT1_PORT, BOARD_INITPINS_PRESSURE_INT1_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_ACCEL_INT2_PORT, BOARD_INITPINS_ACCEL_INT2_GPIO_PIN, kPORT_InterruptFallingEdge);

    /* Enable interrupts on this port */
    EnableIRQ(PORTD_IRQn);

    /* Clear pending interrupts */
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_RTC_INT_GPIO, 1U << BOARD_INITPINS_RTC_INT_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_GYRO_INT1_GPIO, 1U << BOARD_INITPINS_GYRO_INT1_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_PRESSURE_INT2_GPIO, 1U << BOARD_INITPINS_PRESSURE_INT2_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_PRESSURE_INT1_GPIO, 1U << BOARD_INITPINS_PRESSURE_INT1_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_ACCEL_INT2_GPIO, 1U << BOARD_INITPINS_ACCEL_INT2_GPIO_PIN);
}

void PORT_IRQ_EnablePortEIrq(void)
{
#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Set interrupt priority below RTOS system interrupts */
    NVIC_SetPriority(PORTE_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY-1);
#else
    NVIC_SetPriority(PORTE_IRQn, 7);
#endif

#if !defined(BOOTLOADER) && defined(FSL_RTOS_FREE_RTOS)
    /* Register the IRQ handler with the RTOS */
    OSA_InstallIntHandler(PORTE_IRQn, PORTE_IRQHandler);
#endif

    /* Set interrupt configuration for SW2/SW2/SW3 switch inputs */
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW2_PORT, BOARD_INITPINS_USER_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW3_PORT, BOARD_INITPINS_USER_SW3_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW4_PORT, BOARD_INITPINS_USER_SW4_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_KW41_UART_RTS_PORT, BOARD_INITPINS_KW41_UART_RTS_PIN, kPORT_InterruptRisingEdge);


    /* Enable interrupts on this port */
    EnableIRQ(PORTE_IRQn);

    /* Clear pending interrupts */
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW2_GPIO, 1U << BOARD_INITPINS_USER_SW2_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW3_GPIO, 1U << BOARD_INITPINS_USER_SW3_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_USER_SW4_GPIO, 1U << BOARD_INITPINS_USER_SW4_GPIO_PIN);
    GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_KW41_UART_RTS_GPIO, 1U << BOARD_INITPINS_KW41_UART_RTS_PIN);

}

void PORT_IRQ_UiEnableIrq(void)
{
    PORT_SetPinInterruptConfig(BOARD_INITPINS_TOUCH_INT_PORT, BOARD_INITPINS_TOUCH_INT_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW1_PORT, BOARD_INITPINS_USER_SW1_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW2_PORT, BOARD_INITPINS_USER_SW2_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW3_PORT, BOARD_INITPINS_USER_SW3_GPIO_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW4_PORT, BOARD_INITPINS_USER_SW4_GPIO_PIN, kPORT_InterruptFallingEdge);
}

void PORT_IRQ_UiDisableIrq(void)
{
    PORT_SetPinInterruptConfig(BOARD_INITPINS_TOUCH_INT_PORT, BOARD_INITPINS_TOUCH_INT_GPIO_PIN, kPORT_InterruptOrDMADisabled);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW1_PORT, BOARD_INITPINS_USER_SW1_GPIO_PIN, kPORT_InterruptOrDMADisabled);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW2_PORT, BOARD_INITPINS_USER_SW2_GPIO_PIN, kPORT_InterruptOrDMADisabled);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW3_PORT, BOARD_INITPINS_USER_SW3_GPIO_PIN, kPORT_InterruptOrDMADisabled);
    PORT_SetPinInterruptConfig(BOARD_INITPINS_USER_SW4_PORT, BOARD_INITPINS_USER_SW4_GPIO_PIN, kPORT_InterruptOrDMADisabled);
}
