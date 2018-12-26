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
 * @file port_interrupts.h
 * This is the header file for the Port Interrupts module
 */

#ifndef _PORT_INTERRUPTS_H_
#define _PORT_INTERRUPTS_H_

/*!
 * @addtogroup port_interrupts Port Interrupts
 *
 * The port_interrupts module provides a set of turn-key interrupt handlers and enablement functions.
 *
 * The following interrupts are captured by the port_interrupts module:
 *      - PORTA: 
 *          + User switch SW1       (hw label: USER_SW1)
 *          + Touch interrupt       (hw label: TOUCH_INT)
 *          + Air quality interrupt (hw label: AIR_INTN)
 *          + NTAG field detect     (hw label: NTAG_FD)
 *          + Charging state        (hw label: CHG_STATE)
 *      - PORTB:
 *          + External module 1 interrupt (hw label: MB1_INT)
 *          + External module 2 interrupt (hw label: MB2_INT)
 *          + External module 3 interrupt (hw label: MB3_INT)
 *      - PORTC:
 *          + Ambient light interrupt      (hw label: AMB_INT)
 *          + Accelerometer interrupt INT1 (hw label: ACCEL_INT1)
 *          + Gyroscope interrupt INT2     (hw label: GYRO_INT2)
 *      - PORTD:
 *          + Real Time Clock interrupt     (hw label: RTC_INT)
 *          + Gyroscope interrupt INT1      (hw label: GYRO_INT1)
 *          + Pressure interrupt INT2       (hw label: PRESSURE_INT2)
 *          + Pressure interrupt INT1       (hw label: PRESSURE_INT1)
 *          + Accelerometer interrupt INT2  (hw label: ACCEL_INT2)
 *      - PORTE:
 *          + User switch SW2 (hw label: USER_SW2)
 *          + User switch SW3 (hw label: USER_SW3)
 *          + User switch SW4 (hw label: USER_SW4)
 *
 * Usage
 * -------------------------------------------------------------------------------------------------
 * 
 * Initialization:
 * @code
 *
 *      // Enable all Rapid IoT PORT Interrupts
 *      PORT_IRQ_EnablePortAIrq();
 *      PORT_IRQ_EnablePortBIrq();
 *      PORT_IRQ_EnablePortCIrq();
 *      PORT_IRQ_EnablePortDIrq();
 *      PORT_IRQ_EnablePortEIrq();
 *
 * @endcode
 *
 * Adding Callbacks:
 * @code
 * 
 *      // Create a non-blocking callback function
 *      // NOTE: callback functions's signature can vary
 *      void myAmbLightCallback(void)
 *      {
 *          // Ambient Light interrupt Callback code
 *          ...
 *      }
 *
 *      // In port_interrupts.c edit associated interrupt handler
 *      void PORTC_IRQHandler(void)
 *      {
 *          uint32_t pin_nb;
 *          pin_nb = PORT_GetPinsInterruptFlags(PORTC);
 *
 *          if (pin_nb & (1 << BOARD_INITPINS_AMB_INT_GPIO_PIN))
 *          {
 *              //Add callback for Ambient Light Interrupt
 *               myAmbLightCallback();
 *               GPIO_ClearPinsInterruptFlags(BOARD_INITPINS_AMB_INT_GPIO, 1U << BOARD_INITPINS_AMB_INT_GPIO_PIN);
 *          }
 *          ...
 *      } 
 *
 * @endcode
 *
 * @{
 * @brief Port Interrupt enablement and handling functions
 */

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Enable IRQs on PORTA for sensors with interrupt lines and push button switch SW1
 *
 * @return None
 *
 */
void PORT_IRQ_EnablePortAIrq(void);

/*!
 * @brief Enable IRQs on PORTB for external modules interrupt lines
 *
 * @return None
 *
 */
void PORT_IRQ_EnablePortBIrq(void);

/*!
 * @brief Enable IRQs on PORTC for sensors interrupt lines
 *
 * @return None
 *
 */
void PORT_IRQ_EnablePortCIrq(void);

/*!
 * @brief Enable IRQs on PORTD for sensors interrupt lines
 *
 * @return None
 *
 */
void PORT_IRQ_EnablePortDIrq(void);

/*!
 * @brief Enable IRQs on PORTE for push button switches SW2, SW3, SW4
 *
 * @return None
 *
 */
void PORT_IRQ_EnablePortEIrq(void);

/*!
 * @brief PORTA_IRQHandler IRQ handler for all PORT A interrupts:
 *        - User switch SW1       (hw label: USER_SW1)
 *        - Touch interrupt       (hw label: TOUCH_INT)
 *        - Air quality interrupt (hw label: AIR_INTN)
 *        - NTAG field detect     (hw label: NTAG_FD)
 *        - Charging state        (hw label: CHG_STATE)
 *
 * @return None
 *
 */
void PORTA_IRQHandler(void);

/*!
 * @brief PORTB_IRQHandler IRQ handler for all PORT B interrupts:
 *        - External module 1 interrupt (hw label: MB1_INT)
 *        - External module 2 interrupt (hw label: MB2_INT)
 *        - External module 3 interrupt (hw label: MB3_INT)
 *
 * @return None
 *
 */
void PORTB_IRQHandler(void);

/*!
 * @brief PORTC_IRQHandler IRQ handler for all PORT C interrupts:
 *        - Ambient light interrupt      (hw label: AMB_INT)
 *        - Accelerometer interrupt INT1 (hw label: ACCEL_INT1)
 *        - Gyroscope interrupt INT2     (hw label: GYRO_INT2)
 *
 * @return None
 *
 */
void PORTC_IRQHandler(void);

/*!
 * @brief PORTD_IRQHandler IRQ handler for all PORT D interrupts:
 *        - Real Time Clock interrupt     (hw label: RTC_INT)
 *        - Gyroscope interrupt INT1      (hw label: GYRO_INT1)
 *        - Pressure interrupt INT2       (hw label: PRESSURE_INT2)
 *        - Pressure interrupt INT1       (hw label: PRESSURE_INT1)
 *        - Accelerometer interrupt INT2  (hw label: ACCEL_INT2)
 *
 * @return None
 *
 */
void PORTD_IRQHandler(void);

/*!
 * @brief PORTE_IRQHandler IRQ handler for all PORT E interrupts:
 *        - User switch SW2 (hw label: USER_SW2)
 *        - User switch SW3 (hw label: USER_SW3)
 *        - User switch SW4 (hw label: USER_SW4)
 *
 * @return None
 *
 */
void PORTE_IRQHandler(void);

/*!
 * @brief Enable IRQ for All UI related interrupt lines
 *
 * @return None
 *
 */
void PORT_IRQ_UiEnableIrq(void);

/*!
 * @brief Disable IRQ for All UI related interrupt lines
 *
 * @return None
 *
 */
void PORT_IRQ_UiDisableIrq(void);

/*! @}*/

#if defined(__cplusplus)
}
#endif

#endif /* _PORT_INTERRUPTS_H_ */
