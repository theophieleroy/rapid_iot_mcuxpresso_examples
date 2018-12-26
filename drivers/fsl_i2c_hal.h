/*
 * The Clear BSD License
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
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
#if !defined(__FSL_I2C_HAL_H__)
#define __FSL_I2C_HAL_H__

#include "utilities/fsl_assert.h"
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup i2c_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief I2C status return codes.*/
typedef enum _i2c_status
{
    kStatus_I2C_Success = 0x0U,
    kStatus_I2C_OutOfRange = 0x1U,
    kStatus_I2C_Fail = 0x2U,
    kStatus_I2C_Busy = 0x3U,            /*!< The master is already performing a transfer.*/
    kStatus_I2C_Timeout = 0x4U,         /*!< The transfer timed out.*/
    kStatus_I2C_ReceivedNak = 0x5U,     /*!< The slave device sent a NAK in response to a byte.*/
    kStatus_I2C_SlaveTxUnderrun = 0x6U, /*!< I2C Slave TX Underrun error.*/
    kStatus_I2C_SlaveRxOverrun = 0x7U,  /*!< I2C Slave RX Overrun error.*/
    kStatus_I2C_AribtrationLost = 0x8U, /*!< I2C Arbitration Lost error.*/
    kStatus_I2C_BusBusy = 0x9U,         /*!< Bus is stuck in busy after a send */
} i2c_status_t;

/*! @brief I2C status flags. */
typedef enum _i2c_status_flag
{
    kI2CTransferComplete = I2C_S_TCF_SHIFT,
    kI2CAddressAsSlave = I2C_S_IAAS_SHIFT,
    kI2CBusBusy = I2C_S_BUSY_SHIFT,
    kI2CArbitrationLost = I2C_S_ARBL_SHIFT,
    kI2CAddressMatch = I2C_S_RAM_SHIFT,
    kI2CSlaveTransmit = I2C_S_SRW_SHIFT,
    kI2CInterruptPending = I2C_S_IICIF_SHIFT,
    kI2CReceivedNak = I2C_S_RXAK_SHIFT
} i2c_status_flag_t;

/*! @brief Direction of master and slave transfers.*/
typedef enum _i2c_direction
{
    kI2CReceive = 0U, /*!< Master and slave receive.*/
    kI2CSend = 1U     /*!< Master and slave transmit.*/
} i2c_direction_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern const uint32_t g_i2cBaseAddr[];

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Module controls
 * @{
 */

/*!
 * @brief Restores the I2C peripheral to reset state.
 *
 * @param baseAddr The I2C peripheral base address
 */
void I2C_HAL_Init(I2C_Type *baseAddr);

/*!
 * @brief Enables the I2C module operation.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_Enable(I2C_Type *baseAddr)
{
    I2C_WR_C1_IICEN(baseAddr, 0x1U);
}

/*!
 * @brief Disables the I2C module operation.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_Disable(I2C_Type *baseAddr)
{
    I2C_WR_C1_IICEN(baseAddr, 0x0U);
}

/*@}*/

/*!
 * @name DMA
 * @{
 */

/*!
 * @brief Enables or disables the DMA support.
 *
 * @param baseAddr The I2C peripheral base address
 * @param enable Pass true to enable DMA transfer signalling
 */
static inline void I2C_HAL_SetDmaCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_C1_DMAEN(baseAddr, (uint8_t)enable);
}

/*!
 * @brief Returns whether I2C DMA support is enabled.
 *
 * @param baseAddr The I2C peripheral base address.
 * @retval true I2C DMA is enabled.
 * @retval false I2C DMA is disabled.
 */
static inline bool I2C_HAL_GetDmaCmd(I2C_Type *baseAddr)
{
    return I2C_RD_C1_DMAEN(baseAddr);
}

/*@}*/

/*!
 * @name Pin functions
 * @{
 */

/*!
 * @brief Controls the drive capability of the I2C pads.
 *
 * @param baseAddr The I2C peripheral base address
 * @param enable Passing true will enable high drive mode of the I2C pads. False sets normal
 *     drive mode.
 */
static inline void I2C_HAL_SetHighDriveCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_C2_HDRS(baseAddr, (uint8_t)enable);
}

/*!
 * @brief Controls the width of the programmable glitch filter.
 *
 * Controls the width of the glitch, in terms of bus clock cycles, that the filter must absorb.
 * The filter does not allow any glitch whose size is less than or equal to this width setting,
 * to pass.
 *
 * @param baseAddr The I2C peripheral base address
 * @param glitchWidth Maximum width in bus clock cycles of the glitches that is filtered.
 *     Pass zero to disable the glitch filter.
 */
static inline void I2C_HAL_SetGlitchWidth(I2C_Type *baseAddr, uint8_t glitchWidth)
{
    I2C_WR_FLT_FLT(baseAddr, glitchWidth);
}

/*@}*/

/*!
 * @name Low power
 * @{
 */

/*!
 * @brief Controls the I2C wakeup enable.
 *
 * The I2C module can wake the MCU from low power mode with no peripheral bus running when
 * slave address matching occurs.
 *
 * @param baseAddr The I2C peripheral base address.
 * @param enable true - Enables the wakeup function in low power mode.<br>
 *     false - Normal operation. No interrupt is  generated when address matching in
 *     low power mode.
 */
static inline void I2C_HAL_SetWakeupCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_C1_WUEN(baseAddr, (uint8_t)enable);
}

#if FSL_FEATURE_I2C_HAS_STOP_HOLD_OFF
/*!
 * @brief Controls the stop mode hold off.
 *
 * This function lets you enable the hold off entry to low power stop mode when any data transmission
 * or reception is occurring.
 *
 * @param baseAddr The I2C peripheral base address
 * @param enable false - Stop hold off is disabled. The MCU's entry to stop mode is not gated.<br>
 *     true - Stop hold off is enabled.
 */

static inline void I2C_HAL_SetStopHoldoffCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_FLT_SHEN(baseAddr, (uint8_t)enable);
}
#endif /* FSL_FEATURE_I2C_HAS_STOP_HOLD_OFF*/

/*@}*/

/*!
 * @name Baud rate
 * @{
 */

/*!
 * @brief Sets the I2C bus frequency for master transactions.
 *
 * @param baseAddr The I2C peripheral base address
 * @param sourceClockInHz I2C source input clock in Hertz
 * @param kbps Requested bus frequency in kilohertz. Common values are either 100 or 400.
 * @param absoluteError_Hz If this parameter is not NULL, it is filled in with the
 *     difference in Hertz between the requested bus frequency and the closest frequency
 *     possible given available divider values.
 *
 * @retval kStatus_Success The baud rate was changed successfully. However, there is no
 *      guarantee on the minimum error. If you want to ensure that the baud was set to within
 *      a certain error, then use the @a absoluteError_Hz parameter.
 * @retval kStatus_OutOfRange The requested baud rate was not within the range of rates
 *      supported by the peripheral.
 */
i2c_status_t I2C_HAL_SetBaudRate(I2C_Type *baseAddr,
                                 uint32_t sourceClockInHz,
                                 uint32_t kbps,
                                 uint32_t *absoluteError_Hz);

/*!
 * @brief Sets the I2C baud rate multiplier and table entry.
 *
 * Use this function to set the I2C bus frequency register values directly, if they are
 * known in advance.
 *
 * @param baseAddr The I2C peripheral base address
 * @param mult Value of the MULT bitfield, ranging from 0-2.
 * @param icr The ICR bitfield value, which is the index into an internal table in the I2C
 *     hardware that selects the baud rate divisor and SCL hold time.
 */
static inline void I2C_HAL_SetFreqDiv(I2C_Type *baseAddr, uint8_t mult, uint8_t icr)
{
    I2C_WR_F(baseAddr, I2C_F_MULT(mult) | I2C_F_ICR(icr));
}

/*!
 * @brief Slave baud rate control
 *
 * Enables an independent slave mode baud rate at the maximum frequency. This forces clock stretching
 * on the SCL in very fast I2C modes.
 *
 * @param baseAddr The I2C peripheral base address
 * @param enable true - Slave baud rate is independent of the master baud rate;<br>
 *     false - The slave baud rate follows the master baud rate and clock stretching may occur.
 */
static inline void I2C_HAL_SetSlaveBaudCtrlCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_C2_SBRC(baseAddr, (uint8_t)enable);
}

/*@}*/

/*!
 * @name Bus operations
 * @{
 */

/*!
 * @brief Sends a START or a Repeated START signal on the I2C bus.
 *
 * This function is used to initiate a new master mode transfer by sending the START signal. It
 * is also used to send a Repeated START signal when a transfer is already in progress.
 *
 * @param baseAddr The I2C peripheral base address
 */
void I2C_HAL_SendStart(I2C_Type *baseAddr);

/*!
 * @brief Sends a STOP signal on the I2C bus.
 *
 * This function will change the direction to receive.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_SendStop(I2C_Type *baseAddr)
{
    assert(I2C_RD_C1_MST(baseAddr) == 1);
    I2C_CLR_C1(baseAddr, I2C_C1_MST_MASK | I2C_C1_TX_MASK);
}

/*!
 * @brief Causes an ACK to be sent on the bus.
 *
 * This function specifies that an ACK signal is sent in response to the next received byte.
 *
 * Note that the behavior of this function is changed when the I2C peripheral is placed in
 * Fast ACK mode. In this case, this function causes an ACK signal to be sent in
 * response to the current byte, rather than the next received byte.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_SendAck(I2C_Type *baseAddr)
{
    I2C_WR_C1_TXAK(baseAddr, 0x0U);
}

/*!
 * @brief Causes a NAK to be sent on the bus.
 *
 * This function specifies that a NAK signal is sent in response to the next received byte.
 *
 * Note that the behavior of this function is changed when the I2C peripheral is placed in the
 * Fast ACK mode. In this case, this function causes an NAK signal to be sent in
 * response to the current byte, rather than the next received byte.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_SendNak(I2C_Type *baseAddr)
{
    I2C_WR_C1_TXAK(baseAddr, 0x1U);
}

/*!
 * @brief Selects either transmit or receive mode.
 *
 * @param baseAddr The I2C peripheral base address.
 * @param mode Specifies either transmit mode or receive mode. The valid values are:
 *     - #kI2CTransmit
 *     - #kI2CReceive
 */
static inline void I2C_HAL_SetDirMode(I2C_Type *baseAddr, i2c_direction_t direction)
{
    I2C_WR_C1_TX(baseAddr, (uint8_t)direction);
}

/*!
 * @brief Returns the currently selected transmit or receive mode.
 *
 * @param baseAddr The I2C peripheral base address.
 * @retval #kI2CTransmit I2C is configured for master or slave transmit mode.
 * @retval #kI2CReceive I2C is configured for master or slave receive mode.
 */
static inline i2c_direction_t I2C_HAL_GetDirMode(I2C_Type *baseAddr)
{
    return (i2c_direction_t)I2C_RD_C1_TX(baseAddr);
}

/*@}*/

/*!
 * @name Data transfer
 * @{
 */

/*!
 * @brief Returns the last byte of data read from the bus and initiate another read.
 *
 * In a master receive mode, calling this function initiates receiving  the next byte of data.
 *
 * @param baseAddr The I2C peripheral base address
 * @return This function returns the last byte received while the I2C module is configured in master
 *     receive or slave receive mode.
 */
static inline uint8_t I2C_HAL_ReadByte(I2C_Type *baseAddr)
{
    return I2C_RD_D(baseAddr);
}

/*!
 * @brief Writes one byte of data to the I2C bus.
 *
 * When this function is called in the master transmit mode, a data transfer is initiated. In slave
 * mode, the same function is available after an address match occurs.
 *
 * In a master transmit mode, the first byte of data written following the start bit or repeated
 * start bit is used for the address transfer and must consist of the slave address (in bits 7-1)
 * concatenated with the required R/\#W bit (in position bit 0).
 *
 * @param baseAddr The I2C peripheral base address.
 * @param byte The byte of data to transmit.
 */
static inline void I2C_HAL_WriteByte(I2C_Type *baseAddr, uint8_t byte)
{
    I2C_WR_D(baseAddr, byte);
}

/*@}*/

/*!
 * @name Slave address
 * @{
 */

/*!
 * @brief Sets the primary 7-bit slave address.
 *
 * @param baseAddr The I2C peripheral base address
 * @param address The slave address in the upper 7 bits. Bit 0 of this value must be 0.
 */
void I2C_HAL_SetAddress7bit(I2C_Type *baseAddr, uint8_t address);

/*!
 * @brief Sets the primary slave address and enables 10-bit address mode.
 *
 * @param baseAddr The I2C peripheral base address
 * @param address The 10-bit slave address, in bits [10:1] of the value. Bit 0 must be 0.
 */
void I2C_HAL_SetAddress10bit(I2C_Type *baseAddr, uint16_t address);

/*!
 * @brief Enable or disable the extension address (10-bit).
 *
 * @param baseAddr The I2C peripheral base address
 * @enable true: 10-bit address is enabled.
 *         false: 10-bit address is not enabled.
 */
static inline void I2C_HAL_SetExtensionAddrCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_C2_ADEXT(baseAddr, (uint8_t)enable);
}

/*!
 * @brief Return whether the extension address is enabled or not.
 *
 * @param baseAddr The I2C peripheral base address
 * @return true: 10-bit address is enabled.
 *         false: 10-bit address is not enabled.
 */
static inline bool I2C_HAL_GetExtensionAddrCmd(I2C_Type *baseAddr)
{
    return I2C_RD_C2_ADEXT(baseAddr);
}

/*!
 * @brief Controls whether the general call address is recognized.
 *
 * @param baseAddr The I2C peripheral base address
 * @param enable Whether to enable the general call address.
 */
static inline void I2C_HAL_SetGeneralCallCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_C2_GCAEN(baseAddr, (uint8_t)enable);
}

/*!
 * @brief Enables or disables the slave address range matching.
 *
 * @param baseAddr The I2C peripheral base address.
 * @param enable Pass true to enable range address matching. You must also call
 *     I2C_HAL_SetUpperAddress7bit() to set the upper address.
 */
static inline void I2C_HAL_SetRangeMatchCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_C2_RMEN(baseAddr, (uint8_t)enable);
}

/*!
 * @brief Sets the upper slave address.
 *
 * This slave address is used as a secondary slave address. If range address
 * matching is enabled, this slave address acts as the upper bound on the slave address
 * range.
 *
 * This function sets only a 7-bit slave address. If 10-bit addressing was enabled by calling
 * I2C_HAL_SetAddress10bit(), then the top 3 bits set with that function are also used
 * with the address set with this function to form a 10-bit address.
 *
 * Passing 0 for the @a address parameter  disables  matching the upper slave address.
 *
 * @param baseAddr The I2C peripheral base address
 * @param address The upper slave address in the upper 7 bits. Bit 0 of this value must be 0.
 *     In addition, this address must be greater than the primary slave address that is set by
 *     calling I2C_HAL_SetAddress7bit().
 */
static inline void I2C_HAL_SetUpperAddress7bit(I2C_Type *baseAddr, uint8_t address)
{
    assert((address & 1) == 0);
    assert((address == 0) || (address > I2C_RD_A1(baseAddr)));
    I2C_WR_RA(baseAddr, address);
}

/*@}*/

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get I2C status flag state
 *
 * @param baseAddr The I2C peripheral base address.
 * @param statusFlag The status flag, defined in type i2c_status_flag_t.
 * @return State of the status flag: asserted (true) or not-asserted (false).
 *         - true: related status flag is being set.
 *         - false: related status flag is not set.
 */
static inline bool I2C_HAL_GetStatusFlag(I2C_Type *baseAddr, i2c_status_flag_t statusFlag)
{
    return (bool)((I2C_RD_S(baseAddr) >> statusFlag) & 0x1U);
}

/*!
 * @brief Returns whether the I2C module is in master mode.
 *
 * @param baseAddr The I2C peripheral base address.
 * @retval true The module is in master mode, which implies it is also performing a transfer.
 * @retval false The module is in slave mode.
 */
static inline bool I2C_HAL_IsMaster(I2C_Type *baseAddr)
{
    return (bool)I2C_RD_C1_MST(baseAddr);
}

/*!
 * @brief Clears the arbitration lost flag.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_ClearArbitrationLost(I2C_Type *baseAddr)
{
    I2C_WR_S_ARBL(baseAddr, 0x1U);
}

/*@}*/

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Enables or Disables I2C interrupt requests.
 *
 * @param baseAddr The I2C peripheral base address
 * @param enable   Pass true to enable interrupt, flase to disable.
 */
static inline void I2C_HAL_SetIntCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_C1_IICIE(baseAddr, (uint8_t)enable);
}

/*!
 * @brief Returns whether the I2C interrupts are enabled.
 *
 * @param baseAddr The I2C peripheral base address
 * @retval true I2C interrupts are enabled.
 * @retval false I2C interrupts are disabled.
 */
static inline bool I2C_HAL_GetIntCmd(I2C_Type *baseAddr)
{
    return (bool)I2C_RD_C1_IICIE(baseAddr);
}

/*!
 * @brief Returns the current I2C interrupt flag.
 *
 * @param baseAddr The I2C peripheral base address
 * @retval true An interrupt is pending.
 * @retval false No interrupt is pending.
 */
static inline bool I2C_HAL_IsIntPending(I2C_Type *baseAddr)
{
    return (bool)I2C_RD_S_IICIF(baseAddr);
}

/*!
 * @brief Clears the I2C interrupt if set.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_ClearInt(I2C_Type *baseAddr)
{
    I2C_WR_S_IICIF(baseAddr, 0x1U);
}

/*@}*/

#if FSL_FEATURE_I2C_HAS_STOP_DETECT

/*!
 * @name Bus stop detection status
 * @{
 */

/*!
 * @brief Gets the flag indicating a STOP signal was detected on the I2C bus.
 *
 * @param baseAddr The I2C peripheral base address
 * @retval true STOP signal detected on bus.
 * @retval false No STOP signal was detected on the bus.
 */
static inline bool I2C_HAL_GetStopFlag(I2C_Type *baseAddr)
{
    return (bool)I2C_RD_FLT_STOPF(baseAddr);
}

/*!
 * @brief Clears the bus STOP signal detected flag.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_ClearStopFlag(I2C_Type *baseAddr)
{
    I2C_WR_FLT_STOPF(baseAddr, 0x1U);
}

/*@}*/

#if FSL_FEATURE_I2C_HAS_START_DETECT

/*!
 * @name Bus stop detection interrupt
 * @{
 */

/*!
 * @brief Enables the I2C bus stop detection interrupt.
 *
 * @param baseAddr The I2C peripheral base address
 * @param enable   Pass true to enable interrupt, flase to disable.
 */
static inline void I2C_HAL_SetStopIntCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_FLT_SSIE(baseAddr, enable);
}

/*!
 * @brief Returns whether  the I2C bus stop detection interrupts are enabled.
 *
 * @param baseAddr The I2C peripheral base address
 * @retval true Stop detect interrupts are enabled.
 * @retval false Stop detect interrupts are disabled.
 */
static inline bool I2C_HAL_GetStopIntCmd(I2C_Type *baseAddr)
{
    return (bool)I2C_RD_FLT_SSIE(baseAddr);
}

#else

/*! @name Bus stop detection interrupt*/
/*@{*/

/*!
 * @brief Enables the I2C bus stop detection interrupt.
 *
 * @param baseAddr The I2C peripheral base address
 */
static inline void I2C_HAL_SetStopIntCmd(I2C_Type *baseAddr, bool enable)
{
    I2C_WR_FLT_STOPIE(baseAddr, enable);
}

/*!
 * @brief Returns whether the I2C bus stop detection interrupts are enabled.
 *
 * @param baseAddr The I2C peripheral base address
 * @retval true Stop detect interrupts are enabled.
 * @retval false Stop detect interrupts are disabled.
 */
static inline bool I2C_HAL_GetStopIntCmd(I2C_Type *baseAddr)
{
    return (bool)I2C_RD_FLT_STOPIE(baseAddr);
}

#endif /* FSL_FEATURE_I2C_HAS_START_DETECT*/

/*@}*/
#endif /* FSL_FEATURE_I2C_HAS_STOP_DETECT*/

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* __FSL_I2C_HAL_H__*/
       /*******************************************************************************
        * EOF
        ******************************************************************************/
