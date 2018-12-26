/*
 * The Clear BSD License
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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
#if !defined(__FSL_LPI2C_HAL_H__)
#define __FSL_LPI2C_HAL_H__

#include <assert.h>
#include <stdbool.h>
#include "fsl_device_registers.h"

/*!
 * @addtogroup lpi2c_hal
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief LPI2C slave status flags. */
typedef enum _lpi2c_slave_status_flag
{
    kLPI2CSlaveTransmitData = LPI2C_SSR_TDF_SHIFT,
    kLPI2CSlaveReceiveData = LPI2C_SSR_RDF_SHIFT,
    kLPI2CSlaveAddressValid = LPI2C_SSR_AVF_SHIFT,
    kLPI2CSlaveTransmitAck = LPI2C_SSR_TAF_SHIFT,
    kLPI2CSlaveStopDetect = LPI2C_SSR_SDF_SHIFT,
    kLPI2CSlaveBitError = LPI2C_SSR_BEF_SHIFT,
    kLPI2CSlaveAddressMatch0 = LPI2C_SSR_AM0F_SHIFT,
    kLPI2CSlaveAddressMatch1 = LPI2C_SSR_AM1F_SHIFT,
    kLPI2CSlaveBusy = LPI2C_SSR_SBF_SHIFT,
    kLPI2CSlaveBusBusy = LPI2C_SSR_BBF_SHIFT
} lpi2c_slave_status_flag_t;

typedef enum _lpi2c_slave_transmit_flag_configuration
{
    kLPI2CSlaveTransmitFlagAssertDuringTransfer = 0,
    kLPI2CSlaveTransmitFlagAssertWheneverEmpty
} lpi2c_slave_transmit_flag_configuration_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/

extern const uint32_t g_lpi2cBaseAddr[];

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
 * @brief Restores the I2C slave peripheral to reset state.
 *
 * @param baseAddr The I2C peripheral base address
 */
void lpi2c_hal_reset_slave(LPI2C_Type *baseAddr);

/*!
 * @brief Enables the LPI2C slave module operation
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enable_slave(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SCR_SEN(baseAddr, 0x1U);
}

/*! @} */

/*!
 * @name Configurations
 * @{
 */

/*!
 * @brief Enables clock stretching during slave-receiver transfers
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enbale_slave_ack_scl_stall(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SCFGR1_ACKSTALL(baseAddr, 0x1U);
}

/*!
 * @brief Enables SCL clock stretching when the transmit data flag is set
 *     during a slave-transmit transfer.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enbale_slave_tx_data_scl_stall(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SCFGR1_TXDSTALL(baseAddr, 0x1U);
}

/*!
 * @brief Enables SCL clock stretching when receive data flag is set
 *     during a slave-receive transfer.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enbale_slave_rx_scl_stall(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SCFGR1_RXSTALL(baseAddr, 0x1U);
}

/*!
 * @brief Enables SCL clock stretching when the address valid flag is asserted.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enbale_slave_address_scl_stall(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SCFGR1_ADRSTALL(baseAddr, 0x1U);
}

/*!
 * @brief Sets the LPI2C slave glitch filters for SDA input.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_set_slave_sda_glitch_filter(LPI2C_Type *baseAddr, uint32_t cycles)
{
    LPI2C_WR_SCFGR2_FILTSDA(baseAddr, cycles);
}

/*!
 * @brief Sets the LPI2C slave glitch filters for SCL input.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_set_slave_scl_glitch_filter(LPI2C_Type *baseAddr, uint32_t cycles)
{
    LPI2C_WR_SCFGR2_FILTSCL(baseAddr, cycles);
}

/*!
 * @brief Enables the LPI2C slave digital filtering.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enable_slave_digital_filtering(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SCR_FILTEN(baseAddr, 0x1U);
}

/*!
 * @brief Sets the SDA data valid delay time for the LPI2C slave.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_set_slave_data_valid_delay(LPI2C_Type *baseAddr, uint32_t cycles)
{
    LPI2C_WR_SCFGR2_DATAVD(baseAddr, cycles);
}

/*!
 * @brief Sets the minimum clock hold time for the LPI2C slave.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_set_slave_clock_hold_time(LPI2C_Type *baseAddr, uint32_t cycles)
{
    LPI2C_WR_SCFGR2_CLKHOLD(baseAddr, cycles);
}

/*!
 * @brief Config transmit data flag for the LPI2C slave.
 *
 * @param baseAddr The LPI2C peripheral base address
 * @param config The config value, defined in type lpi2c_slave_transmit_flag_configuration_t.
 */
static inline void lpi2c_hal_config_slave_transmit_data_flag(LPI2C_Type *baseAddr,
                                                             lpi2c_slave_transmit_flag_configuration_t config)
{
    LPI2C_WR_SCFGR1_TXCFG(baseAddr, config);
}

/*! @} */

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Enable the slave receive buffer full interrupt.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enable_slave_receive_data_interrupt(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SIER_RDIE(baseAddr, 0x1U);
}

/*!
 * @brief Enable the slave transmit buffer empty interrupt.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enable_slave_transmit_data_interrupt(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SIER_TDIE(baseAddr, 0x1U);
}

/*!
 * @brief Enable the slave address valid interrupt.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enable_slave_address_valid_interrupt(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SIER_AVIE(baseAddr, 0x1U);
}

/*!
 * @brief Enable the slave bit error interrupt.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enable_slave_bit_error_interrupt(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SIER_BEIE(baseAddr, 0x1U);
}

/*!
 * @brief Enable the slave address match 0 interrupt.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enable_slave_address_match0_interrupt(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SIER_AM0IE(baseAddr, 0x1U);
}

/*!
 * @brief Enable the slave stop detect interrupt.
 *
 * @param baseAddr The LPI2C peripheral base address
 */
static inline void lpi2c_hal_enable_slave_stop_detect_interrupt(LPI2C_Type *baseAddr)
{
    LPI2C_WR_SIER_SDIE(baseAddr, 0x1U);
}

/*! @} */

/*!
 * @name Data transfer
 * @{
 */

/*!
 * @brief Returns the last byte of data read from the bus and initiate another read.
 *
 * @param baseAddr The LPI2C peripheral base address
 * @return This function returns the last byte received while the I2C module is configured in slave
 *     slave receive mode.
 */
static inline uint8_t lpi2c_hal_read_slave_byte(LPI2C_Type *baseAddr)
{
    return LPI2C_RD_SRDR_DATA(baseAddr);
}

/*!
 * @brief Writes one byte of data to the I2C bus.
 *
 * This function is called in the slave transmit mode, a data transfer is initiated
 * after an address match occurs.
 *
 * @param baseAddr The LPI2C peripheral base address.
 * @param byte The byte of data to transmit.
 */
static inline void lpi2c_hal_write_slave_byte(LPI2C_Type *baseAddr, uint8_t byte)
{
    LPI2C_WR_STDR(baseAddr, byte);
}

/*!
 * @brief transmits an ack to the I2C bus.
 *
 * @param baseAddr The LPI2C peripheral base address.
 */
static inline void lpi2c_hal_slave_transmit_ack(LPI2C_Type *baseAddr)
{
    LPI2C_WR_STAR_TXNACK(baseAddr, 0);
}

/*! @} */

/*!
 * @name Slave address
 * @{
 */

/*!
 * @brief Sets the primary 7-bit slave address 0.
 *
 * @param baseAddr The LPI2C peripheral base address
 * @param address The slave address in the upper 7 bits. Bit 0 of this value must be 0.
 */
void lpi2c_hal_set_slave_7bit_address0(LPI2C_Type *baseAddr, uint8_t address);

/*!
 * @brief Get LPI2C slave address status.
 *
 * @param baseAddr The LPI2C peripheral base address
 * @return status of slave address.
 */
static inline uint16_t lpi2c_hal_get_slave_address_status(LPI2C_Type *baseAddr)
{
    return LPI2C_RD_SASR_RADDR(baseAddr);
}

/*! @} */

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get LPI2C slave status flag state
 *
 * @param baseAddr The LPI2C peripheral base address.
 * @param statusFlag The status flag, defined in type lpi2c_slave_status_flag_t.
 * @return State of the status flag: asserted (true) or not-asserted (false).
 *         - true: related status flag is being set.
 *         - false: related status flag is not set.
 */
static inline bool lpi2c_hal_get_slave_status_flag(LPI2C_Type *baseAddr, lpi2c_slave_status_flag_t statusFlag)
{
    return (bool)((LPI2C_RD_SSR(baseAddr) >> statusFlag) & 0x1U);
}

/*!
 * @brief Clear LPI2C slave status flag state
 *
 * @param baseAddr The LPI2C peripheral base address.
 * @param statusFlag The status flag, defined in type lpi2c_slave_status_flag_t.
 */
static inline void lpi2c_hal_clear_slave_status_flag(LPI2C_Type *baseAddr, lpi2c_slave_status_flag_t statusFlag)
{
    LPI2C_WR_SSR(baseAddr, 0x1U << statusFlag);
}

/*! @} */

#endif /* __FSL_LPI2C_HAL_H__*/
       /*******************************************************************************
        * EOF
        ******************************************************************************/
