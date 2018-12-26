/*
 * The Clear BSD License
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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
#if !defined(__FSL_DSPI_HAL_H__)
#define __FSL_DSPI_HAL_H__

#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "dspi/fsl_dspi_types.h"
#include <stdint.h>
#include <stdbool.h>

/*!
 * @addtogroup dspi_hal
 * @{
 */

/*! @file*/
/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* define constant value arrays for the baud rate prescalar and scaler divider values*/
static const uint32_t s_baudratePrescaler[] = {2, 3, 5, 7};
static const uint32_t s_baudrateScaler[] = {2,   4,   6,    8,    16,   32,   64,    128,
                                            256, 512, 1024, 2048, 4096, 8192, 16384, 32768};

/*!
 * @brief DSPI data format settings configuration structure.
 *
 * This structure contains the data format settings.  These settings apply to a specific
 * CTARn register, which the user must provide in this structure.
 */
typedef struct DspiDataFormatConfig
{
    uint32_t bitsPerFrame;             /*!< Bits per frame, min 4, max 16 (master), 32 (slave) */
    dspi_clock_polarity_t clkPolarity; /*!< Active high or low clock polarity*/
    dspi_clock_phase_t clkPhase;       /*!< Clock phase setting to change and capture data*/
    dspi_shift_direction_t direction;  /*!< MSB or LSB data shift direction
                                            This setting relevant only in master mode and
                                            can be ignored in slave  mode */
} dspi_data_format_config_t;

/*!
 * @brief DSPI hardware configuration settings for master mode.
 *
 * Use an instance of this struct with dspi_hal_master_init(). This allows you to configure the
 * most common settings of the DSPI peripheral in master mode with a single function call.
 *
 * The @c bitsPerSec member is handled specially. If this value is set to 0, then the baud is
 * not set by dspi_hal_master_init(), and must be set with a separate call to either
 * dspi_hal_set_baud() or dspi_hal_set_baud_divisors(). This can be useful if you know the
 * divisors in advance and don't want to spend the time to compute them for the provided rate
 * in bits/sec.
 *
 * This structure also contains another structure template as a member:
 * @c dspi_data_format_config_t @c dataConfig.
 * An example useage of this for the user, assuming declaration @c dspi_master_config_t
 * @c dspiConfig:
   @code
    dspiConfig.dataConfig.bitsPerFrame = 16;
    dspiConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    dspiConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
    dspiConfig.dataConfig.direction = kDspiMsbFirst;
   @endcode
 */
typedef struct DspiMasterConfig
{
    bool isEnabled;                            /*!< Set to true to enable the DSPI peripheral.*/
    dspi_ctar_selection_t whichCtar;           /*!< Desired Clock and Transfer Attributes Register (CTAR)*/
    uint32_t bitsPerSec;                       /*!< Baud rate in bits per second.*/
    uint32_t sourceClockInHz;                  /*!< Module source clock  */
    dspi_data_format_config_t dataConfig;      /*!< Data format configuration structure*/
    bool isSckContinuous;                      /*!< Disable(0) or Enable(1) continuous SCK operation*/
    dspi_which_pcs_config_t whichPcs;          /*!< Desired Peripheral Chip Select (pcs) */
    dspi_pcs_polarity_config_t pcsPolarity;    /*!< Peripheral Chip Select (pcs) polarity setting.*/
    dspi_master_sample_point_t masterInSample; /*!< Master data-in (SIN) sample point setting.*/
    bool isModifiedTimingFormatEnabled;        /*!< Disable(0) or Enable(1) modified timing format.*/
    bool isTxFifoDisabled;                     /*!< Disable(1) or Enable(0) tx fifo  */
    bool isRxFifoDisabled;                     /*!< Disable(1) or Enable(0) rx fifo */
} dspi_master_config_t;

/*!
 * @brief DSPI hardware configuration settings for slave mode.
 *
 * Use an instance of this struct with dspi_hal_slave_init(). This allows you to configure the
 * most common settings of the DSPI peripheral in slave mode with a single function call.
 */
typedef struct DspiSlaveConfig
{
    bool isEnabled;                       /*!< Set to true to enable the DSPI peripheral. */
    dspi_data_format_config_t dataConfig; /*!< Data format configuration structure */
    bool isTxFifoDisabled;                /*!< Disable(1) or Enable(0) tx fifo */
    bool isRxFifoDisabled;                /*!< Disable(1) or Enable(0) rx fifo */
} dspi_slave_config_t;

/*!
 * @brief DSPI baud rate divisors settings configuration structure.
 *
 * Note: These settings are relevant only in master mode.
 * This structure contains the baud rate divisor settings. This provides the user the option of
 * explicitly setting these baud rate divisors. In addition, the user must also set the
 * CTARn register they wish to use along with these divisor settings.
 */
typedef struct DspiBaudRateDivisors
{
    bool doubleBaudRate;      /*!< Double Baud rate parameter setting */
    uint32_t prescaleDivisor; /*!< Baud Rate Prescalar parameter setting*/
    uint32_t baudRateDivisor; /*!< Baud Rate scaler parameter setting */
} dspi_baud_rate_divisors_t;

/*!
 * @brief DSPI delay settings configuration structure.
 *
 * Note: These settings are relevant only in master mode.
 * This structure contains the various delay settings.  These settings apply to a specific
 * CTARn register, which the user must provide in this structure.
 */
typedef struct DspiDelaySettingsConfig
{
    uint32_t pcsToSckPre;      /*!< PCS to SCK delay prescalar (PCSSCK) */
    uint32_t pcsToSck;         /*!< PCS to SCK Delay scalar (CSSCK) */
    uint32_t afterSckPre;      /*!< After SCK delay prescalar (PASC)*/
    uint32_t afterSck;         /*!< After SCK delay scalar (ASC)*/
    uint32_t afterTransferPre; /*!< Delay after transfer prescalar (PDT)*/
    uint32_t afterTransfer;    /*!< Delay after transfer scalar (DT) */
} dspi_delay_settings_config_t;

/*!
 * @brief DSPI command and data configuration structure.
 *
 * Note: This structure is used in conjunction with the PUSHR register.
 * PUSHR provides the means to write to the TX FIFO. Data written to this register is
 * transferred to the TX FIFO. Eight- or sixteen-bit write accesses to the PUSHR transfer all
 * 32 register bits to the TX FIFO. The register structure is different in master and slave
 * modes. In master mode the register provides 16-bit command and 16-bit data to the TX
 * FIFO. In slave mode all 32 register bits can be used as data, supporting up to 32-bit SPI
 * frame operation.
 */
typedef struct DspiCommandDataConfig
{
    bool isChipSelectContinuous;      /*!< Option to enable the continuous assertion of chip select
                                           between transfers*/
    dspi_ctar_selection_t whichCtar;  /*!< The desired Clock and Transfer Attributes
                                           Register (CTAR) to use for CTAS*/
    dspi_which_pcs_config_t whichPcs; /*!< The desired PCS signal to use for the data transfer*/
    bool isEndOfQueue;                /*!< Signals that the current transfer is the last in the queue*/
    bool clearTransferCount;          /*!< Clears SPI_TCNT field; cleared before transmission starts*/
} dspi_command_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Configuration
 * @{
 */

/*!
 * @brief Configure the DSPI peripheral in master mode.
 *
 * This function will initialize the module to user defined settings and default settings in master
 * mode.  Here is an example demonstrating how to define the dspi_master_config_t structure and call
 * the dspi_hal_master_init function:
   @code
    dspi_master_config_t dspiConfig;
    dspiConfig.isEnabled = false;
    dspiConfig.whichCtar = kDspiCtar0;
    dspiConfig.bitsPerSec = 0;
    dspiConfig.sourceClockInHz = dspiSourceClock;
    dspiConfig.isSckContinuous = false;
    dspiConfig.whichPcs = kDspiPcs0;
    dspiConfig.pcsPolarity = kDspiPcs_ActiveLow;
    dspiConfig.masterInSample = kDspiSckToSin_0Clock;
    dspiConfig.isModifiedTimingFormatEnabled = false;
    dspiConfig.isTxFifoDisabled = false;
    dspiConfig.isRxFifoDisabled = false;
    dspiConfig.dataConfig.bitsPerFrame = 16;
    dspiConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    dspiConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
    dspiConfig.dataConfig.direction = kDspiMsbFirst;
    dspi_hal_master_init(baseAddr, &dspiConfig, calculatedBaudRate);
   @endcode
 *
 * @param baseAddr Module base address
 * @param config   Pointer to the master mode configuration data structure.
 * @param calculatedBaudRate The calculated baud rate passed back to the user for them to determine
 *                           if the calculated baud rate is close enough to meet their needs.
 * @return  An error code or kStatus_Success.
 */
status_t dspi_hal_master_init(SPI_Type *baseAddr, const dspi_master_config_t *config, uint32_t *calculatedBaudRate);

/*!
 * @brief Configure the DSPI peripheral in slave mode.
 *
 * This function initializes the DSPI module for slave mode. Here is an example demonstrating how
 * to define the dspi_slave_config_t structure and call the dspi_hal_slave_init function:
   @code
    dspi_slave_config_t dspiConfig;
    dspiConfig.isEnabled = false;
    dspiConfig.isTxFifoDisabled = false;
    dspiConfig.isRxFifoDisabled = false;
    dspiConfig.dataConfig.bitsPerFrame = 16;
    dspiConfig.dataConfig.clkPolarity = kDspiClockPolarity_ActiveHigh;
    dspiConfig.dataConfig.clkPhase = kDspiClockPhase_FirstEdge;
    dspi_hal_slave_init(baseAddr, &dspiConfig);
   @endcode
 *
 * @param baseAddr Module base address
 * @param config   Pointer to the slave mode configuration data structure.
 * @return  An error code or kStatus_Success.
 */
status_t dspi_hal_slave_init(SPI_Type *baseAddr, const dspi_slave_config_t *config);

/*!
 * @brief Restore DSPI to reset configuration.
 *
 * This function basically resets all of the DSPI registers to their default setting including
 * disabling the module.
 *
 * @param baseAddr Module base address
 */
void dspi_hal_reset(SPI_Type *baseAddr);

/*!
 * @brief Enable the DSPI peripheral, set MCR MDIS to 0.
 *
 * @param baseAddr Module base address
 */
static inline void dspi_hal_enable(SPI_Type *baseAddr)
{
    SPI_CLR_MCR(baseAddr, SPI_MCR_MDIS_MASK);
}

/*!
 * @brief Disable the DSPI peripheral, set MCR MDIS to 1.
 *
 * @param baseAddr Module base address
 */
static inline void dspi_hal_disable(SPI_Type *baseAddr)
{
    SPI_SET_MCR(baseAddr, SPI_MCR_MDIS_MASK);
}

/*!
 * @brief Set the DSPI baud rate in bits per second.
 *
 * This function will take in the desired bitsPerSec (baud rate) and will calculate the nearest
 * possible baud rate without exceeding the desired baud rate, and will return the calculated
 * baud rate in bits-per-second. It requires that the caller also provide the frequency of the
 * module source clock (in Hz).
 *
 * @param baseAddr Module base address
 * @param whichCtar The desired Clock and Transfer Attributes Register (CTAR) of type
 *                  dspi_ctar_selection_t.
 * @param bitsPerSec The desired baud rate in bits per second.
 * @param sourceClockInHz Module source input clock in Hz.
 * @return  The actual calculated baud rate.
 */
uint32_t dspi_hal_set_baud(SPI_Type *baseAddr,
                           dspi_ctar_selection_t whichCtar,
                           uint32_t bitsPerSec,
                           uint32_t sourceClockInHz);

/*!
 * @brief Configure the baud rate divisors manually.
 *
 * This function allows the caller to manually set the baud rate divisors in the event that
 * these dividers are known and the caller does not wish to call the dspi_hal_set_baud function.
 *
 * @param baseAddr Module base address
 * @param whichCtar The desired Clock and Transfer Attributes Register (CTAR) of type
 *                  dspi_ctar_selection_t.
 * @param divisors Pointer to a structure containing the user defined baud rate divisor settings.
 */
void dspi_hal_set_baud_divisors(SPI_Type *baseAddr,
                                dspi_ctar_selection_t whichCtar,
                                const dspi_baud_rate_divisors_t *divisors);

/*!
 * @brief Configure DSPI for master or slave.
 *
 * @param baseAddr Module base address
 * @param mode Mode setting (master or slave) of type dspi_master_slave_mode_t.
 */
static inline void dspi_hal_set_master_slave(SPI_Type *baseAddr, dspi_master_slave_mode_t mode)
{
    SPI_BWR_MCR_MSTR(baseAddr, (uint32_t)mode);
}

/*!
 * @brief Configure DSPI for continuous SCK operation.
 *
 * @param baseAddr Module base address
 * @param enable Enables (true) or disables(false) continuous sck operation.
 */
static inline void dspi_hal_configure_continuous_sck(SPI_Type *baseAddr, bool enable)
{
    SPI_BWR_MCR_CONT_SCKE(baseAddr, (enable == true));
}

/*!
 * @brief Configure DSPI to enable modified timing format.
 *
 * @param baseAddr Module base address
 * @param enable Enables (true) or disables(false) modified timing format.
 */
static inline void dspi_hal_configure_modified_timing_format(SPI_Type *baseAddr, bool enable)
{
    SPI_BWR_MCR_MTFE(baseAddr, (enable == true));
}

/*!
 * @brief Configure DSPI peripheral chip select strobe enable. Configures PCS[5] to be the
 *        active-low PCS Strobe output.
 *
 * PCS[5] is a special case that can be configured as an active low PCS strobe or as a Peripheral
 * Chip Select in master mode. When configured as a strobe, it provides a signal to an external
 * demultiplexer to decode PCS[0] to PCS[4] signals into as many as 128 glitch-free PCS signals.
 *
 * @param baseAddr Module base address
 * @param enable Enable (true) PCS[5] to operate as the peripheral chip select (PCS) strobe
 *               If disable (false), PCS[5] operates as a peripheral chip select.
 */
static inline void dspi_hal_configure_pcs_strobe(SPI_Type *baseAddr, bool enable)
{
#if !defined(KV11Z7_SERIES)
    SPI_BWR_MCR_PCSSE(baseAddr, (enable == true));
#endif //! defined (KV11Z7_SERIES)
}

/*!
 * @brief Configure DSPI received fifo overflow overwrite enable.
 *
 * When enabled, this function allows incoming receive data to overwrite existing data in the
 * receive shift register when the RX FIFO is full.  Otherwise when disabled, the incoming data
 * is ignored when the RX FIFO is full.
 *
 * @param baseAddr Module base address
 * @param enable If enabled (true), allows incoming data to overwrite rx fifo contents when full,
 *               else incoming data will be ignored.
 */
static inline void dspi_hal_configure_rx_fifo_overwrite(SPI_Type *baseAddr, bool enable)
{
    SPI_BWR_MCR_ROOE(baseAddr, (enable == true));
}

/*!
 * @brief Configure DSPI peripheral chip select polarity.
 *
 * This function will take in the desired peripheral chip select (PCS) and it's
 * corresponding desired polarity and will configure the PCS signal to operate with the
 * desired characteristic.
 *
 * @param baseAddr Module base address
 * @param pcs The particular peripheral chip select (parameter value is of type
 *            dspi_which_pcs_config_t) for which we wish to apply the active high or active
 *            low characteristic.
 * @param activeLowOrHigh The setting for either "active high, inactive low (0)"  or
 *                        "active low, inactive high(1)" of type dspi_pcs_polarity_config_t.
 */
void dspi_hal_configure_pcs_polarity(SPI_Type *baseAddr,
                                     dspi_which_pcs_config_t pcs,
                                     dspi_pcs_polarity_config_t activeLowOrHigh);

/*!
 * @brief Configure DSPI fifos.
 *
 * This function will allow the caller to disable/enable the TX and RX FIFOs (independently).
 * Note that to disable, the caller must pass in a logic 1 (true) for the particular FIFO
 * configuration.  To enable, the caller must pass in a logic 0 (false).  For example, to enable
 * both the TX and RX FIFOs, the caller will make this function call
   @code
    dspi_hal_configure_fifos(baseAddr, false, false);
   @endcode
 *
 * @param baseAddr Module base address
 * @param disableTxFifo Disables (false) the TX Fifo, else enables (true) the TX Fifo
 * @param disableRxFifo Disables (false) the RX Fifo, else enables (true) the RX Fifo
 */
void dspi_hal_configure_fifos(SPI_Type *baseAddr, bool disableTxFifo, bool disableRxFifo);

/*!
 * @brief Flush DSPI fifos.
 *
 * @param baseAddr Module base address
 * @param enableFlushTxFifo Flushes (true) the TX Fifo, else do not flush (false) the TX Fifo
 * @param enableFlushRxFifo Flushes (true) the RX Fifo, else do not flush (false) the RX Fifo
 */
void dspi_hal_flush_fifos(SPI_Type *baseAddr, bool enableFlushTxFifo, bool enableFlushRxFifo);

/*!
 * @brief Configures when the DSPI master samples SIN in Modified Transfer Format
 *
 * This function controls when the DSPI master samples SIN (data in) in the Modified Transfer
 * Format.  Note that this is valid only when the CPHA bit in the CTAR register is 0.
 *
 * @param baseAddr Module base address
 * @param samplePnt selects when the data in (SIN) is sampled, of type dspi_master_sample_point_t.
 *                  This value selects either 0, 1, or 2 system clocks between the SCK edge
 *                  and the SIN (data in) sample.
 */
static inline void dspi_hal_set_datain_samplepoint(SPI_Type *baseAddr, dspi_master_sample_point_t samplePnt)
{
    SPI_BWR_MCR_SMPL_PT(baseAddr, samplePnt);
}

/*!
 * @brief Starts DSPI transfers, clears HALT bit in MCR.
 *
 * This function call called whenever the module is ready to begin data transfers in either master
 * or slave mode.
 *
 * @param baseAddr Module base address
 */
static inline void dspi_hal_start_transfer(SPI_Type *baseAddr)
{
    SPI_CLR_MCR(baseAddr, SPI_MCR_HALT_MASK);
}

/*!
 * @brief Stops (halts) DSPI transfers, sets HALT bit in MCR.
 *
 * This function call called stop data transfers in either master or slave mode.
 *
 * @param baseAddr Module base address
 */
static inline void dspi_hal_stop_transfer(SPI_Type *baseAddr)
{
    SPI_SET_MCR(baseAddr, SPI_MCR_HALT_MASK);
}

/*!
 * @brief Configure the data format for a particular CTAR.
 *
 * This function configures the bits-per-frame, polarity, phase, and shift direction for a
 * particular CTAR. An example use case is as follows:
   @code
    dspi_data_format_config_t dataFormat;
    dataFormat.bitsPerFrame = 16;
    dataFormat.clkPolarity = kDspiClockPolarity_ActiveLow;
    dataFormat.clkPhase = kDspiClockPhase_FirstEdge;
    dataFormat.direction = kDspiMsbFirst;
    dspi_hal_configure_data_format(instance, kDspiCtar0, &dataFormat);
   @endcode
 *
 * @param baseAddr Module base address
 * @param whichCtar The desired Clock and Transfer Attributes Register (CTAR) of type
 *                  dspi_ctar_selection_t.
 * @param config Pointer to a structure containing the user defined data format config settings.
 * @return  An error code or kStatus_Success
 */
status_t dspi_hal_configure_data_format(SPI_Type *baseAddr,
                                        dspi_ctar_selection_t whichCtar,
                                        const dspi_data_format_config_t *config);

/*!
 * @brief Configure the delays for a particular CTAR, master mode only.
 *
 * This function configures the PCS to SCK delay prescalar (PCSSCK),
 * the PCS to SCK Delay scalar (CSSCK),
 * the After SCK delay prescalar (PASC),
 * the After SCK delay scalar (ASC),
 * the Delay after transfer prescalar (PDT),
 * and the Delay after transfer scalar (DT).
 * The following is an example use case of this function:
 * @code
    dspi_delay_settings_config_t delayConfig;
    delayConfig.pcsToSckPre = 0x3;
    delayConfig.pcsToSck = 0xF;
    delayConfig.afterSckPre = 0x2;
    delayConfig.afterSck = 0xA;
    delayConfig.afterTransferPre = 0x1;
    delayConfig.afterTransfer = 0x5;
    dspi_hal_configure_delays(instance, kDspiCtar0, &delayConfig);
 * @endcode
 *
 * @param baseAddr Module base address
 * @param whichCtar The desired Clock and Transfer Attributes Register (CTAR) of type
 *                  dspi_ctar_selection_t.
 * @param config Pointer to a structure containing the user defined delay config settings.
 */
void dspi_hal_configure_delays(SPI_Type *baseAddr,
                               dspi_ctar_selection_t whichCtar,
                               const dspi_delay_settings_config_t *config);

/*@}*/

/*!
 * @name DMA
 * @{
 */

/*!
 * @brief Configure transmit and receive DMA requests.
 *
 * This function configures the FIFOs to generate a DMA or interrupt request.
 *
 * @param enableTransmit Configures tx fifo fill request to generate a dma or interrupt request
 * @param enableReceive Configures rx fifo fill request to generate a dma or interrupt request
 */
void dspi_hal_configure_dma(SPI_Type *baseAddr, bool enableTransmit, bool enableReceive);

/*@}*/

/*!
 * @name Low power
 * @{
 */

/*!
 * @brief Configure DSPI operation during doze mode.
 *
 * This function provides support for an externally controlled Doze mode power-saving mechanism.
 * When disabled, the Doze mode has no effect on the DSPI, and when enable, the Doze mode
 * will disable the DSPI.
 *
 * @param baseAddr Module base address
 * @param enable If disable (false) doze mode has no effect on DSPI, if enabled (true), doze mode
 *                 disables DSPI.
 */
static inline void dspi_hal_configure_doze_mode(SPI_Type *baseAddr, bool enable)
{
    SPI_BWR_MCR_DOZE(baseAddr, (enable == true));
}

/*@}*/

/*!
 * @name Interrupts
 * @{
 */

/*!
 * @brief Configure DSPI interrupts.
 *
 * This function will configure the various interrupt sources of the DSPI.  The parameters to pass
 * in are instance, interrupt source, and enable/disable setting.
 * The interrupt source will be of a typedef enum whose value will be the bit position of the
 * interrupt source setting within the RSER register.  In the DSPI, all of the interrupt
 * configuration settings reside within the one register.  The typedef enum will equate each
 * interrupt source to the bit position defined in the device header file.
 * The function will use these bit positions in its algorithm to enable/disable the
 * interrupt source, where interrupt source is of type dspi_status_and_interrupt_request_t.
 * @code
    temp = (SPI_RD_RSER(instance) & ~interruptSrc) | (enable << interruptSrc);
    SPI_WR_RSER(instance, temp);

    dspi_hal_configure_interrupt(instance, kDspiTxComplete, true); <- example use-case
 * @endcode
 *
 * @param baseAddr Module base address
 * @param interruptSrc The interrupt source, of type dspi_status_and_interrupt_request_t
 * @param enable Enable (true) or disable (false) the interrupt source to generate requests
 */
void dspi_hal_configure_interrupt(SPI_Type *baseAddr, dspi_status_and_interrupt_request_t interruptSrc, bool enable);

/*!
 * @brief Get DSPI interrupt configuration, returns if interrupt request is enabled or disabled.
 *
 * This function will return the requested interrupt source setting (enabled or disabled, of
 * type bool).  The parameters to pass in are instance and interrupt source.  It will utilize the
 * same enum defines for the interrupt sources as described in the "interrupt configuration"
 * function. The function will use these bit positions in its algorithm to obtain the desired
 * interrupt source setting.
 * @code
    return ((SPI_RD_RSER(instance) & interruptSrc) >> interruptSrc);

   getInterruptSetting = dspi_hal_get_interrupt_config(instance, kDspiTxComplete);
 * @endcode
 *
 * @param baseAddr Module base address
 * @param interruptSrc The interrupt source, of type dspi_status_and_interrupt_request_t.
 * @return Configuration of interrupt request: enable (true) or disable (false).
 */
static inline bool dspi_hal_get_interrupt_config(SPI_Type *baseAddr, dspi_status_and_interrupt_request_t interruptSrc)
{
    return ((SPI_RD_RSER(baseAddr) >> interruptSrc) & 0x1);
}

/*@}*/

/*!
 * @name Status
 * @{
 */

/*!
 * @brief Get DSPI status flag state
 *
 * The status flag will be defined in the same enum as the interrupt source enable as the bit
 * position of the interrupt source and corresponding status flag are the same in the RSER and
 * SR registers.  The function will use these bit positions in its algorithm to obtain the desired
 * flag state, similar to the dspi_get_interrupt_config function.
 * @code
    return ((SPI_RD_SR(instance) & statusFlag) >> statusFlag);

    getStatus = dspi_hal_get_status_flag(instance, kDspiTxComplete);
 * @endcode
 *
 * @param instance   Module instance number.
 * @param statusFlag The status flag, of type dspi_status_and_interrupt_request_t.
 */
static inline bool dspi_hal_get_status_flag(SPI_Type *baseAddr, dspi_status_and_interrupt_request_t statusFlag)
{
    return ((SPI_RD_SR(baseAddr) >> statusFlag) & 0x1);
}

/*!
 * @brief Clear DSPI status flag
 *
 * This function will clear the desired status bit, using a write-1-to-clear.  The user passes in
 * the instance and the desired status bit to clear.  The list of status bits are defined in
 * dspi_status_and_interrupt_request_t.  The function will use these bit positions in its algorithm
 * to clear the desired flag state. It will make use of the macro:
 * @code
    SPI_WR_SR(instance, statusFlag);

    dspi_hal_clear_status_flag(instance, kDspiTxComplete);
 * @endcode
 *
 * @param instance   Module instance number.
 * @param statusFlag The status flag, of type dspi_status_and_interrupt_request_t
 */
static inline void dspi_hal_clear_status_flag(SPI_Type *baseAddr, dspi_status_and_interrupt_request_t statusFlag)
{
    SPI_SET_SR(baseAddr, (0x1U << statusFlag));
}

/*!
 * @brief Get DSPI fifo counter or pointer
 *
 * This function will return the number of entries or the next pointer in the tx or rx fifo.
 * The parameters to pass in are the instance and whether it is the tx or rx fifo counter or
 * pointer you want.  The later will be of an enum type that will be defined as the bitmask of
 * those particular bit fields found in the device header file. For example:
 * @code
    return ((SPI_RD_SR_(instance) >> desiredParamter) & 0xF);

    dspi_hal_get_fifo_counter_or_pointer(instance, kDspiRxFifoCounter);
 * @endcode
 *
 * @param instance         Module instance number.
 * @param desiredParameter Desired parameter to obtain, of type dspi_fifo_counter_pointer_t.
 */
static inline uint32_t dspi_hal_get_fifo_counter_or_pointer(SPI_Type *baseAddr,
                                                            dspi_fifo_counter_pointer_t desiredParameter)
{
    return ((SPI_RD_SR(baseAddr) >> desiredParameter) & 0xFU);
}

/*@}*/

/*!
 * @name Data transfer
 * @{
 */

/*!
 * @brief Read data from the data buffer.
 *
 * @param baseAddr Module base address
 */
static inline uint32_t dspi_hal_read_data(SPI_Type *baseAddr)
{
    return SPI_RD_POPR(baseAddr);
}

/*!
 * @brief Write data into the data buffer, slave mode.
 *
 * In slave mode, up to 32-bit words may be written.
 *
 * @param baseAddr Module base address
 * @param data The data to send.
 */
static inline void dspi_hal_write_data_slave_mode(SPI_Type *baseAddr, uint32_t data)
{
    SPI_WR_PUSHR_SLAVE(baseAddr, data);
}

/*!
 * @brief Write data into the data buffer, master mode.
 *
 * In master mode, the 16-bit data is appended with the 16-bit command info. The command portion
 * provides characteristics of the data being sent such as: optional continuous chip select
 * operation between transfers, the desired Clock and Transfer Attributes register to use for the
 * associated SPI frame, the desired PCS signal to use for the data transfer, whether the current
 * transfer is the last in the queue, and whether to clear the transfer count (normally needed when
 * sending the first frame of a data packet). An example use case is as follows:
 * @code
    dspi_command_config_t commandConfig;
    commandConfig.isChipSelectContinuous = true;
    commandConfig.whichCtar = kDspiCtar0;
    commandConfig.whichPcs = kDspiPcs1;
    commandConfig.clearTransferCount = false;
    commandConfig.isEndOfQueue = false;
    dspi_hal_write_data_master_mode(instance, &commandConfig, dataWord);
 * endcode
 *
 * @param baseAddr Module base address
 * @param command Pointer to command structure.
 * @param data The data word to be sent
 */
void dspi_hal_write_data_master_mode(SPI_Type *baseAddr, dspi_command_config_t *command, uint16_t data);

/*!
 * @brief Get the transfer count.
 *
 * This function, when called, will return the current value of the DSPI Transfer Count Register.
 *
 * @param baseAddr Module base address
 * @return The current transfer count.
 */
static inline uint32_t dspi_hal_get_transfer_count(SPI_Type *baseAddr)
{
    return SPI_BRD_TCR_SPI_TCNT(baseAddr);
}

/*!
 * @brief Preset the transfer count.
 *
 * This function allows the caller to preset the DSI Transfer Count Register to a desired value up
 * to 65535; incrementing past this resets the counter back to 0.
 *
 * @param baseAddr Module base address
 * @param presetValue The desired preset value for the transfer counter.
 */
static inline void dspi_hal_preset_transfer_count(SPI_Type *baseAddr, uint16_t presetValue)
{
    SPI_BWR_TCR_SPI_TCNT(baseAddr, presetValue);
}

/*@}*/

/*!
 * @name Debug
 * @{
 */

/*!
 * @brief Read fifo registers for debug purposes.
 *
 * @param baseAddr Module base address
 * @param whichFifo Selects tx or rx fifo, of type dspi_fifo_t.
 * @param whichFifoEntry Selects which fifo entry to read: 0, 1, 2, or 3.
 * @retrun The desired fifo register contents.
 */
uint32_t dspi_hal_get_fifo_data(SPI_Type *baseAddr, dspi_fifo_t whichFifo, uint32_t whichFifoEntry);

/*!
 * @brief Configure DSPI to halt during debug mode.
 *
 * @param baseAddr Module base address
 * @param enable Enables (true) debug mode to halt transfers, else disable to not halt transfer
 *               in debug mode.
 */
static inline void dspi_hal_configure_halt_in_debug_mode(SPI_Type *baseAddr, bool enable)
{
    SPI_BWR_MCR_FRZ(baseAddr, (enable == true));
}

/* @}*/

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* __FSL_DSPI_HAL_H__*/
       /*******************************************************************************
        * EOF
        ******************************************************************************/
