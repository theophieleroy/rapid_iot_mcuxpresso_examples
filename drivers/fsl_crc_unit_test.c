#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "bootloader_common.h"
#include "bootloader/bl_peripheral.h"
#include "fsl_crc_driver.h"
#include "fsl_device_registers.h"
#if defined(K22F51212_SERIES)
#include "scuart.h"
#elif defined(K80F25615_SERIES) || defined(KL28T7_SERIES)
#include "lpuart/hal/fsl_lpuart_hal.h"
#endif
#include <assert.h>

#define DEBUG_UART_BAUD (9600)

#define CRC_INSTANCE (0)

#define ERROR_DEBUG() printf("CRC DRIVER UNIT TEST ERROR: function[%s] line[%d]\r\n", __func__, __LINE__)

extern void uart_pinmux_config(unsigned int instance, pinmux_type_t pinmux);

/* test CRC-CCITT calculation for 32-bit, 16-bit and 8-bit values */
int32_t CRC_DRV_test_CCITT(void)
{
    crc_user_config_t userConfigPtr;
    uint32_t result;
    uint8_t buffer[10] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30};

    userConfigPtr.crcWidth = kCrc16Bits;
    userConfigPtr.seed = 0xFFFFU;
    userConfigPtr.polynomial = 0x1021U;
    userConfigPtr.writeTranspose = kCrcNoTranspose;
    userConfigPtr.readTranspose = kCrcNoTranspose;
    userConfigPtr.complementRead = false;

    CRC_DRV_Init(CRC_INSTANCE, &userConfigPtr);

    result = CRC_DRV_GetCrcBlock(CRC_INSTANCE, buffer, 10);
    if (result != 0x3218)
    {
        ERROR_DEBUG();
        return -1;
    }
    CRC_DRV_Deinit(CRC_INSTANCE);
    return 0;
}

/* test CRC-KERMIT calculation for 32-bit, 16-bit and 8-bit values */
int32_t CRC_DRV_test_KERMIT(void)
{
    crc_user_config_t userConfigPtr;
    uint32_t result;
    uint8_t buffer[11] = {0x00, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x30};

    userConfigPtr.crcWidth = kCrc16Bits;
    userConfigPtr.seed = 0x0000U;
    userConfigPtr.polynomial = 0x1021U;
    userConfigPtr.writeTranspose = kCrcTransposeBits;
    userConfigPtr.readTranspose = kCrcTransposeBits;
    userConfigPtr.complementRead = false;

    CRC_DRV_Init(CRC_INSTANCE, &userConfigPtr);

    result = CRC_DRV_GetCrcBlock(CRC_INSTANCE, &buffer[1], 10);
    if (result != 0x6B28)
    {
        ERROR_DEBUG();
        return -1;
    }
    CRC_DRV_Deinit(CRC_INSTANCE);
    return 0;
}

int main(void)
{
    int32_t status = 0;
    init_hardware();

#if defined(K22F51212_SERIES)
    uart_pinmux_config(1, kPinmuxType_Peripheral);
    while (scuart_init(UART1, get_uart_clock(1), DEBUG_UART_BAUD, NULL) != kStatus_Success)
        ;
#elif defined(K80F25615_SERIES)
    uint32_t baseAddr = LPUART0_BASE;

    uart_pinmux_config(0, kPinmuxType_Peripheral);

    // Ungate the LPUART clock.
    BW_SIM_SCGC2_LPUART0(SIM_BASE, 0x1U);

    // Must disable transmitter and receiver before changing the baud rate.
    lpuart_hal_disable_transmitter(baseAddr);
    lpuart_hal_disable_receiver(baseAddr);

    lpuart_hal_set_baud_rate(baseAddr, get_uart_clock(0), TERMINAL_BAUD);

    // Reenable transmitter.
    lpuart_hal_enable_transmitter(baseAddr);
    lpuart_hal_enable_receiver(baseAddr);
#elif defined(KL28T7_SERIES)
    uint32_t baseAddr = LPUART2_BASE;

    uart_pinmux_config(2, kPinmuxType_Peripheral);

    // Ungate the LPUART clock.
    BW_PCC_CLKCFGn_CGC(PCC0_BASE, PCC_CLKCFG_ADDR_LPUART2_OFFSET, BV_PCC_CLKCFGn_CGC__ENABLED);

    // Must disable transmitter and receiver before changing the baud rate.
    lpuart_hal_disable_transmitter(baseAddr);
    lpuart_hal_disable_receiver(baseAddr);

    lpuart_hal_set_baud_rate(baseAddr, get_uart_clock(2), TERMINAL_BAUD);

    // Reenable transmitter.
    lpuart_hal_enable_transmitter(baseAddr);
    lpuart_hal_enable_receiver(baseAddr);
#endif

    printf("\r\nCRC driver unit test start!\r\n");

    status += CRC_DRV_test_KERMIT();
    status += CRC_DRV_test_CCITT();

    if (status == 0)
    {
        printf("\r\nCRC driver unit test Finished successfully!\r\n");
        return 0;
    }
    else
    {
        printf("\r\nCRC driver unit test Failed!\r\n");
        return 0;
    }
}
