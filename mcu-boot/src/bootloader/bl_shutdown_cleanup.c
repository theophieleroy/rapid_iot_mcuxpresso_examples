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

#include "fsl_device_registers.h"
#include "utilities/fsl_rtos_abstraction.h"
#include "utilities/vector_table_info.h"
#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#if !BL_DEVICE_IS_LPC_SERIES
#include "fsl_flash.h"
#else
#include "flashiap_wrapper/fsl_flashiap_wrapper.h"
#endif
#endif // #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#include "microseconds.h"
#include "bootloader_common.h"

#include "bootloader/bl_shutdown_cleanup.h"
#include "bootloader/bl_context.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
extern void init_interrupts(void);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See bl_shutdown_cleanup.h for documentation of this function.
void shutdown_cleanup(shutdown_type_t shutdown)
{
    if (shutdown != kShutdownType_Reset)
    {
        // Clear (flush) the flash cache.
#if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
#if !BL_DEVICE_IS_LPC_SERIES
        //flash_cache_clear(NULL);
    FTFx_CACHE_ClearCachePrefetchSpeculation(g_bootloaderContext.allFlashCacheState, true);
    FTFx_CACHE_ClearCachePrefetchSpeculation(g_bootloaderContext.allFlashCacheState, false);
#endif // !BL_DEVICE_IS_LPC_SERIES
#endif // #if !BL_FEATURE_HAS_NO_INTERNAL_FLASH
    }

    if (shutdown != kShutdownType_Cleanup)
    {
        // Shutdown all peripherals because they could be active
        uint32_t i;
        for (i = 0; g_peripherals[i].typeMask != 0; i++)
        {
            if (g_peripherals[i].controlInterface->shutdown)
            {
                g_peripherals[i].controlInterface->shutdown(&g_peripherals[i]);
            }
        }
    }

    // If we are permanently exiting the bootloader, there are a few extra things to do.
    if (shutdown == kShutdownType_Shutdown)
    {
        // Turn off global interrupt
        lock_acquire();

        // Shutdown microseconds driver.
        microseconds_shutdown();

        // Disable force ROM.
#if defined(RCM_FM_FORCEROM_MASK)
        RCM->FM = ((~RCM_FM_FORCEROM_MASK) & RCM->FM) | RCM_FM_FORCEROM(0);
#elif defined(SMC_FM_FORCECFG_MASK)
#if defined(SMC0)
        SMC0->FM = ((~SMC_FM_FORCECFG_MASK) & SMC0->FM) | SMC_FM_FORCECFG(0);
#else
        SMC->FM = ((~SMC_FM_FORCECFG_MASK) & SMC->FM) | SMC_FM_FORCECFG(0);
#endif
#endif // defined(RCM_FM_FORCEROM_MASK)

        // Clear status register (bits are w1c).
#if defined(RCM_MR_BOOTROM_MASK)
        RCM->MR = ((~RCM_MR_BOOTROM_MASK) & RCM->MR) | RCM_MR_BOOTROM(3);
#elif defined(SMC_MR_BOOTCFG_MASK)
#if defined(SMC0)
        SMC0->MR = ((~SMC_MR_BOOTCFG_MASK) & SMC0->MR) | SMC_MR_BOOTCFG(3);
#else
        SMC->MR = ((~SMC_MR_BOOTCFG_MASK) & SMC->MR) | SMC_MR_BOOTCFG(3);
#endif
#endif // defined(RCM_MR_BOOTROM_MASK)

        init_interrupts();

        // Set the VTOR to default.
        SCB->VTOR = kDefaultVectorTableAddress;

        // Restore clock to default before leaving bootloader.
        configure_clocks(kClockOption_ExitBootloader);

        // De-initialize hardware such as disabling port clock gate
        deinit_hardware();

        // Restore global interrupt.
        __enable_irq();

#if BL_FEATURE_BYPASS_WATCHDOG
        // De-initialize watchdog
        bootloader_watchdog_deinit();
#endif // BL_FEATURE_BYPASS_WATCHDOG
    }

    // Memory barriers for good measure.
    __ISB();
    __DSB();
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
