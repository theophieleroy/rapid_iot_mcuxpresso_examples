/**
 ****************************************************************************************
 *
 * @file intc.h
 *
 * @brief Declaration of the Interrupt Controller API.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 ****************************************************************************************
 */

#ifndef _INTC_H_
#define _INTC_H_

/**
 ****************************************************************************************
 * @addtogroup INTC INTC
 * @ingroup DRIVERS
 *
 * @brief Declaration of the Interrupt Controller API.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "fsl_common.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// BLE stack configuration interface.
extern struct ble_config_st ble_config;

/** @brief Disable interrupts without tuner tx&rx and frequency hop interrupt globally in the system.
 * This macro must be used in conjunction with the @ref GLOBAL_INT_RESTORE macro since this
 * last one will close the brace that the current macro opens.  This means that both
 * macros must be located at the same scope level.
 */
#define GLOBAL_INT_DISABLE()                  \
    do                                        \
    {                                         \
        uint32_t int_restore0;                \
        uint32_t int_restore1;                \
        int_restore0 = NVIC->ISER[0];         \
        int_restore1 = NVIC->ISER[1];         \
        NVIC->ICER[0] = ble_config.int_mask0; \
        NVIC->ICER[1] = ble_config.int_mask1;

/** @brief Restore interrupts from the previous global disable.
 * @sa GLOBAL_INT_DISABLE
 */
#define GLOBAL_INT_RESTORE()                             \
    NVIC->ISER[0] = int_restore0 & ble_config.int_mask0; \
    NVIC->ISER[1] = int_restore1 & ble_config.int_mask1; \
    }                                                    \
    while (0)

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/// @} INTC

#endif // _INTC_H_
