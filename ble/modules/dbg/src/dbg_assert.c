/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


/*
 * INCLUDES
 ****************************************************************************************
 */
 
#include "dbg_assert.h"      // architectural platform definitions
#include "reg_assert_mgr.h"
#include "fsl_common.h"

/**
 ****************************************************************************************
 * @addtogroup DRIVERS
 * @{
 *
 *
 * ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */



/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */



#if (PLF_DEBUG)
/// Variable to enable infinite loop on assert
volatile int dbg_assert_block = 1;

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void assert_err(const char *condition, const char * file, int line)
{
    asrt_line_set(line);
    asrt_addr_setf((uint32_t)file);
    asrt_trigg_setf(1);

    DisableGlobalIRQ();
    while(dbg_assert_block);
}

void assert_param(int param0, int param1, const char * file, int line)
{
    asrt_line_set(line);
    asrt_addr_setf((uint32_t)file);
    asrt_params_setf(1);
    asrt_param_1_setf(param0);
    asrt_param_2_setf(param1);
    asrt_params_setf(1);
    asrt_trigg_setf(1);

    DisableGlobalIRQ();
    while(dbg_assert_block);
}

void assert_warn(int param0, int param1, const char * file, int line)
{
    asrt_line_set(line);
    asrt_addr_setf((uint32_t)file);
    asrt_params_setf(0);
    asrt_warn_setf(1);
}

void dump_data(uint8_t* data, uint16_t length)
{
    asrt_param_1_setf(length);
    asrt_params_setf(1);
    asrt_addr_setf((uint32_t)data);
    asrt_warn_setf(1);
}
#endif //PLF_DEBUG

#if RW_DEBUG_STACK_PROF
uint16_t get_stack_usage(void)
{
    // TODO
    return 0;
}
#endif

/// @} DRIVERS
