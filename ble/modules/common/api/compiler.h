/**
 ****************************************************************************************
 *
 * @file rvds/compiler.h
 *
 * @brief Definitions of compiler specific directives.
 *
 * Copyright (C) RivieraWaves 2009-2013
 *
 *
 ****************************************************************************************
 */

#ifndef _COMPILER_H_
#define _COMPILER_H_

#if (!defined __ICCARM__) && (!defined __GNUC__)

#ifndef __ARMCC_VERSION
#error "File only included with RVDS!"
#endif // __ARMCC_VERSION

/// define the static keyword for this compiler
#define __STATIC static

/// define the force inlining attribute for this compiler
#undef __FORCEINLINE
#define __FORCEINLINE __forceinline static

/// define the IRQ handler attribute for this compiler
#define __IRQ __irq

/// define the BLE IRQ handler attribute for this compiler
#define __BTIRQ

/// define the BLE IRQ handler attribute for this compiler
#define __BLEIRQ

/// define the FIQ handler attribute for this compiler
#define __FIQ __irq

#ifdef __cplusplus
#define __ARRAY_EMPTY 1
#else
/// define size of an empty array (used to declare structure with an array size not defined)
#define __ARRAY_EMPTY
#endif

/// Put a variable in a memory maintained during deep sleep
#define __LOWPOWER_SAVED

/// Put a variable in a memory not maintained during deep sleep
#define __LOWPOWER_UNSAVED __attribute__((section(".lpus")));

#else

#define __STATIC static

#undef __FORCEINLINE
#define __FORCEINLINE static inline
#define __ARRAY_EMPTY

/// define the BLE IRQ handler attribute for this compiler
#define __BLEIRQ

#endif /* __ICCARM__ */

#endif // _COMPILER_H_
