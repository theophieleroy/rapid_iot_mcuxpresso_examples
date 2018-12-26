/******************************************************************************
 *
 * Freescale Semiconductor Inc.
 * (c) Copyright 2004-2009 Freescale Semiconductor, Inc.
 * ALL RIGHTS RESERVED.
 *
 ******************************************************************************
 *
 * THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/
/*
 * Note: This file is recreated by the project wizard whenever the MCU is
 *       changed and should not be edited by hand
 */

/* Include the derivative-specific header file */

#include "bootloader_common.h"

#include "fsl_device_registers.h"

/*
 * Include the platform specific header file
 */
//#if (defined(TWR_K40X256))
//  #include <MK40N512VMD100.h>
//#elif (defined(TWR_K53N512))
//  #include <MK53N512CMD100.h>
//#elif (defined(TWR_K60N512))
//  #include <MK60N512VMD100.h>
//#elif (defined(TWR_K20X128))
//  #include <MK20D5.h>
//#elif (defined(TWR_K21D50M))
//  #include <MK21D5.h>
//#elif (defined(CPU_MKL25Z128VLK4))
//  #include <MKL25Z4.h>
//#else
//  #error "No valid platform defined"
//#endif

#define __MK_xxx_H__
#define LITLLE_ENDIAN

