/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
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

/*!=================================================================================================
\file       app_temp_sensor.c
\brief      This is a public source file for the application temperature sensor.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/

/* General Includes */
#include "EmbeddedTypes.h"
#include "network_utils.h"
#include "stdlib.h"

/* Drivers */
#include "board.h"
#include "fsl_adc16.h"
#include "fsl_pmc.h"

#include "app_temp_sensor.h"
#include "FunctionLib.h"
#include "MemManager.h"

/*==================================================================================================
Private macros
==================================================================================================*/
#define ADC16_INSTANCE                (0)   /* ADC instance */
#define ADC16_CHN_GROUP               (0)   /* ADC group configuration selection */

#define ADC16_TEMP_SENSOR_CHN         (26) /* ADC channel of the Temperature Sensor */
#define ADC16_BANDGAP_CHN             (27) /* ADC channel of BANDGAP Voltage reference */

#define ADCR_VDD                      (65535U)    /* Maximum value when use 16b resolution */
#define V_BG                          (1000U)     /* BANDGAP voltage in mV (trim to 1.0V) */
#define V_TEMP25                      (716U)      /* Typical converted value at 25 oC in mV */
#define M                             (1620U)     /* Typical slope:uV/oC */
#define STANDARD_TEMP                 (25) 

#define TEMP_BUFF_SIZE     (20U)
/*==================================================================================================
Private global variables declarations
==================================================================================================*/
#if USE_TEMPERATURE_SENSOR  

#if !defined(CPU_MKW41Z512VHT4)
uint32_t offsetVdd = 0;
#endif
static uint32_t adcValue = 0; /* ADC value */
static adc16_config_t adcUserConfig; /* structure for user config */

/*==================================================================================================
Public global variables declarations
==================================================================================================*/

/* None */

/*==================================================================================================
Private prototypes
==================================================================================================*/
static void ADC16_CalibrateParams(void);
static uint16_t ADC16_ReadValue(uint32_t chnIdx, uint8_t diffMode);
static int32_t BOARD_GetTemperature(void);

/*==================================================================================================
Public functions
==================================================================================================*/
/*!*************************************************************************************************
\fn       void APP_InitADC(uint32_t instance)
\brief    ADC module initialization
\return   void
***************************************************************************************************/

void APP_InitADC(uint32_t instance)
{
    CLOCK_EnableClock(kCLOCK_Adc0);
    ADC16_GetDefaultConfig(&adcUserConfig);
    adcUserConfig.resolution = kADC16_ResolutionSE16Bit;
    adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
    ADC16_Init(ADC0, &adcUserConfig);  
    ADC16_CalibrateParams();
}

/*!*************************************************************************************************
\fn       int32_t APP_GetCurrentTempValue(void))
\brief    Gets current temperature of chip.
\return   int32_t temperature
***************************************************************************************************/
int32_t APP_GetCurrentTempValue(void)
{  
    return BOARD_GetTemperature();
}
#endif
/*!*************************************************************************************************
\fn     void* App_GetTempDataString(void)
\brief  Return post data.

\param  [in]    none

\return         return data to be send through post
***************************************************************************************************/
void* App_GetTempDataString
(
    void
)
{
#if USE_TEMPERATURE_SENSOR
    /* Compute temperature */
    int32_t temperature = (int32_t)APP_GetCurrentTempValue();
    uint8_t* pIndex = NULL;
    uint8_t sTemp[] = "Temp:"; 
    uint8_t * sendTemperatureData = MEM_BufferAlloc(TEMP_BUFF_SIZE);
    if(NULL == sendTemperatureData)
    {
      return sendTemperatureData;
    }
    
    /* Clear data and reset buffers */
    FLib_MemSet(sendTemperatureData, 0, TEMP_BUFF_SIZE);
    
    /* Compute output */
    pIndex = sendTemperatureData;
    FLib_MemCpy(pIndex, sTemp, SizeOfString(sTemp));
    pIndex += SizeOfString(sTemp);
    NWKU_PrintDec((uint8_t)(temperature/100), pIndex, 2, TRUE);
    pIndex += 2; /* keep only the first 2 digits */
    *pIndex = '.';
    pIndex++;
    NWKU_PrintDec((uint8_t)(abs(temperature)%100), pIndex, 2, TRUE);
    return sendTemperatureData;
#else
    return NULL;
#endif    
}

#if USE_TEMPERATURE_SENSOR
/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn    static int32_t BOARD_GetTemperature(void)

\brief  Return the board temperature

\return  int32_t board temperature * 100
***************************************************************************************************/
static int32_t BOARD_GetTemperature(void)
{
    uint16_t tempVal, bandgapValue, tempVolt, bgVolt = 100; /*cV*/
    uint32_t vdd, adcrTemp25, adcr100m;       
    
    bandgapValue = ADC16_ReadValue(ADC16_BANDGAP_CHN, false);;
    
    tempVal = ADC16_ReadValue(ADC16_TEMP_SENSOR_CHN, false);
    
    tempVolt = bgVolt * tempVal / bandgapValue;
    (void)tempVolt;
    
    /* Get VDD value measured in mV */
    /* VDD = (ADCR_VDD x V_BG) / ADCR_BG  */
    vdd = ADCR_VDD * V_BG / bandgapValue;
    /* Calibrate ADCR_TEMP25  */
    /* ADCR_TEMP25 = ADCR_VDD x V_TEMP25 / VDD  */
    adcrTemp25 = ADCR_VDD * V_TEMP25 / vdd;
    /* Calculate conversion value of 100mV. */
    /* ADCR_100M = ADCR_VDD x 100 / VDD */
    adcr100m = ADCR_VDD*100/ vdd;    
        
    /* Multiplied by 1000 because M in uM/oC */
    /* Temperature = 25 - (ADCR_T - ADCR_TEMP25) * 100*1000 / ADCR_100M*M */    
    return (int32_t)((int32_t)STANDARD_TEMP*100 - ((int32_t)adcValue - (int32_t)adcrTemp25) * 10000000 /(int32_t)(adcr100m*M));
}

/*!*************************************************************************************************
\private
\fn    static uint16_t ADC16_ReadValue(uint32_t chnIdx, uint8_t diffMode)

\brief  This function used BANDGAP as reference voltage to measure vdd and
        calibrate V_TEMP25 with that vdd value.
***************************************************************************************************/
static void ADC16_CalibrateParams(void)
{
#if gDCDC_Enabled_d == 0    
 #if FSL_FEATURE_ADC16_HAS_CALIBRATION
      ADC16_DoAutoCalibration(ADC0);
 #endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
      ADC16_SetHardwareAverage(ADC0 , kADC16_HardwareAverageCount16);
#endif
  
    pmc_bandgap_buffer_config_t pmcBandgapConfig = 
{
#if (defined(FSL_FEATURE_PMC_HAS_BGBE) && FSL_FEATURE_PMC_HAS_BGBE)
       .enable = true, /*!< Enable bandgap buffer.                   */
#endif
#if (defined(FSL_FEATURE_PMC_HAS_BGEN) && FSL_FEATURE_PMC_HAS_BGEN)
    .enableInLowPowerMode = false, /*!< Enable bandgap buffer in low power mode. */
#endif                         /* FSL_FEATURE_PMC_HAS_BGEN */
#if (defined(FSL_FEATURE_PMC_HAS_BGBDS) && FSL_FEATURE_PMC_HAS_BGBDS)
    .drive = kPmcBandgapBufferDriveLow, /*!< Bandgap buffer drive select.             */
#endif                                       /* FSL_FEATURE_PMC_HAS_BGBDS */
} ;
    
    /* Enable BANDGAP reference voltage */
    PMC_ConfigureBandgapBuffer(PMC, &pmcBandgapConfig);
}

/*!*************************************************************************************************
\private
\fn    static uint16_t ADC16_ReadValue(uint32_t chnIdx, uint8_t diffMode)

\brief  This function measure the ADC channel given as input

\param  [in]    chnIdx           adc channel id
\param  [in]    diffMode         enable Differential Conversion

\return  uint16_t adc 16 bit value
***************************************************************************************************/
static uint16_t ADC16_ReadValue
(
    uint32_t chnIdx, 
    uint8_t diffMode
)
{
  adc16_channel_config_t chnConfig;

    /* Configure the conversion channel */
    chnConfig.channelNumber     = chnIdx;
#if FSL_FEATURE_ADC16_HAS_DIFF_MODE
    chnConfig.enableDifferentialConversion = diffMode;
#endif
    chnConfig.enableInterruptOnConversionCompleted  = false;
    
    /* Software trigger the conversion */
    ADC16_SetChannelConfig(ADC0, ADC16_CHN_GROUP, &chnConfig);
    /* Wait for the conversion to be done */
    while (0U == (kADC16_ChannelConversionDoneFlag & ADC16_GetChannelStatusFlags(ADC0, ADC16_CHN_GROUP)));

    /* Fetch the conversion value */
    adcValue =  ADC16_GetChannelConversionValue(ADC0, ADC16_CHN_GROUP);
    /* Calculates adcValue in 16bit resolution from 12bit resolution 
    in order to convert to reading */
#if (FSL_FEATURE_ADC16_MAX_RESOLUTION < 16)
    adcValue = adcValue << 4;
#endif
    /* Pause the conversion */
   
    return adcValue;
}
#endif /* USE_TEMPERATURE_SENSOR */
/*===============================================================================================
Private debug functions
==================================================================================================*/

