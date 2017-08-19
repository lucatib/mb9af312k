/*************************************************************************************
* Copyright (C) 2013-2015, Cypress Semiconductor Corporation. All rights reserved.    
*                                                                                     
* This software, including source code, documentation and related                     
* materials ( "Software" ), is owned by Cypress Semiconductor                         
* Corporation ( "Cypress" ) and is protected by and subject to worldwide              
* patent protection (United States and foreign), United States copyright              
* laws and international treaty provisions. Therefore, you may use this               
* Software only as provided in the license agreement accompanying the                 
* software package from which you obtained this Software ( "EULA" ).                  
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,             
* non-transferable license to copy, modify, and compile the                           
* Software source code solely for use in connection with Cypress's                    
* integrated circuit products. Any reproduction, modification, translation,           
* compilation, or representation of this Software except as specified                 
* above is prohibited without the express written permission of Cypress.              
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                                
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                                
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                        
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                                     
* PARTICULAR PURPOSE. Cypress reserves the right to make                              
* changes to the Software without notice. Cypress does not assume any                 
* liability arising out of the application or use of the Software or any              
* product or circuit described in the Software. Cypress does not                      
* authorize its products for use in any products where a malfunction or               
* failure of the Cypress product may reasonably be expected to result in              
* significant property damage, injury or death ( "High Risk Product" ). By            
* including Cypress's product in a High Risk Product, the manufacturer                
* of such system or application assumes all risk of such use and in doing             
* so agrees to indemnify Cypress against all liability.                               
*/
/******************************************************************************/
/** \file main.c
 **
 ** Main Module
 **
 ** \brief This example shows how to use the ADC Priority conversion with
 ** polling mode.
 **
 ** In this example the ADC0 channel 0 is set to priority conversion mode.
 ** The conversion is started by software trigger and polling the converison
 ** result. The result print to terminal I/O at the end.
 **
 ** History:
 **   - 2014-01-15  1.0  RZh        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#define ADC_INPUT_CH0       (0)
#define ADC_UNIT            (ADC0)

/**
 ******************************************************************************
 ** \brief  ADC0 prio2 conversion using software trig.
 **
 ** \return
 **
 ******************************************************************************/
void adcPrio2_polling( void )
{
    stc_adcn_t       *pstcAdc         = (stc_adcn_t *)&ADC_UNIT;
    stc_adc_config_t stcConfig;
    stc_adc_prio_t   stcPrioCfg;

    uint32_t         u32AdcValue      = 0;
    uint32_t         u32AdcChannel    = 0;
    uint8_t          u8AdcCause       = 0;
    uint32_t         u32Data          = 0;
    uint32_t         u32SamplingCount = 10;

    // Clear structure
    PDL_ZERO_STRUCT(stcConfig);
    PDL_ZERO_STRUCT(stcPrioCfg);
    
    // Initialize priority conversion
    stcPrioCfg.bPrioExtTrigStartEnable = FALSE;
    stcPrioCfg.bPrioTimerStartEnable = FALSE;
    stcPrioCfg.enPrioTimerTrigger = AdcNoTimer;
    stcPrioCfg.u8PrioFifoDepth = 0;
    stcPrioCfg.u8PrioLevel1AnalogChSel = 0;
    stcPrioCfg.u8PrioLevel2AnalogChSel = ADC_INPUT_CH0;
    
    // Initialize ADC configure structure
    // Conversion time = Sampling time + Compare time
    // The sampling time should comply with a certain range to guarantee the accuracy (Tmin < Ts < Tmax).
    // For the value of Tmin and Tmax (e.g. 10us), see the product datasheet for detail (12-bit A/D converter of Electrical Characteristics chapter)

///// For FM4 and FM0+, the calculation of sampling time and compare time is shown as following:
    // Sampling time = HCLK cycle * Frequency division ratio * {(ST set value + 1) * STX setting multiplier + 3}
    // At the following configuration:
    // Sampling time = 5ns * 5 * {(8+1)*8+3} = 1.875us (if HCLK = 200MHz)
    // Sampling time = 25ns * 5 * {(8+1)*8+3} = 9.375us (if HCLK = 40MHz)

    // Compare time = Compare clock cycle * 14 = Base clock (HCLK) cycle * Frequency division ratio * 14
    // At following configuration:
    // Compare time = 5ns * 5 * 14 = 350ns (if HCLK = 200MHz)
    // Compare time = 25ns * 5 * 14 = 1750ns (if HCLK = 40MHz)

///// For FM3, the calculation of sampling time and compare time is shown as following:
    // Sampling time = HCLK cycle * {(ST set value + 1) * STX setting multiplier + 3}
    // At the following configuration:
    // Sampling time = 7ns * {(8+1)*8+3} = 0.52us (if HCLK = 144MHz)

    // Compare time = Compare clock cycle * 14 = Base clock (HCLK) cycle * Frequency division ratio * 14
    // At following configuration:
    // Compare time = 7ns * 5 * 14 = 490ns (if HCLK = 144MHz)  
    stcConfig.bLsbAlignment = TRUE;
    stcConfig.u32SamplingTimeSelect.u32AD_CHn = 0;  
    stcConfig.enSamplingTimeN0 = Value8;    // STX setting multiplier 0: 8
    stcConfig.u8SamplingTime0 = 8u;         // ST value 0 : 8
    stcConfig.enSamplingTimeN1 = Value8;    // STX setting multiplier 1: 8  
    stcConfig.u8SamplingTime1 = 8u;         // ST value 1 : 8
    stcConfig.u8ComparingClockDiv = 3u;     // Frequency division ratio: 5, 0:Ratio 2, 1:Ratio 3, ...
    stcConfig.pstcPrioInit = &stcPrioCfg;
   
    if (Ok == Adc_Init(pstcAdc, &stcConfig))        ///< Init ADC0
    {
        Adc_EnableWaitReady(pstcAdc);               ///< Enable ADC0 and wait for ready
    }
    else
    {
        return;
    }

    while (u32SamplingCount-- > 0)
    {
        Adc_SwTriggerPrio(pstcAdc);                 ///< Trigger ADC0

        while (1)
        {
            if (TRUE == Adc_GetIrqFlag(pstcAdc, AdcPrioIrq))
            {
                Adc_ClrIrqFlag(pstcAdc, AdcPrioIrq);
                break;
            }
        }

        u32Data = Adc_ReadPrioFifo(pstcAdc);
        if (0xFFFFFFFF == u32Data ||
            AdcFifoDataValid != Adc_GetPrioDataValid(pstcAdc, u32Data))
        {
#ifdef DEBUG_PRINT
            printf("Invalid Record\n");
#endif
        }
        else
        {
            u32AdcChannel = Adc_GetPrioChannel(pstcAdc, u32Data);
            u32AdcValue = Adc_GetPrioData(pstcAdc, u32Data);
            u8AdcCause = Adc_GetPrioDataCause(pstcAdc, u32Data);

#ifdef DEBUG_PRINT
            printf("Prio2, CH:%d,val:%d,R=%d\n", u32AdcChannel, u32AdcValue, u8AdcCause);
#endif
        }
    }
    
    Adc_DeInit(pstcAdc, TRUE);
}

/**
 ******************************************************************************
 ** \brief  ADC0 prio1 conversion using external trig.
 ******************************************************************************/

void adcPrio1_polling( void )
{
    stc_adcn_t       *pstcAdc         = (stc_adcn_t *)&ADC_UNIT;
    stc_adc_config_t stcConfig;
    stc_adc_prio_t   stcPrioCfg;

    uint32_t         u32AdcValue      = 0;
    uint32_t         u32AdcChannel    = 0;
    uint8_t          u8AdcCause       = 0;
    uint32_t         u32Data          = 0;
    uint32_t         u32SamplingCount = 10;

    // Clear structure
    PDL_ZERO_STRUCT(stcConfig);
    PDL_ZERO_STRUCT(stcPrioCfg);
    
    // Initialize priority conversion structure
    stcPrioCfg.bPrioExtTrigStartEnable = TRUE;
    stcPrioCfg.bPrioTimerStartEnable = FALSE;
    stcPrioCfg.enPrioTimerTrigger = AdcNoTimer;
    stcPrioCfg.u8PrioFifoDepth = 0u;
    stcPrioCfg.u8PrioLevel1AnalogChSel = ADC_INPUT_CH0;
    stcPrioCfg.u8PrioLevel2AnalogChSel = 0u;

    // Common setting
    stcConfig.bLsbAlignment = TRUE;
    stcConfig.u32SamplingTimeSelect.u32AD_CHn = 0u;
    stcConfig.enSamplingTimeN0 = Value4;
    stcConfig.u8SamplingTime0 = 9u;
    stcConfig.enSamplingTimeN1 = Value4;
    stcConfig.u8SamplingTime1 = 9u;
    stcConfig.u8ComparingClockDiv = 3u;
    stcConfig.pstcPrioInit = &stcPrioCfg;
    
    if (Ok == Adc_Init(pstcAdc, &stcConfig))        ///< Init ADC0
    {
        Adc_EnableWaitReady(pstcAdc);               ///< Enable ADC0 and wait for ready
    }
    
    // Select ADC trigger pin
    SetPinFunc_ADTG_0_ADC0();
    
    while (u32SamplingCount-- > 0)
    {

        while (1)
        {
            if (TRUE == Adc_GetIrqFlag(pstcAdc, AdcPrioIrq))
            {
                Adc_ClrIrqFlag(pstcAdc, AdcPrioIrq);
                break;
            }
        }

        u32Data = Adc_ReadPrioFifo(pstcAdc);
        if (0xFFFFFFFF == u32Data ||
            AdcFifoDataValid != Adc_GetPrioDataValid(pstcAdc, u32Data))
        {
#ifdef DEBUG_PRINT
            printf("Invalid Record\n");
#endif
        }
        else
        {
            u32AdcChannel = Adc_GetPrioChannel(pstcAdc, u32Data);
            u32AdcValue = Adc_GetPrioData(pstcAdc, u32Data);
            u8AdcCause = Adc_GetPrioDataCause(pstcAdc, u32Data);
#ifdef DEBUG_PRINT
            printf("Prio1, CH:%d,val:%d,R=%d\n", u32AdcChannel, u32AdcValue, u8AdcCause);
#endif
        }

        // Clear FIFO to give up the data in the FIFO sampled by ADTG dethering
        Adc_ClrPrioFifo(pstcAdc);
        Adc_ClrIrqFlag(pstcAdc, AdcPrioIrq);
    }

    Adc_DeInit(pstcAdc, TRUE);
}

/**
 ******************************************************************************
 ** \brief  Main function
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main( void )
{
    adcPrio2_polling();

    adcPrio1_polling();

    while (1)
    {}
}
