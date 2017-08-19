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
 ** \brief This example shows how to use the ADC Conversion with interrupt.
 **
 ** This example set the ADC0 channel 0 in single conversion mode with
 ** interrupt enabled.
 ** Via software trigger the conversion is started. When a ADC conversion is
 ** completed, the interrupt callback function is called, it read the result
 ** data and print to terminal I/O.
 **
 ** History:
 **   - 2014-12-08  1.0  RZh        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#define ADC_INPUT_CH0       (0)
#define ADC_INPUT_CH        (1<<ADC_INPUT_CH0)
#define ADC_UNIT            (ADC0)

#define SAMPLE_COUNT        (10)

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
boolean_t       bADCFinished     = FALSE;
static uint32_t mu32SampleRecord[SAMPLE_COUNT] = { 0 };
static uint32_t mu32SamplingCount   = 0;

/**
 ******************************************************************************
 ** \brief  ADC scan Result callback function
 **
 ** \param  pu32AdcArgument             Pointer to ADC FIFO value
 **
 ** \retval void
 **
 ******************************************************************************/
void Main_adc_result_callback( volatile uint32_t *pu32AdcArgument )
{
    stc_adcn_t        *pstcAdc  = (stc_adcn_t *)&ADC_UNIT;
    uint32_t          u32Data   = 0;

    u32Data = Adc_ReadScanFifo(pstcAdc);
    mu32SampleRecord[mu32SamplingCount++] = u32Data;

    bADCFinished = TRUE;
}


/**
 ******************************************************************************
 ** \brief  Print ADC result data in the terminal window
 **
 ** \param [in] pData       Pointer to print data buffer.
 ** \param [in] u32cnt      data buffer size.
 **
 ** \retval
 **
 ******************************************************************************/
void PrintAdcData( uint32_t *pData, uint32_t u32cnt )
{
    stc_adcn_t *pstcAdc      = (stc_adcn_t *)&ADC_UNIT;
    uint16_t   i             = 0;
    uint32_t   u32AdcValue   = 0;
    uint32_t   u32AdcChannel = 0;
    uint8_t    u8AdcCause    = 0;
    uint32_t   u32Data       = 0;
    uint32_t   u32Times      = u32cnt;

    while (i < u32Times)
    {
        u32Data = *(pData + i);

        u32AdcChannel = Adc_GetPrioChannel(pstcAdc, u32Data);
        u32AdcValue = Adc_GetPrioData(pstcAdc, u32Data);
        u8AdcCause = Adc_GetPrioDataCause(pstcAdc, u32Data);

        if (TRUE == Adc_GetScanDataValid(pstcAdc, u32Data))
        {
#ifdef DEBUG_PRINT
            printf("%03d, CH:%d,val:%d,R=%d\n", i, u32AdcChannel, u32AdcValue, u8AdcCause);
#endif
        }
        else
        {
#ifdef DEBUG_PRINT
            printf("%d: Invalid Record\n", i);
#endif
        }
        i++;
    }
}


/**
 ******************************************************************************
 ** \brief  ADC0 initialization and single conversion start
 ******************************************************************************/
void Main_adc_irq( void )
{
    stc_adcn_t        *pstcAdc         = (stc_adcn_t *)&ADC_UNIT;
    stc_adc_config_t  stcConfig;
    stc_adc_scan_t    stcScanCfg;
    stc_adc_irq_en_t  stcIrqEn;
    stc_adc_irq_cb_t  stcIrqCallBack;

    // Clear structure
    PDL_ZERO_STRUCT(stcConfig);
    PDL_ZERO_STRUCT(stcScanCfg);
    PDL_ZERO_STRUCT(stcIrqEn);
    PDL_ZERO_STRUCT(stcIrqCallBack);
    
    // Scan conversion configuration
    stcScanCfg.u32ScanCannelSelect.u32AD_CHn = ADC_INPUT_CH;
    stcScanCfg.enScanMode = ScanSingleConversion;
    stcScanCfg.enScanTimerTrigger = AdcNoTimer;
    stcScanCfg.bScanTimerStartEnable = FALSE;
    stcScanCfg.u8ScanFifoDepth = 0u;
    
    // Initialize interrupt structures
    stcIrqEn.bScanIrq = TRUE;
    stcIrqCallBack.pfnScanIrqCb = Main_adc_result_callback;
    
    // ADC configuration
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
    stcConfig.pstcScanInit = &stcScanCfg;
    stcConfig.pstcIrqEn = &stcIrqEn;
    stcConfig.pstcIrqCb = &stcIrqCallBack;
    stcConfig.bTouchNvic = TRUE;

    if (Ok == Adc_Init(pstcAdc, &stcConfig))    ///< Init ADC0
    {
        Adc_EnableWaitReady(pstcAdc);           ///< Enable ADC0 and wait for ready
    }

    while (1)
    {
        Adc_SwTriggerScan(pstcAdc);             ///< Trigger ADC0
        while (FALSE == bADCFinished);
        bADCFinished = FALSE;

        if (mu32SamplingCount >= SAMPLE_COUNT)
        {
            break;
        }
    }

    PrintAdcData(mu32SampleRecord, SAMPLE_COUNT);

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
    Main_adc_irq();

    while (1)
    {}

}
