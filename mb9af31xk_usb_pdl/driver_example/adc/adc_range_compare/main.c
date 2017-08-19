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
 ** \brief This example shows how to use the ADC range comparison with interrupts.
 **
 ** In this example the ADC0 channel 0 is set to scan conversion mode with
 ** repeat mode. The conversion is started by software trigger and
 ** the result compare callback is called when the ADC conversion result meet
 ** the settings. (input voltage is within the setting voltage.)
 ** The result print to terminal I/O at the end.
 **
 **
 ** History:
 **   - 2014-01-14  1.0  RZh        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#if (PDL_MCU_CORE != PDL_FM0P_CORE) && (PDL_MCU_CORE != PDL_FM4_CORE)
#error "The ADC range compare funciton is not available in this product!"
#endif

#define ADC_INPUT_NUM       (1)
#define ADC_INPUT_CH0       (0)
#define ADC_INPUT_CH        (1<<ADC_INPUT_CH0)
#define ADC_UNIT            (ADC0)

#define ADC_CMP_LOW         (1000)
#define ADC_CMP_HIGH        (3000)

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
static boolean_t bAdcComparedFinish = FALSE;    ///< Interrupt notification flag

/**
 ******************************************************************************
 ** \brief  ADC compare interrupt callback function
 ******************************************************************************/
void Main_adc_compare_callback( void )
{
    bAdcComparedFinish = TRUE;
}
/**
 ******************************************************************************
 ** \brief  ADC0 initialization and single conversion start
 ******************************************************************************/
void Main_adc_compare( void )
{
    stc_adcn_t              *pstcAdc       = (stc_adcn_t *)&ADC_UNIT;
    stc_adc_config_t        stcConfig;
    stc_adc_scan_t          stcScanCfg;
    stc_adc_range_compare_t stcRangeCmpCfg;
    stc_adc_irq_en_t        stcIrqEn;
    stc_adc_irq_cb_t        stcIrqCallBack;

    PDL_ZERO_STRUCT(stcConfig);
    PDL_ZERO_STRUCT(stcScanCfg);
    PDL_ZERO_STRUCT(stcRangeCmpCfg);
    PDL_ZERO_STRUCT(stcIrqEn);
    PDL_ZERO_STRUCT(stcIrqCallBack);

    // Initialize interrupt structures
    stcIrqCallBack.pfnRangeComparisonIrqCb = Main_adc_compare_callback;
    stcIrqEn.bRangeComparisonIrq = TRUE;

    // Scan conversion configuration
    stcScanCfg.u32ScanCannelSelect.u32AD_CHn = ADC_INPUT_CH;
    stcScanCfg.enScanMode = ScanRepeatConversion;
    stcScanCfg.enScanTimerTrigger = AdcNoTimer;
    stcScanCfg.bScanTimerStartEnable = FALSE;
    stcScanCfg.u8ScanFifoDepth = ADC_INPUT_NUM - 1;
    
    // Range comapre funciton confuiguraiton
    stcRangeCmpCfg.bRangeCompareAllChannels = FALSE;
    stcRangeCmpCfg.bWithinRange = TRUE;
    stcRangeCmpCfg.u16LowerLimitRangeValue = ADC_CMP_LOW;
    stcRangeCmpCfg.u16UpperLimitRangeValue = ADC_CMP_HIGH;
    stcRangeCmpCfg.u8RangeComapreChannel = ADC_INPUT_CH0;
    stcRangeCmpCfg.u8RangeCountValue = 1;

    // ADC structure configuration
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
    stcConfig.pstcRangeComparisonInit = &stcRangeCmpCfg;
    stcConfig.pstcIrqEn = &stcIrqEn;
    stcConfig.pstcIrqCb = &stcIrqCallBack;
    stcConfig.bTouchNvic = TRUE;

    if (Ok == Adc_Init(pstcAdc, &stcConfig))    ///< Init ADC0
    {
        Adc_EnableWaitReady(pstcAdc);           ///< Enable ADC0 and wait for ready
        Adc_SwTriggerScan(pstcAdc);
    }

    // Adjust the resistance to trig the interrupt.
    while (FALSE == bAdcComparedFinish);        ///< Wait for interrupt notification

#ifdef DEBUG_PRINT
    printf("Adc range compared done\n");
#endif

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
    Main_adc_compare();
    while (1);
}


