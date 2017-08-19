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
 ** interrput enabled.
 **
 ** In this example the ADC0 channel 0 is set to priority conversion mode.
 ** The conversion is started by software trigger and interrupt callback called
 ** when a conversion completion is done. The result is print to terminal I/O
 ** at the end.
 **
 ** History:
 **   - 2014-11-15  1.0  RZh        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#if !defined(SetPinFunc_ADTG_0_ADC0)
#error "ADTG_0 is used as ADC trigger pin, choose other pin if it is not \
available in your MCU product!"
#endif

#define ADC_INPUT_CH0       (0)
#define ADC_UNIT            (ADC0)
#define ADC_SAMPLE_COUNT    (5)

/******************************************************************************/
/* Local variable                                                             */
/******************************************************************************/
static boolean_t bAdcFinished     = FALSE;      ///< Interrupt notification flag
static uint32_t  u32SampleCount   = 0;
static uint32_t  u32ConvertRecord[ADC_SAMPLE_COUNT] = { 0 };

/**
 ******************************************************************************
 ** \brief  ADC priority 2  callback function
 **
 ** \param  pu32AdcArgument         Pointer to ADC FIFO value
 **
 ** \retval
 ******************************************************************************/
void Main_adc_prio2_callback( volatile uint32_t *pu32AdcArgument )
{
    stc_adcn_t *pstcAdc                = (stc_adcn_t *)&ADC_UNIT;
    stc_adc_irq_sel_t stcIrqSel        = { 0 };
		
    u32ConvertRecord[u32SampleCount++] = *pu32AdcArgument;

    if (u32SampleCount == ADC_SAMPLE_COUNT)
    {
        stcIrqSel.bPrioIrq = 1;
        Adc_DisableIrq(pstcAdc, &stcIrqSel);
        Adc_Disable(pstcAdc);           ///< Disable ADC0
    }
	
    bAdcFinished = TRUE;                  ///< Set interrupt notification flag
}

/**
 ******************************************************************************
 ** \brief  ADC priority 1  callback function
 **
 ** \param  pu32AdcArgument         Pointer to ADC FIFO value
 **
 ** \retval
 ******************************************************************************/
void Main_adc_prio1_callback( volatile uint32_t *pu32AdcArgument )
{
    stc_adc_irq_sel_t stcIrqSel        = { 0 };
    stc_adcn_t        *pstcAdc         = (stc_adcn_t *)&ADC_UNIT;
  
    u32ConvertRecord[u32SampleCount++] = *pu32AdcArgument;

    // Clear FIFO to give up the data sampled by ADTG dithering
    Adc_ClrPrioFifo(pstcAdc);
    
    if (u32SampleCount == ADC_SAMPLE_COUNT)
    {
        stcIrqSel.bPrioIrq = 1;
        Adc_DisableIrq(pstcAdc, &stcIrqSel);
        Adc_Disable(pstcAdc);           ///< Disable ADC0
    }
		
    bAdcFinished = TRUE;                  ///< Set interrupt notification flag
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
 ** \brief  ADC0 Prio2 conversion using software trigger.
 **
 ** \return  uint32_t   ADC result
 ******************************************************************************/
void adcPrio2_irq( void )
{
    stc_adcn_t        *pstcAdc         = (stc_adcn_t *)&ADC_UNIT;
    stc_adc_config_t  stcConfig;
    stc_adc_prio_t    stcPrioCfg       = { 0 };
    stc_adc_irq_sel_t stcIrqSel        = { 0 };
    stc_adc_irq_cb_t  stcIrqCallBack   = { 0 };
    uint32_t          u32SamplingCount = ADC_SAMPLE_COUNT;

    PDL_ZERO_STRUCT(stcConfig);
    stcConfig.pstcPrioInit = &stcPrioCfg;
    stcConfig.pstcIrqCb = &stcIrqCallBack;

    // Common setting
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
    
    // Priority converison is not used, so don't initialize it.
    stcPrioCfg.bPrioExtTrigStartEnable = FALSE;
    stcPrioCfg.bPrioTimerStartEnable = FALSE;
    stcPrioCfg.enPrioTimerTrigger = AdcNoTimer;
    stcPrioCfg.u8PrioFifoDepth = 0;
    stcPrioCfg.u8PrioLevel1AnalogChSel = 0;
    stcPrioCfg.u8PrioLevel2AnalogChSel = ADC_INPUT_CH0;

    stcIrqCallBack.pfnPrioIrqCb = Main_adc_prio2_callback;
    stcIrqSel.bPrioIrq = TRUE;

    stcConfig.bTouchNvic = TRUE;

    if (Ok == Adc_Init(pstcAdc, &stcConfig))    ///< Init ADC0
    {
        Adc_EnableIrq(pstcAdc, &stcIrqSel);
        Adc_EnableWaitReady(pstcAdc);           ///< Enable ADC0 and wait for ready
    }

    while (u32SamplingCount-- > 0)
    {
        Adc_SwTriggerPrio(pstcAdc);             ///< Trigger ADC0
        while (FALSE == bAdcFinished);
        bAdcFinished = FALSE;
    }

    PrintAdcData(u32ConvertRecord, ADC_SAMPLE_COUNT);

    Adc_DeInit(pstcAdc, TRUE);
}


/**
 ******************************************************************************
 ** \brief  ADC0 Prio1 conversion using external trigger
 ******************************************************************************/

void adcPrio1_irq( void )
{
    stc_adcn_t        *pstcAdc         = (stc_adcn_t *)&ADC_UNIT;
    stc_adc_config_t  stcConfig;
    stc_adc_prio_t    stcPrioCfg;
    stc_adc_irq_en_t  stcIrqEn;
    stc_adc_irq_cb_t  stcIrqCallBack;

    // if MCU does not have ADTG0 trigger.please change to other ADTGx
    // accroding to MCU datasheet.
    SetPinFunc_ADTG_0_ADC0();
    
    // Clear structure
    PDL_ZERO_STRUCT(stcConfig);
    PDL_ZERO_STRUCT(stcPrioCfg);
    PDL_ZERO_STRUCT(stcIrqEn);
    PDL_ZERO_STRUCT(stcIrqCallBack);
    
    // Configure interrupt structures
    stcIrqEn.bPrioIrq = TRUE;
    stcIrqCallBack.pfnPrioIrqCb = Main_adc_prio1_callback;
        
    // Configure priority structure
    stcPrioCfg.bPrioExtTrigStartEnable = TRUE;
    stcPrioCfg.bPrioTimerStartEnable = FALSE;
    stcPrioCfg.enPrioTimerTrigger = AdcNoTimer;
    stcPrioCfg.u8PrioFifoDepth = 0;
    stcPrioCfg.u8PrioLevel1AnalogChSel = ADC_INPUT_CH0;   ///< select channel
    stcPrioCfg.u8PrioLevel2AnalogChSel = 0;
    
    // Initialize ADC configuration structure
    stcConfig.bLsbAlignment = TRUE;
    stcConfig.u32SamplingTimeSelect.u32AD_CHn = 0;
    stcConfig.enSamplingTimeN0 = Value32;
    stcConfig.u8SamplingTime0 = 30u;
    stcConfig.enSamplingTimeN1 = Value32;
    stcConfig.u8SamplingTime1 = 30u;
    stcConfig.u8ComparingClockDiv = 5u;
    stcConfig.pstcPrioInit = &stcPrioCfg;
    stcConfig.pstcIrqEn = &stcIrqEn;
    stcConfig.pstcIrqCb = &stcIrqCallBack;
    stcConfig.bTouchNvic = TRUE;

    if (Ok == Adc_Init(pstcAdc, &stcConfig))        ///< Init ADC0
    {
        Adc_EnableWaitReady(pstcAdc);               ///< Enable ADC0 and wait for ready
    }

    while (u32SampleCount < ADC_SAMPLE_COUNT);

    PrintAdcData(u32ConvertRecord, ADC_SAMPLE_COUNT);

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

#ifdef DEBUG_PRINT
    printf("ADC Prio Conversion using Software Trigger.\n");
#endif

    adcPrio2_irq();
    bAdcFinished = FALSE;

#ifdef DEBUG_PRINT
    printf("ADC Prio Conversion using External Trigger.\n");
#endif

    u32SampleCount = 0;

    adcPrio1_irq();

    while (1);
}
