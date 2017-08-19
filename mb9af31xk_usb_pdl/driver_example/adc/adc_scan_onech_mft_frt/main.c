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
 ** \brief This example shows how to trigger the ADC sample by frt.
 **
 ** History:
 **   - 2014-12-30  0.0.1  DHo        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/
/* Only FM3 supports this example */
#if (PDL_MCU_CORE != PDL_FM3_CORE)
#error Only FM3 product supports triggering ADC with FRT!
#endif

/* MFT */
/*! \brief TRUE: Output AD conversion start signal to corresponding ADC uint upon FRT zreo match event */
#define MFT_FRT_TRIGGER_ADC0   TRUE       
#define MFT_FRT_TRIGGER_ADC1   FALSE
#define MFT_FRT_TRIGGER_ADC2   FALSE

/* ADC */
/*! \brief ADC channel */
#define ADC_INPUT_CH_NUM    (0)
#define ADC_INPUT_CH        (1 << ADC_INPUT_CH_NUM)
/*! \brief ADC unit */
#define ADC_UNIT            (ADC0)
/*! \brief ADC sample count */
#define SAMPLE_COUNT        (10)

/* ADCMP */
/*! \brief Couple channel */
#define USER_ADCMP_COUPLE_CH    MFT_ADCMP_CH10


/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
boolean_t       bADCFinished     = FALSE;
static uint32_t mu32SampleRecord[SAMPLE_COUNT] = { 0 };
static uint32_t mu32SamplingCount  = 0;
 
/**
 ******************************************************************************
 ** \brief   Init user free run timer
 **
 ** \param [in]: void
 **
 ** \retval: void
 **
 ******************************************************************************/
static void InitFrt(void)
{
    stc_mft_frt_config_t stcFrtConfig;

    // Clear structure
    PDL_ZERO_STRUCT(stcFrtConfig);
    
    //configure the FRT parameter
    stcFrtConfig.enFrtMode = FrtUpDownCount;
    stcFrtConfig.enFrtClockDiv = FrtPclkDiv64;
    stcFrtConfig.bEnExtClock = FALSE;
    stcFrtConfig.bEnBuffer = FALSE;

    stcFrtConfig.bTriggerAdc0 = MFT_FRT_TRIGGER_ADC0; 
    stcFrtConfig.bTriggerAdc1 = MFT_FRT_TRIGGER_ADC1; 
    stcFrtConfig.bTriggerAdc2 = MFT_FRT_TRIGGER_ADC2;

    Mft_Frt_Stop(&MFT0_FRT, MFT_FRT_CH0);
    //init frt module
    Mft_Frt_Init(&MFT0_FRT, MFT_FRT_CH0, &stcFrtConfig);
    //set frt cycle value
    Mft_Frt_SetCountCycle(&MFT0_FRT, MFT_FRT_CH0, 50000);

    return;
}

/**
 ******************************************************************************
 ** \brief   Initialize ADC scan conversion trigger signal
 **
 ** \param [in]: void
 **
 ** \retval: void
 **
 ******************************************************************************/
static void InitAdcTriggerSignal(void)
{
    stc_mft_adcmp_config_fm3_t stcAdcmpConfigFm3;
    
    // Clear local configuration to zero.
    PDL_ZERO_STRUCT(stcAdcmpConfigFm3);
    
    stcAdcmpConfigFm3.bAdcScanTriggerSel = TRUE;     ///< FALSE: selects the start signal of ADCMP ch.x as ADC unitx scan conversion start signal.
                                                     ///< TRUE: Selects the logic OR signal of FRT ch.0 to ch.2 start signal as ADC unitx scan conversion start signal.
    stcAdcmpConfigFm3.bAdcPrioTriggerSel = FALSE;    ///< FALSE: selects the start signal of ADCMP ch.x as ADC unitx priority conversion start signal.
                                                     ///< TRUE: Selects the logic OR signal of FRT ch.0 to ch.2 start signal as ADC unitx priority conversion start signal.
    if(Ok != Mft_Adcmp_Init_Fm3(&MFT0_ADCMP, USER_ADCMP_COUPLE_CH, &stcAdcmpConfigFm3))
    {
#ifdef DEBUG_PRINT
        printf("InitAdcmpFm3Mode function failed!\n");
#endif
    }

    return;
}

/**
 ******************************************************************************
 ** \brief   ADC scan result callback function
 **
 ** \param  pu32AdcArgument             Pointer to ADC FIFO value
 **
 ** \retval void
 **
 ******************************************************************************/
static void AdcCallback( volatile uint32_t *pu32AdcArgument )
{
    stc_adcn_t        *pstcAdc  = (stc_adcn_t *)&ADC_UNIT;
    uint32_t          u32Data   = 0;

    if(mu32SamplingCount >= SAMPLE_COUNT)
    {
       bADCFinished = TRUE; /* Finish timer trigger */
       u32Data = Adc_ReadScanFifo(pstcAdc);
       Adc_ForceStop(pstcAdc );
       /*Clear FIFO*/  
       Adc_ClrScanFifo(pstcAdc);
       
       return;
    }
    
    u32Data = Adc_ReadScanFifo(pstcAdc);
    mu32SampleRecord[mu32SamplingCount++] = u32Data;

    return;
}

/**
 ******************************************************************************
 ** \brief  Print ADC result data in the terminal window
 **
 ** \param [in] pData       Pointer to print data buffer.
 ** \param [in] u32cnt      data buffer size.
 **
 ** \retval void
 **
 ******************************************************************************/
static void PrintAdcData( uint32_t *pData, uint32_t u32cnt )
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
        u32AdcValue   = Adc_GetPrioData(pstcAdc, u32Data);
        u8AdcCause    = Adc_GetPrioDataCause(pstcAdc, u32Data);

        if (TRUE == Adc_GetScanDataValid(pstcAdc, u32Data))
        {
#ifdef DEBUG_PRINT
            printf("%03d, CH:%d, val:%d, Adc Trigger Mode = %d\n", i, u32AdcChannel, u32AdcValue, u8AdcCause);
#endif
        }
        else
        {
#ifdef DEBUG_PRINT
            printf("%d: Invalid Record!\n", i);
#endif
        }
        i++;
    }

    return;
}

/**
 ******************************************************************************
 ** \brief  ADC0 initialization and single conversion start
 **
 ** \param [in]: void
 **
 ** \retval: void
 **
 ******************************************************************************/
static void InitAdc( void )
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
    stcScanCfg.enScanTimerTrigger = AdcMft;
    stcScanCfg.bScanTimerStartEnable = TRUE;
    stcScanCfg.u8ScanFifoDepth = 0u;
    
    // Initialize interrupt structures
    stcIrqEn.bScanIrq = TRUE;
    stcIrqCallBack.pfnScanIrqCb = AdcCallback;
    
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
    else
    {
#ifdef DEBUG_PRINT
        printf("InitAdc function failed!\n");
#endif
    }

    return;
}

/**
 ******************************************************************************
 ** \brief  Main function
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main( void )
{
    stc_adcn_t *pstcAdc = (stc_adcn_t *)&ADC_UNIT;
    
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Trigger adc scan sample by frt!\n");
    printf("==================================================\n");
#endif

    InitFrt();
    InitAdcTriggerSignal();
    InitAdc();

    Mft_Frt_Start(&MFT0_FRT, MFT_FRT_CH0); 

    while (1)
    {
       if (TRUE == bADCFinished)
       {
           bADCFinished = FALSE;
           break;
       }
    }

    PrintAdcData(mu32SampleRecord, SAMPLE_COUNT);

    Adc_DeInit(pstcAdc, TRUE);
    
    while (1);
}

/*****************************************************************************/
/* END OF FILE */
