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
 ** \brief This example shows how to trigger the ADC sample by the ADC start compare unit.
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
/*! \brief User ADC compare unit to trigger ADC scan */
#define USER_ADCMP_SEL_SOURCE   (AdcmpTrigAdc0Scan) 

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
/*! \brief User ADC compare channel 0 */
#define USER_ADCMP_CH_X                 MFT_ADCMP_CH0
/*! \brief User ADC compare channel 1 */
#define USER_ADCMP_CH_Y                 MFT_ADCMP_CH1

/* OCU */
/*! \brief User ADC compare unit */
#define USER_MFT_OCU_CH           MFT0_OCU
/*! \brief User OCU channel 0 */
#define USER_OCU_CH               USER_ADCMP_CH_X
#endif

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
boolean_t       m_bADCFinished     = FALSE;
static uint32_t m_au32SampleRecord[SAMPLE_COUNT] = { 0 };
static uint32_t m_u32SamplingCount  = 0;

/**
 ******************************************************************************
 ** \brief  Init user free run timer   .
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

    Mft_Frt_Stop(&MFT0_FRT, MFT_FRT_CH0);
    //init frt module
    Mft_Frt_Init(&MFT0_FRT, MFT_FRT_CH0, &stcFrtConfig);
    //set frt cycle value
    Mft_Frt_SetCountCycle(&MFT0_FRT, MFT_FRT_CH0, 50000);
    
    return;
}

/**
 ******************************************************************************
 ** \brief  Initialize ADC start compare unit with FM3 mode
 **
 ** \param [in]: void
 **
 ** \retval: void
 **
 ******************************************************************************/
static void InitAdcmpFm3Mode(void)
{
    stc_mft_adcmp_config_fm3_t stcAdcmpConfigFm3;
    
    // Clear local configuration to zero.
    PDL_ZERO_STRUCT(stcAdcmpConfigFm3);
    
    stcAdcmpConfigFm3.enFrt = (en_adcmp_frt_fm3_t)MFT_FRT_CH0;
    stcAdcmpConfigFm3.enBuf = AdcmpBufFrtZero;
    stcAdcmpConfigFm3.enMode = AdcmpAccpUpAccpdnDownFm3;
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)      
    stcAdcmpConfigFm3.enTrigSel = USER_ADCMP_SEL_SOURCE;
#else
    stcAdcmpConfigFm3.bAdcScanTriggerSel = FALSE;     ///< FALSE: selects the start signal of ADCMP ch.x as ADC unitx scan conversion start signal.
                                          ///< TRUE: Selects the logic OR signal of FRT ch.0 to ch.2 start signal as ADC unitx scan conversion start signal.
    stcAdcmpConfigFm3.bAdcPrioTriggerSel = FALSE;     ///< FALSE: selects the start signal of ADCMP ch.x as ADC unitx priority conversion start signal.
                                          ///< TRUE: Selects the logic OR signal of FRT ch.0 to ch.2 start signal as ADC unitx priority conversion start signal.
#endif 
    if(Ok == Mft_Adcmp_Init_Fm3(&MFT0_ADCMP, USER_ADCMP_COUPLE_CH, &stcAdcmpConfigFm3))
    {
        Mft_Adcmp_WriteAccp_Fm3(&MFT0_ADCMP, USER_ADCMP_COUPLE_CH, 20000);
        Mft_Adcmp_WriteAccpdn_Fm3(&MFT0_ADCMP, USER_ADCMP_COUPLE_CH, 40000);
        Mft_Adcmp_EnableOperation_Fm3(&MFT0_ADCMP, USER_ADCMP_COUPLE_CH);
    }
    else
    {
#ifdef DEBUG_PRINT
        printf("InitAdcmpFm3Mode function failed!\n");
#endif
    }

    return;
}

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
/**
 ******************************************************************************
 ** \brief Initialize ADC start compare unit with normal mode
 **
 ** \param [in]: void
 **
 ** \retval: void
 **
 ******************************************************************************/
void InitAdcmpNormalMode(void)
{
    stc_mft_adcmp_config_t stcAdcmpConfig;
    stc_mft_adcmp_func_t stcAdcmpFunc0, stcAdcmpFunc1;
      
    PDL_ZERO_STRUCT(stcAdcmpConfig);      // Clear local configuration to zero.
    PDL_ZERO_STRUCT(stcAdcmpFunc0);
    PDL_ZERO_STRUCT(stcAdcmpFunc1);
    
    stcAdcmpConfig.enFrt = (en_adcmp_frt_t)MFT_FRT_CH0;
    stcAdcmpConfig.enMode = AdcmpNormalMode;
    stcAdcmpConfig.enBuf = AdcmpBufFrtZero;  
    stcAdcmpConfig.enTrigSel = USER_ADCMP_SEL_SOURCE;
    stcAdcmpFunc0.bZeroEn = TRUE;
    stcAdcmpFunc0.bUpEn = TRUE;
    stcAdcmpFunc0.bPeakEn = FALSE;
    stcAdcmpFunc0.bDownEn = FALSE;
    stcAdcmpFunc1.bZeroEn = FALSE;
    stcAdcmpFunc1.bUpEn = FALSE;
    stcAdcmpFunc1.bPeakEn = TRUE;
    stcAdcmpFunc1.bDownEn = TRUE;
        
    if( Ok == Mft_Adcmp_Init(&MFT0_ADCMP, USER_ADCMP_CH_X, &stcAdcmpConfig))
    {
        // (1/(40M/64) * 20000 = 32ms (first trigger)
        Mft_Adcmp_WriteAcmp(&MFT0_ADCMP, USER_ADCMP_CH_X, 20000); 
        Mft_Adcmp_EnableOperation(&MFT0_ADCMP, USER_ADCMP_CH_X, &stcAdcmpFunc0);
    }
    
    if( Ok == Mft_Adcmp_Init(&MFT0_ADCMP , USER_ADCMP_CH_Y, &stcAdcmpConfig))
    {
        // (1/(40M/64) * (60000 + (60000-40000)) = 128ms (second trigger)
        Mft_Adcmp_WriteAcmp(&MFT0_ADCMP, USER_ADCMP_CH_Y, 40000);
        Mft_Adcmp_EnableOperation(&MFT0_ADCMP, USER_ADCMP_CH_Y, &stcAdcmpFunc1);
    }

    return;
}

/**
 ******************************************************************************
 ** \brief  Initialize ADC start compare unit with offset mode
 **
 ** \param [in]: void
 **
 ** \retval: void
 **
 ******************************************************************************/

void InitAdcmpOffsetMode(void)
{
    stc_mft_adcmp_config_t stcAdcmpConfig;
    stc_mft_adcmp_func_t stcAdcmpFunc0, stcAdcmpFunc1;
    stc_mft_ocu_config_t stcOcuConfig;
     
    // Clear local configuration to zero.
    PDL_ZERO_STRUCT(stcAdcmpConfig);
    PDL_ZERO_STRUCT(stcAdcmpFunc0);
    PDL_ZERO_STRUCT(stcAdcmpFunc1);
    PDL_ZERO_STRUCT(stcOcuConfig);
    
    stcAdcmpConfig.enFrt = (en_adcmp_frt_t)MFT_FRT_CH0;
    stcAdcmpConfig.enMode = AdcmpOffsetMode;
    stcAdcmpConfig.enBuf = AdcmpBufDisable;  
    stcAdcmpConfig.enTrigSel = USER_ADCMP_SEL_SOURCE;
    stcAdcmpConfig.enOccpSel = AdcmpSelOccp0;
    
    stcAdcmpFunc0.bZeroEn = TRUE;
    stcAdcmpFunc0.bUpEn = TRUE;
    stcAdcmpFunc0.bPeakEn = FALSE;
    stcAdcmpFunc0.bDownEn = FALSE;
    
    stcAdcmpFunc1.bZeroEn = TRUE;
    stcAdcmpFunc1.bUpEn = TRUE;
    stcAdcmpFunc1.bPeakEn = FALSE;
    stcAdcmpFunc1.bDownEn = FALSE;
       
    stcOcuConfig.bFm4 = TRUE;
    stcOcuConfig.enFrtConnect = (en_mft_ocu_frt_t)MFT_FRT_CH0;
    
    // Count from 10000
    if(Ok == Mft_Ocu_Init(&MFT0_OCU, USER_OCU_CH, &stcOcuConfig))
    {
        Mft_Ocu_WriteOccp(&MFT0_OCU, USER_OCU_CH, 10000);
    }
    
    if( Ok == Mft_Adcmp_Init(&MFT0_ADCMP ,USER_ADCMP_CH_X, &stcAdcmpConfig))
    {
        // (1/(40M/64) * (10000+10000) = 32ms (first trigger)
        Mft_Adcmp_WriteAcmp(&MFT0_ADCMP, USER_ADCMP_CH_X, 10000); 
        Mft_Adcmp_EnableOperation(&MFT0_ADCMP, USER_ADCMP_CH_X, &stcAdcmpFunc0);
    }   
    if( Ok == Mft_Adcmp_Init(&MFT0_ADCMP ,USER_ADCMP_CH_Y, &stcAdcmpConfig))
    {
        // (1/(40M/64) * (30000+10000) = 64ms (second trigger)
        Mft_Adcmp_WriteAcmp(&MFT0_ADCMP, USER_ADCMP_CH_Y, 30000);
        Mft_Adcmp_EnableOperation(&MFT0_ADCMP, USER_ADCMP_CH_Y, &stcAdcmpFunc1);
    } 

    return;
}
#endif

/**
 ******************************************************************************
 ** \brief   Disable ADCMP
 **
 ** \param  void
 **
 ** \retval void
 **
 ******************************************************************************/
void DeInitAdcmp(void)
{
    stc_mft_adcmp_func_t stcAdcmpFunc0;
    PDL_ZERO_STRUCT(stcAdcmpFunc0);

    stcAdcmpFunc0.bZeroEn = TRUE;
    stcAdcmpFunc0.bUpEn = TRUE;
    stcAdcmpFunc0.bPeakEn = TRUE;
    stcAdcmpFunc0.bDownEn = TRUE;
    
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
    Mft_Adcmp_DisableOperation(&MFT0_ADCMP, USER_ADCMP_CH_X, &stcAdcmpFunc0);
    Mft_Adcmp_DisableOperation(&MFT0_ADCMP, USER_ADCMP_CH_X, &stcAdcmpFunc0);
#else       
    Mft_Adcmp_DisableOperation_Fm3(&MFT0_ADCMP, USER_ADCMP_COUPLE_CH);
#endif    
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

    if(m_u32SamplingCount >= SAMPLE_COUNT)
    {
       m_bADCFinished = TRUE; /* Finish timer trigger */
       m_u32SamplingCount = 0;
       
       Mft_Frt_Stop(&MFT0_FRT, MFT_FRT_CH0);
       Adc_ForceStop(pstcAdc );
      
       /*Clear FIFO*/  
       Adc_ClrScanFifo(pstcAdc);
       
       return;
    }
    
    u32Data = *pu32AdcArgument; /* Read FIFO */
    m_au32SampleRecord[m_u32SamplingCount++] = u32Data;

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

#if (PDL_ADC_TYPE == PDL_ADC_B)    
    stcConfig.u8EnableTime = 0x80;                   ///< Set enable Time
#endif    
    
    stcConfig.pstcScanInit = &stcScanCfg;
    stcConfig.pstcPrioInit = NULL;
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
    
    InitFrt();
    InitAdc();
    
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("          Trigger adc scan sample by adcmp !\n");
    printf("==================================================\n\n\n");
#endif

    /* 1. Demo FM3 compatible mode of adc start compare unit */
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("          FM3 compatible mode of ADC start compare unit \n");
    printf("==================================================\n");
#endif 

    InitAdcmpFm3Mode();
    Mft_Frt_Start(&MFT0_FRT, MFT_FRT_CH0); 
    while (1)
    {
       if (TRUE == m_bADCFinished)
       {
           m_bADCFinished = FALSE;
           PrintAdcData(m_au32SampleRecord, SAMPLE_COUNT);           
           pdl_memclr((uint8_t*)m_au32SampleRecord, SAMPLE_COUNT);
           break;
       }
    }
    DeInitAdcmp();

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)      
    /* 2. Demo normal mode of adc start compare unit */
    #ifdef DEBUG_PRINT
    printf("\n\n==================================================\n");
    printf("          Normal mode of ADC start compare unit \n");
    printf("==================================================\n");
    #endif
    
    InitAdcmpNormalMode();
    Mft_Frt_Start(&MFT0_FRT, MFT_FRT_CH0); 
    while(1)
    {
       if(TRUE == m_bADCFinished)
       {
           m_bADCFinished = FALSE;
           PrintAdcData(m_au32SampleRecord, SAMPLE_COUNT);
           pdl_memclr((uint8_t*)m_au32SampleRecord, SAMPLE_COUNT);
           break;
       }
    }
    DeInitAdcmp();

    /* 3. Demo offset mode of adc start compare unit */
#ifdef DEBUG_PRINT
    printf("\n\n==================================================\n");
    printf("          Offset mode of ADC start compare unit \n");
    printf("==================================================\n");
#endif    
    InitAdcmpOffsetMode();
    Mft_Frt_Start(&MFT0_FRT, MFT_FRT_CH0); 
    while(1)
    {
        if(TRUE == m_bADCFinished)
        {
            m_bADCFinished = FALSE;
            PrintAdcData(m_au32SampleRecord, SAMPLE_COUNT);            
            pdl_memclr((uint8_t*)m_au32SampleRecord, SAMPLE_COUNT);
            break;
        }
    }
    
    DeInitAdcmp();
#endif

    Adc_DeInit(pstcAdc, TRUE);
    
    while (1);
}

/*****************************************************************************/
/* END OF FILE */
