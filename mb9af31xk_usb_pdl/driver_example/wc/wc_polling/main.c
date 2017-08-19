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
 ** This example demonstrates watch counter polling .
 ** IO polling wave can be observed at P61 during demonstration process.
 **
 ** History:
 **   - 2014-12-31  0.0.2  Ken xiao     universal PDL version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#if ((SCM_CTL_Val & 0x08) != 0x08)
#error Enable sub clock in system_fmx.h before using this example!
#endif

#if !defined(GPIO1PIN_P61_INIT)
#error P61 is not available in this MCU product, \
change to other re-location pin! Then delete "me" !
#endif

/*---------------------------------------------------------------------------*/
/* local functions prototypes                                                */
/*---------------------------------------------------------------------------*/
static void IoInit(void);
static void IoPolling(void);
static void Delay(void);
static void WcDemoUseMainOsc(void);
static void WcDemoUseSubOsc(void);
#if (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)
static void WcDemoUseMainCr(void);
static void WcDemoUseSubCr(void);
#endif
/*---------------------------------------------------------------------------*/
/* local data                                                                */
/*---------------------------------------------------------------------------*/
static uint16_t m_u16ValidFlag = 0;

/*---------------------------------------------------------------------------*/
/* global functions                                                          */
/*---------------------------------------------------------------------------*/

/*!
 ******************************************************************************
 ** \brief Watch counter polling example code
 **
 ** 1. Demo Watch counter with Main Osc.
 ** 2. Demo Watch counter with Sub Osc.
 ** 3. Demo Watch counter with Main Cr.
 ** 4. Demo Watch counter with Sub Cr.
 ******************************************************************************
 */
int32_t main(void)
{
#ifdef DEBUG_PRINT
//   Uart_Io_Init();
#endif
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Watch Counter Polling Example Program Start \n");
    printf("==================================================\n");
#endif
    WcDemoUseMainOsc();
    WcDemoUseSubOsc();
#if (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)
    WcDemoUseMainCr();
    WcDemoUseSubCr();
#endif
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Watch Counter Polling Example Program End \n");
    printf("==================================================\n");
#endif
    while(1);
}

/*!
 ******************************************************************************
 ** \brief  Initialize the IO (P61) to be tested
 **
 ** \param  none
 **
 ** \return none
 ******************************************************************************
 */
void IoInit(void)
{
    Gpio1pin_InitOut(GPIO1PIN_P61, Gpio1pin_InitVal(1u));
    return;
}

/*!
 ******************************************************************************
 ** \brief  Polling the IO (P61)
 **
 ** \param  none
 **
 ** \return none
 ******************************************************************************
 */
void IoPolling(void)
{
    /* ignore first trigger in order to synchronize with PCLK*/
    if(!m_u16ValidFlag)
    {
        m_u16ValidFlag++;
        return;
    }
    m_u16ValidFlag++;
    Gpio1pin_Put(GPIO1PIN_P61, (m_u16ValidFlag%2));
    return;
}

/*
 ******************************************************************************
 ** \brief Delay requested seconds approximately
 ******************************************************************************
 */
static void Delay(void)
{
    int32_t u8Cnt1,u8Cnt2;

    u8Cnt2 = SystemCoreClock;

    u8Cnt1 = 0;
    while (1)
    {
        u8Cnt1++;
        if (u8Cnt1 > u8Cnt2)
        {
            break;
        }
    }
}
/*!
 ******************************************************************************
 ** \brief Watch counter polling example code(Use Main OSC)
 **
 ** 1. Initialization.
 ** 2. Set counter clock as MainOsc.
 ** 3. Enable interrupt.
 ** 4. Enable count operation and delay some time.
 ** 5. Stop counter.
 ** 6. Disable Counter.
 ******************************************************************************
 */

static void WcDemoUseMainOsc(void)
{
    stc_wc_config_t stcWcConfig;
    stc_wc_pres_clk_t stcWcPresClk;
    uint32_t u32Cnt=0UL;

    // Clear structure
    PDL_ZERO_STRUCT(stcWcConfig);
    PDL_ZERO_STRUCT(stcWcPresClk);
    
    // Initialize GPIO for watch counter demo
    IoInit();
#ifdef DEBUG_PRINT
    printf("1st demo\n");
    printf("Source clock = Main OSC \n");
    m_u16ValidFlag = 0;
#endif
#if (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)    
    stcWcPresClk.enOutputClk = WcPresOutClkArray6;          // Set counter value, 2^20/4000000 = 0.26S, 1 step = 0.26S
#else
    stcWcPresClk.enOutputClk = WcPresOutClkArray1;          // Watch counter prescaler output array0: 2^22/src clock       
#endif
    stcWcPresClk.enInputClk = WcPresInClkMainOsc;           // Select watch counter prescaler source clock as Main OSC 
    Wc_Pres_SelClk((stc_wcn_t*)&WC0, &stcWcPresClk);

    stcWcConfig.enCntClk = WcCntClkWcck0;                   // Select watch counter source clock WCCK0                         
    stcWcConfig.u8ReloadValue = 1;                          // Set watch counter reload value
    Wc_Init((stc_wcn_t*)&WC0, &stcWcConfig);  
    
    Wc_Pres_EnableDiv((stc_wcn_t*)&WC0);  
    
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Start counter, waiting several seconds approximately\n");
#endif
    // Start counter
    Wc_EnableCount((stc_wcn_t*)&WC0);
    do
    {
        if( PdlSet == Wc_GetIrqFlag((stc_wcn_t*)&WC0))
        {
            Wc_ClearIrqFlag((stc_wcn_t*)&WC0);
            IoPolling();
        }
        u32Cnt++;
    } while(u32Cnt <SystemCoreClock/10);
    // Delay some time
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Stop counter, waiting several seconds approximately\n");
    printf("\n");
#endif
    // Stop counter
    Wc_DisableCount((stc_wcn_t*)&WC0);
    // Delay some time
    Delay();
}

/*!
 ******************************************************************************
 ** \brief Watch counter polling example code(Use Sub OSC)
 **
 ** 1. Initialization.
 ** 2. Set counter clock as SubOsc.
 ** 3. Enable interrupt.
 ** 4. Enable count operation and delay some time.
 ** 5. Stop counter.
 ******************************************************************************
 */
static void WcDemoUseSubOsc(void)
{
    stc_wc_config_t stcWcConfig;
    stc_wc_pres_clk_t stcWcPresClk;
    uint32_t u32Cnt=0UL;

    // Clear structure
    PDL_ZERO_STRUCT(stcWcConfig);
    PDL_ZERO_STRUCT(stcWcPresClk);
    
    // Initialize GPIO for watch counter demo
    IoInit();
#ifdef DEBUG_PRINT
    printf("2nd demo\n"); 
    printf("Source clock = Sub OSC \n");
    m_u16ValidFlag = 0;
#endif
    stcWcPresClk.enInputClk = WcPresInClkSubOsc;            // Select watch counter prescaler source clock as Sub OSC 
    stcWcPresClk.enOutputClk = WcPresOutClkArray0;          // Set counter value, 2^15/32768, 1 step = 1S 
    Wc_Pres_SelClk((stc_wcn_t*)&WC0, &stcWcPresClk);
 
    stcWcConfig.enCntClk = WcCntClkWcck3;                   // Select watch counter source clock WCCK3                       
    stcWcConfig.u8ReloadValue = 1;                          // Set watch counter reload value   
    Wc_Init((stc_wcn_t*)&WC0, &stcWcConfig);
    Wc_Pres_EnableDiv((stc_wcn_t*)&WC0);
    
    
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Start counter, waiting several seconds approximately\n");
#endif
    // Start counter
    Wc_EnableCount((stc_wcn_t*)&WC0);
    do
    {
        if( PdlSet == Wc_GetIrqFlag((stc_wcn_t*)&WC0))
        {
            Wc_ClearIrqFlag((stc_wcn_t*)&WC0);
            IoPolling();
        }
        u32Cnt++;
    }         
    while(u32Cnt <SystemCoreClock/5);
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Stop counter, waiting several seconds approximately\n");
    printf("\n");
#endif
    // Stop counter
    Wc_DisableCount((stc_wcn_t*)&WC0);
    // Delay some time
    Delay();
}
/*!
 ******************************************************************************
 ** \brief Watch counter polling example code(Use Main CR)
 **
 ** 1. Initialization.
 ** 2. Set counter clock as MainCr.
 ** 3. Enable interrupt.
 ** 4. Enable count operation and delay some time.
 ** 5. Stop counter.
 ******************************************************************************
 */
#if (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)
static void WcDemoUseMainCr(void)
{
    stc_wc_config_t stcWcConfig;
    stc_wc_pres_clk_t stcWcPresClk;
    uint32_t u32Cnt=0UL;

    // Clear structure
    PDL_ZERO_STRUCT(stcWcConfig);
    PDL_ZERO_STRUCT(stcWcPresClk);
    
    // Initialize GPIO for watch counter demo
    IoInit();
#ifdef DEBUG_PRINT
    printf("3rd demo\n");
    printf("Source clock = High CR \n");
    m_u16ValidFlag = 0;
#endif
    
    stcWcPresClk.enOutputClk = WcPresOutClkArray6;          // Set counter value, 2^20/4000000 = 0.26S, 1 step = 0.26S
    stcWcPresClk.enInputClk = WcPresInClkHighCr;            // Select watch counter prescaler source clock as High CR
    Wc_Pres_SelClk((stc_wcn_t*)&WC0, &stcWcPresClk);
    
    stcWcConfig.enCntClk = WcCntClkWcck0;                   // Select watch counter source clock WCCK0                       
    stcWcConfig.u8ReloadValue = 1;                          // Set watch counter reload value 
    Wc_Init((stc_wcn_t*)&WC0, &stcWcConfig);
    
    Wc_Pres_EnableDiv((stc_wcn_t*)&WC0);
     
    
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Start counter, waiting several seconds approximately\n");
#endif
    // Start counter
    Wc_EnableCount((stc_wcn_t*)&WC0);
    do
    {
        if( PdlSet == Wc_GetIrqFlag((stc_wcn_t*)&WC0))
        {
            Wc_ClearIrqFlag((stc_wcn_t*)&WC0);
            IoPolling();
        }
        u32Cnt++;
    } while(u32Cnt <SystemCoreClock/10);
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Stop counter, waiting several seconds approximately\n");
    printf("\n");
#endif
    // Stop counter
    Wc_DisableCount((stc_wcn_t*)&WC0);
    // Delay some time
    Delay();
}
/*!
 ******************************************************************************
 ** \brief Watch counter polling example code(Use Sub Cr)
 **
 ** 1. Initialization.
 ** 2. Set counter clock as SubCr.
 ** 3. Enable interrupt.
 ** 4. Enable count operation and delay some time.
 ** 5. Stop counter.
 ******************************************************************************
 */
static void WcDemoUseSubCr(void)
{
    stc_wc_config_t stcWcConfig;
    stc_wc_pres_clk_t stcWcPresClk;
    uint32_t u32Cnt=0UL;

    // Clear structure
    PDL_ZERO_STRUCT(stcWcConfig);
    PDL_ZERO_STRUCT(stcWcPresClk);
    
    // Initialize GPIO for watch counter demo
    IoInit();
#ifdef DEBUG_PRINT
    printf("4th demo\n");
    printf("Source clock = Low CR \n");
    m_u16ValidFlag = 0;
#endif
    
    stcWcPresClk.enOutputClk = WcPresOutClkArray4;          // Set counter value, 2^10/100000, 1 step = 10.24mS
    stcWcPresClk.enInputClk = WcPresInClkLowCr;             // Select watch counter prescaler source clock as Low CR
    Wc_Pres_SelClk((stc_wcn_t*)&WC0, &stcWcPresClk);
    
    stcWcConfig.enCntClk = WcCntClkWcck1;                   // Select watch counter source clock WCCK0                     
    stcWcConfig.u8ReloadValue = 1;                          // Set watch counter reload value  
    Wc_Init((stc_wcn_t*)&WC0, &stcWcConfig);
    
    Wc_Pres_EnableDiv((stc_wcn_t*)&WC0);
     

#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Start counter, waiting several seconds approximately\n");
#endif
    // Start counter
    Wc_EnableCount((stc_wcn_t*)&WC0);
    do
    {
        if( PdlSet == Wc_GetIrqFlag((stc_wcn_t*)&WC0))
        {
            Wc_ClearIrqFlag((stc_wcn_t*)&WC0);
            IoPolling();
        }
        u32Cnt++;
    } while(u32Cnt <SystemCoreClock/10);
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Stop counter, waiting several seconds approximately\n");
    printf("\n");
#endif
    // Stop counter
    Wc_DisableCount((stc_wcn_t*)&WC0);
    // Delay some time
    Delay();
}
#endif //#if (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)
/*****************************************************************************/
/* END OF FILE */
