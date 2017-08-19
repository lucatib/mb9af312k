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
 ** Main Module for Software watchdog sample (un-use window mode)
 **
 ** History:
**   - 2014-12-15  0.0.1  EZh          First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#if (PDL_MCU_CORE != PDL_FM4_CORE) && (PDL_MCU_CORE != PDL_FM0P_CORE)
#error "Only FM4 and FM0+ products supports windows function of software watchdog!"
#endif
/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static volatile uint8_t u8SwwdtActive;

// Table to store status of window watchdog error
static volatile uint8_t au8ErrorStatus[2] = {0, 0};

// Timing table for reloading to watchdog timer
// Please use these values when debugging performs by flash and optimization is not used.
static const volatile uint32_t au32CountValue[2]  = {
#if (__HCLK == 160000000)  
  #if defined (__ICCARM__)
    2285712,    // Interruption will occur
    22857139    // Interruption will not occur
  #else
    199998,    // Interruption will occur
    31999998    // Interruption will not occur 
  #endif
#elif (__HCLK == 40000000)
#if defined (__ICCARM__)
    5757129,   // Interruption will occur
    8157139    // Interruption will not occur
#else
    2099980,   // Interruption will occur
   10099998    // Interruption will not occur
#endif
#else
#error "This example is related with MCU core clcok. Understand the principle \
of this example, then establish the au32CountValue array by yourself! "          
#endif          
};

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/
static void WdgSwCallback(void)
{
    // Stop software watchdog (*)
    Swwdg_Stop();
  
    // Clear interrupt
    Swwdg_Feed();
	
    // Software watchdog in-acitve
    u8SwwdtActive = FALSE;

    // (*)Software watchdog has to be stopped since interruption occurs,
    //    when interruption cause was cleared and watchdog counter was reloaded.
}
/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    volatile uint32_t u32Count;
    volatile uint8_t  u8Index;    
    stc_swwdg_config_t stcSwwdgConfig;
    
    PDL_ZERO_STRUCT(stcSwwdgConfig);
    u8SwwdtActive = FALSE;
    // Initialize Software watchdog
    stcSwwdgConfig.u32LoadValue = 20000000;     ///< Timer interval   
    stcSwwdgConfig.bResetEnable = TRUE;         ///< Enables SW watchdog reset
    stcSwwdgConfig.bWinWdgEnable = TRUE;       ///< Disables Window watchdog mode  
    stcSwwdgConfig.bWinWdgResetEnable = FALSE;
    stcSwwdgConfig.u8TimingWindow = en_swwdg_timing_window_50;
    stcSwwdgConfig.pfnSwwdgIrqCb = WdgSwCallback;
    
    u8Index = 0;
    if (Ok != Swwdg_Init((stc_swwdg_config_t *)&stcSwwdgConfig))
    {
#ifdef DEBUG_PRINT
        printf("Initial error!\n");
#endif
        while (1);
    }

    u8Index = 0;
    while (2 > u8Index)
    {
        // Restart software watchdog
        Swwdg_Start();
        u8SwwdtActive = TRUE;
        // Adjust the timing for reloading watchdog counter
        u32Count = au32CountValue[u8Index];
        while (0 != (u32Count--))
        {
            continue;
        }
        __disable_irq();
        // Clear interrupt and reload watchdog counter
        Swwdg_Feed();
        __enable_irq();
        // Insert cycle for interrupt
        PDL_WAIT_LOOP_HOOK();
        // If watchdog is in-active, error is occured
        if (FALSE == u8SwwdtActive)
        {
            // Error status set
            au8ErrorStatus[u8Index] = 1;
        }
        u8Index++;
        __disable_irq();
        // If watchdog is active...
        if (TRUE == u8SwwdtActive)
        {
            // Stop software watchdog and clear interrupt
            WdgSwCallback();
        }
        __enable_irq();
    }

    // wait for interrupts
    while(1);
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
