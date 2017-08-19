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
 ** \brief This example shows how to use the system clock initialization by
 ** PDL functions. For this the definition CLOCK_SETUP must be set to
 ** CLOCK_SETTING_NONE in system_fmx.h.
 ** First the reset cause will be determined, judging if there is a system
 ** clock setup needed or not.
 ** Afterwards the main, pll clock is setup with user configuration value.
 **
 ** History:
 **   - 2014-02-15  1.0  EZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/**
 ******************************************************************************
 ** \brief  PDL clock initialization
 ******************************************************************************/
void Main_ClkInit(void)
{
  static stc_reset_result_t stcResetCause;
  stc_clk_config_t          stcClockConfig;

  Reset_GetCause(&stcResetCause);

    do
    {
        if (TRUE == stcResetCause.bSoftware)
        {
            // Place code here
        }

        if (TRUE == stcResetCause.bAnomalousFrequency)
        {
            // Place code here
            break;
        }

        if (TRUE == stcResetCause.bHardwareWatchdog)
        {
            // Place code here
            break;
        }

        if (TRUE == stcResetCause.bSoftwareWatchdog)
        {
            // Place code here
            break;
        }

        if (TRUE == stcResetCause.bInitx)
        {
            // Place code here
        }

        if (TRUE == stcResetCause.bPowerOn)
        {
            // Place code here
        }

        PDL_ZERO_STRUCT(stcClockConfig);

        stcClockConfig.enBaseClkDiv     = BaseClkDiv1;
        stcClockConfig.enAPB0Div        = Apb0Div1;
        stcClockConfig.enAPB1Div        = Apb1Div1;
        stcClockConfig.bAPB1Disable     = FALSE;
        stcClockConfig.enMCOWaitTime    = McoWaitExp117;
        stcClockConfig.enSCOWaitTime    = ScoWaitExp10;
        stcClockConfig.enPLLOWaitTime   = PlloWaitExp19;
        // PLLCLK = Main Osc * (PLLN / PLLK), please refer to Clock chapter in peripehral manual for K, N, M value
        stcClockConfig.u8PllK           = 1;
        stcClockConfig.u8PllN           = 10;
        stcClockConfig.u8PllM           = 2;

        /* Initialize clock */
        if(Ok != Clk_Init(&stcClockConfig))
        {
            while(1);
        }

        Clk_EnableMainClock(TRUE);                          // ... now enable Main oscillator
        Clk_EnablePllClock(TRUE);                           // ... now enable PLL oscillator

        Clk_SetSource(ClkPll);                              // ... and transit to Main Clock

    }while(0);
}

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
  Main_ClkInit();

  while(1)
  {}
}
