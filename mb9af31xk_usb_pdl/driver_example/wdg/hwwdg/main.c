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
 ** Main Module for Hardware watchdog sample
 **
 ** History:
 **   - 2014-11-21  0.0.1  EZh         First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#if !defined(GPIO1PIN_P61_INIT)
#error P61 is not available in this MCU product, \
change to other re-location pin! Then delete "me" !
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
static uint32_t u32CountWdg = 0;

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/
/**
 ******************************************************************************
 ** \brief Initializatio GPIO
 **
 ******************************************************************************/
static void PortInit(void)
{
   Gpio1pin_InitOut(GPIO1PIN_P61, Gpio1pin_InitVal(1u));
}

/**
 ******************************************************************************
 ** \brief Set port for LED
 **
 ******************************************************************************/
static void SetLed(uint32_t u32Led)
{
    if (0 == (u32Led & 0x00000001))
    {
        Gpio1pin_Put(GPIO1PIN_P61, 0u);
    }
    else
    {
        Gpio1pin_Put(GPIO1PIN_P61, 1u);
    }
}

/**
 ******************************************************************************
 ** \brief  Hardware watchdog interrupt function
 **
 ******************************************************************************/
static void WdgHwCallback(void)
{
    // comment following to demonstrate the hardware watchdog reset
    Hwwdg_Feed(0x55, 0xAA);   // Clear Irq and Reset Timer
    ++u32CountWdg;
    SetLed(u32CountWdg);
}

/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    stc_hwwdg_config_t stcHwwdgConfig;
    
    // Clear structure
    PDL_ZERO_STRUCT(stcHwwdgConfig);
    
    // Initialize GPIO for LED
    PortInit();
    
    // Initialize structure
    stcHwwdgConfig.u32LoadValue = 100000;   // Interval:1s (@CLKLC:100kHz)
    stcHwwdgConfig.bResetEnable = TRUE;     // Enables Hardware watchdog reset
    stcHwwdgConfig.pfnHwwdgIrqCb = WdgHwCallback;
    
    // Initialize hardware watchdog
    if (Ok != Hwwdg_Init(&stcHwwdgConfig))
    {
#ifdef DEBUG_PRINT
        printf("Initial error!\n");
#endif
    }
    else
    {
        // Start hardware watchdog
        Hwwdg_Start();
    }

    // wait for interrupts
    while(1);
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
