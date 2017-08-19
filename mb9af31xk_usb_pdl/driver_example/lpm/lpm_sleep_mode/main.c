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
 ** Main Module for low power consumption mode sample
 **
 ** History:
 **   - 2014-02-09  0.0.1  Edison Zhang         First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

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

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief Initializatio GPIO
 **
 ******************************************************************************/
#if !defined(GPIO1PIN_P3F_INIT)
#error P3F is not available in this MCU product, \
change to other re-location pin! Then delete "me" !
#endif

#if !defined(SetPinFunc_INT00_0)
#error INT00_0 is not available in this MCU product, \
change to other re-location pin! Then delete "me" !
#endif  

/**
 ******************************************************************************
 ** \brief Initialize IO port
 ******************************************************************************/
static void PortInit(void)
{
   Gpio1pin_InitOut(GPIO1PIN_P3F, Gpio1pin_InitVal(1u));
}

/**
 ******************************************************************************
 ** \brief Set port for LED
 ******************************************************************************/
static void SetLed(boolean_t bLed)
{
    if (0 == bLed)
    {
        Gpio1pin_Put(GPIO1PIN_P3F, 0);
    }
    else
    {
        Gpio1pin_Put(GPIO1PIN_P3F, 1);
    }
}

/**
 ******************************************************************************
 ** \brief Interruption function callback funciton
 ******************************************************************************/
static void ExIntCallback(void)
{
    ;
}

/**
 ******************************************************************************
 ** \brief External interrupt initialization
 ******************************************************************************/
static void InitExtInt()
{
    stc_exint_config_t stcExintConfig;
  
    SetPinFunc_INT00_0(0u); 	                  // Pin Function: INT00_0

    // Configure interrupt ch.0
    PDL_ZERO_STRUCT(stcExintConfig);
    
    stcExintConfig.abEnable[ExintInstanceIndexExint0] = 1u;
    stcExintConfig.aenLevel[ExintInstanceIndexExint0] = ExIntFallingEdge;
    stcExintConfig.apfnExintCallback[ExintInstanceIndexExint0] = ExIntCallback;
    
    stcExintConfig.bTouchNvic = TRUE;
       
    Exint_Init(&stcExintConfig);
}
/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    PortInit();
    /* Turn off LED */
    SetLed(1);

    /* Intialize external interrupt */
    InitExtInt();
    
    /* Enter sleep mode */
    Lpm_GoToStandByMode(StbSleepMode, TRUE);

    /* Turn on LED */
    SetLed(0);              /*  MCU has been wakeup here !*/

    while(1);
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
