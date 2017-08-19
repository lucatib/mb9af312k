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
 ** Main Module for Dual Timer sample (using interrupt)
 **
 ** History:
 **   - 2014-11-10  1.0  EZh       First release version for FM universal PDL.
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
static uint32_t u32CountDt0;
static uint32_t u32CountDt1;

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
#if !defined(GPIO1PIN_P61_INIT)
#error P61 is not available in this MCU product, \
change to other re-location pin! Then delete "me" !
#endif
    Gpio1pin_InitOut(GPIO1PIN_P61, Gpio1pin_InitVal(1u));
}

/**
 ******************************************************************************
 ** \brief Set port for LED1
 **
 ******************************************************************************/
static void SetLed0(uint32_t u32Led0)
{
    if (0 == (u32Led0 & 0x00000001))
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
 ** \brief Callback Function for DT channel 0
 **
 ******************************************************************************/
static void Dt0Callback(void)
{
    Dt_ClrIrqFlag(DtChannel0);    // Clear Irq
    u32CountDt0++;
    SetLed0(u32CountDt0);
}

/**
 ******************************************************************************
 ** \brief Callback Function for DT channel 1
 **
 ******************************************************************************/
static void Dt1Callback(void)
{
    Dt_ClrIrqFlag(DtChannel1);    // Clear Irq
    u32CountDt1++;
    // Channel 1 repeats 9 times.
    if (u32CountDt1 < 8)
    {
        Dt_WriteLoadVal(312500, DtChannel1);
    }
}

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample uses dual timer 2 channels. Mode of each channel is as follows.
 ** Channel 0 : Interval mode
 ** Channel 1 : One-shot mode
 ** Interval time for channel 0 is 0.5 sec at first, then interval time changes
 ** to 1 sec.
 ** Overflow time for channel 1 is 1sec. This channel repeats 9 times.
 **
 ** Above time interval is calculated when PCLK = 80MHz.
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_dt_channel_config_t stcDtChannelConfig0;
    stc_dt_channel_config_t stcDtChannelConfig1;
    
    // Clear structures
    PDL_ZERO_STRUCT(stcDtChannelConfig0);
    PDL_ZERO_STRUCT(stcDtChannelConfig1);
    
    // Initialize GPOI
    PortInit();
    
    // Initialize configuration
    stcDtChannelConfig0.u8Mode = DtPeriodic;
    stcDtChannelConfig0.u8PrescalerDiv = DtPrescalerDiv256;
    stcDtChannelConfig0.u8CounterSize = DtCounterSize32; 
    stcDtChannelConfig0.bIrqEnable = TRUE;
    stcDtChannelConfig0.pfnIrqCallback = Dt0Callback;
    stcDtChannelConfig0.bTouchNvic = TRUE;
    
    stcDtChannelConfig1.u8Mode = DtOneShot;
    stcDtChannelConfig1.u8PrescalerDiv = DtPrescalerDiv256;
    stcDtChannelConfig1.u8CounterSize = DtCounterSize32;
    stcDtChannelConfig1.bIrqEnable = TRUE;
    stcDtChannelConfig1.pfnIrqCallback = Dt1Callback;
    stcDtChannelConfig1.bTouchNvic = TRUE;

    // Initialize interrupt counter
    u32CountDt0 = 0;
    u32CountDt1 = 0;

    // Initialize dual timer channel 0
    if (Ok != Dt_Init((stc_dt_channel_config_t*)&stcDtChannelConfig0, DtChannel0))
    {
#ifdef DEBUG_PRINT
        printf("Initial error channel0!\n");
#endif
        while(1);
    }

    // Initialize dual timer channel 1
    if (Ok != Dt_Init((stc_dt_channel_config_t*)&stcDtChannelConfig1, DtChannel1))
    {
#ifdef DEBUG_PRINT
        printf("Initial error channel1!\n");
#endif
        while(1);
    }
    // Write load value for channel 0 (0.5sec interval @ PCLK = 80MHz)
    Dt_WriteLoadVal(78125, DtChannel0);
    // Write background load value for channel 0 (0.5sec -> 1sec @ PCLK = 80MHz)
    Dt_WriteBgLoadVal(156250, DtChannel0);
    // Start count for channel 0
    Dt_EnableCount(DtChannel0);

    // Write load value for channel 1 (1sec until overflow @ PCLK = 80MHz)
    Dt_WriteLoadVal(156250, DtChannel1);
    // Start count for channel 1
    Dt_EnableCount(DtChannel1);


    while(1);
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
