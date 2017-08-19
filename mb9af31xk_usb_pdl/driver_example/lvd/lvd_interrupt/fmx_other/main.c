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
 ** Main Module for LVD sample (use interrupt)
 **
 ** History:
 **   - 2014-02-09  0.0.1  EZh         First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#if !defined(GPIO1PIN_P3F_INIT)
#error P3F is not available in this product, please change to other pins and \
delete "me"!
#endif

#if (PDL_MCU_CORE == PDL_FM3_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
#define LVD_IRQ_VOLTAGE           Lvd0IrqDetectVoltage300                                        
#elif (PDL_MCU_CORE == PDL_FM4_CORE)
#if (PDL_MCU_TYPE == PDL_FM4_TYPE1) || (PDL_MCU_TYPE == PDL_FM4_TYPE2) || (PDL_MCU_TYPE == PDL_FM4_TYPE6)
#define LVD_IRQ_VOLTAGE           Lvd0IrqDetectVoltage300   
#else 
#define LVD_IRQ_VOLTAGE           Lvd0IrqDetectVoltage310   
#endif
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
boolean_t bLowVoltageDetect;
/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief Interruption function when the LVD detected
 **
 ******************************************************************************/
static void LvdCallback(void)
{
    bLowVoltageDetect = 1;
}

/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    en_result_t enRet;
    stc_lvd_config_t stcLvdConfig;
    
    // Clear structure
    PDL_ZERO_STRUCT(stcLvdConfig);
    
    // Initialize GPIO
    Gpio1pin_InitOut(GPIO1PIN_P3F, Gpio1pin_InitVal(1u));
    
    // Initialize LVD
    stcLvdConfig.enLvd0IrqDetectVoltage = LVD_IRQ_VOLTAGE;
    stcLvdConfig.pfnIrqCallback[0] = LvdCallback;
    
    enRet = Lvd_Init(&stcLvdConfig);
    
    if(Ok != enRet)
    {
        while(1);
    }
    
    // Enable interrupt detection
    Lvd_EnableIrqDetect(0u);
    
    // Wait operation stability
    while(TRUE != Lvd_GetIrqOperationStatus(0u));
    
    while(1)
    {
        if(bLowVoltageDetect == TRUE)
        {
            bLowVoltageDetect = FALSE;
            Gpio1pin_Put(GPIO1PIN_P3F, 0u);
        }
    }
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
