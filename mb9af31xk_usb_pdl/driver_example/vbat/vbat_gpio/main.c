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
 ** This example configures pin P46,P47,P48,P49 in VBAT domain to GPIO, and
 ** tests the output and input funciton of them.
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
#if ((SCM_CTL_Val & 0x08) == 0x08)
#error Disable sub clock before using P46, P47 as GPIO.
#endif

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static boolean_t bInputLevel;

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    stc_vbat_config_t stcVbatConfig;
    
    stcVbatConfig.u8ClkDiv = 0x6E;
    stcVbatConfig.bLinkSubClk = FALSE; // Disconnect sub clock with clock control module 
    stcVbatConfig.bVbatClkEn = FALSE;  // Stop sub clock supply to VBAT domain 
    #if (PDL_MCU_TYPE == PDL_FM4_TYPE1) || \
        (PDL_MCU_TYPE == PDL_FM4_TYPE2) || \
        (PDL_MCU_TYPE == PDL_FM4_TYPE3)  
    stcVbatConfig.enSustainCurrent = Clk445nA;
    stcVbatConfig.enBoostCurrent = Clk510nA;
    #else
    stcVbatConfig.enSustainCurrent = Clk560nA;
    stcVbatConfig.enBoostCurrent = Clk560nA;
    #endif
    stcVbatConfig.enClkBoostTime = ClkBoost50ms;
    
    if (Ok != Vbat_Init(&stcVbatConfig))
    {
        while(1);
    }
  
    // Initialize P46,P47,P48,P49 to output with initial low
    Vbat_InitGpioOutput(VbatGpioP46, 0u, FALSE);
    Vbat_InitGpioOutput(VbatGpioP47, 0u, FALSE);
    Vbat_InitGpioOutput(VbatGpioP48, 0u, FALSE);
    Vbat_InitGpioOutput(VbatGpioP49, 0u, FALSE);
    
    // Set output level to high
    Vbat_PutPinP46(1u);
    Vbat_PutPinP47(1u);
    Vbat_PutPinP48(1u);
    Vbat_PutPinP49(1u);
    
    // Set output level to low
    Vbat_PutPinP46(0u);
    Vbat_PutPinP47(0u);
    Vbat_PutPinP48(0u);
    Vbat_PutPinP49(0u);
    
    // Initialize P46,P47,P48,P49 to input
    Vbat_InitGpioInput(VbatGpioP46, TRUE);
    Vbat_InitGpioInput(VbatGpioP47, TRUE);
    Vbat_InitGpioInput(VbatGpioP48, TRUE);
    Vbat_InitGpioInput(VbatGpioP49, TRUE);
    
    // Get input level
    bInputLevel = Vbat_GetPinP46();
    if(bInputLevel == 1u)
    {}
    
    bInputLevel = Vbat_GetPinP47();
    if(bInputLevel == 1u)
    {}
    
    bInputLevel = Vbat_GetPinP48();
    if(bInputLevel == 1u)
    {}
    
    bInputLevel = Vbat_GetPinP49();
    if(bInputLevel == 1u)
    {}
  
    while(1)
    {}
}
