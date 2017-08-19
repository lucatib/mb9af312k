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
 ** External interrupt example. 
 ** INT00_2 input is set to falling edge and the dedicated callback 
 ** functions counts the events.
 ** INT03_2 input is set to rising edge and the dedicated callback 
 ** functions counts the events.
 ** NMI callback function is set, and NMIX pin is enabled then. If a falling 
 ** edge is detected on NMIX, NMIX interrupt will occurs.
 **
 ** History:
 **   - 2014-11-21  0.0.1  EZh First version for FM uinversal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

// In this sample, INT00_2 and INT03_2 will be used, check whether these two is 
// available first.
#if !defined(SetPinFunc_INT00_2) || !defined(SetPinFunc_INT03_2)
#error INT00_2 or INT03_2 pins are not available in this MCU product, \
change to other re-location pins or interrupt channels! Then delete "me" !
#endif   
   
uint32_t u32ExtInt0Count = 0, u32ExtInt1Count = 0;
uint32_t u32NmiCount = 0;

/**
 ******************************************************************************
 ** \brief ExtInt0 callback function
 ******************************************************************************/
void Main_ExtInt0Callback(void)
{
    u32ExtInt0Count++;
}

/**
 ******************************************************************************
 ** \brief ExtInt1 callback function
 ******************************************************************************/
void Main_ExtInt1Callback(void)
{
    u32ExtInt1Count++;
}

/**
 ******************************************************************************
 ** \brief NMI callback function
 ******************************************************************************/
void Main_NmiCallback(void)
{
    u32NmiCount++;
}

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    stc_exint_config_t stcExintConfig;
    stc_exint_nmi_config_t stcNmiConfig;

    // The internal pull-up resistance can be connected for the external interrupt pin with falling edge detection if necessary 
    // For example : Gpio1pin_InitIn ( GPIO1PIN_Pxy, Gpio1pin_InitPullup(1u));

    SetPinFunc_INT00_2(0u); 	                  // Pin Function: INT00_2
    SetPinFunc_INT03_2(0u); 	                  // Pin Function: INT03_2

    // Configure interrupt ch.0 and ch.3
    PDL_ZERO_STRUCT(stcExintConfig);
    
    // Before initialize external interrupt ch.0, make sure PDL_PERIPHERAL_ENABLE_EXINT0 is set to PDL_ON in pdl_user.h
    // If external interrupt ch.0 is not used, set PDL_PERIPHERAL_ENABLE_EXINT0 to PDL_OFF !! Otherwise, the external interrupt ch.0 may be mis-enabled.
    stcExintConfig.abEnable[ExintInstanceIndexExint0] = 1u;
    stcExintConfig.aenLevel[ExintInstanceIndexExint0] = ExIntFallingEdge;
    stcExintConfig.apfnExintCallback[ExintInstanceIndexExint0] = Main_ExtInt0Callback;
    
    // Before initialize external interrupt ch.3, make sure PDL_PERIPHERAL_ENABLE_EXINT3 is set to PDL_ON in pdl_user.h
    // If external interrupt ch.3 is not used, set PDL_PERIPHERAL_ENABLE_EXINT3 to PDL_OFF !! Otherwise, the external interrupt ch.3 may be mis-enabled.
    stcExintConfig.abEnable[ExintInstanceIndexExint3] = 1u;
    stcExintConfig.aenLevel[ExintInstanceIndexExint3] = ExIntRisingEdge;
    stcExintConfig.apfnExintCallback[ExintInstanceIndexExint3] = Main_ExtInt1Callback;
    
    stcExintConfig.bTouchNvic = TRUE;
       
    Exint_Init(&stcExintConfig);
    
    // Congfigure NMI    
    PDL_ZERO_STRUCT(stcNmiConfig);
    stcNmiConfig.pfnNmiCallback = Main_NmiCallback;
    
    Exint_Nmi_Init(&stcNmiConfig);
    
    SetPinFunc_NMIX(0u); 	                  // Pin Function: NMIX
   
    while(1)
    {}
}
