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
 ** \brief This example shows the clock gating procedure
 **
 ** The example shows the procedure how the clock of an peripheral is gated,
 ** reset and enabled again.
 **
 ** History:
 **   - 2015-03-19  1.0  EZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#if (PDL_MCU_CORE != PDL_FM4_CORE) && (PDL_MCU_CORE != PDL_FM0P_CORE)
#error Only FM4 and FM0+ device support clock gating function!
#endif

/**
 ******************************************************************************
 ** \brief  Main function
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    Clk_PeripheralClockDisable(ClkGateMfs4);                   // Disable Clock of MFS4
    while (TRUE == Clk_PeripheralGetClockState(ClkGateMfs4))   // Wait for disable (== FALSE)
    {}
    
    Clk_PeripheralSetReset(ClkResetMfs4);                       // Set MFS4 to reset state
    Clk_PeripheralClearReset(ClkResetMfs4);                     // Release MFS4 from reset state
    
    Clk_PeripheralClockEnable(ClkGateMfs4);                    // Enable Clock of MFS4
    while (FALSE == Clk_PeripheralGetClockState(ClkGateMfs4))  // Wait for enable (== TRUE)
    {}
  
    while(1)
    {}
}
