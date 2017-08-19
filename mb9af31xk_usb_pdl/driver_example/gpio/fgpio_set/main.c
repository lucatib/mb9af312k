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
 ** \brief This example shows how to use Fast GPIO macros
 **
 ** This example enables Fast IO output mode of P10 and P3E and initializes P10 
 ** and P3E to output with the default value '0' and P0F to input with no 
 ** pull-up resistor connected. 
 ** Afterwards the P10 and P3E output are set to '1', and set to '0' at the 
 ** following.
 ** Then the status of P0F is read out.
 ** The same is done with P10 and P3E but with inverted logic at the following.
 ** Also the same is done with P0F with inverted logic at the following.  
 ** At last, the Fast IO mode of Port 0 and 3 are disabled.
 **
 ** History:
 **   - 2014-01-15  1.0  EZh      First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#if (PDL_MCU_CORE != PDL_FM0P_CORE)
#error "Only FM0+ supports fast GPIO function!"
#endif

boolean_t bReadData; 

/**
 ******************************************************************************
 ** \brief  Sample program of Fast GPIO
 **
 ** \return int32_t return value, if needed
 ** \note The optimization need to be enabled to achieve high IO polling speed
 ******************************************************************************/
int32_t main(void)
{
    // enable Fast GPIO output mode
    //    F E D C B A 9 8 7 6 5 4 3 2 1 0
    // b' 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
    //                                  |-> P10
    FGpio_EnableOutput(FGpioPort1, 0x0001u); 
    //    F E D C B A 9 8 7 6 5 4 3 2 1 0
    // b' 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0
    //      |-> P3E
    FGpio_EnableOutput(FGpioPort3, 0x4000u);
    
    FGpio1pin_InitOut( FGPIO1PIN_P10, FGpio1pin_InitVal( 0x00u ) );  // intial Fast GPIO as output pin
    FGpio1pin_InitOut( FGPIO1PIN_P3E, FGpio1pin_InitVal( 0x00u ) );  // intial Fast GPIO as output pin
    FGpio1pin_InitIn( FGPIO1PIN_P0F, FGpio1pin_InitPullup(0) );  // intial Fast GPIO as input pin
   
    FGpio1pin_Put(FGPIO1PIN_P10, 0xFFu); // set P10 to high level
    FGpio1pin_Put(FGPIO1PIN_P10, 0x00u); // set P10 to low level
    FGpio1pin_Put(FGPIO1PIN_P3E, 0xFFu); // set P3E to high level
    FGpio1pin_Put(FGPIO1PIN_P3E, 0x00u); // set P3E to low level

    bReadData = FGpio1pin_Get(FGPIO1PIN_P0F);  // read the P0F

    FGpio1pin_Put(FGPIO1PIN_NP10, 0xFFu); // set P10 to high level with invert logic (equal to set to low)
    FGpio1pin_Put(FGPIO1PIN_NP10, 0x00u); // set P10 to low level with invert logic (equal to set to high)
    FGpio1pin_Put(FGPIO1PIN_NP3E, 0xFFu); // set P3E to high level with invert logic (equal to set to low)
    FGpio1pin_Put(FGPIO1PIN_NP3E, 0x00u); // set P3E to low level with invert logic (equal to set to high)
    
    bReadData = FGpio1pin_Get(FGPIO1PIN_NP0F); // read the P0F with invert logic
    // disable Fast GPIO mode for Port 1 and 3
    FGpio_DisableOutput(FGpioPort1); 
    FGpio_DisableOutput(FGpioPort3); 
    
    while(1);
}
