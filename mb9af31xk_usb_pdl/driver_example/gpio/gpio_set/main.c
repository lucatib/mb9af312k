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
 ** \brief This example shows how to use GPIO macros
 **
 ** This example initializes Port P3D not inverted to output with the default
 ** value '0'. Afterwards the port output is set to '1'.
 ** The next step is the initialization of Port P10 to input. The port state
 ** is then read-out.
 ** The same is done with P3D and P10 but with inverted logic.
 ** The last step is just to show, how to set a peripheral pin function. In
 ** this case the external interrupt channel 3 on its relocation '0'.
 **
 ** History:
 **   - 2014-11-15  1.0  EZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#if !defined(GPIO1PIN_P3D_INIT) || !defined(GPIO1PIN_P10_INIT) || !defined(SetPinFunc_INT03_0) 
#error "P3D or P10 or INT03_0 used in the example are not available in the device, \
please change to other pins."
#endif

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static boolean_t bReadData;

/**
 ******************************************************************************
 ** \brief  Main function of project for MB9B560 series.
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    Gpio1pin_InitOut( GPIO1PIN_P3D, Gpio1pin_InitVal( 0u ) );     // Init Output P3D, Value: 0
    Gpio1pin_Put( GPIO1PIN_P3D, 1u);                              // Output P3D, Value: 1

    Gpio1pin_InitIn ( GPIO1PIN_P10, Gpio1pin_InitPullup( 0u ) );  // Init Input P10
    bReadData = Gpio1pin_Get( GPIO1PIN_P10 );                     // Read P10, bReadData = 0: P10 = L, bReadData = 1 : P10 = H
    if(bReadData)
    {}
    
    Gpio1pin_InitOut( GPIO1PIN_NP3D, Gpio1pin_InitVal( 0u ) );    // Init inverted Output P3D, value = 1 (inverted value of 0)
    Gpio1pin_Put( GPIO1PIN_NP3D, 1u);                             // Output inverted P3D, Value: 1

    Gpio1pin_InitIn ( GPIO1PIN_NP10, Gpio1pin_InitPullup( 0u ) ); // Init inverted Input P10
    
    bReadData = Gpio1pin_Get( GPIO1PIN_NP10 );                    // Read inverted P10. bReadData = 0: P10 = H, bReadData = 1 : P10 = L
    if(bReadData)
    {}
    
    SetPinFunc_INT03_0(0u); 	// Pin Function: INT03_0

    while(1);
}
