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
 ** History:
 **   - 2015-02-04  1.0  EZh       First release version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
   
/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
#if (PDL_MCU_CORE == PDL_FM4_CORE)  
    // Reset External Bus for clean start.
    Clk_PeripheralClockDisable(ClkGateExtif);
    Clk_PeripheralSetReset(ClkResetExtif);
    Clk_PeripheralClearReset(ClkResetExtif);
    Clk_PeripheralClockEnable(ClkGateExtif);
#endif
    // Configure address bus
    SetPinFunc_MAD01_0();
    SetPinFunc_MAD02_0();
    SetPinFunc_MAD03_0();
    SetPinFunc_MAD04_0();
    SetPinFunc_MAD05_0();
    SetPinFunc_MAD06_0();
    SetPinFunc_MAD07_0();
    SetPinFunc_MAD08_0();
    SetPinFunc_MAD09_0();
    SetPinFunc_MAD10_0();
    SetPinFunc_MAD11_0();
    SetPinFunc_MAD12_0();
    SetPinFunc_MAD13_0();
    SetPinFunc_MAD14_0();
    SetPinFunc_MAD15_0();
    SetPinFunc_MAD16_0();
    SetPinFunc_MAD17_0();
    SetPinFunc_MAD18_0();
    SetPinFunc_MAD19_0();
    SetPinFunc_MAD20_0();
  
    // Configure data bus
    SetPinFunc_MADATA00_0();
    SetPinFunc_MADATA01_0();
    SetPinFunc_MADATA02_0();
    SetPinFunc_MADATA03_0();
    SetPinFunc_MADATA04_0();
    SetPinFunc_MADATA05_0();
    SetPinFunc_MADATA06_0();
    SetPinFunc_MADATA07_0();
    SetPinFunc_MADATA08_0();
    SetPinFunc_MADATA09_0();
    SetPinFunc_MADATA10_0();
    SetPinFunc_MADATA11_0();
    SetPinFunc_MADATA12_0();
    SetPinFunc_MADATA13_0();
    SetPinFunc_MADATA14_0();
    SetPinFunc_MADATA15_0();
    
    // Configure control bus
    SetPinFunc_MCSX0_0();
    SetPinFunc_MOEX_0();
    SetPinFunc_MWEX_0();
    SetPinFunc_MDQM0_0();
    SetPinFunc_MDQM1_0();
 
    // Initialize PSRAM (locate to a certain external space)
    if(Ok != Sv6p1615_Init(0u, 0x60000000, Psram2MB))
    {
        while(1);
    }
    
    // Access PSRAM with 16 bit width
    *(uint16_t*)0x60000000 = 0x55AA;
    if(0x55AA != *(uint16_t*)0x60000000)
    {
        while(1);
    }
    
    // Access PSRAM with 8 bit width
    *(uint8_t*)0x60010000 = 0x11;
    if(0x11 != *(uint8_t*)0x60010000)
    {
        while(1);
    }
    
    // Access PSRAM with 32 bit width
    *(uint32_t*)0x60080000 = 0x12345678;
    if(0x12345678 != *(uint32_t*)0x60080000)
    {
        while(1);
    }
    
    while(1)
    {}
}
