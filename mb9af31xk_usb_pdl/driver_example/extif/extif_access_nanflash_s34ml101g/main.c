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
 ** This example tests erase, write and read function of Nand Flash S34ML101G.
 **
 ** This example can run on SK-FM4-144L-S6E2GM and SK-U120-9B560-MEM boards.
 **
 ** History:
 **   - 2015-02-04  1.0  EZh       First release version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
uint8_t u8Id[4];
uint32_t u32Block, u32Page;
uint8_t au8WrData[2048], au8WrSpare[64], au8WrDataSpare[2112];
uint8_t au8RdData[2048], au8RdSpare[64], au8RdDataSpare[2112];

/**
 ******************************************************************************
 ** \brief Test the page read and write of the Nand Flash
 ******************************************************************************/
void PageReadWriteTest(void)
{
    uint32_t i;
    
    // Erase block 0
    if(Ok != S34ml01g_EraseBlock(0u))
    {
        while(1);
    }
    
    // Write page 0 (main area from 0), then read back and varify    
    if(Ok != S34ml01g_WritePage(0u, 0u, 0u, au8WrData, 2048))
    {
        while(1);
    }
    
    if(Ok != S34ml01g_ReadPage(0u, 0u, 0u, au8RdData, 2048))
    {
        while(1);
    }
    
    for (i=0; i<2048; i++)
    {
        if (au8WrData[i] != au8RdData[i])
        {
            while(1);
        }
    }
    
    // Erase block 0
    if(Ok != S34ml01g_EraseBlock(0u))
    {
        while(1);
    }
    
    // Write page 0 (main area from page offset 50), then read back and varify    
    if(Ok != S34ml01g_WritePage(0u, 0u, 50u, au8WrData, 500))
    {
        while(1);
    }
    
    if(Ok != S34ml01g_ReadPage(0u, 0u, 50u, au8RdData, 500))
    {
        while(1);
    }
    
    for (i=50; i<550; i++)
    {
        if (au8WrData[i] != au8RdData[i])
        {
            while(1);
        }
    }
    
}

/**
 ******************************************************************************
 ** \brief Test the spare area read and write in a page of the Nand Flash
 ******************************************************************************/
void PageSpareReadWriteTest(void)
{
    uint32_t i;
  
    // Erase block 0
    if(Ok != S34ml01g_EraseBlock(0u))
    {
        while(1);
    }
    
    // Write page 0 (spare area), then read back and varify    
    if(Ok != S34ml01g_WritePage(0u, 0u, 2048u, au8WrSpare, 64u))
    {
        while(1);
    }
    
    if(Ok != S34ml01g_ReadPage(0u, 0u, 2048u, au8RdSpare, 64u))
    {
        while(1);
    }
    
    for (i=0; i<64; i++)
    {
        if (au8WrSpare[i] != au8RdSpare[i])
        {
            while(1);
        }
    }
    
    // Erase block 0
    if(Ok != S34ml01g_EraseBlock(0u))
    {
        while(1);
    }
    
    // Write page 0 (spare area), then read back and varify    
    if(Ok != S34ml01g_WritePage(0u, 0u, 2080u, au8WrSpare, 16u))
    {
        while(1);
    }
    
    if(Ok != S34ml01g_ReadPage(0u, 0u, 2080u, au8RdSpare, 16u))
    {
        while(1);
    }
    
    for (i=0u; i<16u; i++)
    {
        if (au8WrSpare[i] != au8RdSpare[i])
        {
            while(1);
        }
    }
}

/**
 ******************************************************************************
 ** \brief Test block erase, write, read of whole chip
 ******************************************************************************/
void FlashReadWriteTest(void)
{
    uint32_t u32Block, u32Page, i;
    
    for(u32Block=0; u32Block<1024; u32Block++)
    {
        // Erase block 
        if(Ok != S34ml01g_EraseBlock(u32Block))
        {
            while(1);
        }
        
        for(u32Page=0; u32Page<64; u32Page++)
        {
            // Write page (main area + spare area), then read back and varify    
            if(Ok != S34ml01g_WritePage(u32Block, u32Page, 0u, au8WrDataSpare, 2112u))
            {
                while(1);
            }
            
            if(Ok != S34ml01g_ReadPage(u32Block, u32Page, 0u, au8RdDataSpare, 2112u))
            {
                while(1);
            }
            
            for (i=0; i<2112u; i++)
            {
                if (au8WrDataSpare[i] != au8RdDataSpare[i])
                {
                    while(1);
                }
            }
        }
    }
}

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{  
    uint32_t i;
  
    for(i=0; i<2048; i++)
    {
        au8WrData[i] = 0x12^i;
    }

    for(i=0; i<64; i++)
    {
        au8WrSpare[i] = 0x55^i;
    }
    
    for(i=0; i<2112; i++)
    {
        au8WrData[i] = 0x98^i;
    }
    
#if (PDL_MCU_CORE == PDL_FM4_CORE)    
    // Reset External Bus for clean start.
    Clk_PeripheralClockDisable(ClkGateExtif);
    Clk_PeripheralSetReset(ClkResetExtif);
    Clk_PeripheralClearReset(ClkResetExtif);
    Clk_PeripheralClockEnable(ClkGateExtif);
#endif
    
    // Initialize Nand Flash IO
    // Data bus
    SetPinFunc_MADATA00_0();
    SetPinFunc_MADATA01_0();
    SetPinFunc_MADATA02_0();
    SetPinFunc_MADATA03_0();
    SetPinFunc_MADATA04_0();
    SetPinFunc_MADATA05_0();
    SetPinFunc_MADATA06_0();
    SetPinFunc_MADATA07_0();
    
    // Control bus
    SetPinFunc_MNREX_0();
#if (PDL_MCU_TYPE == PDL_FM4_TYPE0)     // SK-U120-9B560-MEM
    SetPinFunc_MCSX0_0();
#else                                   // SK-FM4-144L-S6E2GM
    SetPinFunc_MCSX2_0();
#endif    
    SetPinFunc_MNCLE_0();
    SetPinFunc_MNALE_0();
    SetPinFunc_MNWEX_0();
    
#if (PDL_MCU_TYPE == PDL_FM4_TYPE0)     // SK-U120-9B560-MEM    
    if (Ok != S34ml01g_Init(0u, 0x60000000u))
    {
        while(1);
    }
#else                                   // SK-FM4-144L-S6E2GM
    
    Gpio1pin_InitOut(GPIO1PIN_P16, Gpio1pin_InitVal(1u)); // Switch to SxB for ADG734 to connect with Nand Flash pins
    
    if (Ok != S34ml01g_Init(2u, 0x62000000u))
    {
        while(1);
    }
#endif    
    
    // Read ID
    if(Ok != S34ml01g_ReadID(u8Id, 4))
    {
        while(1);
    }
    
    PageReadWriteTest();
    PageSpareReadWriteTest();
    FlashReadWriteTest();
    
    while(1)
    {}
}
