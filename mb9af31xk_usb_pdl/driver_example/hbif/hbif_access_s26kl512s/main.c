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
 **   - 2014-11-10  1.0  EZh       First release version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

uint16_t au16DataBuffer[512];   
uint16_t u16ManufactureId, u16DeviceId, u16DeviceSize;

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    uint32_t i;
    
    // Dsiable clock
    FM_GSSPRE->GCCR_f.GSSEN = 0u;
    FM_GSSPRE->GCSR = 0u;
    
    // Initialize PLL
    FM_GSSPRE->GPCR1_f.GPINC = 0x00;
    FM_GSSPRE->GPCR2_f.GPOWT = 0;
    FM_GSSPRE->GPCR3_f.GPLLK = 0;
    FM_GSSPRE->GPCR4_f.GPLLN = 99;
    FM_GSSPRE->GPCR1_f.GPLLEN = 1;
    while(FM_GSSPRE->GP_STR_f.GPRDY==0){}; //Wait for PLL-stabilization
    
    FM_GSSPRE->GCSR_f.ASEL = 0u;  
    FM_GSSPRE->GCSR_f.PSEL = 0u;  
    FM_GSSPRE->GCSR_f.ACG = 0u;   
    FM_GSSPRE->GCSR_f.PCG = 0u;   
    FM_GSSPRE->GCCR_f.GSSEN = 1u; 
  
    // Configure IRIS IP
    IRIS_SDL2_SubSysCtrl->AXI_ClockDivider = 0x40000u;
    IRIS_SDL2_SubSysCtrl->ConfigClockControl = 0x01u;
    
    IRIS_SDL2_SubSysCtrl->dsp0_ClockDivider = 0x200000u;
    IRIS_SDL2_SubSysCtrl->dsp0_ClockShift = 0x00u;
    
    IRIS_SDL2_SubSysCtrl->SDRAMC_DomainControl = 0x00010000u;
    IRIS_SDL2_SubSysCtrl->SDRAMC_ClockDivider = 0x400u;
    IRIS_SDL2_SubSysCtrl->SDRAMC_DomainControl = 0x00010001u;
    IRIS_SDL2_SubSysCtrl->SDRAMC_DomainControl = 0x00000001u;
    
    IRIS_SDL2_SubSysCtrl->RPCC_DomainControl = 0u;
    IRIS_SDL2_SubSysCtrl->ConfigMemorySelect = 0u; // Select Hyper Flash/RAM
    IRIS_SDL2_SubSysCtrl->RPCC_ClockDivider = 0u;
    IRIS_SDL2_SubSysCtrl->RPCC_DomainControl = 1u;
    
    // Initialize HyperBus interface
    SetPinFunc_GE_HBDQ0();
    SetPinFunc_GE_HBDQ1();
    SetPinFunc_GE_HBDQ2();
    SetPinFunc_GE_HBDQ3();
    SetPinFunc_GE_HBDQ4();
    SetPinFunc_GE_HBDQ5();
    SetPinFunc_GE_HBDQ6();
    SetPinFunc_GE_HBDQ7();
    SetPinFunc_GE_HBCSX_0();
    SetPinFunc_GE_HBCK();
    SetPinFunc_GE_HBRWDS();

    // Select chip selection area 0 for Hyper flash, base address = 0xC0000000u
    // Whole memeory: 0xC000 0000 - 0xC0FF FFFF
    // Sector 0: 0xC000 0000 - 0xC003 FFFF
    // Sector 1: 0xC004 0000 - 0xC007 FFFF
    // ...
    // Sector 256: 0xC0FC 0000 - 0xC0FF FFFF
    S26kl512s_Init(0u, 0xC0000000u);
    
    // Read manufacture ID
    S26kl512s_ReadId(&u16ManufactureId, 0u, 1u);
    
    // Read device ID
    S26kl512s_ReadId(&u16DeviceId, 1u, 1u);
    
    // Read device size (0x1A: 512Mb)
    S26kl512s_ReadCfi(&u16DeviceSize, 0x27u, 1u);
    
    // Erase sector 0
    if(Ok != S26kl512s_SectorErase((uint16_t*)0xC0000000u))
    {
        while(1);
    }

    // Erase sector blank check
    if (Ok != S26kl512s_BlankCheck((uint16_t*)0xC0000000u))
    {
        while(1);
    }
    
    // Word program 
    if(Ok != S26kl512s_WordProgram((uint16_t*)0xC0000000u, 0x55AA))
    {
        while(1);
    }

    // Word verify
    if(*(uint16_t*)0xC0000000u != 0x55AA)
    {
        while(1);
    }
    
    // Initialize data array
    for(i=0u; i<256u; i++)
    {
        au16DataBuffer[i] = (i | i<<8u);
    }
    
    // Buffer program to first 512 bytes (256 words) of sector 0
    if(Ok != S26kl512s_WriteBufferAndProgram((uint16_t*)0xC0000000u, au16DataBuffer, 256))
    {
        while(1);
    }
    
    // Buffer program to a random 512 bytes (256 words) of sector 0
    if(Ok != S26kl512s_WriteBufferAndProgram((uint16_t*)0xC0001000u, au16DataBuffer, 256))
    {
        while(1);
    }
    
    // Verify the data programmed at the first 512 bytes (256 words) of sector 0
    for(i=0u; i<256u; i++)
    {
        if (au16DataBuffer[i] != *((uint16_t*)0xC0000000u + i))
        {
            while(1);
        }
    }
    // Verify the data programmed at random 512 bytes (256 words) of sector 0
    for(i=0u; i<256u; i++)
    {
        if (au16DataBuffer[i] != *((uint16_t*)0xC0001000u + i))
        {
            while(1);
        }
    }
    
    // Buffer program to first 512 bytes (256 words) of sector 255
    if(Ok != S26kl512s_WriteBufferAndProgram((uint16_t*)0xC0FC0000u, au16DataBuffer, 256))
    {
        while(1);
    }
    
    // Buffer program to a random 512 bytes (256 words) of sector 255
    if(Ok != S26kl512s_WriteBufferAndProgram((uint16_t*)0xC0FC1000u, au16DataBuffer, 256))
    {
        while(1);
    }
    
    // Verify the data programmed at the first 512 bytes (256 words) of sector 255
    for(i=0u; i<256u; i++)
    {
        if (au16DataBuffer[i] != *((uint16_t*)0xC0FC0000u + i))
        {
            while(1);
        }
    }
    
    // Verify the data programmed at random 512 bytes (256 words) of sector 255
    for(i=0u; i<256u; i++)
    {
        if (au16DataBuffer[i] != *((uint16_t*)0xC0FC1000u + i))
        {
            while(1);
        }
    }
    
    // Chip Erase
    if (Ok != S26kl512s_ChipErase())
    {
        while(1);
    }
    
    while(1)
    {}
}
