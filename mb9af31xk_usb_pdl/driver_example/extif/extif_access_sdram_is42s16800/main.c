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
 ** This example shows how to access to SDRAM IS42S16800 via external bus
 ** interface,
 **
 ** History:
 **   - 2015-02-04  1.0  EZh       First release version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#if (PDL_MCU_TYPE != PDL_FM4_TYPE4)
#error This examples is developed on SK-FM4-176L-S6E2DH. And please note that \
this example uses the SDRAM I/F of GDC, so GDC clock has to be intialized. \
If the SDRAM I/F of external bus I/F is used, the GDC part need not be touched. \
And following code can also be referred to.
#endif

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
uint32_t u32Addr;

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    // For demeter (S6E2DH5), if using the SDRAM I/F of GDC, the GDC clock has
    // to be intialized as folloiwng.
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

    IRIS_SDL2_SubSysCtrl->SDRAMC_DomainControl = 0x00010000u; // Software reset, clock disable
    IRIS_SDL2_SubSysCtrl->SDRAMC_ClockDivider = 0x40000u;       // Set clock divide value
    IRIS_SDL2_SubSysCtrl->SDRAMC_DomainControl = 0x00010001u; // Enable clock
    IRIS_SDL2_SubSysCtrl->SDRAMC_DomainControl = 0x00000001u; // Release software reset

#if (PDL_MCU_CORE == PDL_FM4_CORE)    
    // Reset External Bus for clean start.
    Clk_PeripheralClockDisable(ClkGateExtif);
    Clk_PeripheralSetReset(ClkResetExtif);
    Clk_PeripheralClearReset(ClkResetExtif);
    Clk_PeripheralClockEnable(ClkGateExtif);
#endif

    // Initialize SDRAM IO
    // Data bus
    SetPinFunc_GE_SDA0();
    SetPinFunc_GE_SDA1();
    SetPinFunc_GE_SDA2();
    SetPinFunc_GE_SDA3();
    SetPinFunc_GE_SDA4();
    SetPinFunc_GE_SDA5();
    SetPinFunc_GE_SDA6();
    SetPinFunc_GE_SDA7();
    SetPinFunc_GE_SDA8();
    SetPinFunc_GE_SDA9();
    SetPinFunc_GE_SDA10();
    SetPinFunc_GE_SDA11();

    // Address bus
    SetPinFunc_GE_SDDQ0();
    SetPinFunc_GE_SDDQ1();
    SetPinFunc_GE_SDDQ2();
    SetPinFunc_GE_SDDQ3();
    SetPinFunc_GE_SDDQ4();
    SetPinFunc_GE_SDDQ5();
    SetPinFunc_GE_SDDQ6();
    SetPinFunc_GE_SDDQ7();
    SetPinFunc_GE_SDDQ8();
    SetPinFunc_GE_SDDQ9();
    SetPinFunc_GE_SDDQ10();
    SetPinFunc_GE_SDDQ11();
    SetPinFunc_GE_SDDQ12();
    SetPinFunc_GE_SDDQ13();
    SetPinFunc_GE_SDDQ14();
    SetPinFunc_GE_SDDQ15();

    // Bank lines
    SetPinFunc_GE_SDBA0();
    SetPinFunc_GE_SDBA1();

    // Control lines
    SetPinFunc_GE_SDCSX();
    SetPinFunc_GE_SDCASX();
    SetPinFunc_GE_SDRASX();
    SetPinFunc_GE_SDWEX();
    SetPinFunc_GE_SDCKE();
    SetPinFunc_GE_SDCLK();
    SetPinFunc_GE_SDDQM0();
    SetPinFunc_GE_SDDQM1();

    // Base address of FM4 TYPE4 product: 0xB0000000 (Check this information in product datasheet)
    // Size = (2^(Column line number + Row line number))*Bank number*bit width = (2^(9+12))*4*16bit = 128Mb = 16MB
    // SDRAM area: 0xB0000000 - 0xB1000000
    if (Ok != Is42s16800_Init(TRUE, 9u, 12u, SdramWidth16Bit))
    {
        while(1);
    }

    // Test SDRAM with 16-bit width
    for(u32Addr = 0xB0000000u; u32Addr < 0xB1000000u; u32Addr += 2u)
    {
        *(uint16_t*)u32Addr = 0x5555u;
        if(*(uint16_t*)u32Addr != 0x5555u)
        {
            while(1);
        }
    }

    // Test SDRAM with 8-bit width
    for(u32Addr = 0xB0000000u; u32Addr < 0xB1000000u; u32Addr += 1u)
    {
        *(uint8_t*)u32Addr = 0xAAu;
        if(*(uint8_t*)u32Addr != 0xAAu)
        {
            while(1);
        }
    }

    // Test SDRAM with 32-bit width
    for(u32Addr = 0xB0000000u; u32Addr < 0xB1000000u; u32Addr += 4u)
    {
        *(uint32_t*)u32Addr = 0x55555555u;
        if(*(uint32_t*)u32Addr != 0x55555555u)
        {
            while(1);
        }
    }

    while(1)
    {}
}
