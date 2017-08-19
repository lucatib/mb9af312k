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
 ** \brief This example shows how to use LCD CL010
 **
 ** History:
 **   - 2015-01-30  1.0  DHo        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"


/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#if ((SCM_CTL_Val & 0x08u) != 0x08u) // Sub clock not enable?
#error Before using this example, enable sub clock in system_fmx.h by setting bit 3 of \
definition "SCM_CTL_Val"  to 1!
#endif

#define DELAY_TIME  (2U)

/******************************************************************************/
/* Local variable                                                             */
/******************************************************************************/

/******************************************************************************/
/* local functions prototypes                                                */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief  Time delay
 **
 ** \param [in]   u32Cnt    delay count value
 **
 ** \return none
 ******************************************************************************/
static void Delay(uint32_t u32Cnt)
{
    uint32_t u32i;
    for(;u32Cnt;u32Cnt--)
    {
        for(u32i=(SystemCoreClock/5); u32i; u32i--)
        {
            ;
        }
    }
}

/**
 ******************************************************************************
 ** \brief  Scan every word in the LCD
 **
 ** \param [in]   none
 **
 ** \return none
 ******************************************************************************/
static void Cl010ScanEveryWord(void)
{
    uint16_t i;
    uint16_t u16Cnt = g_u16Cl010WordMapSize;
    
    for(i = 0; i < u16Cnt; i++) 
    {      
        Cl010_DispLcdWord(i);
        Delay(DELAY_TIME);
        Cl010_ClrLcdWord(i);
    }

    return;
}

/**
 ******************************************************************************
 ** \brief  Display all word in the LCD
 **
 ** \param [in]   none
 **
 ** \return none
 ******************************************************************************/
static void Cl010DispAllWord(void)
{
    uint16_t i;
    uint16_t u16Cnt = g_u16Cl010WordMapSize;

    for(i = 0;i < u16Cnt; i++) 
    {      
        Delay(DELAY_TIME);
        Cl010_DispLcdWord(i);
    } 
    Delay(DELAY_TIME);

    return;
}

/**
 ******************************************************************************
 ** \brief  Display blank on the LCD 
 **
 ** \param [in]   none
 **
 ** \return none
 ******************************************************************************/
static void Cl010DispBlank(void)
{
    Lcd_EnableBlankDisp();
    Delay(DELAY_TIME);
    Lcd_DisableBlankDisp();
    Delay(DELAY_TIME);  

    return;
}

/**
 ******************************************************************************
 ** \brief  Display commmon numbers on the LCD 
 **
 ** \param [in]   none
 **
 ** \return none
 ******************************************************************************/
static void Cl010DispCommNumber(void)
{
    uint8_t i;
    uint8_t num = 12u;

    Cl010_ClrScreen();
    for(i = 0u; i < num; i++)
    {
        Cl010_DispLcdCommonNum(0, i);
        Cl010_DispLcdCommonNum(1, i); 
        Cl010_DispLcdCommonNum(2, i); 
        Cl010_DispLcdCommonNum(3, i); 
        Cl010_DispLcdCommonNum(4, i); 
        Cl010_DispLcdCommonNum(5, i); 
        Cl010_DispLcdCommonNum(6, i); 
        Cl010_DispLcdCommonNum(7, i); 
        Cl010_DispLcdCommonNum(8, i); 
        Cl010_DispLcdCommonNum(9, i); 
        Delay(DELAY_TIME);
        Cl010_ClrLcdCommonNum(0);
        Cl010_ClrLcdCommonNum(1); 
        Cl010_ClrLcdCommonNum(2); 
        Cl010_ClrLcdCommonNum(3); 
        Cl010_ClrLcdCommonNum(4); 
        Cl010_ClrLcdCommonNum(5); 
        Cl010_ClrLcdCommonNum(6); 
        Cl010_ClrLcdCommonNum(7); 
        Cl010_ClrLcdCommonNum(8); 
        Cl010_ClrLcdCommonNum(9); 
    }

    return;
}

/**
 ******************************************************************************
 ** \brief  Word blinking 0.5s/1s internal time
 **
 ** \param [in]   none
 **
 ** \return none
 ******************************************************************************/
static void Cl010BlinkWord(void)
{
    /* 0.5s blinking internal time */    
    Lcd_SetBlinkDot(LCDC_Blik8COMS0C0, LcdBlinkOff);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C0, LcdBlinkOff);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C7, LcdBlinkOff);
    Cl010_DispLcdWord(10u);    
    Cl010_DispLcdWord(11u);
    Cl010_DispLcdWord(64u);
    Lcd_SetBinkInterval(LcdBlinkIntHalfSecond);
    Lcd_SetBlinkDot(LCDC_Blik8COMS0C0, LcdBlinkOn);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C0, LcdBlinkOn);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C7, LcdBlinkOn);
    Delay(2*DELAY_TIME);
    
    /* 1s blinking internal time */      
    Lcd_SetBlinkDot(LCDC_Blik8COMS0C0, LcdBlinkOff);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C0, LcdBlinkOff);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C7, LcdBlinkOff);
    Lcd_SetBinkInterval(LcdBlinkIntOneSecond);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C7, LcdBlinkOn);
    Lcd_SetBlinkDot(LCDC_Blik8COMS0C0, LcdBlinkOn);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C0, LcdBlinkOn);
    Lcd_SetBlinkDot(LCDC_Blik8COMS0C0, LcdBlinkOff);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C0, LcdBlinkOff);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C7, LcdBlinkOff);
    Delay(2*DELAY_TIME);
    
    return;
}

/**
 ******************************************************************************
 ** \brief  Main function
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main( void )
{     
    /* Initial LCD */
    Cl010_Init();
    Cl010_EnableLcdBackLight(); 
    /* Turn off beep for PM-FM0P-SOLU-MP board.*/
    Gpio1pin_InitOut( GPIO1PIN_P00, Gpio1pin_InitVal( 0u ) );
    while(1)
    {
        /* 1. Scan every word in the LCD */
        #ifdef DEBUG_PRINT
        printf("Scan every word in the LCD\n");
        #endif
        Cl010ScanEveryWord();

        /* 2. Display all word in the LCD */
        #ifdef DEBUG_PRINT
        printf("Display all word in the LCD\n");
        #endif
        Cl010DispAllWord();

        /* 3. Display blank on the LCD  */ 
        #ifdef DEBUG_PRINT
        printf("Display blank on the LCD\n");
        #endif
        Cl010DispBlank();

        /* 4. Display commmon numbers on the LCD */
        #ifdef DEBUG_PRINT
        printf("Display commmon numbers on the LCD\n");
        #endif
        Cl010DispCommNumber();
   
        /* 5. Blinking function */
        #ifdef DEBUG_PRINT
        printf("Blink word on the LCD\n");
        #endif
        Cl010BlinkWord();

        /* Clr LCD */
        Lcd_ClrWholeRam();          
    }
  
}

