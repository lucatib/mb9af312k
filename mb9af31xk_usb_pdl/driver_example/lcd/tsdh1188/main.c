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
 ** \brief This example shows how to use LCD TSDH1188
 **
 ** History:
 **   - 2015-02-02  1.0  DHo        First version.
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

#define DELAY_TIME  (1U)

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
static void Tsdh1188ScanEveryWord(void)
{
    uint16_t i;
    uint16_t u16Cnt = g_u16Tsdh1188WordMapSize;

    for(i = 0; i < u16Cnt; i++) 
    {      
        Tsdh1188_DispLcdWord(i);
        Delay(DELAY_TIME);
        Tsdh1188_ClrLcdWord(i);
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
static void Tsdh1188DispAllWord(void)
{
    uint16_t i;
    uint16_t u16Cnt = g_u16Tsdh1188WordMapSize;

    for(i = 0;i < u16Cnt; i++) 
    {      
        Delay(DELAY_TIME);
        Tsdh1188_DispLcdWord(i);
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
static void Tsdh1188DispBlank(void)
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
static void Tsdh1188DispCommNumber(void)
{
    uint8_t i;
    uint8_t num = 10u;

    Tsdh1188_ClrScreen();
    for(i = 0u; i < num; i++)
    {
        Tsdh1188_DispLcdCommonNum(0, i);
        Tsdh1188_DispLcdCommonNum(1, i); 
        Tsdh1188_DispLcdCommonNum(2, i); 
        Tsdh1188_DispLcdCommonNum(3, i); 
        Tsdh1188_DispLcdCommonNum(4, i); 
        Tsdh1188_DispLcdCommonNum(5, i); 
        Tsdh1188_DispLcdCommonNum(6, i); 
        Tsdh1188_DispLcdCommonNum(7, i); 
        Delay(DELAY_TIME);
        Tsdh1188_ClrLcdCommonNum(0);
        Tsdh1188_ClrLcdCommonNum(1); 
        Tsdh1188_ClrLcdCommonNum(2); 
        Tsdh1188_ClrLcdCommonNum(3); 
        Tsdh1188_ClrLcdCommonNum(4); 
        Tsdh1188_ClrLcdCommonNum(5); 
        Tsdh1188_ClrLcdCommonNum(6); 
        Tsdh1188_ClrLcdCommonNum(7); 
    }

    return;
}

/**
 ******************************************************************************
 ** \brief  Display month number on the LCD
 **
 ** \param [in]   none
 **
 ** \return none
 ******************************************************************************/
static void Tsdh1188DispMonthNumber(void)
{
    uint8_t i;

    Tsdh1188_ClrScreen();
    for(i = 1u; i <= 12u; i++)
    {
        Tsdh1188_DispLcdMonthNum(i);
        Delay(DELAY_TIME);
        Tsdh1188_ClrLcdMonthNum(i);
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
static void Tsdh1188BlinkWord(void)
{
    /* 0.5s blinking internal time */    
    Lcd_SetBlinkDot(LCDC_Blik8COMS0C0, LcdBlinkOff);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C0, LcdBlinkOff);
    Lcd_SetBlinkDot(LCDC_Blik8COMS1C7, LcdBlinkOff);
    Tsdh1188_DispLcdWord(10u);    
    Tsdh1188_DispLcdWord(11u);
    Tsdh1188_DispLcdWord(64u);
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
    Tsdh1188_Init();
    Tsdh1188_EnableLcdBackLight(); 

    while(1)
    {
        /* 1. Scan every word in the LCD */
        #ifdef DEBUG_PRINT
        printf("Scan every word in the LCD\n");
        #endif
        Tsdh1188ScanEveryWord();

        /* 2. Display all word in the LCD */
        #ifdef DEBUG_PRINT
        printf("Display all word in the LCD\n");
        #endif
        Tsdh1188DispAllWord();

        /* 3. Display blank on the LCD  */ 
        #ifdef DEBUG_PRINT
        printf("Display blank on the LCD\n");
        #endif
        Tsdh1188DispBlank();

        /* 4. Display commmon numbers on the LCD */
        #ifdef DEBUG_PRINT
        printf("Display commmon numbers on the LCD\n");
        #endif
        Tsdh1188DispCommNumber();

        /* 5. Display month number on the LCD */  
        #ifdef DEBUG_PRINT
        printf("Display month number on the LCD\n");
        #endif
        Tsdh1188DispMonthNumber();
   
        /* 6. Blinking function */
        #ifdef DEBUG_PRINT
        printf("Blink word on the LCD\n");
        #endif
        Tsdh1188BlinkWord();

        /* Clr LCD */
        Lcd_ClrWholeRam();    
    }
  
}

