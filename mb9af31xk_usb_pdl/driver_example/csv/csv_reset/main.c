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
 ** Demonstrate CSV reset when main and sub clock is abmornal
 **
 ** History:
 **   - 2014-12-16  0.0.1  DHo        First version.
 **
 ******************************************************************************/
 
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#if ((SCM_CTL_Val & 0x08u) != 0x08u) // Sub clock not enable?
#error Before using this example, enable sub clock in system_fmx.h by setting bit 3 of \
definition "SCM_CTL_Val"  to 1!
#endif

/* Initialize P3D */
#define MainClockFailure_LedInit()   Gpio1pin_InitOut(GPIO1PIN_P3D, Gpio1pin_InitDirectionOutput)
#define MainClockFailure_LedOn()     Gpio1pin_Put(GPIO1PIN_P3D, 0)
#define MainClockFailure_LedOff()    Gpio1pin_Put(GPIO1PIN_P3D, 1)

/* Initialize P3E */
#define SubClockFailure_LedInit()    Gpio1pin_InitOut(GPIO1PIN_P3E, Gpio1pin_InitDirectionOutput)
#define SubClockFailure_LedOn()      Gpio1pin_Put(GPIO1PIN_P3E, 0)
#define SubClockFailure_LedOff()     Gpio1pin_Put(GPIO1PIN_P3E, 1)

#define RESET_CSV_MASK         (0x0040)

/******************************************************************************/
/* Local Functions                                                            */
/******************************************************************************/
static void Delay(uint32_t Cnt);
static uint16_t ResetCause(void);

/*
 ******************************************************************************
 ** \brief Clock supervisor mode main function
 ******************************************************************************
 */
int32_t main(void)
{   
    static uint16_t u16ResetCause = 0;
    static stc_csv_status_t stcCsvStatus ; 
    boolean_t bClockSupervisor = FALSE;
    /* Initialize led*/
    MainClockFailure_LedInit(); 
    MainClockFailure_LedOff();
    SubClockFailure_LedInit(); 
    SubClockFailure_LedOff();
    
    Csv_EnableMainCsv();
    Csv_EnableSubCsv();
    
    while(1)
    {  
        /* Read causes of resets, reading the register clears all bits */
        u16ResetCause = ResetCause(); 
        // Check Clock Supervisor bit
        if (u16ResetCause&RESET_CSV_MASK)
        {
            Csv_DisableMainCsv();    
            Csv_DisableSubCsv();  
            bClockSupervisor = TRUE;
        }
        else
        {
            bClockSupervisor = FALSE;
        }
        
        /* Read CSV status */
        Csv_GetCsvFailCause(&stcCsvStatus);
       
        /* Clock Failure Detection(CSV) reset is generated */    
        if (TRUE == bClockSupervisor)
        {    
            /* Main Clock Failure Detection(CSV) reset is generated.*/
            if (TRUE == stcCsvStatus.bCsvMainClockStatus) 
            { 
                while(1)
                {
                    MainClockFailure_LedOff();
                    Delay(1);
                    MainClockFailure_LedOn();
                    Delay(1);
                }
            }
            
            /* Sub Clock Failure Detection(CSV) reset is generated */
            if (TRUE == stcCsvStatus.bCsvSubClockStatus)
            {  
                while(1)
                {
                    SubClockFailure_LedOff();
                    Delay(1);
                    SubClockFailure_LedOn();
                    Delay(1);
                }
            }
        } 
    }
}

/*
 ******************************************************************************
 ** \brief Time delay
 ******************************************************************************
 */
static void Delay(uint32_t u32Cnt)
{
    uint32_t u32i;
    for(; u32Cnt; u32Cnt--)
        for(u32i=SystemCoreClock/5; u32i; u32i--);
}

/*
 ******************************************************************************
 ** \brief Get reset cause
 ******************************************************************************
 */
static uint16_t ResetCause(void)
{
    uint16_t u16ResetCause = 0;
    
    u16ResetCause = FM_CRG->RST_STR; 
    return u16ResetCause; 
}
/*****************************************************************************/
/* END OF FILE */


