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
 ** Demonstrate FCS (Anomalous Frequency Detection) reset when Main clock 
 ** frequency is abnormal.
 **
 ** History:
 **   - 2014-12-11  0.0.1  DHo        First version.
 **
 ******************************************************************************/
 
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
 
/* Initialize P3D */
#define Led_Init() Gpio1pin_InitOut(GPIO1PIN_P3D, Gpio1pin_InitDirectionOutput)
#define Led_Off()  Gpio1pin_Put(GPIO1PIN_P3D, 1)
#define Led_On()   Gpio1pin_Put(GPIO1PIN_P3D, 0)


#define RESET_CSVR_MASK         (0x0040)
#define RESET_FCSR_MASK         (0x0080)

#define NORMAL_FREQUENCY_LOWER_VAL      (1000)
#define NORMAL_FREQUENCY_UPPER_VAL      (1350)

#define ERR_FREQUENCY_LOWER_VAL         (800)
#define ERR_FREQUENCY_UPPER_VAL         (900)

/******************************************************************************/
/* Local Functions                                                            */
/******************************************************************************/
static void Delay(uint32_t Cnt);
static uint16_t ResetCause(void);
static void FcsIntCallback(void);

/******************************************************************************/
/* Local variable                                                             */
/******************************************************************************/
static uint16_t m_u16MainClockCycle;
static uint8_t m_u8FcsIntFlag;

/*
 ******************************************************************************
 ** \brief Clock supervisor mode main function
 ******************************************************************************
 */
int32_t main(void)
{        
    stc_csv_status_t stcCsvStatus ; 
    static uint16_t u16ResetCause = 0;
    boolean_t bClockSupervisor = FALSE;
    boolean_t bAnomalousFrequency = FALSE;

    /* Initialize Green LED IO */
    Led_Init(); 
    Led_Off();
    
    /* Read causes of resets, reading the register clears all bits */
    u16ResetCause = ResetCause();  
    // Check Clock Supervisor bit
    if (u16ResetCause&RESET_CSVR_MASK)
    {
        bClockSupervisor = TRUE;
    }
    else
    {
        bClockSupervisor = FALSE;
    }

    // Check Anomalous Frequency bit
    if (u16ResetCause&RESET_FCSR_MASK)
    {
        bAnomalousFrequency = TRUE;
    }
    else
    {
        bAnomalousFrequency = FALSE;
    }
    /* Read CSV status */
    Csv_GetCsvFailCause(&stcCsvStatus);
   
    Csv_DisableMainCsv();    
    Csv_DisableSubCsv();   
     
    /* Clock Failure Detection(CSV) reset is generated */    
    if (TRUE == bClockSupervisor)
    {
        /* Sub Clock Failure Detection(CSV) reset is generated.*/
        if (stcCsvStatus.bCsvMainClockStatus == TRUE) 
        { 
            while(1);
        }
        
        /* Main Clock Failure Detection(CSV) reset is generated */
        if (stcCsvStatus.bCsvSubClockStatus == TRUE)
        {  
            while(1);
        }
    } 
    
    if(TRUE == bAnomalousFrequency)
    {
        while(1)
        {
            Led_Off();
            Delay(1);
            Led_On();
            Delay(1);
        }
    }
    
    /* FCS and CSV reset are not generated */
    if ((TRUE != stcCsvStatus.bCsvMainClockStatus) && 
        (TRUE != stcCsvStatus.bCsvSubClockStatus)) 
    {
        /* Assume High speed CR = 4MHz, the expected main clock cycle is 1024  */
        Csv_SetFcsCrDiv(FcsCrDiv1024);
        
        /* Assume High speed CR = 4MHz, this is a correct range */
        Csv_SetFcsDetectRange(NORMAL_FREQUENCY_LOWER_VAL, NORMAL_FREQUENCY_UPPER_VAL);
        /* Assume High speed CR = 4MHz, this is a wrong range to simulate a FCS reset */
        //Csv_SetFcsDetectRange(ERR_FREQUENCY_LOWER_VAL, ERR_FREQUENCY_UPPER_VAL);
        
        /* Enable FCS interrupt */
        Csv_EnableFcsIrq(FcsIntCallback);
        
        /* Enable FCS reset */
        Csv_EnableFcsReset();
        
        /* Enable FCS function */
        Csv_EnableFcs();
        
        while(1)
        {
            if(m_u8FcsIntFlag)
            {
                m_u8FcsIntFlag = 0;
             
                /* Read detected main clock cycle */
                m_u16MainClockCycle = Csv_GetFcsDetectCount();
                if(m_u16MainClockCycle)  // Avoid warning
                {}
                
            }
        }
    
    } 
    while(1){;}
    
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
 ** \brief FCS interrupt callback function
 ******************************************************************************
 */
static void FcsIntCallback(void)
{
    /* Disable FCS interrupt */
    //Csv_DisableFcsInt();
    
    /* Disable FCS function */
    //Csv_DisableFcs();
    
    m_u8FcsIntFlag = 1;
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


