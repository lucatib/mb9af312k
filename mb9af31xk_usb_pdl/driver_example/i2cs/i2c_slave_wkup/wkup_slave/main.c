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
 ** This example demonstrates I2C Slave wakup function. By receive the matched 
 ** device address or reserved address by I2C Slave, MCU can wakeup from standby
 ** mode. 
 ** This example must be used with the example "wakeup_master".
 ** Make following connection before using this example:
 ** Master (1st board "wakeup_slave")   Slave (2nd board with "wakeup_master")
 ** SI2CSCL6_1   ---------------------------     SCK7_1
 ** SI2CSDA6_1   ---------------------------     SOT7_1
 **
 **
 ** History:
 **   - 2015-07-08  1.0  EZh       First release version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local varible                                                              */
/******************************************************************************/
volatile boolean_t bWakupFlag;  

/**
 ******************************************************************************
 ** \brief  Delay some time
 ******************************************************************************/
void Delay(uint32_t u32Cnt)
{
    uint32_t i,j;
      
    for (i=u32Cnt; i; i--)
    {
        for (j=SystemCoreClock/10; j; j--);
    }
}

/**
 ******************************************************************************
 ** \brief  I2cs Slave status callback function
 ******************************************************************************/
void I2csSlaveStatusIrqCallback(void)
{
    bWakupFlag = TRUE;
}

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    stc_i2cs_config_t stcConfig;
    stc_i2cs_irq_cb_t stcIrqCb;
    stc_i2cs_irq_en_t stcIrqEn;
    
    // Set I2C Slave function pins
    SetPinFunc_SI2CSCL6_1();
    SetPinFunc_SI2CSDA6_1();
    
    // Clear structures
    PDL_ZERO_STRUCT(stcConfig);
    PDL_ZERO_STRUCT(stcIrqCb);
    PDL_ZERO_STRUCT(stcIrqEn);
    
    // Initialize I2C Slave interrupts
    stcIrqCb.pfnStatusIrqCb = I2csSlaveStatusIrqCallback;
    stcIrqEn.bStatusIrq = TRUE;
    
    // Initialize I2C Slave
    stcConfig.bEnableNoiseFilter = TRUE;
    stcConfig.bEnableReservedAddr = TRUE;
    stcConfig.bEnableSlaveAddr = TRUE;
    stcConfig.u8SlaveAddr = (0xA0u >> 1u); 
    stcConfig.bWaitSelection = FALSE;
    stcConfig.u8SetupTime = 0x80u;
    stcConfig.u8SlaveMaskAddr = (0xFFu >> 1u);
    stcConfig.pstcIrqCb = &stcIrqCb;
    stcConfig.pstcIrqEn = &stcIrqEn;
    stcConfig.bTouchNvic = TRUE;
  
    // Initialize key input
    Gpio1pin_InitIn(GPIO1PIN_P30, Gpio1pin_InitPullup(1u));
    
    // Initialize LED
    Gpio1pin_InitOut(GPIO1PIN_P3F, Gpio1pin_InitVal(1u));
    
    while(0u != Gpio1pin_Get(GPIO1PIN_P30));  // If key press, start test
    
    while(1)
    {
        if(Ok != I2cs_Init(&I2CS0, &stcConfig)) // Initialize I2C Slave before enter into RTC mode
        {
            while(1);
        }
      
        Lpm_GoToStandByMode(StbRtcMode, TRUE); 
        
        Gpio1pin_Put(GPIO1PIN_P3F, 0u);
        
        if(Ok != I2cs_DeInit(&I2CS0, TRUE))  // Reset I2C Slave after wakeup 
        {
            while(1);
        }
  
        if(bWakupFlag == TRUE)
        {
            bWakupFlag = FALSE;
            
            Gpio1pin_Put(GPIO1PIN_P3F, 0u);
            Delay(5);
            Gpio1pin_Put(GPIO1PIN_P3F, 1u);
        }
    }
}
