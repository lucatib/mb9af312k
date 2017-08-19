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
 ** This example demonstrates RTC mode of low power consumption mode. Every
 ** 5 second the MCU is wakeup from RTC mode and polling a LED for 1 second.
 **
 ** History:
 **   - 2015-01-08  0.0.1  EZh         First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#if !defined(GPIO1PIN_P3F_INIT)
#error P3F is not available in this MCU product, \
change to other GPIO pin! Then delete "me" !
#endif

#if (PDL_PERIPHERAL_RTC_AVAILABLE != PDL_ON)
#error This example only supports the products which has RTC!
#endif

#if ((SCM_CTL_Val & 0x08) != 0x08)
#error Enable sub clock in system_fmx.h before using this example!
#endif

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/
uint8_t u8TimerIrqtFlag = 0;

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief Initializatio GPIO
 **
 ******************************************************************************/
static void PortInit(void)
{
   Gpio1pin_InitOut(GPIO1PIN_P3F, Gpio1pin_InitVal(1u));
}

/**
 ******************************************************************************
 ** \brief Set port for LED
 **
 ******************************************************************************/
static void SetLed(boolean_t bLed)
{
    if (0 == bLed)
    {
        Gpio1pin_Put(GPIO1PIN_P3F, 0);
    }
    else
    {
        Gpio1pin_Put(GPIO1PIN_P3F, 1);
    }
}

/**
 ******************************************************************************
 ** \brief RTC Timer Interrupt callback function
 **
 ******************************************************************************/
static void SampleRtcTimerCb(void)
{
    // Set timer interruption flag
    u8TimerIrqtFlag = TRUE;
}

/**
 ******************************************************************************
 ** \brief Configure and start RTC
 ******************************************************************************/
static void InitRtc(void)
{   
    stc_rtc_config_t stcRtcConfig;
    stc_rtc_irq_en_t stcRtcEn;
    stc_rtc_irq_cb_t stcRtcCb;
    stc_rtc_time_t stcRtcTime;
    stc_rtc_timer_t stcRtcTimer;
    
    // Clear structures
    PDL_ZERO_STRUCT(stcRtcConfig);
    PDL_ZERO_STRUCT(stcRtcEn);
    PDL_ZERO_STRUCT(stcRtcCb);
    PDL_ZERO_STRUCT(stcRtcTime);
    PDL_ZERO_STRUCT(stcRtcTimer);
    
    // Time setting (23:59:00 5th of January 2015)
    stcRtcTime.u8Second         = 0;                            // Second      : 00
    stcRtcTime.u8Minute         = 59;                           // Minutes     : 59
    stcRtcTime.u8Hour           = 23;                           // Hour        : 23
    stcRtcTime.u8Day            = 5;                            // Date        : 5th
    stcRtcTime.u8DayOfWeek      = RtcMonday;                    // Week        : monday       
    stcRtcTime.u8Month          = RtcJanuary;                   // Month       : January
    stcRtcTime.u16Year          = 2015;                         // Year        : 2015
    
    // Intialize RTC timer
    stcRtcTimer.enMode = RtcTimerPeriod;
    stcRtcTimer.u32TimerCycle = 5;                              // Generate interrupt every 5s
    
    // Configure interrupt
    stcRtcEn.bTimerIrq = TRUE;
    stcRtcCb.pfnTimerIrqCb = SampleRtcTimerCb;
    
    // Clear seconds interrupt flag
    u8TimerIrqtFlag = FALSE;

    // Initialize RTC configuration
    stcRtcConfig.bEnSuboutDivider = FALSE;
    stcRtcConfig.enRtccoSel = RtccoOutput1Hz;
#if (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEB)
    stcRtcConfig.enClkSel = RtcUseSubClk;
    stcRtcConfig.u32ClkPrescaler = __CLKSO;                      // sub clock 
#endif
    stcRtcConfig.pstcTimeDate = &stcRtcTime;
    stcRtcConfig.pstcTimer = &stcRtcTimer;
    stcRtcConfig.pstcIrqEn = &stcRtcEn;
    stcRtcConfig.pstcIrqCb = &stcRtcCb;  
    stcRtcConfig.bTouchNvic = TRUE;
    
    // Initialize the RTC
    Rtc_Init(&RTC0, &stcRtcConfig);
    
    // Start RTC
    Rtc_EnableFunc(&RTC0, RtcCount);
    
    // Enable timer
    Rtc_EnableFunc(&RTC0, RtcTimer);
}
/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    PortInit();
    /* Turn off LED */
    SetLed(0);
    SetLed(1);
    /* Configure and start RTC */
    InitRtc();

    while(1)
    {
        /* Enter RTC mode */
        Lpm_GoToStandByMode(StbRtcMode, TRUE);
        if(u8TimerIrqtFlag == TRUE) // wakeup here!, RTC interrupt flag is set in the callback function
        {
            u8TimerIrqtFlag = FALSE;
            /* Turn on LED */
            SetLed(0);
            Rtc_ClrIrqFlag(&RTC0, RtcHalfSecondIrq);
            while(Rtc_GetIrqFlag(&RTC0, RtcHalfSecondIrq) != TRUE); //  Wait 0.5s
            /* Turn off LED */
            SetLed(1);
            Rtc_ClrIrqFlag(&RTC0, RtcHalfSecondIrq);
            while(Rtc_GetIrqFlag(&RTC0, RtcHalfSecondIrq) != TRUE); //  Wait 0.5s
        }
    }
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
