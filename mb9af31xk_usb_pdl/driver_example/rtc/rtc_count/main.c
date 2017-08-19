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
 ** This example demonstrates the calendar funciton of Real Time Clock. Every
 ** second the time and date are read, and when the alarm occurs, a LDE will
 ** start blinking.
 **
 ** History:
 **   - 2014-12-15  0.1  EZh         First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
#if ((SCM_CTL_Val & 0x08u) != 0x08u) // Sub clock not enable?
#error Before using this example, enable sub clock in system_fmx.h by setting bit 3 of \
definition "SCM_CTL_Val"  to 1!
#endif

#if !defined(GPIO1PIN_P61_INIT)
#error "P61 pin is used in this sample, if it is not available in your product,\
change to other GPIO!"
#endif
/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/
static uint32_t u32IntHalfSec = 0;
static uint8_t  u8AlarmOccur = 0;
static uint8_t  u8IntSec = 0;
static uint8_t  u8BlinkFlag = 0;
static uint8_t  u8BlinkPolling = 0;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
/**
 ******************************************************************************
 ** \brief Initializatio GPIO
 **
 ******************************************************************************/
static void PortInit(void)
{
   Gpio1pin_InitOut(GPIO1PIN_P61, Gpio1pin_InitVal(0u));
}

/**
 ******************************************************************************
 ** \brief Set port for LED
 **
 ******************************************************************************/
static void LedPolling(void)
{
    if(u8BlinkFlag == 1)
    {
        if(u32IntHalfSec == 1)
        {
            Gpio1pin_Put(GPIO1PIN_P61, u8BlinkPolling);
            u8BlinkPolling = ~u8BlinkPolling;
            u32IntHalfSec = 0;
        }
    }
}

/**
 ******************************************************************************
 ** \brief Alarm Interrupt
 **
 ******************************************************************************/
static void SampleRtcAlarmCb(void)
{
    u8AlarmOccur = 1;
    u8BlinkFlag = 1;
}

/**
 ******************************************************************************
 ** \brief 0.5-Second Interrupt
 **
 ******************************************************************************/
static void SampleRtcHalfSencondCb(void)
{
    u32IntHalfSec = 1;
}

/**
 ******************************************************************************
 ** \brief One Second Interrupt
 **
 ******************************************************************************/
static void SampleRtcOneSencondCb(void)
{
    // Set the one second interruption flag
    u8IntSec = 1;
}

/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    en_result_t enResult;
    stc_rtc_config_t stcRtcConfig;
    stc_rtc_irq_en_t stcIrqEn;
    stc_rtc_irq_cb_t stcIrqCb;
    stc_rtc_time_t   stcTimeDate;
    stc_rtc_alarm_t  stcAlarm;
    
    // Clear structures
    PDL_ZERO_STRUCT(stcRtcConfig);
    PDL_ZERO_STRUCT(stcTimeDate);
    PDL_ZERO_STRUCT(stcAlarm);
    PDL_ZERO_STRUCT(stcIrqEn);
    PDL_ZERO_STRUCT(stcIrqCb);

    // Initialize GPIO for LED
    PortInit();

    // Clear seconds interrupt flag
    u8IntSec = FALSE;

    // Time setting (23:59:00 1st of January 2014)
    stcTimeDate.u8Second = 0;                    // Second      : 00
    stcTimeDate.u8Minute = 59;                   // Minutes     : 59
    stcTimeDate.u8Hour   = 23;                   // Hour        : 23
    stcTimeDate.u8Day    = 30;                   // Date        : 30th
    stcTimeDate.u8Month  = RtcNovember;          // Month       : November
    stcTimeDate.u16Year   = 2014;                // Year        : 2014
    (void)Rtc_SetDayOfWeek(&stcTimeDate);        // Set Day of the Week in stcRtcTime

    // Alarm setting (00:00:00 1st of December 2014)
    stcAlarm.u8Minute = 0;              // Minutes : 00
    stcAlarm.u8Hour   = 0;              // Hour   : 00
    stcAlarm.u8Day    = 1;              // Date   : 1st
    stcAlarm.u8Month  = RtcDecember;    // Month  : December
    stcAlarm.u16Year   = 2014;          // Year   : 2014

    // Initilialize interrupts
    stcIrqEn.bHalfSecondIrq = 1u;
    stcIrqEn.bOneSecondIrq  = 1u;
    stcIrqEn.bAlarmIrq      = 1u;
    stcIrqCb.pfnHalfSecondIrqCb = SampleRtcHalfSencondCb;
    stcIrqCb.pfnOneSecondIrqCb  = SampleRtcOneSencondCb;
    stcIrqCb.pfnAlarmIrqCb      = SampleRtcAlarmCb;

    // Set time, alarm and interrupt structure pointer
#if (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEB)         
    stcRtcConfig.enClkSel = RtcUseSubClk;
    stcRtcConfig.u32ClkPrescaler = 32768;
#endif    
    stcRtcConfig.pstcTimeDate = &stcTimeDate;
    stcRtcConfig.pstcAlarm = &stcAlarm;
    stcRtcConfig.pstcIrqEn = &stcIrqEn;
    stcRtcConfig.pstcIrqCb = &stcIrqCb;

    stcRtcConfig.bRunNotInit = FALSE; // Don't initialize when RTC is running
    stcRtcConfig.bTouchNvic = TRUE;

    // Initialize the RTC
    enResult = Rtc_Init(&RTC0, &stcRtcConfig);

    if (Ok != enResult)
    {
#ifdef DEBUG_PRINT
        printf("Initial error!\n");
#endif
        while(1);
    }

    // Compare min,hour,day,month,year for Alarm
    Rtc_EnableFunc(&RTC0, RtcAlarmMinEn);
    Rtc_EnableFunc(&RTC0, RtcAlarmHourEn);
    Rtc_EnableFunc(&RTC0, RtcAlarmDayEn);
    Rtc_EnableFunc(&RTC0, RtcAlarmMonthEn);
    Rtc_EnableFunc(&RTC0, RtcAlarmYearEn);

    // Start RTC counting
    Rtc_EnableFunc(&RTC0, RtcCount);

    while (1)
    {
        // If one second interruption occurs
        if (TRUE == u8IntSec)
        {
            u8IntSec = FALSE;
            Rtc_ReadDateTime(&RTC0, &stcTimeDate);
        #ifdef DEBUG_PRINT
            // Print RTC to console
            printf("%04d/%02d/%02d %02d:%02d:%02d ",
                   stcTimeDate.u16Year,
                   stcTimeDate.u8Month,
                   stcTimeDate.u8Day,
                   stcTimeDate.u8Hour,
                   stcTimeDate.u8Minute,
                   stcTimeDate.u8Second);
            switch (stcTimeDate.u8DayOfWeek)
            {
                case RtcSunday:
                    printf("Sunday\n");
                    break;
                case RtcMonday:
                    printf("Monday\n");
                    break;
                case RtcTuesday:
                    printf("Tuesday\n");
                    break;
                case RtcWednesday:
                    printf("Wednesday\n");
                    break;
                case RtcThursday:
                    printf("Thursday\n");
                    break;
                case RtcFriday:
                    printf("Friday\n");
                    break;
                case RtcSaturday:
                    printf("Saturday\n");
                    break;
                default:
                    break;
            }
        #endif
        }

        if(u8AlarmOccur == 1)
        {
            u8AlarmOccur = 0;
        #ifdef DEBUG_PRINT
            printf("Alarm occurs!\n");
        #endif

            // Change time
            stcTimeDate.u8Second = 0;                    // Second      : 00
            stcTimeDate.u8Minute = 30;                   // Minutes     : 30
            stcTimeDate.u8Hour   = 9;                    // Hour        : 9

            Rtc_SetDateTime(&RTC0, &stcTimeDate, TRUE, TRUE, FALSE);

        #ifdef DEBUG_PRINT
            printf("New time is set\n");
        #endif
        }

        LedPolling();
    }
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
