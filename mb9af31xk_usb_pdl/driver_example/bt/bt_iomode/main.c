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
 ** This example demonstrates BT IO mode 1~7.
 **
 ** History:
 **   - 2014-07-23  0.0.1  KXi        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/
/*! \brief Demo BT IO mode 1 */
#define  DEMO_BT_IOMODE_1               0
/*! \brief Demo BT IO mode 2,3  */
#define  DEMO_BT_IOMODE_2_3             1
/*! \brief Demo BT IO mode 2,8  */
#define  DEMO_BT_IOMODE_2_8             2
/*! \brief Demo BT IO mode 4  */
#define  DEMO_BT_IOMODE_4               3
/*! \brief Demo BT IO mode 5  */
#define  DEMO_BT_IOMODE_5               4
/*! \brief Demo BT IO mode 6  */
#define  DEMO_BT_IOMODE_6               5
/*! \brief Demo BT IO mode 7  */
#define  DEMO_BT_IOMODE_7               6

/*! \brief Select a demo mode  */
#define  DEMO_MODE                      DEMO_BT_IOMODE_1

/*---------------------------------------------------------------------------*/
/* global data                                                               */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* global functions                                                          */
/*---------------------------------------------------------------------------*/

#if (DEMO_MODE == DEMO_BT_IOMODE_1)

#if !defined(SetPinFunc_TIOA0_1_OUT) || !defined(SetPinFunc_TIOB0_1_IN) || \
    !defined(SetPinFunc_TIOA1_1_IN) || !defined(SetPinFunc_TIOB1_1_IN)
#error TIOA0_1, TIOA1_1 or TIOB1_1 pins are not available in this MCU product, \
change to other re-location pins or BT channels! Then delete "me" !
#endif

/*!
 ******************************************************************************
 ** \brief BT IO mode 1 demo code
 ******************************************************************************
 */
static void DemoBtIoMode1(void)
{
    stc_bt_pwm_config_t stcPwmConfig;
#ifdef DEBUG_PRINT 
    printf("==================================================\n");
    printf("Base Timer IO Mode 1 \n");
    printf("==================================================\n");
    printf("ECK  ------ TIOB1\n");
    printf("TGIN ------ TIOA1\n");
    printf("TIN  ------ TIOB0\n");
    printf("TOUT ------ TIOA0\n");
    printf("==================================================\n");
#endif
    // Set Basetimer IO port
    SetPinFunc_TIOA0_1_OUT();
    SetPinFunc_TIOB0_1_IN();
    SetPinFunc_TIOA1_1_IN();
    SetPinFunc_TIOB1_1_IN();
    

    // Set requested I/O mode
    Bt_ConfigIOMode(&BT0, BtIoMode1);


    // Initialize PWM timer
    stcPwmConfig.enPres = PwmPres1Div4;  // PWM clock = 5MHz @ PCLK = 20MHz
    stcPwmConfig.enMode = PwmContinuous;
    stcPwmConfig.enExtTrig = PwmExtTrigBoth;
    stcPwmConfig.enOutputMask = PwmOutputNormal;
    stcPwmConfig.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig.enRestartEn = PwmRestartDisable;
    Bt_Pwm_Init(&BT0, &stcPwmConfig);

    // Set cycle and duty value
    Bt_Pwm_WriteCycleVal(&BT0, 999); // Cycle = (1+m)*PWM clock cycle = 200us
    Bt_Pwm_WriteDutyVal(&BT0, 199);  // Duty = (1+m)*PWM clock cycle = 40us
#ifdef DEBUG_PRINT
    printf("Give the TIOA1_1 low triger\n");
    printf("PWM wave output from TIOA0_1\n");
#endif

    // Enable count operatoin
    Bt_Pwm_EnableCount(&BT0);

    while(1);
}
#endif

#if (DEMO_MODE == DEMO_BT_IOMODE_2_3)

#if !defined(SetPinFunc_TIOA0_0_OUT) || !defined(SetPinFunc_TIOB0_0_IN)  || \
    !defined(SetPinFunc_TIOA1_0_OUT) || !defined(SetPinFunc_TIOB1_0_IN)  || \
    !defined(SetPinFunc_TIOA2_0_OUT) || !defined(SetPinFunc_TIOB2_0_IN)  || \
    !defined(SetPinFunc_TIOA3_1_OUT)
#error TIOA0_1, TIOA1_1 or TIOB1_1 pins are not available in this MCU product, \
change to other re-location pins or BT channels! Then delete "me!
#endif

/*!
 ******************************************************************************
 ** \brief BT IO mode 2,3 demo code
 ******************************************************************************
 */
static void DemoBtIoMode23(void)
{
    stc_bt_pwm_config_t stcPwmConfig;
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Base Timer IO Mode 2 \n");
    printf("==================================================\n");
    printf("ECK  ------ TIOB1\n");
    printf("TGIN ------ TIOA1\n");
    printf("TIN  ------ TIOB0\n");
    printf("TOUT ------ TIOA0\n");
    printf("==================================================\n");
    printf("Base Timer IO Mode 3 \n");
    printf("==================================================\n");
    printf("ECK  ------ TIOB1\n");
    printf("TGIN ------ TIOA1\n");
    printf("TIN  ------ TIOB0\n");
    printf("TOUT ------ TIOA0\n");
#endif

    // Set Basetimer IO port
    SetPinFunc_TIOA0_0_OUT();
    SetPinFunc_TIOB0_0_IN();
    SetPinFunc_TIOA1_0_OUT();
    SetPinFunc_TIOB1_0_IN();
    
    SetPinFunc_TIOA2_0_OUT();
    SetPinFunc_TIOB2_0_IN();
    SetPinFunc_TIOA3_1_OUT();
    //SetPinFunc_TIOB3_0_IN();

    // Set requested I/O mode
    Bt_ConfigIOMode(&BT0, BtIoMode2);
    Bt_ConfigIOMode(&BT2, BtIoMode3);

    // Initialize PWM timer
    stcPwmConfig.enPres = PwmPres1Div4;  // PWM clock = 5MHz @ PCLK = 20MHz
    stcPwmConfig.enMode = PwmContinuous;
    stcPwmConfig.enExtTrig = PwmExtTrigBoth;
    stcPwmConfig.enOutputMask = PwmOutputNormal;
    stcPwmConfig.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig.enRestartEn = PwmRestartDisable;

    Bt_Pwm_Init(&BT0, &stcPwmConfig);
    Bt_Pwm_Init(&BT1, &stcPwmConfig);
    Bt_Pwm_Init(&BT2, &stcPwmConfig);
    Bt_Pwm_Init(&BT3, &stcPwmConfig);

    // Set cycle and duty value
    Bt_Pwm_WriteCycleVal(&BT0, 999); // Cycle = (1+m)*PWM clock cycle = 200us
    Bt_Pwm_WriteDutyVal(&BT0, 199);  // Duty = (1+m)*PWM clock cycle = 40us
    Bt_Pwm_WriteCycleVal(&BT1, 999); // Cycle = (1+m)*PWM clock cycle = 200us
    Bt_Pwm_WriteDutyVal(&BT1, 199);  // Duty = (1+m)*PWM clock cycle = 40us
    Bt_Pwm_WriteCycleVal(&BT2, 999); // Cycle = (1+m)*PWM clock cycle = 200us
    Bt_Pwm_WriteDutyVal(&BT2, 199);  // Duty = (1+m)*PWM clock cycle = 40us
    Bt_Pwm_WriteCycleVal(&BT3, 999); // Cycle = (1+m)*PWM clock cycle = 200us
    Bt_Pwm_WriteDutyVal(&BT3, 199);  // Duty = (1+m)*PWM clock cycle = 40us
#ifdef DEBUG_PRINT
    printf("TIOA0~TIOA3 shares same trigger.\n");
    printf("Trigger->TIOB0->COUT->CIN(BT2)->COUT(BT2)\n");  
    printf("Give the TIOB0_0 low triger\n");
    printf("PWM wave output from TIOA0_0,TIOA1_0,TIOA2_0,TIOA3_1\n");
#endif
    // Enable count operatoin
    Bt_Pwm_EnableCount(&BT0);
    Bt_Pwm_EnableCount(&BT1);
    Bt_Pwm_EnableCount(&BT2);
    Bt_Pwm_EnableCount(&BT3);

    while(1);

}
#endif

#if (DEMO_MODE == DEMO_BT_IOMODE_2_8)

#if !defined(SetPinFunc_TIOA0_0_OUT) || !defined(SetPinFunc_TIOB0_0_IN) || \
    !defined(SetPinFunc_TIOA1_0_OUT) || !defined(SetPinFunc_TIOB1_0_IN) || \
    !defined(SetPinFunc_TIOA2_0_OUT) || !defined(SetPinFunc_TIOB2_0_IN) || \
    !defined(SetPinFunc_TIOA3_1_OUT)
#error TIOA0_0, TIOA1_0, TIOA2_0, TIOB1_0, TIOA3_1, TIOB1_0  or TIOB2_0 pins are not available in this MCU product, \
change to other re-location pins or BT channels! Then delete "me" !
#endif

/*!
 ******************************************************************************
 ** \brief BT IO mode 2,3,8 demo code
 ******************************************************************************
 */
static void DemoBtIoMode28(void)
{
    stc_bt_pwm_config_t stcPwmConfig;
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Base Timer IO Mode 2 \n");
    printf("==================================================\n");
    printf("ECK  ------ TIOB1\n");
    printf("TGIN ------ TIOA1\n");
    printf("TIN  ------ TIOB0\n");
    printf("TOUT ------ TIOA0\n");
    printf("==================================================\n");
    printf("Base Timer IO Mode 8 \n");
    printf("==================================================\n");
    printf("ECK  ------ TIOB3\n");
    printf("TGIN ------ TIOA3\n");
    printf("TIN  ------ TIOB2\n");
    printf("TOUT ------ TIOA2\n");
    printf("==================================================\n");
#endif

    // Set Basetimer IO port
    SetPinFunc_TIOA0_0_OUT();
    SetPinFunc_TIOB0_0_IN();
    SetPinFunc_TIOA1_0_OUT();
    SetPinFunc_TIOB1_0_IN();

    SetPinFunc_TIOA2_0_OUT();
    SetPinFunc_TIOB2_0_IN();
    SetPinFunc_TIOA3_1_OUT();

    // Set requested I/O mode
    Bt_ConfigIOMode(&BT0, BtIoMode2);
    Bt_ConfigIOMode(&BT2, BtIoMode8);

    // Initialize PWM timer
    stcPwmConfig.enPres = PwmPres1Div4;  // PWM clock = 5MHz @ PCLK = 20MHz
    stcPwmConfig.enMode = PwmContinuous;
    stcPwmConfig.enExtTrig = PwmExtTrigRising;
    stcPwmConfig.enOutputMask = PwmOutputNormal;
    stcPwmConfig.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig.enRestartEn = PwmRestartEnable;

    Bt_Pwm_Init(&BT0, &stcPwmConfig);
    Bt_Pwm_Init(&BT1, &stcPwmConfig);
    Bt_Pwm_Init(&BT2, &stcPwmConfig);
    Bt_Pwm_Init(&BT3, &stcPwmConfig);

    // Set cycle and duty value
    Bt_Pwm_WriteCycleVal(&BT0, 5999); // Cycle = (1+m)*PWM clock cycle = 1.2ms
    Bt_Pwm_WriteDutyVal(&BT0, 2999);  // Duty = (1+m)*PWM clock cycle = 0.6ms
    Bt_Pwm_WriteCycleVal(&BT1, 5999); // Cycle = (1+m)*PWM clock cycle = 1.2ms
    Bt_Pwm_WriteDutyVal(&BT1, 2999);  // Duty = (1+m)*PWM clock cycle = 0.6ms
    Bt_Pwm_WriteCycleVal(&BT2, 999); // Cycle = (1+m)*PWM clock cycle = 200us
    Bt_Pwm_WriteDutyVal(&BT2, 199);  // Duty = (1+m)*PWM clock cycle = 40us
    Bt_Pwm_WriteCycleVal(&BT3, 999); // Cycle = (1+m)*PWM clock cycle = 200us
    Bt_Pwm_WriteDutyVal(&BT3, 199);  // Duty = (1+m)*PWM clock cycle = 40us
#ifdef DEBUG_PRINT
    printf("Connect TIOB0 with P0F. \n");
    printf("TIOA2~TIOA7 shares same trigger.\n");
    printf("Trigger->TIOB0->COUT->CIN(BT2)->COUT(BT2)\n");
    printf("Give the TIOB0_0 low triger\n");
    printf("2) PWM output from TIOA0~TIOA2,");
    printf("TIOA3_1\n");
    printf("Give the TIOB0_0 high triger\n");
#endif
    // Enable count operatoin
    Bt_Pwm_EnableCount(&BT0);
    Bt_Pwm_EnableCount(&BT1);
    Bt_Pwm_EnableCount(&BT2);
    Bt_Pwm_EnableCount(&BT3);

    while(1);

}
#endif

#if (DEMO_MODE == DEMO_BT_IOMODE_4)

#if !defined(SetPinFunc_TIOA0_0_OUT) || !defined(SetPinFunc_TIOB0_0_IN) || \
    !defined(SetPinFunc_TIOA1_0_OUT)
#error TIOA0_0, TIOB0_0 or TIOA1_0 pins are not available in this MCU product, \
change to other re-location pins or BT channels! Then delete "me" !
#endif

/*!
 ******************************************************************************
 ** \brief BT IO mode 4 demo code
 ******************************************************************************
 */
static void DemoBtIoMode4(void)
{
    stc_bt_pwm_config_t stcPwmConfig;
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Base Timer IO Mode 4 \n");
    printf("==================================================\n");
#endif

    // Set Basetimer IO port
    SetPinFunc_TIOA0_0_OUT();
    SetPinFunc_TIOB0_0_IN();
    SetPinFunc_TIOA1_0_OUT();

    // Set requested I/O mode
    Bt_ConfigIOMode(&BT0, BtIoMode4);

    // Initialize PWM timer
    stcPwmConfig.enPres = PwmPres1Div4;  // PWM clock = 5MHz @ PCLK = 20MHz
    stcPwmConfig.enMode = PwmContinuous;
    stcPwmConfig.enExtTrig = PwmExtTrigRising;
    stcPwmConfig.enOutputMask = PwmOutputNormal;
    stcPwmConfig.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig.enRestartEn = PwmRestartEnable;
    Bt_Pwm_Init(&BT0, &stcPwmConfig);
    Bt_Pwm_Init(&BT1, &stcPwmConfig);

    // Set cycle and duty value
    Bt_Pwm_WriteCycleVal(&BT0, 59999); // Cycle = (1+m)*PWM clock cycle = 12ms
    Bt_Pwm_WriteDutyVal(&BT0, 29999);  // Duty = (1+m)*PWM clock cycle = 6ms
    Bt_Pwm_WriteCycleVal(&BT1, 399); // Cycle = (1+m)*PWM clock cycle = 80us
    Bt_Pwm_WriteDutyVal(&BT1, 199);  // Duty = (1+m)*PWM clock cycle = 40us

    // Enable count operatoin
    Bt_Pwm_EnableCount(&BT0);
    Bt_Pwm_EnableCount(&BT1);

    // Software trigger
    Bt_Pwm_EnableSwTrig(&BT0);
#ifdef DEBUG_PRINT
    printf("PWM wave output from TIOA0_0,TIOA1_0\n");
#endif
    while(1);
}
#endif

#if (DEMO_MODE == DEMO_BT_IOMODE_5)

#if !defined(SetPinFunc_TIOA0_0_OUT) || !defined(SetPinFunc_TIOA1_0_OUT)
#error TIOA0_0 or TIOA1_0 pins are not available in this MCU product, \
change to other re-location pins or BT channels! Then delete "me" !
#endif

/*!
 ******************************************************************************
 ** \brief BT IO mode 5 demo code
 ******************************************************************************
 */
static void DemoBtIoMode5(void)
{
    stc_bt_pwm_config_t stcPwmConfig;
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Base Timer IO Mode 5 \n");
    printf("=================================================\n");
#endif

    // Set Basetimer IO port
    SetPinFunc_TIOA0_0_OUT();
    SetPinFunc_TIOA1_0_OUT();
    // Set requested I/O mode
    Bt_ConfigIOMode(&BT0, BtIoMode5);

    // Initialize PWM timer
    stcPwmConfig.enPres = PwmPres1Div4;  // PWM clock = 20MHz @ PCLK = 80MHz
    stcPwmConfig.enMode = PwmContinuous;
    stcPwmConfig.enExtTrig = PwmExtTrigRising;
    stcPwmConfig.enOutputMask = PwmOutputNormal;
    stcPwmConfig.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig.enRestartEn = PwmRestartEnable;
    Bt_Pwm_Init(&BT0, &stcPwmConfig);
    Bt_Pwm_Init(&BT1, &stcPwmConfig);

    // Set cycle and duty value
    Bt_Pwm_WriteCycleVal(&BT0, 399); // Cycle = (1+m)*PWM clock cycle = 20us
    Bt_Pwm_WriteDutyVal(&BT0, 199);  // Duty = (1+m)*PWM clock cycle = 10us
    Bt_Pwm_WriteCycleVal(&BT1, 399); // Cycle = (1+m)*PWM clock cycle = 20us
    Bt_Pwm_WriteDutyVal(&BT1, 199);  // Duty = (1+m)*PWM clock cycle = 10us

    // Enable count operatoin
    Bt_Pwm_EnableCount(&BT0);
    Bt_Pwm_EnableCount(&BT1);

    // Enable Simultaneously
    Bt_SetSimultaneousStart(0x0003);
#ifdef DEBUG_PRINT
    printf("PWM wave output from TIOA0_0,TIOA1_0\n");
#endif
    while(1);
}
#endif

#if (DEMO_MODE == DEMO_BT_IOMODE_6)

#if !defined(SetPinFunc_TIOA0_0_OUT) || !defined(SetPinFunc_TIOA1_0_OUT)
#error TIOA0_0 or TIOA1_0 pins are not available in this MCU product, \
change to other re-location pins or BT channels! Then delete "me" !
#endif

/*!
 ******************************************************************************
 ** \brief BT IO mode 6 demo code
 ******************************************************************************
 */
static void DemoBtIoMode6(void)
{
    stc_bt_pwm_config_t stcPwmConfig;
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Base Timer IO Mode 6 \n");
    printf("==================================================\n");
#endif

    // Set Basetimer IO port
    SetPinFunc_TIOA0_0_OUT();
    SetPinFunc_TIOA1_0_OUT();

    // Set requested I/O mode
    Bt_ConfigIOMode(&BT0, BtIoMode6);


    // Initialize PWM timer
    stcPwmConfig.enPres = PwmPres1Div4;  // PWM clock = 5MHz @ PCLK = 20MHz
    stcPwmConfig.enMode = PwmContinuous;
    stcPwmConfig.enExtTrig = PwmExtTrigRising;
    stcPwmConfig.enOutputMask = PwmOutputNormal;
    stcPwmConfig.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig.enRestartEn = PwmRestartEnable;
    Bt_Pwm_Init(&BT0, &stcPwmConfig);
    Bt_Pwm_Init(&BT1, &stcPwmConfig);

    // Set cycle and duty value
    Bt_Pwm_WriteCycleVal(&BT0, 59999); // Cycle = (1+m)*PWM clock cycle = 12ms
    Bt_Pwm_WriteDutyVal(&BT0, 29999);  // Duty = (1+m)*PWM clock cycle = 6ms
    Bt_Pwm_WriteCycleVal(&BT1, 399); // Cycle = (1+m)*PWM clock cycle = 80us
    Bt_Pwm_WriteDutyVal(&BT1, 199);  // Duty = (1+m)*PWM clock cycle = 40us

    // Enable count operatoin
    Bt_Pwm_EnableCount(&BT0);
    Bt_Pwm_EnableCount(&BT1);

    // Software trigger
    Bt_SetSimultaneousStart(0x0001);
#ifdef DEBUG_PRINT
    printf("PWM wave output from TIOA0_0,TIOA1_0\n");
#endif
    while(1);

}
#endif

#if (DEMO_MODE == DEMO_BT_IOMODE_7)

#if !defined(SetPinFunc_TIOA0_0_OUT) || !defined(SetPinFunc_TIOA1_0_OUT)
#error TIOA0_0 or TIOA1_0 pins are not available in this MCU product, \
change to other re-location pins or BT channels! Then delete "me" !
#endif

/*!
 ******************************************************************************
 ** \brief BT IO mode 7 demo code
 ******************************************************************************
 */
static void DemoBtIoMode7(void)
{
    stc_bt_pwm_config_t stcPwmConfig0, stcPwmConfig1;
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Base Timer IO Mode 7 \n");
    printf("==================================================\n");
#endif

    // Set Basetimer IO port
    SetPinFunc_TIOA0_0_OUT();
    SetPinFunc_TIOA1_0_OUT();

    // Set requested I/O mode
    Bt_ConfigIOMode(&BT0, BtIoMode7);

    // Initialize PWM timer
    stcPwmConfig0.enPres = PwmPres1Div4;  // PWM clock = 5MHz @ PCLK = 20MHz
    stcPwmConfig0.enMode = PwmContinuous;
    stcPwmConfig0.enExtTrig = PwmExtTrigRising;
    stcPwmConfig0.enOutputMask = PwmOutputNormal;
    stcPwmConfig0.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig0.enRestartEn = PwmRestartEnable;

    stcPwmConfig1.enPres = PwmPres1Div4;  // PWM clock = 5MHz @ PCLK = 20MHz
    stcPwmConfig1.enMode = PwmOneshot;
    stcPwmConfig1.enExtTrig = PwmExtTrigDisable;
    stcPwmConfig1.enOutputMask = PwmOutputNormal;
    stcPwmConfig1.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig1.enRestartEn = PwmRestartEnable;

    Bt_Pwm_Init(&BT0, &stcPwmConfig1);
    Bt_Pwm_Init(&BT1, &stcPwmConfig0);

    // Set cycle and duty value
    Bt_Pwm_WriteCycleVal(&BT0, 59999); // Cycle = (1+m)*PWM clock cycle = 12ms
    Bt_Pwm_WriteDutyVal(&BT0, 29999);  // Duty = (1+m)*PWM clock cycle = 6ms
    Bt_Pwm_WriteCycleVal(&BT1, 399); // Cycle = (1+m)*PWM clock cycle = 80us
    Bt_Pwm_WriteDutyVal(&BT1, 199);  // Duty = (1+m)*PWM clock cycle = 40us

    // Enable count operatoin
    Bt_Pwm_EnableCount(&BT0);
    Bt_Pwm_EnableCount(&BT1);

    // Software trigger
    Bt_Pwm_EnableSwTrig(&BT0); // trigger start after 1.5ms
#ifdef DEBUG_PRINT
    printf("PWM wave output from TIOA1_0\n");
#endif
    while(1);
}
#endif

/*!
 ******************************************************************************
 ** \brief BT IO mode example code
 ******************************************************************************
 */
int32_t main(void)
{
#if (DEMO_MODE == DEMO_BT_IOMODE_1)
    DemoBtIoMode1();
#endif
#if (DEMO_MODE == DEMO_BT_IOMODE_2_3)
    DemoBtIoMode23();
#endif
#if (DEMO_MODE == DEMO_BT_IOMODE_2_8)
    DemoBtIoMode28();
#endif
#if (DEMO_MODE == DEMO_BT_IOMODE_4)
    DemoBtIoMode4();
#endif
#if (DEMO_MODE == DEMO_BT_IOMODE_5)
    DemoBtIoMode5();
#endif
#if (DEMO_MODE == DEMO_BT_IOMODE_6)
    DemoBtIoMode6();
#endif
#if (DEMO_MODE == DEMO_BT_IOMODE_7)
    DemoBtIoMode7();
#endif

    while(1);
}

/*****************************************************************************/
/* END OF FILE */
