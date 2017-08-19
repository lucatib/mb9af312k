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
 ** This example tests the SD card block read/write via SDIO interface.
 **
 ** History:
 **   - 2014-11-10  1.0  EZh       First release version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include <string.h>
#include "pdl_header.h"

/******************************************************************************
 * Local definitions
 ******************************************************************************/
#define SECTORNUM  (8)
#define TOTAL_SIZE (100*1024*1024)

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static uint8_t      u8RxBuff[BLOCK_SZ*SECTORNUM];
static uint8_t      u8TxBuff[BLOCK_SZ*SECTORNUM];
static uint8_t      gu8StartCh      = 0x5;
static uint32_t     gu8StartSector  = 0;
uint32_t gcount = 0;

/******************************************************************************
 * Local type definitions
 ******************************************************************************/
typedef enum en_test_pin
{
    TEST_GPIO_PIN1,
    TEST_GPIO_PIN2,
    LED_B,
    LED_G,
    LED_R,
    
}en_test_pin_t;

/**
 ******************************************************************************
 ** \brief Test pin initialization.
 ******************************************************************************/
void TestPin_init( void )
{
    //set P10
    Gpio1pin_InitOut(GPIO1PIN_P10, Gpio1pin_InitVal(0u));

    //set P14
    Gpio1pin_InitOut(GPIO1PIN_P14, Gpio1pin_InitVal(0u));

    //LED_B
    Gpio1pin_InitOut(GPIO1PIN_PE0, Gpio1pin_InitVal(0u));

    //LED_G
    Gpio1pin_InitOut(GPIO1PIN_P38, Gpio1pin_InitVal(0u));

    //LED_R
    Gpio1pin_InitOut(GPIO1PIN_P27, Gpio1pin_InitVal(0u));
}

/**
 ******************************************************************************
 ** \brief Test pin set
 **
 ** \param u32Idx test pin index
 ** \param bOn    Pin level
 **
 ******************************************************************************/
void TestPin_Set( uint32_t u32Idx, boolean_t bOn )
{
    switch ( u32Idx )
    {
        case TEST_GPIO_PIN1:
            Gpio1pin_Put(GPIO1PIN_P10, bOn);
            break;
        case TEST_GPIO_PIN2:
            Gpio1pin_Put(GPIO1PIN_P14, bOn);
            break;
        case LED_B:
            Gpio1pin_Put(GPIO1PIN_PE0, bOn);
            break;
        case LED_G:
            Gpio1pin_Put(GPIO1PIN_P38, bOn);
            break;
        case LED_R:
            Gpio1pin_Put(GPIO1PIN_P27, bOn);
            break;
        default:
            break;
    }
}

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main( void )
{
    en_result_t         enRet           = Ok;
    uint32_t            i               = 0;
    boolean_t           bLedOn          = 0;
    boolean_t           bSuccess        = TRUE;
    stc_sdcard_config_t stcSdcardCfg ;
    uint8_t             u8ch            = 0;

    PDL_ZERO_STRUCT(stcSdcardCfg);

    stcSdcardCfg.bHighSpeedMode = TRUE;
    stcSdcardCfg.bUsingADMA = TRUE;
    stcSdcardCfg.enBusWidth = SdBusWidth4;
    stcSdcardCfg.u32Clock = SdClk20M;

    TestPin_init();

    enRet = Sdcard_Init(&stcSdcardCfg);

    if (Ok != enRet)
    {
        while(1); // Initialization failed
    }

    //test start
    memset(&u8TxBuff, 0x0, sizeof(u8TxBuff));

    u8ch = gu8StartCh;

#if 1
    //write test.
    for ( i = 0; i <  TOTAL_SIZE / (BLOCK_SZ * SECTORNUM); i++ )
    {
        //build data
        memset(u8TxBuff, u8ch, BLOCK_SZ*SECTORNUM);
        u8ch++;

        if(2048 == i)
        {
          i = i;
        }
        enRet = Sdcard_Write_MultiBlock(&stcSdcardCfg,
                                gu8StartSector + i * SECTORNUM,
                                SECTORNUM,
                                u8TxBuff);

        if (Ok != enRet)
        {
            bSuccess = FALSE;
            break;
        }
        if (i % 100 == 0)
        {
            bLedOn = !bLedOn;
            TestPin_Set(LED_B, bLedOn);
        }
        
        gcount = i;
    }
#endif

#if 1
    u8ch = gu8StartCh;
    //read back & check.
    for ( i = 0; i < TOTAL_SIZE / (BLOCK_SZ * SECTORNUM); i++ )
    {
        memset(u8RxBuff, 0x0, sizeof(u8RxBuff));

        enRet = Sdcard_Read_MultiBlock(&stcSdcardCfg,
                                       gu8StartSector + i * SECTORNUM,
                                       SECTORNUM,
                                       u8RxBuff);

        if (Ok != enRet)
        {
            bSuccess = FALSE;
            break;
        }
        else
        {
            //check value
            memset(u8TxBuff, u8ch, BLOCK_SZ*SECTORNUM);
            u8ch++;

            if (memcmp(u8RxBuff, u8TxBuff, BLOCK_SZ*SECTORNUM))
            {
                bSuccess = FALSE;
                break;
            }
        }

        if (i % 100 == 0)
        {
            bLedOn = !bLedOn;
            TestPin_Set(LED_G, bLedOn);
        }

    }
#endif
    
    if (bSuccess == TRUE)  
    {   
        // After test success, green LED is on
        TestPin_Set(LED_R, 1);  
        TestPin_Set(LED_G, 0);
        TestPin_Set(LED_B, 1);
    }
    else
    {
        // After test fail, red LED is on
        TestPin_Set(LED_R, 0);
    }
    
    while (1);
}