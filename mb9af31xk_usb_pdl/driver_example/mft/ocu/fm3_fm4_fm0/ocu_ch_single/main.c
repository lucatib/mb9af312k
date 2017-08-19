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
 ** This example demonstrates single channel mode of OCU.
 ** Ch.0 and Ch.1 implement 1-change mode
 ** Ch.2 and Ch.3 implement active-high mode
 ** Ch.4 and Ch.5 implement active-low mode
 **
 ** Uncomment line 319, 320 to test ch.0 and ch.1 with 1-change mode
 ** Uncomment line 323, 324 to test ch.2 and ch.3 with active-high mode
 ** Uncomment line 327, 328 to test ch.4 and ch.5 with active-low mode
 **
 ** History:
 **   - 2014-11-07  0.0.1  EZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/
/*  FRT module  */
// MFT channel
#define USER_MFT_FRT          MFT0_FRT
// FRT 0
#define USER_FRT_CH0          MFT_FRT_CH0
#define USER_FRT_CH0_MOD      FrtUpCount
// FRT 1
#define USER_FRT_CH1          MFT_FRT_CH1 
#define USER_FRT_CH1_MOD      FrtUpDownCount
// FRT cycle
#define FRT_CYCLE       60000  

/*  OCU module  */
// MFT channel
#define USER_MFT_OCU        MFT0_OCU
// OCU compare value
#define USER_OCU_CH0_CMPVAL1 (5000)
#define USER_OCU_CH1_CMPVAL1 (15000)
#define USER_OCU_CH2_CMPVAL1 (20000)
#define USER_OCU_CH3_CMPVAL1 (30000)
#define USER_OCU_CH4_CMPVAL1 (40000)
#define USER_OCU_CH5_CMPVAL1 (50000)

/*---------------------------------------------------------------------------*/
/* local data                                                                */
/*---------------------------------------------------------------------------*/
static uint8_t m_u8OcuMatchFlag[6] = {0};

/*---------------------------------------------------------------------------*/
/* local functions prototypes                                                */
/*---------------------------------------------------------------------------*/
/**
 ******************************************************************************
 ** \brief  OCU channel 0 interrupt callback function   .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void Ocu0IntCallback(void)
{
    m_u8OcuMatchFlag[0] = 1;
}

/**
 ******************************************************************************
 ** \brief  OCU channel 1 interrupt callback function  .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void Ocu1IntCallback(void)
{
    m_u8OcuMatchFlag[1] = 1;
}

/**
 ******************************************************************************
 ** \brief  OCU channel 2 interrupt callback function   .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void Ocu2IntCallback(void)
{
    m_u8OcuMatchFlag[2] = 1;
}

/**
 ******************************************************************************
 ** \brief  OCU channel 3 interrupt callback function   .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void Ocu3IntCallback(void)
{
    m_u8OcuMatchFlag[3] = 1;
}

/**
 ******************************************************************************
 ** \brief  OCU channel 4 interrupt callback function   .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void Ocu4IntCallback(void)
{
    m_u8OcuMatchFlag[4] = 1;
}

/**
 ******************************************************************************
 ** \brief  OCU channel 5 interrupt callback function   .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void Ocu5IntCallback(void)
{
    m_u8OcuMatchFlag[5] = 1;
}

/**
 ******************************************************************************
 ** \brief  Init user free run timer   .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void InitFrt(void)
{
    stc_mft_frt_config_t stcFrtConfig0; //channel 1 configure structure
    stc_mft_frt_config_t stcFrtConfig1; //channel 1 configure structure
    
    // Clear structure
    PDL_ZERO_STRUCT(stcFrtConfig0);
    PDL_ZERO_STRUCT(stcFrtConfig1);
    
    //configure the FRT channel 0 parameter
    stcFrtConfig0.enFrtMode = USER_FRT_CH0_MOD;
    stcFrtConfig0.enFrtClockDiv = FrtPclkDiv128;
    stcFrtConfig0.bEnExtClock = FALSE;
    stcFrtConfig0.bEnBuffer = TRUE;
    
    //configure the FRT channel 1 parameter
    stcFrtConfig1.enFrtMode = USER_FRT_CH1_MOD;
    stcFrtConfig1.enFrtClockDiv = FrtPclkDiv256;
    stcFrtConfig1.bEnExtClock = FALSE;
    stcFrtConfig1.bEnBuffer = TRUE;

    //stop frt
    Mft_Frt_Stop(&USER_MFT_FRT,USER_FRT_CH0);
    Mft_Frt_Stop(&USER_MFT_FRT,USER_FRT_CH1);
    //init frt module
    Mft_Frt_Init(&USER_MFT_FRT,USER_FRT_CH0,&stcFrtConfig0);
    Mft_Frt_Init(&USER_MFT_FRT,USER_FRT_CH1,&stcFrtConfig1);
    //set frt cycle value
    Mft_Frt_SetCountCycle(&USER_MFT_FRT,USER_FRT_CH0,FRT_CYCLE);
    Mft_Frt_SetCountCycle(&USER_MFT_FRT,USER_FRT_CH1,FRT_CYCLE);
}

/**
 ******************************************************************************
 ** \brief  Init user output compare unit  .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void InitOcu(void)
{
    stc_mft_ocu_config_t stcOcuConfig0;
    stc_mft_ocu_config_t stcOcuConfig1;
        
    // Init OCU ch.0 and ch.1
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    stcOcuConfig0.enFrtConnect = Frt0ToOcu;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu0IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = Frt0ToOcu;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu1IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;
    
    Mft_Ocu_Init(&USER_MFT_OCU, MFT_OCU_CH0 ,&stcOcuConfig0);
    Mft_Ocu_Init(&USER_MFT_OCU, MFT_OCU_CH1 ,&stcOcuConfig1);
    
    // Init OCU ch.2 and ch.3
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    stcOcuConfig0.enFrtConnect = Frt1ToOcu;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufTrsfByFrtZero;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu2IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = Frt1ToOcu;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufTrsfByFrtZero;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu3IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;
    
    Mft_Ocu_Init(&USER_MFT_OCU, MFT_OCU_CH2, &stcOcuConfig0);
    Mft_Ocu_Init(&USER_MFT_OCU, MFT_OCU_CH3, &stcOcuConfig1);
    
    // Init OCU ch.4 and ch.5
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    stcOcuConfig0.enFrtConnect = Frt1ToOcu;
    stcOcuConfig0.enPinState = RtHighLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufTrsfByFrtZero;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu4IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = Frt1ToOcu;
    stcOcuConfig1.enPinState = RtHighLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufTrsfByFrtZero;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu5IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;
    
    Mft_Ocu_Init(&USER_MFT_OCU, MFT_OCU_CH4, &stcOcuConfig0);
    Mft_Ocu_Init(&USER_MFT_OCU, MFT_OCU_CH5, &stcOcuConfig1);

    // OCU1&OCU0: 1 change mode
    Mft_Ocu_SetCompareMode_Fm3(&USER_MFT_OCU, MFT_OCU_CH10, OcuOdd1ChangeEven1Change);
    
    // OCU3&OCU2: Active-High mode    
    Mft_Ocu_SetCompareMode_Fm3(&USER_MFT_OCU, MFT_OCU_CH32, OcuOddActiveHighEvenActiveHigh);
    
    // OCU5&OCU4: Active-Low mode    
    Mft_Ocu_SetCompareMode_Fm3(&USER_MFT_OCU, MFT_OCU_CH54, OcuOddActiveLowEvenActiveLow);
    
    //write OCU compare value
    Mft_Ocu_WriteOccp(&USER_MFT_OCU, MFT_OCU_CH0, USER_OCU_CH0_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU, MFT_OCU_CH1, USER_OCU_CH1_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU, MFT_OCU_CH2, USER_OCU_CH2_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU, MFT_OCU_CH3, USER_OCU_CH3_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU, MFT_OCU_CH4, USER_OCU_CH4_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU, MFT_OCU_CH5, USER_OCU_CH5_CMPVAL1);
}
/*---------------------------------------------------------------------------*/
/* global data                                                               */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* global functions                                                          */
/*---------------------------------------------------------------------------*/

/**
 ******************************************************************************
 ** \brief  Main function of project for S6E1A1 series.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("          OCU_CH_SINGLE Sample Start \n");
    printf("OCU0 and OCU1 connect to FRT0 and running with 1-change mode\n");
    printf("OCU2 and OCU3 connect to FRT1 and running with Active-High mode\n");
    printf("OCU4 and OCU5 connect to FRT1 and running with Active-Low mode\n");
    printf("==================================================\n");
#endif
    //init FRT
    InitFrt();
    //init OCU
    InitOcu();

    //enable OCU operation
    // Enable ch.0 and ch.1 to demo 1-change mode, comment the code of enable ch.2,3,4,5 to see the ch.0 and ch.1 behavior only
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,MFT_OCU_CH0);
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,MFT_OCU_CH1);
    // Enable ch.2 and ch.3 to demo active high mode, comment the code of enable ch.0,1,4,5 to see the ch.2 and ch.3 behavior only
//    Mft_Ocu_EnableOperation(&USER_MFT_OCU,MFT_OCU_CH2);
//    Mft_Ocu_EnableOperation(&USER_MFT_OCU,MFT_OCU_CH3);
    // Enable ch.2 and ch.3 to demo active low mode, comment the code of enable ch.0,1,2,3 to see the ch.4 and ch.5 behavior only
//    Mft_Ocu_EnableOperation(&USER_MFT_OCU,MFT_OCU_CH4);
//    Mft_Ocu_EnableOperation(&USER_MFT_OCU,MFT_OCU_CH5);
    
    //start FRT
    Mft_Frt_Start(&USER_MFT_FRT,USER_FRT_CH0);
    Mft_Frt_Start(&USER_MFT_FRT,USER_FRT_CH1);

    while(1)
    {
        if(m_u8OcuMatchFlag[0] == 1)
        { 
            m_u8OcuMatchFlag[0] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT0 counts at %d)\n", MFT_OCU_CH0, USER_OCU_CH0_CMPVAL1);
        #endif
        }

        if(m_u8OcuMatchFlag[1] == 1)
        {
            m_u8OcuMatchFlag[1] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT0 counts at %d)\n", MFT_OCU_CH1, USER_OCU_CH1_CMPVAL1);
        #endif
        }
        
        if(m_u8OcuMatchFlag[2] == 1)
        {
            m_u8OcuMatchFlag[2] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT1 counts at %d)\n", MFT_OCU_CH2, USER_OCU_CH2_CMPVAL1);
        #endif
        }
        
        if(m_u8OcuMatchFlag[3] == 1)
        {
            m_u8OcuMatchFlag[3] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT1 counts at %d)\n", MFT_OCU_CH3, USER_OCU_CH3_CMPVAL1);
        #endif
        }
        
        if(m_u8OcuMatchFlag[4] == 1)
        {
            m_u8OcuMatchFlag[4] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT1 counts at %d)\n", MFT_OCU_CH4, USER_OCU_CH4_CMPVAL1);
        #endif
        }
        
        if(m_u8OcuMatchFlag[5] == 1)
        {
            m_u8OcuMatchFlag[5] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT1 counts at %d)\n", MFT_OCU_CH5, USER_OCU_CH5_CMPVAL1);
        #endif
        }

    }
}
