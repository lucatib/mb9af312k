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
// OCU channel
#define USER_OCU_CH0    MFT_OCU_CH0
#define USER_OCU_CH1    MFT_OCU_CH1
#define USER_OCU_CH2    MFT_OCU_CH2
#define USER_OCU_CH3    MFT_OCU_CH3
#define USER_OCU_CH4    MFT_OCU_CH4
#define USER_OCU_CH5    MFT_OCU_CH5
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
    stcFrtConfig0.enFrtClockDiv = FrtPclkDiv1024;
    stcFrtConfig0.bEnExtClock = FALSE;
    stcFrtConfig0.bEnBuffer = TRUE;
    
    //configure the FRT channel 1 parameter
    stcFrtConfig1.enFrtMode = USER_FRT_CH1_MOD;
    stcFrtConfig1.enFrtClockDiv = FrtPclkDiv512;
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
    
    stc_ocu_even_compare_config_t stcCh0CompareConfig, stcCh2CompareConfig, stcCh4CompareConfig;
    stc_ocu_odd_compare_config_t  stcCh1CompareConfig, stcCh3CompareConfig, stcCh5CompareConfig;
    
    // Init OCU ch.0 and ch.1
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    stcOcuConfig0.enFrtConnect = Frt0ToOcu;
    stcOcuConfig0.bFm4 = TRUE;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufDisable;
    stcOcuConfig0.enOcseBufMode = OcseBufDisable;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu0IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = Frt0ToOcu;
    stcOcuConfig1.bFm4 = TRUE;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufDisable;
    stcOcuConfig1.enOcseBufMode = OcseBufDisable;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu1IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;
    
    Mft_Ocu_Init(&USER_MFT_OCU,USER_OCU_CH0,&stcOcuConfig0);
    Mft_Ocu_Init(&USER_MFT_OCU,USER_OCU_CH1,&stcOcuConfig1);
    
    // Init OCU ch.2 and ch.3
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    stcOcuConfig0.enFrtConnect = Frt1ToOcu;
    stcOcuConfig0.bFm4 = TRUE;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufTrsfByFrtZero;
    stcOcuConfig0.enOcseBufMode = OcseBufDisable;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu2IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = Frt1ToOcu;
    stcOcuConfig1.bFm4 = TRUE;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufTrsfByFrtZero;
    stcOcuConfig1.enOcseBufMode = OcseBufDisable;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu3IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;
    
    Mft_Ocu_Init(&USER_MFT_OCU,USER_OCU_CH2,&stcOcuConfig0);
    Mft_Ocu_Init(&USER_MFT_OCU,USER_OCU_CH3,&stcOcuConfig1);
    
    // Init OCU ch.4 and ch.5
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    stcOcuConfig0.enFrtConnect = Frt1ToOcu;
    stcOcuConfig0.bFm4 = TRUE;
    stcOcuConfig0.enPinState = RtHighLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufTrsfByFrtZero;
    stcOcuConfig0.enOcseBufMode = OcseBufDisable;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu4IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = Frt1ToOcu;
    stcOcuConfig1.bFm4 = TRUE;
    stcOcuConfig1.enPinState = RtHighLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufTrsfByFrtZero;
    stcOcuConfig1.enOcseBufMode = OcseBufDisable;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu5IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;
    
    Mft_Ocu_Init(&USER_MFT_OCU,USER_OCU_CH4,&stcOcuConfig0);
    Mft_Ocu_Init(&USER_MFT_OCU,USER_OCU_CH5,&stcOcuConfig1);

    // OCU1&OCU0: 1 change mode
    // OCSE0:        0x00000FFF
    // OCSE1:        0x0FF00FFF
    stcCh0CompareConfig.enFrtZeroEvenMatchEvenChRtStatus = RtOutputReverse;
    stcCh0CompareConfig.enFrtZeroEvenNotMatchEvenChRtStatus = RtOutputHold;
    stcCh0CompareConfig.enFrtUpCntEvenMatchEvenChRtStatus = RtOutputReverse;
    stcCh0CompareConfig.enFrtPeakEvenMatchEvenChRtStatus = RtOutputReverse;
    stcCh0CompareConfig.enFrtPeakEvenNotMatchEvenChStatus = RtOutputHold;
    stcCh0CompareConfig.enFrtDownCntEvenMatchEvenChRtStatus = RtOutputReverse;
    stcCh0CompareConfig.enIopFlagWhenFrtZeroEvenMatch = IopFlagSet;
    stcCh0CompareConfig.enIopFlagWhenFrtUpCntEvenMatch = IopFlagSet;
    stcCh0CompareConfig.enIopFlagWhenFrtPeakEvenMatch = IopFlagSet;
    stcCh0CompareConfig.enIopFlagWhenFrtDownCntEvenMatch = IopFlagSet;
    
    stcCh1CompareConfig.enFrtZeroOddMatchEvenMatchOddChRtStatus = RtOutputReverse;
    stcCh1CompareConfig.enFrtZeroOddMatchEvenNotMatchOddChRtStatus = RtOutputReverse;
    stcCh1CompareConfig.enFrtZeroOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh1CompareConfig.enFrtZeroOddNotMatchEvenNotMatchOddChRtStatus = RtOutputHold;
    stcCh1CompareConfig.enFrtUpCntOddMatchEvenMatchOddChRtStatus = RtOutputReverse;
    stcCh1CompareConfig.enFrtUpCntOddMatchEvenNotMatchOddChRtStatus = RtOutputReverse;
    stcCh1CompareConfig.enFrtUpCntOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh1CompareConfig.enFrtPeakOddMatchEvenMatchOddChRtStatus = RtOutputReverse;
    stcCh1CompareConfig.enFrtPeakOddMatchEvenNotMatchOddChRtStatus = RtOutputReverse;
    stcCh1CompareConfig.enFrtPeakOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh1CompareConfig.enFrtPeakOddNotMatchEvenNotMatchOddChRtStatus = RtOutputHold;
    stcCh1CompareConfig.enFrtDownOddMatchEvenMatchOddChRtStatus = RtOutputReverse;
    stcCh1CompareConfig.enFrtDownOddMatchEvenNotMatchOddChRtStatus = RtOutputReverse;
    stcCh1CompareConfig.enFrtDownOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh1CompareConfig.enIopFlagWhenFrtZeroOddMatch = IopFlagSet;
    stcCh1CompareConfig.enIopFlagWhenFrtUpCntOddMatch = IopFlagSet;
    stcCh1CompareConfig.enIopFlagWhenFrtPeakOddMatch = IopFlagSet;
    stcCh1CompareConfig.enIopFlagWhenFrtDownCntOddMatch = IopFlagSet;
    
    Mft_Ocu_SetEvenChCompareMode(&USER_MFT_OCU, USER_OCU_CH0, &stcCh0CompareConfig);
    Mft_Ocu_SetOddChCompareMode(&USER_MFT_OCU, USER_OCU_CH1, &stcCh1CompareConfig);
    
    // OCU3&OCU2: Active-High mode
    // OCSE2:        0x0000852D
    // OCSE3:        0x8520852D
    stcCh2CompareConfig.enFrtZeroEvenMatchEvenChRtStatus = RtOutputHigh;
    stcCh2CompareConfig.enFrtZeroEvenNotMatchEvenChRtStatus = RtOutputLow;
    stcCh2CompareConfig.enFrtUpCntEvenMatchEvenChRtStatus = RtOutputHigh;
    stcCh2CompareConfig.enFrtPeakEvenMatchEvenChRtStatus = RtOutputHold;
    stcCh2CompareConfig.enFrtPeakEvenNotMatchEvenChStatus = RtOutputHold;
    stcCh2CompareConfig.enFrtDownCntEvenMatchEvenChRtStatus = RtOutputLow;
    stcCh2CompareConfig.enIopFlagWhenFrtZeroEvenMatch = IopFlagSet;
    stcCh2CompareConfig.enIopFlagWhenFrtUpCntEvenMatch = IopFlagSet;
    stcCh2CompareConfig.enIopFlagWhenFrtPeakEvenMatch = IopFlagHold;
    stcCh2CompareConfig.enIopFlagWhenFrtDownCntEvenMatch = IopFlagSet;
    
    stcCh3CompareConfig.enFrtZeroOddMatchEvenMatchOddChRtStatus = RtOutputHigh;
    stcCh3CompareConfig.enFrtZeroOddMatchEvenNotMatchOddChRtStatus = RtOutputHigh;
    stcCh3CompareConfig.enFrtZeroOddNotMatchEvenMatchOddChRtStatus = RtOutputLow;
    stcCh3CompareConfig.enFrtZeroOddNotMatchEvenNotMatchOddChRtStatus = RtOutputLow;
    stcCh3CompareConfig.enFrtUpCntOddMatchEvenMatchOddChRtStatus = RtOutputHigh;
    stcCh3CompareConfig.enFrtUpCntOddMatchEvenNotMatchOddChRtStatus = RtOutputHigh;
    stcCh3CompareConfig.enFrtUpCntOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh3CompareConfig.enFrtPeakOddMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh3CompareConfig.enFrtPeakOddMatchEvenNotMatchOddChRtStatus = RtOutputHold;
    stcCh3CompareConfig.enFrtPeakOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh3CompareConfig.enFrtPeakOddNotMatchEvenNotMatchOddChRtStatus = RtOutputHold;
    stcCh3CompareConfig.enFrtDownOddMatchEvenMatchOddChRtStatus = RtOutputLow;
    stcCh3CompareConfig.enFrtDownOddMatchEvenNotMatchOddChRtStatus = RtOutputLow;
    stcCh3CompareConfig.enFrtDownOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh3CompareConfig.enIopFlagWhenFrtZeroOddMatch = IopFlagSet;
    stcCh3CompareConfig.enIopFlagWhenFrtUpCntOddMatch = IopFlagSet;
    stcCh3CompareConfig.enIopFlagWhenFrtPeakOddMatch = IopFlagHold;
    stcCh3CompareConfig.enIopFlagWhenFrtDownCntOddMatch = IopFlagSet;
    
    Mft_Ocu_SetEvenChCompareMode(&USER_MFT_OCU, USER_OCU_CH2, &stcCh2CompareConfig);
    Mft_Ocu_SetOddChCompareMode(&USER_MFT_OCU, USER_OCU_CH3, &stcCh3CompareConfig);
    
    // OCU5&OCU4: Active-Low mode
    // OCSE4        0x00004A1D
    // OCSE5        0x4A104A1D
    stcCh4CompareConfig.enFrtZeroEvenMatchEvenChRtStatus = RtOutputLow;
    stcCh4CompareConfig.enFrtZeroEvenNotMatchEvenChRtStatus = RtOutputHigh;
    stcCh4CompareConfig.enFrtUpCntEvenMatchEvenChRtStatus = RtOutputLow;
    stcCh4CompareConfig.enFrtPeakEvenMatchEvenChRtStatus = RtOutputHold;
    stcCh4CompareConfig.enFrtPeakEvenNotMatchEvenChStatus = RtOutputHold;
    stcCh4CompareConfig.enFrtDownCntEvenMatchEvenChRtStatus = RtOutputHigh;
    stcCh4CompareConfig.enIopFlagWhenFrtZeroEvenMatch = IopFlagSet;
    stcCh4CompareConfig.enIopFlagWhenFrtUpCntEvenMatch = IopFlagSet;
    stcCh4CompareConfig.enIopFlagWhenFrtPeakEvenMatch = IopFlagHold;
    stcCh4CompareConfig.enIopFlagWhenFrtDownCntEvenMatch = IopFlagSet;
    
    stcCh5CompareConfig.enFrtZeroOddMatchEvenMatchOddChRtStatus = RtOutputLow;
    stcCh5CompareConfig.enFrtZeroOddMatchEvenNotMatchOddChRtStatus = RtOutputLow;
    stcCh5CompareConfig.enFrtZeroOddNotMatchEvenMatchOddChRtStatus = RtOutputHigh;
    stcCh5CompareConfig.enFrtZeroOddNotMatchEvenNotMatchOddChRtStatus = RtOutputHigh;
    stcCh5CompareConfig.enFrtUpCntOddMatchEvenMatchOddChRtStatus = RtOutputLow;
    stcCh5CompareConfig.enFrtUpCntOddMatchEvenNotMatchOddChRtStatus = RtOutputLow;
    stcCh5CompareConfig.enFrtUpCntOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh5CompareConfig.enFrtPeakOddMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh5CompareConfig.enFrtPeakOddMatchEvenNotMatchOddChRtStatus = RtOutputHold;
    stcCh5CompareConfig.enFrtPeakOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh5CompareConfig.enFrtPeakOddNotMatchEvenNotMatchOddChRtStatus = RtOutputHold;
    stcCh5CompareConfig.enFrtDownOddMatchEvenMatchOddChRtStatus = RtOutputHigh;
    stcCh5CompareConfig.enFrtDownOddMatchEvenNotMatchOddChRtStatus = RtOutputHigh;
    stcCh5CompareConfig.enFrtDownOddNotMatchEvenMatchOddChRtStatus = RtOutputHold;
    stcCh5CompareConfig.enIopFlagWhenFrtZeroOddMatch = IopFlagSet;
    stcCh5CompareConfig.enIopFlagWhenFrtUpCntOddMatch = IopFlagSet;
    stcCh5CompareConfig.enIopFlagWhenFrtPeakOddMatch = IopFlagHold;
    stcCh5CompareConfig.enIopFlagWhenFrtDownCntOddMatch = IopFlagSet;
    
    Mft_Ocu_SetEvenChCompareMode(&USER_MFT_OCU, USER_OCU_CH4, &stcCh4CompareConfig);
    Mft_Ocu_SetOddChCompareMode(&USER_MFT_OCU, USER_OCU_CH5, &stcCh5CompareConfig);
    
    //write OCU compare value
    Mft_Ocu_WriteOccp(&USER_MFT_OCU,USER_OCU_CH0,USER_OCU_CH0_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU,USER_OCU_CH1,USER_OCU_CH1_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU,USER_OCU_CH2,USER_OCU_CH2_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU,USER_OCU_CH3,USER_OCU_CH3_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU,USER_OCU_CH4,USER_OCU_CH4_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU,USER_OCU_CH5,USER_OCU_CH5_CMPVAL1);
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
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,USER_OCU_CH0);
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,USER_OCU_CH1);
    // Enable ch.2 and ch.3 to demo active high mode, comment the code of enable ch.0,1,4,5 to see the ch.2 and ch.3 behavior only
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,USER_OCU_CH2);
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,USER_OCU_CH3);
    // Enable ch.2 and ch.3 to demo active low mode, comment the code of enable ch.0,1,2,3 to see the ch.4 and ch.5 behavior only
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,USER_OCU_CH4);
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,USER_OCU_CH5);
    
    //start FRT
    Mft_Frt_Start(&USER_MFT_FRT,USER_FRT_CH0);
    Mft_Frt_Start(&USER_MFT_FRT,USER_FRT_CH1);

    while(1)
    {
        if(m_u8OcuMatchFlag[0] == 1)
        { 
            m_u8OcuMatchFlag[0] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT0 counts at %d)\n", USER_OCU_CH0, USER_OCU_CH0_CMPVAL1);
        #endif
        }

        if(m_u8OcuMatchFlag[1] == 1)
        {
            m_u8OcuMatchFlag[1] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT0 counts at %d)\n", USER_OCU_CH1, USER_OCU_CH1_CMPVAL1);
        #endif
        }
        
        if(m_u8OcuMatchFlag[2] == 1)
        {
            m_u8OcuMatchFlag[2] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT1 counts at %d)\n", USER_OCU_CH2, USER_OCU_CH2_CMPVAL1);
        #endif
        }
        
        if(m_u8OcuMatchFlag[3] == 1)
        {
            m_u8OcuMatchFlag[3] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT1 counts at %d)\n", USER_OCU_CH3, USER_OCU_CH3_CMPVAL1);
        #endif
        }
        
        if(m_u8OcuMatchFlag[4] == 1)
        {
            m_u8OcuMatchFlag[4] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT1 counts at %d)\n", USER_OCU_CH4, USER_OCU_CH4_CMPVAL1);
        #endif
        }
        
        if(m_u8OcuMatchFlag[5] == 1)
        {
            m_u8OcuMatchFlag[5] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT1 counts at %d)\n", USER_OCU_CH5, USER_OCU_CH5_CMPVAL1);
        #endif
        }

    }
}
