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
 ** This example demonstrates RT dead timer mode of waveform generator
 **
 ** History:
 **   - 2014-11-23  0.0.1  EZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/
/* FRT module */
#define USER_FRT                 MFT0_FRT   
#define USER_MFT_FRT_CH          MFT_FRT_CH1
#define USER_FRT_MODE            FrtUpCount
#define USER_FRT_CYCLE          (50000)

/* OCU module */
#define USER_OCU                 MFT0_OCU
#define USER_OCU_CH0             MFT_OCU_CH0
#define USER_OCU_CH1             MFT_OCU_CH1
#define USER_OCU_COUPLE_CH       MFT_OCU_CH10
#define USER_OCU_FRT_CH          USER_MFT_FRT_CH

//OCU compare value
#define USER_OCU_CH0_CMPVAL1    (10000)
#define USER_OCU_CH0_CMPVAL2    (30000)
#define USER_OCU_CH1_CMPVAL1    (20000)
#define USER_OCU_CH1_CMPVAL2    (40000)

/* WFG module */
#define USER_WFG_TIMER_VALUE    (10000)
#define USER_WFG                MFT0_WFG
#define USER_WFG_CH             MFT_WFG_CH10
#define InitWfgOutput()         {SetPinFunc_RTO00_0(); SetPinFunc_RTO01_0();}
/*---------------------------------------------------------------------------*/
/* local data                                                                */
/*---------------------------------------------------------------------------*/
static uint8_t m_u8Ocu0IntFlag = 0;
static uint8_t m_u8Ocu1IntFlag = 0;
/*---------------------------------------------------------------------------*/
/* local functions prototypes                                                */
/*---------------------------------------------------------------------------*/
/**
 ******************************************************************************
 ** \brief  OCU channel 0 interrupt callback function   .
 **
 ** \return none
 ******************************************************************************/
static void Ocu0IntCallback(void)
{
    if(m_u8Ocu0IntFlag)
    {
        Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH0, USER_OCU_CH0_CMPVAL1);
    }
    else
    {
        Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH0, USER_OCU_CH0_CMPVAL2);
    }
    m_u8Ocu0IntFlag = !m_u8Ocu0IntFlag;
}

/**
 ******************************************************************************
 ** \brief  OCU channel 1 interrupt callback function  .
 **
 ** \return none
 ******************************************************************************/
static void Ocu1IntCallback(void)
{
    if(m_u8Ocu1IntFlag)
    {
        Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH1, USER_OCU_CH1_CMPVAL1);
    }
    else
    {
        Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH1, USER_OCU_CH1_CMPVAL2);
    }
    m_u8Ocu1IntFlag = !m_u8Ocu1IntFlag;
}

/**
 ******************************************************************************
 ** \brief  Init user free run timer   .
 **
 ** \return none
 ******************************************************************************/
static void InitFrt(void)
{
    stc_mft_frt_config_t stcFrtConfig;

    // Clear structure
    PDL_ZERO_STRUCT(stcFrtConfig);
    
    //configure the FRT parameter
    stcFrtConfig.enFrtMode = USER_FRT_MODE;
    stcFrtConfig.enFrtClockDiv = FrtPclkDiv64;
    stcFrtConfig.bEnExtClock = FALSE;
    stcFrtConfig.bEnBuffer = FALSE;

    Mft_Frt_Stop(&USER_FRT,USER_MFT_FRT_CH);
    //init frt module
    Mft_Frt_Init(&USER_FRT,USER_MFT_FRT_CH,&stcFrtConfig);
    //set frt cycle value
    Mft_Frt_SetCountCycle(&USER_FRT,USER_MFT_FRT_CH, USER_FRT_CYCLE);
}

/**
 ******************************************************************************
 ** \brief  Init user output compare unit  .
 **
 ** \return none
 ******************************************************************************/
static void InitOcu(void)
{
    stc_mft_ocu_config_t stcOcuConfig0, stcOcuConfig1;
    stc_ocu_even_compare_config_t stcCh0CompareConfig;
    stc_ocu_odd_compare_config_t  stcCh1CompareConfig;
    
    // Clear structures
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    PDL_ZERO_STRUCT(stcCh0CompareConfig);
    PDL_ZERO_STRUCT(stcCh1CompareConfig);
    
    // Configure the OCU parameter
    stcOcuConfig0.enFrtConnect = (en_mft_ocu_frt_t)USER_OCU_FRT_CH;
    stcOcuConfig0.bFm4 = TRUE;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufDisable;
    stcOcuConfig0.enOcseBufMode = OcseBufDisable;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu0IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = (en_mft_ocu_frt_t)USER_OCU_FRT_CH;
    stcOcuConfig1.bFm4 = TRUE;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufDisable;
    stcOcuConfig1.enOcseBufMode = OcseBufDisable;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu1IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;

    //init OCU
    Mft_Ocu_Init(&USER_OCU,USER_OCU_CH0,&stcOcuConfig0);
    Mft_Ocu_Init(&USER_OCU,USER_OCU_CH1,&stcOcuConfig1);

    //1 change mode
    //OCSE0:  0x00000FFF 
    //OCSE1:  0x0FF00FFF 
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
    
    Mft_Ocu_SetEvenChCompareMode(&USER_OCU, USER_OCU_CH0, &stcCh0CompareConfig);
    Mft_Ocu_SetOddChCompareMode(&USER_OCU, USER_OCU_CH1, &stcCh1CompareConfig);

    //write OCU compare value
    Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH0,USER_OCU_CH0_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH1,USER_OCU_CH1_CMPVAL1);

}


/**
 ******************************************************************************
 ** \brief  Init User Waveform Generator Unit    .
 **
 ** \return none
 ******************************************************************************/
static void InitWfg(void)
{
    stc_wfg_config_t stcWfgConfig;
    
    // Clear structure
    PDL_ZERO_STRUCT(stcWfgConfig);
    
    // Configure WFG structure
    stcWfgConfig.enMode = WfgRtDeadTimerMode;
    stcWfgConfig.enPselBits = PselBits00B;
    stcWfgConfig.enPgenBits = PgenBits00B;
    stcWfgConfig.enGtenBits = GtenBits00B;
    stcWfgConfig.enDmodBits = DmodBits00B;
    stcWfgConfig.enClk = WfgPlckDiv64;
    
    // Enable RTO0,RTO1 GPIO 
    InitWfgOutput();
    
    // Config WFG module
    Mft_Wfg_Init(&USER_WFG, USER_WFG_CH, &stcWfgConfig);
    Mft_Wfg_WriteTimerCountCycle(&USER_WFG, USER_WFG_CH, USER_WFG_TIMER_VALUE, USER_WFG_TIMER_VALUE);
    Mft_Wfg_StartTimer(&USER_WFG, USER_WFG_CH);
}

/**
 ******************************************************************************
 ** \brief  Main function of project for MB9B560 series.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
   
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("          WFG Sample Start \n");
    printf(" WFG running with RT dead timer mode,\n");
    printf(" please observe the RTO00_0 and RTO01_0 output by \n oscillograph\n");
    printf(".......\n");
#endif
    //init FRT
    InitFrt();
    //init OCU
    InitOcu();
    //init WFG
    InitWfg();
    //enable OCU operation
    Mft_Ocu_EnableOperation(&USER_OCU,USER_OCU_CH0);
    Mft_Ocu_EnableOperation(&USER_OCU,USER_OCU_CH1);
    //start FRT
    Mft_Frt_Start(&USER_FRT,USER_MFT_FRT_CH);
    
    while(1)
    {
        ;
    }
}
