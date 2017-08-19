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
 ** This example demonstrates 2 channel linked mode of OCU.
 **
 ** History:
 **   - 2014-11-08  0.0.1  EZh       First version for FM universal PDL.
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
// FRT cycle
#define FRT_CYCLE       60000  

/*  OCU module  */
// MFT channel
#define USER_MFT_OCU        MFT0_OCU
// OCU channel
#define USER_OCU_CH0          MFT_OCU_CH0
#define USER_OCU_CH1          MFT_OCU_CH1
#define USER_OCU_COUPLE_CH    MFT_OCU_CH10
// OCU compare value
#define USER_OCU_CH0_CMPVAL1 (20000)
#define USER_OCU_CH1_CMPVAL1 (40000)


/*---------------------------------------------------------------------------*/
/* local data                                                                */
/*---------------------------------------------------------------------------*/
static uint8_t m_u8OcuMatchFlag[2] = {0};
static uint8_t m_u8Ch1Level = 0;

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
    if(m_u8Ch1Level != Mft_Ocu_GetRtPinLevel(&USER_MFT_OCU, USER_OCU_CH1))
    {
        m_u8OcuMatchFlag[0] = 1;
    }
    m_u8Ch1Level = Mft_Ocu_GetRtPinLevel(&USER_MFT_OCU, USER_OCU_CH1);
}

/**
 ******************************************************************************
 ** \brief  OCU channel 1 interrupt callback function  .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void Ocu1IntCallback(void)
{
    if(m_u8Ch1Level != Mft_Ocu_GetRtPinLevel(&USER_MFT_OCU, USER_OCU_CH1))
    {
        m_u8OcuMatchFlag[1] = 1;
    }
    m_u8Ch1Level = Mft_Ocu_GetRtPinLevel(&USER_MFT_OCU, USER_OCU_CH1);
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
    
    // Clear structures
    PDL_ZERO_STRUCT(stcFrtConfig0);

    //configure the FRT channel 0 parameter
    stcFrtConfig0.enFrtMode = USER_FRT_CH0_MOD;
    stcFrtConfig0.enFrtClockDiv = FrtPclkDiv256;
    stcFrtConfig0.bEnExtClock = FALSE;
    stcFrtConfig0.bEnBuffer = TRUE;

    //stop frt
    Mft_Frt_Stop(&USER_MFT_FRT,USER_FRT_CH0);
    //init frt module
    Mft_Frt_Init(&USER_MFT_FRT,USER_FRT_CH0,&stcFrtConfig0);
    //set frt cycle value
    Mft_Frt_SetCountCycle(&USER_MFT_FRT,USER_FRT_CH0,FRT_CYCLE);
}

/**
 ******************************************************************************
 ** \brief  Init user output compare unit  .
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
static void InitOcu(void)
{
    stc_mft_ocu_config_t stcOcuConfig0, stcOcuConfig1;
    
    // Clear structures
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    /*configure the OCU parameter*/
    stcOcuConfig0.enFrtConnect = Frt0ToOcu;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufDisable;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu0IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = Frt0ToOcu;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufDisable;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu1IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;

    //init OCU
    Mft_Ocu_Init(&USER_MFT_OCU,USER_OCU_CH0,&stcOcuConfig0);
    Mft_Ocu_Init(&USER_MFT_OCU,USER_OCU_CH1,&stcOcuConfig1);
    
    // Set comapre mode
    Mft_Ocu_SetCompareMode_Fm3(&USER_MFT_OCU, USER_OCU_COUPLE_CH, OcuOdd2ChangeEven1Change);
    
    //write OCU compare value
    Mft_Ocu_WriteOccp(&USER_MFT_OCU,USER_OCU_CH0,USER_OCU_CH0_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_MFT_OCU,USER_OCU_CH1,USER_OCU_CH1_CMPVAL1);
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
    printf("===============================================================\n");
    printf("          ocu_ch_linked Sample Start \n");
    printf("OCU0 and OCU1 connect to FRT0 and running \n");
    printf("with 2 channel linked mode\n");
    printf("===============================================================\n");
#endif
    //init FRT
    InitFrt();
    //init OCU
    InitOcu();

    //start FRT
    Mft_Frt_Start(&USER_MFT_FRT,USER_FRT_CH0);
    //enable OCU operation
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,USER_OCU_CH0);
    Mft_Ocu_EnableOperation(&USER_MFT_OCU,USER_OCU_CH1);

    while(1)
    {
        if(m_u8OcuMatchFlag[0] == 1)
        { 
            m_u8OcuMatchFlag[0] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT0 counts at %d)\n", USER_OCU_CH1, USER_OCU_CH0_CMPVAL1);
        #endif
        }

        if(m_u8OcuMatchFlag[1] == 1)
        {
            m_u8OcuMatchFlag[1] = 0;
        #ifdef DEBUG_PRINT
            printf("OCU%d match occurs! (FRT0 counts at %d)\n", USER_OCU_CH1, USER_OCU_CH1_CMPVAL1);
        #endif
        }

    }
}
