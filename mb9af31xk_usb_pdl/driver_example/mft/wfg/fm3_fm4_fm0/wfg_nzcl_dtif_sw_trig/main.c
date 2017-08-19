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
 ** This example demonstrates motor emergency stop function of waveform
 ** generator. WFG will stop output after low level is detected on the
 ** DTTI0X pin.
 **
 ** History:
 **   - 2014-12-09  0.0.1  DHo        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/
#if !defined(GPIO1PIN_P50_INIT)  || !defined(SetPinFunc_RTO00_0) || \
    !defined(SetPinFunc_RTO00_0)
#error "Pin P50, RTO00_0, RTO01_0 and DTTI0X_0 are used in this example, \
change to other pins if these pins are not available."
#endif

/* FRT module */
#define USER_FRT                 MFT0_FRT
#define USER_MFT_FRT_CH          MFT_FRT_CH1
#define USER_FRT_MODE            FrtUpCount
#define USER_FRT_CYCLE           (50000)

/* OCU module */
#define USER_OCU                 MFT0_OCU
#define USER_OCU_CH0             MFT_OCU_CH0
#define USER_OCU_CH1             MFT_OCU_CH1
#define USER_OCU_COUPLE_CH       MFT_OCU_CH10
#define USER_OCU_FRT_CH          USER_MFT_FRT_CH
//OCU compare value
#define USER_OCU_CH0_CMPVAL1     (10000)
#define USER_OCU_CH0_CMPVAL2     (30000)
#define USER_OCU_CH1_CMPVAL1     (20000)
#define USER_OCU_CH1_CMPVAL2     (40000)

/* WFG module */
#define USER_WFG                  MFT0_WFG
#define USER_WFG_CH               MFT_WFG_CH10
#define InitWfgOutput()           {SetPinFunc_RTO00_0(); SetPinFunc_RTO01_0();}

/* Key */
#define KeyInit()                 Gpio1pin_InitIn(GPIO1PIN_P50, Gpio1pin_InitPullup( 0u ) )    // Init Input P50
#define GetKeyInput()             Gpio1pin_Get(GPIO1PIN_P50)

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
 ** \brief  Time delay
 **
 ** \return none
 ******************************************************************************/
static void Delay(uint32_t u32Cnt)
{
    uint32_t u32i;
    for(;u32Cnt;u32Cnt--)
    {
        for(u32i=(SystemCoreClock/5); u32i; u32i--)
        {
            ;
        }
    }
}

/**
 ******************************************************************************
 ** \brief  OCU channel 0 interrupt callback function
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

    return;
}

/**
 ******************************************************************************
 ** \brief  OCU channel 1 interrupt callback function
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

    return;
}

/**
 ******************************************************************************
 ** \brief  Init user free run timer
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

    //init frt module
    Mft_Frt_Init(&USER_FRT,USER_MFT_FRT_CH,&stcFrtConfig);
    //set frt cycle value
    Mft_Frt_SetCountCycle(&USER_FRT,USER_MFT_FRT_CH, USER_FRT_CYCLE);

    return;
}

/**
 ******************************************************************************
 ** \brief  De-Init user free run timer
 ******************************************************************************/
static void DeInitFrt(void)
{
    Mft_Frt_DeInit(&USER_FRT,USER_MFT_FRT_CH, TRUE);

    return;
}

/**
 ******************************************************************************
 ** \brief  Init user output compare unit
 **
 ** \return none
 ******************************************************************************/
static void InitOcu(void)
{
    stc_mft_ocu_config_t stcOcuConfig0, stcOcuConfig1;

    // Clear structures
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);

    // Configure the OCU parameter
    stcOcuConfig0.enFrtConnect = (en_mft_ocu_frt_t)USER_OCU_FRT_CH;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufDisable;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu0IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;

    stcOcuConfig1.enFrtConnect = (en_mft_ocu_frt_t)USER_OCU_FRT_CH;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufDisable;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu1IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;

    //init OCU
    Mft_Ocu_Init(&USER_OCU,USER_OCU_CH0,&stcOcuConfig0);
    Mft_Ocu_Init(&USER_OCU,USER_OCU_CH1,&stcOcuConfig1);

    //1 change mode
    Mft_Ocu_SetCompareMode_Fm3(&USER_OCU, USER_OCU_COUPLE_CH, OcuOdd1ChangeEven1Change);

    //write OCU compare value
    Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH0,USER_OCU_CH0_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH1,USER_OCU_CH1_CMPVAL1);

    return;
}

/**
 ******************************************************************************
 ** \brief  De-Init user output compare unit
 ******************************************************************************/
static void DeInitOcu(void)
{
    Mft_Ocu_DeInit(&USER_OCU,USER_OCU_CH0, TRUE);
    Mft_Ocu_DeInit(&USER_OCU,USER_OCU_CH1, TRUE);

    return;
}

/**
 ******************************************************************************
 ** \brief  Init DTIF callback function
 **
 ** \return none
 ******************************************************************************/
static void DtifIntCallback(void)
{
    if(GetKeyInput() == PdlHigh)
    {
        Mft_Wfg_Nzcl_ClrDigitalFilterIrqFlag(&USER_WFG);

        //Clear Flag
        m_u8Ocu0IntFlag = 0;
        m_u8Ocu1IntFlag = 0;
        Delay(1);

        //de-initialize FRT, OCU
        DeInitFrt();
        DeInitOcu();

        //re-initialize FRT, OCU
        InitFrt();
        InitOcu();

        //re-start FRT, OCU
        Mft_Ocu_EnableOperation(&USER_OCU,USER_OCU_CH0);
        Mft_Ocu_EnableOperation(&USER_OCU,USER_OCU_CH1);

        Mft_Frt_Start(&USER_FRT,USER_MFT_FRT_CH);
    }

    return;
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
    stc_wfg_nzcl_config_t  stcNzclConfig;
    // Clear structure
    PDL_ZERO_STRUCT(stcWfgConfig);
    PDL_ZERO_STRUCT(stcNzclConfig);

    // Configure WFG structure
    stcWfgConfig.enMode = WfgThroughMode;
    stcWfgConfig.enPselBits = PselBits00B;
    stcWfgConfig.enPgenBits = PgenBits00B;
    stcWfgConfig.enGtenBits = GtenBits00B;
    stcWfgConfig.enDmodBits = DmodBits00B;

    // Enable RTO0,RTO1 GPIO
    InitWfgOutput();
    // Config WFG module
    Mft_Wfg_Init(&USER_WFG, USER_WFG_CH, &stcWfgConfig);

    /* config NZCL */
    stcNzclConfig.bEnDigitalFilter = FALSE;
    stcNzclConfig.enDigitalFilterWidth = NzlcWidth8Cycle;
    stcNzclConfig.bSwitchToGpio = TRUE;
    stcNzclConfig.pfnDtifDigtalFilterIrqCallback = DtifIntCallback;

    Mft_Wfg_Nzcl_Init(&USER_WFG, &stcNzclConfig, TRUE);

    return;
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
    printf(" WFG running with through mode\n");
    printf(" Connect P50 with key, press key to trigger DTIF interrupt!\n");
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
    KeyInit();

    while(1)
    {
        if(GetKeyInput() == PdlLow)
        {
             Delay(1);
             Mft_Wfg_Nzcl_SwTiggerDtif(&USER_WFG);

        }
    }
}
