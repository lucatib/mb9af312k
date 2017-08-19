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
 ** This example demonstrates PPG-dead timer mode of waveform generator
 ** PPG0 connect to WFG10
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

/* WFG module */
#define USER_WFG              MFT0_WFG
#define USER_WFG_CH           MFT_WFG_CH10
#define WFG_GTEN_BITS         GtenBits11B 
#define WFG_PSEL_BITS         PselBits00B 
#define WFG_PGEN_BITS         PgenBits11B
#define WFG_DMOD_BITS         DmodBits00B 
#define WFG_WFTA              50
#define WFG_WFTB              WFG_WFTA
#define InitWfgOutput()       {SetPinFunc_RTO00_0(); SetPinFunc_RTO01_0();}

/* PPG module */
#define USER_PPG_CH           PPG_CH0
#define PPG_LOW_WIDTH         200
#define PPG_HIGH_WIDTH        200

/*---------------------------------------------------------------------------*/
/* local data                                                                */
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/* local functions prototypes                                                */
/*---------------------------------------------------------------------------*/

/**
 ******************************************************************************
 ** \brief  Init User PPG module    .
 **
 ** \return none
 ******************************************************************************/
static void InitPpg(void)
{
    stc_ppg_config_t stcPpgConfig;

    //clear structure
    PDL_ZERO_STRUCT(stcPpgConfig);
    
    //set the ppg config value
    stcPpgConfig.enEvenClock = PpgPclkDiv64;
    stcPpgConfig.enOddClock = PpgPclkDiv64;
    stcPpgConfig.enEvenLevel = PpgNormalLevel;
    stcPpgConfig.enOddLevel = PpgNormalLevel;
    stcPpgConfig.enTrig = PpgSoftwareTrig;
    stcPpgConfig.enMode = Ppg8Bit8Bit;

    //init ppg
    Ppg_Init(USER_PPG_CH,&stcPpgConfig);
    Ppg_SetLevelWidth(USER_PPG_CH, PPG_LOW_WIDTH, PPG_HIGH_WIDTH);
    Ppg_StartSoftwareTrig(USER_PPG_CH);

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
    
    // Clear structure
    PDL_ZERO_STRUCT(stcWfgConfig);
    
    // Configure WFG structure
    stcWfgConfig.enMode = WfgPpgDeadTimerMode;
    stcWfgConfig.enPselBits = WFG_PSEL_BITS;
    stcWfgConfig.enPgenBits = WFG_PGEN_BITS;
    stcWfgConfig.enGtenBits = WFG_GTEN_BITS;
    stcWfgConfig.enDmodBits = WFG_DMOD_BITS;
    stcWfgConfig.enClk = WfgPlckDiv64;

    /*enable WFG GPIO*/
    InitWfgOutput();

    // Config WFG module
    Mft_Wfg_Init(&USER_WFG, USER_WFG_CH, &stcWfgConfig);
    Mft_Wfg_WriteTimerCountCycle(&USER_WFG, USER_WFG_CH, WFG_WFTA, WFG_WFTB);
    Mft_Wfg_StartTimer(&USER_WFG, USER_WFG_CH);

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
    printf(" WFG running with PPG-dead timer mode;\n");
    printf(" please observe the RTO00_0 and RTO01_0 output by \n oscillograph\n");
    printf(".......\n");
#endif
    //init PPG, use ppg0
    InitPpg();
    //init WFG
    InitWfg();

    while(1)
    {
        ;
    }
}
