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
 ** This example demonstrates NEC mode of Remote Control operation
 **
 ** History:
 **   - 2015-01-20 1.0  KXi First version for FM Universal PDL.
 **S
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/ 
#if (PDL_PERIPHERAL_ENABLE_RC0 == PDL_ON) 
#define RC_CHANNEL RC0
#if (PDL_MCU_TYPE == PDL_FM3_TYPE8 || PDL_MCU_TYPE == PDL_FM3_TYPE12 || \
     PDL_MCU_TYPE == PDL_FM0P_TYPE2 || PDL_MCU_TYPE == PDL_FM0P_TYPE3)
#define SetPinFunc_CEC() SetPinFunc_CEC0_0()
#else 
#define SetPinFunc_CEC() SetPinFunc_CEC0()
#endif
#elif (PDL_PERIPHERAL_ENABLE_RC1 == PDL_ON) 
#define RC_CHANNEL RC1
#if (PDL_MCU_TYPE == PDL_FM3_TYPE8 || PDL_MCU_TYPE == PDL_FM3_TYPE12 || \
     PDL_MCU_TYPE == PDL_FM0P_TYPE2 || PDL_MCU_TYPE == PDL_FM0P_TYPE3)
#define SetPinFunc_CEC() SetPinFunc_CEC1_0()
#else 
#define SetPinFunc_CEC() SetPinFunc_CEC1()
#endif
#endif

#if (PDL_MCU_CORE == PDL_FM0P_CORE)
#define APBC_PSR_Val APBC1_PSR_Val
#elif (PDL_MCU_CORE == PDL_FM3_CORE)
#define APBC_PSR_Val APBC2_PSR_Val
#endif

#if ((APBC_PSR_Val&0x3) == 0)
#define APB_DIV 1
#elif ((APBC_PSR_Val&0x3) == 1)
#define APB_DIV 2
#elif ((APBC_PSR_Val&0x3) == 2)
#define APB_DIV 4
#elif ((APBC_PSR_Val&0x3) == 3)
#define APB_DIV 8
#endif

#define APB_CLOCK      (SystemCoreClock/APB_DIV)

#define RC_DIV      APB_CLOCK/32000           ///< set RC frequency about 32KHz

#define TEST_ADDR 0x1c
#define TEST_SIZE 3

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/


/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/ 
typedef union un_rc_rx_data_t
{
    struct
    {       
        uint8_t u8LlDat;
        uint8_t u8LhDat;
        uint8_t u8HlDat;
        uint8_t u8HhDat;
    }stcDat;
    uint32_t u32Dat;
}un_rc_rx_data_t;


/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static un_rc_rx_data_t unDat;
/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
/**
 ******************************************************************************
 ** \brief  Init RcNec 
 **
 ** \return none
 ******************************************************************************/
static void InitNec(void)
{
    stc_rc_rx_nec_config_t stcRcNecConfig;
    // Clear structures
    PDL_ZERO_STRUCT(stcRcNecConfig);
    
    
    // Configure the Sircs parameter
    stcRcNecConfig.u16DivVal = RC_DIV;
    stcRcNecConfig.enSrcClk  = RcPeripheralClk;             ///< set peripheral clk
    stcRcNecConfig.enThresholdType = RcThresholdType0;      ///< when W>MinPulseWidth and W < u8ThresholdWidth, the value is '0'\
                                                                when W>MinPulseWidth and W >= u8ThresholdWidth, the value is '1'
    stcRcNecConfig.u8MinPulseWidth = 13u;
    stcRcNecConfig.u8ThresholdWidth = 50u;
    stcRcNecConfig.u8StartBitWidth = 140u;
    stcRcNecConfig.u8RepeatWidth = 80u;
    stcRcNecConfig.enOverflowCycle = RcOverflow256Cycle;         
    stcRcNecConfig.bAddrCmpEn = FALSE;                      ///< disable address comparison
    
    Rc_Rx_Nec_Init((stc_rcn_t*)&RC_CHANNEL ,&stcRcNecConfig);
    SetPinFunc_CEC();                                       ///< set rc pin
}
/**
 ******************************************************************************
 ** \brief  Nec receiver polling process 
 **
 ** \return none
 ******************************************************************************/
static void NecRxPollingProc(void)
{
    static stc_rcn_t* pstcRc =  ((stc_rcn_t *) FM_HDMICEC0_BASE);
    do
    {
        if(TRUE == Rc_Rx_Nec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxNecRepeatCodeIrq)) ///< if repeat code was detected 
        {
            Rc_Rx_Nec_ClrIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxNecRepeatCodeIrq);    ///< clear repeat code irq flag
        }
    }
    while(FALSE == Rc_Rx_Nec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxNecStartIrq));  ///< wait start bit occur
    Rc_Rx_Nec_ClrIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxNecStartIrq);
    while(FALSE == Rc_Rx_Nec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxNecCntOvfIrq)); ///< wait count over
    Rc_Rx_Nec_ClrIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxNecCntOvfIrq);
    /** get NEC receive data */
    unDat.stcDat.u8HhDat = pstcRc->RCDTHH;
    unDat.stcDat.u8HlDat = pstcRc->RCDTHL;
    unDat.stcDat.u8LhDat = pstcRc->RCDTLH;
    unDat.stcDat.u8LlDat = pstcRc->RCDTLL;   
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("The receive data is  : 0x%x\n", unDat.u32Dat);   
    printf("==================================================\n");
#endif    
    
}

boolean_t Rc_Rx_Nec_GetIrqFlag(volatile stc_rcn_t *pstcRc, 
                               en_rc_rx_nec_irq_sel_t enIrqSel);
en_result_t Rc_Rx_Nec_ClrIrqFlag(volatile stc_rcn_t *pstcRc, 
                                 en_rc_rx_nec_irq_sel_t enIrqSel);
/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/


/**
 ******************************************************************************
 ** \brief  Main function of project for MCU evaluation board
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{  
#ifdef DEBUG_PRINT
   //Uart_Io_Init();
#endif  

    InitNec();                                              ///< initial NEC

#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("NEC rx polling Example Program Start \n");
    printf("==================================================\n");
#endif
    Rc_Rx_Nec_EnableRx((stc_rcn_t*)&RC_CHANNEL);            ///< enable NEC reiceive
    while(1)
    {
        NecRxPollingProc();
    }
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
