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
 ** This example demonstrates CEC mode of Remote Control operation
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


/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static stc_rcn_t* pstcRc =  ((stc_rcn_t *) FM_HDMICEC0_BASE);

static    uint8_t u8Addr, u8Count;                      ///< 'u8Addr' for get address, 'u8Count' for count the data length
static    uint32_t u32Dat;                              ///< receive data( there are only three bytes for the test )

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief  Init RcCec
 **
 ** \retval Ok                    Interrupt disabled normally
 ** \retval  NULL
 **                               
 ******************************************************************************/
static void InitCec(void)
{
    stc_rc_rx_cec_config_t stcRcCecConfig;

    // Clear structures
    PDL_ZERO_STRUCT(stcRcCecConfig);

    
    // Configure the Sircs parameter
    stcRcCecConfig.u16DivVal = RC_DIV;
    stcRcCecConfig.enSrcClk  = RcPeripheralClk;             ///< set peripheral clk
    stcRcCecConfig.enThresholdType = RcThresholdType1;      ///< when W>MinPulseWidth and W < u8ThresholdWidth, the value is '0'\
                                                                when W>MinPulseWidth and W >= u8ThresholdWidth, the value is '1'
/**
 ******************************************************************************
 ** sign width:
 **     1. '0'          600us
 **     2. '1'          1.56ms
 **     3. Start bit    3.8ms                                                                
 ******************************************************************************/
    stcRcCecConfig.u8MinPulseWidth = 13u;                   ///< set min pulse width 400us
    stcRcCecConfig.u8ThresholdWidth = 48u;                  ///< set threshold width 1.5ms
    stcRcCecConfig.u8StartBitWidth = 114u;                  ///< set start bit width 3.5ms
    stcRcCecConfig.u8MaxDataWidth = 91u;                    ///< set max data width   2.8ms 
    stcRcCecConfig.u8MinDataWidth = 65u;                    ///< set min data width   2ms 
    stcRcCecConfig.enOverflowCycle = RcOverflow256Cycle;     
    stcRcCecConfig.bBusErrorPulseOutput = TRUE;    
    stcRcCecConfig.stcAddr.u8Addr1 = TEST_ADDR;
    stcRcCecConfig.stcAddr.u8Addr2 = 0u;
    stcRcCecConfig.bAddrCmpEn = TRUE;                      ///< enable address comparison
    
    // Configure irq
    
    Rc_Rx_Cec_Init((stc_rcn_t*)&RC_CHANNEL ,&stcRcCecConfig);
    FM_LCDC->LCDCC3_f.PICTL = 1u;				///< set LCD port is not cut off
    SetPinFunc_CEC();                                       ///< set rc pin
    
}

/**
 ******************************************************************************
 ** \brief  RcCec receive process
 **
 ** \retval Ok                    Interrupt disabled normally
 ** \retval Error                 address unmatched
 **                               data width error
 ******************************************************************************/
static en_result_t CecRxProcess(void)
{
    u32Dat = 0;
    u8Addr = 0;
    u8Count = 0;
    Rc_Rx_Cec_EnableRx((stc_rcn_t*)&RC_CHANNEL);                                ///< enable CEC reiceive
    while(0u == Rc_Rx_Cec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecStartIrq)); ///< waitting for start bit flag            
    if((1u == Rc_Rx_Cec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecMinDataIrq))|| \
      (1u == Rc_Rx_Cec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecMaxDataIrq))) ///< data detection
    {
        return Error;
    }
    while(0u == Rc_Rx_Cec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecAckIrq));      ///< if data received
    u8Addr = pstcRc->RCDTHH;                                                 ///< get receive data                                              
    Rc_Rx_Cec_ClrIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecAckIrq);            ///< clear ack for get data next byte
    while(1)
    {
        if((1u == Rc_Rx_Cec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecMinDataIrq))|| \
            (1u == Rc_Rx_Cec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecMaxDataIrq))) ///< data detection
        {
            Rc_Rx_Cec_DisableRx((stc_rcn_t*)&RC_CHANNEL);                        ///< disable Rx for clear flags
            return Error;
        }
        if(1u == Rc_Rx_Cec_GetIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecAckIrq))  ///< if data received
        { 
            u32Dat |= (pstcRc->RCDTHH << u8Count*8u);                           ///< get receive data                                                         
            Rc_Rx_Cec_ClrIrqFlag((stc_rcn_t*)&RC_CHANNEL, RcRxCecAckIrq);       ///< clear ack for get data next byte
            u8Count ++;  
        }
        if(1u == Rc_Rx_Cec_GetEomState((stc_rcn_t*)&RC_CHANNEL))                ///< data complete
        {
           u32Dat |= (pstcRc->RCDTHH << u8Count*8u);
           if(TEST_ADDR != u8Addr)                                                   ///< device compare
           {
                Rc_Rx_Cec_DisableRx((stc_rcn_t*)&RC_CHANNEL);                        ///< disable Rx for clear flags
                return Error; 
           }
#ifdef DEBUG_PRINT
            printf("The address is  : 0x%x\n", u8Addr);                             ///< print address 
            printf("The data is  : 0x%x\n", u32Dat);                            ///< print data value
#endif 
           Rc_Rx_Cec_DisableRx((stc_rcn_t*)&RC_CHANNEL);                        ///< disable Rx for clear flags
           return Ok; 
        }
    
    }

}
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
    InitCec();                                              ///< initial CEC
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("CEC rx polling Example Program Start \n");
    printf("==================================================\n");
#endif
    while(1)
    {
        CecRxProcess();
    }
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
