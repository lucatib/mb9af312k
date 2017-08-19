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
boolean_t bTxBusErrorSta;
boolean_t bTxCompleteSta;

uint8_t au8Testbuf[TEST_SIZE]= {0x11,0x22,0x33};			///< test data is '0x332211'					
/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
/**
 ******************************************************************************
 ** \brief  Cec polling Tx init 
 **
 ** \retval Ok                    Data has been successfully sent
 ** \retval Error                 bus error
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **             - pstcConfig == NULL
 ******************************************************************************/
static en_result_t CecTxPollingInit(void)
{
    stc_rc_tx_cec_config_t stcRcTxConfig;
    
    // Clear structures
    PDL_ZERO_STRUCT(stcRcTxConfig);
        
    // Configure the Tx parameter
    stcRcTxConfig.u16DivVal = RC_DIV;
    stcRcTxConfig.enSrcClk = RcPeripheralClk;  
    stcRcTxConfig.u8FreeCycle = 7;                                      ///<Set 8 cycle free time
    if(ErrorInvalidParameter == Rc_Tx_Cec_Init((stc_rcn_t*)&RC_CHANNEL ,&stcRcTxConfig))
    {
        return ErrorInvalidParameter;
    }
    if(ErrorInvalidParameter == Rc_Tx_Cec_EnableTx((stc_rcn_t*)&RC_CHANNEL))    ///< enable tx
    {
        return ErrorInvalidParameter;
    }
    return Ok; 
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
    uint32_t u32Delay; 
#if !defined(GPIO1PIN_P50_INIT)
#error P50 is not available in this MCU product, \
change to other re-location pin! Then delete "me" !
#endif
    Gpio1pin_InitIn ( GPIO1PIN_P50, Gpio1pin_InitPullup(0u));                   ///< set the key IO, Output P50, Value: 0 
    switch(CecTxPollingInit())                                                  ///< Init cec transmission block with interrupt mode
    {
        case ErrorInvalidParameter:
        ///< add Invalid parameter function
        break;
        case Ok:
        ///< add init OK function
        break;
    }  
    SetPinFunc_CEC();
    while(1)
    {
        if(0u == Gpio1pin_Get(GPIO1PIN_P50))                                    ///< wait for key press
        {
            while(0u == Gpio1pin_Get(GPIO1PIN_P50));                            ///< wait for key unpress
            switch(Rc_Tx_Cec_SendDataPolling((stc_rcn_t*)&RC_CHANNEL, TEST_ADDR ,\
                  (uint8_t*)au8Testbuf, sizeof(au8Testbuf)))                    ///< send the CEC data polling
            {
                case ErrorInvalidParameter:
                ///< add Invalid parameter function
                break;
                case Error:
                ///< add bus Error or time out function
                break;  
                case Ok:
                ///< add send OK function
                break;
            }                                                       ///< send the CEC data
            u32Delay = 0xfffffffu/(SystemCoreClock/ 5000u ) ;     ///< delay 
            while(u32Delay--);
        }    
    }
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
