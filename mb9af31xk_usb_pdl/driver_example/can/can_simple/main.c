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
/************************************************************************/
/** \file main.c
 **
 ** CAN example.
 **
 ** History:
 **   - 2013-08-21  0.1  MWi  First version
 **   - 2015-03-07  0.2  EZh  Port to FM universal PDL
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
#include "string.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#define CAN_MAX_RX_TEST_MESSAGES 2

/******************************************************************************/
/* Global variable definitions ('static')                                     */
/******************************************************************************/
static uint8_t u8StatusCounter = 0;
static uint8_t u8ErrorCounter = 0;
static uint8_t u8TransmitCounter = 0;
static uint8_t u8ReceiveCounter = 0;
static boolean_t bReceiveFlag = 0;
static uint32_t u32MsgIdMask1;
volatile static en_can_status_t aenStatus[CAN_MAX_RX_TEST_MESSAGES];
static stc_can_config_t   stcCanConfig;
static stc_can_msg_data_t stcMsgData;
static stc_can_msg_id_t   stcMsgId;
static stc_can_msg_t      stcMsgBuffer1;
static stc_can_msg_t      stcMsgBuffer2;

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/******************************************************************************
 ** \brief Can Transmission Complete Callback
 ******************************************************************************/
static void CanTxCompleteCb(uint8_t u8MsgBuf)
{
  u8TransmitCounter++;
}

/******************************************************************************
 ** \brief Can Reception Callback
 ******************************************************************************/
static void CanRxReceivedCb(uint8_t u8MsgBuf, stc_can_msg_t* pstcRxMsg)
{
  u8ReceiveCounter++;
  bReceiveFlag = 1u;
}

/******************************************************************************
 ** \brief Can Status Callback
 ******************************************************************************/
static void CanStatusCb(en_can_status_t enCanStatus)
{
  aenStatus[u8StatusCounter] = enCanStatus;  // Collect CAN status callback data

  u8StatusCounter++;
}

/******************************************************************************
 ** \brief Can Error Callback
 ******************************************************************************/
static void CanErrorCb(en_can_error_t enCanError)
{
  u8ErrorCounter++;
}

/**
 ******************************************************************************
 ** \brief CAN0 example
 **
 ** This test requires a configured and connected CAN-USB tool.
 **
 ** After Can_Init() to 100k Bit/s. The echo test is implemented. And
 ** if CAN-USB tool send the data frame with ID = 0x0A, the data will be echo
 ** back. Otherwise, the data frame is filtered.
 **
 ******************************************************************************/
int32_t main(void)
{
  stc_cann_t*         pstcCan        = NULL; // CAN instance pointer
  stc_can_msg_id_t*   pstcMsgId      = NULL; // Message ID pointer
  stc_can_msg_data_t* pstcMsgData    = NULL; // Message Data pointer
  stc_can_msg_t*      pstcMsgBuffer1 = NULL; // Message Buffer 1 pointer
  
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF50X)   // FSSDC-9B506-EVB
  Gpio1pin_InitOut(GPIO1PIN_P64, Gpio1pin_InitVal(0u)); // Set STB pin to low
#endif
  
  SetPinFunc_RX0_2(0u);         // CAN reception pin
  SetPinFunc_TX0_2(0u);         // CAN transmission pin

  // Set CAN instance
  pstcCan = (stc_cann_t*)(&CAN0);

  // Set Message ID pointer
  pstcMsgId = &stcMsgId;

  // Set Message Data pointer
  pstcMsgData = &stcMsgData;

  // Set Message Buffer 1 pointer
  pstcMsgBuffer1 = &stcMsgBuffer1;

  stcCanConfig.pfnStatusCallback = (can_status_chg_func_ptr_t)CanStatusCb;
  stcCanConfig.pfnErrorCallback = (can_error_func_ptr_t)CanErrorCb;
  stcCanConfig.bDisableAutomaticRetransmission = FALSE;

#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF50X)   // FSSDC-9B506-EVB
  // 100k Bit/s
  // Fsys / (TSEG1 + TSEG2 + 1)
  // (16M / (13 + 6 + 1)*8)
  stcCanConfig.bTouchPrescaler = TRUE;
  stcCanConfig.stcBitrate.u8TimeSegment1  = 13;
  stcCanConfig.stcBitrate.u8TimeSegment2  = 6;
  stcCanConfig.stcBitrate.u8SyncJumpWidth = 4;
  stcCanConfig.stcBitrate.u16Prescaler = 8;
  stcCanConfig.stcBitrate.enCanPrescaler = CanPreDiv15; // 80 MHz / 5 = 16 MHz
#else                      // SK-FM4-216-ETHERNET
  // 100k Bit/s
  // Fsys / (TSEG1 + TSEG2 + 1)
  // (40M / (13 + 6 + 1)*20)
  stcCanConfig.bTouchPrescaler = TRUE;
  stcCanConfig.stcBitrate.u8TimeSegment1  = 13;
  stcCanConfig.stcBitrate.u8TimeSegment2  = 6;
  stcCanConfig.stcBitrate.u8SyncJumpWidth = 4;
  stcCanConfig.stcBitrate.u16Prescaler = 20;
  stcCanConfig.stcBitrate.enCanPrescaler = CanPreDiv15; // 200 MHz / 5 = 40 MHz
#endif

  // RX buffer 1 settings (CANoe auto-response)
  stcMsgBuffer1.stcIdentifier.u32Identifier = 0x0A;
  stcMsgBuffer1.stcIdentifier.bExtended = FALSE;
  u32MsgIdMask1 = 0xFFFFFFFF; // Compare all the IDE bits

  if (Ok == Can_Init(pstcCan, &stcCanConfig))
  {
    u8TransmitCounter = 0;
    u8ReceiveCounter = 0;

    while(1)
    {
        // Set message buffer 2 to receive buffer
        Can_SetReceiveMsgBuffer(pstcCan, 2, pstcMsgBuffer1, u32MsgIdMask1, (can_rx_msg_func_ptr_t)CanRxReceivedCb);

        // Wait for receive finish
        while(1u != bReceiveFlag);
        bReceiveFlag = 0u;

        // TX buffer settings
        stcMsgId.u32Identifier = 0x12 ;
        stcMsgId.bExtended = FALSE ;

        // Prepare for transfer
        Can_SetTransmitMsgBuffer(pstcCan, 1, pstcMsgId, (can_tx_msg_func_ptr_t)CanTxCompleteCb);

        // Send back the data received
        pstcMsgData->u8DataLengthCode = 8u;
        memcpy(pstcMsgData->au8Data, &pstcMsgBuffer1->stcData, pstcMsgData->u8DataLengthCode);

        Can_UpdateAndTransmitMsgBuffer(pstcCan, 1, pstcMsgData, CanImmediateTransmit);
    }
  }
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
