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
 ** CAN FD example.
 ** 
 ** History:
 **   - 2014-08-28  1.0  AI   First version.
 **   - 2015-03-26  1.1  EZh  Port to FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Sample application control definitions                                     */
/******************************************************************************/
#define	MCANSAMPLE_MODE				(1)		// Specify master/slave mode.
											//   0 : Master mode. Sample notifies message (ID:0x100) periodically.
											//   1 : Slave mode. Sample performs to send reply messages (ID:0x200) only.

#define	MCANSAMPLE_CHANNELS			(1)		// Specify number of CAN FD channels.
											//   Currently, this is applied as the size of channel related arrays.

											// Specify CAN operation mode.
#define	MCANSAMPLE_CANMODE			(CanfdModeClassic)
											//   Set one of following mode definition.
											//     CanfdModeClassic : CAN 2.0 classic
											//     CanfdModeFDFixed : CAN FD with fixed data rate
											//     CanfdModeFDFlex  : CAN FD with flexible data rate

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
// Check CAN FD module activation.
// CAN FD 0 must be activated.
#if (PDL_PERIPHERAL_ENABLE_CANFD0 != PDL_ON)
	#error "CAN FD channel 0 must be configured to use."
#endif

#define	MCANSAMPLE_WAITLOOPS		(SystemCoreClock / 3300)	// Loop count to wait during 100ms.
																// (SystemCoreClock = PLL = 200MHz)

#define	MCANSAMPLE_ID_NOTIFY		(0x100)		// Standard ID (0x100) of the message that is sent periodically.
#define	MCANSAMPLE_ID_RETURN		(0x200)		// Standard ID (0x200) of the message to reply to the above message.
#define MCANSAMPLE_NUM_BUS_STATUS	(2)			// Number of the kind of bus statuses.

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
static void CanfdTxCompleteCb0(uint8_t u8MsgBuf);
static void CanfdRxReceivedCb0(uint8_t u8MsgBuf, stc_canfd_msg_t* pstcRxMsg);
static void CanfdStatusCb0(en_canfd_status_t enCanfdStatus);
static void CanfdErrorCb0(uint32_t u32CanfdError);

static void CopyMessage(stc_canfd_msg_t* pstcSrc, stc_canfd_msg_t* pstcDst);
static void assign_pins_canfd(void);

// Implemented only for using CAN FD board.
static void CanfdDriverInit(void);
static void CanfdDriverWait(uint8_t c);
static void CanfdDriverSetMode(uint8_t c, uint8_t m);

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static uint16_t u16StatusCounter[MCANSAMPLE_CHANNELS];
static uint16_t u16ErrorCounter[MCANSAMPLE_CHANNELS];
static uint16_t u16TransmitCounter[MCANSAMPLE_CHANNELS];
static uint16_t u16ReceiveCounter[MCANSAMPLE_CHANNELS];
static uint16_t u16SequenceCounter;
static uint16_t u16RecoverCounter;
volatile static uint16_t u16Status[MCANSAMPLE_CHANNELS][MCANSAMPLE_NUM_BUS_STATUS];
static stc_canfd_config_t   stcCanfdConfig;

/******************************************************************************/
/* Function implementation - global (no 'static') and local ('static')        */
/******************************************************************************/

/******************************************************************************/
/* Global Functions                                                           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief CAN FD example
 **
 ** This sample application is built as the master or the slave mode.
 ** The macro 'MCANSAMPLE_MODE' specifies building mode. If it was set to zero, 
 ** this is built as the master mode. Otherwise this is built as the slave mode.
 ** Behavior of each modes is as follows;
 **
 ** > Sends the notification message with following contents at 10ms interval.
 **   (Master mode only)
 **     Message ID  : Standard ID, 0x100.
 **     Data length : 8 bytes (classic CAN).
 **                   64 bytes (CAN FD).
 **     Data        : SS SS RR RR FF FF 00 00 (classic CAN)
 **                   SS SS RR RR FF FF 00 00 .. 00 00 (CAN FD)
 **     Contents (Little endian format)
 **       SSSS = Sequence number.
 **       RRRR = Reply message reception counter.
 **       FFFF = Fault recovery counter.
 ** > Sends the reply message with following contents when this application 
 **   received the notification message.
 **     Message ID  : Standard ID, 0x200.
 **     Data length : 8 bytes (in classic CAN) or 64 bytes (in CAN FD).
 **     Data        : Copy of received notification message.
 ******************************************************************************/
int32_t main(void)
{
	stc_canfdn_t*		pstcCanfd0 = NULL; 	// CAN FD 0 instance pointer
	stc_canfd_msg_t		stcMsg;
	en_result_t			enResCanfd0;
	uint32_t			u32Count;
	uint8_t				u8Idx;

	// Update system clock frequency information
	SystemCoreClockUpdate();

	// Assign CAN FD interface to external pins.
	assign_pins_canfd();

	// Set CAN FD instance
	pstcCanfd0 = (stc_canfdn_t*)(&CANFD0);

	CanfdDriverInit();			// Initialzie CSIO to control the tranceiver.
	CanfdDriverWait(0);			// Wait until transceivers wake up.
	CanfdDriverSetMode(0, 7);	// Change mode of transceivers to normal.

	// Initialize prescaler
	//   PLL(200MHz) / 5 = 40MHz
	Canpre_Init(CanfdPreDiv15);

	// Set configuration parameters of CAN FD #0.
	// (Modes)
	stcCanfdConfig.enCanfdMode = MCANSAMPLE_CANMODE;
	// (CAN prescaler output clock frequency)
	stcCanfdConfig.enCanfdClock = CanfdClock40MHz;	// CAN operation clock : 40MHz
	// (Callbacks)
	stcCanfdConfig.pfnReceiveMsgCallback  = CanfdRxReceivedCb0;
	stcCanfdConfig.pfnTransmitMsgCallback = CanfdTxCompleteCb0;
	stcCanfdConfig.pfnStatusCallback      = CanfdStatusCb0;
	stcCanfdConfig.pfnErrorCallback       = CanfdErrorCb0;

	// Initialize CAN FD #0.
	enResCanfd0 = Canfd_Init(pstcCanfd0, &stcCanfdConfig);

	if (enResCanfd0 == Ok)
	{
		// Clear counters.
		u16TransmitCounter[0] = 0;
		u16ReceiveCounter[0] = 0;
		u16StatusCounter[0] = 0;
		u16ErrorCounter[0] = 0;

		u16SequenceCounter = 0;
		u16RecoverCounter = 0;

		// Start CAN communication.
		Canfd_Start(pstcCanfd0);

		// Repeat permanently.
		while (1)
		{
			// Wait about 10ms
			for (u32Count = 0; u32Count < MCANSAMPLE_WAITLOOPS; u32Count++)
			{
				// Polling messages and get bus status
				Canfd_ReceiveMsg(pstcCanfd0);
				Canfd_GetBusStatus(pstcCanfd0);
			}

			// Master mode
			#if defined(MCANSAMPLE_MODE) && MCANSAMPLE_MODE == 0

				// Send cyclic notification via CAN FD 0
				// (ID : standard, 0x100)
				stcMsg.stcIdentifier.u32Identifier = MCANSAMPLE_ID_NOTIFY;
				stcMsg.stcIdentifier.bExtended = FALSE;
				// (data)
				stcMsg.stcData.au32Data[0] = (uint32_t)u16SequenceCounter
										   | ((uint32_t)u16ReceiveCounter[0] << 16);
				stcMsg.stcData.au32Data[1] = u16RecoverCounter;
				u16SequenceCounter++;
				if (stcCanfdConfig.enCanfdMode == CanfdModeClassic)
				{
					stcMsg.stcData.u8DataLengthCode = 8;
					stcMsg.bCanfd = FALSE;
				}
				else
				{
					for (u8Idx = 2; u8Idx < CANFD_MESSAGE_DATA_BUFFER_SIZEW; u8Idx++)
					{
						stcMsg.stcData.au32Data[u8Idx] = 0;
					}
					stcMsg.stcData.u8DataLengthCode = 15;
					stcMsg.bCanfd = TRUE;
				}
				// (Request to send)
				Canfd_TransmitMsg(pstcCanfd0, 0, &stcMsg);

			#endif
		}
	}
}


/******************************************************************************/
/* Local Functions                                                            */
/******************************************************************************/

/******************************************************************************
 ** \brief CAN FD Transmission Complete Callbacks
 **
 ** \param [in] u8MsgBuf              Index of transmitted Tx buffer.
 ******************************************************************************/
static void CanfdTxCompleteCb0(uint8_t u8MsgBuf)
{
	u16TransmitCounter[0]++;
}


/******************************************************************************
 ** \brief CAN FD Reception Callbacks
 **
 ** \param [in] u8MsgBuf              Index of received Rx buffer.
 ** \param [in] pstcRxMsg             Address of the message content.
 ******************************************************************************/
static void CanfdRxReceivedCb0(uint8_t u8MsgBuf, stc_canfd_msg_t* pstcRxMsg)
{
	stc_canfd_msg_t	stcMsg;

	// Udate receive counter.
	u16ReceiveCounter[0]++;

	// If received message is the notification message (ID=0x100), ...
	if (pstcRxMsg->stcIdentifier.u32Identifier == MCANSAMPLE_ID_NOTIFY)
	{
		// Return same message with ID 0x200.
		CopyMessage(pstcRxMsg, &stcMsg);
		stcMsg.stcIdentifier.u32Identifier = MCANSAMPLE_ID_RETURN;
		while (Canfd_TransmitMsg((volatile stc_canfdn_t*)&CANFD0, 0, &stcMsg) == ErrorOperationInProgress)
			;
	}
}


/******************************************************************************
 ** \brief CAN FD Status Callbacks
 **
 ** \param [in] enCanfdStatus         Detected status.
 ******************************************************************************/
static void CanfdStatusCb0(en_canfd_status_t enCanfdStatus)
{
	// BusOff state.
	if ( enCanfdStatus == CanfdBusOff )
	{
		u16Status[0][0]++;
		Canfd_Restart((volatile stc_canfdn_t*)(&CANFD0));
		u16RecoverCounter++;
	}
	// ErrorWarning state.
	else if ( enCanfdStatus == CanfdWarning )
	{
		u16Status[0][1]++;
	}
	else
	{
		// Do nothing.
	}

	u16StatusCounter[0]++;
}


/******************************************************************************
 ** \brief CAN FD Error Callbacks
 **
 ** The parameter 'u32CanfdError' includes all error flags that were detected at 
 ** a time. This value consists of some ORed 'CANFD_ERROR_*' values.
 **
 ** \param [in] u32CanfdError         Flags of detected errors.
 ******************************************************************************/
static void CanfdErrorCb0(uint32_t u32CanfdError)
{
	u16ErrorCounter[0]++;
}


/******************************************************************************
 ** \brief Copy content of intarnal CAN message structure
 **
 ** Copy whole of message structure 'stc_canfd_msg_data_t' content from source 
 ** to destination.
 **
 ** \param [in]     pstcSrc           Address of source message structure.
 ** \param [in/out] pstcDst           Address to store the content of source.
 ******************************************************************************/
static void CopyMessage(stc_canfd_msg_t* pstcSrc, stc_canfd_msg_t* pstcDst)
{
	uint8_t	u8Idx;

	// Copy if both source and destination pointers are valid.
	if (pstcSrc != NULL && pstcDst != NULL)
	{
		// Copy ID members.
		pstcDst->stcIdentifier.u32Identifier = pstcSrc->stcIdentifier.u32Identifier;
		pstcDst->stcIdentifier.bExtended     = pstcSrc->stcIdentifier.bExtended;

		// Copy data members.
		for (u8Idx = 0; u8Idx < CANFD_MESSAGE_DATA_BUFFER_SIZEW; u8Idx++)
		{
			pstcDst->stcData.au32Data[u8Idx] = pstcSrc->stcData.au32Data[u8Idx];
		}
		pstcDst->stcData.u8DataLengthCode = pstcSrc->stcData.u8DataLengthCode;

		// Copy extended DLC flag.
		pstcDst->bCanfd = pstcSrc->bCanfd;
	}
}

/******************************************************************************
 ** \brief Assign CAN FD interface to external pins.
 ******************************************************************************/
// channel-2
static void assign_pins_canfd(void)
{
    SetPinFunc_TX2_2();
    SetPinFunc_RX2_2();
}


/******************************************************************************
 ** \brief Initialize CSIO I/F to control CAN FD transceiver
 **
 ** Initializes the MFS as the CSIO to control the CAN FD transceiver.
 ** This function premises following environment;
 ** > Implemented CAN FD transceiver is NXP TJA1145T/FD.
 ** > Following CSIO signals are connected to the transceiver;
 **     CAN FD 0 : MFS ch.12 - SCK12_1, SIN12_1, SOT6_12, CS: P0F
 ******************************************************************************/
static void CanfdDriverInit(void)
{
    stc_mfs_csio_config_t stcCsioConfig;
    
    PDL_ZERO_STRUCT(stcCsioConfig);
    
	// Assign pins for MFS/CSIO 
    SetPinFunc_SCK12_1();
    SetPinFunc_SIN12_1();
    SetPinFunc_SOT12_1();
    
    // CS P7E
    Gpio1pin_InitOut(GPIO1PIN_P7E, Gpio1pin_InitVal(1u));
    
    // Initialize CSIO
    stcCsioConfig.enMsMode = CsioMaster;
    stcCsioConfig.enActMode = CsioActNormalMode;
    stcCsioConfig.bInvertClk = TRUE;
    stcCsioConfig.u32BaudRate = 500000;
    stcCsioConfig.enDataLength = CsioSixteenBits;
    stcCsioConfig.enBitDirection = CsioDataMsbFirst;
    stcCsioConfig.enSyncWaitTime = CsioSyncWaitZero;
    stcCsioConfig.pstcFifoConfig = NULL;
    
    // Initialize CSIO
    Mfs_Csio_Init(&CSIO12, &stcCsioConfig);
    
}

/******************************************************************************
 ** \brief Wait CAN FD transceiver start
 **
 ** Wait until correct ID was read from the CAN FD transceiver.
 ** Identification register (0x7E) holds the device ID as follows;
 **   Device ID : 70h TJA1145T, TJA1145TK
 **               74h TJA1145T/FD, TJA1145TK/FD
 ** The transceiver on the target environment is TJA1145T/FD(ID=0x74).
 **
 ** \param [in] c                     CAN FD channel number (0-).
 ******************************************************************************/
static void CanfdDriverWait(uint8_t c)
{
    uint16_t u16Data;
	// Check parameter.
	if (c == 0)
	{
        // CS = L
        Gpio1pin_Put(GPIO1PIN_P7E, 0u);
      
		// Enable Rx/Tx.
        Mfs_Csio_EnableFunc(&CSIO12, CsioTx);
        Mfs_Csio_EnableFunc(&CSIO12, CsioRx);

		do {
			// Set data count.
            u16Data = (0x7Eul << 9)		// Get ID commnad
					   | (1ul << 8)	    // R/O = 1 (read only)
					   | 0ul;           // Don't care
          
			// Send command to read ID (ReadOnly).
            while (TRUE != Mfs_Csio_GetStatus(&CSIO12, CsioTxEmpty)); // wait until TX buffer empty 
            Mfs_Csio_SendData(&CSIO12, u16Data, TRUE);  
            
            // Wait until master TX bus idle
            while (TRUE != Mfs_Csio_GetStatus(&CSIO12, CsioTxIdle));
           
		// Check obtained ID.
		} while ((Mfs_Csio_ReceiveData(&CSIO12) & 0xFFul) != 0x74u);
        
		// Disable Rx/Tx.        
        Mfs_Csio_DisableFunc(&CSIO12, CsioTx);
        Mfs_Csio_DisableFunc(&CSIO12, CsioRx);
        
        // CS = H
        Gpio1pin_Put(GPIO1PIN_P7E, 1u);
	}
}

/******************************************************************************
 ** \brief Change CAN FD transceiver mode
 **
 ** Changes the CAN FD transceiver mode for specified channel.
 ** In the target environment, TJA1145 is mounted. This transceiver transits to 
 ** the 'Standby' mode after starting by turning the power on. So its mode should 
 ** be changed to the 'Normal' mode to communicate via CAN.
 **
 ** \param [in] c                     CAN FD channel number (0-).
 ** \param [in] m                     CAN FD transceiver mode
 **                                     1(001B) : Sleep mode
 **                                     4(100B) : Standby mode
 **                                     7(111B) : Normal mode
 ******************************************************************************/
static void CanfdDriverSetMode(uint8_t c, uint8_t m)
{
    uint16_t u16Data;
	// Check parameter.
	if (c == 0)
	{
        // CS = L
        Gpio1pin_Put(GPIO1PIN_P7E, 0u);
      
		// Enable Rx/Tx.        
        Mfs_Csio_EnableFunc(&CSIO12, CsioTx);
        Mfs_Csio_EnableFunc(&CSIO12, CsioRx);

		// Send command to change mode.        
        u16Data = (0x01u << 9)		// Mode control command.
					 | (0u << 8)		// R/O = 0 (read and write)
					 | (uint16_t)m;		// Mode.

        while (TRUE != Mfs_Csio_GetStatus(&CSIO12, CsioTxEmpty)); // wait until TX buffer empty 
        Mfs_Csio_SendData(&CSIO12, u16Data, TRUE);  
        
        // Wait until master TX bus idle
        while (TRUE != Mfs_Csio_GetStatus(&CSIO12, CsioTxIdle));
        
		// Disable Rx/Tx.        
        Mfs_Csio_DisableFunc(&CSIO12, CsioTx);
        Mfs_Csio_DisableFunc(&CSIO12, CsioRx);

        // CS = H
        Gpio1pin_Put(GPIO1PIN_P7E, 1u);
	}
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/  
