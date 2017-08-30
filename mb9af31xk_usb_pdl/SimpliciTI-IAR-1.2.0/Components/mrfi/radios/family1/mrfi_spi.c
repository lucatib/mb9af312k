/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007 Texas Instruments Incorporated.  All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com. 
**************************************************************************************************/

/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Radios: CC2500, CC1100, CC1101
 *   SPI interface code.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
 
#include "bsp.h"
#include "mrfi_board_defs.h"
#include "mrfi_spi.h"

/* ------------------------------------------------------------------------------------------------
 *                                            Defines
 * ------------------------------------------------------------------------------------------------
 */
#define DUMMY_BYTE                  0xDB

#define READ_BIT                    0x80
#define BURST_BIT                   0x40

/* ------------------------------------------------------------------------------------------------
 *                                            Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MRFI_SPI_TURN_CHIP_SELECT_ON()        MRFI_SPI_DRIVE_CSN_LOW()
#define MRFI_SPI_TURN_CHIP_SELECT_OFF()       MRFI_SPI_DRIVE_CSN_HIGH()
#define MRFI_SPI_CHIP_SELECT_IS_OFF()         MRFI_SPI_CSN_IS_HIGH()

#define MRFI_SPI_DEBUG
#ifdef MRFI_SPI_DEBUG
#define MRFI_SPI_ASSERT(x)      			BSP_ASSERT(x)
#else
#define MRFI_SPI_ASSERT(x)
#endif


/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */

static uint8_t spiRegAccess(uint8_t addrByte, uint8_t writeValue);
static bool spiBurstFifoAccess(uint8_t addrByte, uint8_t * pData, uint8_t len);


#ifdef MRFI_TIMER_ALWAYS_ACTIVE
bool sActiveSPI = false;
#endif

/**************************************************************************************************
 * @fn          mrfiSpiInit
 *
 * @brief       Initialize SPI.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void mrfiSpiInit(void){
#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  sActiveSPI = false; // initialize interface status
#endif
  //mrfiSpiIState_t s;

  /* configure all SPI related pins */
  MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT();
  MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT();
  MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT();	
  MRFI_SPI_CONFIG_SO_PIN_AS_INPUT();	

  /* set CSn to default high level */
  MRFI_SPI_DRIVE_CSN_HIGH();
  
  /* initialize the SPI registers */
  MRFI_SPI_ENTER_CRITICAL_SECTION(s);
  MRFI_SPI_INIT();
  MRFI_SPI_EXIT_CRITICAL_SECTION(s);
} 


/**************************************************************************************************
 * @fn          mrfiSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio.  Returns status byte read during transfer
 *              of strobe command.
 *
 * @param       addr - address of register to strobe
 *
 * @return      status byte of radio
 **************************************************************************************************
 */
uint8_t mrfiSpiCmdStrobe(uint8_t addr){
		uint8_t statusByte;
//  mrfiSpiIState_t s;

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  bool comm_state = sActiveSPI; // save comm state
#endif

  MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() );       /* SPI is not initialized */
  MRFI_SPI_ASSERT((addr >= 0x30) && (addr <= 0x3D));  /* invalid address */

  /* disable interrupts that use SPI */
  MRFI_SPI_ENTER_CRITICAL_SECTION(s);

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  sActiveSPI = true;            // indicate active comm state
#endif

  /* turn chip select "off" and then "on" to clear any current SPI access */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_TURN_CHIP_SELECT_ON();

	Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
	Mfs_Csio_EnableFunc(CsioCh0, CsioRx);

	while (TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty)); /* wait until TX buffer empty */
	Mfs_Csio_SendData(CsioCh0, addr , TRUE);  
	while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioRxFull)); /* wait until RX buffer full */
  statusByte = Mfs_Csio_ReceiveData(CsioCh0);	//Get status 	 
	
	Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
  Mfs_Csio_DisableFunc(CsioCh0, CsioRx);

#ifdef MRFI_TIMER_ALWAYS_ACTIVE
  sActiveSPI = comm_state; // restore comm state
#endif

  MRFI_SPI_WAIT_DONE();

  /* turn off chip select; enable interrupts that call SPI functions */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();

  MRFI_SPI_EXIT_CRITICAL_SECTION(s);

  /* return the status byte */
	return statusByte;
}


/**************************************************************************************************
 * @fn          mrfiSpiReadReg
 *
 * @brief       Read value from radio register.
 *
 * @param       addr - address of register
 *
 * @return      register value
 **************************************************************************************************
 */
uint8_t mrfiSpiReadReg(uint8_t addr){
  uint8_t mval;
	
	MRFI_SPI_ASSERT(addr <= 0x3B);    /* invalid address */
  
  /*
   *  The burst bit is set to allow access to read-only status registers.
   *  This does not affect normal register reads.
   */
	
  MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() );   /* SPI is not initialized */

  /* disable interrupts that use SPI */
  MRFI_SPI_ENTER_CRITICAL_SECTION(s);
	
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_TURN_CHIP_SELECT_ON();
	
	mval = spiRegAccess(addr | BURST_BIT | READ_BIT, 0);
	
  /* turn off chip select; enable interrupts that call SPI functions */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();
	
	MRFI_SPI_EXIT_CRITICAL_SECTION(s);
		
	return mval;
}


/**************************************************************************************************
 * @fn          mrfiSpiWriteReg
 *
 * @brief       Write value to radio register.
 *
 * @param       addr  - address of register
 * @param       value - register value to write
 *
 * @return      none
 **************************************************************************************************
 */
void mrfiSpiWriteReg(uint8_t addr, uint8_t value){
  MRFI_SPI_ASSERT((addr <= 0x2E) || (addr == 0x3E));    /* invalid address */

  MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() );   /* SPI is not initialized */

  /* disable interrupts that use SPI */
  MRFI_SPI_ENTER_CRITICAL_SECTION(s);
	
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_TURN_CHIP_SELECT_ON();
	
	spiRegAccess(addr,value);
	
  /* turn off chip select; enable interrupts that call SPI functions */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();	
	
	MRFI_SPI_EXIT_CRITICAL_SECTION(s);
}

/**************************************************************************************************
 * @fn          mrfiSpiWriteTxFifo
 *
 * @brief       Write data to radio transmit FIFO.
 *
 * @param       pData - pointer for storing write data
 * @param       len   - length of data in bytes
 *
 * @return      true if an interrupt was detected during the transfer, false otherwise
 **************************************************************************************************
 */
bool mrfiSpiWriteTxFifo(uint8_t * pData, uint8_t len){
  return spiBurstFifoAccess(TXFIFO , pData, len);
}

/**************************************************************************************************
 * @fn          macSpiReadRxFifo
 *
 * @brief       Read data from radio receive FIFO.
 *
 * @param       pData - pointer for storing read data
 * @param       len   - length of data in bytes
 *
 * @return      true if an interrupt was detected during the transfer, false otherwise
 **************************************************************************************************
 */
bool mrfiSpiReadRxFifo(uint8_t * pData, uint8_t len){
  return spiBurstFifoAccess(RXFIFO | READ_BIT, pData, len);
}

//2 Bytes for OOK modulation
bool mrfiSpiWritePaTable(uint8_t * pData, uint8_t len){
	return spiBurstFifoAccess(PA_TABLE0 , pData, len);
}
bool mrfiSpiReadPaTable(uint8_t * pData, uint8_t len){
	return spiBurstFifoAccess(PA_TABLE0 | READ_BIT , pData, len);
}

/*=================================================================================================
 * @fn          spiBurstFifoAccess
 *
 * @brief       Burst mode access used for reading or writing to radio FIFOs.
 *
 *              For more efficient interrupt latency, this function does not keep interrupts
 *              disabled for its entire execution.  It is designed to recover if an interrupt
 *              occurs that accesses SPI.  See comments in code for further details.
 *
 * @param       addrByte - first byte written to SPI, contains address and mode bits
 * @param       pData    - pointer to data to read or write
 * @param       len      - length of data in bytes
 *
 * @return      true if an interrupt was detected during the transfer, false otherwise
 *=================================================================================================
 */

static bool spiBurstFifoAccess(uint8_t addrByte, uint8_t * pData, uint8_t len){
	uint8_t msts;
	MRFI_SPI_ASSERT( MRFI_SPI_IS_INITIALIZED() );   /* SPI is not initialized */
  MRFI_SPI_ASSERT(len != 0);                      /* zero length is not allowed */
  //MRFI_SPI_ASSERT(addrByte & BURST_BIT);          /* only burst mode supported */
	
  /* disable interrupts that use SPI */
  MRFI_SPI_ENTER_CRITICAL_SECTION(s);
	
	MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_TURN_CHIP_SELECT_ON();

	Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
	Mfs_Csio_EnableFunc(CsioCh0, CsioRx);

	while (TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty)); /* wait until TX buffer empty */
	Mfs_Csio_SendData(CsioCh0, addrByte | BURST_BIT , TRUE);  
	while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioRxFull)); /* wait until RX buffer full */
  msts = Mfs_Csio_ReceiveData(CsioCh0);	//Get status 
	
	if (addrByte & READ_BIT){	//Read
    stcRxFifoInfo.u32RxCnt = len;
    stcRxFifoInfo.pRxBuf = pData;
    stcRxFifoInfo.bRxFinish = FALSE;
		Mfs_Csio_EnableIrq(CsioCh0, CsioRxIrq);
    while(len--){
			while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty)); // wait TX idle
			Mfs_Csio_SendData(CsioCh0, 0x00u, FALSE);   /* Dummy write */
    }
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait TX idle
    while(stcRxFifoInfo.bRxFinish != TRUE);    /* Wait until Master finish reading FIFO */
  }
  else{	//Write
		stcTxFifoInfo.u32TxCnt = len;
    stcTxFifoInfo.pTxBuf = pData;
    stcTxFifoInfo.bTxFinish = FALSE;
		Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
		Mfs_Csio_DisableFunc(CsioCh0, CsioRx);
		Mfs_Csio_EnableIrq(CsioCh0, CsioTxFifoIrq);    
	  while(stcTxFifoInfo.bTxFinish != TRUE); // wait for Master TX finish
		while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait TX idle
    while(stcRxFifoInfo.bRxFinish != TRUE);    /* Wait until Master finish reading FIFO */
	}
	
	Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
	Mfs_Csio_DisableFunc(CsioCh0, CsioRx);

	/* turn off chip select; enable interrupts that call SPI functions */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();	
	
	MRFI_SPI_EXIT_CRITICAL_SECTION(s);	
	return false;
}	

static uint8_t spiRegAccess(uint8_t addrByte, uint8_t writeValue){
	stcRxFifoInfo.u32RxCnt = 2;
	stcRxFifoInfo.pRxBuf = au8CsioMasterRxBuf;
	stcRxFifoInfo.bRxFinish = FALSE;
	
	stcTxFifoInfo.u32TxCnt = 2;
	stcTxFifoInfo.pTxBuf = au8CsioMasterTxBuf;
	stcTxFifoInfo.bTxFinish = FALSE;

  au8CsioMasterTxBuf[0] = addrByte;
	au8CsioMasterTxBuf[1] = writeValue;	//issue clock in case of reading byte
	
	Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
	Mfs_Csio_EnableFunc(CsioCh0, CsioRx);
	Mfs_Csio_EnableIrq(CsioCh0, CsioRxIrq);
	Mfs_Csio_EnableIrq(CsioCh0, CsioTxFifoIrq);
	
	while(stcTxFifoInfo.bTxFinish != TRUE); 								// wait for Master TX finish
	while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait TX idle	
	while(stcRxFifoInfo.bRxFinish != TRUE);    							/* Wait until Master finish reading FIFO */

	Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
  Mfs_Csio_DisableFunc(CsioCh0, CsioRx);
	
	return au8CsioMasterRxBuf[1];
}
/**************************************************************************************************
*/
