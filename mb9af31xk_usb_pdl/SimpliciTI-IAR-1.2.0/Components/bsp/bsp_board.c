/**************************************************************************************************
  Filename:       bsp_board.c
  Revised:        $Date: 2009-10-11 16:48:20 -0700 (Sun, 11 Oct 2009) $
  Revision:       $Revision: 20896 $

  Copyright 2007-2009 Texas Instruments Incorporated.  All rights reserved.

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

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Target : Texas Instruments MSP-EXP430FG4618
 *            "MSP430FG4618/F2013 Experimenter Board"
 *   Top-level board code file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

/* ------------------------------------------------------------------------------------------------
 *                                            Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "bsp.h"
#include "bsp_config.h"

#define BSP_WAIT_USEC   (40000000 / 1000000)

/* ------------------------------------------------------------------------------------------------
 *                                            Prototypes
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                            Defines
 * ------------------------------------------------------------------------------------------------
 */
#define BSP_TIMER_CLK_MHZ   (BSP_CONFIG_CLOCK_MHZ_SELECT)

//--------------------------------------------------------
//SPI
//--------------------------------------------------------
#define InitCsio0Io(void)  {SetPinFunc_SIN0_0();SetPinFunc_SOT0_0();SetPinFunc_SCK0_0();Gpio1pin_InitOut( GPIO1PIN_P00, Gpio1pin_InitVal( 1u ) );}

volatile stc_mfsn_csio_t* CsioCh0 = &CSIO0;
uint8_t au8CsioMasterTxBuf[SAMPLE_CSIO_MASTER_TX_BUFFSIZE];
uint8_t au8CsioMasterRxBuf[SAMPLE_CSIO_MASTER_RX_BUFFSIZE];
stc_tx_fifo_info_t stcTxFifoInfo = {0};
stc_rx_fifo_info_t stcRxFifoInfo = {0};

static stc_mfs_csio_config_t 	stcCsio0Config;
static stc_csio_irq_cb_t 			stcCsio0IrqCb;
static stc_mfs_fifo_config_t 	stcFifoConfig;

/**
 ******************************************************************************
 ** \brief  CSIO master FIFO transfer interrupt callback function
 ******************************************************************************/
static void CsioMasterFifoTxIntCallback(void)
{
    uint8_t u8i = 0;
    if(stcTxFifoInfo.u32TxCnt > SAMPLE_CSIO_FIFO_MAX_CAPACITY)
    {
        while(u8i < SAMPLE_CSIO_FIFO_MAX_CAPACITY)
        {
            Mfs_Csio_SendData(CsioCh0, *stcTxFifoInfo.pTxBuf++, TRUE);
            u8i++;
        }
        stcTxFifoInfo.u32TxCnt -= SAMPLE_CSIO_FIFO_MAX_CAPACITY;
        return;
    }
    
    while(u8i < stcTxFifoInfo.u32TxCnt)
    {
        Mfs_Csio_SendData(CsioCh0, *stcTxFifoInfo.pTxBuf++, TRUE);
        u8i++;
    }
  
    Mfs_Csio_DisableIrq(CsioCh0, CsioTxFifoIrq);
    
    stcTxFifoInfo.bTxFinish = TRUE;
}

/**
 ******************************************************************************
 ** \brief  CSIO Master receive interrupt callback function
 ******************************************************************************/
static void CsioMasterRxIntCallback(void)
{
    uint8_t u8i = 0;

    if(stcRxFifoInfo.u32RxCnt > SAMPLE_CSIO_FIFO_RX_CNT) 
    {
        /* Receive data when RX FIFO count match with SAMPLE_UART_FIFO_RX_CNT */
        while(u8i < SAMPLE_CSIO_FIFO_RX_CNT)
        {
            *stcRxFifoInfo.pRxBuf++ = Mfs_Csio_ReceiveData(CsioCh0); 
            u8i++;
        }
        stcRxFifoInfo.u32RxCnt -= SAMPLE_CSIO_FIFO_RX_CNT;
        return;
    }
  
    /* Receive data when RX FIFO is idle */
    /* idle means FIFO count is less than SAMPLE_UART_FIFO_RX_CNT and
       RX FIFO don't receive data from then on for a short time. */
    while(u8i < stcRxFifoInfo.u32RxCnt)
    {
        *stcRxFifoInfo.pRxBuf++ = Mfs_Csio_ReceiveData(CsioCh0);
        u8i++;
    }
    
    Mfs_Csio_DisableIrq(CsioCh0, CsioRxIrq);
    
    stcRxFifoInfo.bRxFinish = TRUE;
}

void CsioMasterInit(void){
    /* Clear configuration structure */
    PDL_ZERO_STRUCT(stcCsio0Config);
    PDL_ZERO_STRUCT(stcCsio0IrqCb);
    
    /* Initialize CSIO function I/O */    
    InitCsio0Io();
    
    /* Initialize CSIO interrupt callback functions */
    stcCsio0IrqCb.pfnTxFifoIrqCb = CsioMasterFifoTxIntCallback;
    stcCsio0IrqCb.pfnRxIrqCb = CsioMasterRxIntCallback;
    
    /* Initialize FIFO configuration */
    stcFifoConfig.enFifoSel = MfsTxFifo1RxFifo2;
    stcFifoConfig.u8ByteCount1 = 0u;
    stcFifoConfig.u8ByteCount2 = SAMPLE_CSIO_FIFO_RX_CNT;
    
    /* Initialize CSIO master  */
    stcCsio0Config.enMsMode = CsioMaster;
    stcCsio0Config.enActMode = CsioActNormalMode;
    stcCsio0Config.bInvertClk = FALSE;
    stcCsio0Config.u32BaudRate = 100000;
    stcCsio0Config.enDataLength = CsioEightBits;
    stcCsio0Config.enBitDirection = CsioDataMsbFirst;
    stcCsio0Config.pstcFifoConfig = &stcFifoConfig;
    stcCsio0Config.pstcIrqCb = &stcCsio0IrqCb;
    stcCsio0Config.pstcIrqEn = NULL;
    stcCsio0Config.bTouchNvic = TRUE;
    
    Mfs_Csio_Init(CsioCh0, &stcCsio0Config);		
}

void Spi_CheckTxRx(void){
	  uint8_t u8i;
    
    u8i = 0;
	
		Gpio1pin_Put( GPIO1PIN_P00, 0u);
	
	  /*************************************************************************/
    /*                Master sends data to slave                             */
    /*************************************************************************/
		stcTxFifoInfo.u32TxCnt = SAMPLE_CSIO_MASTER_TX_BUFFSIZE;
    stcTxFifoInfo.pTxBuf = au8CsioMasterTxBuf;
    stcTxFifoInfo.bTxFinish = FALSE;
		Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
		Mfs_Csio_EnableIrq(CsioCh0, CsioTxFifoIrq);    
	  while(stcTxFifoInfo.bTxFinish != TRUE)
				; // wait for Master TX finish
		while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle))
				; // wait TX idle
    Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
		
		/*************************************************************************/
    /*                Master receives data from slave                        */
    /*************************************************************************/
		/* Initialize the FIFO information */
    stcRxFifoInfo.u32RxCnt = SAMPLE_CSIO_MASTER_RX_BUFFSIZE;
    stcRxFifoInfo.pRxBuf = au8CsioMasterRxBuf;
    stcRxFifoInfo.bRxFinish = FALSE;
    Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
    Mfs_Csio_EnableFunc(CsioCh0, CsioRx);
    Mfs_Csio_EnableIrq(CsioCh0, CsioRxIrq);
    /* Read the data by writing data synchronously */
    u8i = 0;
    while(u8i < 8){
        while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty)); // wait TX idle
        Mfs_Csio_SendData(CsioCh0, 0x00u, FALSE);   /* Dummy write */
        u8i++;
    }
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle))
			; // wait TX idle
    while(stcRxFifoInfo.bRxFinish != TRUE)
			;    /* Wait until Master finish reading FIFO */
    Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
    Mfs_Csio_DisableFunc(CsioCh0, CsioRx);
		
		Gpio1pin_Put( GPIO1PIN_P00, 1u);
}
//!SPI

/* ------------------------------------------------------------------------------------------------
 *                                            Local Variables
 * ------------------------------------------------------------------------------------------------
 */
#if defined(SW_TIMER)
static uint8_t sIterationsPerUsec = 0;
#endif

/**************************************************************************************************
 * @fn          BSP_EARLY_INIT
 *
 * @brief       This function is called by start-up code before doing the normal initialization
 *              of data segments. If the return value is zero, initialization is not performed.
 *              The global macro label "BSP_EARLY_INIT" gets #defined in the bsp_msp430_defs.h
 *              file, according to the specific compiler environment (CCE or IAR). In the CCE
 *              environment this macro invokes "_system_pre_init()" and in the IAR environment
 *              this macro invokes "__low_level_init()".
 *
 * @param       None
 *
 * @return      0 - don't intialize data segments / 1 - do initialization
 **************************************************************************************************
*/
BSP_EARLY_INIT(void)
{
	/* Disable watchdog timer */
	//SCU_APBPeriphReset(__WDG, DISABLE);

	/* Return 1 - run seg_init */
	return (1);
}

/**************************************************************************************************
 * @fn          BSP_InitBoard
 *
 * @brief       Initialize the board.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void BSP_InitBoard(void){
	//SCU_APBPeriphReset(__WDG, ENABLE);
}

/**************************************************************************************************
 * @fn          BSP_Delay
 *
 * @brief       Sleep for the requested amount of time.
 *
 * @param       # of microseconds to sleep.
 *
 * @return      none
 **************************************************************************************************
 */
void BSP_Delay(uint16_t usec){
	int i;
	for (i=0;i<BSP_WAIT_USEC;i++)	__asm("nop");	
}


