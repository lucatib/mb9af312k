/*******************************************************************************
* Copyright (C) 2013-2016, Cypress Semiconductor Corporation or a              *
* subsidiary of Cypress Semiconductor Corporation.  All rights reserved.       *
*                                                                              *
* This software, including source code, documentation and related              *
* materials ("Software"), is owned by Cypress Semiconductor Corporation or     *
* one of its subsidiaries ("Cypress") and is protected by and subject to       *
* worldwide patent protection (United States and foreign), United States       *
* copyright laws and international treaty provisions. Therefore, you may use   *
* this Software only as provided in the license agreement accompanying the     *
* software package from which you obtained this Software ("EULA").             *
*                                                                              *
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,     *
* non-transferable license to copy, modify, and compile the                    *
* Software source code solely for use in connection with Cypress's             *
* integrated circuit products.  Any reproduction, modification, translation,   *
* compilation, or representation of this Software except as specified          *
* above is prohibited without the express written permission of Cypress.       *
*                                                                              *
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                         *
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                         *
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                 *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                              *
* PARTICULAR PURPOSE. Cypress reserves the right to make                       *
* changes to the Software without notice. Cypress does not assume any          *
* liability arising out of the application or use of the Software or any       *
* product or circuit described in the Software. Cypress does not               *
* authorize its products for use in any products where a malfunction or        *
* failure of the Cypress product may reasonably be expected to result in       *
* significant property damage, injury or death ("High Risk Product"). By       *
* including Cypress's product in a High Risk Product, the manufacturer         *
* of such system or application assumes all risk of such use and in doing      *
* so agrees to indemnify Cypress against all liability.                        *
*******************************************************************************/

/******************************************************************************/
/** \file main.c
 **
 ** \brief Empty main function.
 **
 ** Main Module
 **
 ** History:
 ** 2015-07-09  V1.0.0  MSCH first version
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/* Includes for USB */
#include "usb.h"
#if FM_PERIPHERAL_USB_DEVICE_ENABLED == ON
    #include "usbdevice.h"
#endif
#if FM_PERIPHERAL_USB_HOST_ENABLED == ON
    #include "usbhost.h"
    #include "usbhostclasssupervisor.h"
#endif 
#if (USBDEVICECDCCOM_ENABLED == ON)
    #include "UsbDeviceCdcCom.h"
#endif
#if (USBDEVICEHIDCOM_ENABLED == ON)
    #include "UsbDeviceHidCom.h"
#endif
#if (USBDEVICEHIDJOYSTICK_ENABLED == ON)
    #include "UsbDeviceHidJoystick.h"
#endif
#if (USBDEVICEHIDKEYBOARD_ENABLED == ON)
    #include "UsbDeviceHidKeyboard.h"         
#endif 
#if (USBDEVICEHIDMOUSE_ENABLED == ON)
    #include "UsbDeviceHidMouse.h"
#endif    
#if (USBDEVICELIBUSB_ENABLED == ON)
    #include "UsbDeviceLibUsb.h"
#endif
#if (USBDEVICEPRINTER_ENABLED == ON)
    #include "UsbDevicePrinter.h"
#endif
#if (USBDEVICEMASSSTORAGE_ENABLED == ON)
    #include "UsbDeviceMassStorage.h"
#endif 
/* End of includes for USB */

#define SAMPLE_CSIO_MASTER_RX_BUFFSIZE  8
#define SAMPLE_CSIO_MASTER_TX_BUFFSIZE  8
	
#if (PDL_MCU_CORE == PDL_FM3_CORE)
	#define SAMPLE_CSIO_FIFO_MAX_CAPACITY         (16u)
#endif

#define SAMPLE_CSIO_FIFO_RX_CNT               (8u)

#define InitCsio0Io(void)  {SetPinFunc_SIN0_0();SetPinFunc_SOT0_0();SetPinFunc_SCK0_0();Gpio1pin_InitOut( GPIO1PIN_P00, Gpio1pin_InitVal( 1u ) );}
/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/
/// CSIO TX FIFO information structure
typedef struct stc_tx_fifo_info
{
    uint32_t u32TxCnt;
    uint8_t* pTxBuf;
    boolean_t bTxFinish;
    
}stc_tx_fifo_info_t;

/// UART RX FIFO information structure
typedef struct stc_rx_fifo_info
{
    uint32_t u32RxCnt;
    uint8_t* pRxBuf;
    boolean_t bRxFinish;
    
}stc_rx_fifo_info_t;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static volatile stc_mfsn_csio_t* CsioCh0 = &CSIO0;
static uint8_t au8CsioMasterTxBuf[] = "01234567";
static uint8_t au8CsioMasterRxBuf[SAMPLE_CSIO_MASTER_RX_BUFFSIZE];
static stc_tx_fifo_info_t stcTxFifoInfo = {0};
static stc_rx_fifo_info_t stcRxFifoInfo = {0};
stc_mfs_csio_config_t 	stcCsio0Config;
stc_csio_irq_cb_t 			stcCsio0IrqCb;
stc_mfs_fifo_config_t 	stcFifoConfig;

/*      START GLOBAL VARIABLES FOR VIRTUAL COM PORT EXAMPLE       */

 char_t pu8DeviceCdcReceiveBuffer[512];
 uint32_t u32DeviceCdcReceiveSize;
 boolean_t bDeviceCdcComConnected;
/*      END GLOBAL VARIABLES FOR VIRTUAL COM PORT EXAMPLE        */

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
    if(stcRxFifoInfo.u32RxCnt > SAMPLE_CSIO_MASTER_RX_BUFFSIZE) 
    {
        /* Receive data when RX FIFO count match with SAMPLE_UART_FIFO_RX_CNT */
        while(u8i < SAMPLE_CSIO_MASTER_RX_BUFFSIZE)
        {
            *stcRxFifoInfo.pRxBuf++ = Mfs_Csio_ReceiveData(CsioCh0); 
            u8i++;
        }
        stcRxFifoInfo.u32RxCnt -= SAMPLE_CSIO_MASTER_RX_BUFFSIZE;
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

/**
 ******************************************************************************
 ** \brief  Main function
 **
 ** \return int return value, if needed
 ******************************************************************************/
int main(void)
{
    UsbConfig_UsbInit();

		UsbDeviceCdcCom_SetSeparator('\r');    // there is the possibility to set end of buffer by a seperator
		UsbDeviceCdcCom_SetEchomode(TRUE); // all input shall be echoed

		CsioMasterInit();

    for(;;)
    {
				Spi_CheckTxRx();
			
        #if (((USB_USE_PDL == 1) || (USB_USE_L3 == 1) || (USB_USE_EXT_INT == 0)) && (!defined(BOARD_USB) || (BOARD_USB == OFF)))
        UsbConfig_SwitchMode();  // switch USB<n> if required, otherwise, initialize USB host/device mode
        #endif

       /**************************************************************************************************/
       /*                                                                                                */
       /* START VIRTUAL COM PORT EXAMPLE USAGE                                                           */
       /*                                                                                                */
       /**************************************************************************************************/
       #if (FM_PERIPHERAL_USB_DEVICE_ENABLED == ON)
       /* waiting for a connection */
       if (bDeviceCdcComConnected != UsbDeviceCdcCom_IsConnected())
       {
           bDeviceCdcComConnected = UsbDeviceCdcCom_IsConnected();
           if (bDeviceCdcComConnected == TRUE)
           {
               /* sending welcome message after connection*/
               UsbDeviceCdcCom_SendString("\r\n");
               UsbDeviceCdcCom_SendString("Welcome to Spansion Virtual Comm Port Example!\r\n");
               UsbDeviceCdcCom_SendString("waiting for your message:\r\n");
           }
           else
           {
           }
       }
	   
       if (UsbDeviceCdcCom_IsConnected()) 
       {
           if (UsbDeviceCdcCom_ReceivedLength() > 0) {
               /* receive data from buffer */
               u32DeviceCdcReceiveSize = UsbDeviceCdcCom_ReceiveBuffer((uint8_t *)pu8DeviceCdcReceiveBuffer); //this clears also the receive buffer
               pu8DeviceCdcReceiveBuffer[u32DeviceCdcReceiveSize] = '\0';    //adding zero termination to string
         
               /* print16_t out pu8DeviceCdcReceiveBuffer through Virtual Comm Port */
               UsbDeviceCdcCom_SendByte('\n');
               UsbDeviceCdcCom_SendString("Received String: ");
               UsbDeviceCdcCom_SendString(pu8DeviceCdcReceiveBuffer);
               UsbDeviceCdcCom_SendString("\r\n");		 
           }  
       }  
       #endif
       /**************************************************************************************************/
       /*                                                                                                */
       /* END VIRTUAL COM PORT EXAMPLE USAGE                                                             */
       /*                                                                                                */
       /**************************************************************************************************/

			 
			 
    } /* End Loop Forever */

}

/*****************************************************************************/
/* EOF (not truncated)                                                       */
/*****************************************************************************/
