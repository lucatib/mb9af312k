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


#define InitCsio0Io(void)  {SetPinFunc_SIN0_0();SetPinFunc_SOT0_0();SetPinFunc_SCK0_0();Gpio1pin_InitOut( GPIO1PIN_P00, Gpio1pin_InitVal( 1u ) );}
volatile stc_mfsn_csio_t* CsioCh0 = &CSIO0;
uint32_t SendCnt = 0, ReceiveCnt = 0;
static uint8_t au8CsioMasterTxBuf[64] = "01234567";
static uint8_t au8CsioMasterRxBuf[64];

/*      START GLOBAL VARIABLES FOR VIRTUAL COM PORT EXAMPLE       */

 char_t pu8DeviceCdcReceiveBuffer[512];
 uint32_t u32DeviceCdcReceiveSize;
 boolean_t bDeviceCdcComConnected;
/*      END GLOBAL VARIABLES FOR VIRTUAL COM PORT EXAMPLE        */

/**
 ******************************************************************************
 ** \brief  CSIO master transfer interrupt callback function
 ******************************************************************************/
static void CsioMasterTxIntCallback(void)
{
    if(SendCnt == 8)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh0, CsioTxIrq);
        return;
    }
  
    Mfs_Csio_SendData(CsioCh0, au8CsioMasterTxBuf[SendCnt], TRUE);  
    SendCnt++;

}
/**
 ******************************************************************************
 ** \brief  CSIO Master receive interrupt callback function
 ******************************************************************************/
static void CsioMasterRxIntCallback(void)
{
    au8CsioMasterRxBuf[ReceiveCnt] = Mfs_Csio_ReceiveData(CsioCh0);
    ReceiveCnt++;
    
//    if(ReceiveCnt == 8)
//    {
//        /* Disable interrupt */
//        Mfs_Csio_DisableIrq(CsioCh1, CsioRxIrq);
//        return;
//    }
}

void Spi_CheckTxRx(void){
		/* Enable TX and RX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
    Mfs_Csio_EnableFunc(CsioCh0, CsioRx);
		SendCnt = 0;
    ReceiveCnt = 0;

		/* Master write */
		Mfs_Csio_EnableIrq(CsioCh0, CsioRxIrq);
		Gpio1pin_Put( GPIO1PIN_P00, 0u);
    while(SendCnt < 8){
        while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty));
        Mfs_Csio_SendData(CsioCh0, 0x00u, FALSE);   /* Dummy write */    
    }
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait until TX bus idle
    Gpio1pin_Put( GPIO1PIN_P00, 1u);
		
    /* Wait receive finish */
    while(ReceiveCnt < 8){
        ;
    }
		
    /* Disable TX and RX function of CSIO1   */
    Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
    Mfs_Csio_DisableFunc(CsioCh0, CsioRx);
}
/**
 ******************************************************************************
 ** \brief  Main function
 **
 ** \return int return value, if needed
 ******************************************************************************/
int main(void)
{
		stc_mfs_csio_config_t stcCsio1Config;
		stc_csio_irq_cb_t stcCsio1IrqCb;
	
	    /* Clear configuration structure */
    PDL_ZERO_STRUCT(stcCsio1Config);
    PDL_ZERO_STRUCT(stcCsio1IrqCb);

      /* Initialize interrupt callback functions */
    stcCsio1IrqCb.pfnTxIrqCb = CsioMasterTxIntCallback;
		stcCsio1IrqCb.pfnRxIrqCb = CsioMasterRxIntCallback;
	  
		/* Initialize CSIO function I/O */    
    InitCsio0Io();
		
		/* Initialize CSIO master  */
    stcCsio1Config.enMsMode = CsioMaster;
    stcCsio1Config.enActMode = CsioActNormalMode;
    stcCsio1Config.bInvertClk = FALSE;
    stcCsio1Config.u32BaudRate = 100000;
    stcCsio1Config.enDataLength = CsioEightBits;
    stcCsio1Config.enBitDirection = CsioDataMsbFirst;
    stcCsio1Config.enSyncWaitTime = CsioSyncWaitZero;
    stcCsio1Config.pstcFifoConfig = NULL;
    //stcCsio1Config.pstcCsConfig = &stcCsio1CsConfig;
    //stcCsio1Config.pstcSerialTimer = NULL;
    stcCsio1Config.pstcIrqEn = NULL;
    stcCsio1Config.pstcIrqCb = &stcCsio1IrqCb;
    stcCsio1Config.bTouchNvic = TRUE;
    
    Mfs_Csio_Init(CsioCh0, &stcCsio1Config);
	
    UsbConfig_UsbInit();

		UsbDeviceCdcCom_SetSeparator('\r');    // there is the possibility to set end of buffer by a seperator
		UsbDeviceCdcCom_SetEchomode(TRUE); // all input shall be echoed

    for(;;)
    {

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
						 
							 Spi_CheckTxRx();
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
