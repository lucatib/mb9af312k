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
/** \file UsbHostHidCom.h
 **
 ** Part of FSEU USB Host Driver Module for use with Fujitsu HID Com Device
 **
 **
 ** A detailed description is available at 
 ** @link UsbHostHidComGroup USB Host HID Com Module description @endlink
 **
 ** History:
 **   - 2012-07-18    1.0  MSc  First version 
 **   - 2013-10-14    1.1  MSc  PDL support added
 **   - 2014-09-03    1.2  MSc  Deinit routine fixed
 **                             Switched to dynamic driver registering
 **   - 2015-05-04    1.3  MSCH deinitialization added after unsuccessful init
 **   - 2015-07-21    1.4  MSCH correct initialization for all variables added
 **   - 2015-09-04    1.5  MSCH Usb_WaitHook() added
 *****************************************************************************/

#ifndef __USBHOSTHIDCOM_H__
#define __USBHOSTHIDCOM_H__

#include "usb.h"

#ifndef USBHOSTHIDCOM_ENABLED
    #define USBHOSTHIDCOM_ENABLED OFF
#endif
     
#if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDCOM_ENABLED == ON))

#include "UsbHost.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 ******************************************************************************
 ** \defgroup UsbHostHidComGroup USB Host Middleware: HID Com - Data Communicaton
 **
 ** Provided functions of USB Host HID Communication module:
 ** 
 ** - UsbHostHidCom_RegisterDriver()
 ** - UsbHostHidCom_SetDataSentHandler()
 ** - UsbHostHidCom_SetDataReceivedHandler()
 ** - UsbHostHidCom_InitiateSending()
 ** - UsbHostHidCom_SendPolled()
 ** - UsbHostHidCom_ReceivePolled()
 ** - UsbHostHidCom_IsSent()
 ** - UsbHostHidCom_IsSending()
 ** - UsbHostHidCom_IsReceived()
 ** - UsbHostHidCom_DriverActive()
 **
 ** Following procedures are used for the UsbHostClassSupervisor:
 ** - UsbHostHidCom_InitHandler()
 ** - UsbHostHidCom_DeinitHandler()
 **
 ** Used to send or receive 64 byte block data
 **   
 ******************************************************************************/
//@{
    
/**
 ******************************************************************************    
 ** \page usbhosthidcom_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "usb.h"   
 ** #if (USBHOSTHIDCOM_ENABLED == ON)
 **     #include "UsbHostHidCom.h"
 ** #endif  
 ** @endcode
 **
 ******************************************************************************/
    
/**
 ****************************************************************************** 
 ** \page usbhosthidcom_module_init Example: Initialization
 ** \brief Following initialization is required to register the driver at the 
 **        @link UsbHostGroup USB Host Module@endlink
 ** @code
 ** UsbConfig_UsbInit();   
 ** #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDCOM_ENABLED == ON))
 **     UsbHostHidCom_RegisterDriver();
 ** #endif 
 ** @endcode
 **
 ******************************************************************************/
    
/**
 ******************************************************************************  
 ** \page usbhosthidcom_example_transfer_polled Example: Sending / receiving data polled 
 ** @code 
 ** #include "usb.h"   
 ** #if (USBHOSTHIDCOM_ENABLED == ON)
 **     #include "UsbHostHidCom.h"
 ** #endif  
 **
 ** uint8_t au8Data[64]; // data always is 64 bytes
 ** uint8_t pu8Data;
 **   
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDCOM_ENABLED == ON))
 **         UsbHostHidCom_RegisterDriver();
 **     #endif 
 **     for(;;)
 **     {
 **         UsbConfig_SwitchMode();
 **         pu8Data = UsbHostHidCom_ReceivePolled(void);
 **         if (pu8Data != NULL)
 **         {
 **             memcpy(au8Data,pu8Data,64);   
 **             UsbHostHidCom_SendPolled(au8Data);
 **         }
 **     }
 ** }
 ** @endcode
 **
 ******************************************************************************/
     
/**
 ****************************************************************************** 
 ** \page usbhosthidcom_example_transfer_irq Example: Sending / receiving data with IRQs 
 ** @code 
 ** #include "usb.h"   
 ** #if (USBHOSTHIDCOM_ENABLED == ON)
 **     #include "UsbHostHidCom.h"
 ** #endif  
 **
 ** uint8_t au8Data[64]; // data always is 64 bytes
 ** uint8_t pu8Data;
 **   
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDCOM_ENABLED == ON))
 **         UsbHostHidCom_RegisterDriver();
 **     #endif 
 **     for(;;)
 **     {
 **         UsbConfig_SwitchMode();
 **         if (UsbHostHidCom_IsReceived())
 **         {   
 **             pu8Data = UsbHostHidCom_ReceivePolled(void);
 **             if (pu8Data != NULL)
 **             {
 **                 memcpy(au8Data,pu8Data,64);   
 **                 UsbHostHidCom_InitiateSending(au8Data);
 **                 while(UsbHostHidCom_IsSending());
 **             }
 **     }
 ** }
 ** @endcode
 **
 ******************************************************************************/
     
/**
 ******************************************************************************    
 ** \page usbhosthidcom_example_transfer_callback Example: Using callbacks:  
 ** @code 
 ** #include "usb.h"   
 ** #if (USBHOSTHIDCOM_ENABLED == ON)
 **     #include "UsbHostHidCom.h"
 ** #endif  
 **   
 ** void CallbackRx(uint8_t* pu8Data)
 ** {
 **     //data was received, process data here
 ** }
 **
 ** void CallbackTx(void)
 ** {
 **     //data was sent
 ** }
 **
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDCOM_ENABLED == ON))
 **         UsbHostHidCom_RegisterDriver();
 **     #endif 
 **     UsbHostHidCom_SetDataReceivedHandler(CallbackRx);
 **     UsbHostHidCom_SetDataSentHandler(CallbackTx);
 **     for(;;)
 **     {
 **         UsbConfig_SwitchMode();
 **     }
 ** }
 ** @endcode
 **
 ******************************************************************************/


    
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/    

#define USBHOSTHIDCOM_VERSION  0120
#define USBHOSTHIDCOM_DATE     20140903
     
/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
    
typedef void (*pfn_usbhosthidcom_data_sent_t)(void); 
typedef void (*pfn_usbhosthidcom_data_received_t)(uint8_t* pu8Buffer); 

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void UsbHostHidCom_RegisterDriver(void);  
void UsbHostHidCom_SetDataSentHandler(pfn_usbhosthidcom_data_sent_t pfnDataSent);
void UsbHostHidCom_SetDataReceivedHandler(pfn_usbhosthidcom_data_received_t pfnDataReceived);
void UsbHostHidCom_InitiateSending(uint8_t* pu8Buffer);
void UsbHostHidCom_SendPolled(uint8_t* pu8Buffer);
uint8_t* UsbHostHidCom_ReceivePolled(void);
boolean_t UsbHostHidCom_IsSent(void);
boolean_t UsbHostHidCom_IsSending(void);
boolean_t UsbHostHidCom_IsReceived(void);
boolean_t UsbHostHidCom_DriverActive(void);

/* USED FOR USBCLASSSUPERVISOR */
boolean_t UsbHostHidCom_InitHandler(stc_usbn_t* pstcUsb, uint8_t* pu8Configuration, uint32_t u32Length);
boolean_t UsbHostHidCom_DeinitHandler(stc_usbn_t* pstcUsb);

#ifdef __cplusplus
}
#endif

//@} // UsbHostHidComGroup

#endif
#endif

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
