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
/** \file UsbHostPrinter.h
 **
 ** Part of FSEU USB Host Driver Module
 **
 **
 ** A detailed description is available at 
 ** @link UsbHostPrinterGroup USB Host Printer Module description @endlink
 **
 ** History:
 **   - 2014-09-03    1.0  MSc  First version
 **   - 2015-05-04    1.1  MSCH deinitialization added after unsuccessful init
 **   - 2015-07-21    1.2  MSCH correct initialization for all variables added
 **   - 2015-09-04    1.3  MSCH Usb_WaitHook() added
 *****************************************************************************/

#ifndef __USBHOSTPRINTER_H__
#define __USBHOSTPRINTER_H__

#include "usb.h"

#ifndef USBHOSTPRINTER_ENABLED
    #define USBHOSTPRINTER_ENABLED ON
#endif
     
#if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTPRINTER_ENABLED == ON))

#include "UsbHost.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 ******************************************************************************
 ** \defgroup UsbHostPrinterGroup USB Host Middleware: Printer
 **
 ** Provided functions of USB Host Printer module:
 ** 
 ** - UsbHostPrinter_RegisterDriver()
 ** - UsbHostPrinter_DataReceivedHandler()
 ** - UsbHostPrinter_DataSentHandler()
 ** - UsbHostPrinter_SetDataSentHandler()
 ** - UsbHostPrinter_SetDataReceivedHandler()
 ** - UsbHostPrinter_InitiateSending()
 ** - UsbHostPrinter_InitiateReceiving()
 ** - UsbHostPrinter_SendPolled()
 ** - UsbHostPrinter_ReceivePolled()
 ** - UsbHostPrinter_IsSent()
 ** - UsbHostPrinter_IsSending()
 ** - UsbHostPrinter_IsReceived()
 ** - UsbHostPrinter_DriverActive()
 **
 ** Following procedures are used for the UsbHostClassSupervisor:
 ** - UsbHostPrinter_InitHandler()
 ** - UsbHostPrinter_DeinitHandler()
 **
 **
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page usbhostprinter_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "usb.h"   
 ** #if (USBHOSTPRINTER_ENABLED == ON)
 **     #include "UsbHostPrinter.h"
 ** #endif  
 ** @endcode
 **
 ******************************************************************************/
    
/**
 ****************************************************************************** 
 ** \page usbhostprinter_module_init Example: Initialization
 ** \brief Following initialization is required to register the driver at the 
 **        @link UsbHostGroup USB Host Module@endlink
 ** @code
 ** UsbConfig_UsbInit();   
 ** #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTPRINTER_ENABLED == ON))
 **     UsbHostPrinter_RegisterDriver();
 ** #endif 
 ** @endcode
 **
 ******************************************************************************/
    
/**
 ******************************************************************************  
 ** \page usbhostprinter_example_transfer_polled Example: Sending data polled 
 ** @code 
 ** #include "usb.h"   
 ** #if (USBHOSTPRINTER_ENABLED == ON)
 **     #include "UsbHostPrinter.h"
 ** #endif  
 **
 ** const char_t pcHelloWorld[] = "Hello World!\r\n";   
 **   
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTPRINTER_ENABLED == ON))
 **         UsbHostPrinter_RegisterDriver();
 **     #endif 
 **     for(;;)
 **     {
 **         UsbConfig_SwitchMode(); 
 **         if (UsbHostPrinter_DriverActive()
 **         {
 **             UsbHostPrinter_SendPolled(pcHelloWorld,strlen(pcHelloWorld));
 **         }
 **     }
 ** }
 ** @endcode
 **
 ******************************************************************************/
     
/**
 ****************************************************************************** 
 ** \page usbhostprinter_example_transfer_irq Example: Sending data with IRQs 
 ** @code 
 ** #include "usb.h"   
 ** #if (USBHOSTPRINTER_ENABLED == ON)
 **     #include "UsbHostPrinter.h"
 ** #endif  
 **
 ** const char_t pcHelloWorld[] = "Hello World!\r\n";  
 **   
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTPRINTER_ENABLED == ON))
 **         UsbHostPrinter_RegisterDriver();
 **     #endif 
 **     for(;;)
 **     {
 **         UsbConfig_SwitchMode();
 **         if (UsbHostPrinter_DriverActive()
 **         {
 **             UsbHostPrinter_InitiateSending(pcHelloWorld,strlen(pcHelloWorld));
 **             while(UsbHostPrinter_IsSending());
 **         }
 **     }
 ** }
 ** @endcode
 **
 ******************************************************************************/
  
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/    

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

#define USBHOSTMOUSE_VERSION  0100
#define USBHOSTMOUSE_DATE     20140903
    
typedef void (*pfn_usbhostprinter_data_sent_t)(void); 
typedef void (*pfn_usbhostprinter_data_received_t)(uint8_t* pu8Buffer); 

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
    
void UsbHostPrinter_RegisterDriver(void);    
void UsbHostPrinter_DataReceivedHandler(stc_usbn_t* pstcUsb);
void UsbHostPrinter_DataSentHandler(stc_usbn_t* pstcUsb);
void UsbHostPrinter_SetDataSentHandler(pfn_usbhostprinter_data_sent_t pfnCallback);
void UsbHostPrinter_SetDataReceivedHandler(pfn_usbhostprinter_data_received_t pfnCallback);
void UsbHostPrinter_InitiateSending(uint8_t* pu8Buffer, uint32_t u32DataSize);
void UsbHostPrinter_InitiateReceiving(uint8_t* pu8Buffer, uint32_t u32DataSize);
void UsbHostPrinter_SendPolled(uint8_t* pu8Buffer, uint32_t u32DataSize);
uint8_t* UsbHostPrinter_ReceivePolled(uint8_t* pu8Buffer, uint32_t u32DataSize);
boolean_t UsbHostPrinter_IsSent(void);
boolean_t UsbHostPrinter_IsSending(void);
boolean_t UsbHostPrinter_IsReceived(void);
boolean_t UsbHostPrinter_DriverActive(void);

/* USED FOR USBCLASSSUPERVISOR */
boolean_t UsbHostPrinter_InitHandler(stc_usbn_t* pstcUsb, uint8_t* pu8Configuration, uint32_t u32Length);
boolean_t UsbHostPrinter_DeinitHandler(stc_usbn_t* pstcUsb);

#ifdef __cplusplus
}
#endif

//@} // UsbHostPrinterGroup

#endif
#endif
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
