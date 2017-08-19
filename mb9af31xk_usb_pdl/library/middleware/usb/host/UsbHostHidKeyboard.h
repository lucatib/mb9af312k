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
/** \file UsbHostHidKeyboard.h
 **
 ** Part of Spansion USB Host Driver Module
 **
 ** A detailed description is available at 
 ** @link UsbHostKeyboardGroup USB Host HID Keyboard Module description @endlink
 **
 ** History:
 **   - 2010-12-13    1.0  MSc  First version  (works with 16FX)
 **   - 2010-01-05    1.1  MSc  API Updates
 **   - 2011-03-30    1.2  MSc  Public Release
 **   - 2011-08-24    1.3  MSc  Bug fixes
 **   - 2012-06-05    1.4  MSc  New verison for use with new USB driver for FM3 L3
 **                             Rename HidKeyboard.h -> UsbHostHidKeyboard.h
 **   - 2013-10-14    1.5  MSc  PDL support added
 **   - 2014-09-03    1.6  MSc  Deinit routine fixed
 **                             Switched to dynamic driver registering
 **   - 2015-05-04    1.7  MSCH deinitialization added after unsuccessful init
 **   - 2015-07-21    1.8  MSCH correct initialization for all variables added
 **   - 2015-09-04    1.9  MSCH Usb_WaitHook() added
 *****************************************************************************/

#ifndef __USBHOSTHIDKEYBOARD_H__
#define __USBHOSTHIDKEYBOARD_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "usb.h"

#ifndef USBHOSTHIDKEYBOARD_ENABLED
    #define USBHOSTHIDKEYBOARD_ENABLED OFF
#endif
     
#if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDKEYBOARD_ENABLED == ON))

#include "UsbHost.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 ******************************************************************************
 ** \defgroup UsbHostKeyboardGroup USB Host Middleware: HID Keyboard
 **
 ** Provided functions of USB Host HID Keyboard module:
 ** 
 ** - UsbHostHidKeyboard_RegisterDriver()
 ** - UsbHostHidKeyboard_DriverActive()
 ** - UsbHostHidKeyboard_SetCallback()
 ** - UsbHostHidKeyboard_RemoveCallback()
 ** - UsbHostHidKeyboard_GetCh()
 ** - HidKeyboard_SetLeds()
 **
 ** Following procedures are used for the UsbHostClassSupervisor:
 ** - UsbHostHidKeyboard_InitHandler()
 ** - UsbHostHidKeyboard_DeinitHandler()
 **
 ** Used to connect USB keyboards to the MCU
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page usbhostkeyboard_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "usb.h"   
 ** #if (USBHOSTHIDKEYBOARD_ENABLED == ON)
 **     #include "UsbHostHidKeyboard.h"
 ** #endif  
 ** @endcode
 **
 ******************************************************************************/
     
/**
 ****************************************************************************** 
 ** \page usbhostkeyboard_module_init Example: Initialization
 ** \brief Following initialization is required to register the driver at the 
 **        @link UsbHostGroup USB Host Module@endlink
 ** @code
 ** UsbConfig_UsbInit();   
 ** #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDKEYBOARD_ENABLED == ON))
 **     UsbHostHidKeyboard_RegisterDriver();
 ** #endif 
 ** @endcode
 **
 ******************************************************************************/
    
/**
 ******************************************************************************    
    ** \page usbhostkeyboard_example_receive Example: Receiving data  
 ** @code 
 ** #include "usb.h"   
 ** #if (USBHOSTHIDKEYBOARD_ENABLED == ON)
 **     #include "UsbHostHidKeyboard.h"
 ** #endif  
 **
 ** char_t c;
 **   
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDKEYBOARD_ENABLED == ON))
 **         UsbHostHidKeyboard_RegisterDriver();
 **     #endif 
 **     for(;;)
 **     {
 **         UsbConfig_SwitchMode();
 **         c = UsbHostHidKeyboard_GetCh(1000); //Timeout in ticks
 **         // if c == 0, no data was received, if c != 0, c = received char
 **     }
 ** }
 ** @endcode
 **
 ******************************************************************************/
     
/**
 ******************************************************************************    
 ** \page usbhostkeyboard_example_callbacks Example: Using callbacks
 ** @code 
 ** #include "usb.h"   
 ** #if (USBHOSTHIDKEYBOARD_ENABLED == ON)
 **     #include "UsbHostHidKeyboard.h"
 ** #endif  
 **   
 ** void KeyboardCallback(stc_usbhostkeyboard_data_t *pstcData)
 ** {
 **    //data received stored in pstcData
 ** }
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDKEYBOARD_ENABLED == ON))
 **         UsbHostHidKeyboard_RegisterDriver();
 **     #endif 
 **     UsbHostHidKeyboard_SetCallback(KeyboardCallback);
 **     for(;;)
 **     {
 **         UsbConfig_SwitchMode();
 **     }
 ** }
 ** @endcode
 ******************************************************************************/

    
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
     
#define USBHOSTHIDKEYBOARD_VERSION  0160
#define USBHOSTHIDKEYBOARD_DATE     20140903
     
#define KEYCODE_ASCII(keycode)  u8Keycodes[keycode]

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

typedef struct stc_usbhostkeyboard_data KeyboardData_t;

typedef struct stc_usbhostkeyboard_data
{
    uint8_t u8ModifierKeys;
    uint8_t u8Reserved;
    uint8_t u8KeyCode1;
    uint8_t u8KeyCode2;
    uint8_t u8KeyCode3;
    uint8_t u8KeyCode4;
    uint8_t u8KeyCode5;
    uint8_t u8KeyCode6;
} stc_usbhostkeyboard_data_t;

typedef void (*pfn_usbhostkeyboard_datareceived_t)(stc_usbhostkeyboard_data_t *pstcData);

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

#ifndef __HIDKEYBOARD_C__
    extern const uint8_t u8Keycodes[];
#endif

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
void UsbHostHidKeyboard_RegisterDriver(void);
boolean_t UsbHostHidKeyboard_InitHandler(stc_usbn_t* pstcUsb, uint8_t* pu8Configuration, uint32_t u32Length);
boolean_t UsbHostHidKeyboard_DeinitHandler(stc_usbn_t* pstcUsb);
boolean_t UsbHostHidKeyboard_DriverActive(void);
void UsbHostHidKeyboard_SetCallback(pfn_usbhostkeyboard_datareceived_t pfnCallback);
void UsbHostHidKeyboard_RemoveCallback(void);
char_t UsbHostHidKeyboard_GetCh(volatile uint32_t u32TimeOut);
void HidKeyboard_SetLeds(uint8_t u8Data);

#ifdef __cplusplus
}
#endif

//@} // UsbHostKeyboardGroup

#endif /* (USE_USB_HOST == 1) */
#endif /* __HidKeyboard_H__*/

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
