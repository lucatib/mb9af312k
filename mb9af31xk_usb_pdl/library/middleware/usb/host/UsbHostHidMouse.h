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
/** \file UsbHostHidMouse.h
 **
 ** Part of Spansion USB Host Driver Module
 **
 ** A detailed description is available at 
 ** @link UsbHostMouseGroup USB Host HID Mouse Module description @endlink
 **
 ** History:
 **   - 2011-03-30    1.0  MSc  First version 
 **   - 2011-08-24    1.1  MSc  Some fixes in X/Y calculation
 **   - 2012-06-05    1.2  MSc  New verison for use with new USB driver for FM3 L3
 **                             Rename HidMouse.h -> UsbHostHidMouse.h
 **   - 2013-10-14    1.3  MSc  PDL support added
 **   - 2014-07-30    1.5  MSc  Report descriptor parsing added
 **   - 2014-09-03    1.6  MSc  Deinit routine fixed
 **                             Switched to dynamic driver registering
 **   - 2015-05-04    1.7  MSCH deinitialization added after unsuccessful init
 **   - 2015-07-21    1.8  MSCH correct initialization for all variables added
 *****************************************************************************/


#ifndef __USBHOSTHIDMOUSE_H__
#define __USBHOSTHIDMOUSE_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "usb.h"

#ifndef USBHOSTHIDMOUSE_ENABLED
    #define USBHOSTHIDMOUSE_ENABLED OFF
#endif
     
#if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDMOUSE_ENABLED == ON))

#include "UsbHost.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 ******************************************************************************
 ** \defgroup UsbHostMouseGroup USB Host Middleware: HID Mouse
 **
 ** Provided functions of USB Host HID Mouse module:
 ** 
 ** - UsbHostHidMouse_SetCallback()
 ** - UsbHostHidMouse_RemoveCallback()
 ** - UsbHostHidMouse_GetCurrentPosition()
 ** - UsbHostHidMouse_SetCurrentPosition()
 ** - UsbHostHidMouse_GetCurrentScrollPosition()
 ** - UsbHostHidMouse_SetCurrentScrollPosition()
 ** - UsbHostHidMouse_GetButtons()
 ** - UsbHostHidMouse_Moved()
 ** - UsbHostHidMouse_DriverActive()
 **
 ** Following procedures are used for the UsbHostClassSupervisor:
 ** - UsbHostHidMouse_InitHandler()
 ** - UsbHostHidMouse_DeinitHandler()
 ** - UsbHostHidMouse_Configured()
 ** - UsbHostHidMouse_IsActive()   
 **
 ** Used to connect USB mouse to the MCU
 **
 ******************************************************************************/
//@{

/**
 ******************************************************************************    
 ** \page usbhostmouse_module_includes Required includes in main application
 ** \brief Following includes are required
 ** @code   
 ** #include "usb.h"   
 ** #if (USBHOSTHIDMOUSE_ENABLED == ON)
 **     #include "UsbHostHidMouse.h"
 ** #endif  
 ** @endcode
 **
 ******************************************************************************/
     
/**
 ****************************************************************************** 
 ** \page usbhostmouse_module_init Example: Initialization
 ** \brief Following initialization is required to register the driver at the 
 **        @link UsbHostGroup USB Host Module@endlink
 ** @code
 ** UsbConfig_UsbInit();   
 ** #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDMOUSE_ENABLED == ON))
 **     UsbHostHidMouse_RegisterDriver();
 ** #endif 
 ** @endcode
 **
 ******************************************************************************/
    
/**
 ******************************************************************************    
 ** \page usbhostmouse_example_receive Example: Receiving data  
 ** @code 
 ** #include "usb.h"   
 ** #if (USBHOSTHIDMOUSE_ENABLED == ON)
 **     #include "UsbHostHidMouse.h"
 ** #endif  
 **   
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDMOUSE_ENABLED == ON))
 **         UsbHostHidMouse_RegisterDriver();
 **     #endif 
 **     for(;;)
 **     {
 **         UsbConfig_SwitchMode();
 **         if (UsbHostHidMouse_Moved())
 **         {
 **             printf("\r\nMouse Position %d,%d\r\n",
 **                     UsbHostHidMouse_GetCurrentPosition().u32X,    
 **                     UsbHostHidMouse_GetCurrentPosition().u32Y);
 **             printf("Scroll Position %d\r\n",
 **                     UsbHostHidMouse_GetCurrentScrollPosition());
 **             printf("Button Bitmap 0x%02X\r\n",
 **                     UsbHostHidMouse_GetButtons());
 **         }
 **     }
 ** }
 ** @endcode
 **
 ******************************************************************************/
     
/**
 ******************************************************************************    
 ** \page usbhostmouse_example_callbacks Example: Using callbacks
 ** @code  
 ** #include "usb.h"   
 ** #if (USBHOSTHIDMOUSE_ENABLED == ON)
 **     #include "UsbHostHidMouse.h"
 ** #endif  
 **   
 ** void MouseCallback(MouseEventType_t u8EventType, stc_mousedata_t* pstcInternalMouseData);
 ** {
 **    if (MOUSEEVENT_POSITIONX_CHANGED & u8EventType)
 **    {
 **        //your code...
 **    }
 **    if (MOUSEEVENT_POSITIONY_CHANGED & u8EventType)
 **    {
 **        //your code...
 **    }
 **    if (MOUSEEVENT_BUTTON_CHANGED & u8EventType)
 **    {
 **        //your code...
 **    }
 **    if (MOUSEEVENT_SCROLLING_CHANGED & u8EventType)
 **    {
 **        //your code...
 **    }
 ** }
 ** int main()
 ** {
 **     UsbConfig_UsbInit();   
 **     #if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDMOUSE_ENABLED == ON))
 **         UsbHostHidMouse_RegisterDriver();
 **     #endif 
 **     UsbHostHidMouse_SetCallback(MouseCallback);
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

#define USBHOSTHIDMOUSE_VERSION  0160
#define USBHOSTHIDMOUSE_DATE     20140903

/**
 ******************************************************************************
 ** \brief Get current cursor position
 **
 ** \returns stc_point_t
 **
 ******************************************************************************/     
#define UsbHostHidMouse_GetCurrentPosition()    (stcUsbHostMouseData.stcPosition)
#define USBHOSTMOUSE_CONVERTREPORT(data,value) \
  if ((value.u8ReportID == 0) || (value.u8ReportID == data[0]))\
  {\
     *((uint32_t*)&value.i32Data) = *((uint32_t*)&data[((value.u32Position - 1) / 8)]);\
     *((uint32_t*)&value.i32Data) = *((uint32_t*)&value.i32Data) >> ((value.u32Position - 1) % 8);\
     *((uint32_t*)&value.i32Data) = *((uint32_t*)&value.i32Data) & (0xFFFFFFFF >> (32 - value.u8Size));\
     if ((*((uint32_t*)&value.i32Data) & (1 << (value.u8Size - 1))) && (value.i32Min < 0))\
     {\
         *((uint32_t*)&value.i32Data) |= (0xFFFFFFFF << (value.u8Size -1));\
     }\
  }
  
/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
typedef uint8_t buttons_t;

typedef struct stc_usbdevicemouse_internal_data_value
{
    int32_t i32Data;
    uint8_t u8Size;
    int32_t i32Min;
    int32_t i32Max;
    uint32_t u32Position;
    uint8_t u8ReportID;
    uint8_t u8Index;
    uint8_t u8UsageId;
} stc_usbdevicemouse_internal_data_value_t;

typedef struct stc_usbdevicemouse_internal_data
{
    uint8_t au8Data[10];
    stc_usbdevicemouse_internal_data_value_t stcButtons;
    stc_usbdevicemouse_internal_data_value_t stcX;
    stc_usbdevicemouse_internal_data_value_t stcY;
    stc_usbdevicemouse_internal_data_value_t stcScroll;
} stc_usbdevicemouse_internal_data_t;

typedef struct stc_point
{
    uint32_t u32X;
    uint32_t u32Y;
} stc_point_t;

typedef struct stc_pointdifference
{
    int8_t i8X;
    int8_t i8Y;
} stc_pointdifference_t;

typedef struct stc_mousedata
{
    stc_point_t   stcPosition;
    buttons_t tButtons;
    stc_pointdifference_t    stcDiffXY;
    uint32_t  u32ScrollPosition;
    int8_t    i8Scrolling;    
} stc_mousedata_t;


typedef uint8_t MouseEventType_t;

typedef void (*pfn_usbhostmouse_data_change_t)(MouseEventType_t u8EventType, stc_mousedata_t* pstcInternalMouseData);

#define MOUSEEVENT_POSITIONX_CHANGED 1
#define MOUSEEVENT_POSITIONY_CHANGED 2
#define MOUSEEVENT_BUTTON_CHANGED    4
#define MOUSEEVENT_SCROLLING_CHANGED  8
#define MOUSEEVENT_ANY_CHANGE        0xFF
/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern stc_mousedata_t stcUsbHostMouseData;


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void UsbHostHidMouse_RegisterDriver(void);
boolean_t UsbHostHidMouse_InitHandler(stc_usbn_t* pstcUsb, uint8_t* pu8Configuration, uint32_t u32Length);
boolean_t UsbHostHidMouse_DeinitHandler(stc_usbn_t* pstcUsb);
void UsbHostHidMouse_Configured(stc_usbn_t* pstcUsb);
void UsbHostHidMouse_SetCallback(pfn_usbhostmouse_data_change_t pfnCallback);
void UsbHostHidMouse_RemoveCallback(void);

void UsbHostHidMouse_SetCurrentPosition(uint32_t u32X, uint32_t u32Y);
uint32_t UsbHostHidMouse_GetCurrentScrollPosition(void);
void UsbHostHidMouse_SetCurrentScrollPosition(uint32_t u32ScrollPosition);
buttons_t UsbHostHidMouse_GetButtons(void);
boolean_t UsbHostHidMouse_Moved(void);
boolean_t UsbHostHidMouse_DriverActive(void);
boolean_t UsbHostHidMouse_IsActive(stc_usbn_t* pstcUsb);

#ifdef __cplusplus
}
#endif

//@} // UsbHostMouseGroup

#endif /* ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTHIDMOUSE_ENABLED == ON)) */
#endif /* __USBHOSTHIDMOUSE_H__ */
