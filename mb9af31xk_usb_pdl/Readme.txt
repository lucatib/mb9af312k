================================================================================
                               Template Project
================================================================================

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

================================================================================
History

# Virtual Com Port (CDC) Information:
- 2009-08-31    1.0  MSc  First version  (works with 16FX)
- 2010-04-16    1.1  MSc  new USB library added - API had changed
- 2011-08-30    1.2  MSc  bug fixes while echo data
                          added UsbDeviceCdcCom_SetEchoAtSeperatorOnly for
                          echo data only after seperator received
- 2012-07-24    1.3  MSc  Version for USB Library >= FM3 
- 2012-11-26    1.4  MSc  Data received routine added 
- 2013-01-30    1.5  MSc  DMA routines added 
- 2013-04-24    1.6  MSc  Data sent handle added
- 2013-10-14    1.7  MSc  PDL support added

# MCU Template Information:
Date        Ver       Initials   Description
2015-06-30  2.0.0     MSCH/ACEH  New MCU Template / Package release, replacing
                                 MCU templates before v20

2015-08-03  2.0.1     MSCH       Updated generation script to fix FPU and MPU usage
                                 for ARM MDK

2015-08-18  2.0.2     NOSU       Updated SVD files to improve compatibility with
                                 register/bit definitions of MCU templates before
                                 v20
                                 Removed HDMICEC related definitions in S6E2Cx
                                 series
                                 Fixed the folder structure descriptions in Readme.txt

2016-03-11  2.1.0     MISH       Updated Merged and Clean ALL FM3 Series PDSC Files

2016-03-15  2.1.1     ACEH/NOSU/MISH Updated system files

2016-04-18  2.1.2     ACEH       iSystem linker file and startup updated
                                 pdsc vendor information updated
                                 IAR: map file enabled
                                 ADC: added SD_FDAS and PD_FDAS definition

2016-04-18  2.1.3     ACEH       public release

2016-05-19  2.2.0     NOSU       Updated gpio header files, Updated system files

2016-05-19  2.2.1     NOSU       Flash loader update: IAR: [S6E2D, S6E2G] fixed
                                 the problem that Flash loaders may not program
                                 the CR Trimming area / ARM: [S6E2G] fixed the
                                 problem that chip erase command generates timeout
                                 error

2016-05-31  2.2.2     ACEH/NOSU  added missing FASTIO definitions

2016-06-07  2.2.3     MSCH       Public release


Supported toolchain versions

ARM        5.11
Atollic    4.3.1
IAR        7.30
iSYSTEM    9.12.213
================================================================================
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################

This is a project template for MB9AF31xK (with x = flash size). It includes some
basic settings for e.g. Linker, C-Compiler.


Clock settings:
---------------
Crystal:   4 MHz
HCLK:    160 MHz
PCLK0 :   80 MHz
PCLK1:   160 MHz
PCLK2:    80 MHz




================================================================================
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################
File Structure
--------------

 + cmsis                                     CMSIS related files
 | + include                                 CMSIS header files (for GNU toolchains)
 | + svd                                     SVD files
 | | |-> mb9af31xk.sfr                       precompiled SVD file
 | | |-> mb9af31xk.svd                       SVD file defined in CMSIS
 |
 + common                                    Common files folder
 | |-> base_types.h                          Defines base types variables
 | |-> fgpio.h                               GPIO macros used for fast GPIOs
 | |-> gpio.h                                GPIO macros used for GPIOs
 | |-> gpio_mb9af31xk.h                      GPIO macros used for GPIOs
 | |-> MB9AF31xK.h                           MCU Headerfile
 | |-> system_mb9a310k.c                     System file defined in CMSIS
 | |-> system_mb9a310k.h                     System file defined in CMSIS
 |
 + doc                                       Documentation folder
 | |-> + doxy                                Output folder for Doxygen
 | |-> Doxyfile                              Doxygen configuration
 | |-> mb9af31xk.pin                         Pin list
 |
 + example                                   Example folder, containing source & IDE workspaces
 | + ARM                                     ARM/µVision IDE workspace
 | | |-> *.FLM                               Flash loaders, please copy to C:\Keil_v5\ARM\Flash\
 | | |-> mb9af31xk_example_178172004.uvprojx          ARM/µVision IDE project/workspace file
 | | |-> startup_mb9a310k.s                  Startup file for ARM/µVision
 | | + output                                output folder
 | | | + release                             output folder for release build
 | | | | |-> mb9af31xk_example_178172004.srec         firmware file
 | | | |
 | | | + debug                               output folder for debug build
 | | | | |-> mb9af31xk_example_178172004.srec         firmware file
 | | |                                       +--------------------------------------------------------------------+
 | | |                                       |  General Hints to setup ARM µVision                                |
 | | |                                       |  ======================================                            |
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################
 | | |                                       |  - Copy the *.FLM files to C:\Keil\ARM\Flash\                      |
 | | |                                       |  - Install legacy support if using Keil V5 for Cortex-M Devices:   |
 | | |                                       |    http://www2.keil.com/mdk5/legacy/                               |
 | | |                                       +--------------------------------------------------------------------+
 | |
 | + Atollic                                 Atollic IDE workspace
 | | |-> .project                            Atollic IDE project/workspace file
 | | |-> startup_mb9a310k.s                  Startup file for Atollic
 | | + ..._release                           output folder for release build
 | | | |-> mb9af31xk_example_178172004.srec           firmware file
 | | + ..._debug                             output folder for debug build
 | | | |-> mb9af31xk_example_178172004.srec           firmware file
 | |
 | + IAR                                     IAR IDE workspace
 | | |-> mb9af31xk_example_178172004.eww              IAR project/workspace file
 | | |-> startup_mb9a310k.s                  Startup file for IAR
 | | + output                                output folder
 | | | | + release                           output folder for release build
 | | | | | + exe                             
 | | | | |   |-> mb9af31xk_example_178172004.srec     firmware file
 | | | | + debug                             output folder for debug build
 | | | | | + exe                             
 | | | | |   |-> mb9af31xk_example_178172004.srec     firmware file
 | |
 | + iSYSTEM                                 iSYSTEM IDE workspace
 | | |-> mb9af31xk_example_178172004.xjrf             iSYSTEM project/workspace file
 | | |-> startup_mb9a310k.s                  Startup file for iSYSTEM
 | | + output                                output folder
 | | | + release                             output folder for release build
 | | | | |-> mb9af31xk_example_178172004.srec         firmware file
 | | | + debug                               output folder for debug build
 | | | | |-> mb9af31xk_example_178172004.srec         firmware file
 | + make                                    Makefile environment
 | | |-> startup_mb9a310k.s                  Startup file for GNU toolchain
 | | |-> makefile                            makefile
 | | |-> clean.bat                           Cleanup script
 | | |-> make.bat                            Start make process
 | | |-> MakeFileUpdate.exe                  Run every time, new folders were added to update the makefile
 | | | + release                             output folder for release build
 | | | | |-> mb9af31xk_example_178172004.srec         firmware file
 | | | + debug                               output folder for debug build
 | | | | |-> mb9af31xk_example_178172004.srec         firmware file
 | |                                         +--------------------------------------------------------------------+
 | |                                         |  General Hints to setup make on Windows                            |
 | |                                         |  ======================================                            |
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################
 | |                                         |  - Download and install http://gnuwin32.sourceforge.net/           |
 | |                                         |  - Download and install MinGW http://www.mingw.org/                |
 | |                                         |  - Check in system advanced properties if the PATH variable        |
 | |                                         |    includes the MinGW binary path                                  |
 | |                                         |  - Install iSYSTEM WinIDEA Open or change the TOOLSDIR variable    |
 | |                                         |   in the makefile to your GNU tool chain                           |
 | |                                         +--------------------------------------------------------------------+
 | |
 | + source                                  Source files
 | | |-> main.c                              Main file
 | | |-> mcu.h                               Includes mcu related header files
 | | + config                                Configuration directory
 | | | |-> RTE_Device.h                      Configuration file RTE_Device.h defined by CMSIS
 | | | |-> iomux.c                           Used with Pin & Code Wizard
 | | | |-> iomux.h                           Used with Pin & Code Wizard
 |
 + library                                   Library folder
 | + examples                                Library examples folder (empty)
 | + highlevel                               Library high level folder (empty)
 | + lowlevel                                Library low level folder (empty)
 | + middleware                              Library middle ware folder (empty)
 + thirdparty                                Third party folder (empty)
--------------------------------------------------------------------------------

    mb9af31xk            = MCU name, x = flash size
    mb9a310k             = MCU series name
    mb9af31xk_example_178172004   = Project name
    Debug                = Project compiled to run in RAM
    Release              = Project compiled to run in Flash
================================================================================
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################
 
================================================================================ 
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################
  Automatic created by McuTemplateGen 
  [2016-06-07 09:19:36.563]  
================================================================================ 
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################
 
================================================================================ 
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################
EOF (not truncated) 
================================================================================ 
# Virtual Com Port (CDC) Information:
This is a project demonstrates the virtual com port usage under windows 32-bit.
Drivers can be found under directory "Windows Drivers". After a successful 
driver installation, start a serial terminal like "Hyper Terminal".
Try to connect with 9600,N,8 without hardware flow control. The connection speed
has no effect to the transfer speed and will be all the time the maximum speed
which will be reached by the usb library. After connect the 7 segments
should display "OK" and a welcome message will be displayed on the terminal.
Messages can be typed in and every char will be displayed as hex in the 7 segment 
display, too. After a return, the terminal answers. 

Notes:
  The virtual comm port does not work in every serial terminal configuration.
  For example it does not work with SK Wizard. For own applications, set the timeout delay
  in the serial component in your program / serial terminal to "0". (not -1)
  This should work.

For more information see also application note mcu-an-300122-e-usb_virtual_com_port.


The Virtual Comm Port was tested on following platforms:
- Windows 2000
- Windows XP
- Windows Vista 32/64 Bit
- Linux
- Mac OS X

CDC API:
  void UsbClassCdc_Init(void);                                           /* Initialisation */
  void UsbClassCdc_SetSeparator(const char_t cNewSeperator);             /* defines buffer end character nad splits buffer here */
  void UsbClassCdc_SetEchomode(boolean_t bEchoOnOff);                    /* echo on/off */
  uint8_t UsbClassCdc_IsConnected(void);                                 /* get vcomm connection state */
  uint8_t UsbClassCdc_SendByte(char_t c);                                /* send a byte or char */
  uint8_t UsbClassCdc_SendString(char_t *pcBuffer);                      /* send a string */
  uint8_t UsbClassCdc_SendBuffer(uint8_t *pu8Buffer, uint32_t u32Size);  /* send a buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* send a hex value in ASCII */
  uint8_t UsbClassCdc_SendDec(uint32_t x);                               /* send a decimal value in ASCII */
  void UsbClassCdc_SendBinAsci4(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBinAsci8(uint16_t a, uint8_t base, char_t fill);  /* bin to ascii functions */
  void UsbClassCdc_SendBin2DotDec99(uint8_t a);                          /* bin to ascii functions */
  uint32_t UsbClassCdc_ReceivedLength(void);                             /* get received length */
  uint32_t UsbClassCdc_ReceiveBuffer(uint8_t *pu8Buffer);                /* receive a buffer */
  uint8_t UsbClassCdc_ReceiveByte(void);                                 /* receive a byte or char (first in the buffer) */
  uint8_t UsbClassCdc_ReceiveLastByte(void);                             /* receive the last received byte (last one in the receive buffer */
  uint8_t UsbClassCdc_SendHex(uint32_t n, uint8_t digits);               /* receive a hex value */

To use the standard uart functions (like in uart.c), set in UsbClassCdc.h
VCOMM_USE_UARTCOMMANDS to 1. Available functions are:
  puts(...)
  putch(...)
  puthex(...,...)
  putdec(...)
  binasci8(...,...,...)
  binasci4(...,...,...)
  bin2_dot_dec99(...)
  getch()
  Inputhex(...)

MCU Template Information:
#########################
 
