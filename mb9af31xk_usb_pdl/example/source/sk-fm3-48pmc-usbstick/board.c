/* board.c - Source Code for Spansion's Evaluation Board Support Package */
/**************************************************************************
* Copyright (C)2011 Spansion LLC. All Rights Reserved . 
*
* This software is owned and published by: 
* Spansion LLC, 915 DeGuigne Dr. Sunnyvale, CA  94088-3453 ("Spansion").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND 
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software constitutes driver source code for use in programming Spansion's 
* Flash memory components. This software is licensed by Spansion to be adapted only 
* for use in systems utilizing Spansion's Flash memories. Spansion is not be 
* responsible for misuse or illegal use of this software for devices not 
* supported herein.  Spansion is providing this source code "AS IS" and will 
* not be responsible for issues arising from incorrect user implementation 
* of the source code herein.  
*
* SPANSION MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE, 
* REGARDING THE SOFTWARE, ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED 
* USE, INCLUDING, WITHOUT LIMITATION, NO IMPLIED WARRANTY OF MERCHANTABILITY, 
* FITNESS FOR A  PARTICULAR PURPOSE OR USE, OR NONINFRINGEMENT.  SPANSION WILL 
* HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT, NEGLIGENCE OR 
* OTHERWISE) FOR ANY DAMAGES ARISING FROM USE OR INABILITY TO USE THE SOFTWARE, 
* INCLUDING, WITHOUT LIMITATION, ANY DIRECT, INDIRECT, INCIDENTAL, 
* SPECIAL, OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA, SAVINGS OR PROFITS, 
* EVEN IF SPANSION HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.  
*
* This software may be replicated in part or whole for the licensed use, 
* with the restriction that this Copyright notice must be included with 
* this software, whether used in part or whole, at all times.  
******************************************************************************/
/** \file board.c
 **
 **  for SK-FM3-48PMC-USBSTICK
 **
 ** History:
 **   - 2012-02-03  1.0  MSc  First Version
 **   - 2014-03-11  1.1  MSc  Updated for USB Wizard V3.0
 *************************************************************************/


/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "board.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

void Board_UpdateTasks(void)
{
    #if BOARD_USB == ON     
        UsbConfig_SwitchMode();
    #endif
}

void Board_Init(void)
{
    #if defined(bFM3_FLASH_IF_FBFCR_BE)
    bFM3_FLASH_IF_FBFCR_BE = 1; // Enable Flash Accelerator
    #endif 

    #if BOARD_LED == ON
    LED_INIT;
    #endif
        
    #if BOARD_UART == ON
        Uart_Init(BOARD_UART_CH,UART_BAUDRATE(BOARD_UART_SPEED));
    #endif
    
    #if BOARD_BUTTON == ON
        Button_Init();
    #endif

    #if BOARD_SYSTICK == ON
        BoardSystick_Init();
    #endif

    #if BOARD_USB == ON
       UsbConfig_UsbInit();
    #endif

    #if BOARD_SENSOR_ADC == ON
    SensorAdc_AdcHardwareInit();
    #endif
}
