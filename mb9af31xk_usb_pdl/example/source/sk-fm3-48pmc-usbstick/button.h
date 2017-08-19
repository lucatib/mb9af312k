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
*******************************************************************************/
/** \file button.h
 **
 ** for SK-FM3-48PMC-USBSTICK
 **
 ** History:
 **   - 2012-02-03  1.0  MSc  First Version
 **   - 2012-07-17  1.1  MSc  base_type.h -> base_types.h
 **   - 2014-03-11  1.2  MSc  base_type_l3.h -> base_types.h
 *****************************************************************************/

#ifndef __BUTTON_H__
#define __BUTTON_H__
#include "board.h"

#if BOARD_BUTTON == ON
#define _SK_FM3_48PMC_USBSTICK_
#define _BUTTON_API_MSC_
#define API_NUMBEROFBUTTONS 1
/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "mcu.h"
#include "base_types.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define BOARD_USERBUTTON bFM3_GPIO_PDIR0_PF

#ifndef ON
#define ON 1
#endif

#ifndef OFF
#define OFF 0
#endif

#ifndef BUTTON_PRESSED
#define BUTTON_PRESSED 0
#endif

#ifndef TOGGLE_PORT
#define TOGGLE_PORT(x) x = ~x
#endif

#define BUTTON_SW1_PRESSED() (BOARD_USERBUTTON == BUTTON_PRESSED)
#define BUTTON_USER_PRESSED() (BOARD_USERBUTTON == BUTTON_PRESSED)
#define BUTTON_CENTER_PRESSED() (BOARD_USERBUTTON == BUTTON_PRESSED)
/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

void Button_Init(void);
#endif
#endif /* __BUTTON_H__ */
