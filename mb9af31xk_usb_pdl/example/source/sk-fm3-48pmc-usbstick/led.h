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
/** \file led.h
 **
 ** for SK-FM3-48PMC-USBSTICK
 **
 ** History:
 **   - 2012-02-03  1.0  MSc  First Version
 **   - 2012-07-17    1.1  MSc  base_type.h -> base_types.h
 **   - 2012-08-31    1.2  MSc  for use with L3
 **   - 2014-03-11    1.3  MSc  base_type_l3.h -> base_types.h
 *****************************************************************************/

#ifndef __LED_H__
#define __LED_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "mcu.h"
#include "base_types.h"
#include "ledpwm.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
#define LED_DIMMABLE 1

#if (LED_DIMMABLE == 1)
  #define LED_RED       0
  #define LED_GREEN     2
  #define LED_BLUE      4

  #define LED_ON(x)     LedPwm_SetDuty(x,255);
  #define LED_OFF(x)    LedPwm_SetDuty(x,0);
  #define LED_DUTY(x,y) LedPwm_SetDuty(x,y);

  #define LED_INIT     LedPwm_Init(); LED_OFF(0); LED_OFF(2); LED_OFF(4)
#else
  #define LED_RED       (1<<10)
  #define LED_GREEN     (1<<12)
  #define LED_BLUE      (1<<14)

  #define LED_MASK    (LED_RED  | LED_GREEN | LED_BLUE )

  #define LED_INIT     FM3_GPIO->PFR3 &=~LED_MASK; FM3_GPIO->PDOR3 &= ~LED_MASK; FM3_GPIO->DDR3 |= LED_MASK;

  #define LED_ON(x)     FM3_GPIO->PDOR3 |= x;
  #define LED_OFF(x)    FM3_GPIO->PDOR3 &= ~x;
#endif

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/


#endif /* __LED_H__ */
