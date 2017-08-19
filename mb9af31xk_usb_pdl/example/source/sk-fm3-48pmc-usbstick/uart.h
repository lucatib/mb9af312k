/* uart.h - Source Code for Spansion's Evaluation Board Support Package */
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
/** \file uart.h
 **
 ** Universal UART routines for FM3 & FM4 & FM0
 **
 ** Supported boards:
 ** - FSSDC-9B506-EVB
 ** - SK-FM3-48PMC-MB9BF524K
 ** - SK-FM3-48PMC-USBSTICK
 ** - SK-FM3-64PMC1
 ** - SK-FM3-80PMC-MB9BF524M
 ** - SK-FM3-100PMC
 ** - SK-FM3-100PMC-MB9AFB44N
 ** - SK-FM3-100PMC-MB9BF516N
 ** - SK-FM3-176PMC-ETHERNET
 ** - SK-FM3-176PMC-TFT
 ** - SK-FM4-U120-9B560
 ** - SK-FM4-120PMC-TFT
 ** - S6SE1A12C0ASA0002
 **
 ** History:
 **   - 2011-03-30  1.0  MSc  First Version for FM3
 **   - 2013-06-19  1.1  MSc  Universal UART routines for FM3 & FM4
 **   - 2013-08-01  1.2  MSc  SK-FM3-176PMC-TFT & SK-FM4-120PMC-TFT added
 **   - 2014-01-29  1.3  MSc  Fixed define _SK_FM3_80PMC_MB9BF524M_
 **   - 2014-03-03  1.4  MSc  Fixed define SK-FM3-176PMC-FA
 **   - 2014-03-12  1.5  MSc  Compatibility to different mcu headerfiles fixed
 **   - 2014-04-11  1.6  MSc  Support for Bulb-Board-Mini Added
 **   - 2014-06-05  1.7  MSc  Warnings fixed, SK-FM0P-48LQFP-9AF160K_S6SE1A12C0ASA0002 added
 *************************************************************************/

#ifndef __UART_H__
#define __UART_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "mcu.h"
   

#include "base_types.h"

#include <string.h>

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
#define USE_PRINTF            1
#define UART_USE_L3                                       0
#define UART_USEWITH_EVABOARD 1






#if (UART_USEWITH_EVABOARD == 1)
    #include "board.h"
#endif

#if (USE_PRINTF == 1)
    #include <stdio.h>
#endif
#define  HEAP_SIZE     16

#ifndef CLOCK_FREQ
    #ifdef __PLLCLK
        #define CLOCK_FREQ __PLLCLK
    #else
        #error __PLLCLK is undefined!
    #endif
#endif

//#if (UART_USE_L3 == 0)
    #define UART_BAUDRATE(x)  (uint16_t)((CLOCK_FREQ/((uint32_t)x - 1))/2)
//#else
//    #define UART_BAUDRATE(x)  x
//#endif

//#if UART_USE_L3 == 1
//    #include "mfs.h"
//#endif

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

typedef struct stc_uart_mfs_uart_smr_field
{
  __IO  uint8_t SOE        : 1;
        uint8_t RESERVED1  : 1;
  __IO  uint8_t BDS        : 1;
  __IO  uint8_t SBL        : 1;
  __IO  uint8_t WUCR       : 1;
  __IO  uint8_t MD         : 3;
} stc_uart_mfs_uart_smr_field_t;

typedef struct stc_uart_mfs_uart_scr_field
{
  __IO  uint8_t TXE        : 1;
  __IO  uint8_t RXE        : 1;
  __IO  uint8_t TBIE       : 1;
  __IO  uint8_t TIE        : 1;
  __IO  uint8_t RIE        : 1;
        uint8_t RESERVED1  : 2;
  __IO  uint8_t UPCL       : 1;
} stc_uart_mfs_uart_scr_field_t;

typedef struct stc_uart_mfs_uart_escr_field
{
  __IO  uint8_t L0         : 1;
  __IO  uint8_t L1         : 1;
  __IO  uint8_t L2         : 1;
  __IO  uint8_t P          : 1;
  __IO  uint8_t PEN        : 1;
  __IO  uint8_t INV        : 1;
  __IO  uint8_t ESBL       : 1;
  __IO  uint8_t FLWEN      : 1;
} stc_uart_mfs_uart_escr_field_t;

typedef struct stc_uart_mfs_uart_ssr_field
{
  __IO  uint8_t TBI        : 1;
  __IO  uint8_t TDRE       : 1;
  __IO  uint8_t RDRF       : 1;
  __IO  uint8_t ORE        : 1;
  __IO  uint8_t FRE        : 1;
  __IO  uint8_t PE         : 1;
        uint8_t RESERVED1  : 1;
  __IO  uint8_t REC        : 1;
} stc_uart_mfs_uart_ssr_field_t;

typedef struct stc_uart_mfs_uart_rdr_field
{
       uint16_t RESERVED1  : 8;
  __IO uint16_t AD         : 1;
} stc_uart_mfs_uart_rdr_field_t;

typedef struct stc_uart_mfs_uart_tdr_field
{
       uint16_t RESERVED1  : 8;
  __IO uint16_t AD         : 1;
} stc_uart_mfs_uart_tdr_field_t;

typedef struct stc_uart_mfs_uart_bgr_field
{
       uint16_t RESERVED1  : 15;
  __IO uint16_t EXT        : 1;
} stc_uart_mfs_uart_bgr_field_t;

typedef struct stc_uart_mfs_uart_bgr1_field
{
        uint8_t RESERVED1  : 7;
  __IO  uint8_t EXT        : 1;
} stc_uart_mfs_uart_bgr1_field_t;

typedef struct stc_uart_mfs_uart
{
  union {
    __IO  uint8_t SMR;
    stc_uart_mfs_uart_smr_field_t SMR_f;
  };
  union {
    __IO  uint8_t SCR;
    stc_uart_mfs_uart_scr_field_t SCR_f;
  };
       uint8_t RESERVED1[2];
  union {
    __IO  uint8_t ESCR;
    stc_uart_mfs_uart_escr_field_t ESCR_f;
  };
  union {
    __IO  uint8_t SSR;
    stc_uart_mfs_uart_ssr_field_t SSR_f;
  };
       uint8_t RESERVED3[2];
  union {
    union {
      __IO uint16_t RDR;
      stc_uart_mfs_uart_rdr_field_t RDR_f;
    };
    union {
      __IO uint16_t TDR;
      stc_uart_mfs_uart_tdr_field_t TDR_f;
    };
  };
       uint8_t RESERVED5[2];
  union {
    union {
      __IO uint16_t BGR;
      stc_uart_mfs_uart_bgr_field_t BGR_f;
    };
    struct {
      __IO  uint8_t BGR0;
      union {
        __IO  uint8_t BGR1;
        stc_uart_mfs_uart_bgr1_field_t BGR1_f;
      };
    };
  };
} stc_uart_mfs_uart_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

boolean_t Uart_Init(uint8_t u8Uart, uint32_t Baudrate);
void Uart_Putch(uint8_t u8Uart, char_t c);
boolean_t Uart_HasData(uint8_t u8Uart);
boolean_t Uart_DefaultHasData(void);
void putch(char_t c);
char_t Uart_Getch(uint8_t u8Uart);
char_t getch(void);
void Uart_Puts(uint8_t u8Uart, char_t* String);
int puts(const char_t* String);
void puthex(uint32_t n, uint8_t digits);
void putdec(uint32_t x);
unsigned long ASCIItobin(uint8_t k);
void receive_line(void);
int receive_line_echo(int *cnt);
int scan_line(char_t *str) ;
unsigned long Inputhex(uint8_t digits);
char upcase(char_t k);

/* Low-Level functions */
#if (USE_PRINTF == 1)
    int __write(int , char *, unsigned int);
    int __close(int);
    int __read(int , char *, unsigned int);
#endif /* (USE_PRINTF == 1) */

#endif /* __UART_H__ */
