/* uart.c - Source Code for Spansion's Evaluation Board Support Package */
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
/** \file uart.c
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
 ** - SK-FM3-176PMC-FA
 ** - BULB-BOARD-MINI
 ** - SK-FM0P-48LQFP-9AF160K
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


/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "uart.h"
#if (UART_USEWITH_EVABOARD == 0) || (defined(BOARD_UART) && defined(ON) && (BOARD_UART == ON))
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/


#define UARTREG(x,y)    (*(((volatile uint8_t*)UART_BASE[x]) + y))
/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/* global variables */
char_t line_buffer[255];
char_t *line_buffer_ptr;

/* constants */
const char_t ASCII[] = "0123456789ABCDEF";
static uint8_t u8DefaultUart = 0;



#if (USE_PRINTF == 1)
    char_t tbuf;

    static   long        brk_siz = 0;
    #if    HEAP_SIZE
        typedef  int         _heap_t;
        #define ROUNDUP(s)   (((s)+sizeof(_heap_t)-1)&~(sizeof(_heap_t)-1))
        static   _heap_t     _heap[ROUNDUP(HEAP_SIZE)/sizeof(_heap_t)];
        #define              _heap_size       ROUNDUP(HEAP_SIZE)
    #else
        extern  char         *_heap;
        extern  long         _heap_size;
    #endif
#endif

#if defined(FM3_PERIPH_BASE) 
static stc_uart_mfs_uart_t* UART_BASE[] = {
        #if defined(FM3_MFS0_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM3_MFS0_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM3_MFS1_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM3_MFS1_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM3_MFS2_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM3_MFS2_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM3_MFS3_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM3_MFS3_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM3_MFS4_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM3_MFS4_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM3_MFS5_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM3_MFS5_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM3_MFS6_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM3_MFS6_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM3_MFS7_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM3_MFS7_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
};
#elif defined(FM4_PERIPH_BASE)
static stc_uart_mfs_uart_t* UART_BASE[] = {
        #if defined(FM4_MFS0_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS0_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS1_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS1_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS2_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS2_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS3_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS3_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS4_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS4_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS5_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS5_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS6_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS6_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS7_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS7_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS8_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS8_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS9_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS9_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS10_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS10_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS11_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS11_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS12_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS12_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS13_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS13_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS14_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS14_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM4_MFS15_BASE)
          ((stc_uart_mfs_uart_t *)FM4_MFS15_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
};
#elif defined(FM0P_PERIPH_BASE)
static stc_uart_mfs_uart_t* UART_BASE[] = {
        #if defined(FM0P_MFS0_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM0P_MFS0_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM0P_MFS1_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM0P_MFS1_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM0P_MFS2_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM0P_MFS2_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM0P_MFS3_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM0P_MFS3_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM0P_MFS4_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM0P_MFS4_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM0P_MFS5_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM0P_MFS5_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM0P_MFS6_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM0P_MFS6_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
        #if defined(FM0P_MFS7_UART_BASE)
          ((stc_uart_mfs_uart_t *)FM0P_MFS7_UART_BASE),
        #else
          ((stc_uart_mfs_uart_t *)NULL),
        #endif
};
#endif

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/


boolean_t Uart_Init(uint8_t u8Uart, uint32_t Baudrate)
{
    if (UART_BASE[u8Uart] == NULL)
    {
        return FALSE;
    }
    UART_BASE[u8Uart]->SCR =  0x80;
    UART_BASE[u8Uart]->SMR =  0x01;
    UART_BASE[u8Uart]->SSR =  0x00;
    UART_BASE[u8Uart]->ESCR = 0x00;
    UART_BASE[u8Uart]->BGR = (uint16_t)Baudrate; //(uint16_t)BGRTemp;
    UART_BASE[u8Uart]->SCR = 0x03;
    u8DefaultUart = u8Uart;
    #if defined(UART_USEWITH_EVABOARD) && (UART_USEWITH_EVABOARD == 0)
        #warning UART_USEWITH_EVABOARD = 0, please define GPIO settings for SIN & SOT pin by your own
    #elif defined(_SK_FM4_U120_9B560_)
        switch (u8Uart)
        {
            case 0:
                /* SIN Pin */
                FM4_GPIO->ADE &= ~(1 << 17); 
                bFM4_GPIO_PFR2_P21 = 1;
                FM4_GPIO->EPFR07 &= ~(0x03 << 4);
                FM4_GPIO->EPFR07 |= (0x01 << 4);
                
                /* SOT Pin */
                FM4_GPIO->ADE &= ~(1 << 16); 
                bFM4_GPIO_PFR2_P22 = 1;
                FM4_GPIO->EPFR07 &= ~(0x03 << 6);
                FM4_GPIO->EPFR07 |= (0x01 << 6);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_FSSDC_9B506_EVB_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 7);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM3_48PMC_MB9BF524K_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                bFM3_GPIO_ADE_AN13 = 0;
                bFM3_GPIO_ADE_AN14 = 0;
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM3_48PMC_USBSTICK_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 7);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM3_64PMC1_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 7);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
     #elif defined(_SK_FM3_80PMC_MB9BF524M_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                bFM3_GPIO_ADE_AN13 = 0;
                bFM3_GPIO_ADE_AN14 = 0;
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM3_100PMC_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 7);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
     #elif defined(_SK_FM3_100PMC_MB9AFB44N_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 7);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM3_100PMC_MB9BF516N_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 7);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM3_176PMC_ETHERNET_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 31);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM3_176PMC_FA_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 31);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM3_176PMC_TFT_)
        switch (u8Uart)
        {
            case 0:
                FM3_GPIO->PFR2 |= (1 << 0x01) | (1 << 0x02);
                FM3_GPIO->EPFR07 |= (1 << 6);
                FM3_GPIO->ADE &= ~(1 << 31);
                break;
            case 3:
                FM3_GPIO->PFR4 |= (1 << 8) | (1 << 9);
                FM3_GPIO->EPFR07 |= (0x03 << 24) | (0x03 << 22);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM4_120PMC_TFT_)
        switch (u8Uart)
        {
            case 0:
                /* SIN Pin */
                FM4_GPIO->ADE &= ~(1 << 17); 
                bFM4_GPIO_PFR2_P21 = 1;
                FM4_GPIO->EPFR07 &= ~(0x03 << 4);
                FM4_GPIO->EPFR07 |= (0x01 << 4);
                
                /* SOT Pin */
                FM4_GPIO->ADE &= ~(1 << 16); 
                bFM4_GPIO_PFR2_P22 = 1;
                FM4_GPIO->EPFR07 &= ~(0x03 << 6);
                FM4_GPIO->EPFR07 |= (0x01 << 6);
                break;
            case 3:
                /* SIN Pin */ 
                bFM4_GPIO_PFR6_P66 = 1;
                FM4_GPIO->EPFR07 &= ~(0x03 << 22);
                FM4_GPIO->EPFR07 |= (0x01 << 22);
                
                /* SOT Pin */
                bFM4_GPIO_PFR6_P67 = 1;
                FM4_GPIO->EPFR07 &= ~(0x03 << 24);
                FM4_GPIO->EPFR07 |= (0x01 << 24);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_BULB_BOARD_MINI_)
        switch (u8Uart)
        {
            case 0:
                /* SIN Pin */
                bFM0P_GPIO_PFR2_P21 = 1;
                FM0P_GPIO->EPFR07 &= ~(0x03 << 4);
                FM0P_GPIO->EPFR07 |= (0x01 << 4);
                
                /* SOT Pin */
                FM0P_GPIO->ADE &= ~(1 << 7); 
                bFM0P_GPIO_PFR2_P22 = 1;
                FM0P_GPIO->EPFR07 &= ~(0x03 << 6);
                FM0P_GPIO->EPFR07 |= (0x01 << 6);
                break;
            case 3:
                /* SIN Pin */ 
                bFM0P_GPIO_PFR6_P60 = 1;
                FM0P_GPIO->EPFR07 &= ~(0x03 << 22);
                FM0P_GPIO->EPFR07 |= (0x01 << 22);
                
                /* SOT Pin */
                bFM0P_GPIO_PFR6_P61 = 1;
                FM0P_GPIO->EPFR07 &= ~(0x03 << 24);
                FM0P_GPIO->EPFR07 |= (0x01 << 24);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #elif defined(_SK_FM0P_48LQFP_9AF160K_) || defined(_S6SE1A12C0ASA0002_) || defined(_SK_FM0P_48LQFP_9AF160K_S6SE1A12C0ASA0002_)
        switch (u8Uart)
        {
            case 0:
                /* SIN Pin */
                bFM0P_GPIO_PFR2_P21 = 1;
                FM0P_GPIO->EPFR07 &= ~(0x03 << 4);
                FM0P_GPIO->EPFR07 |= (0x01 << 4);
                
                /* SOT Pin */
                FM0P_GPIO->ADE &= ~(1 << 7); 
                bFM0P_GPIO_PFR2_P22 = 1;
                FM0P_GPIO->EPFR07 &= ~(0x03 << 6);
                FM0P_GPIO->EPFR07 |= (0x01 << 6);
                break;
            default:
                // not supported by eva board, but may usable
                return FALSE;
        }
    #else
        #error Evaluation board unknown, set in uart.h UART_USEWITH_EVABOARD to 0
    #endif
    return TRUE;
}

void Uart_Putch(uint8_t u8Uart, char_t c)
{
    if (UART_BASE[u8Uart] == NULL)
    {
        return;
    }
    while (!(UART_BASE[u8Uart]->SSR & (1 << 1)));		/* wait for transmit buffer empty 	*/
    UART_BASE[u8Uart]->RDR = c;            /* put ch into buffer	*/
}

char_t Uart_Getch(uint8_t u8Uart)
{
    if (UART_BASE[u8Uart] == NULL)
    {
        return 0;
    }
    while(!(UART_BASE[u8Uart]->SSR & (1 << 2)));			/* wait for data in buffer */
    if (UART_BASE[u8Uart]->SSR & 0x38) 
    {	/* overrun or parity error */
        UART_BASE[u8Uart]->SSR |= (1 << 7);
        return 0;//(-1);
    } 
    else
    {
	return (UART_BASE[u8Uart]->RDR);
    }
}

boolean_t Uart_HasData(uint8_t u8Uart)
{
    if (UART_BASE[u8Uart] == NULL)
    {
        return FALSE;
    }
    if ((UART_BASE[u8Uart]->SSR & (1 << 2)))
    {
        if (UART_BASE[u8Uart]->SSR & 0x38) 
        {	/* overrun or parity error */
            UART_BASE[u8Uart]->SSR |= (1 << 7);
            return FALSE;//(-1);
        } 
        else
        {
            return TRUE;
        }
    }
    return FALSE;
}

boolean_t Uart_DefaultHasData(void)
{
    return Uart_HasData(u8DefaultUart);
}


void putch(char_t c)
{
    Uart_Putch(u8DefaultUart, c);
}


char_t getch()
{
    return Uart_Getch(u8DefaultUart);
}

void Uart_Puts(uint8_t u8Uart, char_t* String)
{
	while (*String != '\0')
	{ 
        //if(*String == '\n') Uart_Putch(u8Uart,'\r');
        Uart_Putch(u8Uart,*String++);        /* send every char of string */
    }
}

int puts(const char_t* String)
{
    Uart_Puts(u8DefaultUart,(char_t*)String);
    return 0;
}

/*****************************************************************************
 *  DESCRIPTION:    sends a x-digit Hex-number (as ASCII charcaters)
 *                  to terminal via ext. UART
 *
 *  PARAMETERS:     Value and number of Hex-digits (e.g. FF = 2 Hex digits)
 *
 *  RETURNS:        NONE
 *****************************************************************************/
void puthex(uint32_t n, uint8_t digits)
{
	unsigned char i,ch,div=0;

	puts("0x");				/* hex string */
	div=(digits-1) << 2;	/* init shift divisor */

	for (i=0; i<digits; i++) {
	  ch = (n >> div) & 0xF;/* get hex-digit value */
	  putch(ASCII[ch]);		/* prompt to terminal as ASCII */
	  div-=4;				/* next digit shift */
   }
}

/*****************************************************************************
 *  DESCRIPTION:    send a x-digit Dec-number (as ASCII charcaters)
 *                  to terminal via ext. UART
 *
 *  PARAMETERS:     integer value
 *
 *  RETURNS:        None
 *****************************************************************************/
void putdec(uint32_t x)
{
	int i;
	char buf[9];
	
	buf[8]='\0';				/* end sign of string */
	
	for (i=8; i>0; i--) 
	{
       buf[i-1] = ASCII[x % 10];
	   x = x/10;

	}

	for (i=0; buf[i]=='0'; i++) // no print of zero 
	{
	   buf[i] = ' ';
    }
	puts(buf);					/* send string */
}


/*****************************************************************************
 *  DESCRIPTION:    Converts ASCII inputs (0..8,a..f,A..F) to num. value
 *
 *  PARAMETERS:     ASCII input
 *
 *  RETURNS:        value or -1 (error)
 *****************************************************************************/
unsigned long ASCIItobin(unsigned char k)
{
  char d=(char) -1;
  if ((k > 47) & (k < 58)) d = k - 48;  /* 0..9 */
  if ((k > 64) & (k < 71)) d = k - 55;  /* A..F */
  if ((k > 96) & (k < 103)) d = k - 87; /* a..f */
  return d;
}

/*****************************************************************************
 *  DESCRIPTION:    Receives a string until CR from ext UART
 *
 *  PARAMETERS:     (stored in global variable "line_buffer")
 *
 *  RETURNS:        None
 *****************************************************************************/
void receive_line(void)
{
  unsigned char ch;
  unsigned short i=0;

    do {
            ch = getch();             
            if((ch == '\r') | (ch=='\n')) break;
            line_buffer[i++] = ch;
    } while(1);
   
    line_buffer[i]='\0';
    line_buffer_ptr = line_buffer + i;
}

/*****************************************************************************
 *  DESCRIPTION:    Receives a string until CR from ext UART + Echo
 *
 *  PARAMETERS:     (stored in global variable "line_buffer")
 *
 *  RETURNS:        None
 *****************************************************************************/
int receive_line_echo(int *cnt)
{
  
  unsigned char ch;
  unsigned short i=0;
  memset(line_buffer,0,sizeof(line_buffer));

	do {
		ch = getch();             
        putch(ch);
        if((ch == 13) | (ch==27)) break;
        line_buffer[i++] = ch;
   } while(1);

   line_buffer[i]='\0';
   line_buffer_ptr = line_buffer + i;
   if (cnt) *cnt = i;
   //BufferPtr = (char)  &line_buffer;
   return (int) &line_buffer;
}

/***********************************************************************
 * DESCRIPTION: Scans the string 'line_buffer' for the string 'string'.
 *              Both strings must be null-terminated. 'line_buffer_ptr'
 *              is set appropriately.
 * RETURN VALUE: 1, if string 'string' is contained in string 'line_buffer'
 *                  at its beginning. 'line_buffer_ptr' points to the
 *                  next character of 'line_buffer' after the equal field.
 *               2, if string 'string' was not found in string 'line_buffer'.
 *               0, if strings are identical to the final null character.
 ************************************************************************/
int scan_line(char *str) 
{
	line_buffer_ptr = line_buffer;

    while((int)*line_buffer_ptr==(int)*str)
	{
		if((int)*str=='\0') return(0);
		line_buffer_ptr++;
		str++;
	}
	
	if((int)*str=='\0') return(1);
	return(2);
}

/*****************************************************************************
 *  DESCRIPTION:    Inputs a x-digit Hex-number (ASCII characters expected)
 *                      ESC aborts input
 *
 *  PARAMETERS:     Number of Hex-digits (e.g. FF = 2 Hex digits)
 *
 *  RETURNS:        Input value or -1 (abort)
 *****************************************************************************/
unsigned long Inputhex(unsigned char digits)
{
   unsigned long number=0,digit=0;
   unsigned char abort=0,mlt=0,i,key;

   mlt=(4*(digits-1));  /* 20 for 6 hex-digit numers, 4 for 2 hex-digits */
   for (i=0;i<digits;i++)
   {
     digit = (char)-1;     
     while ((digit==(char)-1) & (!abort))     /* input next valid digit */
     {
       key = getch();             /* wait for next key input (ASCII) */
       putch(key);
       if (key == 27) abort=1;    /* ESC aborts */
       digit = ASCIItobin(key);   /* convert to number */ 
       if (digit == (char)-1) putch(8); /* backspace if not valid */
     }
     number+= (digit << mlt);     /* add digit value to number */
     mlt-=4;                      /* next digit shift */
   }

   if (!abort) 
      return number;             /* return input value */
   else
   {
      puts("\n\n input cancled \n");
      return (char)-1;                /* return abort indicator */
   }
}

char getkey(char LKey, char HKey)
{
   char key;

   do                           /* input next valid digit */
   {
     key = upcase(getch());     /* wait for next key input (0-9,A-Z,a-z) */
     if (key == 27)
     {
       puts("\r>>> cancel input \n");
       return (char)-1;            /* return with ESC aborts */
     }

     if ( (key < LKey) || (key > HKey) )
     {
       /* undefinded key pressed */
       puts("\r>>> key not defined \r");
     }
     else
     {
       puts("\r>                   \r");
       putch(key);
       return key;              /* return input value */
     }

   } while(1);
}

char upcase(char k)
{
  char d=(char) -1;
  if ((k > 47) & (k < 58))  d = k;      /* 0..9 */
  if ((k > 64) & (k < 71))  d = k;      /* A..F */
  if ((k > 96) & (k < 123)) d = k - 32; /* a..f */
  if (k == 27) d = k;                   /* ESC  */ 
  return d;
}

#if (USE_PRINTF == 1)
/*********************@FUNCTION_HEADER_START*************************
*@FUNCTION NAME:	write()                                         *
*                                                                   *
*@DESCRIPTION:		Low-Level function for printf()                 *
*                                                                   *
*@PARAMETER:		file no., pointer to buffer, size of data       *
*                                                                   *
*@RETURN:			size or error(-1)                               *
*                                                                   *
***********************@FUNCTION_HEADER_END*************************/
int __write(int fileno, char *buf, unsigned int size)
{
    unsigned int cnt;
    switch(fileno)
    {
        case 0 :  
            //return(0);                            /* stdin */
        case 1 :  
            for(cnt = 0;size > cnt;cnt++)         /* stdout */
            {
                tbuf = *buf++;
                if(tbuf == '\n')                      //add CR to newline for terminal output
                {
                    while((UART_BASE[u8DefaultUart]->SSR & 0x02) == 0)
                    {
                    }
                    UART_BASE[u8DefaultUart]->RDR = '\r';                        //send carriage return
                }
                while((UART_BASE[u8DefaultUart]->SSR & 0x02) == 0)
                {
                }
                UART_BASE[u8DefaultUart]->RDR = tbuf;

            }
            return(cnt);                          /* successful output */
        case 2 :  
            return(-1);                           /* stderr */
        default:  
            return(-1);                           /* should never happend */
    }
}

/*****************************************************************************
 *  DESCRIPTION:    low level read function, read characters via UART1
 *                  carrige return (ASCII:13) is translated into line feed
 *                  and carrige returmn (ASCII: 10 and 13)
 *
 *  PARAMETERS:     file number, buffer pointer, maximal buffer size
 *
 *  RETURNS:        successfull: number of read characters
 *                  error: -1
 *****************************************************************************/

int __read( int fileno, char *buf, unsigned int size) {

	unsigned int cnt;
	unsigned char helpchar;
    switch(fileno)
    {
        case 0 :  
            for(cnt = 0;size > cnt;cnt++)         /* stdin */
            {
                while((UART_BASE[u8DefaultUart]->SSR & 0x04) == 0)
                {
                }
                if ((UART_BASE[u8DefaultUart]->SSR & 0x28) > 0)
                {
                    return (char_t)-1;
                }
                helpchar = UART_BASE[u8DefaultUart]->RDR;
                if (helpchar == 13) 
                {
                    *buf=10;
		       		return(cnt+1);			/* successful input */
                }
                *buf=helpchar;
                buf++;
            }
            return(cnt);                          /* successful output */
        case 1 :  
            return(0);                            /* stdout */
        case 2 :  
            return(-1);                           /* stderr */
        default:  
            return(-1);                           /* should never happend */
    }
}
/*********************@FUNCTION_HEADER_START*************************
*@FUNCTION NAME:	close()                                         *
*                                                                   *
*@DESCRIPTION:		Low-Level function to close specific file       *
*                                                                   *
*@PARAMETER:		file number                                     *
*                                                                   *
*@RETURN:			successful or error (-1)                        *
*                                                                   *
***********************@FUNCTION_HEADER_END*************************/
int __close(int fileno)
{
    if((fileno >= 0) && (fileno <= 2))
    {
        return(0);
    }
    else
    {
        return(-1);
    }
}

extern  char   *sbrk(int size)
{
   if (brk_siz + size > _heap_size || brk_siz + size < 0)
        return((char*)-1);
   brk_siz += size;
   return( (char *)_heap + brk_siz - size);
}


#endif /* (USE_PRINTF == 1) */
#endif /* (UART_USEWITH_EVABOARD == 0) || (defined(BOARD_UART) && defined(ON) && (BOARD_UART == ON)) */
