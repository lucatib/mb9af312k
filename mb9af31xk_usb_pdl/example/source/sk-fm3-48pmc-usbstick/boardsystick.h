/* boardsystick.h - Source Code for Spansion's Evaluation Board Support Package */
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
/** \file boardsystick.h
 **
 ** for SK-FM3-48PMC-USBSTICK
 **
 ** History:
 **   - 2014-03-11  1.0  MSc  First Version
 *****************************************************************************/
#ifndef __BOARDSYSTICK_H__
#define __BOARDSYSTICK_H__
#include "board.h"
#if BOARD_SYSTICK == ON
#include "mcu.h"
#include <stdint.h>

typedef struct stc_board_uptime
{
  volatile uint32_t u32MSeconds;
  volatile uint32_t u32Seconds;
  volatile uint32_t u32Minutes;
  volatile uint32_t u32Hours;
  volatile uint32_t u32Days;
} stc_board_uptime_t;

#define MINISCHEDULER_MAX_TASKS 10

typedef void (*boardsystick_minischeduler_taskcallback_t)(void);
typedef struct stc_boardsystick_minischeduler_task
{
  boardsystick_minischeduler_taskcallback_t pfnCallback;
  uint32_t u32Interval;
  volatile uint32_t u32IntervalCount;
} stc_minischeduler_task_t;

#define BOARDTICK u32BoardMsTick;

extern volatile uint32_t u32BoardMsTick;
extern stc_board_uptime_t stcBoardUptime;

void BoardSystick_Init(void);
void BoardSystick_WaitMs(uint32_t u32Ms);
boolean_t BoardSystick_StartMeasure(volatile uint32_t* pu32Measure);
boolean_t BoardSystick_StopMeasure(volatile uint32_t* pu32Measure);
void BoardSystick_ConvertToUptime(uint32_t u32Measure, stc_board_uptime_t* pstcUptime);
void BoardSystick_AddTask(boardsystick_minischeduler_taskcallback_t pfnCallback, uint32_t u32Interval);
void BoardSystick_RemoveTask(boardsystick_minischeduler_taskcallback_t pfnCallback);
#endif /* BOARD_SYSTICK == ON */
#endif
