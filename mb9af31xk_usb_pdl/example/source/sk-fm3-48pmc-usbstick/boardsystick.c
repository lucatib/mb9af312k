/* boardsystick.c - Source Code for Spansion's Evaluation Board Support Package */
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
/** \file boardsystick.c
 **
 ** for SK-FM3-48PMC-USBSTICK
 **
 ** History:
 **   - 2014-03-11  1.0  MSc  First Version
 *****************************************************************************/
#include "boardsystick.h"

#if BOARD_SYSTICK == ON
volatile uint32_t* pu32MeasureA;
volatile uint32_t* pu32MeasureB;
volatile uint32_t* pu32MeasureC;
volatile uint32_t* pu32MeasureD;
static stc_minischeduler_task_t pstcTasks[MINISCHEDULER_MAX_TASKS];

extern volatile uint32_t u32PixelActive;

volatile uint32_t u32BoardMsTick;
stc_board_uptime_t stcBoardUptime;

void BoardSystick_Init()
{
  stcBoardUptime.u32MSeconds = 0;
  stcBoardUptime.u32Seconds = 0;
  stcBoardUptime.u32Minutes = 0;
  stcBoardUptime.u32Hours = 0;
  stcBoardUptime.u32Days = 0;
  
  NVIC_ClearPendingIRQ(SysTick_IRQn); 
  NVIC_EnableIRQ(SysTick_IRQn);
  NVIC_SetPriority(SysTick_IRQn,5);
  SysTick_Config(__HCLK/100000);
}

boolean_t BoardSystick_StartMeasure(volatile uint32_t* pu32Measure)
{
    *pu32Measure = 0;
    if (pu32MeasureA == NULL)
    {
        pu32MeasureA = pu32Measure;
        return TRUE;
    }
    if (pu32MeasureB == NULL)
    {
        pu32MeasureB = pu32Measure;
        return TRUE;
    }
    if (pu32MeasureC == NULL)
    {
        pu32MeasureC = pu32Measure;
        return TRUE;
    }
    if (pu32MeasureD == NULL)
    {
        pu32MeasureD = pu32Measure;
        return TRUE;
    }
    return FALSE;
}
boolean_t BoardSystick_StopMeasure(volatile uint32_t* pu32Measure)
{
    if (pu32MeasureA == pu32Measure)
    {
        pu32MeasureA = NULL;
        return TRUE;
    }
    if (pu32MeasureB == pu32Measure)
    {
        pu32MeasureB = NULL;
        return TRUE;
    }
    if (pu32MeasureC == pu32Measure)
    {
        pu32MeasureC = NULL;
        return TRUE;
    }
    if (pu32MeasureD == pu32Measure)
    {
        pu32MeasureD = NULL;
        return TRUE;
    } 
    return FALSE;
}

void BoardSystick_ConvertToUptime(volatile uint32_t u32Measure, stc_board_uptime_t* pstcUptime)
{
    memset(pstcUptime,0,sizeof(pstcUptime[0]));
    pstcUptime->u32Seconds = u32Measure / 1000;
    pstcUptime->u32MSeconds = u32Measure % 1000;
    pstcUptime->u32Minutes = pstcUptime->u32Seconds / 60;
    pstcUptime->u32Seconds = pstcUptime->u32Seconds % 60;
    pstcUptime->u32Hours = pstcUptime->u32Minutes / 60;
    pstcUptime->u32Minutes = pstcUptime->u32Minutes % 60;
    pstcUptime->u32Days = pstcUptime->u32Hours / 24;
    pstcUptime->u32Hours = pstcUptime->u32Hours % 24;
}

void BoardSystick_AddTask(boardsystick_minischeduler_taskcallback_t pfnCallback, uint32_t u32Interval)
{
    volatile uint8_t i;
    for(i = 0;i < MINISCHEDULER_MAX_TASKS;i++)
    {
        if (pstcTasks[i].pfnCallback == pfnCallback)
        {
            break;
        }
        if (pstcTasks[i].pfnCallback == NULL)
        {
            pstcTasks[i].pfnCallback = pfnCallback;
            pstcTasks[i].u32Interval = u32Interval;
            pstcTasks[i].u32IntervalCount = u32Interval;
            break;
        }
    }
}

void BoardSystick_RemoveTask(boardsystick_minischeduler_taskcallback_t pfnCallback)
{
    volatile uint8_t i;
    for(i = 0;i < MINISCHEDULER_MAX_TASKS;i++)
    {
        if (pstcTasks[i].pfnCallback == pfnCallback)
        {
            pstcTasks[i].pfnCallback = NULL;
            break;
        }
    }
}


void SysTick_Handler (void) 
{
    static uint32_t u32MsTick = 100;
    uint8_t i;
    u32MsTick--;
    
    if (u32MsTick == 0)
    {
        if (pu32MeasureA != NULL) *pu32MeasureA = *pu32MeasureA + 1;
        if (pu32MeasureB != NULL) *pu32MeasureB = *pu32MeasureB + 1;
        if (pu32MeasureC != NULL) *pu32MeasureC = *pu32MeasureC + 1;
        if (pu32MeasureD != NULL) *pu32MeasureD = *pu32MeasureD + 1;
        u32MsTick = 100;
        u32BoardMsTick++;
        stcBoardUptime.u32MSeconds++;
        if (stcBoardUptime.u32MSeconds == 1000)
        {
            stcBoardUptime.u32MSeconds = 0;
            stcBoardUptime.u32Seconds++;
            if (stcBoardUptime.u32Seconds == 60)
            {
                stcBoardUptime.u32Seconds = 0;
                stcBoardUptime.u32Minutes++;
                if (stcBoardUptime.u32Minutes == 60)
                {
                    stcBoardUptime.u32Minutes = 0;
                    stcBoardUptime.u32Hours++;
                    if (stcBoardUptime.u32Hours == 24)
                    {
                        stcBoardUptime.u32Hours = 0;
                        stcBoardUptime.u32Days++;
                    }
                }
            }
        }
    }
    for(i = 0;i < MINISCHEDULER_MAX_TASKS;i++)
    {
        if (pstcTasks[i].pfnCallback != NULL)
        {
            pstcTasks[i].u32IntervalCount = pstcTasks[i].u32IntervalCount - 1;
            if (pstcTasks[i].u32IntervalCount == 0)
            {
                pstcTasks[i].pfnCallback();
                pstcTasks[i].u32IntervalCount = pstcTasks[i].u32Interval;
            }
        }
    }
}
void BoardSystick_WaitMs(uint32_t u32Ms)
{
    uint32_t u32Temp;
    while(u32Ms > 0)
    {
        u32Temp = stcBoardUptime.u32MSeconds;
        while(u32Temp == stcBoardUptime.u32MSeconds);
        u32Ms--;
    }
}
#endif /* BOARD_SYSTICK == ON */
