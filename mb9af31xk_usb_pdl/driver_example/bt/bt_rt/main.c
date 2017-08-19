/*************************************************************************************
* Copyright (C) 2013-2015, Cypress Semiconductor Corporation. All rights reserved.    
*                                                                                     
* This software, including source code, documentation and related                     
* materials ( "Software" ), is owned by Cypress Semiconductor                         
* Corporation ( "Cypress" ) and is protected by and subject to worldwide              
* patent protection (United States and foreign), United States copyright              
* laws and international treaty provisions. Therefore, you may use this               
* Software only as provided in the license agreement accompanying the                 
* software package from which you obtained this Software ( "EULA" ).                  
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,             
* non-transferable license to copy, modify, and compile the                           
* Software source code solely for use in connection with Cypress's                    
* integrated circuit products. Any reproduction, modification, translation,           
* compilation, or representation of this Software except as specified                 
* above is prohibited without the express written permission of Cypress.              
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                                
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                                
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                        
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                                     
* PARTICULAR PURPOSE. Cypress reserves the right to make                              
* changes to the Software without notice. Cypress does not assume any                 
* liability arising out of the application or use of the Software or any              
* product or circuit described in the Software. Cypress does not                      
* authorize its products for use in any products where a malfunction or               
* failure of the Cypress product may reasonably be expected to result in              
* significant property damage, injury or death ( "High Risk Product" ). By            
* including Cypress's product in a High Risk Product, the manufacturer                
* of such system or application assumes all risk of such use and in doing             
* so agrees to indemnify Cypress against all liability.                               
*/
/******************************************************************************/
/** \file main.c
 **
 ** This example demonstrates both interrupt and polling access mode of Reload timer.
 **
 ** History:
 **   - 2013-05-20  0.0.1  Edison Zhang        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/
#define  USER_BT             BT2
#define  USER_TIMER_SIZE     RtSize32Bit

/*---------------------------------------------------------------------------*/
/* local datatypes                                                           */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* local data                                                                */
/*---------------------------------------------------------------------------*/
static uint32_t m_u32CntIrqTrg, m_u32CntIrqUnder;

/*---------------------------------------------------------------------------*/
/* local functions prototypes                                                */
/*---------------------------------------------------------------------------*/
static void RtUnderflowIrqHandler(void);
static void RtTrigIrqHandler(void);
static void Delay(void);
/*---------------------------------------------------------------------------*/
/* global data                                                               */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* global functions                                                          */
/*---------------------------------------------------------------------------*/
/*!
 ******************************************************************************
 ** \brief Reload timer example code
 **
 ** 1. Register initialization
 ** 3. Enable interrupt.
 ** 4. Enable count operation.
 ** 5. Start software trigger.
 ** 6. Disable interrupt.
 ** 7. ReStart software trigger.
 ** 8. Disable count operation.
 ******************************************************************************
 */
int32_t main(void)
{
    stc_bt_rt_config_t stcRtConfig;
    stc_rt_irq_en_t   stcIrqEn;
    stc_rt_irq_cb_t    stcIrqCb;
    uint32_t u32Cnt=0;
   
#ifdef DEBUG_PRINT 
    printf("==================================================\n");
    printf("Reload timer Example Program Start \n");
    printf("==================================================\n");
#endif
    // Clear structures
    PDL_ZERO_STRUCT(stcRtConfig);
    PDL_ZERO_STRUCT(stcIrqEn);
    PDL_ZERO_STRUCT(stcIrqCb);
    
    // Initialize interrupt
    stcRtConfig.pstcRtIrqEn = &stcIrqEn;
    stcRtConfig.pstcRtIrqCb = &stcIrqCb;
    // Set BT IO mode  
    Bt_ConfigIOMode(&USER_BT, BtIoMode0);
        
    // Initialize BT 
    stcRtConfig.enPres = RtPres1Div4; // BT_CLK_DIV_4: T = 0.2us @ PCLK = 20MHz
    stcRtConfig.enSize = USER_TIMER_SIZE;
    stcRtConfig.enMode = RtReload;
    stcRtConfig.enExtTrig = RtExtTiggerDisable;
    stcRtConfig.enOutputPolarity = RtPolarityLow;
    stcRtConfig.pstcRtIrqEn->bRtTrigIrq = 1;
    stcRtConfig.pstcRtIrqEn->bRtUnderflowIrq = 1;
    stcRtConfig.pstcRtIrqCb->pfnRtTrigIrqCb = RtTrigIrqHandler ;
    stcRtConfig.pstcRtIrqCb->pfnRtUnderflowIrqCb = RtUnderflowIrqHandler;
    stcRtConfig.bTouchNvic = TRUE;
    Bt_Rt_Init(&USER_BT, &stcRtConfig);
    
    // Write cycle value
    Bt_Rt_WriteCycleVal(&USER_BT, 19999);  // Cycle = T*(m+1)=4ms @ T = 0.2us
#ifdef DEBUG_PRINT 
    printf("Cycle: %d\n", 19999);
#endif     
    // Start Reload timer by software trigger    
    Bt_Rt_EnableCount(&USER_BT);
    Bt_Rt_EnableSwTrig(&USER_BT);
   
    // Delay some time
    Delay(); 
    
#ifdef DEBUG_PRINT 
    printf("Interrupt times:\n"); 
    printf("Trigger interrupt  : %d\n", m_u32CntIrqTrg);       
    printf("Underflow interrupt: %d\n", m_u32CntIrqUnder);  
#endif
    
    Bt_Rt_DisableIrq(&USER_BT, RtTrigIrq);
    Bt_Rt_DisableIrq(&USER_BT, RtUnderflowIrq);
 
#ifdef DEBUG_PRINT 
    printf("==================================================\n");
    printf("Disable Interrupt mode, waiting several seconds approximately\n");
#endif
    
    m_u32CntIrqTrg = 0;
    m_u32CntIrqUnder = 0;     
    
    // Delay some time
    Delay(); 
    
#ifdef DEBUG_PRINT 
    printf("Interrupt times:\n"); 
    printf("Trigger interrupt  : %d\n", m_u32CntIrqTrg);       
    printf("Underflow interrupt: %d\n", m_u32CntIrqUnder);  
#endif 
     
#ifdef DEBUG_PRINT 
    printf("==================================================\n");
    printf("Enable query mode, waiting several seconds approximately\n");
#endif
 
    m_u32CntIrqTrg = 0;
    m_u32CntIrqUnder = 0;   
    
    // Restart triggered by software 
    Bt_Rt_EnableSwTrig(&USER_BT);
    
    // Interrupt query mode 
    do 
    {   
        if (PdlSet == Bt_Rt_GetIrqFlag(&USER_BT, RtTrigIrq))
        {
            Bt_Rt_ClrIrqFlag(&USER_BT, RtTrigIrq);
            m_u32CntIrqTrg++;
        }
        else if (PdlSet == Bt_Rt_GetIrqFlag(&USER_BT, RtUnderflowIrq))
        {
            Bt_Rt_ClrIrqFlag(&USER_BT, RtUnderflowIrq);
            m_u32CntIrqUnder++;
        }
        u32Cnt++;
    } while(u32Cnt < 2500000ul);
    
    Bt_Rt_DisableCount(&USER_BT);
    
#ifdef DEBUG_PRINT
    printf("Interrupt request times:\n"); 
    printf("Trigger interrupt  : %d\n", m_u32CntIrqTrg);       
    printf("Underflow interrupt: %d\n", m_u32CntIrqUnder);
#endif   
      
#ifdef DEBUG_PRINT 
    printf("==================================================\n");
    printf("Reload timer Example Program End \n");
    printf("==================================================\n");
#endif
    
    while(1);
}
/*---------------------------------------------------------------------------*/
/* local functions                                                           */
/*---------------------------------------------------------------------------*/

/*
 ******************************************************************************
 ** \brief RT underflow interrupt handler
 ******************************************************************************
 */
static void RtUnderflowIrqHandler(void)
{
    m_u32CntIrqUnder++;
}

/*
 ******************************************************************************
 ** \brief RT trigger interrupt handler
 ******************************************************************************
 */
static void RtTrigIrqHandler(void)
{
    m_u32CntIrqTrg++;
}

/*
 ******************************************************************************
 ** \brief Delay requested secondes approximately
 ******************************************************************************
 */
static void Delay(void)
{
    int32_t u8Cnt1,u8Cnt2;
   
    u8Cnt2 = SystemCoreClock;
     
    u8Cnt1 = 0;
    while (1) 
    {
        u8Cnt1++;
        if (u8Cnt1 > u8Cnt2) 
        {
            break;
        }
    }   
}
