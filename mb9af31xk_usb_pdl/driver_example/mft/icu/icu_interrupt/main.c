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
 ** This example demonstrates ICU capture function with interrupt. 
 ** A pulse generated by a GPIO is input to ICU input pin. 
 **
 ** History:
 **   - 2014-12-01  0.1.0  DHo        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/

// Check the availability of GPIO used in this example
#if !defined(SetPinFunc_IC02_0)
#error "IC02_0 pin is not available in this example, select other IC input pin or ICU channel!"
#endif

//FRT
#define  USER_MFT_FRT_UNIT          MFT0_FRT  
/*! \brief define Mft Frt Channel */
#define  USER_MFT_FRT_CH         MFT_FRT_CH2
/*! \brief define Mft Frt cycle value */
#define  USER_FRT_CYCLE_VAL     0xFFFF
#define  USER_FRT_MODE          FrtUpCount  

//ICU
#define  USER_MFT_ICU_UNIT       MFT0_ICU 
/*! \brief define Mft Frt Channel */
#define  USER_MFT_ICU_CH         MFT_ICU_CH2

//ICU IO
/* Initialize P3D */
#define InitWaveGenIo()    Gpio1pin_InitOut(GPIO1PIN_P3D, Gpio1pin_InitVal(0u))
#define WaveGenIoOut(v)    Gpio1pin_Put(GPIO1PIN_P3D, v)     

#define SetPinFunc_ICxx()  SetPinFunc_IC02_0(0)

/*---------------------------------------------------------------------------*/
/* local data                                                                */
/*---------------------------------------------------------------------------*/
/*! \brief Mft Frt Peak Interrupt count value */
static uint32_t m_u32CntFrtPeakIntTrg;
/*! \brief Mft Icu Interrupt count value */
static uint32_t m_u32CntIcuIntTrg;
/*! \brief Mft Icu captured rising edge number */
static uint32_t m_u32CntRisingEdgeTrg;
/*! \brief Mft Icu captured falling edge number */
static uint32_t m_u32CntFallingEdgeTrg;
/*! \brief Mft Icu captured edge type buffer */
static uint8_t m_au8EdgeTypeBuf[50];
/*! \brief Mft Icu captured data value buffer */
static uint32_t m_au32CapDataValBuf[50];
/*! \brief Mft Icu captured edge number */
static uint16_t m_u16CntCapEdgeNum;
static uint8_t m_u8IoStatus = 0xFF;
/*---------------------------------------------------------------------------*/
/* local functions prototypes                                                */
/*---------------------------------------------------------------------------*/ 
static void MftFrtPeakIntHandler(void);
static void MftIcuIntHandler(void);
#ifdef DEBUG_PRINT
static void MftIcuDebugPrint(void);
#endif
static void MftIcuResetParam(void);
static void PollingWaveGenIo(void);

static void Delay(void);
/*---------------------------------------------------------------------------*/
/* global data                                                               */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* global functions                                                          */
/*---------------------------------------------------------------------------*/

/*!
 ******************************************************************************
 ** \brief Mft Icu example code
 **
 ** 1. Set GPIO function to Icu
 ** 2. Configure Frt&Icu parameter
 ** 3. Enable Icu interrupt
 ** 4. Start Frt&Icu operation
 ** 5. Stop  Frt&Icu operation, disable interrupt
 ** 6. print Icu detected information
 ******************************************************************************
 */
int32_t main(void)
{
    stc_mft_frt_config_t stcFrtConfig;
    stc_frt_irq_sel_t stcFrtIrqSel;
    stc_frt_irq_cb_t stcFrtIrqCallBack;

    // Clear structures
    PDL_ZERO_STRUCT(stcFrtConfig);
    PDL_ZERO_STRUCT(stcFrtIrqSel);
    PDL_ZERO_STRUCT(stcFrtIrqCallBack);
      
    // Initialize Gpio to generate the pulse which inputs to ICU
    InitWaveGenIo();
   
    //configure the FRT parameter
    stcFrtConfig.enFrtMode = USER_FRT_MODE;
    stcFrtConfig.enFrtClockDiv = FrtPclkDiv256;
    stcFrtConfig.bEnExtClock = FALSE;
    stcFrtConfig.bEnBuffer = TRUE;
    stcFrtConfig.pstcIrqCb = &stcFrtIrqCallBack;
    stcFrtConfig.pstcIrqEn = &stcFrtIrqSel;
    stcFrtConfig.bTouchNvic = TRUE;

    //configure frt interrupt mode
    stcFrtIrqSel.bFrtPeakMatchIrq = TRUE; //enable peak match interrupt
    stcFrtIrqCallBack.pfnFrtPeakIrqCb = MftFrtPeakIntHandler;
    
    //initialize FRT
    Mft_Frt_Init(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH, &stcFrtConfig);
    
    // Set Frt count cycle
    Mft_Frt_SetCountCycle(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH, USER_FRT_CYCLE_VAL);
    
#ifdef DEBUG_PRINT 
    printf("==================================================\n");
    printf("Mft Icu Interrupt Example Program Start \n");
    printf("==================================================\n");
#endif

    //initialize ICU function pins
    SetPinFunc_ICxx(); 
    // Select Frt channel
    Mft_Icu_SelFrt(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, (en_mft_icu_frt_t)USER_MFT_FRT_CH);
    // Disable Icu operation
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, IcuDisable);
    // Enable Interrupt
    Mft_Icu_EnableIrq(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, MftIcuIntHandler, TRUE);

    // Initialize trig counters
    MftIcuResetParam();

#ifdef  DEBUG_PRINT
    printf("==================================================\n");
    printf("Icu Channel %d: Start\n", USER_MFT_ICU_CH);
    printf("Enable  Mft Icu Rising Edge Detection\n");
#endif
    // Start Icu operation, Set detecting rising edge mode
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, IcuRisingDetect);  

    // Start Frt operation
    Mft_Frt_Start(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);
    Delay();

    while(m_u32CntIcuIntTrg < 10)
    {
        PollingWaveGenIo();
        Delay();
    }

    // Stop FRT
    Mft_Frt_Stop(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);    
    Mft_Frt_SetCountVal(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH, 0x0000);
    // Disable Mft Icu Operation and set mode to IcuDisable
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH,IcuDisable);

#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Disable Mft Icu Rising Edge Detection\n");
    printf("==================================================\n");
    MftIcuDebugPrint();
#endif
    // Clear print counter buffer and related variables
    MftIcuResetParam();
    
/********************************************************************/
#ifdef  DEBUG_PRINT
    printf("==================================================\n");
    printf("Enable  Mft Icu Falling Edge Detection\n");
#endif
    // Start Icu operation, Set detecting falling edge mode
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, IcuFallingDetect);  

    // Start Frt operation
    Mft_Frt_Start(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);
    Delay();

    while(m_u32CntIcuIntTrg < 10)
    {
        PollingWaveGenIo();
        Delay();
    }

    // Stop FRT
    Mft_Frt_Stop(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);    
    Mft_Frt_SetCountVal(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH, 0x0000);
    // Disable Mft Icu Operation and set mode to IcuDisable
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH,IcuDisable);

#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Disable Mft Icu Falling Edge Detection\n");
    printf("==================================================\n");
    MftIcuDebugPrint();    
#endif
    // Clear print counter buffer and related variables
    MftIcuResetParam();
    
/********************************************************************/
#ifdef  DEBUG_PRINT
    printf("==================================================\n");
    printf("Enable  Mft Icu Both Edge Detection\n");
#endif
    
    // Start Icu operation, Set detecting both edge mode
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, IcuBothDetect);  

    // Start Frt operation
    Mft_Frt_Start(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);
    Delay();

    while(m_u32CntIcuIntTrg < 10)
    {
        PollingWaveGenIo();
        Delay();
    }

    // Stop FRT
    Mft_Frt_Stop(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);    
    Mft_Frt_SetCountVal(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH, 0x0000);
    // Disable Mft Icu Operation and set mode to IcuDisable
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH,IcuDisable);
    

#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Disable Mft Icu Both Edge Detection\n");
    printf("==================================================\n");
    MftIcuDebugPrint();    
#endif
    MftIcuResetParam();

#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("Icu Channel %d: Example Program End \n", USER_MFT_ICU_CH);
    printf("==================================================\n\n\n");
#endif

    while(1);
    
}

/*!
 ******************************************************************************
 ** \brief Mft Icu trigger interrupt handler
 ******************************************************************************
 */
static void MftIcuIntHandler(void)
{
    m_u32CntIcuIntTrg++;

    m_au8EdgeTypeBuf[m_u16CntCapEdgeNum] = Mft_Icu_GetLastEdge(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH);
    // this is just a demo
    // if absolute captured data value is needed
    // please add Frt counter value: m_u32CntFrtPeakIntTrg*65536
    m_au32CapDataValBuf[m_u16CntCapEdgeNum] = Mft_Icu_GetCaptureData(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH)
                                                    + m_u32CntFrtPeakIntTrg*0xFFFF;
    
    if(IcuFallingEdge == m_au8EdgeTypeBuf[m_u16CntCapEdgeNum])
    {
        m_u32CntFallingEdgeTrg++;
    }
    else if(IcuRisingEdge == m_au8EdgeTypeBuf[m_u16CntCapEdgeNum])
    {
        m_u32CntRisingEdgeTrg++;
    }
    m_u16CntCapEdgeNum++;

    return;
}

/*!
 ******************************************************************************
 ** \brief Mft Frt callback function
 ******************************************************************************
 */
static void MftFrtPeakIntHandler(void)
{
    m_u32CntFrtPeakIntTrg++;

    return;    
}

#ifdef DEBUG_PRINT
/*
 ******************************************************************************
 ** \brief Print debug information
 ******************************************************************************
 */
static void MftIcuDebugPrint(void)
{
    uint16_t u16CntPrint;
    
    printf("Interrupt request times:      %d\n", m_u32CntIcuIntTrg);
    printf("Mft Icu Rising  Edge trigger: %d\n", m_u32CntRisingEdgeTrg);
    printf("Mft Icu Falling Edge trigger: %d\n", m_u32CntFallingEdgeTrg);
    for(u16CntPrint = 0; u16CntPrint < 10; u16CntPrint++)
    {
        printf("Captured Edge Type : %d\n", m_au8EdgeTypeBuf[u16CntPrint]);
        printf("Captured Data Value: %d\n", m_au32CapDataValBuf[u16CntPrint]);
    }

    return;
}
#endif

/*
 ******************************************************************************
 ** \brief Reset Mft Icu parameter values
 ******************************************************************************
 */
static void MftIcuResetParam(void)
{
    uint16_t u16CntPrint;
    // Clear print counter buffer and related variables
    for(u16CntPrint = 0; u16CntPrint < 10; u16CntPrint++)
    {
        m_au8EdgeTypeBuf[u16CntPrint] = 0;
    }
    
    m_u32CntIcuIntTrg = 0;
    m_u16CntCapEdgeNum = 0;
    m_u16CntCapEdgeNum = 0;
    m_u32CntRisingEdgeTrg  = 0;
    m_u32CntFallingEdgeTrg = 0;
    m_u32CntFrtPeakIntTrg = 0;

    return;
}

/*
 ******************************************************************************
 ** \brief Polling a GPIO for ICU input
 ******************************************************************************
 */
static void PollingWaveGenIo(void)
{
    m_u8IoStatus = ~m_u8IoStatus;     
    WaveGenIoOut(m_u8IoStatus);

    return;
}

/*!
 ******************************************************************************
 ** \brief time delay function
 ******************************************************************************
 */
static void Delay(void)
{
    uint32_t u32Cnt = SystemCoreClock/4000;
    while (0 != (u32Cnt--));

    return;
}
/*****************************************************************************/
/* END OF FILE */
