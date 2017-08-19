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
 ** This example demonstrates PC overflow or undowflow or ZIN trigger mode of 
 ** Revolution Counter.
 **
 ** History:
 **   - 2014-01-28  0.0.1  Edison Zhang        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/

/*------------------------- FRT ----------------------------*/
/*! \brief define Mft Frt Unit */
#define  USER_FRT             (MFT0_FRT)
/*! \brief define Mft Frt Channel */
#define  USER_FRT_CH          (MFT_FRT_CH0)

/*------------------------- Ocu ----------------------------*/
/*! \brief define Mft Ocu Unit */
#define  USER_OCU           (MFT0_OCU)
/*! \brief define Mft Ocu Channel */
#define  USER_OCU_CH0       (MFT_OCU_CH0)
#define  USER_OCU_CH1       (MFT_OCU_CH1)
#define  USER_OCU_COUPLE_CH (MFT_OCU_CH10)

#define  USER_OCU_CH0_CMPVAL1   (2000)  ///< OCU Ch0 compare value 1
#define  USER_OCU_CH0_CMPVAL2   (6000)  ///< OCU Ch0 compare value 2
#define  USER_OCU_CH1_CMPVAL1   (4000)  ///< OCU Ch1 compare value 1
#define  USER_OCU_CH1_CMPVAL2   (8000)  ///< OCU Ch1 compare value 2

/*------------------------ Wfg ----------------------------*/
/*! \brief define Mft Wfg Unit */
#define  USER_WFG             (MFT0_WFG)
/*! \brief define Mft Wfg Channel */
#define  USER_WFG_COUPLE_CH   (MFT_WFG_CH10)

/*------------------------ Qprc ---------------------------*/
/*! \brief define Qprc Unit */
#define  USER_QPRC            (QPRC0)
/*----------------------- GPIO -----------------------------*/
#if !defined(SetPinFunc_RTO00_0) || !defined(SetPinFunc_RTO01_0) || \
    !defined(SetPinFunc_AIN0_0) || !defined(SetPinFunc_BIN0_0) || !defined(SetPinFunc_ZIN0_0)
#error RTO00_0, RTO01_0, AIN0_0, BIN0_0 or ZIN0_0 pins are not available in this MCU product, \
change to other re-location pins or BT channels! Then delete "me" !
#endif 
#define InitMftIo()         {SetPinFunc_RTO00_0(); SetPinFunc_RTO01_0();}
#define InitQprcIo()        {SetPinFunc_AIN0_0();SetPinFunc_BIN0_0();SetPinFunc_ZIN0_0();}

/* Key */           
#if !defined(GPIO1PIN_P0F_INIT)
#error P0F is not available in this MCU product, \
change to other re-location pin! Then delete "me" !
#endif
#define InitKey()           Gpio1pin_InitIn(GPIO1PIN_P0F, Gpio1pin_InitDirectionInput)    
#define GetKey()            Gpio1pin_Get(GPIO1PIN_P0F) 

/*  ZIN trigger IO */
#if !defined(GPIO1PIN_P14_INIT)
#error P14 is not available in this MCU product, \
change to other re-location pin! Then delete "me" !
#endif
#define InitZinTriggerIo()  Gpio1pin_InitOut(GPIO1PIN_P14, Gpio1pin_InitDirectionOutput)
#define SetZinTriggerIo()   Gpio1pin_Put(GPIO1PIN_P14, 1)
#define ClrZinTriggerIo()   Gpio1pin_Put(GPIO1PIN_P14, 0)

/*---------------------------------------------------------------------------*/
/* Local data                                                                */
/*---------------------------------------------------------------------------*/
/*! \brief OCU match count */
static uint8_t  m_au8OcuMatchCnt[2] = {0};
/*! \brief PC overflow interrupt flag */
static uint8_t  m_u8PcOfFlag = 0;
/*! \brief PC underflow interrupt flag */
static uint8_t  m_u8PcUfFlag = 0;
/*! \brief PC underflow interrupt flag */
static uint8_t  m_u8PcMatchFlag = 0;
/*! \brief RC underflow interrupt flag */
static uint8_t  m_u8RcMatchFlag = 0;
/*! \brief RC comapre value */
static uint32_t m_u32RcCmpVal = 1;

/*---------------------------------------------------------------------------*/
/* local functions prototypes                                                */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* global data                                                               */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* global functions                                                          */
/*---------------------------------------------------------------------------*/

/*!
 ******************************************************************************
 ** \brief FRT initialization
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void InitFrt(void)
{
    stc_mft_frt_config_t stcFrtConfig;

    // Clear structure
    PDL_ZERO_STRUCT(stcFrtConfig);
    
    //configure the FRT parameter
    stcFrtConfig.enFrtMode = FrtUpCount;
    stcFrtConfig.enFrtClockDiv = FrtPclkDiv64;
    stcFrtConfig.bEnExtClock = FALSE;
    stcFrtConfig.bEnBuffer = FALSE;

    Mft_Frt_Stop(&MFT0_FRT, MFT_FRT_CH0);
    //init frt module
    Mft_Frt_Init(&MFT0_FRT, MFT_FRT_CH0, &stcFrtConfig);
    //set frt cycle value
    Mft_Frt_SetCountCycle(&MFT0_FRT, MFT_FRT_CH0, 10000);
}
/*!
 ******************************************************************************
 ** \brief OCU Ch0 interrupt callback
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void Ocu0IrqCallback(void)
{
    if (0 == m_au8OcuMatchCnt[0])
    {
        Mft_Ocu_WriteOccp(&USER_OCU, USER_OCU_CH0, USER_OCU_CH0_CMPVAL2);
    }
    else
    {
        Mft_Ocu_WriteOccp(&USER_OCU, USER_OCU_CH0, USER_OCU_CH0_CMPVAL1);
    }

    m_au8OcuMatchCnt[0] = !m_au8OcuMatchCnt[0];

    return;
}

/*!
 ******************************************************************************
 ** \brief OCU Ch1 interrupt callback
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void Ocu1IrqCallback(void)
{
    if (0 == m_au8OcuMatchCnt[1])
    {
        Mft_Ocu_WriteOccp(&USER_OCU, USER_OCU_CH1, USER_OCU_CH1_CMPVAL2);
    }
    else
    {
        Mft_Ocu_WriteOccp(&USER_OCU, USER_OCU_CH1, USER_OCU_CH1_CMPVAL1);
    }

    m_au8OcuMatchCnt[1] = !m_au8OcuMatchCnt[1];

    return;
}

/*!
 ******************************************************************************
 ** \brief OCU initialization
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void InitOcu(void)
{
    stc_mft_ocu_config_t stcOcuConfig0, stcOcuConfig1;
    
    // Clear structures
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    /*configure the OCU parameter*/
    stcOcuConfig0.enFrtConnect = Frt0ToOcu;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufDisable;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu0IrqCallback;
    stcOcuConfig0.bTouchNvic = TRUE;

    stcOcuConfig1.enFrtConnect = Frt0ToOcu;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufDisable;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu1IrqCallback;
    stcOcuConfig1.bTouchNvic = TRUE;

    //init OCU
    Mft_Ocu_Init(&USER_OCU,USER_OCU_CH0,&stcOcuConfig0);
    Mft_Ocu_Init(&USER_OCU,USER_OCU_CH1,&stcOcuConfig1);
    
    // Set comapre mode
    Mft_Ocu_SetCompareMode_Fm3(&USER_OCU, USER_OCU_COUPLE_CH, OcuOdd1ChangeEven1Change);
    
    
    //write OCU compare value
    Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH0,USER_OCU_CH0_CMPVAL1);
    Mft_Ocu_WriteOccp(&USER_OCU,USER_OCU_CH1,USER_OCU_CH1_CMPVAL1);
}

/*!
 ******************************************************************************
 ** \brief WFG initialization
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void InitWfg(void)
{
    stc_wfg_config_t stcWfgConfig;
    
    // Clear structure
    PDL_ZERO_STRUCT(stcWfgConfig);
    
    InitMftIo();
    // Configure WFG structure
    stcWfgConfig.enMode = WfgThroughMode;
    stcWfgConfig.enPselBits = PselBits00B;
    stcWfgConfig.enPgenBits = PgenBits00B;
    stcWfgConfig.enGtenBits = GtenBits00B;
    stcWfgConfig.enDmodBits = DmodBits00B;

    // Config WFG module
    Mft_Wfg_Init(&MFT0_WFG, USER_WFG_COUPLE_CH, &stcWfgConfig);

    return;
}

/*!
 ******************************************************************************
 ** \brief Pc match interrupt handler
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void PcMatchIrqHandler(void)
{
    m_u8PcMatchFlag = 1;

    return;
}

/*!
 ******************************************************************************
 ** \brief  PC status interrupt function
 **
 ** \param [in]  u8IntType   QPRC_PC_OVERFLOW_INT
 **                                   QPRC_PC_UNDERFLOW_INT
 **                                   QPRC_PC_ZERO_INDEX_INT
 ** \retval None
 **
 ******************************************************************************
 */
static void PcUfOfZeroIrqHandler(uint8_t u8IntType)
{
    if (QPRC_PC_OVERFLOW_INT == u8IntType)
    {
        m_u8PcOfFlag = 1;
    }
    else if (QPRC_PC_UNDERFLOW_INT == u8IntType)
    {
        m_u8PcUfFlag = 1;
    }

    return;
}

/*!
 ******************************************************************************
 ** \brief  RC Match interrupt function
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void RcMatchIrqHandler(void)
{
    m_u8RcMatchFlag = 1;
    m_u32RcCmpVal++;
    Qprc_SetPcRcCompareValue(&USER_QPRC, m_u32RcCmpVal);

    return;
}

/*!
 ******************************************************************************
 ** \brief QPRC initialization
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void InitQprc(void)
{
    stc_qprc_config_t stcInitQprc;
    stc_qprc_irq_en_t stcQprcIrqEn;
    stc_qprc_irq_cb_t stcQprcIrqCb;
    
    /* Enable AIN, BIN, ZIN */
    InitQprcIo();
 
    // Clear structures
    PDL_ZERO_STRUCT(stcInitQprc);
    PDL_ZERO_STRUCT(stcQprcIrqEn);
    PDL_ZERO_STRUCT(stcQprcIrqCb);
    
    /* Enable PcMatch && Overflow, undowflow, zero match interrupt */
    stcQprcIrqEn.bQprcPcOfUfZeroIrq = TRUE; //Enable Overflow, undowflow, zero match interrupt
    stcQprcIrqEn.bQprcPcMatchIrq    = TRUE; //Enable PcMatch interrupt
    stcQprcIrqEn.bQprcPcRcMatchIrq  = TRUE;
    stcQprcIrqEn.bQprcPcMatchRcMatchIrq = FALSE;
    stcQprcIrqEn.bQprcPcCountInvertIrq  = FALSE;
    stcQprcIrqEn.bQprcRcOutrangeIrq = FALSE;
    
    stcQprcIrqCb.pfnPcOfUfZeroIrqCb = PcUfOfZeroIrqHandler; //Overflow, undowflow, zero match interrupt callback function
    stcQprcIrqCb.pfnPcMatchIrqCb = PcMatchIrqHandler;    //PC match interrupt callback function
    stcQprcIrqCb.pfnPcRcMatchIrqCb = RcMatchIrqHandler;
    stcQprcIrqCb.pfnPcMatchRcMatchIrqCb = NULL;
    stcQprcIrqCb.pfnPcCountInvertIrqCb = NULL;
    stcQprcIrqCb.pfnRcOutrangeIrqCb = NULL;
    
    /*Initialize QPRC*/
    stcInitQprc.bSwapAinBin   = FALSE; //TRUE: Swap AIN and BIN inputs
    stcInitQprc.enCompareMode = QprcCompareWithRevolution;
    
    stcInitQprc.enAinEdge = QprcAinRisingEdge;
    stcInitQprc.enBinEdge = QprcBinRisingEdge;
    stcInitQprc.enZinEdge = QprcZinFallingEdge;
    
    #if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
    stcInitQprc.stcAinFilter.bInputMask   = FALSE; //No input mask setting
    stcInitQprc.stcAinFilter.bInputInvert = FALSE; //No invert mask setting
    stcInitQprc.stcAinFilter.enWidth      = QprcNoFilter; //No filter

    stcInitQprc.stcBinFilter.bInputMask   = FALSE;
    stcInitQprc.stcBinFilter.bInputInvert = FALSE;
    stcInitQprc.stcBinFilter.enWidth      = QprcNoFilter;

    stcInitQprc.stcCinFilter.bInputMask   = FALSE;
    stcInitQprc.stcCinFilter.bInputInvert = FALSE;
    stcInitQprc.stcCinFilter.enWidth      = QprcNoFilter;
    #endif
    
    stcInitQprc.enPcResetMask = QprcResetMaskDisable; // Position counter reset mask
    stcInitQprc.b8KValue      = FALSE; //TRUE: Outrange mode from 0 to 0x7FFF, FALSE: Outrange mode from 0 to 0xFFFF:      
    stcInitQprc.pstcIrqEn = &stcQprcIrqEn;
    stcInitQprc.pstcIrqCb = &stcQprcIrqCb;
    stcInitQprc.bTouchNvic = TRUE;
    Qprc_Init(&USER_QPRC,&stcInitQprc);
    
    /* Disable position counter &&  revolution counter*/
    Qprc_ConfigPcMode(&USER_QPRC, QprcPcMode0); // PC_Mode0:Disable position counter
    Qprc_ConfigRcMode(&USER_QPRC, QprcRcMode0); // RC_Mode0: Disable revolution counter

    /* Set Value */
    Qprc_SetPcCompareValue(&USER_QPRC, 100);
    Qprc_SetPcRcCompareValue(&USER_QPRC, m_u32RcCmpVal);
    Qprc_SetPcMaxValue(&USER_QPRC, 200);
    
    /* Clear PC count */
    Qprc_SetPcCount(&USER_QPRC, 0);
    Qprc_ClrIrqFlag(&USER_QPRC, QprcPcOfUfZeroIrq);
    Qprc_ClrIrqFlag(&USER_QPRC, QprcPcMatchIrq);
    Qprc_ClrIrqFlag(&USER_QPRC, QprcPcRcMatchIrq);
    Qprc_ClrIrqFlag(&USER_QPRC, QprcPcMatchRcMatchIrq);
    Qprc_ClrIrqFlag(&USER_QPRC, QprcPcCountInvertIrq);
    Qprc_ClrIrqFlag(&USER_QPRC, QprcRcOutrangeIrq);
        
    /* Set PC &&RC mode */
    Qprc_ConfigPcMode(&USER_QPRC, QprcPcMode1);// PC_Mode1: Increments with AIN active edge and decrements with BIN active edge
    Qprc_ConfigRcMode(&USER_QPRC, QprcRcMode3);// RC_Mode3: Up/down count of revolution counter on overflow or underflow in position count match and ZIN active edge
    return;
}
/*!
 ******************************************************************************
 ** \brief Delay time
 **
 ** \param  [in]   u32Cnt
 **
 ** \retval   None
 **
 ******************************************************************************
 */
static void Delay(uint32_t u32Cnt)
{
    uint32_t i = 0;
    for(; u32Cnt; u32Cnt--)
        for(i=1000; i; i--);

    return;
}

/*!
 ******************************************************************************
 ** \brief ZIN trigger initialization
 **
 ** \param  None
 **
 ** \retval   None
 **
 ******************************************************************************
 */
static void InitZinTrig(void)
{
    InitZinTriggerIo();
    ClrZinTriggerIo();

    return;
}

/*!
 ******************************************************************************
 ** \brief ZIN trigger signal generation
 **
 ** \param  None
 **
 ** \retval   None
 **
 ******************************************************************************
*/
static void ZinTrigGen(void)
{
    Delay(2000);
    SetZinTriggerIo();
    Delay(2000);
    ClrZinTriggerIo();

    return;
}

/*!
 ******************************************************************************
 ** \brief QPRC example code
 **
  ** \return uint32_t return value, if needed
 ******************************************************************************
 */
int32_t main(void)
{
#ifdef DEBUG_PRINT
    /* Initialize UART printf function (printf via UART0) */
    Uart_Io_Init();
#endif
     
#ifdef DEBUG_PRINT
    printf("******************************************************************************************\n\r");
    printf("*    QPRC Revolution Counter example with ZIN or PC overflow or underflow trigger mode   *\n\r");
    printf("******************************************************************************************\n\n\r");

    printf("Connect RTO00_0 with AIN0_0, and ZIN0_0 with Key\n\r");
    printf("If active edge of ZIN is detected or PC count overflows, PC count clears to 0 and RC count adds by 1 \n\n\r");
#endif

    InitWfg();
    InitOcu();
    InitFrt();
    InitKey();
    InitZinTrig();
    InitQprc();

    /*Start FRT operation */
    Mft_Frt_Start(&USER_FRT, USER_FRT_CH);

    /*Start OCU operation */
    Mft_Ocu_EnableOperation(&USER_OCU, USER_OCU_CH0);
    Mft_Ocu_EnableOperation(&USER_OCU, USER_OCU_CH1);

    while(1)
    {
        if (0 != m_u8PcMatchFlag)
        {
        #ifdef DEBUG_PRINT
            printf("The count of Position Counter matchs with compare value!\n");      
        #endif          
            m_u8PcMatchFlag = 0;
        }

        if (0 != m_u8PcOfFlag)
        {
        #ifdef DEBUG_PRINT
            printf("The count of Position Counter overflows!\n");      
        #endif   
            m_u8PcOfFlag = 0;
        }

        if (0 != m_u8PcUfFlag)
        {
        #ifdef DEBUG_PRINT
            printf("The count of Position Counter underflows!\n");      
        #endif    
            m_u8PcUfFlag = 0;
        }

        if (0 != m_u8RcMatchFlag)
        {
        #ifdef DEBUG_PRINT
            printf("The count of Revolution Counter matchs!\n");      
        #endif  
            m_u8RcMatchFlag = 0;
        }

        if (FALSE == GetKey())
        {
            ZinTrigGen();
        }
    }
}
/*****************************************************************************/
/* END OF FILE */
