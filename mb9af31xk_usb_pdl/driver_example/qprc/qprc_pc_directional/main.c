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
 ** This example demonstrates directional count mode of position counter.
 ** A couple of wave from MFT is counted by a position counter.
 **
 ** History:
 **   - 2014-12-22  0.0.1  DHo        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*---------------------------------------------------------------------------*/
/* local defines                                                             */
/*---------------------------------------------------------------------------*/
// OCU channel
#define  USER_OCU_CH0         (MFT_OCU_CH0)
#define  USER_OCU_CH1         (MFT_OCU_CH1)
#define  USER_OCU_COUPLE_CH   (MFT_OCU_CH10)

// WFG channel
#define  USER_WFG_COUPLE_CH   (MFT_WFG_CH10)
// WFG IO 
/*Note: if below WFG io is conflict with QPRC I/O, please change*/
#define  InitWfgIo() {SetPinFunc_RTO00_0(); SetPinFunc_RTO01_0();}

// QPRC unit
#define  USER_QPRC            (QPRC0)

// QPRC IO 
#define InitQprcIo() {SetPinFunc_AIN0_0(); SetPinFunc_BIN0_0(); SetPinFunc_ZIN0_0();}

/*---------------------------------------------------------------------------*/
/* Local data                                                                */
/*---------------------------------------------------------------------------*/
/*! \brief OCU match count */
static uint8_t m_u8Ocu0IntFlag = 0;
static uint8_t m_u8Ocu1IntFlag = 0;
/*! \brief PC match interrupt flag */
static uint8_t m_u8PcMatchFlag = 0;
/*! \brief PC overflow interrupt flag */
static uint8_t m_u8PcOfFlag = 0;
/*! \brief PC underflow interrupt flag */
static uint8_t m_u8PcUfFlag = 0;

/*---------------------------------------------------------------------------*/
/* local functions prototypes                                                */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* global data                                                               */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* global functions                                                          */
/*---------------------------------------------------------------------------*/


/**
 ******************************************************************************
 ** \brief  OCU channel 0 interrupt callback function   .
 **
 ** \return none
 ******************************************************************************/
static void Ocu0IntCallback(void)
{
    if(m_u8Ocu0IntFlag)
    {
        Mft_Ocu_WriteOccp(&MFT0_OCU, USER_OCU_CH0, 10000);
    }
    else
    {
        Mft_Ocu_WriteOccp(&MFT0_OCU, USER_OCU_CH0, 30000);
    }
    m_u8Ocu0IntFlag = !m_u8Ocu0IntFlag;

    return;
}

/**
 ******************************************************************************
 ** \brief  OCU channel 1 interrupt callback function  .
 **
 ** \return none
 ******************************************************************************/
static void Ocu1IntCallback(void)
{
    if(m_u8Ocu1IntFlag)
    {
        Mft_Ocu_WriteOccp(&MFT0_OCU, USER_OCU_CH1, 20000);
    }
    else
    {
        Mft_Ocu_WriteOccp(&MFT0_OCU, USER_OCU_CH1, 40000);
    }
    m_u8Ocu1IntFlag = !m_u8Ocu1IntFlag;

    return;
}


/**
 ******************************************************************************
 ** \brief  Init user free run timer   .
 **
 ** \return none
 ******************************************************************************/
static void InitFrt(void)
{
    stc_mft_frt_config_t stcFrtConfig;

    // Clear structure
    PDL_ZERO_STRUCT(stcFrtConfig);
    
    //configure the FRT parameter
    stcFrtConfig.enFrtMode = FrtUpCount;
    stcFrtConfig.enFrtClockDiv = FrtPclkDiv8;
    stcFrtConfig.bEnExtClock = FALSE;
    stcFrtConfig.bEnBuffer = FALSE;

    Mft_Frt_Stop(&MFT0_FRT, MFT_FRT_CH0);
    //init frt module
    Mft_Frt_Init(&MFT0_FRT, MFT_FRT_CH0, &stcFrtConfig);
    //set frt cycle value
    Mft_Frt_SetCountCycle(&MFT0_FRT, MFT_FRT_CH0, 50000);

    return;
}

/**
 ******************************************************************************
 ** \brief  Init user output compare unit  .
 **
 ** \return none
 ******************************************************************************/
static void InitOcu(void)
{
    stc_mft_ocu_config_t stcOcuConfig0, stcOcuConfig1;

    // Clear structures
    PDL_ZERO_STRUCT(stcOcuConfig0);
    PDL_ZERO_STRUCT(stcOcuConfig1);
    
    // Configure the OCU parameter
    stcOcuConfig0.enFrtConnect = Frt0ToOcu;
    stcOcuConfig0.enPinState = RtLowLevel;
    stcOcuConfig0.enOccpBufMode = OccpBufDisable;
    stcOcuConfig0.bIrqEnable = TRUE;
    stcOcuConfig0.pfnIrqCallback = Ocu0IntCallback;
    stcOcuConfig0.bTouchNvic = TRUE;
    
    stcOcuConfig1.enFrtConnect = Frt0ToOcu;
    stcOcuConfig1.enPinState = RtLowLevel;
    stcOcuConfig1.enOccpBufMode = OccpBufDisable;
    stcOcuConfig1.bIrqEnable = TRUE;
    stcOcuConfig1.pfnIrqCallback = Ocu1IntCallback;
    stcOcuConfig1.bTouchNvic = TRUE;

    //init OCU
    Mft_Ocu_Init(&MFT0_OCU, USER_OCU_CH0, &stcOcuConfig0);
    Mft_Ocu_Init(&MFT0_OCU, USER_OCU_CH1, &stcOcuConfig1);

    //1 change mode
    Mft_Ocu_SetCompareMode_Fm3(&MFT0_OCU, USER_OCU_COUPLE_CH, OcuOdd1ChangeEven1Change);
    
    //write OCU compare value
    Mft_Ocu_WriteOccp(&MFT0_OCU, USER_OCU_CH0, 10000);
    Mft_Ocu_WriteOccp(&MFT0_OCU, USER_OCU_CH1, 20000);

    return;
}

/**
 ******************************************************************************
 ** \brief  Init User Waveform Generator Unit    .
 **
 ** \return none
 ******************************************************************************/
static void InitWfg(void)
{
    stc_wfg_config_t stcWfgConfig;
    
    // Clear structure
    PDL_ZERO_STRUCT(stcWfgConfig);
    
    // Configure WFG structure
    stcWfgConfig.enMode = WfgThroughMode;
    stcWfgConfig.enPselBits = PselBits00B;
    stcWfgConfig.enPgenBits = PgenBits00B;
    stcWfgConfig.enGtenBits = GtenBits00B;
    stcWfgConfig.enDmodBits = DmodBits00B;
    
    // Enable WFG I/O 
    InitWfgIo();
    
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
 **                          QPRC_PC_UNDERFLOW_INT
 **                          QPRC_PC_ZERO_INDEX_INT
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
    stc_qprc_config_t    stcQprcConfig;
    stc_qprc_irq_en_t   stcQprcIrqen;
    stc_qprc_irq_cb_t    stcQprcIrqCallback;

    /* Enable AIN, BIN function */
    InitQprcIo();
    
    // Clear structure
    PDL_ZERO_STRUCT(stcQprcConfig);
    PDL_ZERO_STRUCT(stcQprcIrqen);
    PDL_ZERO_STRUCT(stcQprcIrqCallback);
    
    /* Initialize interrupt */
    stcQprcIrqen.bQprcPcOfUfZeroIrq = TRUE;
    stcQprcIrqen.bQprcPcMatchIrq = TRUE;
    stcQprcIrqen.bQprcPcRcMatchIrq = FALSE;
    stcQprcIrqen.bQprcPcMatchRcMatchIrq = FALSE;
    stcQprcIrqen.bQprcPcCountInvertIrq = FALSE;
    stcQprcIrqen.bQprcRcOutrangeIrq = FALSE;

    stcQprcIrqCallback.pfnPcOfUfZeroIrqCb = PcUfOfZeroIrqHandler;
    stcQprcIrqCallback.pfnPcMatchIrqCb = PcMatchIrqHandler;
    stcQprcIrqCallback.pfnPcRcMatchIrqCb = NULL;
    stcQprcIrqCallback.pfnPcMatchRcMatchIrqCb = NULL;
    stcQprcIrqCallback.pfnPcCountInvertIrqCb = NULL;
    stcQprcIrqCallback.pfnRcOutrangeIrqCb = NULL;
    
    /*Initialize QPRC*/
    stcQprcConfig.bSwapAinBin   = FALSE; //TRUE: Swap AIN and BIN inputs
    stcQprcConfig.enCompareMode = QprcCompareWithRevolution;
    stcQprcConfig.enAinEdge = QprcAinRisingEdge;
    stcQprcConfig.enBinEdge = QprcBinRisingEdge;
    stcQprcConfig.enZinEdge = QprcZinDisable;
    #if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
    stcQprcConfig.stcAinFilter.bInputMask   = FALSE;
    stcQprcConfig.stcAinFilter.bInputInvert = FALSE;
    stcQprcConfig.stcAinFilter.enWidth      = QprcNoFilter;

    stcQprcConfig.stcBinFilter.bInputMask   = FALSE;
    stcQprcConfig.stcBinFilter.bInputInvert = FALSE;
    stcQprcConfig.stcBinFilter.enWidth      = QprcNoFilter;

    stcQprcConfig.stcCinFilter.bInputMask   = FALSE;
    stcQprcConfig.stcCinFilter.bInputInvert = FALSE;
    stcQprcConfig.stcCinFilter.enWidth      = QprcNoFilter;
    #endif
    stcQprcConfig.enPcResetMask = QprcResetMaskDisable; // Position counter reset mask
    stcQprcConfig.b8KValue      = FALSE; //TRUE: Outrange mode from 0 to 0x7FFF, FALSE: Outrange mode from 0 to 0xFFFF:
    stcQprcConfig.pstcIrqEn = &stcQprcIrqen;
    stcQprcConfig.pstcIrqCb = &stcQprcIrqCallback;
    stcQprcConfig.bTouchNvic = TRUE;

    Qprc_Init(&USER_QPRC, &stcQprcConfig);

    /* Disable PC & RC   */
    Qprc_ConfigPcMode(&USER_QPRC, QprcPcMode0); //Disable position counter
    Qprc_ConfigRcMode(&USER_QPRC, QprcRcMode0); //Disable revolution counter
    
    /* Set PC & RC compare value  */
    Qprc_SetPcCompareValue(&USER_QPRC, 100);
    Qprc_SetPcRcCompareValue(&USER_QPRC, 100);
    Qprc_SetPcMaxValue(&USER_QPRC, 200);

    /* Clear PC count */
    Qprc_SetPcCount(&USER_QPRC, 0);

    /* Set PC mode */
    Qprc_ConfigPcMode(&USER_QPRC, QprcPcMode3); //PC_Mode3: Directional count mode: Counts up/down with BIN active edge and AIN level

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
    printf("******************************************************\n");
    printf(" QPRC Position counter directional count mode \n");
    printf("******************************************************\n\n");

    printf("Trial 1 \n");
    printf("-----------\n");
    printf("Connect VCC with AIN \n\r");
    printf("Connect WFG I/O with BIN \n\r");
    printf("The wave inputs BIN when AIN is high, it will increase PC count \n\r");

    printf("Trial 2 \n");
    printf("-----------\n");
    printf("Connect GND with AIN \n\r");
    printf("Connect WFG I/O  with BIN \n\r");
    printf("The wave inputs BIN when AIN is low, it will increase PC count \n\r");
#endif

    InitWfg();
    InitOcu();
    InitFrt();
    InitQprc();

    /*Start OCU operation */
    Mft_Ocu_EnableOperation(&MFT0_OCU, USER_OCU_CH0);
    Mft_Ocu_EnableOperation(&MFT0_OCU, USER_OCU_CH1);

    /*Start FRT operation */
    Mft_Frt_Start(&MFT0_FRT, MFT_FRT_CH0);

    while(1)
    {
        if (0 != m_u8PcMatchFlag)
        {
        #ifdef DEBUG_PRINT
            printf("The count of Position Count matchs with the compare value! \n");
        #endif
            m_u8PcMatchFlag = 0;
        }

        if (0 != m_u8PcOfFlag)
        {
        #ifdef DEBUG_PRINT
            printf("The count of Position Count overflows! \n");
        #endif
            m_u8PcOfFlag = 0;
        }

        if (0 != m_u8PcUfFlag)
        {
        #ifdef DEBUG_PRINT
            printf("The count of Position Count underflows! \n");
        #endif
            m_u8PcUfFlag = 0;
        }
    }
}
/*****************************************************************************/
/* END OF FILE */
