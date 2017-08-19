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
 ** This example demonstrates LIN slave mode with interrupt mode,
 ** which must be used with LIN master sample ("lin_master_polling")
 ** A LIN transceiver must be used and make following connection before starting
 ** this example:
 ** ----------------------------------------------------------------------------
 ** LIN Slave                                                         LIN master
 **          TX  ------------------           ------------------   TX
 ** SOT1_1-------|                 | LIN Bus |                  |--------SOT1_1
 **          RX  | LIN transceiver |---------|   LIN transceiver|  RX
 ** SIN1_1-------|                 |         |                  |--------SIN1_1
 **              ------------------          -------------------
 ** To modify channel, follow the steps:
 ** - Change the definition of "LinCh1" to other channel
 ** - Change the IO pin definitions
 ** - Change the definition "LIN_SYNC_ICU" to the ICU channel used.
 ** - Change ICU IO definition "SetPinFunc_IC01_LSYN1" to the ICU IO used.
 ** - Set the unused channel to "PDL_OFF", set the used channel to "PDL_ON"
 **   in the pdl_user.h
 **
 ** History:
 **   - 2014-11-14  1.0  EZh         First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
#include <string.h>

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
#define LIN_SYNC_ICU              (1u) //LIN ch.1's SYNC connected with ICU1

#define LIN_M_TX_S_RX_ID           0x30
#define LIN_M_RX_S_TX_ID           0x31

#define LinCh1                     &LIN1
#define InitLinIo()                {SetPinFunc_SIN1_1();SetPinFunc_SOT1_1();}

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM3_CORE)
#define LIN_FRT_DIV                FrtPclkDiv8	
#define FRT_PCLK_DIV               APBC1_PSR_Val
#define LIN_PCLK_DIV               APBC2_PSR_Val               
#else
#define LIN_FRT_DIV                FrtPclkDiv1	
#define FRT_PCLK_DIV               APBC1_PSR_Val
#define LIN_PCLK_DIV               APBC1_PSR_Val 	
#endif	

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/
/**
 ******************************************************************************
 ** \brief Lin sequence
 ******************************************************************************/
typedef enum en_lin_seq
{
    LinSeqBreak = 0u,            ///< Lin break filed handler process
    LinSeqSync,                  ///< Lin sync filed handler process
    LinSeqId,                    ///< Lin ID handler process
    LinSeqData,                  ///< Lin data handler process
    LinSeqChecksum,              ///< Lin checksum handler process
    LinSeqEnd,                   ///< Lin frame transfer end

}en_lin_seq_t;

/**
 ******************************************************************************
 ** \brief Lin mode
 ******************************************************************************/
typedef enum en_lin_mode
{
    LinTxMode,          ///< TX mode
    LinRxMode,          ///< RX mode

}en_lin_mode_t;

/**
 ******************************************************************************
 ** \brief Lin information structure
 ******************************************************************************/
typedef struct lin_info
{
    en_lin_seq_t enSeq;        ///< Lin sequence
    uint8_t u8Id;              ///< Lin frame ID
    en_lin_mode_t enMode;      ///< Lin mode
    uint8_t* pu8TxData;        ///< Pointer to TX buffer
    uint8_t  u8TxSize;         ///< TX size
    uint8_t* pu8RxData;        ///< Pointer to RX buffer
    uint8_t  u8RxSize;         ///< RX size
    uint8_t  u8Cnt;            ///< Data transfer count
    boolean_t bErrorFlag;      ///< Error flag
    uint8_t  u8Checksum;       ///< Lin frame check sum
    boolean_t bRwSyncFlag;     ///< Lin transfer and receive sync flag (in TX mode)

}lin_info_t;

/**
 ******************************************************************************
 ** \brief Lin sync field handler structure
 ******************************************************************************/
typedef struct lin_sync_info
{
    uint32_t u32a;         ///< 1st value
    uint32_t u32b;         ///< 2nd value
    uint32_t u32Pclk;      ///< MFS PCLK
    uint32_t u32bgr;       ///< BGR value
    uint32_t u32Baudrate;  ///< LIN baud rate
    uint8_t u8Cnt;         ///< Capture times count
    uint8_t u8Overflow;    ///< Overflow flag

}lin_sync_info_t;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
static uint8_t LinCalChecksum(uint8_t *pu8Data, uint8_t u8Len);
static void LinTxIntCallback(void);
static void LinRxIntCallback(void);
static void FrtPeakMatchIntCallback(void);
static void IcuIntCallback(void);
static void LinBreakIntCallback(void);

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static uint8_t au8LinTxBuf[8];
static uint8_t au8LinRxBuf[8];
static lin_info_t stcLinInfo;
static lin_sync_info_t stcLinSyncInfo;

/**
 ******************************************************************************
 ** \brief Calculate LIN data checksum
 **
 ** \param pu8Data Poitner to data buffer
 ** \param u8Len   Data length
 **
 ** \relval Data checksum
 ******************************************************************************/
static uint8_t LinCalChecksum(uint8_t *pu8Data, uint8_t u8Len)
{
    uint16_t u16Sum = 0;
    uint8_t u8i;
    for(u8i = 0; u8i < u8Len; u8i++)
    {
        u16Sum += pu8Data[u8i];
        if(u16Sum & 0xFF00)
        {
            u16Sum = (u16Sum & 0x00FF) + 1;
        }
    }
    u16Sum ^= 0x00FF;
    return (uint8_t)u16Sum;
}

/**
 ******************************************************************************
 ** \brief LIN TX interrupt callback function
 ******************************************************************************/
static void LinTxIntCallback(void)
{
    if(stcLinInfo.bRwSyncFlag != TRUE)
    {
        return;
    }

    switch (stcLinInfo.enSeq)
    {
        case LinSeqData:
            if(stcLinInfo.enMode == LinTxMode)
            {
                Mfs_Lin_SendData(LinCh1, *stcLinInfo.pu8TxData);
                stcLinInfo.bRwSyncFlag = FALSE;
            }
            break;
        case LinSeqChecksum:
            if(stcLinInfo.enMode == LinTxMode)
            {
                Mfs_Lin_SendData(LinCh1, stcLinInfo.u8Checksum);
                stcLinInfo.bRwSyncFlag = FALSE;

                Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
            }
            break;
        default:
            break;
    }

}

/**
 ******************************************************************************
 ** \brief LIN RX interrupt callback function
 ******************************************************************************/
static void LinRxIntCallback(void)
{
    uint8_t u8RdData;

    if((TRUE == Mfs_Lin_GetStatus(LinCh1, LinFrameError)) ||
       (TRUE == Mfs_Lin_GetStatus(LinCh1, LinOverrunError)))
    {
        stcLinInfo.bErrorFlag = 1;
        Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
        Mfs_Lin_DisableIrq(LinCh1, LinRxIrq);
        return;
    }

    u8RdData = Mfs_Lin_ReceiveData(LinCh1);

    switch (stcLinInfo.enSeq)
    {
        case LinSeqId:
            if(u8RdData != stcLinInfo.u8Id)
            {
                stcLinInfo.bErrorFlag = 1;
                Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
                Mfs_Lin_DisableIrq(LinCh1, LinRxIrq);
                return;
            }
            stcLinInfo.enSeq++;

            if(stcLinInfo.enMode == LinTxMode)
            {
                Mfs_Lin_EnableIrq(LinCh1, LinTxIrq);

                /* Enable TX function of LIN   */
                Mfs_Lin_EnableFunc(LinCh1, LinTx);

                stcLinInfo.bRwSyncFlag = TRUE;
            }

            break;
        case LinSeqData:
            if(stcLinInfo.enMode == LinTxMode)
            {
                if(u8RdData != *stcLinInfo.pu8TxData)
                {
                    stcLinInfo.bErrorFlag = 1;
                    Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
                    Mfs_Lin_DisableIrq(LinCh1, LinRxIrq);
                }

                stcLinInfo.u8Cnt++;
                stcLinInfo.pu8TxData++;
                stcLinInfo.bRwSyncFlag = TRUE;

                if(stcLinInfo.u8Cnt == stcLinInfo.u8TxSize)
                {
                    stcLinInfo.enSeq++;
                }
            }
            else
            {
                *stcLinInfo.pu8RxData = u8RdData;
                stcLinInfo.pu8RxData++;
                stcLinInfo.u8Cnt++;

                if(stcLinInfo.u8Cnt == stcLinInfo.u8RxSize)
                {
                    stcLinInfo.enSeq++;
                }
            }
            break;
        case LinSeqChecksum:
            if(stcLinInfo.enMode == LinTxMode)
            {
                if(u8RdData != stcLinInfo.u8Checksum)
                {
                    stcLinInfo.bErrorFlag = 1;
                    Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
                    Mfs_Lin_DisableIrq(LinCh1, LinRxIrq);
                    return;
                }

                Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
                Mfs_Lin_DisableIrq(LinCh1, LinRxIrq);

                stcLinInfo.enSeq = LinSeqEnd;
            }
            else
            {
                stcLinInfo.u8Checksum = LinCalChecksum((stcLinInfo.pu8RxData-stcLinInfo.u8RxSize), stcLinInfo.u8RxSize);
                if(stcLinInfo.u8Checksum  != u8RdData)
                {
                    stcLinInfo.bErrorFlag = 1;
                }
                else
                {
                    stcLinInfo.enSeq = LinSeqEnd;
                }

                Mfs_Lin_DisableIrq(LinCh1, LinRxIrq);
                break;
            }
            break;
        default:
            break;
    }

}

/**
 ******************************************************************************
 ** \brief FRT peak match interrupt callback function
 ******************************************************************************/
static void FrtPeakMatchIntCallback(void)
{
    stcLinSyncInfo.u8Overflow++;
}

/**
 ******************************************************************************
 ** \brief ICU interrupt callback function
 ******************************************************************************/
static void IcuIntCallback(void)
{
    uint8_t u8DivRate;
    if(stcLinSyncInfo.u8Cnt == 0u)
    {
        stcLinSyncInfo.u32a = Mft_Icu_GetCaptureData(&MFT0_ICU, LIN_SYNC_ICU);
        stcLinSyncInfo.u8Cnt++;
    }
    else if(stcLinSyncInfo.u8Cnt == 1u)
    {
        stcLinSyncInfo.u32b = Mft_Icu_GetCaptureData(&MFT0_ICU, LIN_SYNC_ICU);
        u8DivRate = ((1 << (LIN_PCLK_DIV & 0x03u)) * 8u)/((1 << (FRT_PCLK_DIV & 0x03u))*(1 << LIN_FRT_DIV));
        stcLinSyncInfo.u32bgr = ((50000+1)*stcLinSyncInfo.u8Overflow + stcLinSyncInfo.u32b - stcLinSyncInfo.u32a)/u8DivRate - 1;

        Mft_Frt_Stop(&MFT0_FRT, 0u);
        Mft_Icu_ConfigDetectMode(&MFT0_ICU, LIN_SYNC_ICU, IcuDisable);

        stcLinSyncInfo.u32Pclk = __HCLK/(1ul << (LIN_PCLK_DIV & 0x03u)); /* MFS is attached on APB1 bus */
        stcLinSyncInfo.u32Baudrate = stcLinSyncInfo.u32Pclk/(stcLinSyncInfo.u32bgr+1);
        Mfs_Lin_SetBaudRate(LinCh1, stcLinSyncInfo.u32Baudrate);

        /* Configure interrupt */
        Mfs_Lin_EnableIrq(LinCh1, LinRxIrq);

        /* Enable RX function of LIN   */
        Mfs_Lin_EnableFunc(LinCh1, LinRx);

        stcLinInfo.enSeq++;
    }
}

/**
 ******************************************************************************
 ** \brief LIN break field interrupt callback interrupt
 ******************************************************************************/
static void LinBreakIntCallback(void)
{
    /* Capture sync field and calculate baudrate */
    Mft_Icu_ConfigDetectMode(&MFT0_ICU, LIN_SYNC_ICU, IcuBothDetect);
    Mft_Frt_Start(&MFT0_FRT, 0u);

    PDL_ZERO_STRUCT(stcLinSyncInfo);

    stcLinInfo.enSeq++;
}


/**
 ******************************************************************************
 ** \brief Initialize LIN, FRT, ICU hardware
 ******************************************************************************/
void InitLinSlave(void)
{
    stc_mfs_lin_config_t stcLinConfig;
    stc_mft_frt_config_t stcFrtConfig;
    stc_lin_irq_cb_t  stcLinIrqCb;
    stc_frt_irq_cb_t  stcFrtIrqCb;
    stc_frt_irq_sel_t stcFrtSel;

    /* Clear structures */
    PDL_ZERO_STRUCT(stcLinConfig);
    PDL_ZERO_STRUCT(stcFrtConfig);
    PDL_ZERO_STRUCT(stcLinIrqCb);
    PDL_ZERO_STRUCT(stcFrtIrqCb);
    PDL_ZERO_STRUCT(stcFrtSel);

    /* Initialize LIN interrupt callback functions */
    stcLinIrqCb.pfnTxIrqCb = LinTxIntCallback;
    stcLinIrqCb.pfnRxIrqCb = LinRxIntCallback;
    stcLinIrqCb.pfnLinBreakIrqCb = LinBreakIntCallback;

    /* Initialize LIN function I/O */
    InitLinIo();

    /* Initialize LIN configuration */
    stcLinConfig.enMsMode = LinSlaveMode;
    stcLinConfig.u32BaudRate = 10000;   /* Set an initial value */
    stcLinConfig.enStopBits = LinOneStopBit;
    stcLinConfig.pstcFifoConfig = NULL;
    stcLinConfig.pstcIrqEn = NULL;
    stcLinConfig.pstcIrqCb = &stcLinIrqCb;
    stcLinConfig.bTouchNvic = TRUE;

    Mfs_Lin_Init(LinCh1, &stcLinConfig);

    /* Initialize FRT interrupt callback functions */
    stcFrtIrqCb.pfnFrtPeakIrqCb = FrtPeakMatchIntCallback;
    stcFrtIrqCb.pfnFrtZeroIrqCb = NULL;
    stcFrtSel.bFrtPeakMatchIrq = TRUE;
    stcFrtSel.bFrtZeroMatchIrq = FALSE;

    /* Initialize FRT */
    stcFrtConfig.enFrtMode = FrtUpCount;
    stcFrtConfig.enFrtClockDiv = LIN_FRT_DIV;
    stcFrtConfig.bEnExtClock = FALSE;
    stcFrtConfig.bEnBuffer = TRUE;
    stcFrtConfig.pstcIrqEn = &stcFrtSel;
    stcFrtConfig.pstcIrqCb = &stcFrtIrqCb;
    stcFrtConfig.bTouchNvic = TRUE;

    Mft_Frt_Stop(&MFT0_FRT ,0u);
    Mft_Frt_Init(&MFT0_FRT ,0u, &stcFrtConfig);
    Mft_Frt_SetCountCycle(&MFT0_FRT, 0u, 50000);

    /* Initialize ICU */
    //FM0P_GPIO->EPFR01_f.IC01S = 4u; // select LIN ch.1 LSYN to IC01
    SetPinFunc_IC01_LSYN1();

    Mft_Icu_SelFrt(&MFT0_ICU, LIN_SYNC_ICU, Frt0ToIcu);
    Mft_Icu_ConfigDetectMode(&MFT0_ICU, LIN_SYNC_ICU, IcuDisable);
    Mft_Icu_EnableIrq(&MFT0_ICU, LIN_SYNC_ICU, IcuIntCallback, TRUE);

}

/**
 ******************************************************************************
 ** \brief Transfer a LIN frame
 **
 ** \param u8Id    LIN frame ID
 ** \param pu8Data Poitner to data buffer
 ** \param u32Size Data size
 **
 ** \relval Ok Frame transfer OK
 ** \relval Error Frame transfer failed
 ******************************************************************************/
en_result_t LinSlaveTx(uint8_t u8Id, uint8_t *pu8Data, uint32_t u32Size)
{
    stcLinInfo.enSeq = LinSeqBreak;
    stcLinInfo.u8Id = u8Id;
    stcLinInfo.enMode = LinTxMode;
    stcLinInfo.pu8TxData = pu8Data;
    stcLinInfo.u8TxSize = u32Size;
    stcLinInfo.u8Cnt = 0;
    stcLinInfo.u8Checksum = LinCalChecksum(pu8Data, u32Size);

    /* Configure interrupt */
    Mfs_Lin_EnableIrq(LinCh1, LinBreakIrq);

    while(stcLinInfo.enSeq < LinSeqEnd)
    {
        if(stcLinInfo.bErrorFlag == 1)
        {
             /* Disable RX function of LIN   */
            Mfs_Lin_DisableFunc(LinCh1, LinTx);
            Mfs_Lin_DisableFunc(LinCh1, LinRx);
            return Error;
        }
    }

     /* Disable RX function of LIN   */
    Mfs_Lin_DisableFunc(LinCh1, LinTx);
    Mfs_Lin_DisableFunc(LinCh1, LinRx);

    return Ok;
}

/**
 ******************************************************************************
 ** \brief Receive a LIN frame
 **
 ** \param u8Id    LIN frame ID
 ** \param pu8Data Poitner to data buffer
 ** \param u32Size Data size
 **
 ** \relval Ok Frame receive OK
 ** \relval Error Frame receive failed
 ******************************************************************************/
en_result_t LinSlaveRx(uint8_t u8Id, uint8_t *pu8Data, uint32_t u32Size)
{
    stcLinInfo.u8Id = u8Id;
    stcLinInfo.u8Cnt = 0;
    stcLinInfo.pu8RxData = pu8Data;
    stcLinInfo.u8RxSize = u32Size;
    stcLinInfo.enMode = LinRxMode;
    stcLinInfo.enSeq = LinSeqBreak;

    /* Configure interrupt */
    Mfs_Lin_EnableIrq(LinCh1, LinBreakIrq);

    while(stcLinInfo.enSeq < LinSeqEnd)
    {
        if(stcLinInfo.bErrorFlag == 1)
        {
             /* Enable RX function of LIN   */
            Mfs_Lin_DisableFunc(LinCh1, LinRx);
            return Error;
        }
    }

     /* Disable RX function of LIN   */
    Mfs_Lin_DisableFunc(LinCh1, LinRx);

    return Ok;
}

/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    InitLinSlave();

    while(1)
    {
        if(Ok != LinSlaveRx(LIN_M_TX_S_RX_ID, au8LinRxBuf, sizeof(au8LinRxBuf)))
        {
            while(1);
        }

        memcpy(au8LinTxBuf, au8LinRxBuf, sizeof(au8LinRxBuf));

        if(Ok != LinSlaveTx(LIN_M_RX_S_TX_ID, au8LinTxBuf, sizeof(au8LinTxBuf)))
        {
            while(1);
        }
    }
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
