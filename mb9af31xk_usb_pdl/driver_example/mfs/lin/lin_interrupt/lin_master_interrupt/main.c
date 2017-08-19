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
 ** This example demonstrates LIN master mode with interrupt mode,
 ** which must be used with LIN slave sample ("lin_slave_interrupt")
 ** A LIN transceiver must be used and make following connect before starting
 ** this example:
 ** ----------------------------------------------------------------------------
 ** LIN Master                                                         LIN slave
 **          TX  ------------------           ------------------   TX
 ** SIN1_1-------|                 | LIN Bus |                  |--------SIN1_1
 **          RX  | LIN transceiver |---------|   LIN transceiver|  RX
 ** SOT1_1-------|                 |         |                  |--------SOT1_1
 **              ------------------          -------------------
 ** To modify channel, follow the steps:
 ** - Change the definition of "LinCh1" to other channel
 ** - Change the IO pin definitions
 ** - Set the unused channel to "PDL_OFF", set the used channel to "PDL_ON"
 **   in the pdl_user.h
 **
 ** History:
 **   - 2014-03-14  1.0  Edison Zhang         First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
#define LIN_M_TX_S_RX_ID           0x30
#define LIN_M_RX_S_TX_ID           0x31

#define LinCh1                     &LIN1
#define InitLinIo()                {SetPinFunc_SIN1_1();SetPinFunc_SOT1_1(); }
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
    LinSeqBreak,            ///< Lin break filed transfer period
    LinSeqSync,             ///< Lin sync filed transfer period
    LinSeqId,               ///< Lin ID transfer period
    LinSeqData,             ///< Lin data transfer/receive period
    LinSeqChecksum,         ///< Lin checksum transfer/receive period
    LinSeqEnd,              ///< Lin frame end

}en_lin_seq_t;

/**
 ******************************************************************************
 ** \brief Lin mode
 ******************************************************************************/
typedef enum en_lin_mode
{
    LinTxMode,     ///< Lin TX mode
    LinRxMode,     ///< Lin RX mode

}en_lin_mode_t;

/**
 ******************************************************************************
 ** \brief Lin information structure
 ******************************************************************************/
typedef struct lin_info
{
    en_lin_seq_t enSeq;         ///< Lin sequency
    uint8_t u8Id;               ///< Lin frame ID
    en_lin_mode_t enMode;       ///< Lin Mode
    uint8_t* pu8TxData;         ///< Pointer to TX data
    uint8_t  u8TxSize;          ///< TX data size
    uint8_t* pu8RxData;         ///< Pointer to RX data
    uint8_t  u8RxSize;          ///< RX data size
    uint8_t  u8Cnt;             ///< Data transfer count
    boolean_t bErrorFlag;       ///< Error flag
    uint8_t  u8Checksum;        ///< Data checksum
    boolean_t bRwSyncFlag;      ///< Transfer and receive sync flag

}lin_info_t;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
static en_result_t CompareData(uint8_t* pBuf1, uint8_t* pBuf2, uint8_t u8Length);
static uint8_t LinCalChecksum(uint8_t *pu8Data, uint8_t u8Len);
static void LinTxIntCallback(void);
static void LinRxIntCallback(void);
static void LinBreakIntCallback(void);

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static uint8_t au8LinTxBuf[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
static uint8_t au8LinRxBuf[8];
static lin_info_t stcLinInfo;
/**
 ******************************************************************************
 ** \brief  Compare each data in the input two buffers
 **
 ** \param pBuf1 Pointer to buffer 1 to be compared with buffer 2
 ** \param pBuf2 Pointer to buffer 2 to be compared with buffer 1
 **
 ** \relval Ok     The data in buffer1 are same with that in buffer2
 ** \relval Error  The data in buffer1 are not same with that in buffer2
 ******************************************************************************/
static en_result_t CompareData(uint8_t* pBuf1, uint8_t* pBuf2, uint8_t u8Length)
{
    while(u8Length--)
    {
        if(*pBuf1++ != *pBuf2++)
        {
            return Error;
        }
    }

    return Ok;
}

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
 ** \brief Initialize LIN hardware
 ******************************************************************************/
void InitLinMaster(void)
{
    stc_mfs_lin_config_t stcLinConfig;
    stc_lin_irq_cb_t  stcLinIrqCb;

    PDL_ZERO_STRUCT(stcLinConfig);
    PDL_ZERO_STRUCT(stcLinIrqCb);

    /* Initialize LIN function I/O */
    InitLinIo();

    /* Initialize interrupt callback functions */
    stcLinIrqCb.pfnTxIrqCb = LinTxIntCallback;
    stcLinIrqCb.pfnRxIrqCb = LinRxIntCallback;
    stcLinIrqCb.pfnLinBreakIrqCb = LinBreakIntCallback;

    /* Initialize LIN configuration */
    stcLinConfig.enMsMode = LinMasterMode;
    stcLinConfig.u32BaudRate = 9600;
    stcLinConfig.enBreakLength = LinBreakLength13;
    stcLinConfig.enDelimiterLength = LinDelimiterLength1;
    stcLinConfig.enStopBits = LinOneStopBit;
    stcLinConfig.pstcFifoConfig = NULL;
    stcLinConfig.pstcIrqEn = NULL;
    stcLinConfig.pstcIrqCb = &stcLinIrqCb;
    stcLinConfig.bTouchNvic = TRUE;

    Mfs_Lin_Init(LinCh1, &stcLinConfig);
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
        case LinSeqSync:
            Mfs_Lin_SendData(LinCh1, 0x55u);
            stcLinInfo.bRwSyncFlag = FALSE;
            break;
        case LinSeqId:
            Mfs_Lin_SendData(LinCh1, stcLinInfo.u8Id);
            stcLinInfo.bRwSyncFlag = FALSE;

            if(stcLinInfo.enMode == LinRxMode)
            {
                Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
            }
            break;
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
    uint32_t u32Cnt = 200;

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
        case LinSeqSync:
            if(u8RdData != 0x55u)
            {
                stcLinInfo.bErrorFlag = 1;
                Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
                Mfs_Lin_DisableIrq(LinCh1, LinRxIrq);
                return;
            }
            stcLinInfo.bRwSyncFlag = TRUE;
            stcLinInfo.enSeq++;

            while(u32Cnt--); /* Wait some time for slave to calcuclate baudrate */

            break;
        case LinSeqId:
            if(u8RdData != stcLinInfo.u8Id)
            {
                stcLinInfo.bErrorFlag = 1;
                Mfs_Lin_DisableIrq(LinCh1, LinTxIrq);
                Mfs_Lin_DisableIrq(LinCh1, LinRxIrq);
                return;
            }
            stcLinInfo.bRwSyncFlag = TRUE;
            stcLinInfo.enSeq++;
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
                stcLinInfo.u8Checksum = LinCalChecksum((stcLinInfo.pu8RxData - stcLinInfo.u8RxSize), stcLinInfo.u8RxSize);
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
 ** \brief LIN break filed detection interrupt callback function
 ******************************************************************************/
static void LinBreakIntCallback(void)
{
    Mfs_Lin_EnableIrq(LinCh1, LinTxIrq);
    Mfs_Lin_EnableIrq(LinCh1, LinRxIrq);

    stcLinInfo.bRwSyncFlag = TRUE;
    stcLinInfo.enSeq = LinSeqSync;
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
en_result_t LinMasterTx(uint8_t u8Id, uint8_t *pu8Data, uint32_t u32Size)
{
    stcLinInfo.enSeq = LinSeqBreak;
    stcLinInfo.u8Id = u8Id;
    stcLinInfo.enMode = LinTxMode;
    stcLinInfo.pu8TxData = pu8Data;
    stcLinInfo.u8TxSize = u32Size;
    stcLinInfo.u8Cnt = 0;
    stcLinInfo.u8Checksum = LinCalChecksum(pu8Data, u32Size);

    /* Enable TX and RX function of LIN   */
    Mfs_Lin_EnableFunc(LinCh1, LinTx);
    Mfs_Lin_EnableFunc(LinCh1, LinRx);

    /* Generate LIN break field */
    Mfs_Lin_GenerateBreakField(LinCh1);

    /* Configure interrupt */
    Mfs_Lin_EnableIrq(LinCh1, LinBreakIrq);

    while(stcLinInfo.enSeq < LinSeqEnd)
    {
        if(stcLinInfo.bErrorFlag == 1)
        {
             /* Enable TX and RX function of LIN   */
            Mfs_Lin_DisableFunc(LinCh1, LinTx);
            Mfs_Lin_DisableFunc(LinCh1, LinRx);
            return Error;
        }
    }

     /* Enable TX and RX function of LIN   */
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
en_result_t LinMasterRx(uint8_t u8Id, uint8_t *pu8Data, uint32_t u32Size)
{
    stcLinInfo.enSeq = LinSeqBreak;
    stcLinInfo.u8Id = u8Id;
    stcLinInfo.enMode = LinRxMode;
    stcLinInfo.pu8RxData = pu8Data;
    stcLinInfo.u8RxSize = u32Size;
    stcLinInfo.u8Cnt = 0;
    stcLinInfo.u8Checksum = LinCalChecksum(pu8Data, u32Size);

    /* Enable TX and RX function of LIN   */
    Mfs_Lin_EnableFunc(LinCh1, LinTx);
    Mfs_Lin_EnableFunc(LinCh1, LinRx);

    /* Generate LIN break field */
    Mfs_Lin_GenerateBreakField(LinCh1);

    /* Configure interrupt */
    Mfs_Lin_EnableIrq(LinCh1, LinBreakIrq);

    while(stcLinInfo.enSeq < LinSeqEnd)
    {
        if(stcLinInfo.bErrorFlag == 1)
        {
             /* Enable TX and RX function of LIN   */
            Mfs_Lin_DisableFunc(LinCh1, LinTx);
            Mfs_Lin_DisableFunc(LinCh1, LinRx);
            return Error;
        }
    }

     /* Enable TX and RX function of LIN   */
    Mfs_Lin_DisableFunc(LinCh1, LinTx);
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
    uint8_t u8Flag = 1;
    InitLinMaster();

    while(1)
    {
        if(u8Flag)
        {
            if(Ok != LinMasterTx(LIN_M_TX_S_RX_ID, au8LinTxBuf, sizeof(au8LinTxBuf)))
            {
                while(1);
            }

            if(Ok != LinMasterRx(LIN_M_RX_S_TX_ID, au8LinRxBuf, sizeof(au8LinTxBuf)))
            {
                while(1);
            }

            if(Ok != CompareData(au8LinTxBuf, au8LinRxBuf, sizeof(au8LinTxBuf)))
            {
                while(1);
            }

            u8Flag--;
        }
    }
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
