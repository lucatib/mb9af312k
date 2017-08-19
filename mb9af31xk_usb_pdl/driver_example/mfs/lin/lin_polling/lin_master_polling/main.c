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
 ** This example demonstrates LIN master mode with interrupt polling mode,
 ** which must be used with LIN slave sample ("lin_slave_polling")
 ** A LIN transceiver must be used and make following connect before starting
 ** this example:
 ** ----------------------------------------------------------------------------
 ** LIN Master                                                         LIN slave
 **          TX  ------------------           ------------------   TX
 ** SOT1_1-------|                 | LIN Bus |                  |--------SOT1_1
 **          RX  | LIN transceiver |---------|   LIN transceiver|  RX
 ** SIN1_1-------|                 |         |                  |--------SIN1_1
 **              ------------------          -------------------
 ** To modify channel, follow the steps:
 ** - Change the definition of "LinCh1" to other channel
 ** - Change the IO pin definitions
 ** - Set the unused channel to "PDL_OFF", set the used channel to "PDL_ON"
 **   in the pdl_user.h
 **
 ** History:
 **   - 2014-03-14  1.0  EZh         First version for FM uinversal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/
#define LIN_M_TX_S_RX_ID           0x30
#define LIN_M_RX_S_TX_ID           0x31

#define LinCh1                     &LIN1
#define InitLinIo()                {SetPinFunc_SIN1_1();SetPinFunc_SOT1_1(); }

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static uint8_t au8LinTxBuf[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
static uint8_t au8LinRxBuf[8];

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
uint8_t LinCalChecksum(uint8_t *pu8Data, uint8_t u8Len)
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
 ** \brief Write LIN hardware data register
 **
 ** \param pu8Data Poitner to data buffer
 ** \param u32Size   Data size
 **
 ** \relval Ok Data written OK
 ** \relval Error The data received is not same with the data sent
 ******************************************************************************/
en_result_t LinWriteData(uint8_t* pData, uint32_t u32Size)
{
    uint32_t u32i;
    uint8_t u8RdData;
    for(u32i=0; u32i<u32Size; u32i++)
    {
        while(Mfs_Lin_GetStatus(LinCh1, LinTxEmpty) != TRUE);
        Mfs_Lin_SendData(LinCh1, pData[u32i]);
        while(Mfs_Lin_GetStatus(LinCh1, LinRxFull) != TRUE);
        u8RdData = Mfs_Lin_ReceiveData(LinCh1);
        if(u8RdData != pData[u32i])
        {
            return Error;
        }
    }

    while(Mfs_Lin_GetStatus(LinCh1, LinTxIdle) != TRUE);

    return Ok;
}

/**
 ******************************************************************************
 ** \brief Read LIN hardware data register
 **
 ** \param pu8Data Poitner to data buffer
 ** \param u32Size   Data size
 **
 ** \relval Ok Data read OK
 ** \relval Error The data received normally
 ******************************************************************************/
en_result_t LinReadData(uint8_t* pData, uint32_t u32Size)
{
    uint32_t u32i;

    for(u32i=0; u32i<u32Size; u32i++)
    {
        while(Mfs_Lin_GetStatus(LinCh1, LinRxFull) != TRUE);
        *pData++ = Mfs_Lin_ReceiveData(LinCh1);
    }

    return Ok;
}

/**
 ******************************************************************************
 ** \brief Initialize LIN hardware
 ******************************************************************************/
void InitLin(void)
{
    stc_mfs_lin_config_t stcLinConfig;

    /* Initialize LIN function I/O */
    InitLinIo();

    /* Initialize LIN  */
    stcLinConfig.enMsMode = LinMasterMode;
    stcLinConfig.u32BaudRate = 9600;
    stcLinConfig.enBreakLength = LinBreakLength13;
    stcLinConfig.enDelimiterLength = LinDelimiterLength1;
    stcLinConfig.enStopBits = LinOneStopBit;
    stcLinConfig.pstcFifoConfig = NULL;

    Mfs_Lin_Init(LinCh1, &stcLinConfig);
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
    uint8_t u8Checksum;
    uint8_t u8Sync;

    /* Generate LIN break field */
    Mfs_Lin_GenerateBreakField(LinCh1);
    while(Mfs_Lin_GetStatus(LinCh1, LinBreakFlag) != TRUE);
    Mfs_Lin_ClrStatus(LinCh1, LinBreakFlag);

    /* Enable TX and RX function of LIN   */
    Mfs_Lin_EnableFunc(LinCh1, LinTx);
    Mfs_Lin_EnableFunc(LinCh1, LinRx);

    /* Send sync byte */
    u8Sync = 0x55;
    if(Ok != LinWriteData(&u8Sync, 1))
    {
        return Error;
    }

    /* Send ID */
    if(Ok != LinWriteData(&u8Id, 1))
    {
        return Error;
    }

    /* Send Data and checksum */
    if(Ok != LinWriteData(pu8Data, u32Size))
    {
        return Error;
    }

    u8Checksum = LinCalChecksum(pu8Data, u32Size);

    if(Ok != LinWriteData(&u8Checksum, 1))
    {
        return Error;
    }

    /* Enable TX function of UART0   */
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
    uint8_t u8Checksum, u8CalChecksum;
    uint8_t u8Sync;

    /* Generate LIN break field */
    Mfs_Lin_GenerateBreakField(LinCh1);
    while(Mfs_Lin_GetStatus(LinCh1, LinBreakFlag) != TRUE);

    Mfs_Lin_EnableFunc(LinCh1, LinTx);
    Mfs_Lin_EnableFunc(LinCh1, LinRx);

    /* Send sync byte */
    u8Sync = 0x55;
    if(Ok != LinWriteData(&u8Sync, 1))
    {
        return Error;
    }

    /* Send ID */
    if(Ok != LinWriteData(&u8Id, 1))
    {
        return Error;
    }

    Mfs_Lin_DisableFunc(LinCh1, LinTx);

    /* Read Data and checksum */
    if(Ok != LinReadData(pu8Data, u32Size))
    {
        return Error;
    }

    if(Ok != LinReadData(&u8Checksum, 1))
    {
        return Error;
    }

    u8CalChecksum = LinCalChecksum(pu8Data, u32Size);

    if(u8Checksum != u8CalChecksum)
    {
        return Error;
    }

    /* Enable TX function of UART0   */
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
    InitLin();

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
