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
 ** This example demonstrates LIN slave mode with interrupt polling mode,
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
 **   - 2014-11-14  1.0  EZh         First version for FM uinversal PDL.
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
/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
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
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static uint8_t au8LinTxBuf[8];
static uint8_t au8LinRxBuf[8];
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
 ** \brief Initialize LIN, FRT, ICU hardware
 ******************************************************************************/
void InitLin(void)
{
    stc_mfs_lin_config_t stcLinConfig;
    stc_mft_frt_config_t stcFrtConfig;

    PDL_ZERO_STRUCT(stcLinConfig);
    PDL_ZERO_STRUCT(stcFrtConfig);

    /* Initialize LIN function I/O */
    InitLinIo();

    /* Initialize LIN  */
    stcLinConfig.enMsMode = LinSlaveMode;
    stcLinConfig.u32BaudRate = 10000;   /* Set an initial value */
    stcLinConfig.enStopBits = LinOneStopBit;
    stcLinConfig.pstcFifoConfig = FALSE;

    Mfs_Lin_Init(LinCh1, &stcLinConfig);

    /* Initialize FRT */
    stcFrtConfig.enFrtMode = FrtUpCount;
    stcFrtConfig.enFrtClockDiv = LIN_FRT_DIV;
    stcFrtConfig.bEnExtClock = FALSE;
    stcFrtConfig.bEnBuffer = TRUE;

    Mft_Frt_Stop(&MFT0_FRT ,0u);
    Mft_Frt_Init(&MFT0_FRT ,0u, &stcFrtConfig);
    Mft_Frt_SetCountCycle(&MFT0_FRT, 0u, 50000);

    /* Initialize ICU */
    SetPinFunc_IC01_LSYN1(); // select LIN ch.1 LSYN to IC01

    Mft_Icu_SelFrt(&MFT0_ICU, LIN_SYNC_ICU, Frt0ToIcu);
    Mft_Icu_ConfigDetectMode(&MFT0_ICU, LIN_SYNC_ICU, IcuDisable);

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
    uint8_t u8Checksum, u8DivRate;
    uint8_t u8IcuFlag = 0;
    uint32_t u32a, u32b, u32bgr, u32bauddate, u32Pclk1, u32Overflow = 0;

    /* Wait LIN break field */
    while(Mfs_Lin_GetStatus(LinCh1, LinBreakFlag) != TRUE);
    Mfs_Lin_ClrStatus(LinCh1, LinBreakFlag);

    /* Capture sync field and calculate baudrate */
    Mft_Icu_ConfigDetectMode(&MFT0_ICU, LIN_SYNC_ICU, IcuBothDetect);
    Mft_Frt_Start(&MFT0_FRT, 0u);

    while(1)
    {
        if(Mft_Frt_GetIrqFlag(&MFT0_FRT, 0u, enFrtPeakMatchIrq) == TRUE)
        {
            Mft_Frt_ClrIrqFlag(&MFT0_FRT, 0u, enFrtPeakMatchIrq);
            u32Overflow++;
        }
        if(Mft_Icu_GetIrqFlag(&MFT0_ICU, LIN_SYNC_ICU) == TRUE)
        {
            Mft_Icu_ClrIrqFlag(&MFT0_ICU, LIN_SYNC_ICU);
            u8IcuFlag++;

            if(u8IcuFlag == 1)
            {
                u32a = Mft_Icu_GetCaptureData(&MFT0_ICU, LIN_SYNC_ICU);
            }

            if(u8IcuFlag == 2)
            {
                u32b = Mft_Icu_GetCaptureData(&MFT0_ICU, LIN_SYNC_ICU);
                u8DivRate = ((1 << (LIN_PCLK_DIV & 0x03u)) * 8u)/((1 << (FRT_PCLK_DIV & 0x03u))*(1 << LIN_FRT_DIV));
                u32bgr = ((50000+1)*u32Overflow + u32b - u32a)/u8DivRate - 1;
                Mft_Frt_Stop(&MFT0_FRT, 0u);
                Mft_Icu_ConfigDetectMode(&MFT0_ICU, LIN_SYNC_ICU, IcuDisable);
                break;
            }
        }
    }
    u32Pclk1 = __HCLK/(1ul << (LIN_PCLK_DIV & 0x03u)); /* MFS is attached on APB1 bus */
    u32bauddate = u32Pclk1/(u32bgr+1);
    Mfs_Lin_SetBaudRate(LinCh1, u32bauddate);

    Mfs_Lin_EnableFunc(LinCh1, LinRx);

    /* Receive ID */
    if(Ok != LinReadData(&u8Id, 1))
    {
        return Error;
    }

    Mfs_Lin_EnableFunc(LinCh1, LinTx);
    if(LIN_M_RX_S_TX_ID == u8Id)
    {
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
    }
    else
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
en_result_t LinSlaveRx(uint8_t u8Id, uint8_t *pu8Data, uint32_t u32Size)
{
    uint8_t u8Checksum, u8CalChecksum, u8DivRate;
    uint8_t u8IcuFlag = 0;
    uint8_t u8ReceiveId;
    uint32_t u32a, u32b, u32bgr, u32bauddate, u32Pclk1, u32Overflow = 0;

    /* Wait LIN break field */
    while(Mfs_Lin_GetStatus(LinCh1, LinBreakFlag) != TRUE);
    Mfs_Lin_ClrStatus(LinCh1, LinBreakFlag);

    /* Capture sync field and calculate baudrate */
    Mft_Icu_ConfigDetectMode(&MFT0_ICU, LIN_SYNC_ICU, IcuBothDetect);
    Mft_Frt_Start(&MFT0_FRT, 0u);

    while(1)
    {
        if(Mft_Frt_GetIrqFlag(&MFT0_FRT, 0u, enFrtPeakMatchIrq) == TRUE)
        {
            Mft_Frt_ClrIrqFlag(&MFT0_FRT, 0u, enFrtPeakMatchIrq);
            u32Overflow++;
        }
        if(Mft_Icu_GetIrqFlag(&MFT0_ICU, LIN_SYNC_ICU) == TRUE)
        {
            Mft_Icu_ClrIrqFlag(&MFT0_ICU, LIN_SYNC_ICU);
            u8IcuFlag++;

            if(u8IcuFlag == 1)
            {
                u32a = Mft_Icu_GetCaptureData(&MFT0_ICU, LIN_SYNC_ICU);
            }

            if(u8IcuFlag == 2)
            {
                u32b = Mft_Icu_GetCaptureData(&MFT0_ICU, LIN_SYNC_ICU);
                u8DivRate = ((1 << (LIN_PCLK_DIV & 0x03u)) * 8u)/((1 << (FRT_PCLK_DIV & 0x03u))*(1 << LIN_FRT_DIV));
		u32bgr = ((50000+1)*u32Overflow + u32b - u32a)/u8DivRate - 1;
                Mft_Frt_Stop(&MFT0_FRT, 0u);
                Mft_Icu_ConfigDetectMode(&MFT0_ICU, LIN_SYNC_ICU, IcuDisable);
                break;
            }
        }
    }
    u32Pclk1 = __HCLK/(1ul << (LIN_PCLK_DIV & 0x03u)); 
    u32bauddate = u32Pclk1/(u32bgr+1);
    Mfs_Lin_SetBaudRate(LinCh1, u32bauddate);

    /* Enable RX function of LIN   */
    Mfs_Lin_EnableFunc(LinCh1, LinRx);

    /* Receive ID */
    if(Ok != LinReadData(&u8ReceiveId, 1))
    {
        return Error;
    }

    if(u8ReceiveId == u8Id) /* Read data from Master */
    {
        /* Read Data and checksum */
        if(Ok != LinReadData(pu8Data, u32Size))
        {
            return Error;
        }

        u8CalChecksum = LinCalChecksum(pu8Data, u32Size);

        if(Ok != LinReadData(&u8Checksum, 1))
        {
            return Error;
        }

        if(u8CalChecksum != u8Checksum)
        {
            return Error;
        }
    }
    else
    {
        return Error;
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
    InitLin();

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
