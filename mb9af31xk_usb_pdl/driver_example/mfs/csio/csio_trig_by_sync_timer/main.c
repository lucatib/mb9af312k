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
 ** This example demonstrates communication between CSIO1 and CSIO3 with 
 ** interrupt mode. The CSIO1 acts as master which is triggered by CSIO
 ** series timer.
 ** Make following connection before using this example:
 ** Master             Slave
 ** SOT1_1  -------    SIN3_1
 ** SCK1_1  -------    SCK3_1
 ** SIN1_1  -------    SOT3_1
 ** 
 ** Attention:
 ** ============================================================================
 ** 1) This example only supports MFS channels equipped with Chip Selection 
 **    function.
 ** 2) Check if the MFS has serial timer function in the device data sheet
 ** 3) Change MFS channel by modifing following setting:
 **    - The variable of "CsioCh1" and "CsioCh3"
 **    - The GPIO setting of SIN, SCK , SOT, SCS pins. 
 ** ============================================================================
 ** History:
 **   - 2014-11-17  1.0  EZh         First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#if (PDL_MCU_CORE != PDL_FM0P_CORE) && (PDL_MCU_CORE != PDL_FM4_CORE)
#error "see the attention in the header of the file."
#endif

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/
#define SAMPLE_CSIO_SLAVE_RX_BUFFSIZE   sizeof(au8CsioMasterTxBuf)/sizeof(char)
#define SAMPLE_CSIO_MASTER_TX_BUFFSIZE  SAMPLE_CSIO_SLAVE_RX_BUFFSIZE

#define SAMPLE_CSIO_MASTER_RX_BUFFSIZE  sizeof(au8CsioSlaveTxBuf)/sizeof(char)
#define SAMPLE_CSIO_SLAVE_TX_BUFFSIZE   SAMPLE_CSIO_MASTER_RX_BUFFSIZE

#define InitCsio1Io(void)  {SetPinFunc_SIN1_1();SetPinFunc_SOT1_1();SetPinFunc_SCK1_1();}
#define InitCsio3Io(void)  {SetPinFunc_SIN3_1();SetPinFunc_SOT3_1();SetPinFunc_SCK3_1();}   

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
volatile stc_mfsn_csio_t* CsioCh1 = &CSIO1;
volatile stc_mfsn_csio_t* CsioCh3 = &CSIO3;
uint32_t u32SendCnt = 0, u32ReceiveCnt = 0;
static uint8_t au8CsioMasterTxBuf[8] = "01234567";
static uint8_t au8CsioSlaveTxBuf[8] = "abcdefgh";
static uint8_t au8CsioMasterRxBuf[SAMPLE_CSIO_MASTER_RX_BUFFSIZE];
static uint8_t au8CsioSlaveRxBuf[SAMPLE_CSIO_SLAVE_RX_BUFFSIZE];

/**
 ******************************************************************************
 ** \brief  Compare each data in the input two buffers
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
 ** \brief  CSIO master transfer interrupt callback function
 ******************************************************************************/
static void CsioMasterTxIntCallback(void)
{
    if(u32SendCnt == SAMPLE_CSIO_MASTER_TX_BUFFSIZE)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh1, CsioTxIrq);
        return;
    }
  
    Mfs_Csio_SendData(CsioCh1, au8CsioMasterTxBuf[u32SendCnt], TRUE);  
    u32SendCnt++;

}

/**
 ******************************************************************************
 ** \brief  CSIO slave receive interrupt callback function
 ******************************************************************************/
static void CsioSlaveRxIntCallback(void)
{
    au8CsioSlaveRxBuf[u32ReceiveCnt] = Mfs_Csio_ReceiveData(CsioCh3);
    u32ReceiveCnt++;
    
    if(u32ReceiveCnt == SAMPLE_CSIO_SLAVE_RX_BUFFSIZE)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh3, CsioRxIrq);
    }
}

/**
 ******************************************************************************
 ** \brief  CSIO Master receive interrupt callback function
 ******************************************************************************/
static void CsioMasterRxIntCallback(void)
{
    au8CsioMasterRxBuf[u32ReceiveCnt] = Mfs_Csio_ReceiveData(CsioCh1);
    u32ReceiveCnt++;
    
    if(u32ReceiveCnt == SAMPLE_CSIO_MASTER_RX_BUFFSIZE)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh1, CsioRxIrq);
        return;
    }
}

/**
 ******************************************************************************
 ** \brief  CSIO slave transfer interrupt callback function
 ******************************************************************************/
static void CsioSlaveTxIntCallback(void)
{
    if(u32SendCnt == SAMPLE_CSIO_SLAVE_TX_BUFFSIZE)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh3, CsioTxIrq);
        return;
    }
  
    Mfs_Csio_SendData(CsioCh3, au8CsioSlaveTxBuf[u32SendCnt], TRUE);  
    u32SendCnt++;
}

/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{   
    stc_mfs_csio_config_t stcCsio1Config, stcCsio3Config;
    stc_csio_irq_cb_t stcCsio1IrqCb, stcCsio3IrqCb;
    stc_csio_serial_timer_t stcCsioSerialTimer;
    
    /* Clear configuration structure */
    PDL_ZERO_STRUCT(stcCsio1Config);
    PDL_ZERO_STRUCT(stcCsio3Config);
    PDL_ZERO_STRUCT(stcCsio1IrqCb);
    PDL_ZERO_STRUCT(stcCsio3IrqCb);
    
    /* Initialize CSIO function I/O */    
    InitCsio1Io();
    InitCsio3Io();
    
    u32SendCnt = 0;
    u32ReceiveCnt = 0;
    
    /* Initialize interrupt callback functions */
    stcCsio1IrqCb.pfnTxIrqCb = CsioMasterTxIntCallback;
    stcCsio3IrqCb.pfnRxIrqCb = CsioSlaveRxIntCallback;
    
    stcCsio1IrqCb.pfnRxIrqCb = CsioMasterRxIntCallback;
    stcCsio3IrqCb.pfnTxIrqCb = CsioSlaveTxIntCallback;
    
    /* Initialize serial timer */
    stcCsioSerialTimer.enClkDiv = CsioTimerNoDiv;
    stcCsioSerialTimer.u16CompareVal = SystemCoreClock/1000;  // Set interval to 1ms
    stcCsioSerialTimer.u8TransferByteCnt = 1; 
    
    /* Initialize CSIO master  */
    stcCsio1Config.enMsMode = CsioMaster;
    stcCsio1Config.enActMode = CsioActNormalMode;
    stcCsio1Config.bInvertClk = FALSE;
    stcCsio1Config.u32BaudRate = 100000;
    stcCsio1Config.enDataLength = CsioEightBits;
    stcCsio1Config.enBitDirection = CsioDataMsbFirst;
    stcCsio1Config.enSyncWaitTime = CsioSyncWaitZero;
    stcCsio1Config.pstcFifoConfig = NULL;
    stcCsio1Config.pstcCsConfig = NULL;
    stcCsio1Config.pstcSerialTimer = &stcCsioSerialTimer;
    stcCsio1Config.pstcIrqEn = NULL;
    stcCsio1Config.pstcIrqCb = &stcCsio1IrqCb;
    stcCsio1Config.bTouchNvic = TRUE;

    Mfs_Csio_Init(CsioCh1, &stcCsio1Config);
    
    /* Initialize CSIO slave  */
    stcCsio3Config.enMsMode = CsioSlave;
    stcCsio3Config.enActMode = CsioActNormalMode;
    stcCsio3Config.bInvertClk = FALSE;
    stcCsio3Config.u32BaudRate = 100000;
    stcCsio3Config.enDataLength = CsioEightBits;
    stcCsio3Config.enBitDirection = CsioDataMsbFirst;
    stcCsio3Config.pstcFifoConfig = NULL;
    stcCsio3Config.pstcCsConfig = NULL;
    stcCsio3Config.pstcSerialTimer = NULL;
    stcCsio3Config.pstcIrqEn = NULL;
    stcCsio3Config.pstcIrqCb = &stcCsio3IrqCb;
    stcCsio3Config.bTouchNvic = TRUE;
    
    Mfs_Csio_Init(CsioCh3, &stcCsio3Config);
    
    /*************************************************************************/
    /*                Master sends data to slave                             */
    /*************************************************************************/
    /* Enable TX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioTx);
    
    /* Enable RX function of CSIO3   */
    Mfs_Csio_EnableFunc(CsioCh3, CsioRx);
    
    /* Start Sync Timer   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioSerialTimer);
    
    /* Configure interrupt */
    Mfs_Csio_EnableIrq(CsioCh3, CsioRxIrq);
    Mfs_Csio_EnableIrq(CsioCh1, CsioTxIrq);
    
    /* Wait until send finish */
    while(u32SendCnt < SAMPLE_CSIO_MASTER_TX_BUFFSIZE)
    {
        ;
    }
    
    while(TRUE != Mfs_Csio_GetStatus(CsioCh1, CsioTxIdle)); // wait TX idle
    
    /* Wait until receive finish */
    while(u32ReceiveCnt < SAMPLE_CSIO_SLAVE_RX_BUFFSIZE)
    {
        ;
    }
    
    /* Compare receive data with transfer data */
    if(Error == CompareData(au8CsioMasterTxBuf, au8CsioSlaveRxBuf, SAMPLE_CSIO_SLAVE_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }
    
    /* Disable TX function of CSIO0   */
    Mfs_Csio_DisableFunc(CsioCh1, CsioTx);
    /* Disable RX function of CSIO1   */
    Mfs_Csio_DisableFunc(CsioCh3, CsioRx);
    /* Stop Sync Timer   */
    Mfs_Csio_DisableFunc(CsioCh1, CsioSerialTimer);
    
    /*************************************************************************/
    /*                Master receives data from slave                        */
    /*************************************************************************/
    /* Enable TX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh3, CsioTx);
    
    /* Enable TX and RX function of CSIO0   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioTx);
    Mfs_Csio_EnableFunc(CsioCh1, CsioRx);
   
    /* Configure interrupt */
    u32SendCnt = 0;
    u32ReceiveCnt = 0;
    
    Mfs_Csio_EnableIrq(CsioCh1, CsioRxIrq);
    Mfs_Csio_EnableIrq(CsioCh3, CsioTxIrq);
   
    /* Start Sync Timer   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioSerialTimer);
     
    /* Dummy write */
    while(u32SendCnt < SAMPLE_CSIO_SLAVE_TX_BUFFSIZE)
    {
        while(TRUE != Mfs_Csio_GetStatus(CsioCh1, CsioTxEmpty));
        Mfs_Csio_SendData(CsioCh1, 0x00u, FALSE);   /* Dummy write */
    }
    
    while(TRUE != Mfs_Csio_GetStatus(CsioCh1, CsioTxIdle)); // wait TX idle
    
    /* wait receive finish */
    while(u32ReceiveCnt < SAMPLE_CSIO_MASTER_RX_BUFFSIZE)
    {
        ;
    }
    
    /* Compare receive data with transfer data */
    if(Error == CompareData(au8CsioMasterRxBuf, au8CsioSlaveTxBuf, SAMPLE_CSIO_MASTER_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }
    
    Mfs_Csio_DeInit(CsioCh1, TRUE);
    Mfs_Csio_DeInit(CsioCh3, TRUE);
    
    while(1); /* Data is normally sent and received */
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
