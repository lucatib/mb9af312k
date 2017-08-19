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
 ** This example demonstrates communication between CSIO0 and CSIO1 with 
 ** interrupt mode. 
 ** Make following connection before using this example:
 ** Master             Slave
 ** SIN0_0  -------    SOT1_1
 ** SCK0_0  -------    SCK1_1
 ** SOT0_0  -------    SIN1_1
 ** 
 ** History:
 **   - 2014-02-17  1.0  Edison Zhang         First version.
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
#define SAMPLE_CSIO_SLAVE_RX_BUFFSIZE   sizeof(au8CsioMasterTxBuf)/sizeof(char)
#define SAMPLE_CSIO_MASTER_TX_BUFFSIZE  SAMPLE_CSIO_SLAVE_RX_BUFFSIZE

#define SAMPLE_CSIO_MASTER_RX_BUFFSIZE  sizeof(au8CsioSlaveTxBuf)/sizeof(char)
#define SAMPLE_CSIO_SLAVE_TX_BUFFSIZE   SAMPLE_CSIO_MASTER_RX_BUFFSIZE

#define InitCsio0Io(void)  {SetPinFunc_SIN0_0();SetPinFunc_SOT0_0();SetPinFunc_SCK0_0();}
#define InitCsio1Io(void)  {SetPinFunc_SIN1_1();SetPinFunc_SOT1_1();SetPinFunc_SCK1_1();}   

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
volatile stc_mfsn_csio_t* CsioCh0 = &CSIO0;
volatile stc_mfsn_csio_t* CsioCh1 = &CSIO1;
uint32_t u32SendCnt = 0, u32ReceiveCnt = 0;
static uint8_t au8CsioMasterTxBuf[] = "CSIO master sends data to CSIO Slave";
static uint8_t au8CsioSlaveTxBuf[] = "CSIO master reads data from CSIO Slave";
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
static void CsioMasterTxIrqCallback(void)
{
    if(u32SendCnt == SAMPLE_CSIO_MASTER_TX_BUFFSIZE)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh0, CsioTxIrq);
        return;
    }
  
    Mfs_Csio_SendData(CsioCh0, au8CsioMasterTxBuf[u32SendCnt], TRUE);  
    u32SendCnt++;
}

/**
 ******************************************************************************
 ** \brief  CSIO slave receive interrupt callback function
 ******************************************************************************/
static void CsioSlaveRxIrqCallback(void)
{
    au8CsioSlaveRxBuf[u32ReceiveCnt] = Mfs_Csio_ReceiveData(CsioCh1);
    u32ReceiveCnt++;
    
    if(u32ReceiveCnt == SAMPLE_CSIO_SLAVE_RX_BUFFSIZE)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh1, CsioRxIrq);
    }
}

/**
 ******************************************************************************
 ** \brief  CSIO Master receive interrupt callback function
 ******************************************************************************/
static void CsioMasterRxIrqCallback(void)
{
    au8CsioMasterRxBuf[u32ReceiveCnt] = Mfs_Csio_ReceiveData(CsioCh0);
    u32ReceiveCnt++;
    
    if(u32ReceiveCnt == SAMPLE_CSIO_MASTER_RX_BUFFSIZE)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh0, CsioRxIrq);
        return;
    }
}

/**
 ******************************************************************************
 ** \brief  CSIO slave transfer interrupt callback function
 ******************************************************************************/
static void CsioSlaveTxIrqCallback(void)
{
    if(u32SendCnt == SAMPLE_CSIO_SLAVE_TX_BUFFSIZE)
    {
        /* Disable interrupt */
        Mfs_Csio_DisableIrq(CsioCh1, CsioTxIrq);
        return;
    }
  
    Mfs_Csio_SendData(CsioCh1, au8CsioSlaveTxBuf[u32SendCnt], TRUE);  
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
    stc_mfs_csio_config_t stcCsio0Config, stcCsio1Config;
    stc_csio_irq_cb_t stcCsio0IrqCb, stcCsio1IrqCb;
    
    /* Clear configuration structure */
    PDL_ZERO_STRUCT(stcCsio0Config);
    PDL_ZERO_STRUCT(stcCsio1Config);
    PDL_ZERO_STRUCT(stcCsio0IrqCb);
    PDL_ZERO_STRUCT(stcCsio1IrqCb);
    
    /* Initialize CSIO function I/O */    
    InitCsio0Io();
    InitCsio1Io();

    u32SendCnt = 0;
    u32ReceiveCnt = 0;
    
    /* Initialize CSIO interrupt callback functions */
    stcCsio0IrqCb.pfnTxIrqCb = CsioMasterTxIrqCallback;
    stcCsio1IrqCb.pfnRxIrqCb = CsioSlaveRxIrqCallback;
    
    stcCsio0IrqCb.pfnRxIrqCb = CsioMasterRxIrqCallback;
    stcCsio1IrqCb.pfnTxIrqCb = CsioSlaveTxIrqCallback;      
    
    /* Initialize CSIO master  */
    stcCsio0Config.enMsMode = CsioMaster;
    stcCsio0Config.enActMode = CsioActNormalMode;
    stcCsio0Config.bInvertClk = FALSE;
    stcCsio0Config.u32BaudRate = 100000;
    stcCsio0Config.enDataLength = CsioEightBits;
    stcCsio0Config.enBitDirection = CsioDataMsbFirst;
    stcCsio0Config.enSyncWaitTime = CsioSyncWaitZero;
    stcCsio0Config.pstcFifoConfig = NULL;
    stcCsio0Config.pstcIrqCb = &stcCsio0IrqCb;
    stcCsio0Config.pstcIrqEn = NULL;
    stcCsio0Config.bTouchNvic = TRUE;
    
    Mfs_Csio_Init(CsioCh0, &stcCsio0Config);
    
    /* Initialize CSIO slave  */
    stcCsio1Config.enMsMode = CsioSlave;
    stcCsio1Config.enActMode = CsioActNormalMode;
    stcCsio1Config.bInvertClk = FALSE;
    stcCsio1Config.u32BaudRate = 100000;
    stcCsio1Config.enDataLength = CsioEightBits;
    stcCsio1Config.enBitDirection = CsioDataMsbFirst;
    stcCsio1Config.pstcFifoConfig = NULL;
    stcCsio1Config.pstcIrqCb = &stcCsio1IrqCb;
    stcCsio1Config.pstcIrqEn = NULL;
    stcCsio1Config.bTouchNvic = TRUE;
    
    Mfs_Csio_Init(CsioCh1, &stcCsio1Config);
    
    /*************************************************************************/
    /*                Master sends data to slave                             */
    /*************************************************************************/
    /* Configure interrupt */
    Mfs_Csio_EnableIrq(CsioCh1, CsioRxIrq);
    Mfs_Csio_EnableIrq(CsioCh0, CsioTxIrq);
    
    /* Enable RX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioRx);
    /* Enable TX function of CSIO0   */
    Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
    
    /* Wait for TX finish */
    while(u32SendCnt < SAMPLE_CSIO_MASTER_TX_BUFFSIZE)
    {
        ;
    }
    /* Wait for TX bus idle (until last data is sent) */
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle));
    
    /* Wait for RX finish */
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
    Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
    /* Disable RX function of CSIO1   */
    Mfs_Csio_DisableFunc(CsioCh1, CsioRx);
    
    /*************************************************************************/
    /*                Master receives data from slave                        */
    /*************************************************************************/
    u32SendCnt = 0;
    u32ReceiveCnt = 0;
    
    /* Configure interrupt */
    Mfs_Csio_EnableIrq(CsioCh0, CsioRxIrq);
    Mfs_Csio_EnableIrq(CsioCh1, CsioTxIrq);
    
    /* Enable TX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioTx);
    
    /* Enable TX and RX function of CSIO0   */
    Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
    Mfs_Csio_EnableFunc(CsioCh0, CsioRx);
 
    /* Send dummy data */
    while(u32SendCnt < SAMPLE_CSIO_SLAVE_TX_BUFFSIZE)
    {
        while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty));
        Mfs_Csio_SendData(CsioCh0, 0x00u, FALSE);   /* Dummy write */
    }
        
    /* Wait for Master TX bus idle (until last data is sent) */
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle));
    
    /* Wait for receive finish */
    while(u32ReceiveCnt < SAMPLE_CSIO_MASTER_RX_BUFFSIZE)
    {
        ;
    }
    
    /* Compare receive data with transfer data */
    if(Error == CompareData(au8CsioMasterRxBuf, au8CsioSlaveTxBuf, SAMPLE_CSIO_MASTER_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }
    
    Mfs_Csio_DeInit(CsioCh0, TRUE);
    Mfs_Csio_DeInit(CsioCh1, TRUE);
    
    while(1); /* Data is normally sent and received */
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
