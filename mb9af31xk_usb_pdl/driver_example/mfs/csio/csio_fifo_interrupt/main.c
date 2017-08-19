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
 ** interrupt mode using FIFO. 
 ** Make following connection before using this example:
 ** Master             Slave
 ** SIN0_0  -------    SOT1_1
 ** SCK0_0  -------    SCK1_1
 ** SOT0_0  -------    SIN1_1
 **  
 ** Attention:
 ** ============================================================================
 ** This example only supports the MFS channel which is equipped with FIFO. 
 ** Check the FIFO avaibility in the device data sheet. 
 ** The MFS channel can be changed by modifying following setting:
 ** - The variable of "CsioCh0" and "CsioCh1"
 ** - The MFS GPIO pin setting
 ** ============================================================================
 ** 
 ** History:
 **   - 2014-11-10  1.0  EZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
#define SAMPLE_CSIO_SLAVE_RX_BUFFSIZE   sizeof(au8CsioMasterTxBuf)/sizeof(char)
#define SAMPLE_CSIO_MASTER_TX_BUFFSIZE  SAMPLE_CSIO_SLAVE_RX_BUFFSIZE

#define SAMPLE_CSIO_MASTER_RX_BUFFSIZE  sizeof(au8CsioSlaveTxBuf)/sizeof(char)
#define SAMPLE_CSIO_SLAVE_TX_BUFFSIZE   SAMPLE_CSIO_MASTER_RX_BUFFSIZE

#if (PDL_MCU_CORE == PDL_FM0P_CORE)
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
#define SAMPLE_CSIO_FIFO_MAX_CAPACITY         (32u)
#else
#define SAMPLE_CSIO_FIFO_MAX_CAPACITY         (128u)
#endif
#elif (PDL_MCU_CORE == PDL_FM4_CORE)
#define SAMPLE_CSIO_FIFO_MAX_CAPACITY         (32u)
#elif (PDL_MCU_CORE == PDL_FM3_CORE)
#define SAMPLE_CSIO_FIFO_MAX_CAPACITY         (16u)
#endif

#define SAMPLE_CSIO_FIFO_RX_CNT               (8u)

#define InitCsio0Io(void)  {SetPinFunc_SIN0_0();SetPinFunc_SOT0_0();SetPinFunc_SCK0_0();}
#define InitCsio1Io(void)  {SetPinFunc_SIN1_1();SetPinFunc_SOT1_1();SetPinFunc_SCK1_1();}   
/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/
/// CSIO TX FIFO information structure
typedef struct stc_tx_fifo_info
{
    uint32_t u32TxCnt;
    uint8_t* pTxBuf;
    boolean_t bTxFinish;
    
}stc_tx_fifo_info_t;

/// UART RX FIFO information structure
typedef struct stc_rx_fifo_info
{
    uint32_t u32RxCnt;
    uint8_t* pRxBuf;
    boolean_t bRxFinish;
    
}stc_rx_fifo_info_t;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static volatile stc_mfsn_csio_t* CsioCh0 = &CSIO0;
static volatile stc_mfsn_csio_t* CsioCh1 = &CSIO1;
static uint8_t au8CsioMasterTxBuf[50] = "CSIO master sends data to CSIO Slave";
static uint8_t au8CsioSlaveTxBuf[50] = "CSIO master reads data from CSIO Slave";
static uint8_t au8CsioMasterRxBuf[SAMPLE_CSIO_MASTER_RX_BUFFSIZE];
static uint8_t au8CsioSlaveRxBuf[SAMPLE_CSIO_SLAVE_RX_BUFFSIZE];
static stc_tx_fifo_info_t stcTxFifoInfo = {0};
static stc_rx_fifo_info_t stcRxFifoInfo = {0};

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
 ** \brief  CSIO master FIFO transfer interrupt callback function
 ******************************************************************************/
static void CsioMasterFifoTxIntCallback(void)
{
    uint8_t u8i = 0;
    if(stcTxFifoInfo.u32TxCnt > SAMPLE_CSIO_FIFO_MAX_CAPACITY)
    {
        while(u8i < SAMPLE_CSIO_FIFO_MAX_CAPACITY)
        {
            Mfs_Csio_SendData(CsioCh0, *stcTxFifoInfo.pTxBuf++, TRUE);
            u8i++;
        }
        stcTxFifoInfo.u32TxCnt -= SAMPLE_CSIO_FIFO_MAX_CAPACITY;
        return;
    }
    
    while(u8i < stcTxFifoInfo.u32TxCnt)
    {
        Mfs_Csio_SendData(CsioCh0, *stcTxFifoInfo.pTxBuf++, TRUE);
        u8i++;
    }
  
    Mfs_Csio_DisableIrq(CsioCh0, CsioTxFifoIrq);
    
    stcTxFifoInfo.bTxFinish = TRUE;
}

/**
 ******************************************************************************
 ** \brief  CSIO slave receive interrupt callback function
 ******************************************************************************/
static void CsioSlaveRxIntCallback(void)
{
    uint8_t u8i = 0;
    if(stcRxFifoInfo.u32RxCnt > SAMPLE_CSIO_FIFO_RX_CNT) 
    {
        /* Receive data when RX FIFO count match with SAMPLE_UART_FIFO_RX_CNT */
        while(u8i < SAMPLE_CSIO_FIFO_RX_CNT)
        {
            *stcRxFifoInfo.pRxBuf++ = Mfs_Csio_ReceiveData(CsioCh1); 
            u8i++;
        }
        stcRxFifoInfo.u32RxCnt -= SAMPLE_CSIO_FIFO_RX_CNT;
        return;
    }
    
    /* Receive data when RX FIFO is idle */
    /* idle means FIFO count is less than SAMPLE_UART_FIFO_RX_CNT and
       RX FIFO don't receive data from then on for a short time. */
    while(u8i < stcRxFifoInfo.u32RxCnt)
    {
        *stcRxFifoInfo.pRxBuf++ = Mfs_Csio_ReceiveData(CsioCh1);
        u8i++;
    }
    
    Mfs_Csio_DisableIrq(CsioCh1, CsioRxIrq);
    
    stcRxFifoInfo.bRxFinish = TRUE;
}

/**
 ******************************************************************************
 ** \brief  CSIO Master receive interrupt callback function
 ******************************************************************************/
static void CsioMasterRxIntCallback(void)
{
    uint8_t u8i = 0;
    if(stcRxFifoInfo.u32RxCnt > SAMPLE_CSIO_FIFO_RX_CNT) 
    {
        /* Receive data when RX FIFO count match with SAMPLE_UART_FIFO_RX_CNT */
        while(u8i < SAMPLE_CSIO_FIFO_RX_CNT)
        {
            *stcRxFifoInfo.pRxBuf++ = Mfs_Csio_ReceiveData(CsioCh0); 
            u8i++;
        }
        stcRxFifoInfo.u32RxCnt -= SAMPLE_CSIO_FIFO_RX_CNT;
        return;
    }
    
    /* Receive data when RX FIFO is idle */
    /* idle means FIFO count is less than SAMPLE_UART_FIFO_RX_CNT and
       RX FIFO don't receive data from then on for a short time. */
    while(u8i < stcRxFifoInfo.u32RxCnt)
    {
        *stcRxFifoInfo.pRxBuf++ = Mfs_Csio_ReceiveData(CsioCh0);
        u8i++;
    }
    
    Mfs_Csio_DisableIrq(CsioCh0, CsioRxIrq);
    
    stcRxFifoInfo.bRxFinish = TRUE;
}

/**
 ******************************************************************************
 ** \brief  CSIO slave FIFO transfer interrupt callback function
 ******************************************************************************/
static void CsioSlaveFifoTxIntCallback(void)
{
    uint8_t u8i = 0;
    if(stcTxFifoInfo.u32TxCnt > SAMPLE_CSIO_FIFO_MAX_CAPACITY)
    {
        while(u8i < SAMPLE_CSIO_FIFO_MAX_CAPACITY)
        {
            Mfs_Csio_SendData(CsioCh1, *stcTxFifoInfo.pTxBuf++, TRUE);
            u8i++;
        }
        stcTxFifoInfo.u32TxCnt -= SAMPLE_CSIO_FIFO_MAX_CAPACITY;
        return;
    }
    
    while(u8i < stcTxFifoInfo.u32TxCnt)
    {
        Mfs_Csio_SendData(CsioCh1, *stcTxFifoInfo.pTxBuf++, TRUE);
        u8i++;
    }
  
    Mfs_Csio_DisableIrq(CsioCh1, CsioTxFifoIrq);
    
    stcTxFifoInfo.bTxFinish = TRUE;
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
    stc_mfs_fifo_config_t stcFifoConfig;
    uint8_t u8i;
    
    /* Clear configuration structure */
    PDL_ZERO_STRUCT(stcCsio0Config);
    PDL_ZERO_STRUCT(stcCsio1Config);
    PDL_ZERO_STRUCT(stcCsio0IrqCb);
    PDL_ZERO_STRUCT(stcCsio1IrqCb);
    
    /* Initialize CSIO function I/O */    
    InitCsio0Io();
    InitCsio1Io();
    
    /* Initialize CSIO interrupt callback functions */
    stcCsio0IrqCb.pfnTxFifoIrqCb = CsioMasterFifoTxIntCallback;
    stcCsio1IrqCb.pfnRxIrqCb = CsioSlaveRxIntCallback;
    
    stcCsio0IrqCb.pfnRxIrqCb = CsioMasterRxIntCallback;
    stcCsio1IrqCb.pfnTxFifoIrqCb = CsioSlaveFifoTxIntCallback;
    
    /* Initialize FIFO configuration */
    stcFifoConfig.enFifoSel = MfsTxFifo1RxFifo2;
    stcFifoConfig.u8ByteCount1 = 0u;
    stcFifoConfig.u8ByteCount2 = SAMPLE_CSIO_FIFO_RX_CNT;
    
    /* Initialize CSIO master  */
    stcCsio0Config.enMsMode = CsioMaster;
    stcCsio0Config.enActMode = CsioActNormalMode;
    stcCsio0Config.bInvertClk = FALSE;
    stcCsio0Config.u32BaudRate = 100000;
    stcCsio0Config.enDataLength = CsioEightBits;
    stcCsio0Config.enBitDirection = CsioDataMsbFirst;
    stcCsio0Config.pstcFifoConfig = &stcFifoConfig;
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
    stcCsio1Config.pstcFifoConfig = &stcFifoConfig;;
    stcCsio1Config.pstcIrqCb = &stcCsio1IrqCb;
    stcCsio1Config.pstcIrqEn = NULL;
    stcCsio1Config.bTouchNvic = TRUE;
    
    Mfs_Csio_Init(CsioCh1, &stcCsio1Config);
    
    /*************************************************************************/
    /*                Master sends data to slave                             */
    /*************************************************************************/
    /* Initialize the FIFO information */
    // Master
    stcTxFifoInfo.u32TxCnt = SAMPLE_CSIO_MASTER_TX_BUFFSIZE;
    stcTxFifoInfo.pTxBuf = au8CsioMasterTxBuf;
    stcTxFifoInfo.bTxFinish = FALSE;
    // Slave
    stcRxFifoInfo.u32RxCnt = SAMPLE_CSIO_SLAVE_RX_BUFFSIZE;
    stcRxFifoInfo.pRxBuf = au8CsioSlaveRxBuf;
    stcRxFifoInfo.bRxFinish = FALSE;
    
    /* Enable RX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioRx);
    /* Enable TX function of CSIO0   */
    Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
    
    /* Configure interrupt */
    Mfs_Csio_EnableIrq(CsioCh1, CsioRxIrq);
    Mfs_Csio_EnableIrq(CsioCh0, CsioTxFifoIrq);
    
    while(stcTxFifoInfo.bTxFinish != TRUE) // wait for Master TX finish
    {};
    
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait TX idle
    
    while(stcRxFifoInfo.bRxFinish != TRUE) // wait for Slave RX finish
    {};
    
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
    /* Initialize the FIFO information */
    // Master
    stcRxFifoInfo.u32RxCnt = SAMPLE_CSIO_MASTER_RX_BUFFSIZE;
    stcRxFifoInfo.pRxBuf = au8CsioMasterRxBuf;
    stcRxFifoInfo.bRxFinish = FALSE;
    
    // Slave
    stcTxFifoInfo.u32TxCnt = SAMPLE_CSIO_SLAVE_TX_BUFFSIZE;
    stcTxFifoInfo.pTxBuf = au8CsioSlaveTxBuf;
    stcTxFifoInfo.bTxFinish = FALSE;
    
    /* Enable TX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioTx);
    
    /* Enable TX and RX function of CSIO0   */
    Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
    Mfs_Csio_EnableFunc(CsioCh0, CsioRx);
    
    /* Configure interrupt */
    Mfs_Csio_EnableIrq(CsioCh0, CsioRxIrq);
    Mfs_Csio_EnableIrq(CsioCh1, CsioTxFifoIrq); // Slave TX FIFO interrupt will occur soon after this line.
       
    /* Now data has been prepared in the Slave side */
    /* Read the data by writing data synchronously */
    u8i = 0;
    while(u8i < SAMPLE_CSIO_SLAVE_TX_BUFFSIZE)
    {
        while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty)); // wait TX idle
        Mfs_Csio_SendData(CsioCh0, 0x00u, FALSE);   /* Dummy write */
        u8i++;
    }
    
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait TX idle
    
    /* Wait until Master finish reading FIFO */
    while(stcRxFifoInfo.bRxFinish != TRUE)
    {};
    
    /* Compare receive data with transfer data */
    if(Error == CompareData(au8CsioMasterRxBuf, au8CsioSlaveTxBuf, SAMPLE_CSIO_MASTER_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }
    
    /* Disable TX and RX function of CSIO0   */
    Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
    Mfs_Csio_DisableFunc(CsioCh0, CsioRx);
    /* Disable TX function of CSIO1   */
    Mfs_Csio_DisableFunc(CsioCh1, CsioTx);
    
    Mfs_Csio_DeInit(CsioCh0, TRUE);
    Mfs_Csio_DeInit(CsioCh1, TRUE);
    
    while(1); /* Data is normally sent and received */
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
