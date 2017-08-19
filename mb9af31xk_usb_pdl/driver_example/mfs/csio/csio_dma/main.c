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
 ** interrupt mode using DMA. 
 ** 
 ** History:
 **   - 2015-01-04  1.0  DHo         First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
#define SAMPLE_CSIO_SLAVE_RX_BUFFSIZE   (8u)
#define SAMPLE_CSIO_MASTER_TX_BUFFSIZE  (8u)

#define SAMPLE_CSIO_MASTER_RX_BUFFSIZE  (8u)
#define SAMPLE_CSIO_SLAVE_TX_BUFFSIZE   (8u)

#define InitCsio0Io(void)  {SetPinFunc_SIN0_0();SetPinFunc_SOT0_0();SetPinFunc_SCK0_0();}
#define InitCsio1Io(void)  {SetPinFunc_SIN1_1();SetPinFunc_SOT1_1();SetPinFunc_SCK1_1();} 

#define DMA_CH0_IDRQ_MASTER_TX   MfsTx0
#define DMA_CH1_IDRQ_SLAVE_RX    MfsRx1

#define DMA_CH0_IDRQ_MASTER_RX   MfsRx0
#define DMA_CH1_IDRQ_SLAVE_TX    MfsTx1

#define DMA_CH0_DEST_ADDRESS_MASTER_TX      MFS0_DATA_REG_ADDR
#define DMA_CH1_SOURCE_ADDRESS_SLAVE_RX     MFS1_DATA_REG_ADDR

#define DMA_CH0_SOURCE_ADDRESS_MASTER_RX    MFS0_DATA_REG_ADDR
#define DMA_CH1_DEST_ADDRESS_SLAVE_TX       MFS1_DATA_REG_ADDR

#define CSIO_CH0    CSIO0
#define CSIO_CH1    CSIO1

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/
  

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static volatile stc_mfsn_csio_t* CsioCh0 = &CSIO_CH0;
static volatile stc_mfsn_csio_t* CsioCh1 = &CSIO_CH1;
static boolean_t m_bDma0Finished, m_bDma1Finished;
static uint8_t m_au8CsioMasterTxBuf[SAMPLE_CSIO_MASTER_TX_BUFFSIZE];
static uint8_t m_au8CsioSlaveTxBuf[SAMPLE_CSIO_SLAVE_TX_BUFFSIZE];
static uint8_t m_au8CsioMasterRxBuf[SAMPLE_CSIO_MASTER_RX_BUFFSIZE];
static uint8_t m_au8CsioSlaveRxBuf[SAMPLE_CSIO_SLAVE_RX_BUFFSIZE];

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
 ** \brief  DMA Error callback function
 ******************************************************************************/
void Main_dma0_error_callback(uint8_t u8ErrorCode)
{
    // Error handling ... Should never happen.
    return;
}

/**
 ******************************************************************************
 ** \brief  DMNA finish callback function
 ******************************************************************************/
void Main_dma0_finish_callback(void)
{  
    Mfs_Csio_DisableIrq(CsioCh0, CsioTxIrq);
    Mfs_Csio_DisableIrq(CsioCh0, CsioRxIrq);
    
    m_bDma0Finished = TRUE;         // Set DMA finished notification flag
    
    return;
}

/**
 ******************************************************************************
 ** \brief  DMA Error callback function
 ******************************************************************************/
void Main_dma1_error_callback(uint8_t u8ErrorCode)
{
    // Error handling ... Should never happen.
    return;
}

/**
 ******************************************************************************
 ** \brief  DMNA finish callback function
 ******************************************************************************/
void Main_dma1_finish_callback(void)
{    
    Mfs_Csio_DisableIrq(CsioCh1, CsioTxIrq);
    Mfs_Csio_DisableIrq(CsioCh1, CsioRxIrq);
    m_bDma1Finished = TRUE;         // Set DMA finished notification flag
    
    return;
}

/**
 ******************************************************************************
 ** \brief Dummy callback function
 ******************************************************************************/
static void DummyIntCallback(void)
{
    // Should never enter
    return;
}

/**
 ******************************************************************************
 ** \brief  Initialize and enable DMA ch.0 and ch.1 for CSIO master TX and
 **         CSIO slave RX
 ******************************************************************************/
void MasterTxSlaveRxDmaInit(void)
{
    stc_dma_config_t stcDma0Config, stcDma1Config;
    stc_dma_irq_en_t stcDma0IrqEn, stcDma1IrqEn;
    stc_dma_irq_cb_t stcDma0IrqCb, stcDma1IrqCb;

    // Clear local configuration structure to zero.
    PDL_ZERO_STRUCT(stcDma0Config);	
    PDL_ZERO_STRUCT(stcDma0IrqEn);	
    PDL_ZERO_STRUCT(stcDma0IrqCb);	
    PDL_ZERO_STRUCT(stcDma1Config);	
    PDL_ZERO_STRUCT(stcDma1IrqEn);	
    PDL_ZERO_STRUCT(stcDma1IrqCb);

    // DMAC0  Irq configuration
    stcDma0IrqEn.bErrorIrq = TRUE; 
    stcDma0IrqEn.bCompleteIrq = TRUE;    
    stcDma0IrqCb.pfnDmaErrorIrqCb = Main_dma0_error_callback;
    stcDma0IrqCb.pfnDmaCompletionIrqCb = Main_dma0_finish_callback;
    // DMAC0 configuration
    stcDma0Config.enDmaIdrq = DMA_CH0_IDRQ_MASTER_TX;
    stcDma0Config.u8BlockCount = 1u;
    stcDma0Config.u16TransferCount = SAMPLE_CSIO_MASTER_TX_BUFFSIZE ;   
    stcDma0Config.enTransferMode = DmaDemandTransfer;
    stcDma0Config.enTransferWdith = Dma8Bit;
    stcDma0Config.u32SourceAddress = (uint32_t)&(m_au8CsioMasterTxBuf[0u]); // UART data address
    stcDma0Config.u32DestinationAddress = DMA_CH0_DEST_ADDRESS_MASTER_TX;	   // Destination array's address
    stcDma0Config.bFixedSource = FALSE;
    stcDma0Config.bFixedDestination = TRUE;
    stcDma0Config.bReloadCount = TRUE;       // Reload count for next data package
    stcDma0Config.bReloadSource = TRUE;      // Reload source address for next data package
    stcDma0Config.bReloadDestination = FALSE; 
    stcDma0Config.bEnableBitMask = TRUE;    // Don't clear enable bit after transfer completion  
    stcDma0Config.pstcIrqEn = &stcDma0IrqEn;     ///< Pointer to DMA interrupt enable structure
    stcDma0Config.pstcIrqCb = &stcDma0IrqCb;     ///< Pointer to DMA interrupt callback function structure
    stcDma0Config.bTouchNvic = TRUE;             ///< TRUE: enable NVIC, FALSE: disable NVIC

    if (Ok == Dma_InitChannel(0u, &stcDma0Config))      // Initialize DMA channel 0
    {
        Dma_SetChannel(0u, TRUE, FALSE, FALSE);	      // Enable channel
    }
    
    // DMAC1  Irq configuration
    stcDma1IrqEn.bErrorIrq = TRUE; 
    stcDma1IrqEn.bCompleteIrq = TRUE;    
    stcDma1IrqCb.pfnDmaErrorIrqCb = Main_dma1_error_callback;
    stcDma1IrqCb.pfnDmaCompletionIrqCb = Main_dma1_finish_callback;
    // DMAC1 configuration
    stcDma1Config.enDmaIdrq = DMA_CH1_IDRQ_SLAVE_RX;
    stcDma1Config.u8BlockCount = 1u;
    stcDma1Config.u16TransferCount = SAMPLE_CSIO_SLAVE_RX_BUFFSIZE ;   
    stcDma1Config.enTransferMode = DmaDemandTransfer;
    stcDma1Config.enTransferWdith = Dma8Bit;
    stcDma1Config.u32SourceAddress = DMA_CH1_SOURCE_ADDRESS_SLAVE_RX ; // UART data address
    stcDma1Config.u32DestinationAddress = (uint32_t)&(m_au8CsioSlaveRxBuf[0u]);	 // Destination array's address
    stcDma1Config.bFixedSource = TRUE;
    stcDma1Config.bFixedDestination = FALSE;
    stcDma1Config.bReloadCount = TRUE;       // Reload count for next data package
    stcDma1Config.bReloadSource = FALSE;
    stcDma1Config.bReloadDestination = TRUE; // Reload destination for next data package
    stcDma1Config.bEnableBitMask = TRUE;     // Don't clear enable bit after transfer completion   
    stcDma1Config.pstcIrqEn = &stcDma1IrqEn;     ///< Pointer to DMA interrupt enable structure
    stcDma1Config.pstcIrqCb = &stcDma1IrqCb;     ///< Pointer to DMA interrupt callback function structure
    stcDma1Config.bTouchNvic = TRUE;             ///< TRUE: enable NVIC, FALSE: disable NVIC

    if (Ok == Dma_InitChannel(1u, &stcDma1Config))      // Initialize DMA channel 1
    {
        Dma_SetChannel(1u, TRUE, FALSE, FALSE);	      // Enable channel
    }
    
    Dma_Enable();            // Overall enable of DMA

    return;
}

/**
 ******************************************************************************
 ** \brief  De-Initialize DMA ch.0 and ch.1 for CSIO master RX and
 **         CSIO slave TX
 ******************************************************************************/
void MasterTxSlaveRxDmaDeInit(void)
{
    Dma_Disable();            // Overall enable of DMA
    
    Dma_DeInitChannel(0u, TRUE);
    Dma_DeInitChannel(1u, TRUE);

    return;
}

/**
 ******************************************************************************
 ** \brief  Initialize and enable DMA ch.0 and ch.1 for CSIO master RX and
 **         CSIO slave TX
 ******************************************************************************/
void MasterRxSlaveTxDmaInit(void)
{
    stc_dma_config_t stcDma0Config, stcDma1Config;
    stc_dma_irq_en_t stcDma0IrqEn, stcDma1IrqEn;
    stc_dma_irq_cb_t stcDma0IrqCb, stcDma1IrqCb;

    // Clear local configuration structure to zero.
    PDL_ZERO_STRUCT(stcDma0Config);	
    PDL_ZERO_STRUCT(stcDma0IrqEn);	
    PDL_ZERO_STRUCT(stcDma0IrqCb);	
    PDL_ZERO_STRUCT(stcDma1Config);	
    PDL_ZERO_STRUCT(stcDma1IrqEn);	
    PDL_ZERO_STRUCT(stcDma1IrqCb);

    /* Configure interrupt */
    stcDma0IrqEn.bCompleteIrq = TRUE;
    stcDma0IrqEn.bErrorIrq = TRUE;
    stcDma0IrqCb.pfnDmaCompletionIrqCb = Main_dma0_finish_callback;
    stcDma0IrqCb.pfnDmaErrorIrqCb = Main_dma0_error_callback; 
    // DMAC0 configuration
    stcDma0Config.enDmaIdrq = DMA_CH0_IDRQ_MASTER_RX;
    stcDma0Config.u8BlockCount = 1u;
    stcDma0Config.u16TransferCount = SAMPLE_CSIO_MASTER_RX_BUFFSIZE ;   
    stcDma0Config.enTransferMode = DmaDemandTransfer;
    stcDma0Config.enTransferWdith = Dma8Bit;
    stcDma0Config.u32SourceAddress = DMA_CH0_SOURCE_ADDRESS_MASTER_RX; // CSIO data register address
    stcDma0Config.u32DestinationAddress = (uint32_t)&(m_au8CsioMasterRxBuf[0u]); // destination array address
    stcDma0Config.bFixedSource = TRUE;
    stcDma0Config.bFixedDestination = FALSE;
    stcDma0Config.bReloadCount = TRUE;       // Reload count for next data package
    stcDma0Config.bReloadSource = FALSE;      
    stcDma0Config.bReloadDestination = TRUE; // Reload destination address for next data package
    stcDma0Config.bEnableBitMask = TRUE;     // Don't clear enable bit after transfer completion   
    stcDma0Config.pstcIrqEn = &stcDma0IrqEn;     ///< Pointer to DMA interrupt enable structure
    stcDma0Config.pstcIrqCb = &stcDma0IrqCb;     ///< Pointer to DMA interrupt callback function structure
    stcDma0Config.bTouchNvic = TRUE;             ///< TRUE: enable NVIC, FALSE: disable NVIC

    if (Ok == Dma_InitChannel(0u, &stcDma0Config))      // Initialize DMA channel 0
    {
        Dma_SetChannel(0u, TRUE, FALSE, FALSE);	      // Enable channel
    }

    /* Configure interrupt */
    stcDma1IrqEn.bCompleteIrq = TRUE;
    stcDma1IrqEn.bErrorIrq = TRUE;
    stcDma1IrqCb.pfnDmaCompletionIrqCb = Main_dma1_finish_callback;
    stcDma1IrqCb.pfnDmaErrorIrqCb = Main_dma1_error_callback; 
    // DMAC1 configuration
    stcDma1Config.enDmaIdrq = DMA_CH1_IDRQ_SLAVE_TX;
    stcDma1Config.u8BlockCount = 1u;
    stcDma1Config.u16TransferCount = SAMPLE_CSIO_SLAVE_TX_BUFFSIZE ;   
    stcDma1Config.enTransferMode = DmaDemandTransfer;
    stcDma1Config.enTransferWdith = Dma8Bit;
    stcDma1Config.u32SourceAddress = (uint32_t)&(m_au8CsioSlaveTxBuf[0u]);	 // Source array's address
    stcDma1Config.u32DestinationAddress =  DMA_CH1_DEST_ADDRESS_SLAVE_TX ; // CSIO data register address
    stcDma1Config.bFixedSource = FALSE;
    stcDma1Config.bFixedDestination = TRUE;
    stcDma1Config.bReloadCount = TRUE;       // Reload count for next data package
    stcDma1Config.bReloadSource = TRUE;
    stcDma1Config.bReloadDestination = FALSE; // Reload destination for next data package
    stcDma1Config.bEnableBitMask = TRUE;     // Don't clear enable bit after transfer completion  
    stcDma1Config.pstcIrqEn = &stcDma1IrqEn;     ///< Pointer to DMA interrupt enable structure
    stcDma1Config.pstcIrqCb = &stcDma1IrqCb;     ///< Pointer to DMA interrupt callback function structure
    stcDma1Config.bTouchNvic = TRUE;             ///< TRUE: enable NVIC, FALSE: disable NVIC

    if (Ok == Dma_InitChannel(1u, &stcDma1Config))      // Initialize DMA channel 1
    {
        Dma_SetChannel(1u, TRUE, FALSE, FALSE);	      // Enable channel
    }
    
    Dma_Enable();            // Overall enable of DMA

    return;
}

/**
 ******************************************************************************
 ** \brief  De-Initialize DMA ch.0 and ch.1 for CSIO master RX and
 **         CSIO slave TX
 ******************************************************************************/
void MasterRxSlaveTxDmaDeInit(void)
{
    Dma_Disable();            // Overall enable of DMA
    Dma_DeInitChannel(0u, TRUE);
    Dma_DeInitChannel(1u, TRUE);

    return;
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
    stc_csio_irq_en_t stcCsio0IrqEn, stcCsio1IrqEn;           
    stc_csio_irq_cb_t stcCsio0IrqCb, stcCsio1IrqCb;
    uint8_t u8i;
    
    /* Clear configuration structure */
    PDL_ZERO_STRUCT(stcCsio0Config);
    PDL_ZERO_STRUCT(stcCsio0IrqEn);
    PDL_ZERO_STRUCT(stcCsio0IrqCb);
    PDL_ZERO_STRUCT(stcCsio1Config);
    PDL_ZERO_STRUCT(stcCsio1IrqEn);
    PDL_ZERO_STRUCT(stcCsio1IrqCb);
    
    /* Initialize CSIO function I/O */    
    InitCsio0Io();
    InitCsio1Io();

    stcCsio0IrqCb.pfnRxIrqCb = DummyIntCallback;    
    stcCsio0IrqCb.pfnTxIrqCb = DummyIntCallback;
    /* Initialize CSIO master  */
    stcCsio0Config.enMsMode = CsioMaster;
    stcCsio0Config.enActMode = CsioActNormalMode;
    stcCsio0Config.bInvertClk = FALSE;
    stcCsio0Config.u32BaudRate = 100000;
    stcCsio0Config.enDataLength = CsioEightBits;
    stcCsio0Config.enBitDirection = CsioDataMsbFirst;
    stcCsio0Config.enSyncWaitTime = CsioSyncWaitZero;      
    stcCsio0Config.pstcIrqEn = &stcCsio0IrqEn;      
    stcCsio0Config.pstcIrqCb = &stcCsio0IrqCb;        
    stcCsio0Config.bTouchNvic = TRUE;                
    Mfs_Csio_Init(CsioCh0, &stcCsio0Config);

    stcCsio1IrqCb.pfnRxIrqCb = DummyIntCallback;
    stcCsio1IrqCb.pfnTxIrqCb = DummyIntCallback;
    /* Initialize CSIO slave  */
    stcCsio1Config.enMsMode = CsioSlave;
    stcCsio1Config.enActMode = CsioActNormalMode;
    stcCsio1Config.bInvertClk = FALSE;
    stcCsio1Config.u32BaudRate = 100000;
    stcCsio1Config.enDataLength = CsioEightBits;
    stcCsio1Config.enBitDirection = CsioDataMsbFirst;
    stcCsio1Config.pstcFifoConfig = NULL; 
    stcCsio1Config.pstcIrqEn = &stcCsio1IrqEn;      
    stcCsio1Config.pstcIrqCb = &stcCsio1IrqCb;        
    stcCsio1Config.bTouchNvic = TRUE; 
    Mfs_Csio_Init(CsioCh1, &stcCsio1Config);
    
    /*************************************************************************/
    /*                Master sends data to slave                             */
    /*************************************************************************/
    // Initialize and enable DMA
    MasterTxSlaveRxDmaInit();
    
    /* Enable RX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioRx);
    /* Enable TX function of CSIO0   */
    Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
///////////// 1st test   
    // Initialize send data
    for(u8i=0; u8i < SAMPLE_CSIO_MASTER_TX_BUFFSIZE; u8i++)
    {
        m_au8CsioMasterTxBuf[u8i] = u8i;
    }
    // Clear DMA transfer finish flag
    m_bDma0Finished = FALSE;
    m_bDma1Finished = FALSE;

    Mfs_Csio_EnableIrq(CsioCh0, CsioTxIrq);
    Mfs_Csio_EnableIrq(CsioCh1, CsioRxIrq);
    // Wait for TX finish
    while(m_bDma0Finished != TRUE)
    {};
    
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait for TX idle
    
    // Wait for RX finish
    while(m_bDma1Finished != TRUE)
    {};
    /* Compare receive data with transfer data */
    if(Error == CompareData(m_au8CsioMasterTxBuf, m_au8CsioSlaveRxBuf, SAMPLE_CSIO_SLAVE_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }

///////////// 2nd test 
    // Initialize send data
    for(u8i=0; u8i<SAMPLE_CSIO_MASTER_TX_BUFFSIZE; u8i++)
    {
        m_au8CsioMasterTxBuf[u8i] = ~u8i;
    }
    
    // Clear DMA transfer finish flag
    m_bDma0Finished = FALSE;
    m_bDma1Finished = FALSE;
    
    Mfs_Csio_EnableIrq(CsioCh0, CsioTxIrq);
    Mfs_Csio_EnableIrq(CsioCh1, CsioRxIrq);

    // Wait for TX finish
    while(m_bDma0Finished != TRUE)
    {};
    
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait for TX idle
    
    // Wait for RX finish
    while(m_bDma1Finished != TRUE)
    {};

    /* Compare receive data with transfer data */
    if(Error == CompareData(m_au8CsioMasterTxBuf, m_au8CsioSlaveRxBuf, SAMPLE_CSIO_SLAVE_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }

    pdl_memclr((uint8_t*)m_au8CsioMasterTxBuf, SAMPLE_CSIO_SLAVE_RX_BUFFSIZE);    
    pdl_memclr((uint8_t*)m_au8CsioSlaveRxBuf,  SAMPLE_CSIO_SLAVE_RX_BUFFSIZE);
    /* Disable TX function of CSIO0   */
    Mfs_Csio_DisableFunc(CsioCh0, CsioTx);
    /* Disable RX function of CSIO1   */
    Mfs_Csio_DisableFunc(CsioCh1, CsioRx);
    
    MasterTxSlaveRxDmaDeInit();
    
    /*************************************************************************/
    /*                Master receives data from slave                        */
    /*************************************************************************/ 
    // Initialize and enable DMA
    MasterRxSlaveTxDmaInit();

    /* Enable TX function of CSIO1   */
    Mfs_Csio_EnableFunc(CsioCh1, CsioTx);
    
    /* Enable TX and RX function of CSIO0   */
    Mfs_Csio_EnableFunc(CsioCh0, CsioTx);
    Mfs_Csio_EnableFunc(CsioCh0, CsioRx);
    
////////// 1st test       
    // Initialize send data
    for(u8i=0; u8i<SAMPLE_CSIO_SLAVE_TX_BUFFSIZE; u8i++)
    {
        m_au8CsioSlaveTxBuf[u8i] = u8i;
    }
    // Clear DMA transfer finish flag
    m_bDma0Finished = FALSE;
    m_bDma1Finished = FALSE;

    Mfs_Csio_EnableIrq(CsioCh0, CsioRxIrq);
    Mfs_Csio_EnableIrq(CsioCh1, CsioTxIrq);

    // Wait for TX finish
    while(m_bDma1Finished != TRUE)
    {
        while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty));
        Mfs_Csio_SendData(CsioCh0, 0x00u, FALSE); // Dummy write until DMA interrupt occurs
    };
    
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait for TX idle
    
    // Wait for RX finish
    while(m_bDma0Finished != TRUE)
    {};

    /* Compare receive data with transfer data */
    if(Error == CompareData(m_au8CsioMasterRxBuf, m_au8CsioSlaveTxBuf, SAMPLE_CSIO_MASTER_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }

    pdl_memclr((uint8_t*)m_au8CsioMasterRxBuf, SAMPLE_CSIO_SLAVE_RX_BUFFSIZE);    
    pdl_memclr((uint8_t*)m_au8CsioSlaveTxBuf,  SAMPLE_CSIO_SLAVE_RX_BUFFSIZE);

////////// 2nd test       
    // Initialize send data
    for(u8i=0; u8i<SAMPLE_CSIO_SLAVE_TX_BUFFSIZE; u8i++)
    {
        m_au8CsioSlaveTxBuf[u8i] = ~u8i;
    }
    // Clear DMA transfer finish flag
    m_bDma0Finished = FALSE;
    m_bDma1Finished = FALSE;
    
    Mfs_Csio_EnableIrq(CsioCh0, CsioRxIrq);
    Mfs_Csio_EnableIrq(CsioCh1, CsioTxIrq);
    // Wait for TX finish
    while(m_bDma1Finished != TRUE)
    {
        while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxEmpty));
        Mfs_Csio_SendData(CsioCh0, 0x00u, FALSE); // Dummy write until DMA interrupt occurs
    };
    
    while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle)); // wait for TX idle
    
    // Wait for RX finish
    while(m_bDma0Finished != TRUE)
    {};
    
    /* Compare receive data with transfer data */
    if(Error == CompareData(m_au8CsioMasterRxBuf, m_au8CsioSlaveTxBuf, SAMPLE_CSIO_MASTER_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }

    MasterRxSlaveTxDmaDeInit();
    
    Mfs_Csio_DeInit(CsioCh0, TRUE);
    Mfs_Csio_DeInit(CsioCh1, TRUE);
    
    while(1); /* Data is normally sent and received */
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
