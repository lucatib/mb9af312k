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
 ** This example demonstrates communication between UART0 and UART1 with
 ** interrupt mode using DMA.
 ** The SOT0_0 should be connected with SIN1_1 before using this example.
 **
 ** History:
 **   - 2014-03-10  1.0  EZh       First version for FM universal PDL.
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
#define SAMPLE_UART_RX_BUFFSIZE     (8u)
#define SAMPLE_UART_TX_BUFFSIZE     (8u)

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
volatile stc_mfsn_uart_t* UartCh0 = &UART0;
volatile stc_mfsn_uart_t* UartCh1 = &UART1;
stc_uart_irq_cb_t  stcUart0IrqCb, stcUart1IrqCb;
static uint8_t au8UartTxBuf[SAMPLE_UART_TX_BUFFSIZE];
static uint8_t au8UartRxBuf[SAMPLE_UART_RX_BUFFSIZE];
static boolean_t bDma0Finished, bDma1Finished;
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
 ** \brief  UART0 TX interrupt callback function
 ******************************************************************************/
void Uart0TxIntCallback(void)
{
    // Should never enter
}

/**
 ******************************************************************************
 ** \brief  UART0 RX interrupt callback function
 ******************************************************************************/
void Uart1RxIntCallback(void)
{
   // Should never enter
}

/**
 ******************************************************************************
 ** \brief  DMA Error callback function
 ******************************************************************************/
void Main_dma0_error_callback(uint8_t u8ErrorCode)
{
    // Error handling ... Should never happen.
}

/**
 ******************************************************************************
 ** \brief  DMNA finish callback function
 ******************************************************************************/
void Main_dma0_finish_callback(void)
{
    Mfs_Uart_DisableIrq(UartCh0, UartTxIrq);

    bDma0Finished = TRUE;         // Set DMA finished notification flag
}

/**
 ******************************************************************************
 ** \brief  DMA Error callback function
 ******************************************************************************/
void Main_dma1_error_callback(uint8_t u8ErrorCode)
{
    // Error handling ... Should never happen.
}

/**
 ******************************************************************************
 ** \brief  DMNA finish callback function
 ******************************************************************************/
void Main_dma1_finish_callback(void)
{
    Mfs_Uart_DisableIrq(UartCh1, UartRxIrq);

    bDma1Finished = TRUE;         // Set DMA finished notification flag
}


/**
 ******************************************************************************
 ** \brief  DMA configuration
 ******************************************************************************/
void Main_Dma_Init(void)
{
    stc_dma_config_t stcDma0Config, stcDma1Config;
    stc_dma_irq_en_t stcDma0IrqEn, stcDma1IrqEn;
    stc_dma_irq_cb_t  stcDma0IrqCb, stcDma1IrqCb;

    // Clear local configuration structure to zero.
    PDL_ZERO_STRUCT(stcDma0Config);
    PDL_ZERO_STRUCT(stcDma0IrqEn);
    PDL_ZERO_STRUCT(stcDma0IrqCb);
    PDL_ZERO_STRUCT(stcDma1Config);
    PDL_ZERO_STRUCT(stcDma1IrqEn);
    PDL_ZERO_STRUCT(stcDma1IrqCb);

    // Initialize interrupts
    stcDma0IrqEn.bCompleteIrq = 1u;
    stcDma0IrqEn.bErrorIrq = 1u;
    stcDma0IrqCb.pfnDmaCompletionIrqCb = Main_dma0_finish_callback;
    stcDma0IrqCb.pfnDmaErrorIrqCb = Main_dma0_error_callback;
    
    stcDma1IrqEn.bCompleteIrq = 1u;
    stcDma1IrqEn.bErrorIrq = 1u;
    stcDma1IrqCb.pfnDmaCompletionIrqCb = Main_dma1_finish_callback;
    stcDma1IrqCb.pfnDmaErrorIrqCb = Main_dma1_error_callback;
    
    // DMAC0 configuration
    stcDma0Config.enDmaIdrq = MfsTx0;
    stcDma0Config.u8BlockCount = 1u;
    stcDma0Config.u16TransferCount = SAMPLE_UART_TX_BUFFSIZE ;
    stcDma0Config.enTransferMode = DmaDemandTransfer;
    stcDma0Config.enTransferWdith = Dma8Bit;
    stcDma0Config.u32SourceAddress = (uint32_t)&(au8UartTxBuf[0u]); // UART data address
    stcDma0Config.u32DestinationAddress = MFS0_DATA_REG_ADDR;	   // Destination array's address
    stcDma0Config.bFixedSource = FALSE;
    stcDma0Config.bFixedDestination = TRUE;
    stcDma0Config.bReloadCount = TRUE;       // Reload count for next data package
    stcDma0Config.bReloadSource = TRUE;      // Reload source address for next data package
    stcDma0Config.bReloadDestination = FALSE;
    stcDma0Config.bEnableBitMask = TRUE;    // Don't clear enable bit after transfer completion
    stcDma0Config.pstcIrqEn = &stcDma0IrqEn;
    stcDma0Config.pstcIrqCb = &stcDma0IrqCb;
    stcDma0Config.bTouchNvic = TRUE;

    if (Ok == Dma_InitChannel(0u, &stcDma0Config))      // Initialize DMA channel 0
    {
         Dma_SetChannel(0u, TRUE, FALSE, FALSE);	      // Enable channel
    }

    // DMAC1 configuration
    stcDma1Config.enDmaIdrq = MfsRx1;
    stcDma1Config.u8BlockCount = 1u;
    stcDma1Config.u16TransferCount = SAMPLE_UART_RX_BUFFSIZE ;
    stcDma1Config.enTransferMode = DmaDemandTransfer;
    stcDma1Config.enTransferWdith = Dma8Bit;
    stcDma1Config.u32SourceAddress = MFS1_DATA_REG_ADDR ; // UART data address
    stcDma1Config.u32DestinationAddress = (uint32_t)&(au8UartRxBuf[0u]);	 // Destination array's address
    stcDma1Config.bFixedSource = TRUE;
    stcDma1Config.bFixedDestination = FALSE;
    stcDma1Config.bReloadCount = TRUE;       // Reload count for next data package
    stcDma1Config.bReloadSource = FALSE;
    stcDma1Config.bReloadDestination = TRUE; // Reload destination for next data package
    stcDma1Config.bEnableBitMask = TRUE;     // Don't clear enable bit after transfer completion
    stcDma1Config.pstcIrqEn = &stcDma1IrqEn;
    stcDma1Config.pstcIrqCb = &stcDma1IrqCb;
    stcDma1Config.bTouchNvic = TRUE;

    if (Ok == Dma_InitChannel(1u, &stcDma1Config))      // Initialize DMA channel 1
    {
        Dma_SetChannel(1u, TRUE, FALSE, FALSE);	      // Enable channel
    }

    Dma_Enable();            // Overall enable of DMA

}

/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    stc_mfs_uart_config_t stcUartConfig;
    uint8_t u8i;

    /* Initialize UART function I/O */
    SetPinFunc_SIN0_0();
    SetPinFunc_SOT0_0();
    SetPinFunc_SIN1_1();
    SetPinFunc_SOT1_1();
    
    /* Clear structures */
    PDL_ZERO_STRUCT(stcUart0IrqCb);
    PDL_ZERO_STRUCT(stcUart1IrqCb);
   
    /* Initialize interrupt callback functions */
    stcUart0IrqCb.pfnTxIrqCb = Uart0TxIntCallback;
    stcUart1IrqCb.pfnRxIrqCb = Uart1RxIntCallback;
    
    /* Initialize UART TX and RX channel  */
    stcUartConfig.enMode = UartNormal;
    stcUartConfig.u32BaudRate = 115200;
    stcUartConfig.enDataLength = UartEightBits;
    stcUartConfig.enParity = UartParityNone;
    stcUartConfig.enStopBit = UartOneStopBit;
    stcUartConfig.enBitDirection = UartDataLsbFirst;
    stcUartConfig.bInvertData = FALSE;
    stcUartConfig.bHwFlow = FALSE;
    stcUartConfig.pstcFifoConfig = NULL;
    stcUartConfig.pstcIrqEn = NULL;
    stcUartConfig.bTouchNvic = TRUE;
    
    stcUartConfig.pstcIrqCb = &stcUart0IrqCb;
    Mfs_Uart_Init(UartCh0, &stcUartConfig);
    
    stcUartConfig.pstcIrqCb = &stcUart1IrqCb;
    Mfs_Uart_Init(UartCh1, &stcUartConfig);

    // Initialize and enable DMA
    Main_Dma_Init();

    /* Enable RX function of UART0   */
    Mfs_Uart_EnableFunc(UartCh1, UartRx);
    /* Enable TX function of UART0   */
    Mfs_Uart_EnableFunc(UartCh0, UartTx);

/******************************************************/
/* Test 1st UART transfer/receive data with DMA       */
/******************************************************/
    // Initialize send data
    for(u8i=0; u8i<SAMPLE_UART_TX_BUFFSIZE; u8i++)
    {
        au8UartTxBuf[u8i] = u8i;
    }
    // Clear DMA transfer finish flag
    bDma0Finished = FALSE;
    bDma1Finished = FALSE;

    /* Configure interrupt */
    Mfs_Uart_EnableIrq(UartCh1, UartRxIrq);
    Mfs_Uart_EnableIrq(UartCh0, UartTxIrq);
    
    // Wait for TX finish
    while(bDma0Finished != TRUE)
    {};

    // Wait for RX finish
    while(bDma1Finished != TRUE)
    {};

    /* Compare receive data with transfer data */
    if(Error == CompareData(au8UartTxBuf, au8UartRxBuf, SAMPLE_UART_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }

/******************************************************/
/* Test 2nd UART transfer/receive data with DMA       */
/******************************************************/
    // Initialize send data
    for(u8i=0; u8i<SAMPLE_UART_TX_BUFFSIZE; u8i++)
    {
        au8UartTxBuf[u8i] = ~u8i;
    }
    // Clear DMA transfer finish flag
    bDma0Finished = FALSE;
    bDma1Finished = FALSE;

    // Configure interrupt
    Mfs_Uart_EnableIrq(UartCh1, UartRxIrq);
    Mfs_Uart_EnableIrq(UartCh0, UartTxIrq);

    // Wait for TX finish
    while(bDma0Finished != TRUE)
    {};

    // Wait for RX finish
    while(bDma1Finished != TRUE)
    {};

    /* Compare receive data with transfer data */
    if(Error == CompareData(au8UartTxBuf, au8UartRxBuf, SAMPLE_UART_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }

    Mfs_Uart_DeInit(UartCh0, TRUE);
    Mfs_Uart_DeInit(UartCh1, TRUE);

    Dma_Disable() ;
    Dma_DeInitChannel(0u, TRUE);
    Dma_DeInitChannel(1u, TRUE);

    while(1); /* Data is normally sent and received */
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
