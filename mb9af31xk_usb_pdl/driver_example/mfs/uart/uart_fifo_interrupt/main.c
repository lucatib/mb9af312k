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
 ** interrupt mode using FIFO. 
 ** The SOT0_0 should be connected with SIN1_1 before using this example.
 **
 ** Attention: 
 ** ===========================================================================
 ** 1) This example only supports the MFS equipped with FIFO.
 ** 2) Change UART channel by modifying following:
 **    - The variable of "UartCh0" and "UartCh1"
 **    - The UART GPIO pin setting
 **    - Enable the used UART peripehral definitions in pdl_user.h
 ** ===========================================================================
 **
 ** History:
 **   - 2014-11-10  1.0  EZh         First version for FM universal PDL.
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
#define SAMPLE_UART_RX_BUFFSIZE sizeof(au8UartTxBuf)/sizeof(char)
#define SAMPLE_UART_TX_BUFFSIZE SAMPLE_UART_RX_BUFFSIZE

#if (PDL_MCU_CORE == PDL_FM0P_CORE)
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || (PDL_MCU_TYPE == PDL_FM0P_TYPE2)
#define SAMPLE_UART_FIFO_MAX_CAPACITY         (128u)
#else
#define SAMPLE_UART_FIFO_MAX_CAPACITY         (64u)
#endif
#elif (PDL_MCU_CORE == PDL_FM4_CORE)          
#define SAMPLE_UART_FIFO_MAX_CAPACITY         (64u)
#elif (PDL_MCU_CORE == PDL_FM3_CORE)          
#define SAMPLE_UART_FIFO_MAX_CAPACITY         (16u)
#endif

#define SAMPLE_UART_FIFO_RX_CNT               (8u)

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/// UART TX FIFO information structure
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
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
volatile stc_mfsn_uart_t* UartCh0 = &UART0;
volatile stc_mfsn_uart_t* UartCh1 = &UART1;
static uint8_t au8UartTxBuf[200] = "Data sent from UART0 to UART1";
static uint8_t au8UartRxBuf[SAMPLE_UART_RX_BUFFSIZE];
stc_tx_fifo_info_t stcTxFifoInfo = {0};
stc_rx_fifo_info_t stcRxFifoInfo = {0};

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
 ** \brief  UART0 FIFO TX interrupt callback function
 ******************************************************************************/
void Uart0FifoTxIntCallback(void)
{
    uint8_t u8i = 0;
    if(stcTxFifoInfo.u32TxCnt > SAMPLE_UART_FIFO_MAX_CAPACITY)
    {
        while(u8i < SAMPLE_UART_FIFO_MAX_CAPACITY)
        {
            Mfs_Uart_SendData(UartCh0, *stcTxFifoInfo.pTxBuf++);
            u8i++;
        }
        stcTxFifoInfo.u32TxCnt -= SAMPLE_UART_FIFO_MAX_CAPACITY;
        return;
    }
    
    while(u8i < stcTxFifoInfo.u32TxCnt)
    {
        Mfs_Uart_SendData(UartCh0, *stcTxFifoInfo.pTxBuf++);
        u8i++;
    }
  
     Mfs_Uart_DisableIrq(UartCh0, UartTxFifoIrq);
    
    stcTxFifoInfo.bTxFinish = TRUE;
}

/**
 ******************************************************************************
 ** \brief  UART0 RX interrupt callback function
 ******************************************************************************/
void Uart1RxIntCallback(void)
{
    uint8_t u8i = 0;
    if(stcRxFifoInfo.u32RxCnt > SAMPLE_UART_FIFO_RX_CNT) 
    {
        /* Receive data when RX FIFO count match with SAMPLE_UART_FIFO_RX_CNT */
        while(u8i < SAMPLE_UART_FIFO_RX_CNT)
        {
            *stcRxFifoInfo.pRxBuf++ = Mfs_Uart_ReceiveData(UartCh1); 
            u8i++;
        }
        stcRxFifoInfo.u32RxCnt -= SAMPLE_UART_FIFO_RX_CNT;
        return;
    }
    
    /* Receive data when RX FIFO is idle */
    /* idle means FIFO count is less than SAMPLE_UART_FIFO_RX_CNT and
       RX FIFO don't receive data from then on for a short time. */
    while(u8i < stcRxFifoInfo.u32RxCnt)
    {
        *stcRxFifoInfo.pRxBuf++ = Mfs_Uart_ReceiveData(UartCh1);
        u8i++;
    }
    
    Mfs_Uart_DisableIrq(UartCh1, UartRxIrq);
    
    stcRxFifoInfo.bRxFinish = TRUE;
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
    stc_mfs_fifo_config_t stcFifoConfig;
    stc_uart_irq_cb_t stcUart0IrqCb, stcUart1IrqCb;
 
    // Initialize the FIFO information   
    stcTxFifoInfo.u32TxCnt = SAMPLE_UART_TX_BUFFSIZE;
    stcTxFifoInfo.pTxBuf = au8UartTxBuf;
    stcTxFifoInfo.bTxFinish = FALSE;
    
    stcRxFifoInfo.u32RxCnt = SAMPLE_UART_RX_BUFFSIZE;
    stcRxFifoInfo.pRxBuf = au8UartRxBuf;
    stcRxFifoInfo.bRxFinish = FALSE;
    
    /* Initialize UART function I/O */
    SetPinFunc_SIN0_0();
    SetPinFunc_SOT0_0();
    SetPinFunc_SIN1_1();
    SetPinFunc_SOT1_1();

    // Clear all configuration structures
    PDL_ZERO_STRUCT(stcUartConfig);
    PDL_ZERO_STRUCT(stcFifoConfig);
    PDL_ZERO_STRUCT(stcUart0IrqCb);
    PDL_ZERO_STRUCT(stcUart1IrqCb);
    
    // Initialize FIFO configuration
    stcFifoConfig.enFifoSel = MfsTxFifo1RxFifo2;
    stcFifoConfig.u8ByteCount1 = 0;
    stcFifoConfig.u8ByteCount2 = 8;
    
    // Initialize interrupt callback structure
    stcUart0IrqCb.pfnTxFifoIrqCb = Uart0FifoTxIntCallback;
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
    stcUartConfig.pstcFifoConfig = &stcFifoConfig;
    stcUartConfig.bUseExtClk = FALSE;
    stcUartConfig.pstcIrqEn = NULL;
    stcUartConfig.pstcIrqCb = &stcUart0IrqCb;
    stcUartConfig.bTouchNvic = TRUE;
    Mfs_Uart_Init(UartCh0, &stcUartConfig);
    
    stcUartConfig.pstcIrqCb = &stcUart1IrqCb; // The configuration is same except interrupt callback functions
    Mfs_Uart_Init(UartCh1, &stcUartConfig);
    
    /* Enable RX function of UART0   */
    Mfs_Uart_EnableFunc(UartCh1, UartRx);
    /* Enable TX function of UART0   */
    Mfs_Uart_EnableFunc(UartCh0, UartTx);
    
    /* Configure interrupt */    
    Mfs_Uart_EnableIrq(UartCh1, UartRxIrq);
    Mfs_Uart_EnableIrq(UartCh0, UartTxFifoIrq);
    
    while(stcRxFifoInfo.bRxFinish != TRUE)
    {
        ;
    }
        
    Mfs_Uart_DeInit(UartCh0, TRUE);
    Mfs_Uart_DeInit(UartCh1, TRUE);
    
    /* Compare receive data with transfer data */
    if(Error == CompareData(au8UartTxBuf, au8UartRxBuf, SAMPLE_UART_RX_BUFFSIZE))
    {
        while(1);  /* If code runs here, the communicate error occurs */
    }
    
    while(1); /* Data is normally sent and received */
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
