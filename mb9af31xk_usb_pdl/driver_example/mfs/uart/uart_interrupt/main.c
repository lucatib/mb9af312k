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
 ** interrupt mode. 
 ** The SOT0_0 should be connected with SIN1_1 before using this example.
 ** 
 ** History:
 **   - 2014-02-14  1.0  EZh         First version for FM universal PDL.
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

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
volatile stc_mfsn_uart_t* UartCh0 = &UART0;
volatile stc_mfsn_uart_t* UartCh1 = &UART1;
static uint8_t au8UartTxBuf[] = "Data sent from UART0 to UART1";
static uint8_t au8UartRxBuf[SAMPLE_UART_RX_BUFFSIZE];
static uint8_t u8TxCnt = 0, u8RxCnt = 0;
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
    Mfs_Uart_SendData(UartCh0, au8UartTxBuf[u8TxCnt]);  
    u8TxCnt++;
    if(u8TxCnt >= SAMPLE_UART_TX_BUFFSIZE )
    {
        /* Disable transfer interrupt of UART0 */
        Mfs_Uart_DisableIrq(UartCh0, UartTxIrq);
        return;
    }

}

/**
 ******************************************************************************
 ** \brief  UART0 RX interrupt callback function
 ******************************************************************************/
void Uart1RxIntCallback(void)
{
    au8UartRxBuf[u8RxCnt] = Mfs_Uart_ReceiveData(UartCh1);
    u8RxCnt++;
    if(u8RxCnt >= SAMPLE_UART_RX_BUFFSIZE )
    {
        /* Disable transfer interrupt of UART0 */
        Mfs_Uart_DisableIrq(UartCh1, UartRxIrq);
        return;
    }
 
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
    stc_uart_irq_cb_t stcUart0IrqCb, stcUart1IrqCb;
    
    /* Clear structures */
    PDL_ZERO_STRUCT(stcUartConfig);
    PDL_ZERO_STRUCT(stcUart0IrqCb);
    PDL_ZERO_STRUCT(stcUart1IrqCb);
    
    /* Initialize UART function I/O */
    SetPinFunc_SIN0_0();
    SetPinFunc_SOT0_0();
    SetPinFunc_SIN1_1();
    SetPinFunc_SOT1_1();

    /* Initail UART interrupt callback functions */
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
    stcUartConfig.bUseExtClk = FALSE;
    stcUartConfig.pstcIrqEn = NULL;
    stcUartConfig.pstcIrqCb = &stcUart0IrqCb;
    stcUartConfig.bTouchNvic = TRUE;
    Mfs_Uart_Init(UartCh0, &stcUartConfig);
    
    stcUartConfig.pstcIrqCb = &stcUart1IrqCb; // The configuration is all same except interrupt callback functions
    Mfs_Uart_Init(UartCh1, &stcUartConfig);
    
    /* Enable RX function of UART0   */
    Mfs_Uart_EnableFunc(UartCh1, UartRx);
    /* Enable TX function of UART0   */
    Mfs_Uart_EnableFunc(UartCh0, UartTx);
    
    /* Configure interrupt */    
    Mfs_Uart_EnableIrq(UartCh1, UartRxIrq);
    Mfs_Uart_EnableIrq(UartCh0, UartTxIrq);
    
    while(u8RxCnt < SAMPLE_UART_RX_BUFFSIZE)
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
