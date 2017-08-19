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
 ** This example demonstrates I2C data transfer and receive in master mode. 
 ** This example must be used with the example "i2c_slave_polling".
 ** Make following connection before using this example:
 ** Master (1st board "i2c_master_polling")   Slave (2nd board with "i2c_slave_polling")
 ** SCK0_0   ---------------------------     SCK1_1
 ** SOT0_0   ---------------------------     SOT1_1
 **
 ** History:
 **   - 2014-11-18  1.0  EZh         First version for universal PDL.
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
#if !defined(SetPinFunc_SOT0_0) || !defined(SetPinFunc_SCK0_0)
#error SOT0_0 or SCK0_0 is not available on this product, please use other \
pins or channels and detele "me".
#endif

#define SAMPLE_I2C_MASTER_TX_BUFFSIZE   sizeof(au8I2cMasterTxBuf)/sizeof(char)
#define SAMPLE_I2C_MASTER_RX_BUFFSIZE   SAMPLE_I2C_MASTER_TX_BUFFSIZE

#define I2C_RET_OK                  0
#define I2C_RET_ERROR               1

#define I2C_DEV_ADDR              (0x3Au)
#define I2C_DEV_ADDR_W            (0x3Au | 0u)
#define I2C_DEV_ADDR_R            (0x3Au | 1u)

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
volatile stc_mfsn_i2c_t* I2cCh0 = &I2C0;
static uint8_t au8I2cMasterTxBuf[10] = "0123456789";
static uint8_t au8I2cMasterRxBuf[SAMPLE_I2C_MASTER_RX_BUFFSIZE];

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
 ** \brief  Initialize I2C Master
 ******************************************************************************/
static void InitI2cMaster(void)
{
    stc_mfs_i2c_config_t stcI2c0Config;
        
    SetPinFunc_SOT0_0();
    SetPinFunc_SCK0_0();
    
    stcI2c0Config.enMsMode = I2cMaster;
    stcI2c0Config.u32BaudRate = 100000u;
    stcI2c0Config.bWaitSelection = FALSE;
    stcI2c0Config.bDmaEnable = FALSE;
    stcI2c0Config.pstcFifoConfig = NULL;
    
    Mfs_I2c_Init(I2cCh0, &stcI2c0Config);

}

/*!
 ******************************************************************************
 ** \brief  Config the start condition
 **
 ** \param  none
 **
 ** \return none
 ******************************************************************************
 */
static uint8_t I2c_Start(uint8_t Addr)
{
    /* Prepare I2C device address */
    Mfs_I2c_SendData(I2cCh0, Addr);
    /* Generate I2C start signal */
    if(Ok != Mfs_I2c_GenerateStart(I2cCh0))
    {
        return I2C_RET_ERROR; /* Timeout or other error */
    }

    while(1)
    {
        if(TRUE != Mfs_I2c_GetStatus(I2cCh0, I2cRxTxIrq))
        {
            break;
        }
    }
   
    if(I2cNAck == Mfs_I2c_GetAck(I2cCh0))
    {
        return I2C_RET_ERROR;   /* NACK */
    }
    
    if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cBusErr))
    {
        return I2C_RET_ERROR; /* Bus error occurs? */
    }
    
    if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cOverrunError))
    {
        return I2C_RET_ERROR; /* Overrun error occurs? */
    }
   
    return I2C_RET_OK;
}
/*!
 ******************************************************************************
 ** \brief  Transmit the dedicated size data to device
 **
 ** \param  pTxData Transmit data pointer
 **
 ** \param  u8Size the number of transmitted data
 **
 ** \return none
 ******************************************************************************
 */
static uint8_t I2c_Write(uint8_t *pTxData, uint8_t u8Size)
{
    uint8_t i;
    
    for(i=0;i<u8Size;i++)
    {
        /* Transmit the data */
        Mfs_I2c_SendData(I2cCh0, pTxData[i]);
        Mfs_I2c_ClrStatus(I2cCh0, I2cRxTxIrq);
        /* Wait for end of transmission */
        while(1)
        {
            if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cRxTxIrq))
            {
                break;
            }
        }
        
        while(1)
        {
            if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cTxEmpty))
            {
                break;
            }
        }
       
        if(I2cNAck == Mfs_I2c_GetAck(I2cCh0))
        {
            return I2C_RET_ERROR;   /* NACK */
        }
        
        if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cBusErr))
        {
            return I2C_RET_ERROR; /* Bus error occurs? */
        }
        
        if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cOverrunError))
        {
            return I2C_RET_ERROR; /* Overrun error occurs? */
        }
    }
    
    return I2C_RET_OK;
}
/*!
 ******************************************************************************
 ** \brief  Receive the dedicated size data to device
 **
 ** \param  pRxData The receive buffer pointer
 **
 ** \param  u8Size The number of receiving data
 **
 ** \return none
 ******************************************************************************
 */
static uint8_t I2c_Read(uint8_t *pRxData, uint8_t u8Size)
{
    uint8_t i = 0;
    /* Clear interrupt flag generated by device address send */
    Mfs_I2c_ClrStatus(I2cCh0, I2cRxTxIrq);
    
    if(I2cNAck == Mfs_I2c_GetAck(I2cCh0))
    {
        return I2C_RET_ERROR;   /* NACK */
    }
    
    while(i < u8Size)
    {   
        /* Wait for the receive data */
        while(1)
        {
            if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cRxTxIrq))
            {
                break;
            }
        }
        
        if(i == u8Size-1)
        {
            Mfs_I2c_ConfigAck(I2cCh0, I2cNAck); /* Last byte send a NACK */
        }
        else
        {
            Mfs_I2c_ConfigAck(I2cCh0, I2cAck);
        }
        
        /* Clear interrupt flag and receive data to RX buffer */
        Mfs_I2c_ClrStatus(I2cCh0, I2cRxTxIrq);
        
        /* Wait for the receive data */
        while(1)
        {
            if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cRxFull))
            {
                break;
            }
        }
         
        if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cBusErr))
        {
            return I2C_RET_ERROR;   /* Bus error occurs? */
        }
        
        if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cOverrunError))
        {
            return I2C_RET_ERROR;  /* Overrun error occurs? */
        }
        
        pRxData[i++] = Mfs_I2c_ReceiveData(I2cCh0);
    }
    return I2C_RET_OK;
}
/*!
 ******************************************************************************
 ** \brief  Config the stop condition
 **
 ** \param  none
 **
 ** \return none
 ******************************************************************************
 */
static uint8_t I2c_Stop(void)
{
    /* Generate I2C start signal */
    if(Ok != Mfs_I2c_GenerateStop(I2cCh0))
    {
        return I2C_RET_ERROR; /* Timeout or other error */
    }
    /* Clear Stop condition flag */
    while(1)
    {
        if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cStopDetect))
        {
            break;
        }
    }
   
    if(TRUE == Mfs_I2c_GetStatus(I2cCh0, I2cBusErr))
    {
        return I2C_RET_ERROR;
    }
   
    Mfs_I2c_ClrStatus(I2cCh0, I2cStopDetect);
    Mfs_I2c_ClrStatus(I2cCh0, I2cRxTxIrq);
    
    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Main function of project for FM family.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{     
    InitI2cMaster();
    
    /* I2C Master Write data */
    I2c_Start(I2C_DEV_ADDR_W);
    I2c_Write(au8I2cMasterTxBuf, SAMPLE_I2C_MASTER_TX_BUFFSIZE);
    I2c_Stop();
    
    /* I2C Master Read data */
    I2c_Start(I2C_DEV_ADDR_R);
    I2c_Read(au8I2cMasterRxBuf, SAMPLE_I2C_MASTER_RX_BUFFSIZE);
    I2c_Stop();
            
    if(Ok != CompareData(au8I2cMasterTxBuf, au8I2cMasterRxBuf, SAMPLE_I2C_MASTER_TX_BUFFSIZE))
    {
        while(1); /* Communication fails if code runs here. */
    }
    
    while(1); /* Data is normally sent and received */
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
