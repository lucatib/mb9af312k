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
 ** This example demonstrates I2C data transfer and receive in slave mode. 
 ** This example must be used with the example "i2c_master_polling".
 ** Make following connection before using this example:
 ** Master (1st board "i2c_master_polling")   Slave (2nd board with "i2c_slave_polling")
 ** SCK0_0   ---------------------------     SCK1_1
 ** SOT0_0   ---------------------------     SOT1_1
 ** SCK1_1 and SOT1_1 must be pull-up in the slave node.
 **
 ** History:
 **   - 2014-11-18  1.0  EZh         First version for universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
#include "string.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/
#if !defined(SetPinFunc_SOT1_1) || !defined(SetPinFunc_SCK1_1)
#error SOT1_1 or SCK1_1 is not available on this product, please use other pins \
or channels and detele "me".
#endif

#define SAMPLE_I2C_MASTER_TX_BUFFSIZE   sizeof(au8I2cSlaveTxBuf)/sizeof(char)
#define SAMPLE_I2C_MASTER_RX_BUFFSIZE   sizeof(au8O2cSlaveRxBuf)/sizeof(char)

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
volatile stc_mfsn_i2c_t* I2cCh1 = &I2C1;
static uint8_t au8I2cSlaveTxBuf[50];
static uint8_t au8I2cSlaveRxBuf[50];

/*!
 ******************************************************************************
 ** \brief Dummy delay
 ** \param u32Cnt Time count
 ******************************************************************************
 */
void DelayTime(uint32_t u32Cnt)
{
   while(u32Cnt--) ;
}


/*!
 ******************************************************************************
 ** \brief Initialize I2C slave
 ******************************************************************************
 */
void InitI2cSlave(void)
{
    stc_mfs_i2c_config_t stcI2c1Config;
    
    PDL_ZERO_STRUCT(stcI2c1Config);
        
    SetPinFunc_SOT1_1();
    SetPinFunc_SCK1_1();
    
    stcI2c1Config.enMsMode = I2cSlave;
    stcI2c1Config.u32BaudRate = 100000u;
    stcI2c1Config.u8SlaveAddr = I2C_DEV_ADDR;
    stcI2c1Config.u8SlaveMaskAddr = 0x00u;
    stcI2c1Config.bWaitSelection = FALSE;
    stcI2c1Config.bDmaEnable = FALSE;
    stcI2c1Config.pstcFifoConfig = NULL;

    Mfs_I2c_Init(I2cCh1, &stcI2c1Config);

}

/*!
 ******************************************************************************
 ** \brief  Transmit the dedicated size data to device
 **
 ** \param  pu8TxData Transmit data pointer
 **
 ** \param  pu32Size the number of transmitted data
 **
 ** \return none
 ******************************************************************************
 */
void I2c_SlaveWriteData(uint8_t *pu8TxData, uint32_t *pu32Size)
{
    uint8_t i=0;
    while(1)
    {
        /* Transmit the data */
        Mfs_I2c_SendData(I2cCh1, pu8TxData[i++]);
        Mfs_I2c_ClrStatus(I2cCh1, I2cRxTxIrq);
        
        /* Wait for end of transmission */
        while(1)
        {
            if(Mfs_I2c_GetStatus(I2cCh1, I2cRxTxIrq) == TRUE)
            {
                break;
            }
        }
        
        while(1)
        {    
            if(Mfs_I2c_GetStatus(I2cCh1, I2cTxEmpty) == TRUE)
            {
                break;
            }
        }
        
        if(Mfs_I2c_GetAck(I2cCh1) == I2cAck)
        {
            ; /* continue to send */
        }
        else
        {
            /* Master tells that it is last data he wants to read, check the stop signal then */
            Mfs_I2c_ClrStatus(I2cCh1, I2cRxTxIrq); /* Release bus  */
            break;
        }
    }
    
    
    /* Check the stop condition */
    while(1)
    {
        if(Mfs_I2c_GetStatus(I2cCh1, I2cStopDetect) == TRUE)
        {            
            Mfs_I2c_ClrStatus(I2cCh1, I2cStopDetect);
            Mfs_I2c_ClrStatus(I2cCh1, I2cRxTxIrq);
            break;
        }
    }
    
    *pu32Size = i;
    
}
/*!
 ******************************************************************************
 ** \brief  Receive the dedicated size data to device
 **
 ** \param  pu8RxData The receive buffer pointer
 **
 ** \param  pu32Sizethe number of receiving data
 **
 ** \return none
 ******************************************************************************
 */
void I2c_SlaveReadData(uint8_t *pu8RxData, uint32_t *pu32Size)
{
    uint8_t i = 0;
    while(1)
    {
        /*Stop condition and NACK*/
        if(Mfs_I2c_GetStatus(I2cCh1, I2cStopDetect) == TRUE)
        {            
            Mfs_I2c_ClrStatus(I2cCh1, I2cStopDetect);
            Mfs_I2c_ClrStatus(I2cCh1, I2cRxTxIrq);
            
            break;
        }
        
        /*Bus Error*/
        if(Mfs_I2c_GetStatus(I2cCh1, I2cBusErr) == TRUE)
        {
            Mfs_I2c_ClrStatus(I2cCh1, I2cRxTxIrq);
            break;
        }
        
        Mfs_I2c_ClrStatus(I2cCh1, I2cRxTxIrq);
                
        if(Mfs_I2c_GetStatus(I2cCh1, I2cRxFull) == TRUE)
        {
            pu8RxData[i++] = Mfs_I2c_ReceiveData(I2cCh1);
            Mfs_I2c_ConfigAck(I2cCh1, I2cAck);
            Mfs_I2c_ClrStatus(I2cCh1, I2cRxTxIrq);
            
        }
    }
    
    *pu32Size = i;
}

/**
 ******************************************************************************
 ** \brief  Main function of project 
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{   
    uint32_t WriteLength, ReadLength;
    InitI2cSlave();
    
    while(1)
    {
        /* polling flag of first data*/
        if(TRUE == Mfs_I2c_GetStatus(I2cCh1, I2cDevAddrMatch))
        {
            if(i2c_slave_tx_master_rx == Mfs_I2c_GetDataDir(I2cCh1))//Tx
            {
                DelayTime(10);
                I2c_SlaveWriteData(au8I2cSlaveTxBuf, &WriteLength);
            }
            else//Rx
            {
                I2c_SlaveReadData(au8I2cSlaveRxBuf,&ReadLength);
                memset(au8I2cSlaveTxBuf, 0, sizeof(au8I2cSlaveTxBuf));
                memcpy(au8I2cSlaveTxBuf, au8I2cSlaveRxBuf, ReadLength);
            }
        }
    }
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
