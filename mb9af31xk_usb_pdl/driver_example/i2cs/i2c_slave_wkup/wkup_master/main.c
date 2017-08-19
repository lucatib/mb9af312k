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
 ** This example is used to wakeup slave MCU by sending corresponding slave
 ** address or reserved address.
 ** 
 ** Make following connection before using this example:
 ** Master (1st board "wkup_master")   Slave (2nd board with "wkup_slave")
 ** SCK7_1   ---------------------------     SI2CSCL6_1
 ** SOT7_1   ---------------------------     SI2CSDA6_1
 **
 ** History:
 **   - 2015-07-08  1.0  EZh         First version for universal PDL.
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


#define I2C_RET_OK                  0
#define I2C_RET_ERROR               1

#define I2C_DEV_ADDR              (0xA0u)
#define I2C_DEV_ADDR_W            (0xA0u | 0u)
#define I2C_DEV_ADDR_R            (0xA0u | 1u)

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
volatile stc_mfsn_i2c_t* I2cCh = &I2C7;

/**
 ******************************************************************************
 ** \brief  Initialize I2C Master
 ******************************************************************************/
static void InitI2cMaster(void)
{
    stc_mfs_i2c_config_t stcI2c0Config;
        
    SetPinFunc_SOT7_1();
    SetPinFunc_SCK7_1();
    
    stcI2c0Config.enMsMode = I2cMaster;
    stcI2c0Config.u32BaudRate = 100000u;
    stcI2c0Config.bWaitSelection = FALSE;
    stcI2c0Config.bDmaEnable = FALSE;
    stcI2c0Config.pstcFifoConfig = NULL;
    
    Mfs_I2c_Init(I2cCh, &stcI2c0Config);

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
    Mfs_I2c_SendData(I2cCh, Addr);
    /* Generate I2C start signal */
    if(Ok != Mfs_I2c_GenerateStart(I2cCh))
    {
        return I2C_RET_ERROR; /* Timeout or other error */
    }

    while(1)
    {
        if(TRUE != Mfs_I2c_GetStatus(I2cCh, I2cRxTxIrq))
        {
            break;
        }
    }
   
    if(I2cNAck == Mfs_I2c_GetAck(I2cCh))
    {
        return I2C_RET_ERROR;   /* NACK */
    }
    
    if(TRUE == Mfs_I2c_GetStatus(I2cCh, I2cBusErr))
    {
        return I2C_RET_ERROR; /* Bus error occurs? */
    }
    
    if(TRUE == Mfs_I2c_GetStatus(I2cCh, I2cOverrunError))
    {
        return I2C_RET_ERROR; /* Overrun error occurs? */
    }
   
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
    
    while(1)
    {
        /* Master send salve address (Wakeup slave MCU by Slave address match) */
        I2c_Start(I2C_DEV_ADDR_W);
        
        /* Master send random address (Can't Wakeup slave MCU) */
        I2c_Start(0x3A);
        
        /* Master send reserved address (Wakeup slave MCU by reserved address) */
        I2c_Start(0xF0u);
        
        /* Master send reserved address (Wakeup slave MCU by reserved address) */
        I2c_Start(0x00u);
    }
}


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
