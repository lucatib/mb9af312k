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
 ** This example demonstrates how to access to AT24C02 with I2C interface.
 **
 ** 
 ** Use I2C polling method:
 **    - Set the "PDL_UTILITY_ENABLE_I2C_POLLING_AT24CXX" to "PDL_ON" in the  
 **      pdl_user.h
 **    - Set the "PDL_UTILITY_ENABLE_I2C_IRQ_AT24CXX" to "PDL_OFF" in the 
 **      pdl_user.h
 ** Use I2C interrupt method:
 **    - Set the "PDL_UTILITY_ENABLE_I2C_POLLING_AT24CXX" to "PDL_OFF" in the  
 **      pdl_user.h
 **    - Set the "PDL_UTILITY_ENABLE_I2C_INT_AT24CXX" to "PDL_ON" in the 
 **      pdl_user.h
 ** Note: Never set both definitions to "PDL_ON" at the same time.
 **
 ** History:
 **   - 2014-01-10  1.0  Edison Zhang        First release version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Global variable                                                            */
/******************************************************************************/
uint8_t au8WrBuf[256], au8RdBuf[256]; 
   
/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{  
    uint8_t u8DevAddr;
    uint8_t u8i;
    uint16_t u16j;
    
    u8DevAddr = AT24CXX_7BIT_DEVICE_ADDR;
    
    At24cxx_Init();
    
    /* Byte write and Random read */
    At24cxx_ByteWrite(u8DevAddr, 0x0000u, 0x55);
    At24cxx_Delayms(10);
    
    At24cxx_ByteWrite(u8DevAddr, 0x0080u, 0xAA);
    At24cxx_Delayms(10);
      
    At24cxx_RandomRead(u8DevAddr, 0x0000u, &au8RdBuf[0]);
    At24cxx_Delayms(10);
    At24cxx_RandomRead(u8DevAddr, 0x0080u, &au8RdBuf[1]);
    At24cxx_Delayms(10);
    
    /* Page write, Random read and squence read */
    for(u8i=0;u8i<AT24CXX_PAGE_SIZE;u8i++)
    {
        au8WrBuf[u8i] = 0x12+0x22*u8i;
    }
    
    At24cxx_PageWrite(u8DevAddr, 0x0008u, au8WrBuf, AT24CXX_PAGE_SIZE);
    At24cxx_Delayms(10);

    At24cxx_RandomRead(u8DevAddr, 0x0008u, &au8RdBuf[0]); // read first data 
    At24cxx_Delayms(10);
    
    At24cxx_SequentialRead(u8DevAddr, &au8RdBuf[1], (AT24CXX_PAGE_SIZE-1)); // read following data
    At24cxx_Delayms(10);
    
    /* Page write (only write 4bytes, from random address), Random read and squence read */
    for(u8i=0;u8i<4;u8i++)
    {
        au8WrBuf[u8i] = 0x12+0x11*u8i;
    }
    At24cxx_PageWrite(u8DevAddr, 0x0015u, au8WrBuf, 4);
    At24cxx_Delayms(10);
    
    At24cxx_RandomRead(u8DevAddr, 0x0015u, au8RdBuf); // read first data
    At24cxx_Delayms(10);
    
    At24cxx_SequentialRead(u8DevAddr, &au8RdBuf[1], 3); // read following data
    At24cxx_Delayms(10);
      
    /* Write whole chip of AT2402C, Random read and squence read  */
    for(u16j=0;u16j<256;u16j++)
    {
        au8WrBuf[u16j] = u16j;
    }
    
    for(u8i = 0; u8i<32; u8i++)
    {
        At24cxx_PageWrite(u8DevAddr, u8i*AT24CXX_PAGE_SIZE, &au8WrBuf[u8i*AT24CXX_PAGE_SIZE], AT24CXX_PAGE_SIZE);
        At24cxx_Delayms(10);
    }
    
    At24cxx_RandomRead(u8DevAddr, 0x0000u, &au8RdBuf[0]); // read first data 
    At24cxx_Delayms(10);
    
    At24cxx_SequentialRead(u8DevAddr, &au8RdBuf[1], 255); // read following data
    At24cxx_Delayms(10);
    
    At24cxx_CurrentAddrRead(u8DevAddr, &au8RdBuf[255]);  // should be the first data (at 0x0000)
    
    /* Erase whole chip */
    for(u16j = 0; u16j<256; u16j++)
    {
        At24cxx_ByteWrite(u8DevAddr, u16j, 0x00u);
        At24cxx_Delayms(10);
    }
    
    while(1)
    {}
}
