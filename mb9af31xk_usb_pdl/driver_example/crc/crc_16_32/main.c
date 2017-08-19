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
 ** Main Module for CRC sample
 **
 ** History:
 **   - 2014-11-15  1.0  DHo        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
/* CRC table for check */
static const uint32_t au32ConstData[64] = {
    0x6393B370, 0xF2BB4FC0, 0x6D793D2C, 0x508B2092, 0x6697DDF6, 0xB7BF1228, 0xF7BB601A, 0x76D15ECA, 
    0x3C6FDC10, 0xA8C94C74, 0x65397F56, 0x29C5EA64, 0x9073F6CE, 0x76038E2E, 0xE183948A, 0xBB67E860, 
    0x5EABC934, 0xE5830890, 0x9D4D08D8, 0xEC1F28F8, 0xC4BBB344, 0x2C45998E, 0xEF7717DC, 0x13BFA29C, 
    0x72F92CF2, 0xE72BB6D6, 0x160512A6, 0xEA7DCCF6, 0xBD5FF930, 0xB40538CE, 0x972782C2, 0xF1FBDBE6, 
    0x4C8F38E0, 0x6A850590, 0x42312D34, 0x7E41DA84, 0x6ED15DE6, 0xF5499BF2, 0xD0A5F576, 0x5EBDA620, 
    0x091BFE16, 0xCAD17E80, 0x2727A07C, 0x274BC946, 0x2805863E, 0x068F7E80, 0x368577AE, 0x192936B4, 
    0x30D1CD26, 0x21D3E6F0, 0x7CB5CBF0, 0xABFFE464, 0xB269868E, 0x4CDB8780, 0x25D7589C, 0xB6DD4686, 
    0xD663962A, 0x67CB9FA2, 0xCD318688, 0x393DAA84, 0x71F342AA, 0x9BAFA978, 0xDE2F8EFA, 0xC3FF71FE
};

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    stc_crc_config_t stcCrcConfig;
    uint32_t         u32CrcResult;
    uint8_t          u8Count;
    uint8_t          u8CrcError;

    u8CrcError = 0;
    
#ifdef DEBUG_PRINT
    printf("CRC test start!\n");
#endif    
/******************************************************************************/
/*                 CRC32 Test                                                 */     
/******************************************************************************/    
    stcCrcConfig.enMode              = Crc32;
    stcCrcConfig.bUseDma             = FALSE;
    stcCrcConfig.bFinalXor           = FALSE;
    stcCrcConfig.bResultLsbFirst     = FALSE;
    stcCrcConfig.bResultLittleEndian = TRUE;
    stcCrcConfig.bDataLsbFirst       = FALSE;
    stcCrcConfig.bDataLittleEndian   = TRUE;
    stcCrcConfig.u32CrcInitValue     = 0xFFFFFFFF;

// 1. Test 32-bit pushing    
    if (Ok != Crc_Init(&stcCrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("Initial error1!\n");
#endif
        while(1);
    }

    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        Crc_Push32(au32ConstData[u8Count]);
    }

    u32CrcResult = Crc_ReadResult();
    if (0x6AEA5AF1 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("CRC32 Err (32bit pushing) : 0x%x\n", u32CrcResult);
#endif
        u8CrcError++;
    }
    Crc_DeInit();

// 2. Test 16-bit pushing    
    if (Ok != Crc_Init(&stcCrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("Initial error2!\n");
#endif
        while(1);
    }

    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        Crc_Push16((uint16_t)(0xFFFF & au32ConstData[u8Count]));
        Crc_Push16((uint16_t)(0xFFFF & (au32ConstData[u8Count] >> 16)));
    }

    u32CrcResult = Crc_ReadResult();
    if (0x6AEA5AF1 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("CRC32 Err (16bit pushing) : 0x%x\n", u32CrcResult);
#endif
        u8CrcError++;
    }
    Crc_DeInit();

// 3. Test 8-bit pushing    
    if (Ok != Crc_Init(&stcCrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("Initial error3!\n");
#endif
        while(1);
    }

    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        Crc_Push8((uint8_t)(0xFF & au32ConstData[u8Count]));
        Crc_Push8((uint8_t)(0xFF & (au32ConstData[u8Count] >> 8)));
        Crc_Push8((uint8_t)(0xFF & (au32ConstData[u8Count] >> 16)));
        Crc_Push8((uint8_t)(0xFF & (au32ConstData[u8Count] >> 24)));
    }

    u32CrcResult = Crc_ReadResult();
    if (0x6AEA5AF1 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("CRC32 Err (8bit pushing) : 0x%x\n", u32CrcResult);
#endif
        u8CrcError++;
    }
    Crc_DeInit();

/******************************************************************************/
/*                 CRC16 Test                                                 */     
/******************************************************************************/    
    stcCrcConfig.enMode              = Crc16;
    stcCrcConfig.bUseDma             = FALSE;
    stcCrcConfig.bFinalXor           = FALSE;
    stcCrcConfig.bResultLsbFirst     = FALSE;
    stcCrcConfig.bResultLittleEndian = TRUE;
    stcCrcConfig.bDataLsbFirst       = FALSE;
    stcCrcConfig.bDataLittleEndian   = TRUE;
    stcCrcConfig.u32CrcInitValue     = 0xFFFFFFFF;
    
// 1. Test 32-bit pushing    
    if (Ok != Crc_Init(&stcCrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("Initial error4!\n");
#endif
        while(1);
    }


    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        Crc_Push32(au32ConstData[u8Count]);
    }

    u32CrcResult = Crc_ReadResult();
    if (0x14B40000 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Err (32bit pushing) : 0x%x\n", u32CrcResult);
#endif
        u8CrcError++;
    }
    Crc_DeInit();

// 2. Test 16-bit pushing    
    if (Ok != Crc_Init(&stcCrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("Initial error5!\n");
#endif
        while(1);
    }

    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        Crc_Push16((uint16_t)(0xFFFF & au32ConstData[u8Count]));
        Crc_Push16((uint16_t)(0xFFFF & (au32ConstData[u8Count] >> 16)));
    }

    u32CrcResult = Crc_ReadResult();
    if (0x14B40000 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Err (16bit pushing) : 0x%x\n", u32CrcResult);
#endif
        u8CrcError++;
    }
    Crc_DeInit();

// 3. Test 8-bit pushing    
    if (Ok != Crc_Init(&stcCrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("Initial error6!\n");
#endif
        while(1);
    }

    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        Crc_Push8((uint8_t)(0xFF & au32ConstData[u8Count]));
        Crc_Push8((uint8_t)(0xFF & (au32ConstData[u8Count] >> 8)));
        Crc_Push8((uint8_t)(0xFF & (au32ConstData[u8Count] >> 16)));
        Crc_Push8((uint8_t)(0xFF & (au32ConstData[u8Count] >> 24)));
    }

    u32CrcResult = Crc_ReadResult();
    if (0x14B40000 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Err (8bit pushing) : 0x%x\n", u32CrcResult);
#endif
        u8CrcError++;
    }
    Crc_DeInit();

    
    if (0 != u8CrcError)
    {
#ifdef DEBUG_PRINT
        printf("Total CRC error count : %d\n", u8CrcError);
#endif
    }
    else
    {
#ifdef DEBUG_PRINT
        printf("CRC test finish normally!\n");
#endif    
    }
    
    while(1);
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
