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
 ** This example demonstrates flash operation
 **
 ** History:
 **   - 2015-01-20 1.0  KXi First version for FM Universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#define TEST_SIZE 100
#define TEST_MAINSECTOR 0x4000
#define TEST_WORKSECTOR 0x200C0000
/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/
uint8_t u8TestData[TEST_SIZE];

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static void delay(void)
{
    uint32_t u32dly;
    u32dly = SystemCoreClock/5;
    while(u32dly--);
}
/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/


/**
 ******************************************************************************
 ** \brief  Main function of project for MCU evaluation board
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/

int32_t main(void)
{
    uint32_t i;
    uint8_t * pt;
    uint8_t *paddr = (uint8_t *)TEST_MAINSECTOR;
    pt = (uint8_t *)u8TestData;
    SystemCoreClockUpdate();
#ifdef DEBUG_PRINT
    //Uart_Io_Init();
#endif
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("main flash Example Program Start \n");
    printf("==================================================\n");
#endif
    delay();
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("main flash erase \n");
    printf("==================================================\n");
#endif
    delay();
    MFlash_SectorErase((uint16_t*)TEST_MAINSECTOR);
    MFlash_SectorErase((uint16_t*)(TEST_MAINSECTOR+4));
    for(i=0;i<TEST_SIZE;i++)
    {
        u8TestData[i] = i;
    }
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("main flash write 16bit data \n");
    printf("==================================================\n");
#endif
    delay();
    MFlash_WriteData16Bit((uint16_t*)TEST_MAINSECTOR, (uint16_t*)u8TestData, sizeof(u8TestData)/sizeof(uint16_t));
    for(i=0;i<TEST_SIZE;i++)
    {
        u8TestData[i] = 0xff;
    }
    for(i=0;i<TEST_SIZE;i++)
    {
        *pt++ = *paddr++;
    }
    for(i=0;i<TEST_SIZE;i++)
    {
        if(i != u8TestData[i])
        {
#ifdef DEBUG_PRINT  
            printf("main flash 16bit write data fail \n");
#endif 
	    while(1);
        }  
    }
#ifdef DEBUG_PRINT
    printf("main flash 16bit write data successfully \n");
#endif
  delay();
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("main flash erase \n");
    printf("==================================================\n");
#endif
    delay();
    MFlash_SectorErase((uint16_t*)TEST_MAINSECTOR);
    MFlash_SectorErase((uint16_t*)(TEST_MAINSECTOR+4));
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("main flash write 32bit data \n");
    printf("==================================================\n");
#endif
    for(i=0;i<TEST_SIZE;i++)
    {
        u8TestData[i] = TEST_SIZE - i;
    }
    delay();
    MFlash_WriteData32Bit((uint32_t*)TEST_MAINSECTOR, (uint32_t*)u8TestData, sizeof(u8TestData)/sizeof(uint32_t),TRUE);
    for(i=0;i<TEST_SIZE;i++)
    {
        u8TestData[i] = 0xff;
    }
    paddr = (uint8_t *)TEST_MAINSECTOR;
    pt = (uint8_t *)u8TestData;
    for(i=0;i<TEST_SIZE;i++)
    {
        *pt++ = *paddr++;
    }
    for(i=0;i<TEST_SIZE;i++)
    {
        if((TEST_SIZE-i) != u8TestData[i])
        {
#ifdef DEBUG_PRINT  
            printf("main flash 32bit write data fail \n");
#endif 
	    while(1);
        } 
    }
#ifdef DEBUG_PRINT
    printf("main flash 32bit write data successfully \n");
    printf("==================================================\n");
    printf("main flash Example Program End \n");
    printf("==================================================\n");
#endif

    paddr = (uint8_t *)TEST_WORKSECTOR;
    pt = (uint8_t *)u8TestData;
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("work flash Example Program Start \n");
    printf("==================================================\n");
#endif
    delay();
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("work flash erase \n");
    printf("==================================================\n");
#endif
    delay();
    WFlash_SectorErase((uint16_t*)TEST_WORKSECTOR);
      
    for(i=0;i<TEST_SIZE;i++)
    {
        u8TestData[i] = i;
    }
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("work flash write 16bit data \n");
    printf("==================================================\n");
#endif
    delay();
    WFlash_WriteData16Bit((uint16_t*)TEST_WORKSECTOR, (uint16_t*)u8TestData, sizeof(u8TestData)/sizeof(uint16_t));
    for(i=0;i<TEST_SIZE;i++)
    {
        u8TestData[i] = 0xff;
    }
    for(i=0;i<TEST_SIZE;i++)
    {
        *pt++ = *paddr++;
    }
    for(i=0;i<TEST_SIZE;i++)
    {
        if(i != u8TestData[i])
        {
#ifdef DEBUG_PRINT  
            printf("work flash 16bit write data fail \n");
#endif 
	    while(1);
        }  
    }
#ifdef DEBUG_PRINT
    printf("work flash 16bit write data successfully \n");
#endif 
  delay();
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("work flash erase \n");
    printf("==================================================\n");
#endif
    delay();
    WFlash_SectorErase((uint16_t*)TEST_WORKSECTOR);
#ifdef DEBUG_PRINT
    printf("==================================================\n");
    printf("work flash write 32bit data \n");
    printf("==================================================\n");
#endif
    for(i=0;i<TEST_SIZE;i++)
    {
        u8TestData[i] = TEST_SIZE - i;
    }
    delay();
    WFlash_WriteData32Bit((uint32_t*)TEST_WORKSECTOR, (uint32_t*)u8TestData, sizeof(u8TestData)/sizeof(uint32_t));
    for(i=0;i<TEST_SIZE;i++)
    {
        u8TestData[i] = 0xff;
    }
    paddr = (uint8_t *)TEST_WORKSECTOR;
    pt = (uint8_t *)u8TestData;
    for(i=0;i<TEST_SIZE;i++)
    {
        *pt++ = *paddr++;
    }
    for(i=0;i<TEST_SIZE;i++)
    {
        if((TEST_SIZE-i) != u8TestData[i])
        {
#ifdef DEBUG_PRINT  
            printf("work flash 32bit write data fail \n");
#endif 
	    while(1);
        } 
    }
#ifdef DEBUG_PRINT
    printf("work flash 32bit write data successfully \n");
    printf("==================================================\n");
    printf("work flash Example Program End \n");
    printf("==================================================\n");
#endif
  while(1);
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
