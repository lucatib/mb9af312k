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
 **   - 2014-11-15  1.0  VXu        First version for FM universal PDL.
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
/* Local function  ('static')                                      */
/******************************************************************************/
#if(PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
static void Crc16FormatBIntTest(void);
#endif
static void Crc16FormatATest(void);
static void Crc16FormatBTest(void);
static void Crc16FormatCTest(void);
static void Crc16FormatDTest(void);
static void CcittCrc16Test(void);
static void Crc32Test(void);

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

#if(PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
static boolean_t m_CrcIntFlag = FALSE;
#endif

/**
 ******************************************************************************
 ** \brief  Main function of project.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{    
#ifdef DEBUG_PRINT
    printf("************************************************\n");
    printf("*********Programable CRC test start! ***********\n\n");
#endif    

    /* Programmalbe CRC test iterm */
#if(PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
    Crc16FormatBIntTest(); // CRC16 Format  B with interrupt
#endif    
    Crc16FormatATest();    // CRC16 Format  A
    Crc16FormatBTest();    // CRC16 Format  B
    Crc16FormatCTest();    // CRC16 Format  C
    Crc16FormatDTest();    // CRC16 Format  D
    CcittCrc16Test();      // CCITT CRC16
    Crc32Test();           // CRC32
    
#ifdef DEBUG_PRINT
    printf("******Programable CRC all test item pass! ****\n");
    printf("************************************************\n");
#endif    
    
    while(1);
}

#if(PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
void CrcCallBack(void)
{
    m_CrcIntFlag = TRUE;
    
    return;
}

/**
 ******************************************************************************
 ** \brief Configure program crc to 16 CRC length, and format B (MSB-first/Little Endian) for test  
 **
 ** \param [in]  None
 ** \retval None
 **
 ******************************************************************************
 */
static void Crc16FormatBIntTest(void)
{
    stc_pcrc_config_t stcPcrcConfig = {0};
    uint32_t         u32CrcResult;

#ifdef DEBUG_PRINT
    printf("Crc16FormatBIntTest start!\n");
#endif

    /********************  Test 32-bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData32Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;
    
    stcPcrcConfig.bIrqEn = TRUE;
    stcPcrcConfig.pIrqCb = CrcCallBack;
    stcPcrcConfig.bTouchNvic = TRUE;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Format B Initial error !\n");
#endif
        while(1);
    }
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0xBC9A3512);
    
    while(FALSE == m_CrcIntFlag);
    Pcrc_WriteData(0xBACD3178);

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if(0x00000F1D != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU Calculates CRC16 Format B value : 0x%x\n", u32CrcResult);
        printf("CRC16 Format B right value : 0x00000F1D \n");
#endif
    }
    Pcrc_DeInit();

#ifdef DEBUG_PRINT
    printf("Crc16FormatBIntTest pass !\n\n");
#endif

    return;
}
#endif

/**
 ******************************************************************************
 ** \brief Configure program crc to 16 CRC length, and format A (MSB-first/Big Endian) for test  
 **
 ** \param [in]  None
 ** \retval None
 **
 ******************************************************************************
 */
static void Crc16FormatATest(void)
{
    stc_pcrc_config_t stcPcrcConfig = {0};
    uint32_t         u32CrcResult;

#ifdef DEBUG_PRINT
    printf("Crc16FormatATest start!\n");
#endif 

    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = MsbFirstBigEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstBigEndian;
    stcPcrcConfig.enInputDataSize  = InputData32Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;

    /********************  Test 32-bit pushing  ************************************/   
    while(Pcrc_GetLockStatus());

    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Format A Initial error !\n");
#endif
        while(1);
    }
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x12359ABC);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x7831CDBA);
    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if(0x1D0F0000 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU Calculate CRC16 Format A value : 0x%x\n", u32CrcResult);
        printf("CRC16 Format A right value : 0x1D0F0000 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();

    /********************  Test 24 and 16 bit pushing  ******************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = MsbFirstBigEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstBigEndian;
    stcPcrcConfig.enInputDataSize  = InputData24Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;
    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Format A Initial error !\n");
#endif
        while(1);
    }
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x12359A);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0xBC7831);
    while(Pcrc_GetLockStatus());
    
    Pcrc_SetInputDataSize(InputData16Bit);
    Pcrc_SetInputDataFormat(MsbFirstBigEndian);
    Pcrc_SetOutputDataFormat(MsbFirstBigEndian);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0xCDBA);
    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if(0x1D0F0000 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU Calculate CRC16 Format A value : 0x%x\n", u32CrcResult);
        printf("CRC16 Format A right value : 0x1D0F0000 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();

    /**************************  Test8 bit pushing  ********************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = MsbFirstBigEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstBigEndian;
    stcPcrcConfig.enInputDataSize  = InputData8Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;
    // 1. Test 8-bit pushing    
    while(Pcrc_GetLockStatus());

    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Format A Initial error !\n");
#endif
        while(1);
    }
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x12);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x35);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x9A);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0xBC);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x78);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x31);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0xCD);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0xBA);

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if(0x1D0F0000 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU Calculate CRC16 Format A value : 0x%x\n", u32CrcResult);
        printf("CRC16 Format A right value : 0x1D0F0000 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();
    
#ifdef DEBUG_PRINT
    printf("Crc16FormatATest pass !\n\n");
#endif 

    return;
}

/**
 ******************************************************************************
 ** \brief Configure program crc to 16 CRC length, and format B (MSB-first/Little Endian) for test  
 **
 ** \param [in]  None
 ** \retval None
 **
 ******************************************************************************
 */
static void Crc16FormatBTest(void)
{
    stc_pcrc_config_t stcPcrcConfig = {0};
    uint32_t         u32CrcResult;

#ifdef DEBUG_PRINT
    printf("Crc16FormatBTest start!\n");
#endif

    /********************  Test 32-bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData32Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Format B Initial error !\n");
#endif
        while(1);
    }
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0xBC9A3512);
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0xBACD3178);

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if(0x00000F1D != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU Calculates CRC16 Format B value : 0x%x\n", u32CrcResult);
        printf("CRC16 Format B right value : 0x00000F1D \n");
#endif
    }
    Pcrc_DeInit();

	stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData24Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;
	 // 1. Test 24-bit + 16 pushing    
	while(Pcrc_GetLockStatus());

	if (Ok != Pcrc_Init(&stcPcrcConfig))
	{
#ifdef DEBUG_PRINT
		printf("CRC16 Format B Initial error !\n");
#endif
		while(1);
	}
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x9A3512);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x3178BC);
  
	while(Pcrc_GetLockStatus());
	Pcrc_SetInputDataSize(InputData16Bit);
	Pcrc_SetInputDataFormat(MsbFirstLittleEndian);
	Pcrc_SetOutputDataFormat(MsbFirstLittleEndian);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0xBACD);
	while(Pcrc_GetLockStatus());
	u32CrcResult = Pcrc_ReadResult();
	if(0x00000F1D != u32CrcResult)
	{
#ifdef DEBUG_PRINT
		printf("MCU Calculate CRC16 Format B value : 0x%x\n", u32CrcResult);
		printf("CRC16 Format B right value : 0x00000F1D \n");
#endif
		while(1);
	}
	Pcrc_DeInit();


    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
	stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
	stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
	stcPcrcConfig.enInputDataSize  = InputData8Bit;
	stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
	stcPcrcConfig.u32FinalXorValue = 0x00000000;
	// 1. Test 8-bit pushing	
	while(Pcrc_GetLockStatus());


 if (Ok != Pcrc_Init(&stcPcrcConfig))
	{
#ifdef DEBUG_PRINT
		printf("CRC16 Format B Initial error !\n");
#endif
		while(1);
	}
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x12);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x35);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x9A);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0xBC);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x78);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x31);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0xCD);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0xBA);

	while(Pcrc_GetLockStatus());
	u32CrcResult = Pcrc_ReadResult();
	if(0x00000F1D != u32CrcResult)
	{
#ifdef DEBUG_PRINT
		printf("MCU Calculate CRC16 Format B value : 0x%x\n", u32CrcResult);
		printf("CRC16 Format B right value : 0x00000F1D \n");
#endif
		while(1);
	}
	Pcrc_DeInit();
    
#ifdef DEBUG_PRINT
        printf("Crc16FormatBTest pass !\n\n");
#endif 

    return;
}

/**
 ******************************************************************************
 ** \brief Configure program crc to 16 CRC length, and format C (LSB-first/Big Endian) for test  
 **
 ** \param [in]  None
 ** \retval None
 **
 ******************************************************************************
 */
static void Crc16FormatCTest(void)
{
    stc_pcrc_config_t stcPcrcConfig = {0};
    uint32_t         u32CrcResult;
    
#ifdef DEBUG_PRINT
    printf("Crc16FormatCTest start!\n");
#endif

    /********************  Test 32-bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = LsbFirstBigEndian;
    stcPcrcConfig.enOutputFormat = LsbFirstBigEndian;
    stcPcrcConfig.enInputDataSize  = InputData32Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0xFFFF0000;

    while(Pcrc_GetLockStatus());

    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Format C Initial error !\n");
#endif
        while(1);
    }
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x48AC593D);

    while(Pcrc_GetLockStatus());
    Pcrc_SetInputDataSize(InputData16Bit);
    Pcrc_SetInputDataFormat(LsbFirstBigEndian);
    Pcrc_SetOutputDataFormat(LsbFirstBigEndian);

    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x00001E8C);

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if(0xB35D0000 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU Calculate CRC16 Format C value : 0x%x\n", u32CrcResult);
        printf("CRC16 Format C right value : 0xB35D0000 \n");
#endif
    }
    Pcrc_DeInit();

	stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = LsbFirstBigEndian;
    stcPcrcConfig.enOutputFormat = LsbFirstBigEndian;
    stcPcrcConfig.enInputDataSize  = InputData24Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0xFFFF0000;
	 // 1. Test 24-bit + 16 pushing   
	 
    while(Pcrc_GetLockStatus());
	
	if (Ok != Pcrc_Init(&stcPcrcConfig))
	{
#ifdef DEBUG_PRINT
		printf("CRC16 Format C Initial error !\n");
#endif
		while(1);
	}
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x48AC59);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x3D1E8C);
  
	while(Pcrc_GetLockStatus());
	u32CrcResult = Pcrc_ReadResult();
	if(0xB35D0000 != u32CrcResult)
	{
#ifdef DEBUG_PRINT
		printf("MCU Calculate CRC16 Format C value : 0x%x\n", u32CrcResult);
		printf("CRC16 Format C right value : 0xB35D0000 \n");
#endif
		while(1);
	}
	Pcrc_DeInit();

    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = LsbFirstBigEndian;
    stcPcrcConfig.enOutputFormat = LsbFirstBigEndian;
    stcPcrcConfig.enInputDataSize  = InputData8Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0xFFFF0000;

    // 1. Test 8-bit pushing    
    while(Pcrc_GetLockStatus());
	if (Ok != Pcrc_Init(&stcPcrcConfig))
	{
#ifdef DEBUG_PRINT
		printf("CRC16 Format C Initial error !\n");
#endif
		while(1);
	}
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x48);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0xAC);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x59);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x3D);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x1E);
	while(Pcrc_GetLockStatus());
	Pcrc_WriteData(0x8C);

	while(Pcrc_GetLockStatus());
	u32CrcResult = Pcrc_ReadResult();
	if(0xB35D0000 != u32CrcResult)
	{
#ifdef DEBUG_PRINT
		printf("MCU Calculate CRC16 Format C value : 0x%x\n", u32CrcResult);
		printf("CRC16 Format C right value : 0xB35D0000 \n");
#endif
		while(1);
	}
	Pcrc_DeInit();
    
#ifdef DEBUG_PRINT
    printf("Crc16FormatCTest pass !\n\n");
#endif 

    return;
}

/**
 ******************************************************************************
 ** \brief Configure program crc to 16 CRC length, and D (LSB-first/Little Endian) for test  
 **
 ** \param [in]  None
 ** \retval None
 **
 ******************************************************************************
 */
static void Crc16FormatDTest(void)
{
    stc_pcrc_config_t stcPcrcConfig = {0};
    uint32_t         u32CrcResult;

#ifdef DEBUG_PRINT
    printf("Crc16FormatDTest start!\n");
#endif

    /********************  Test 32 and 16 bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;
    stcPcrcConfig.enInputFormat  = LsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = LsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData32Bit;
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x0000FFFF;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC16 Format D Initial error !\n");
#endif
        while(1);
    }
    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x3D59AC48);

    while(Pcrc_GetLockStatus());
    Pcrc_SetInputDataSize(InputData16Bit);
    Pcrc_SetInputDataFormat(LsbFirstLittleEndian);
    Pcrc_SetOutputDataFormat(LsbFirstLittleEndian);

    while(Pcrc_GetLockStatus());
    Pcrc_WriteData(0x00008C1E);

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if(0x00005DB3 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU Calculate CRC16 Format D value : 0x%x\n", u32CrcResult);
        printf("CRC16 Format D right value : 0x00005DB3 \n");
#endif
    }
    Pcrc_DeInit();
    
#ifdef DEBUG_PRINT
    printf("Crc16FormatDTest pass !\n\n");
#endif

    return;
}

/**
 ******************************************************************************
 ** \brief Configure program crc to CCITT CRC16 for test  
 **
 ** \param [in]  None
 ** \retval None
 **
 ******************************************************************************
 */
static void CcittCrc16Test(void)
{
    stc_pcrc_config_t stcPcrcConfig = {0};
    uint32_t         u32CrcResult;
    uint8_t          u8Count;

#ifdef DEBUG_PRINT
    printf("CcittCrc16Test start!\n");
#endif

    /********************  Test 32 bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;    //CCITT CRC16 : x16+x12+x5+1
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData32Bit;      // Input data size: 32 bit
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CCITT CRC16 Initial error !\n");
#endif
        while(1);
    }
    
    /********* Input data size: 32 bit  **********/
    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(au32ConstData[u8Count]);
    }

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if (0x000014B4 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU calculates CCITT CRC16 value : 0x%x\n", u32CrcResult);
        printf("CCITT CRC16 right value : 0x14B40000 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();

    /********************  Test 16 bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000; //CCITT CRC16 : x16+x12+x5+1
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData16Bit;    // Input data size: 16 bit
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CCITT CRC16 Initial error !\n");
#endif
        while(1);
    }
    
    /********* Input data size: 16 bit  **********/
    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x0000FFFF & au32ConstData[u8Count]);
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x0000FFFF & (au32ConstData[u8Count] >> 16));
    }

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if (0x000014B4 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU calculates CCITT CRC16 value : 0x%x\n", u32CrcResult);
        printf("CCITT CRC16 right value : 0x14B40000 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();

    /********************  Test 8 bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x10210000;     //CCITT CRC16 : x16+x12+x5+1
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData8Bit;         // Input data size: 8 bit
    stcPcrcConfig.u32CrcInitValue  = 0xFFFF0000;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CCITT CRC16 Initial error !\n");
#endif
        while(1);
    }
    
    /********* Input data size: 8 bit  **********/
    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x000000FF & au32ConstData[u8Count]);
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x000000FF & (au32ConstData[u8Count] >> 8));
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x000000FF & (au32ConstData[u8Count] >> 16));
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x000000FF & (au32ConstData[u8Count] >> 24));
    }

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if (0x000014B4 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU calculates CCITT CRC16 value : 0x%x\n", u32CrcResult);
        printf("CCITT CRC16 right value : 0x14B40000 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();
    
#ifdef DEBUG_PRINT
    printf("CcittCrc16Test pass !\n\n");
#endif

    return;
}

/**
 ******************************************************************************
 ** \brief Configure program crc to CRC32 for test  
 **
 ** \param [in]  None
 ** \retval None
 **
 ******************************************************************************
 */
static void Crc32Test(void)
{
    stc_pcrc_config_t stcPcrcConfig = {0};
    uint32_t         u32CrcResult;
    uint8_t          u8Count;

#ifdef DEBUG_PRINT
    printf("Crc32Test start!\n");
#endif

    /********************  Test 32 bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x04C11DB7;  //CRC32  x32+x26+x23+x22+x16+x12+x11+x10+x8+x7+x5+x4+x2+x+1
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData32Bit;     // Input data size: 32 bit
    stcPcrcConfig.u32CrcInitValue  = 0xFFFFFFFF;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC32 Initial error !\n");
#endif
        while(1);
    }
    /********* Input data size: 32 bit  **********/
    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(au32ConstData[u8Count]);
    }

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if (0x6AEA5AF1 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU calculates CRC32 value : 0x%x\n", u32CrcResult);
        printf("CRC32 right value : 0x6AEA5AF1 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();

    /********************  Test 16 bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x04C11DB7;  //CRC32  x32+x26+x23+x22+x16+x12+x11+x10+x8+x7+x5+x4+x2+x+1
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData16Bit;   // Input data size: 16 bit
    stcPcrcConfig.u32CrcInitValue  = 0xFFFFFFFF;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC32 Initial error !\n");
#endif
        while(1);
    }
    
    /********* Input data size: 16 bit  **********/
    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x0000FFFF & au32ConstData[u8Count]);
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x0000FFFF & (au32ConstData[u8Count] >> 16));
    }

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if (0x6AEA5AF1 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU calculates CRC32 value : 0x%x\n", u32CrcResult);
        printf("CRC32 right value : 0x6AEA5AF1 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();

    /********************  Test 8 bit pushing  ************************************/   
    stcPcrcConfig.u32GeneratorPolynomial = 0x04C11DB7;  //CRC32  x32+x26+x23+x22+x16+x12+x11+x10+x8+x7+x5+x4+x2+x+1
    stcPcrcConfig.enInputFormat  = MsbFirstLittleEndian;
    stcPcrcConfig.enOutputFormat = MsbFirstLittleEndian;
    stcPcrcConfig.enInputDataSize  = InputData8Bit;      // Input data size: 8 bit
    stcPcrcConfig.u32CrcInitValue  = 0xFFFFFFFF;
    stcPcrcConfig.u32FinalXorValue = 0x00000000;

    while(Pcrc_GetLockStatus());
    if (Ok != Pcrc_Init(&stcPcrcConfig))
    {
#ifdef DEBUG_PRINT
        printf("CRC32 Initial error !\n");
#endif
        while(1);
    }

    /********* Input data size: 8 bit  **********/
    for (u8Count = 0; u8Count < 64; u8Count++)
    {
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x000000FF & au32ConstData[u8Count]);
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x000000FF & (au32ConstData[u8Count] >> 8));
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x000000FF & (au32ConstData[u8Count] >> 16));
        while(Pcrc_GetLockStatus());
        Pcrc_WriteData(0x000000FF & (au32ConstData[u8Count] >> 24));
    }

    while(Pcrc_GetLockStatus());
    u32CrcResult = Pcrc_ReadResult();
    if (0x6AEA5AF1 != u32CrcResult)
    {
#ifdef DEBUG_PRINT
        printf("MCU calculates CRC32 value : 0x%x\n", u32CrcResult);
        printf("CRC32 right value : 0x6AEA5AF1 \n");
#endif
        while(1);
    }
    Pcrc_DeInit();

#ifdef DEBUG_PRINT
    printf("Crc32Test pass !\n\n");
#endif

    return;
}

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
