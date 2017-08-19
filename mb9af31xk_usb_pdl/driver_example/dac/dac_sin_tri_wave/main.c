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
 ** Main Module
 **
 ** \brief This example shows how to use the DAC
 **
 ** The example calculates 1 sine wave and 1 triangle forms into RAM arrays. 
 ** Later the data of these arrays are output to DAC channel 0 and channel 1.
 ** Note that the DAC outputs do not need any GPIO settings, so that this code
 ** will work without adjustment at any device supporting DAC output.
 **
 ** History:
 **   - 2014-11-24  1.0  RZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
#include <math.h>

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#define DAC_MAX_DATA 1024                       ///< Number of 'samples'

#define DAC_12BIT_ACCURACY              (0xFFFu)
#if (PDL_MCU_CORE == PDL_FM4_CORE)
#define DAC_BASE_ACCURACY               (0xFFFu)
#elif (PDL_MCU_CORE == PDL_FM3_CORE)
#define DAC_BASE_ACCURACY               (0x3FFu)
#elif
#error NO DAC MODULE AVAILABLE
#endif //#if (PDL_MCU_CORE == PDL_FM4_CORE)
 
#define DAC_SPECIFICVALUE         (DAC_12BIT_ACCURACY / DAC_BASE_ACCURACY) 
/******************************************************************************/
/* Global variable definitions                                                */
/******************************************************************************/
static uint16_t  au16Sin[DAC_MAX_DATA];
#if (PDL_MCU_TYPE != PDL_FM3_TYPE11)
static uint16_t  au16Triangle[DAC_MAX_DATA];
#endif //(PDL_MCU_TYPE != PDL_FM3_TYPE11)

const unsigned short au16SinTb[] = { 0x0000, //0
        0x0019, 0x0032, 0x004B, 0x0064, 0x007D, 0x0096, 0x00AF, 0x00C8, 0x00E2, 0x00FB,//10
        0x0114, 0x012D, 0x0146, 0x015F, 0x0178, 0x0191, 0x01AA, 0x01C3, 0x01DC, 0x01F5,//20
        0x020E, 0x0227, 0x0240, 0x0258, 0x0271, 0x028A, 0x02A3, 0x02BC, 0x02D4, 0x02ED,//30
        0x0306, 0x031F, 0x0337, 0x0350, 0x0368, 0x0381, 0x0399, 0x03B2, 0x03CA, 0x03E3,//40
        0x03FB, 0x0413, 0x042C, 0x0444, 0x045C, 0x0474, 0x048C, 0x04A4, 0x04BC, 0x04D4,//50
        0x04EC, 0x0504, 0x051C, 0x0534, 0x054C, 0x0563, 0x057B, 0x0593, 0x05AA, 0x05C2,//60
        0x05D9, 0x05F0, 0x0608, 0x061F, 0x0636, 0x064D, 0x0664, 0x067B, 0x0692, 0x06A9,//70
        0x06C0, 0x06D7, 0x06ED, 0x0704, 0x071B, 0x0731, 0x0747, 0x075E, 0x0774, 0x078A,//80
        0x07A0, 0x07B6, 0x07CC, 0x07E2, 0x07F8, 0x080E, 0x0824, 0x0839, 0x084F, 0x0864,//90
        0x087A, 0x088F, 0x08A4, 0x08B9, 0x08CE, 0x08E3, 0x08F8, 0x090D, 0x0921, 0x0936,//100
        0x094A, 0x095F, 0x0973, 0x0987, 0x099C, 0x09B0, 0x09C4, 0x09D7, 0x09EB, 0x09FF,//110
        0x0A12, 0x0A26, 0x0A39, 0x0A4D, 0x0A60, 0x0A73, 0x0A86, 0x0A99, 0x0AAB, 0x0ABE,//120
        0x0AD1, 0x0AE3, 0x0AF6, 0x0B08, 0x0B1A, 0x0B2C, 0x0B3E, 0x0B50, 0x0B61, 0x0B73,//130
        0x0B85, 0x0B96, 0x0BA7, 0x0BB8, 0x0BC9, 0x0BDA, 0x0BEB, 0x0BFC, 0x0C0C, 0x0C1D,//140
        0x0C2D, 0x0C3E, 0x0C4E, 0x0C5E, 0x0C6E, 0x0C7D, 0x0C8D, 0x0C9C, 0x0CAC, 0x0CBB,//150
        0x0CCA, 0x0CD9, 0x0CE8, 0x0CF7, 0x0D06, 0x0D14, 0x0D23, 0x0D31, 0x0D3F, 0x0D4D,//160
        0x0D5B, 0x0D69, 0x0D76, 0x0D84, 0x0D91, 0x0D9F, 0x0DAC, 0x0DB9, 0x0DC6, 0x0DD2,//170
        0x0DDF, 0x0DEB, 0x0DF8, 0x0E04, 0x0E10, 0x0E1C, 0x0E28, 0x0E33, 0x0E3F, 0x0E4A,//180
        0x0E55, 0x0E60, 0x0E6B, 0x0E76, 0x0E81, 0x0E8B, 0x0E96, 0x0EA0, 0x0EAA, 0x0EB4,//190
        0x0EBE, 0x0EC8, 0x0ED1, 0x0EDB, 0x0EE4, 0x0EED, 0x0EF6, 0x0EFF, 0x0F07, 0x0F10,//200
        0x0F18, 0x0F21, 0x0F29, 0x0F31, 0x0F39, 0x0F40, 0x0F48, 0x0F4F, 0x0F56, 0x0F5D,//210
        0x0F64, 0x0F6B, 0x0F72, 0x0F78, 0x0F7F, 0x0F85, 0x0F8B, 0x0F91, 0x0F96, 0x0F9C,//220
        0x0FA1, 0x0FA7, 0x0FAC, 0x0FB1, 0x0FB6, 0x0FBA, 0x0FBF, 0x0FC3, 0x0FC7, 0x0FCB,//230
        0x0FCF, 0x0FD3, 0x0FD7, 0x0FDA, 0x0FDE, 0x0FE1, 0x0FE4, 0x0FE7, 0x0FE9, 0x0FEC,//240
        0x0FEE, 0x0FF0, 0x0FF2, 0x0FF4, 0x0FF6, 0x0FF8, 0x0FF9, 0x0FFB, 0x0FFC, 0x0FFD,//250
        0x0FFE, 0x0FFE, 0x0FFF, 0x0FFF, 0x0FFF, 0x0FFF }; //256

/**
 ******************************************************************************
 ** \brief  Sine wave preparation and DAC usage
 ******************************************************************************/
void Main_dac( void )
{
    stc_dac_config_t stcDacConfig;
    uint16_t         u16Index;
    uint32_t         u32Delay;

    PDL_ZERO_STRUCT(stcDacConfig);


#if (PDL_MCU_CORE == PDL_FM4_CORE)
    stcDacConfig.bDac12Bit0 = TRUE;
    stcDacConfig.bDac12Bit1 = TRUE;
#endif
    // Calculate sine wave tone data
    for ( u16Index = 0u; u16Index < DAC_MAX_DATA; u16Index++ )
    {
        //make SIN Wave
      if(u16Index<256u)
        au16Sin[u16Index] = (uint16_t)((au16SinTb[u16Index%256u]/(DAC_SPECIFICVALUE*2u) + DAC_BASE_ACCURACY/2u));
      else if((u16Index>=256u) && (u16Index<512u))
        au16Sin[u16Index] = (uint16_t)(DAC_BASE_ACCURACY/2u + au16SinTb[(DAC_MAX_DATA-1-u16Index)%256u]/(DAC_SPECIFICVALUE*2u));
      else if((u16Index>=512u) && (u16Index<768u))
        au16Sin[u16Index] = (uint16_t)((DAC_BASE_ACCURACY/2u - au16SinTb[(u16Index)%256u]/(DAC_SPECIFICVALUE*2u) ));
      else if((u16Index>=768u) && (u16Index<DAC_MAX_DATA))
        au16Sin[u16Index] = (uint16_t)(DAC_BASE_ACCURACY/2u - au16SinTb[(DAC_MAX_DATA-1-u16Index)%256u]/(DAC_SPECIFICVALUE*2u));
        //make Triangle Form
#if (PDL_MCU_TYPE != PDL_FM3_TYPE11)
        au16Triangle[u16Index] = (uint16_t)(u16Index * 1.0f / DAC_MAX_DATA * DAC_BASE_ACCURACY);
#endif //(PDL_MCU_TYPE != PDL_FM3_TYPE11)
    }

    Dac_Init((stc_dacn_t *)&DAC0, &stcDacConfig);

    Dac_Enable0((stc_dacn_t *)&DAC0);
#if (PDL_MCU_TYPE != PDL_FM3_TYPE11)
    Dac_Enable1((stc_dacn_t *)&DAC0);
#endif //(PDL_MCU_TYPE != PDL_FM3_TYPE11)
    while (1)
    {
        for ( u16Index = 0; u16Index < DAC_MAX_DATA; u16Index++ )
        {
            Dac_SetValue0((stc_dacn_t *)&DAC0, au16Sin[u16Index]);
#if (PDL_MCU_TYPE != PDL_FM3_TYPE11)
            Dac_SetValue1((stc_dacn_t *)&DAC0, au16Triangle[u16Index]);
#endif //(PDL_MCU_TYPE != PDL_FM3_TYPE11)
            u32Delay = 15;
            while (--u32Delay);
        }
    }
}

/**
 ******************************************************************************
 ** \brief  Main function
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main( void )
{
    Main_dac();

    while (1);     ///< Will not be executed
}
