/**************************************************************************
* Copyright (C)2011 Spansion LLC. All Rights Reserved . 
*
* This software is owned and published by: 
* Spansion LLC, 915 DeGuigne Dr. Sunnyvale, CA  94088-3453 ("Spansion").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND 
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software constitutes driver source code for use in programming Spansion's 
* Flash memory components. This software is licensed by Spansion to be adapted only 
* for use in systems utilizing Spansion's Flash memories. Spansion is not be 
* responsible for misuse or illegal use of this software for devices not 
* supported herein.  Spansion is providing this source code "AS IS" and will 
* not be responsible for issues arising from incorrect user implementation 
* of the source code herein.  
*
* SPANSION MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE, 
* REGARDING THE SOFTWARE, ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED 
* USE, INCLUDING, WITHOUT LIMITATION, NO IMPLIED WARRANTY OF MERCHANTABILITY, 
* FITNESS FOR A  PARTICULAR PURPOSE OR USE, OR NONINFRINGEMENT.  SPANSION WILL 
* HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT, NEGLIGENCE OR 
* OTHERWISE) FOR ANY DAMAGES ARISING FROM USE OR INABILITY TO USE THE SOFTWARE, 
* INCLUDING, WITHOUT LIMITATION, ANY DIRECT, INDIRECT, INCIDENTAL, 
* SPECIAL, OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA, SAVINGS OR PROFITS, 
* EVEN IF SPANSION HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.  
*
* This software may be replicated in part or whole for the licensed use, 
* with the restriction that this Copyright notice must be included with 
* this software, whether used in part or whole, at all times.  
******************************************************************************/
/** \file sensoradc.c
 **
 ** Universal Sensor (ADC) routines for FM0, FM3 & FM4
 **
 ** History:
 **   - 2013-06-19  1.0  MSc  Universal Sensor (ADC) routines for FM3 & FM4
 **   - 2013-06-19  1.1  MSc  Updated conversion time
 **   - 2014-11-11  1.2  MSc  Updated compatibility with touch library, added FM0+
 **   - 2015-08-27  1.3  MSCH Added support for new MCU templates / MCU headerfile
 *****************************************************************************/

#include "sensoradc.h"
#if (BOARD_SENSOR_ADC == ON)
#include "math.h"

/******************************************************************************/
/* Global variable definitions                                                */
/******************************************************************************/




/**
 ******************************************************************************
 ** Initializes ADC hardware abstraction layer
 **
 ** \return none
 **
 *****************************************************************************/
void SensorAdc_AdcHardwareInit(void)
{
    #if defined(FM4_PERIPH_BASE)
        FM4_GPIO->ADE   |= BOARD_SENSOR_ADC_MASK;
    #elif defined(FM3_PERIPH_BASE)
        FM3_GPIO->ADE   |= BOARD_SENSOR_ADC_MASK;
    #elif defined(FM0P_PERIPH_BASE)
        FM0P_GPIO->ADE   |= BOARD_SENSOR_ADC_MASK;
    #endif
    
    #if defined(FM_ADC0_SCIS23)
         SENSORADC_ADC->SCIS23 = (BOARD_SENSOR_ADC_MASK >> 16) & 0xFFFF; // Scan channel select AN31-AN16
    #else
         SENSORADC_ADC->SCIS3 = (BOARD_SENSOR_ADC_MASK >> 24) & 0xFF; // Scan channel select AN31-AN24
         SENSORADC_ADC->SCIS2 = (BOARD_SENSOR_ADC_MASK >> 16) & 0xFF; // Scan channel select AN23-AN16
    #endif
    #if defined(FM_ADC0_SCIS01)
         SENSORADC_ADC->SCIS01 = BOARD_SENSOR_ADC_MASK & 0xFF; // Scan channel select AN07-AN00 -> AN00
    #else
        SENSORADC_ADC->SCIS1 = (BOARD_SENSOR_ADC_MASK >> 8) & 0xFF; // Scan channel select AN15-AN08
        SENSORADC_ADC->SCIS0 = BOARD_SENSOR_ADC_MASK & 0xFF; // Scan channel select AN07-AN00 -> AN00
    #endif

    #if defined(FM_ADC0_ADST01)
        SENSORADC_ADC->ADST01 = 0x2F2F; // Sampling Time 0,1
    #else
        SENSORADC_ADC->ADST1 = 0x2F; // Sampling Time 1
        SENSORADC_ADC->ADST0 = 0x2F; // Sampling Time 0
    #endif
    
    #if defined(FM_ADC0_ADSS23)
        SENSORADC_ADC->ADSS23 = 0x0000; // Sampling Time Select AN31-AN16
    #else
        SENSORADC_ADC->ADSS3 = 0x00; // Sampling Time Select AN31-AN24
        SENSORADC_ADC->ADSS2 = 0x00; // Sampling Time Select AN23-AN16
    #endif
    
    #if defined(FM_ADC0_ADSS23)    
        SENSORADC_ADC->ADSS01 = 0x0000; // Sampling Time Select AN15-AN00
    #else
        SENSORADC_ADC->ADSS1 = 0x00; // Sampling Time Select AN15-AN08
        SENSORADC_ADC->ADSS0 = 0x00; // Sampling Time Select AN07-AN00 -> Use Samplin Time 0 for AN0
    #endif
    SENSORADC_ADC->ADCT  = 0x10; // Comparison Time = 14 x (ADCT + 2) / HCLK

    SENSORADC_ADC->ADCEN = 0x01; // Enable ADC    

    while (3 != SENSORADC_ADC->ADCEN); // wait until ADC operation is enabled

    SENSORADC_ADC->ADSR  = 0x00; // Stop ADC, Places conversion result on the MSB side. 

    SENSORADC_ADC->CMPCR = 0x00; // No comparsion
    SENSORADC_ADC->CMPD  = 0x00; // No comparsion value

    SENSORADC_ADC->SFNS  = 0x00; // Set Fifo Stage Count Interrupt
    SENSORADC_ADC->ADCR  = 0x00; // Disable ADC interrupts   
  
    SENSORADC_ADC->SCCR = 0x11; // FIFO clear, start ADC Single Conversion
}

/**
 ******************************************************************************
 ** Read ADC
 **
 ** \param u8Channel Adc Channel
 **
 ** \return value
 **
 *****************************************************************************/
uint16_t SensorAdc_Read(uint8_t u8Channel)
{
    #if defined(FM_ADC0_SCIS23)
         SENSORADC_ADC->SCIS23 = (BOARD_SENSOR_ADC_MASK >> 16) & 0xFFFF; // Scan channel select AN31-AN16
    #else
         SENSORADC_ADC->SCIS3 = (BOARD_SENSOR_ADC_MASK >> 24) & 0xFF; // Scan channel select AN31-AN24
         SENSORADC_ADC->SCIS2 = (BOARD_SENSOR_ADC_MASK >> 16) & 0xFF; // Scan channel select AN23-AN16
    #endif
    #if defined(FM_ADC0_SCIS01)
         SENSORADC_ADC->SCIS01 = BOARD_SENSOR_ADC_MASK & 0xFF; // Scan channel select AN15-AN00 -> AN00
    #else
        SENSORADC_ADC->SCIS1 = (BOARD_SENSOR_ADC_MASK >> 8) & 0xFF; // Scan channel select AN15-AN08
        SENSORADC_ADC->SCIS0 = BOARD_SENSOR_ADC_MASK & 0xFF; // Scan channel select AN07-AN00 -> AN00
    #endif
    SENSORADC_ADC->SCCR_f.RPT = 0;
    SENSORADC_ADC->SCCR_f.SSTR = 1; // Start ADC Single Conversion
    do
    {
     while(( 0 != ((SENSORADC_ADC->SCFDL) & 0x1000))) // Is valid data available?
     {
         
     }
    } while ((SENSORADC_ADC->SCFDL & 0x1F) != u8Channel);
    if ((SENSORADC_ADC->ADSR & 0x40) == 0)
    {
     return (SENSORADC_ADC->SCFD >> 20) & 0xFFF;
    }
    else
    {
     return (SENSORADC_ADC->SCFD >> 16) & 0xFFF;
    }
    //return ((uint16_t)(SENSORADC_ADC->SCFD >> 19)); //reads a ADC value
}

/**
 ******************************************************************************
 ** Read ADC in Ohm
 **
 ** \param u8Channel Adc Channel
 **
 ** \return value
 **
 *****************************************************************************/
uint32_t SensorAdc_ReadSensorOhm(uint8_t u8Channel)
{
    uint32_t u32Adc;
    u32Adc = SensorAdc_Read(u8Channel);    
    return CALC_OHM(u32Adc);
}
#endif /* (BOARD_SENSOR_ADC == ON) */

