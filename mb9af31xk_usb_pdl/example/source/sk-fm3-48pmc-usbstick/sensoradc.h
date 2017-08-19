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
/** \file sensoradc.h
 **
 ** Universal Sensor (ADC) routines for FM3 & FM4
 **
 ** History:
 **   - 2013-06-19  1.0  MSc  Universal Sensor (ADC) routines for FM3 & FM4
 **   - 2013-06-19  1.1  MSc  Updated conversion time
 **   - 2014-11-11  1.2  MSc  Updated compatibility with touch library, added FM0+
 **   - 2015-08-27  1.3  MSCH Added support for new MCU templates / MCU headerfile
 **   - 2016-03-09  1.4  MSCH Check for correct header file definitions added
 *****************************************************************************/

#ifndef __SENSORADC_H__
#define __SENSORADC_H__
#include "mcu.h"


#define SENSORADC_USEWITH_EVABOARD 1 
#define SENSORADC_USE_L3                                  0

#if (SENSORADC_USEWITH_EVABOARD == 1)
    #include "board.h"
#endif

#include "base_types.h"

#if !defined(FM_ADC0)  
    #if defined(FM0P_ADC0)
        #define FM_ADC0 FM0P_ADC0
    #elif defined(FM3_ADC0)
        #define FM_ADC0 FM3_ADC0
    #elif defined(FM4_ADC0)
        #define FM_ADC0 FM4_ADC0   
    #endif
#endif

#if !defined(FM_ADC1)  
    #if defined(FM0P_ADC1)
        #define FM_ADC1 FM0P_ADC1
    #elif defined(FM3_ADC1)
        #define FM_ADC1 FM3_ADC1
    #elif defined(FM4_ADC1)
        #define FM_ADC1 FM4_ADC1   
    #endif
#endif

#if !defined(FM_ADC2)  
    #if defined(FM0P_ADC2)
        #define FM_ADC1 FM0P_ADC2
    #elif defined(FM3_ADC2)
        #define FM_ADC2 FM3_ADC2
    #elif defined(FM4_ADC2)
        #define FM_ADC2 FM4_ADC2   
    #endif
#endif

#if defined(SENSOR_ADC_CH) && (SENSOR_ADC_CH == 0)
    #define SENSORADC_ADC FM_ADC0
#elif SENSOR_ADC_CH == 1
    #define SENSORADC_ADC FM_ADC1
#elif SENSOR_ADC_CH == 2
    #define SENSORADC_ADC FM_ADC2
#else
    #error SENSOR_ADC_CH must be between 0, 1, 2
#endif
   
#define CALC_OHM(x)  ((1000UL*x)/(x))
#define CALC_TEMP(x) (int8_t)((double)3977/(double)(log((double)x/(double)(1024-x))+(double)3977/(double)298)-273)
void SensorAdc_AdcHardwareInit(void);
uint16_t SensorAdc_Read(uint8_t u8Channel);
uint32_t SensorAdc_ReadSensorOhm(uint8_t u8Channel);

#endif /* __ADC_H__ */
