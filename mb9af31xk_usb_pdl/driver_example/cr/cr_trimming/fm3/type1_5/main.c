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
 ** Demonstrate the CR trimming procedure
 **
 ** History:
 **   - 2015-1-4  0.0.2  Ken xiao        modify with universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "math.h"
#include "pdl_header.h"

/******************************************************************************
 * Local definitions
 ******************************************************************************/                  

#define   CR_TRIMMING_RET_OK        0u
#define   CR_TRIMMING_RET_ERR       1u      

/*------------ MFT Unit -------------------------*/
/*! \brief User MFT unit */
#define USER_MFT_UNIT           (MFT_UNIT0)

/*------------ ICU -------------------------*/
/*! \brief User input capture unit connect FRT */
#define  USER_ICU_CONNECT_FRT   (USER_FRT_CH)
/*! \brief User input capture unit mode */
#define  USER_ICU_MODE          (IcuFallingDetect)
/*! \brief User input capture unit capture times */
#define  USER_ICU_CAP_TIMES     (10)
/*! \brief User IO MFT channel */
#define  USER_IO_MFT_CH         (USER_MFT_UNIT)
/*! \brief User IO ICU pin location */
#define  USER_IO_ICU_PIN_LOC    (IO_MFT_ICx3_CRTRIM)


//FRT
#define  USER_MFT_FRT_UNIT          MFT0_FRT  
/*! \brief define Mft Frt Channel */
#define  USER_MFT_FRT_CH         MFT_FRT_CH0
/*! \brief define Mft Frt cycle value */
#define  USER_FRT_CYCLE_VAL     0xffff

/*! \brief User free run timer mode */
#define  USER_FRT_MODE          FrtUpCount  


//ICU
#define  USER_MFT_ICU_UNIT       MFT0_ICU 
/*! \brief define Mft Frt Channel */
#define  USER_MFT_ICU_CH         MFT_ICU_CH3

#define PWC_CNT  40u

#define MEAN_CNT 100u
#define CHECK_CNT 20u

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/   
typedef enum em_cr_proc
{
   CR_TRIMMING_PROC_INIT,                                                                
   CR_TRIMMING_PROC_SET_FINE_VAL,                 
   CR_TRIMMING_PROC_TUNE_FINE_VAL,                
   CR_TRIMMING_PROC_FINISH,
   CR_TRIMMING_PROC_COUNT 
}em_cr_proc_t;

typedef struct stc_fine_cycle
{ 
    float32_t f32TTarget;
    float32_t f32TMaxFine;
    float32_t f32TMinFine;
  
}stc_fine_cycle_t;

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static uint32_t m_aMeasureResult[MEAN_CNT];
static uint32_t m_u32CntIrqMeasure=0u, m_u32CntIrqOverflow=0u;
static uint32_t u32SysClk;
static uint8_t m_u8ICUDumyDataCnt=0u;
static uint8_t m_u8CheckCouter = 0u;
static uint32_t u32dumy;
/*!
 ******************************************************************************
 ** \brief  Calculate CR fine value
 **
 ** \param [out] pu8CoarseData output the CR fine value after calculation  
 ** \param [in] pstcCrCycle CR cycle information, refer to the calulation 
 **                         formula at the High-CR trimming chapter in PM.
 ** \return void
 **
 *****************************************************************************
*/
void CalCrFine(uint8_t* pu8FineData, stc_fine_cycle_t* pstcCrCycle)
{
    *pu8FineData = (uint8_t)((pstcCrCycle->f32TTarget - (pstcCrCycle->f32TMaxFine - pstcCrCycle->f32TMinFine)/
                                (float32_t)31 - pstcCrCycle->f32TMaxFine)/((pstcCrCycle->f32TMinFine - pstcCrCycle->f32TMaxFine)/(float32_t)31));
}
/*!
 ******************************************************************************
 ** \brief  FRZ Zero match interrupt
 **
 ** \retval None
 ******************************************************************************
 */
static void FRTZeroIrqCallback(void)
{
    m_u32CntIrqOverflow++;
    
}
/*!
 ******************************************************************************
 ** \brief Mft Icu trigger interrupt handler
 ******************************************************************************
 */
static void MftIcuIrqHandler(void)
{  
    if(m_u8ICUDumyDataCnt>5)
    {
        m_aMeasureResult[m_u32CntIrqMeasure++] = Mft_Icu_GetCaptureData(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH);
    }
    else
    {
      u32dumy = Mft_Icu_GetCaptureData(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH);  
      m_u8ICUDumyDataCnt ++;
    }     
    Mft_Icu_ClrIrqFlag(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH);
    Mft_Frt_SetCountVal(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH,0);
    return;
}
/*!
 ******************************************************************************
 ** \brief FRT start
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void StartFRT(void)
{
    
    Mft_Frt_Start(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);
}
/*!
 ******************************************************************************
 ** \brief FRT stop
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void StopFRT(void)
{
    Mft_Frt_Stop(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);    
}
/*!
 ******************************************************************************
 ** \brief Enable ICU
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void EnableICU(void)
{
    m_u8ICUDumyDataCnt = 0;
    m_u32CntIrqMeasure = 0;
    Mft_Icu_ClrIrqFlag(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH);
    // Start Icu operation, Set detecting falling edge mode
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, USER_ICU_MODE);     
    Mft_Icu_EnableIrq(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, MftIcuIrqHandler, TRUE);
}

/*!
 ******************************************************************************
 ** \brief Disable ICU
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void DisableICU(void)
{
    // Stop FRT
    Mft_Icu_DisableIrq(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, FALSE);
    Mft_Frt_Stop(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH);    
    Mft_Frt_SetCountVal(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH, 0x0000);
    // Disable Mft Icu Operation and set mode to IcuDisable
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH,IcuDisable);    
}
/*!
 ******************************************************************************
 ** \brief FRT initialization
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void InitFRT(void)
{
    stc_mft_frt_config_t stcFrtConfig;
    stc_frt_irq_sel_t stcFrtIrqSel;
    stc_frt_irq_cb_t stcFrtIrqCallBack;

    // Clear structures
    PDL_ZERO_STRUCT(stcFrtConfig);
    PDL_ZERO_STRUCT(stcFrtIrqSel);
    PDL_ZERO_STRUCT(stcFrtIrqCallBack);
      
   
    //configure the FRT parameter
    stcFrtConfig.enFrtMode = USER_FRT_MODE;
    stcFrtConfig.enFrtClockDiv = FrtPclkDiv1;
    stcFrtConfig.bEnExtClock = FALSE;
    stcFrtConfig.bEnBuffer = FALSE;

    stcFrtConfig.bTouchNvic = FALSE;

    //configure frt interrupt mode
    stcFrtConfig.pstcIrqCb = &stcFrtIrqCallBack;
    stcFrtConfig.pstcIrqEn = &stcFrtIrqSel;
    stcFrtConfig.pstcIrqEn->bFrtZeroMatchIrq = TRUE;
    stcFrtConfig.pstcIrqCb->pfnFrtZeroIrqCb = FRTZeroIrqCallback;
    
    //initialize FRT
    Mft_Frt_Init(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH, &stcFrtConfig);
    
    // Set Frt count cycle
    Mft_Frt_SetCountCycle(&USER_MFT_FRT_UNIT, USER_MFT_FRT_CH, USER_FRT_CYCLE_VAL);
    
    
}
/*!
 ******************************************************************************
 ** \brief ICU initialization
 **
 ** \param None
 **
 ** \retval None
 **
 ******************************************************************************
 */
static void InitIcu(void)
{
    
    ///< set the internal macro pin CRTRIM for the input capture IC03
    bFM_GPIO_EPFR01_IC03S0 = 1;
    bFM_GPIO_EPFR01_IC03S1 = 1;
    bFM_GPIO_EPFR01_IC03S2 = 1; 
   
    // Select Frt channel
    Mft_Icu_SelFrt(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, (en_mft_icu_frt_t)USER_MFT_FRT_CH);
    
    // Disable Icu operation
    Mft_Icu_ConfigDetectMode(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, IcuDisable);
    
    // Enable Interrupt
    Mft_Icu_EnableIrq(&USER_MFT_ICU_UNIT, USER_MFT_ICU_CH, MftIcuIrqHandler, TRUE);
    
}

/*!
 ******************************************************************************
 ** \brief  High-cr trimming procedure
 **
 ** \param [in] f32ExpCrMinFreq  CR minimum frequency after trimming, unit:MHz
 ** \param [in] f32ExpCrMaxFreq  CR maximum frequency after trimming, unit:MHz 
 ** \param [in] f32ExpCrFreq  Expected CR value after trimming, unit:MHz
 ** \param [in] u8DefaultTempTrimData  Temperature trim data, should be
 **                                    regulated according to current temperature.
 ** \param [out] pCrTrimmingData Pointer to final trimming data
 **
 ** \return CR trimming status
 ** \retval CR_TRIMMING_RET_OK
 ** \reval  CR_TRIMMING_RET_ERR
 **
 *****************************************************************************
*/
uint8_t CrTrimmingProcess(float32_t f32CrMinFreq, 
                          float32_t f32CrMaxFreq,
                          float32_t f32ExpCrFreq,
                          uint8_t u8DefaultTempTrimData, 
                          uint32_t* pCrTrimmingData)
{
    uint8_t u8TrimState = CR_TRIMMING_PROC_INIT;
    uint8_t  u8TempTrimData = u8DefaultTempTrimData; 
    uint16_t u16CrTrimData, u16i; 
    float32_t u32MeasData;
    uint8_t  u8FineData; 
    float32_t f32FinalCycle;
    
    while(1)
    {
        switch(u8TrimState)
        {
            case CR_TRIMMING_PROC_INIT:
                /* Set CR dividor */
#if (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)
                Cr_SetFreqDiv(CrFreqDivBy512);
#else
                Cr_SetFreqDiv(CrFreqDivBy32);
#endif
                /* Use CR as input of TIOB */
                bFM_GPIO_EPFR04_TIOB0S0 = 1;
                bFM_GPIO_EPFR04_TIOB0S1 = 1;
                bFM_GPIO_EPFR04_TIOB0S2 = 1;
                
                // Init detection
                InitFRT();
                InitIcu(); 
                u8TrimState++;
                
                u8FineData = 0xf; 
            case CR_TRIMMING_PROC_SET_FINE_VAL:  
                u16CrTrimData = u8FineData & 0xFF;
#if (PDL_MCU_TYPE == PDL_FM3_TYPE8)  || (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE)  || \
    (PDL_MCU_CORE == PDL_FM4_CORE)  
                Cr_SetTempTrimmingData(u8TempTrimData);
#endif
                Cr_SetFreqTrimmingData(u16CrTrimData);
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TUNE_FINE_VAL:  
                // Enable count operatoin 
                EnableICU();
                StartFRT();
                while(m_u32CntIrqMeasure < MEAN_CNT)
                {

                  if(m_u32CntIrqOverflow)
                  {
                      // Disable count operatoin 
                      DisableICU();
                      StopFRT();
                      return CR_TRIMMING_RET_ERR;
                  }
                }
                
                // Disable count operatoin 
                DisableICU();
                StopFRT();
                u32MeasData = 0;
                for(u16i=0;u16i<MEAN_CNT;u16i++)
                {
                    u32MeasData += m_aMeasureResult[u16i];
                }
                u32MeasData = u32MeasData/MEAN_CNT;

                f32FinalCycle = (float32_t)u32MeasData/(float32_t)(u32SysClk*PWC_CNT);  
                
                if(f32FinalCycle < 1/f32ExpCrFreq) 
                {
                    if(u8FineData == 0 )
                    {
                        return CR_TRIMMING_RET_ERR;
                    }
                    u8FineData--;   /* Freq > 4MHz, sub fine data */
                }
                else 
                {
                    if(u8FineData == 0xFF)
                    {
                        return CR_TRIMMING_RET_ERR;
                    }
                    u8FineData++;   /* Freq < 4MHz, add fine data */ 
                }               
                                    
                u8TrimState--;

               if (f32FinalCycle > 1 / f32CrMaxFreq && f32FinalCycle < 1 / f32CrMinFreq) /* Freq > 4MHz, continue to tune */
                {
                    if(m_u8CheckCouter==CHECK_CNT)
                    {
                        u8TrimState = CR_TRIMMING_PROC_FINISH;
                    }
                    else
                    {
                        m_u8CheckCouter++;
                    }
                    
                }

                break;
            case CR_TRIMMING_PROC_FINISH:
                u16CrTrimData = u8FineData & 0xFF;
#if (PDL_MCU_TYPE == PDL_FM3_TYPE8)  || (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE)  || \
    (PDL_MCU_CORE == PDL_FM4_CORE)  
                Cr_SetTempTrimmingData(u8TempTrimData);
#endif
                Cr_SetFreqTrimmingData(u16CrTrimData);
                break;
        }
        
        if(u8TrimState == CR_TRIMMING_PROC_FINISH)
        {
            *pCrTrimmingData = u16CrTrimData | (u8TempTrimData << 16) | (0x00u<<24);
            break;
        }
    }
    
    return CR_TRIMMING_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    uint8_t  u8TempTrimData = 0x10; 
    uint32_t u32CrTrimData;
    float32_t f32CrMaxFreq = 4.005;
    float32_t f32CrFreq = 4.000;
    float32_t f32CrMinFreq = 3.995;
    u32SysClk = 4; 
#ifdef DEBUG_PRINT
   //Uart_Io_Init();
#endif
#ifdef DEBUG_PRINT
    printf("*************************************\n");
    printf("*            CR Trimming start      *\n");
    printf("*************************************\n");
#endif
    if( CR_TRIMMING_RET_OK != CrTrimmingProcess(f32CrMinFreq, f32CrMaxFreq, f32CrFreq, u8TempTrimData, &u32CrTrimData) )
    {
    #ifdef DEBUG_PRINT
        printf("CR Trimming failed!\n");
    #endif
        while(1);
    }
    
#ifdef DEBUG_PRINT
    printf("CR Trimming success!\n");
    printf("CR Trimming Data = 0x%08x\n", u32CrTrimData);
#endif

    // Uncomment following to watch CR output if needed
    // SetPinFunc_CROUT_1();

    // Use the Flash operation API to write the data into CR trimming data
    // address in the Flash.

    while(1);
}
