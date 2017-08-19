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
#include "pdl_header.h"

/******************************************************************************
 * Local definitions
 ******************************************************************************/
#define   CR_TRIMMING_PROC_INIT                         0u
#define   CR_TRIMMING_PROC_TMAXCOARSE_MEAS_INIT         1u
#define   CR_TRIMMING_PROC_TMAXCOARSE_MEAS_START        2u
#define   CR_TRIMMING_PROC_TMAXCOARSE_MEAS_ONGOING      3u
#define   CR_TRIMMING_PROC_TMAXCOARSE_MEAS_FINISH       4u
#define   CR_TRIMMING_PROC_TMINCOARSE_MEAS_INIT         5u
#define   CR_TRIMMING_PROC_TMINCOARSE_MEAS_START        6u
#define   CR_TRIMMING_PROC_TMINCOARSE_MEAS_ONGOING      7u
#define   CR_TRIMMING_PROC_TMINCOARSE_MEAS_FINISH       8u
#define   CR_TRIMMING_PROC_CAL_COARSE_VAL               9u
#define   CR_TRIMMING_PROC_SET_COARSE_VAL               10u
#define   CR_TRIMMING_PROC_TUNE_COARSE_VAL              11u
#define   CR_TRIMMING_PROC_TMAXFINE_MEAS_INIT           21u
#define   CR_TRIMMING_PROC_TMAXFINE_MEAS_START          22u
#define   CR_TRIMMING_PROC_TMAXFINE_MEAS_ONGOING        23u
#define   CR_TRIMMING_PROC_TMAXFINE_MEAS_FINISH         24u
#define   CR_TRIMMING_PROC_TMINFINE_MEAS_INIT           25u
#define   CR_TRIMMING_PROC_TMINFINE_MEAS_START          26u
#define   CR_TRIMMING_PROC_TMINFINE_MEAS_ONGOING        27u
#define   CR_TRIMMING_PROC_TMINFINE_MEAS_FINISH         28u
#define   CR_TRIMMING_PROC_CAL_FINE_VAL                 29u
#define   CR_TRIMMING_PROC_SET_FINE_VAL                 30u
#define   CR_TRIMMING_PROC_TUNE_FINE_VAL                31u
#define   CR_TRIMMING_PROC_FINISH                       50u

#define   CR_TRIMMING_RET_OK        0u
#define   CR_TRIMMING_RET_ERR       1u      

#define PWC_CNT   32u

#define MEAN_CNT 10u
/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/   
typedef struct stc_coarse_cycle
{ 
    float32_t f32TTarget;
    float32_t f32TMaxCoarse;
    float32_t f32TMinCoarse;
  
}stc_coarse_cycle_t;

typedef struct stc_fine_cycle
{ 
    float32_t f32TTarget;
    float32_t f32TMaxFine;
    float32_t f32TMinFine;
  
}stc_fine_cycle_t;

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static uint32_t m_u32CntIrqMeasure, m_u32CntIrqOverflow;
static uint32_t m_aMeasureResult[10u];
static uint32_t u32SysClk;

/*!
 ******************************************************************************
 ** \brief  PWC measurement compete callback function
 *****************************************************************************
*/
static void PwcMeasCmpIrqHandler(void)
{
    m_aMeasureResult[m_u32CntIrqMeasure++] = Bt_Pwc_Get16BitMeasureData(&BT0);
}

/*!
 ******************************************************************************
 ** \brief  PWC overflow callback function
 *****************************************************************************
*/
static void PwcOverflowIrqHandler(void)
{
    m_u32CntIrqOverflow++;
}

/*!
 ******************************************************************************
 ** \brief  Calculate CR coarse value
 **
 ** \param [out] pu8CoarseData output the CR coarse value after calculation  
 ** \param [in] pstcCrCycle CR cycle information, refer to the calulation 
 **                         formula at the High-CR trimming chapter in PM.
 ** \return void
 **
 *****************************************************************************
*/
void CalCrCoarse(uint8_t* pu8CoarseData, stc_coarse_cycle_t* pstcCrCycle)
{   
    *pu8CoarseData = (uint8_t)((pstcCrCycle->f32TTarget - (pstcCrCycle->f32TMaxCoarse - pstcCrCycle->f32TMinCoarse)/
                                (float32_t)31.0f - pstcCrCycle->f32TMaxCoarse)/((pstcCrCycle->f32TMinCoarse - pstcCrCycle->f32TMaxCoarse)/(float32_t)31.0f));
}

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
                                (float32_t)31.0f - pstcCrCycle->f32TMaxFine)/((pstcCrCycle->f32TMinFine - pstcCrCycle->f32TMaxFine)/(float32_t)31.0f));
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
    stc_bt_pwc_config_t stcPwcConfig;
    stc_pwc_irq_en_t stcPwcIrqEn;
    stc_pwc_irq_cb_t  stcPwcIrqCallback;
    uint8_t  u8TempTrimData = u8DefaultTempTrimData, u8i; 
    uint16_t u16CrTrimData; 
    uint32_t u32MeasData;
    uint8_t u8CoarseData, u8FineData; 
    float32_t f32TMaxCoarse, f32TMinCoarse, f32TMaxFine, f32TMinFine, f32FinalCycle;
    stc_coarse_cycle_t stcCoarseCycle;
    stc_fine_cycle_t stcFineCycle;
    
    PDL_ZERO_STRUCT(stcPwcConfig);
    PDL_ZERO_STRUCT(stcPwcIrqEn);
    PDL_ZERO_STRUCT(stcPwcIrqCallback);
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
                bFM_GPIO_EPFR04_TIOB0S0 = 1u;
                bFM_GPIO_EPFR04_TIOB0S1 = 1u;
                bFM_GPIO_EPFR04_TIOB0S2 = 1u;
                /* Init PWC mode of BT0 */
                // Set IO Mode 
                Bt_ConfigIOMode(&BT0, BtIoMode0);

                // PWC register initialization 
                stcPwcConfig.enPres = PwcPresNone;
                stcPwcConfig.enMode = PwcContinuous;
                stcPwcConfig.enMeasureEdge = PwcMeasureRisingToRising;
                stcPwcConfig.enSize = PwcSize16Bit;
                Bt_Pwc_Init(&BT0, &stcPwcConfig);

                // Enable Irqerrupt 
                stcPwcConfig.pstcPwcIrqEn = &stcPwcIrqEn;
                stcPwcConfig.pstcPwcIrqCb = &stcPwcIrqCallback;
                stcPwcConfig.bTouchNvic =  TRUE;
                stcPwcConfig.pstcPwcIrqEn->bPwcMeasureCompleteIrq = TRUE;
                stcPwcConfig.pstcPwcIrqEn->bPwcMeasureOverflowIrq = TRUE;
                stcPwcIrqEn.bPwcMeasureCompleteIrq = 1u;
                stcPwcIrqEn.bPwcMeasureOverflowIrq = 1u;
                stcPwcIrqCallback.pfnPwcMeasureCompleteIrqCb = PwcMeasCmpIrqHandler;
                stcPwcIrqCallback.pfnPwcMeasureOverflowIrqCb = PwcOverflowIrqHandler;
                
                // Set PWC timer mode
                Bt_Pwc_Init(&BT0, &stcPwcConfig); 
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TMAXCOARSE_MEAS_INIT:
            case CR_TRIMMING_PROC_TMAXFINE_MEAS_INIT:  
                if(CR_TRIMMING_PROC_TMAXCOARSE_MEAS_INIT == u8TrimState)
                {
                    u8FineData = 0x00u;  
                    u8CoarseData = 0x00u;
                }
                else
                {
                    u8FineData = 0x00u; 
                }
                u16CrTrimData = (u8FineData & 0x1Fu) | ((u8CoarseData & 0x1Fu)<<5u);
#if (PDL_MCU_TYPE == PDL_FM3_TYPE8)  || (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE)  || \
    (PDL_MCU_CORE == PDL_FM4_CORE)  
                Cr_SetTempTrimmingData(u8TempTrimData);
#endif
                Cr_SetFreqTrimmingData(u16CrTrimData);
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TMAXCOARSE_MEAS_START:
            case CR_TRIMMING_PROC_TMAXFINE_MEAS_START:  
                m_u32CntIrqMeasure = 0;
                // Enable count operatoin 
                Bt_Pwc_EnableCount(&BT0);
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TMAXCOARSE_MEAS_ONGOING:
            case CR_TRIMMING_PROC_TMAXFINE_MEAS_ONGOING:
                while(MEAN_CNT > m_u32CntIrqMeasure)
                {
                    if(m_u32CntIrqOverflow)
                    {
                        // Disable count operatoin 
                        Bt_Pwc_DisableCount(&BT0);
                        return CR_TRIMMING_RET_ERR;
                    }
                }
                // Disable count operatoin 
                Bt_Pwc_DisableCount(&BT0);
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TMAXCOARSE_MEAS_FINISH:
            case CR_TRIMMING_PROC_TMAXFINE_MEAS_FINISH:  
                u32MeasData = 0u;
                for(u8i=0u;u8i<MEAN_CNT;u8i++)
                {
                    u32MeasData += m_aMeasureResult[u8i];
                }
                u32MeasData = u32MeasData/MEAN_CNT;
                if(CR_TRIMMING_PROC_TMAXCOARSE_MEAS_FINISH == u8TrimState)
                {
                    f32TMaxCoarse = (float32_t)u32MeasData/(float32_t)(u32SysClk*PWC_CNT );
                }
                else
                {
                    f32TMaxFine = (float32_t)u32MeasData/(float32_t)(u32SysClk*PWC_CNT );
                }
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TMINCOARSE_MEAS_INIT: 
            case CR_TRIMMING_PROC_TMINFINE_MEAS_INIT: 
                if(CR_TRIMMING_PROC_TMINCOARSE_MEAS_INIT == u8TrimState)
                {
                    u8FineData = 0x00u;  
                    u8CoarseData = 0x1Fu;
                }
                else
                {
                    u8FineData = 0x1Fu;  
                }
                u16CrTrimData = (u8FineData & 0x1Fu) | ((u8CoarseData & 0x1Fu)<<5u);
#if (PDL_MCU_TYPE == PDL_FM3_TYPE8)  || (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE)  || \
    (PDL_MCU_CORE == PDL_FM4_CORE)  
                Cr_SetTempTrimmingData(u8TempTrimData);
#endif
                Cr_SetFreqTrimmingData(u16CrTrimData);
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TMINCOARSE_MEAS_START:
            case CR_TRIMMING_PROC_TMINFINE_MEAS_START:  
                m_u32CntIrqMeasure = 0u;
                // Enable count operatoin 
                Bt_Pwc_EnableCount(&BT0);
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TMINCOARSE_MEAS_ONGOING:
            case CR_TRIMMING_PROC_TMINFINE_MEAS_ONGOING:  
                while(MEAN_CNT > m_u32CntIrqMeasure)
                {
                    if(m_u32CntIrqOverflow)
                    {
                        // Disable count operatoin 
                        Bt_Pwc_DisableCount(&BT0);
                        return CR_TRIMMING_RET_ERR;
                    }
                }
                // Disable count operatoin 
                Bt_Pwc_DisableCount(&BT0);
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TMINCOARSE_MEAS_FINISH:
            case CR_TRIMMING_PROC_TMINFINE_MEAS_FINISH:  
                u32MeasData = 0u;
                for(u8i=0;u8i<MEAN_CNT;u8i++)
                {
                    u32MeasData += m_aMeasureResult[u8i];
                }
                u32MeasData = u32MeasData/MEAN_CNT;
                if(CR_TRIMMING_PROC_TMINCOARSE_MEAS_FINISH == u8TrimState)
                {
                    f32TMinCoarse = (float32_t)u32MeasData/(float32_t)(u32SysClk*PWC_CNT );
                }
                else
                {
                    f32TMinFine = (float32_t)u32MeasData/(float32_t)(u32SysClk*PWC_CNT );
                }
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_CAL_COARSE_VAL:
            case CR_TRIMMING_PROC_CAL_FINE_VAL:
                if(CR_TRIMMING_PROC_CAL_COARSE_VAL == u8TrimState)
                {
                    stcCoarseCycle.f32TMaxCoarse = f32TMaxCoarse;
                    stcCoarseCycle.f32TMinCoarse = f32TMinCoarse;
                    stcCoarseCycle.f32TTarget = 1u/f32ExpCrFreq; 
                    CalCrCoarse(&u8CoarseData, &stcCoarseCycle);
                }
                else
                {
                    stcFineCycle.f32TMaxFine = f32TMaxFine;
                    stcFineCycle.f32TMinFine = f32TMinFine;
                    stcFineCycle.f32TTarget = 1/f32ExpCrFreq; 
                    CalCrFine(&u8FineData, &stcFineCycle);
                }
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_SET_COARSE_VAL:
            case CR_TRIMMING_PROC_SET_FINE_VAL:  
                if(CR_TRIMMING_PROC_SET_COARSE_VAL ==  u8TrimState)
                {
                    u8FineData = 0x00u;  
                    u16CrTrimData = (u8FineData & 0x1Fu) | ((u8CoarseData & 0x1Fu)<<5u);
                }
                else
                {
                    u16CrTrimData = (u8FineData & 0x1Fu) | ((u8CoarseData & 0x1Fu)<<5u);
                }
#if (PDL_MCU_TYPE == PDL_FM3_TYPE8)  || (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE)  || \
    (PDL_MCU_CORE == PDL_FM4_CORE)  
                Cr_SetTempTrimmingData(u8TempTrimData);
#endif
                Cr_SetFreqTrimmingData(u16CrTrimData);
                u8TrimState++;
                break;
            case CR_TRIMMING_PROC_TUNE_COARSE_VAL:
            case CR_TRIMMING_PROC_TUNE_FINE_VAL:  
                m_u32CntIrqMeasure = 0u;
                // Enable count operatoin 
                Bt_Pwc_EnableCount(&BT0);
                
                while(MEAN_CNT > m_u32CntIrqMeasure)
                {
                    if(m_u32CntIrqOverflow)
                    {
                        // Disable count operatoin 
                        Bt_Pwc_DisableCount(&BT0);
                        return CR_TRIMMING_RET_ERR;
                    }
                }
                
                // Disable count operatoin 
                Bt_Pwc_DisableCount(&BT0);
                
                u32MeasData = 0u;
                for(u8i=0u;u8i<MEAN_CNT;u8i++)
                {
                    u32MeasData += m_aMeasureResult[u8i];
                }
                u32MeasData = u32MeasData/MEAN_CNT;
                if(u8TrimState == CR_TRIMMING_PROC_TUNE_COARSE_VAL)
                {
                    f32FinalCycle = (float32_t)u32MeasData/(float32_t)(u32SysClk*PWC_CNT );
                    if (f32FinalCycle < 1u / f32ExpCrFreq) /* Freq > 4MHz, continue to tune */
                    {
                        if(u8CoarseData == 0u)
                        {
                            return CR_TRIMMING_RET_ERR;
                        }
                         
                        u8CoarseData--;
                        u8TrimState--;                        
                    }
                    else
                    {
 
                        u8TrimState = CR_TRIMMING_PROC_TMAXFINE_MEAS_INIT;
                    }
                }
                else
                {
                    f32FinalCycle = (float32_t)u32MeasData/(float32_t)(u32SysClk*PWC_CNT );  
                    
                    if(f32FinalCycle < 1u/f32ExpCrFreq) 
                    {
                        if(0u == u8FineData)
                        {
                            return CR_TRIMMING_RET_ERR;
                        }
                        u8FineData--;   /* Freq > 4MHz, sub fine data */
                    }
                    else
                    {
                        if(0x1Fu == u8FineData)
                        {
                            return CR_TRIMMING_RET_ERR;
                        }
                        u8FineData++;    /* Freq < 4MHz, add fine data */
                    }               
                                        
                    u8TrimState--;

                   if((f32FinalCycle > 1u / f32CrMaxFreq) && (f32FinalCycle < 1u / f32CrMinFreq))/* Freq > 4MHz, continue to tune */
                    {
                        u8TrimState = CR_TRIMMING_PROC_FINISH;
                    }

                }
                break;
            case CR_TRIMMING_PROC_FINISH:
                u16CrTrimData = (u8FineData & 0x1Fu) | ((u8CoarseData & 0x1Fu)<<5u);
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
            *pCrTrimmingData = u16CrTrimData | (u8TempTrimData << 16u) | (0x00u<<24u);
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
    uint8_t  u8TempTrimData = 0x10u; 
    uint32_t u32CrTrimData;
    float32_t f32CrMaxFreq = 4.005f;
    float32_t f32CrFreq = 4.000f;
    float32_t f32CrMinFreq = 3.995f;
    SystemCoreClockUpdate();
    u32SysClk = SystemCoreClock/2000000u; 
    SetPinFunc_CROUT_1();
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
