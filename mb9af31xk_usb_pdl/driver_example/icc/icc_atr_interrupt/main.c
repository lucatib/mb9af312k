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
 ** This example initializes a IC card interface with interrupt used and can  
 ** receive a ATR from ICC card via IC card interface.
 **
 ** History:
 **   - 2014-03-16  0.0.1  EZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/******************************************************************************/
/* Global variable                                                            */
/******************************************************************************/
#if (PDL_MCU_CORE == PDL_FM0P_CORE) // S6E100 EVB
#define IccCh   &ICC1
#define InitIccIo() {SetPinFunc_IC1_CLK_1(); SetPinFunc_IC1_VCC_1(); SetPinFunc_IC1_RST_1(); SetPinFunc_IC1_DATA_1(); SetPinFunc_IC1_CLK_1();}
#define ICC_CLK_DIV       10u  // Card clock = PCLK/ICC_CLK_DIV = 40000000/10 = 4000000Hz
#define ICC_RESET_TIME    10000
#else                              // SK-FM4-144L-S6E2GM
#define IccCh   &ICC1 
#define InitIccIo() {Gpio1pin_InitOut(GPIO1PIN_P16, Gpio1pin_InitVal(0u)); SetPinFunc_IC1_VCC_0(); SetPinFunc_IC1_RST_0(); SetPinFunc_IC1_DATA_0(); SetPinFunc_IC1_CLK_0();}
#define ICC_CLK_DIV     30u  // Card clock = PCLK/ICC_CLK_DIV = 90000000/30 = 3000000Hz
#define ICC_RESET_TIME    100000
#endif

/******************************************************************************/
/* Local variable                                                            */
/******************************************************************************/
static volatile uint16_t u16ICCardData[32];
static volatile uint16_t u16Counter = 0;

/**
 ******************************************************************************
 ** \brief Receive full interrupt call back function
 **
 ** \return none
 ******************************************************************************/
void ReceiveIntCallback(void)
{
    u16ICCardData[u16Counter] = Icc_ReceiveData(IccCh);    
    u16Counter++;
}

// Main fucntion
/**
 ******************************************************************************
 ** \brief Init the ICC IO and receive the ATR by interrupt (specified 15 bytes)
 **
 ** \return none
 ******************************************************************************/
void ICCard_ColdReset_ATR_Int(void)
{
    stc_icc_config_t stcIccConfig;
    stc_icc_irq_cb_t stcIccIrqCb;
    uint32_t u32Temp;
    
    // Init the variable 
    PDL_ZERO_STRUCT(stcIccConfig);
    PDL_ZERO_STRUCT(stcIccIrqCb);	
    
    // Initialize interrupt callback fucntion
    stcIccIrqCb.pfnRxFullIrqCb = ReceiveIntCallback;
    
    stcIccConfig.u16ClkDiv          = ICC_CLK_DIV;      // Card clock
    stcIccConfig.u16BaudRate        = 372u;             // 1 ETU = (F/D) * (1/CardClock[Hz]) = 372 * (1/Card clock)
    stcIccConfig.enTxDataFormat     = IccTx8Even2;      // TX data format: 8bit, Even parity, 2 stop bits
    stcIccConfig.enRxDataFormat     = IccRx8Even1;      // RX data format: 8bit, Even parity, 1 stop bits
    stcIccConfig.bUseGuardTimer     = TRUE;             // Enable gurad timer
    stcIccConfig.u8GuardTime        = 14u;              // Guard time count: 14
    stcIccConfig.enIdleTimerSrcClk  = IccCardClk;       // Idle timer clock sourced by card clock
    stcIccConfig.enBitDirection     = IccDataLsbFirst;  // LSB first
    stcIccConfig.bInvertData        = FALSE;            // No data invert
    stcIccConfig.bDataResend        = TRUE;             // Data resend enabled
    stcIccConfig.bClkPinMode        = FALSE;            // Clock pin controlled by hardware
    stcIccConfig.bDataPinMode       = FALSE;            // Data pin controlled by hardware
    stcIccConfig.bVpenPinOutputEn   = FALSE;            // WPEN pin output enable
    stcIccConfig.bVccPinOutputEn    = TRUE;             // VCC pin output enable
    stcIccConfig.bRstPinOutputEn    = TRUE;             // Reset pin output enable
    stcIccConfig.bDataPinOutputEn   = TRUE;             // Data pin output enable
    stcIccConfig.bClkPinOutputEn    = TRUE;             // Clock pin output enable
    stcIccConfig.bVpenPinLevel      = 0u;               // Set VPEN pin level to low
    stcIccConfig.bVccPinLevel       = 0u;               // Set VCC pin level to low
    stcIccConfig.bRstPinLevel       = 0u;               // Set Reset pin level to low
    stcIccConfig.bDataPinLevel      = 0u;               // Dummy setting as it is controlled by hardware
    stcIccConfig.bClkPinLevel       = 0u;               // Dummy setting as it is controlled by hardware
    stcIccConfig.pstcFifoConfig     = NULL;             // FIFO is unused
    stcIccConfig.pstcIrqEn          = NULL;             // Interrupt not enabled
    stcIccConfig.pstcIrqCb          = &stcIccIrqCb;     // Interrupt callback functions are set
    stcIccConfig.bTouchNvic         = TRUE;             // NVIC is enabled

    // Enable the ICC function 
    Icc_Enable(IccCh);
    
    // ICC setting configuration 
    if ( Ok != Icc_Init(IccCh,&stcIccConfig))
    {
        while(1);
    }

    // ICC IO configuration
    InitIccIo();
    
    // Set reset to low 
    Icc_SetRstPinLevel(IccCh, 0u);
    // Some delay 
    for (u32Temp=0;u32Temp<ICC_RESET_TIME;u32Temp++) {
    }
    
    // Set reset high 
    Icc_SetRstPinLevel(IccCh, 1u);
    
    // Enable interrupt 
    Icc_EnableIrq(IccCh, IccRxFullIrq);
    
    // Wait for receive end 
    while(u16Counter < 15); // Depend on the ATR data number of the card
}

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    ICCard_ColdReset_ATR_Int();
    while(1);
}
