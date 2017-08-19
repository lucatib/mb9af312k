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
 ** This example shows how to use I2S to play wave file via WM8731. 
 ** This example is used with S6E2CC start-kit (SK-FM4-176L-S6E2CC) or 
 ** S6E2DH start-kit (SK-FM4-176L-S6E2DH)
 **
 ** History:
 **   - 2014-11-10  1.0  EZh       First release version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
#include "pixiedust_sound_i2s.h"   

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
const uint16_t *pu16SoundData;
boolean_t bPlaySoundActive = FALSE;

#if (PDL_MCU_CORE == PDL_FM4_CORE)
#if (PDL_MCU_TYPE == PDL_FM4_TYPE1) || (PDL_MCU_TYPE == PDL_FM4_TYPE2)
#error I2S is not available for FM4 TYPE1 and TYPE2 products.  
#endif
#else
#error Only FM4 Support I2S module. 
#endif
   
/**
 ******************************************************************************
 ** \brief  This funciton outputs I2S data when I2S TX FIFO have enough free
 **         space.
 ******************************************************************************/
void I2sTxFifoCallback(void)
{
    uint16_t u16OutLeft;  
    uint16_t u16OutRight;
    uint32_t u32Fifo;
  
    if (TRUE == bPlaySoundActive)
    {
        u16OutLeft =  *pu16SoundData++;
        u16OutRight = *pu16SoundData++;
      
        u32Fifo = u16OutLeft | (u16OutRight << 16);
        
        I2s_WriteTxFifo(&I2S0, u32Fifo);
      
        if (0xFFFFu == *pu16SoundData)     // End of sound reached?
        {
            bPlaySoundActive = FALSE;
        }
    }
    else
    {
        I2s_WriteTxFifo(&I2S0, 0x80008000u);
    }
}

/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    stc_i2s_clk_config_t stcI2sClkConfig;
    stc_i2s_config_t stcI2sConfig;
    stc_i2s_irq_en_t stcI2sIrqEn;
    stc_i2s_irq_cb_t stcI2sIrqCb;
    stc_wm8731_reg_t stcWm8731Reg;
    volatile stc_mfsn_i2c_t* I2cCh;
    
    // Clear structures
    PDL_ZERO_STRUCT(stcI2sClkConfig);
    PDL_ZERO_STRUCT(stcI2sConfig);
    PDL_ZERO_STRUCT(stcI2sIrqEn);
    PDL_ZERO_STRUCT(stcI2sIrqCb);
    PDL_ZERO_STRUCT(stcWm8731Reg);
    
    // Initialize GPIO
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2CCX)    // SK-FM4-176L-S6E2CC 
    SetPinFunc_I2SCK_0();
    SetPinFunc_I2SDO_0();
    SetPinFunc_I2SWS_0();
    SetPinFunc_I2SMCLK_0_IN();
    
    SetPinFunc_SOT2_1(); 
    SetPinFunc_SCK2_1();
    
    Gpio1pin_InitOut(GPIO1PIN_P3C, Gpio1pin_InitVal(0u)); // Codec controls WS
    Gpio1pin_InitIn(GPIO1PIN_P20, Gpio1pin_InitPullup(1u)); //  key
    
    I2cCh = &I2C2;
#else                                               // SK-FM4-176L-S6E2DH
    SetPinFunc_I2SCK0();
    SetPinFunc_I2SDO0();
    SetPinFunc_I2SWS0();
    SetPinFunc_I2SMCLK0_IN();
    
    SetPinFunc_SOT6_1(); 
    SetPinFunc_SCK6_1();
    
    Gpio1pin_InitOut(GPIO1PIN_P1D, Gpio1pin_InitVal(0u)); // Codec controls WS
    Gpio1pin_InitIn(GPIO1PIN_P15, Gpio1pin_InitPullup(1u)); // Enter Key
    
    I2cCh = &I2C6;
#endif    
    
    // Enable I2S clock supply
    Clk_PeripheralClockEnable(ClkGateI2s0);
    
    // Initialize I2s clock
    stcI2sClkConfig.enI2s0ClkIn = I2s0UseI2sMclki;

    if(Ok != I2s_InitClk(&stcI2sClkConfig))
    {
        while(1); // Clock initialization failed.
    }
    
    // Initialize interrupt
    stcI2sIrqEn.bTxFifoIrq = 1u;
    stcI2sIrqCb.pfnTxFifoIrqCb = I2sTxFifoCallback;
    
    // Initialize I2s
    stcI2sConfig.u8ClockDiv           = 0u;       // 0: Bypass: Use Wolfson clock       
    stcI2sConfig.u16OverheadBits      = 0u;       // 0 overhead bits 
    stcI2sConfig.bMaskBit             = FALSE;    // Mask bit = 0               
    stcI2sConfig.bMasterMode          = FALSE;    // Slave mode              
    stcI2sConfig.bSubFrame01          = FALSE;    // Subframe 0 only
    stcI2sConfig.bFifoTwoWords        = TRUE;     // 32-bit FIFO word as two 16-Bit words            
    stcI2sConfig.bBclkDivByMclk       = FALSE;    // Internal clock divider
    stcI2sConfig.bBitExtensionHigh    = FALSE;    // Bit extension not used
    stcI2sConfig.bFreeRunMode         = TRUE;     // If OPRREG.START = 1, FRUN == TRUE starts free running            
    stcI2sConfig.bLsbFirst            = FALSE;    // Shift with MSB first             
    stcI2sConfig.bSampleAtEnd         = FALSE;    // Sampling time not used for transmission
    stcI2sConfig.bClockpolarity       = FALSE;    // Sample edge not used for transmission       
    stcI2sConfig.bWordSelectSamePhase = FALSE;    // I2S word select 1st bit before frame
    stcI2sConfig.bWordSelectLength    = TRUE;     // WS with one channel data
    stcI2sConfig.bWordSelectPolarity  = TRUE;     // I2S word select at '1'
    stcI2sConfig.stcSubframe0.u8Snchn = 1u;       // 1 channel subframe
    stcI2sConfig.stcSubframe0.u8Snchl = 31u;      // 32 bit length
    stcI2sConfig.stcSubframe0.u8Snwdl = 15u;      // word length - 1 = 15
    stcI2sConfig.stcSubframe1.u8Snchn = 1u;       // 1 channel subframe
    stcI2sConfig.stcSubframe1.u8Snchl = 31u;      // 32 bit length
    stcI2sConfig.stcSubframe1.u8Snwdl = 15u;      // word length - 1 = 15
    stcI2sConfig.u32S0ch              = 0x00000003u; // Enable channel 0 and 1               
    stcI2sConfig.u32S1ch              = 0x00000003u; // Enable channel 0 and 1   
    stcI2sConfig.enPacketReceiveCompletionTimer = NoOperation; // No reception used
    stcI2sConfig.bTxEnable                  = TRUE;               
    stcI2sConfig.bRxEnable                  = FALSE;      // No reception used
    stcI2sConfig.u8TxFifoThreshold    = 4u; // It means when TX FIFO empty slot is more than 4 words, a FIFO IRQ will be issued.
    stcI2sConfig.u8RxFifoThreshold    = 0u;
    stcI2sConfig.pstcIrqEn = &stcI2sIrqEn;
    stcI2sConfig.pstcIrqCb = &stcI2sIrqCb;
    stcI2sConfig.bTouchNvic = TRUE;
    
    if (Ok != I2s_Init(&I2S0, &stcI2sConfig))
    {
        while(1);
    }
    
    // Initialize codec
    // Reset register
    stcWm8731Reg.RESET              = 0x00u;    // Reset WM8731
    // Left & right line input 
    stcWm8731Reg.LLIN_f.LINVOL      = 0x17u;    // Left channel line input volume: 0dB
    stcWm8731Reg.LLIN_f.LINMUTE     = 1u;       // Enable left channel line input mute
    stcWm8731Reg.LLIN_f.LRINBOTH    = 0u;       // Disable simultaneous input volume and mute load from left to right
    stcWm8731Reg.RLIN_f.RINVOL      = 0x17u;    // Right channel line input volume 0dB
    stcWm8731Reg.RLIN_f.RINMUTE     = 1u;       // Enable right channel line input mute
    stcWm8731Reg.RLIN_f.RINBOTH     = 0u;       // Disable simultaneous input volume and mute load from right to left
    // Left & right headphone output 
    stcWm8731Reg.LHOUT_f.LHPVOL     = 0x79;     // Set volume of left headphone to 0dB. 0x30(-73dB) ~ 0x7F(+6dB), 0 ~ 0x2F: mute
    stcWm8731Reg.LHOUT_f.LZCEN      = 1u;       // Disable left channel zero cross detect
    stcWm8731Reg.LHOUT_f.LRHPBOTH   = 0u;       // Disable simultaneous output volume and mute load from left to right
    stcWm8731Reg.RHOUT_f.RHPVOL     = 0x79;     // Set volume of right headphone to 0dB. 0x30(-73dB) ~ 0x7F(+6dB), 0 ~ 0x2F: mute
    stcWm8731Reg.RHOUT_f.RZCEN      = 1u;       // Enable right channel zero cross detect
    stcWm8731Reg.RHOUT_f.RLHPBOTH   = 0u;       // Disable simultaneous output volume and mute load from right to left
    // Analog audio path control
    stcWm8731Reg.AAPC_f.MICBOOST    = 0u;       // Disable boost
    stcWm8731Reg.AAPC_f.MUTEMIC     = 1u;       // Enable mute to ADC
    stcWm8731Reg.AAPC_f.INSEL       = 0u;       // Line input select to ADC
    stcWm8731Reg.AAPC_f.BYPASS      = 1u;       // Enbale bypass
    stcWm8731Reg.AAPC_f.DACSEL      = 1u;       // Select DAC
    stcWm8731Reg.AAPC_f.SIDETONE    = 0u;       // Disable side tone
    stcWm8731Reg.AAPC_f.SIDEATT     = 0u;       // 0: -6dB, 1: -12dB, 2: -9dB, 3: -15dB.
    // Digital audio path control
    stcWm8731Reg.DAPC_f.ADCHPD      = 0u;       // Enable high pass filter
    stcWm8731Reg.DAPC_f.DEEMP       = 3u;       // De-emphasis contrl. 0: disable, 1: 32kHz, 2: 44.1kHz, 3: 48kHz
    stcWm8731Reg.DAPC_f.DACMU       = 0u;       // Disable soft mute
    stcWm8731Reg.DAPC_f.HPOR        = 0u;       // Clear offset when high pass 
    // Power down control
    stcWm8731Reg.PDC_f.LINEINPD     = 0u;       // Disable line input power down
    stcWm8731Reg.PDC_f.MICPD        = 0u;       // Disable microphone input power down
    stcWm8731Reg.PDC_f.ADCPD        = 0u;       // Disable ADC power down
    stcWm8731Reg.PDC_f.DACPD        = 0u;       // Disable DAC power down
    stcWm8731Reg.PDC_f.OUTPD        = 0u;       // Disable output power down
    stcWm8731Reg.PDC_f.OSCPD        = 0u;       // Disable oscillator power down
    stcWm8731Reg.PDC_f.CLKOUTPD     = 0u;       // Disable CLKOUT power down
    stcWm8731Reg.PDC_f.POWEROFF     = 0u;       // Disable power off mode
    // Digital audio interface format
    stcWm8731Reg.DAIF_f.FORMAT      = 2u;       // 0: MSB-First, right justified, 1: MSB-first, left justified, 2: I2S-format, 3: DSP mode
    stcWm8731Reg.DAIF_f.IWL         = 0u;       // 0: 16 bits, 1: 20 bits, 2: 24 bits, 3: 32 bits
    stcWm8731Reg.DAIF_f.LRP         = 1u;       // 1: right channel DAC data when DACLRC (WS) is high,  0: right channel DAC data when DACLRC (WS) is low
    stcWm8731Reg.DAIF_f.LRSWAP      = 0u;       // 1: swap left channel and right channel, 0: don't swap  
    stcWm8731Reg.DAIF_f.MS          = 1u;       // 1: Enable master mode, 0: Enable slave mode
    stcWm8731Reg.DAIF_f.BCLKINV     = 0u;       // Don't invert BCLK
    // Sampling control
    stcWm8731Reg.SC_f.NORMAL_USB    = 0u;       // 0: normal mode, 1: USB mode
    stcWm8731Reg.SC_f.BOSR          = 0u;       // Nomrmal mode: 0: 256fs, 1: 384fs
                                                // USB mode: 0: 250fs, 1:272fs
    stcWm8731Reg.SC_f.SR            = 0u;       // Sample rate setting
    stcWm8731Reg.SC_f.CLKDIV2       = 0u;       // 0: core clock is MCLK, 1: core clock is MCLK divided by 2
    stcWm8731Reg.SC_f.CLKODIV2      = 0u;       // 0: output clock is core clock, 1: core clock is core clock/2
    // Active control
    stcWm8731Reg.AC_f.ACTIVE        = 1u;       // 0: inactive, 1: active
    
    if(Ok != Wm8731_Init(I2cCh, &stcWm8731Reg))
    {
        while(1);
    }
    
    Wm8731_SetHpVolume(0x6F,0x6F);  //0x2F-MUTE ~ 0x7F Maximum
    I2s_Start(&I2S0);
    
    while(1)
    {       
    #if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2CCX)    // SK-FM4-176L-S6E2CC   
        if ((FALSE == bPlaySoundActive) && (0u == Gpio1pin_Get(GPIO1PIN_P20)))
    #else                                                // SK-FM4-176L-S6E2DH
        if ((FALSE == bPlaySoundActive) && (0u == Gpio1pin_Get(GPIO1PIN_P15)))  
    #endif      
        {
            pu16SoundData = &au16PixieDustSoundI2s[0];
            bPlaySoundActive = TRUE;
        }
    }
}
