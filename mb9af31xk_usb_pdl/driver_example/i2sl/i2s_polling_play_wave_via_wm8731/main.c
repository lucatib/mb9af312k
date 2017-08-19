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
 ** The example shows how to play wave data via I2S-Lite with polling mode.
 **
 ** History:
 **   - 2015-04-02  1.0  EZh       First release version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
#include "pixiedust_sound_i2s.h"   

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)  // SK-FM0-100L-S6E1BA
#define I2slCh           I2SL1
#define InitI2slIo()     {SetPinFunc_I2SCK6_1(); SetPinFunc_I2SDO6_1(); SetPinFunc_I2SWS6_1(); SetPinFunc_I2SMCK6_1_OUT();}
#define I2SL_FIFO_CAPACITY   128 // Check this data in product datasheet
#define I2SL_FIFO_COUNT      (I2SL_FIFO_CAPACITY/4)  // 2 bytes for left channel, 2 bytes for right channel
#define I2SL_DATA_COUNT       sizeof(au16I2sData)/sizeof(uint16_t)

#define I2cCh            I2C3
#define InitI2cIo()      {SetPinFunc_SOT3_1(); SetPinFunc_SCK3_1();}

#define InitKeyIo()      Gpio1pin_InitIn(GPIO1PIN_P08, Gpio1pin_InitPullup(1u)) //  key
#define GetKey()         Gpio1pin_Get(GPIO1PIN_P08)
#define Switch2Play()    Gpio1pin_InitOut(GPIO1PIN_P4B, Gpio1pin_InitVal(0u))   // Codec controls WS

#define I2SL_CLK_DIV     2     // I2SMCK = 40/(2*2) = 10MHz, I2SCLK = 10MHz/4 = 2.5MHz

#elif (PDL_MCU_TYPE == PDL_FM0P_TYPE3)  // SK-FM0-64L-S6E1C3
#define I2slCh           I2SL0
#define InitI2slIo()     {SetPinFunc_MI2SCK4_1(); SetPinFunc_MI2SDO4_1(); SetPinFunc_MI2SWS4_1(); SetPinFunc_MI2SMCK4_1_OUT();}
#define I2SL_FIFO_CAPACITY   64 // Check this data in product datasheet
#define I2SL_FIFO_COUNT      (I2SL_FIFO_CAPACITY/4)  // 2 bytes for left channel, 2 bytes for right channel
#define I2SL_DATA_COUNT       sizeof(au16I2sData)/sizeof(uint16_t)

#define I2cCh            I2C6
#define InitI2cIo()      {SetPinFunc_SOT6_1(); SetPinFunc_SCK6_1();}

#define InitKeyIo()      Gpio1pin_InitIn(GPIO1PIN_P30, Gpio1pin_InitPullup(1u)) //  key
#define GetKey()         Gpio1pin_Get(GPIO1PIN_P30)
#define Switch2Play()

#define I2SL_CLK_DIV     2     // I2SMCK = 40/(2*2) = 10MHz, I2SCLK = 10MHz/4 = 2.5MHz

#else                                 // SK-FM4-144L-S6E2GM
#define I2slCh           I2SL0
#define InitI2slIo()     {SetPinFunc_MI2SCK1_0(); SetPinFunc_MI2SDO1_0(); SetPinFunc_MI2SWS1_0(); SetPinFunc_MI2SMCK1_0_OUT();}
#define I2SL_FIFO_CAPACITY   64 // Check this data in product datasheet
#define I2SL_FIFO_COUNT      (I2SL_FIFO_CAPACITY/4)  // 2 bytes for left channel, 2 bytes for right channel
#define I2SL_DATA_COUNT       sizeof(au16I2sData)/sizeof(uint16_t)

#define I2cCh            I2C9
#define InitI2cIo()      {SetPinFunc_SOT9_0(); SetPinFunc_SCK9_0();}

#define InitKeyIo()      Gpio1pin_InitIn(GPIO1PIN_P20, Gpio1pin_InitPullup(1u)) //  key
#define GetKey()         Gpio1pin_Get(GPIO1PIN_P20)
#define Switch2Play()    

#define I2SL_CLK_DIV     4     // I2SMCK = 90/(2*4) = 11.25MHz, I2SCLK = 11.25MHz/4 = 2.812MHz

#endif

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
const uint16_t *pu16SoundData;
boolean_t bPlaySoundActive = FALSE;


/**
 ******************************************************************************
 ** \brief  Main function of PDL
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{ 
    stc_i2sl_config_t stcI2slConfig;
    stc_wm8731_reg_t stcWm8731Reg;
    uint8_t u8i;
    uint16_t u16OutLeft, u16OutRight;
    uint32_t u32Fifo;
    
    // Clear strcutures
    PDL_ZERO_STRUCT(stcI2slConfig);
    PDL_ZERO_STRUCT(stcWm8731Reg);
  
    // Initialize I2S IO, I2C IO
    InitI2slIo();
    InitI2cIo();
   
    InitKeyIo();
    
    Switch2Play();
    
    // Initialize I2S-Lite
    stcI2slConfig.enClkIn = I2slUsePclk;            // Use PCLK as I2SCK source clock
    stcI2slConfig.u8ClkInDiv = I2SL_CLK_DIV;        // Set clock div of PCLK           
    stcI2slConfig.bClkOutputEn = FALSE;             // Disable I2SMCLK output
    stcI2slConfig.enTransferMode = I2slPhilipsMode; // Philips standard mode
    stcI2slConfig.bDataOutput = TRUE;               // Data transfer
    stcI2slConfig.bCkWsOutputEn = TRUE;             // I2SCK, I2SWS outpue enable
    stcI2slConfig.bI2sWsPolarity = FALSE;           // WS low level for left channel
    stcI2slConfig.bMaskBitOutput = 0;               // Mask bit set to 1 for invalid frame
    stcI2slConfig.enFrameLength = I2slFrame32Bit;   // 32-bit data frame
    stcI2slConfig.enFifoWidth = I2slFifoWidth32Bit; // 32-bit FIFO access width
    stcI2slConfig.enFifoSel = I2slTxFifo1RxFifo2;   // FIFO1: TX, FIFO2: RX
    stcI2slConfig.u8ByteCount1 = 0;                 // Clear FIFO count

    if (Ok != I2sl_Init(&I2slCh, &stcI2slConfig))
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
    stcWm8731Reg.DAIF_f.FORMAT      = 1u;       // 0: MSB-First, right justified, 1: MSB-first, left justified, 2: I2S-format, 3: DSP mode
    stcWm8731Reg.DAIF_f.IWL         = 0u;       // 0: 16 bits, 1: 20 bits, 2: 24 bits, 3: 32 bits
    stcWm8731Reg.DAIF_f.LRP         = 0u;       // 1: right channel DAC data when DACLRC (WS) is high,  0: right channel DAC data when DACLRC (WS) is low
    stcWm8731Reg.DAIF_f.LRSWAP      = 0u;       // 1: swap left channel and right channel, 0: don't swap  
    stcWm8731Reg.DAIF_f.MS          = 0u;       // 1: Enable master mode, 0: Enable slave mode
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
    
    if(Ok != Wm8731_Init(&I2cCh, &stcWm8731Reg))
    {
        while(1);
    }
    
    Wm8731_SetHpVolume(0x65,0x65);  //0x2F-MUTE ~ 0x7F Maximum
   
    I2sl_Start(&I2slCh); // Start I2S
    
    while(1)
    {                                                   
        if ((FALSE == bPlaySoundActive) && (0u == GetKey()))
        {
            pu16SoundData = &au16PixieDustSoundI2s[0];
            bPlaySoundActive = TRUE;
            
            I2sl_EnableClockOut(&I2slCh); // Enable clock out
        }
        
        if(TRUE == bPlaySoundActive)
        {
        if(I2sl_GetFifoCount(&I2slCh, I2slFifo1) < I2SL_FIFO_COUNT/2) // if FIFO level is less than half
        {
            u8i = I2SL_FIFO_COUNT/2;  // Fill the FIFO with data count of half FIFO capacity
            while(u8i--)
            {
                u16OutLeft = *pu16SoundData++;
                u16OutRight = *pu16SoundData++;
                
                u32Fifo = (u16OutLeft | (u16OutRight << 16));
                
                I2sl_WriteTxFifo(&I2slCh, u32Fifo); 
                
                if(*pu16SoundData == 0xffff)
                {
                  bPlaySoundActive = FALSE;
                  I2sl_DisableClockOut(&I2slCh); // Enable clock out
                }
            }
        }
    }
}
}

