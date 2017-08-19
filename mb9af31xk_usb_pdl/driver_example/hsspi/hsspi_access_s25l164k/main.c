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
 ** \brief This example project shows how to communicate with Spansion S25FL164K
 **        Flash of the SK-FM4-216-ETHERNET board in HSSPI direct
 **	       mode and command sequencer mode.
 **
 ** History:
 **   - 2014-10-24  1.0  MWi        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"
#include <stdio.h>

/*****************************************************************************/
/* Local definitions                                                         */
/*****************************************************************************/
#if (PDL_MCU_SERIES != PDL_DEVICE_SERIES_S6E2CCX)
#error This example is developed on SK-FM4-178L-S6E2CC board!
#endif

/*****************************************************************************/
/* Local variable                                                            */
/*****************************************************************************/
static stc_hsspi_config_t             stcHsspiConfig;
static stc_hsspi_ext_device_config_t  stcExtDeviceConfig;
static uint8_t                        u8DeviceNr = 0u;
static en_result_t                    enResult;

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/
stc_hsspin_t* pstcHsSpi = (stc_hsspin_t*)&HSSPI0;

/******************************************************************************
 ** \brief HP_SPI Test
 **
 ** This module shows how to communicate with the Spansion S25FL164K external
 ** Flash of the SK-FM4-216-ETHERNET board in HSSPI direct mode and command
 ** sequencer mode.
 ** 
 ** *************** Using the Command Sequencer mode ***************
 ** The Spansion S25FL164K external Flash's 8MB is divided into memory banks of
 ** 64KB by setting MSEL[3:0] = 4b'0011. Hence, a total of 128 memory banks.
 ** 
 ** External memory access using system space when memory bank size is 64KB:
 **  - The slave is selected using AHB address bits AHB[17:16]. For
 **    SK-FM4-216-ETHERNET board the external Flash is connected to CS0, so
 **    AHB[17:16] should be 0 to access this device. 
 **  - A specific memory bank is selected using the address extension bits
 **    AEXT[18:3]. The bits AEXT[2:0] are NOT USED in memory bank size = 64KB
 **    configuration.
 **  - AHB[15:0] indicates the offset of a specific memory location from the
 **    start of selected memory bank.
 **
 **  Therefore, the final 32-bit address which is sent out to Serial Flash is a 
 **  combination of AEXT[18:3] and AHB[15:0].
 **  
 ** ******* MSEL = 4b'0011 ******* 
 **  Memory   Serial Flash                 Slave      Bank      AHB[15:0] Range         Address range 
 **   Bank    address range                Select    Select                            on system space
 **  (Size                               AHB[17:16] AEXT[18:3]
 **  = 64KB)							 		
 **  0        0x0000_0000 - 0x0000_FFFF    2b'00     0x0000     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF			
 **  1        0x0001_0000 - 0x0001_FFFF    2b'00     0x0001     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 **  2        0x0002_0000 - 0x0002_FFFF    2b'00     0x0002     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 **  3        0x0003_0000 - 0x0003_FFFF    2b'00     0x0003     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 **  4        0x0004_0000 - 0x0004_FFFF    2b'00     0x0004     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 **  5        0x0005_0000 - 0x0005_FFFF    2b'00     0x0005     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 **  6        0x0006_0000 - 0x0006_FFFF    2b'00     0x0006     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 **  7        0x0007_0000 - 0x0007_FFFF    2b'00     0x0007     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 **  8        0x0008_0000 - 0x0008_FFFF    2b'00     0x0008     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 **  9        0x0009_0000 - 0x0009_FFFF    2b'00     0x0009     0x0000 - 0xFFFF   0xC000_0000 - 0xC000_FFFF
 ** ........and so on
 ** 
 ** For example (MSEL = 4b'0011 in the Command Sequencer mode)
 **  - To access the serial Flash address 0x0001_0000, set AEXT[18:3] = 0x0001
 **    and access the system memory address C000_0000
 **  - To access the serial Flash address 0x0005_0A00, set AEXT[18:3] = 0x0005
 **    and access the system memory address C000_0A00
 ** 
 ** \return Ok              HsSpi test successfull 
 ** \return Error           HsSpi test failed
 ******************************************************************************/
en_result_t hsspi_test(void)
{
  uint8_t u8TxData[64u];
  uint8_t u8RxData[64u];
  uint8_t au8UniqueId[8u];
  uint8_t u8Count;
  
  volatile uint8_t* pu8CSReadAddr;
  volatile uint8_t  u8CSReadData = 0u;
  volatile uint8_t  au8CSstring[10u];
  
  boolean_t bDeviceReady = FALSE;
  uint32_t  u32Timeout   = 10u;
  
  stcHsspiConfig.enClockDivider = HsSpiClkDividerDiv2;                          // - Clock = Hclk/2
  stcHsspiConfig.enClockSelection = HsSpiClockAhb;                              // - source = HCLK See description of #en_spi_clk_selection_t.
  stcHsspiConfig.u8TxFifoThresholdLow = 1u;                                     // Possible values from 0 to 15 (maximum fifo length is 16).
  stcHsspiConfig.pfnTxStatusCallback  = NULL,                                   // Callback function for transmission status.
  stcHsspiConfig.enMemoryBankSize = HsSpiBankSize64kB;                          // HsSpiBankSize8kB = 11 means memory bank size is 64Kb. See description of #en_spin_bank_size_t.
  stcHsspiConfig.enDirectModeProtocol = HsSpiProtocolModeSingle;                // See description of #en_spin_direct_mode_transfer_protocol_t.
  stcHsspiConfig.enCommandSequencerModeProtocol = HsSpiProtocolModeSingle;      // See description of #en_spin_command_sequencer_mode_transfer_protocol_t.
  stcHsspiConfig.enMemoryType = HsSpiFlash;                                     // Serial Flash memory devices are connected. Writes are disabled. See description of #en_spin_memory_type_t.
  stcHsspiConfig.u16IdleTimeOut = 1000u;                                        // The idle timeout interval is in terms of the AHB clock period.
  stcHsspiConfig.enFifoWidth = HsSpiFifoWidth8;                                 // 8-bit width for all of the TX-FIFO, RX-FIFO, and shift registers
  stcHsspiConfig.bMasterOperation   = TRUE;                                     // HS-SPI master       
  stcHsspiConfig.bTxDmaBridgeEnable = FALSE;                                    // No TX DMA
  stcHsspiConfig.bRxDmaBridgeEnable = FALSE;                                    // No RX DMA
  
  stcExtDeviceConfig.enClockDivider = HsSpiClkDividerDiv2;                      // xMHz SPI
  stcExtDeviceConfig.bShiftLsbFirst = FALSE;                                    // MSB first
  stcExtDeviceConfig.enClockDelay   = HsSpiClkStart2ClkAfterSlaveSelect;        // Minimal clock delay
  stcExtDeviceConfig.bSlaveSelectPolarityHigh = FALSE;                          // Chipselect is low active.
  stcExtDeviceConfig.enClockMode    = HsSpiClkLowOutFallingInRising;            // SPI MODE
  stcExtDeviceConfig.bSafeSync      = TRUE;
  stcExtDeviceConfig.enEndianess    = HsSpiBigEndian;
  stcExtDeviceConfig.u8ReadDeselectTime  = 1u;    
  stcExtDeviceConfig.u8WriteDeselectTime = 1u;   
  stcExtDeviceConfig.bCompensatedClock   = TRUE; 
  
  // HSSPI Clock enable
  Clk_PeripheralClockEnable(ClkGateQspi);
  
  // Port drive capability select register
  FM_GPIO->PDSR9 = 0x000000FFul;                // 1 => Pin drive capability for VCC = 3V
                                                 // 0 => Pin drive capability for VCC = 5V
  
  // HS-SPI ports for S6E2CC
  SetPinFunc_Q_IO0_0();
  SetPinFunc_Q_IO1_0();
  SetPinFunc_Q_IO2_0();
  SetPinFunc_Q_IO3_0();
  SetPinFunc_Q_SCK_0();
  SetPinFunc_Q_CS0_0();
  
  enResult = HsSpi_Init(pstcHsSpi, (const stc_hsspi_config_t*) &stcHsspiConfig);
  if(enResult != Ok)
  {
    return Error;
  }
  
  enResult = HsSpi_SetMode(pstcHsSpi, HsSpiModeDirect, HsSpiProtocolModeSingle, HsSpiProtocolTxRx);
  if(enResult != Ok)
  {
    return Error;
  }
  
  enResult = HsSpi_SetExternalDeviceConfig(pstcHsSpi, u8DeviceNr, &stcExtDeviceConfig);
  if(enResult != Ok)
  {
    return Error;
  }
  
  // Set QUAD IO mode
  enResult = FlashS25FL164K_SetQIO(u8DeviceNr, pstcHsSpi, TRUE);
  if (enResult != Ok)
  {
    return Error;
  }
  
  // Try to read the chip ID several times, until the device is ready i.e.
  // until it returns the correct chip ID (timeout: 10 cycles)
  do
  {
      // Read device ID
      enResult = FlashS25FL164K_ReadID(u8DeviceNr, pstcHsSpi, u8RxData, 3u);
      
      if (    (0x01u == u8RxData[0u])          // Manufacturer ID (Spansion)
           && (0x40u == u8RxData[1u])          // Device ID (S25FL1-K)
           && (0x17u == u8RxData[2u]))         // Capacity (64 MBit = 8 MByte)

      {
          bDeviceReady = TRUE;
      }
      else
      {
          u32Timeout--;
      }
  } while ((FALSE == bDeviceReady) && (0u != u32Timeout));

  if (0u == u32Timeout)
  {
    return Error;
  }
  
  // Perform Block Erase
  enResult = FlashS25FL164K_Erase(u8DeviceNr, pstcHsSpi, 0x010000ul, FALSE);    // 0x010000 = 2nd block
  if (enResult != Ok)
  {
    return Error;
  }
  
  // Prepare write Data
  u8TxData[0u] = 0xA5u;         // 1010_0101
  u8TxData[1u] = 0x5Au;         // 0101_1010
  u8TxData[2u] = 0xA5u;
  u8TxData[3u] = 0x5Au;
    
  // Write Data
  enResult = FlashS25FL164K_Write(u8DeviceNr, pstcHsSpi, &u8TxData[0u], 0x010000ul, 0x4ul);
  if (enResult != Ok)
  {
    return Error;
  }
  
  // Read back data
  enResult = FlashS25FL164K_Read(u8DeviceNr, pstcHsSpi, &u8RxData[0u], 0x010000ul, 0x4ul);
  if (enResult != Ok)
  {
    return Error;
  }
  
  // Compare Data read in direct mode
  for (u8Count = 0u; u8Count < 0x04u; u8Count++)
  {
    if (u8RxData[u8Count] != u8TxData[u8Count])
    {
      return Error;
    }
  }
  
  // Read Unique ID
  enResult = FlashS25FL164K_ReadUniqueId(u8DeviceNr, pstcHsSpi, &au8UniqueId[0]);
  if (enResult != Ok)
  {
      return Error;
  }

  // Enable Command Sequencer mode
  FlashS25FL164K_EnableCommandSequencer(u8DeviceNr, pstcHsSpi, TRUE);
  
  // Access the Serial flash memory address 0x0001_0000 by directly accessing 
  // the system memory address 0xC000_0000 with Address Extension (AEXT[18:3] = 1)
  pstcHsSpi->CSAEXT = (1ul << 16ul);         // CSAEXT[31:16] = 1  =>  AEXT[18:3] = 1
  pu8CSReadAddr = (uint8_t*)0xC0000000ul;                               
  
  // Compare Data read in Command Sequencer mode
  // Data is read by directly accessessing the system memory space.
  for (u8Count = 0u; u8Count < 0x04u; u8Count++)
  {
    if (pu8CSReadAddr[u8Count] != u8TxData[u8Count])
    {
      return Error;
    }
  }
  
  enResult = HsSpi_DeInit(pstcHsSpi);
  if (enResult != Ok)
  {
      return Error;
  }
  
  return Ok;
  
} // hsspi_test

/**
 ******************************************************************************
 ** \brief  Main function of project for MB9B560R series.
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
  if(Ok != hsspi_test())
  {
    while(1); // Flash test error
  }
   
  while(1u)  // Flash ok.
  {}
}
/*****************************************************************************/
/* EOF (not truncated)                                                       */
/*****************************************************************************/
