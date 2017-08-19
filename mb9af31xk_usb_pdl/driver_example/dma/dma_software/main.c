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
 ** \brief This example shows how to use the DMA for software block transfer
 **
 ** In this example the DMA channel 0 is used to transfer the contents of
 ** an array to antother array using software block transfer. After the
 ** transfer a callback function is executed, which sets a DMA finished flag.
 ** The transfer is checked in main().
 **
 ** History:
 **   - 2014-02-21  0.0.1  Edison Zhang        First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

#define DMA_MAX_COUNT 256u

boolean_t   bDmaFinished = FALSE;					// DMA finished notification flag
uint32_t    au32SourceData[DMA_MAX_COUNT];		// Source data (filled in main() )
uint32_t    au32DestinationData[DMA_MAX_COUNT];

/**
 ******************************************************************************
 ** \brief  DMA Error callback function
 ******************************************************************************/
void Main_dma_error_callback(uint8_t u8ErrorCode)
{
    // Error handling ... Should never happen.
}

/**
 ******************************************************************************
 ** \brief  DMNA finish callback function
 ******************************************************************************/
void Main_dma_finish_callback(void)
{
    bDmaFinished = TRUE;                  	// Set DMA finished notification flag
}

/**
 ******************************************************************************
 ** \brief  ADC0 initialization and single conversion start
 ******************************************************************************/
void Main_dma(void)
{
    stc_dma_config_t stcConfigDma;
    stc_dma_irq_sel_t stcIrqSel;
    stc_dma_irq_cb_t  stcIrqCb;

    // Clear local configuration structure to zero.
    PDL_ZERO_STRUCT(stcConfigDma);	
    PDL_ZERO_STRUCT(stcIrqSel);	
    PDL_ZERO_STRUCT(stcIrqCb);	

    // Initialize interrupt
    stcIrqSel.bCompleteIrq = TRUE;
    stcIrqSel.bErrorIrq    = TRUE;
    stcIrqCb.pfnDmaCompletionIrqCb = Main_dma_finish_callback;
    stcIrqCb.pfnDmaErrorIrqCb      = Main_dma_error_callback;

    // DMMAC0 configuration
    stcConfigDma.enDmaIdrq = Software;
    stcConfigDma.u8BlockCount = 1u;
    stcConfigDma.u16TransferCount = DMA_MAX_COUNT ;   
    stcConfigDma.enTransferMode = DmaBlockTransfer;
    stcConfigDma.enTransferWdith = Dma32Bit;
    stcConfigDma.u32SourceAddress = (uint32_t) &au32SourceData[0u];			// Source array's address
    stcConfigDma.u32DestinationAddress = (uint32_t) &au32DestinationData[0u];	// Destination array's address
    stcConfigDma.bFixedSource = FALSE;
    stcConfigDma.bFixedDestination = FALSE;
    stcConfigDma.bReloadCount = FALSE;     
    stcConfigDma.bReloadSource = FALSE;
    stcConfigDma.bReloadDestination = FALSE;
    stcConfigDma.bEnableBitMask = FALSE;
    stcConfigDma.pstcIrqCb = &stcIrqCb;
    stcConfigDma.pstcIrqEn = &stcIrqSel;
    stcConfigDma.bTouchNvic = TRUE;

    if (Ok == Dma_InitChannel(0u, &stcConfigDma))      // Initialize DMA channel 0
    {    
        Dma_Enable();            // Overall enable of DMA

        Dma_SetChannel(0u, TRUE, FALSE, TRUE);	      // Enable channel and software trigger
    }
}

/**
 ******************************************************************************
 ** \brief  Main function
 **
 ** \return int32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    uint32_t  u8Count;
    boolean_t bError = FALSE;

    // Fill Source data with "random" data
    for (u8Count = 0; u8Count < DMA_MAX_COUNT; u8Count++)
    {
        au32SourceData[u8Count] = (uint32_t)u8Count ^ 0x12345678u;    
    }

    Main_dma();                   // Setup DMA and start transfer

    while(FALSE == bDmaFinished);  // Wait for DMA finished notification flag

    Dma_Disable();				// Disable DMA

    // Compare data
    for (u8Count = 0; u8Count < DMA_MAX_COUNT; u8Count++)
    {
        if (au32SourceData[u8Count] != au32DestinationData[u8Count])
        {
            bError = TRUE;
            break;
        }
    }

    if (TRUE == bError)			// Should never happen ...
    {
        while(1)
        {}
    }

    while(1)
    {}
  
}
