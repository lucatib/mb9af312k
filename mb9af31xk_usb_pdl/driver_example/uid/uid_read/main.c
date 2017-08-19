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
 ** Unique ID Example
 ** This example uses all provided methods to read out the unique id.
 **
 ** History:
 **   - 2014-12-06  1.0  EZh        First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_header.h"

/**
 ******************************************************************************
 ** \brief  Main function for Unique ID example
 **
 ** \return uint32_t return value, if needed
 ******************************************************************************/
int32_t main(void)
{
    stc_unique_id_t stcUniqueId;          // PDL structure of Unique ID register set
    uint32_t        u32UniqueId0;         // Unique ID 0
    uint32_t        u32UniqueId1;         // Unique ID 1
    uint64_t        u64UniqueIdComplete;  // Complete 41-Bit Unique ID
    
    if (Ok == Uid_ReadUniqueId(&stcUniqueId))
    {
      u32UniqueId0 = Uid_ReadUniqueId0();
      u32UniqueId1 = Uid_ReadUniqueId1();
      
      if (((stcUniqueId.u32Uidr0 >> 4u) != u32UniqueId0) ||
          ((stcUniqueId.u32Uidr1      ) != u32UniqueId1))
      {
         while(1u)
         {}             // Read Error ...
      }
    }
    else
    {
      while(1u)
      {}                // Error handling here ...
    }
    
    u64UniqueIdComplete = Uid_ReadUniqueId64();
    
    if (((u64UniqueIdComplete >> 28ul)        != u32UniqueId1) ||
        ((0x0FFFFFFFul & u64UniqueIdComplete) != u32UniqueId0))
    {
         while(1u)
         {}             // Read Error ...
    }
    
    while(1u)
    {}                  // Everything fine ...
}
