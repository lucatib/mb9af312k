===============================================================================================
                                      CRC sample
===============================================================================================

Copyright (C) 2013-2016, Cypress Semiconductor Corporation. All rights reserved.    
                                                                                     
This software, including source code, documentation and related                     
materials ( "Software" ), is owned by Cypress Semiconductor                         
Corporation ( "Cypress" ) and is protected by and subject to worldwide              
patent protection (United States and foreign), United States copyright              
laws and international treaty provisions. Therefore, you may use this               
Software only as provided in the license agreement accompanying the                 
software package from which you obtained this Software ( "EULA" ).                  
If no EULA applies, Cypress hereby grants you a personal, nonexclusive,             
non-transferable license to copy, modify, and compile the                           
Software source code solely for use in connection with Cypress's                    
integrated circuit products. Any reproduction, modification, translation,           
compilation, or representation of this Software except as specified                 
above is prohibited without the express written permission of Cypress.              
Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                                
WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                                
BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                        
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                                     
PARTICULAR PURPOSE. Cypress reserves the right to make                              
changes to the Software without notice. Cypress does not assume any                 
liability arising out of the application or use of the Software or any              
product or circuit described in the Software. Cypress does not                      
authorize its products for use in any products where a malfunction or               
failure of the Cypress product may reasonably be expected to result in              
significant property damage, injury or death ( "High Risk Product" ). By            
including Cypress's product in a High Risk Product, the manufacturer                
of such system or application assumes all risk of such use and in doing             
so agrees to indemnify Cypress against all liability.  

================================================================================
History 
Date        Ver    AE          IAR     ARM      Description
2014-11-21  0.1   DHO          6.50.1  4.50    first version
2016-06-21  0.2   CCTA         7.50.2  5.18a   FOr PDL V2.0.2
===============================================================================================
Function description:
===============================================================================================
Sample program to check CRC function.


===============================================================================================
Environment
===============================================================================================
Test Board:
---------------------
FM3  TYPE0	MB9BF506R	mini-board
FM3  TYPE1 	MB9AF314L	mini-board
FM3  TYPE2	MB9BF618S	mini-board
FM3  TYPE3	MB9AF132L	mini-board
FM3  TYPE4	MB9BF516R	mini-board
FM3  TYPE5	MB9AF312K	mini-board
FM3  TYPE6	MB9AFB44N	mini-board
FM3  TYPE7	MB9AFA32N	mini-board
FM3  TYPE8	MB9BF156R	mini-board
FM3  TYPE9	MB9BF524M	mini-board
FM3  TYPE10	MB9B121J	mini-board
FM3  TYPE11	MB9AF421L	mini-board
FM3  TYPE12	MB9BF529T	mini-board
FM0P TYPE1      S6E1A12C        SK-FM0-V48-S6E1A1 V1.1.0
FM0P TYPE2      S6E1B86F        SK-FM0-100L-S6E1BA V1.0.0
FM4  TYPE1	MB9BF568R 	SK-FM4-U120-9B560 v1.1.0 
FM4  TYPE3      S6E2CCAL        SK-FM4-216-ETHERNET v1.0
FM4  TYPE4      S6E2DH5J        SK-FM4-176L-S6E2DH v1.1.0

Assistance tool:
---------------------
None

Assistance software:
---------------------
None

===============================================================================================
Usage Procedure:
===============================================================================================
1) Rebuild and run the program.
2) Watch the Terminal I/O window (Debug viewer)
   If character doesn't be outputted to the Terminal I/O window (Debug viewer),
   CRC operates normally.


===============================================================================================
Notice:
===============================================================================================
1) The printf content can be output to debug viewer In IAR with semi-hosting mode.
2) The printf content can be output to debug viewer In Keil MDK via SWO line. 
3) The printf content can also output to UART0 (SOT0_0).
   Make following steps to enable this method:
   - Set "PDL_PERIPHERAL_ENABLE_MFS0" and "PDL_UTILITY_ENABLE_UART_PRINTF" to PDL_ON in pdl.user.h
   - Call "Uart_Io_Init()" at the begin of main function 
4) For FM3 and FM4 products, All of 1), 2), 3) can be supported. However SWO pin is not available 
   in FM0+ product. So only 1), 3) can be suppoted for FM0+ products.
5) Before using printf function, enable the definition "DEBUG_PRINT" in pdl_user.h. When user
   Want to program the code into Flash and run the code standalone, disable the definition 
   "DEBUG_PRINT" in pdl_user.h. 