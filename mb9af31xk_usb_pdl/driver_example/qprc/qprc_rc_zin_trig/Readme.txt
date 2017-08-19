===============================================================================================
                               QPRC example program
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
2014-12-15  0.1   DHO          6.50.1  4.50    first version
2016-06-21  0.2   CCTA         7.50.2  5.18a   FOr PDL V2.0.2
===============================================================================================                                                       
Function description:
===============================================================================================
Demo the Mode 1 of Revolution counter of QPRC.

===============================================================================================
Environment
===============================================================================================
Test Board:
---------------------
MCU evaluation board

Assistance tool:
---------------------
Use a TTL-RS232 cable between UART0 and PC

Assistance software:
---------------------
Hyper Terminal

===============================================================================================
Usage Procedure:
===============================================================================================
1) Open Hyper Terminal and set the configuration:
   Baudrate:     115200
   Data Bits:    8
   Parity:       None
   Stop Bits:    1
   Flow Control: None"
2) Connect RTO00_0 with AIN0_0 and P14 with ZIN0_0, connect a user key to P0F
   Notice: if used the FM4 serial, please change the channel set 10 to 54 with OCU and WFG channel
   include
   (1). Change '#define  USER_OCU_CH0 (MFT_OCU_CH0)' to '#define  USER_OCU_CH0 (MFT_OCU_CH4)'
   (2). Change '#define  USER_OCU_CH0 (MFT_OCU_CH1)' to '#define  USER_OCU_CH0 (MFT_OCU_CH5)'
   (3). Change '#define  USER_OCU_COUPLE_CH (MFT_OCU_CH10)' to '#define USER_OCU_COUPLE_CH    (MFT_OCU_CH54)'
   (4). Change '#define  USER_WFG_COUPLE_CH (MFT_WFG_CH10)' to '#define  USER_WFG_COUPLE_CH   (MFT_WFG_CH54)'
   (5). Change '#define  InitMftIo() {SetPinFunc_RTO00_0(); SetPinFunc_RTO01_0();}' to '#define InitMftIo()   {SetPinFunc_RTO04_0(); SetPinFunc_RTO05_0();}'
3) Rebuild and run the program. 
4) Check the information at the Hyper Ternimal (PC count matchs and PC count overflows, 
   then nothing will occurs)
5) Press Key, and RC count matchs.
6) Check the information at the Hyper Ternimal (PC count matchs and PC count overflows, 
   then nothing will occurs)
...

===============================================================================================
Notice:
===============================================================================================
1) The printf content can be output to debug viewer In IAR with semi-hosting mode.
2) The printf content can be output to debug viewer In Keil MDK via SWO line. 
   However SWO pin is not available in FM0+ product, the "putchar" re-target method can be used here.
   With this method, printf can output to UART0 (SOT0_0).
   Make following steps to enable this method:
   - Set "PDL_PERIPHERAL_ENABLE_MFS0" and "PDL_UTILITY_ENABLE_UART_PRINTF" to PDL_ON in pdl.user.h
   - Call "Uart_Io_Init()" at the begin of main function.
3) Before using printf function, enable the definition "DEBUG_PRINT" in pdl_user.h. When user
   Want to program the code into Flash and run the code standalone, disable the definition 
   "DEBUG_PRINT" in pdl_user.h.  

===============================================================================================