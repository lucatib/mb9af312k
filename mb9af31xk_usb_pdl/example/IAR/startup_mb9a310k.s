;/*******************************************************************************
;*                                                                              *
;*   Abstract    : This file contains interrupt vector and startup code.        *
;*                                                                              *
;*   Functions   : Reset_Handler                                                *
;*                                                                              *
;*   Target      : Cypress FM microcontrollers                                  *
;*                                                                              *
;*   Environment : IAR Embedded Workbench                                       *
;*                                                                              *
;*   Distribution: The file is distributed "as is," without any warranty        *
;*                 of any kind.                                                 *
;*                                                                              *
;*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2013-2016, Cypress Semiconductor Corporation or a              *
* subsidiary of Cypress Semiconductor Corporation.  All rights reserved.       *
*                                                                              *
* This software, including source code, documentation and related              *
* materials ("Software"), is owned by Cypress Semiconductor Corporation or     *
* one of its subsidiaries ("Cypress") and is protected by and subject to       *
* worldwide patent protection (United States and foreign), United States       *
* copyright laws and international treaty provisions. Therefore, you may use   *
* this Software only as provided in the license agreement accompanying the     *
* software package from which you obtained this Software ("EULA").             *
*                                                                              *
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,     *
* non-transferable license to copy, modify, and compile the                    *
* Software source code solely for use in connection with Cypress's             *
* integrated circuit products.  Any reproduction, modification, translation,   *
* compilation, or representation of this Software except as specified          *
* above is prohibited without the express written permission of Cypress.       *
*                                                                              *
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                         *
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                         *
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                 *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                              *
* PARTICULAR PURPOSE. Cypress reserves the right to make                       *
* changes to the Software without notice. Cypress does not assume any          *
* liability arising out of the application or use of the Software or any       *
* product or circuit described in the Software. Cypress does not               *
* authorize its products for use in any products where a malfunction or        *
* failure of the Cypress product may reasonably be expected to result in       *
* significant property damage, injury or death ("High Risk Product"). By       *
* including Cypress's product in a High Risk Product, the manufacturer         *
* of such system or application assumes all risk of such use and in doing      *
* so agrees to indemnify Cypress against all liability.                        *
*******************************************************************************/

;/*****************************************************************************/
;/*  Startup for IAR                                                          */
;/*  Version     V1.0                                                         */
;/*  Date        2013-03-22                                                   */
;/*****************************************************************************/


                MODULE  ?cstartup

                ;; Forward declaration of sections.
                SECTION CSTACK:DATA:NOROOT(3)

                SECTION .intvec:CODE:NOROOT(2)

                EXTERN  __iar_program_start
                EXTERN  SystemInit
                PUBLIC  __vector_table

                DATA
__vector_table  DCD     sfe(CSTACK)               ; Top of Stack
		        DCD     Reset_Handler             ; Reset
                DCD     NMI_Handler               ; NMI
                DCD     HardFault_Handler         ; Hard Fault
                DCD     MemManage_Handler         ; MPU Fault
                DCD     BusFault_Handler          ; Bus Fault
                DCD     UsageFault_Handler        ; Usage Fault
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall
                DCD     DebugMon_Handler          ; Debug Monitor
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV
                DCD     SysTick_Handler           ; SysTick

; Numbered IRQ handler vectors

; Note: renaming to device dependent ISR function names are done in
;       pdl.h (section "IRQ name definition for all type MCUs"

                DCD     CSV_IRQHandler
                DCD     SWDT_IRQHandler
                DCD     LVD_IRQHandler
                DCD     MFT0_2_WFG_DTIF_IRQHandler
                DCD     EXINT0_7_IRQHandler
                DCD     EXINT8_31_IRQHandler
                DCD     DT_QPRC0_2_IRQHandler
                DCD     MFS0_RX_IRQHandler
                DCD     MFS0_TX_IRQHandler
                DCD     MFS1_RX_IRQHandler
                DCD     MFS1_TX_IRQHandler
                DCD     Dummy
                DCD     Dummy
                DCD     MFS3_RX_IRQHandler
                DCD     MFS3_TX_IRQHandler
                DCD     Dummy
                DCD     Dummy
                DCD     MFS5_RX_IRQHandler
                DCD     MFS5_TX_IRQHandler
                DCD     Dummy
                DCD     Dummy
                DCD     Dummy
                DCD     Dummy
                DCD     PPG00_02_04_08_10_12_16_18_20_IRQHandler
                DCD     TIM_WC_RTC_IRQHandler
                DCD     ADC0_IRQHandler
                DCD     ADC1_IRQHandler
                DCD     Dummy
                DCD     MFT0_2_FRT_IRQHandler
                DCD     MFT0_2_ICU_IRQHandler
                DCD     MFT0_2_OCU_IRQHandler
                DCD     BT0_7_IRQHandler
                DCD     Dummy
                DCD     Dummy
                DCD     USB0_F_IRQHandler
                DCD     USB0_H_F_IRQHandler
                DCD     Dummy
                DCD     Dummy
                DCD     DMAC0_IRQHandler
                DCD     DMAC1_IRQHandler
                DCD     DMAC2_IRQHandler
                DCD     DMAC3_IRQHandler
                DCD     DMAC4_IRQHandler
                DCD     DMAC5_IRQHandler
                DCD     DMAC6_IRQHandler
                DCD     DMAC7_IRQHandler
                DCD     Dummy
                DCD     FLASHIF_IRQHandler


                THUMB
; Dummy Exception Handlers (infinite loops which can be modified)

                PUBWEAK Reset_Handler
                SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler

;No FPU                LDR.W R0, =0xE000ED88            ; CPACR is located at address 0xE000ED88
;No FPU                LDR R1, [R0]                     ; Read CPACR
;No FPU                ORR R1, R1, #(0xF << 20)         ; Set bits 20-23 to enable CP10 and CP11 coprocessors
;No FPU                STR R1, [R0]                     ; Write back the modified value to the CPACR

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__iar_program_start
                BX      R0
                
                PUBWEAK NMI_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
                B       NMI_Handler

                PUBWEAK HardFault_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
HardFault_Handler
                B       HardFault_Handler

                PUBWEAK MemManage_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
                B       MemManage_Handler

                PUBWEAK BusFault_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
                B       BusFault_Handler

                PUBWEAK UsageFault_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
                B       UsageFault_Handler

                PUBWEAK SVC_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
                B       SVC_Handler

                PUBWEAK DebugMon_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
                B       DebugMon_Handler

                PUBWEAK PendSV_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
                B       PendSV_Handler

                PUBWEAK SysTick_Handler
                SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
                B       SysTick_Handler

                
                PUBWEAK CSV_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
CSV_IRQHandler
                B       CSV_IRQHandler

                PUBWEAK SWDT_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
SWDT_IRQHandler
                B       SWDT_IRQHandler

                PUBWEAK LVD_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
LVD_IRQHandler
                B       LVD_IRQHandler

                PUBWEAK MFT0_2_WFG_DTIF_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFT0_2_WFG_DTIF_IRQHandler
                B       MFT0_2_WFG_DTIF_IRQHandler

                PUBWEAK EXINT0_7_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
EXINT0_7_IRQHandler
                B       EXINT0_7_IRQHandler

                PUBWEAK EXINT8_31_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
EXINT8_31_IRQHandler
                B       EXINT8_31_IRQHandler

                PUBWEAK DT_QPRC0_2_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DT_QPRC0_2_IRQHandler
                B       DT_QPRC0_2_IRQHandler

                PUBWEAK MFS0_RX_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFS0_RX_IRQHandler
                B       MFS0_RX_IRQHandler

                PUBWEAK MFS0_TX_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFS0_TX_IRQHandler
                B       MFS0_TX_IRQHandler

                PUBWEAK MFS1_RX_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFS1_RX_IRQHandler
                B       MFS1_RX_IRQHandler

                PUBWEAK MFS1_TX_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFS1_TX_IRQHandler
                B       MFS1_TX_IRQHandler

                PUBWEAK MFS3_RX_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFS3_RX_IRQHandler
                B       MFS3_RX_IRQHandler

                PUBWEAK MFS3_TX_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFS3_TX_IRQHandler
                B       MFS3_TX_IRQHandler

                PUBWEAK MFS5_RX_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFS5_RX_IRQHandler
                B       MFS5_RX_IRQHandler

                PUBWEAK MFS5_TX_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFS5_TX_IRQHandler
                B       MFS5_TX_IRQHandler

                PUBWEAK PPG00_02_04_08_10_12_16_18_20_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
PPG00_02_04_08_10_12_16_18_20_IRQHandler
                B       PPG00_02_04_08_10_12_16_18_20_IRQHandler

                PUBWEAK TIM_WC_RTC_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
TIM_WC_RTC_IRQHandler
                B       TIM_WC_RTC_IRQHandler

                PUBWEAK ADC0_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
ADC0_IRQHandler
                B       ADC0_IRQHandler

                PUBWEAK ADC1_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
ADC1_IRQHandler
                B       ADC1_IRQHandler

                PUBWEAK MFT0_2_FRT_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFT0_2_FRT_IRQHandler
                B       MFT0_2_FRT_IRQHandler

                PUBWEAK MFT0_2_ICU_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFT0_2_ICU_IRQHandler
                B       MFT0_2_ICU_IRQHandler

                PUBWEAK MFT0_2_OCU_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
MFT0_2_OCU_IRQHandler
                B       MFT0_2_OCU_IRQHandler

                PUBWEAK BT0_7_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
BT0_7_IRQHandler
                B       BT0_7_IRQHandler

                PUBWEAK USB0_F_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
USB0_F_IRQHandler
                B       USB0_F_IRQHandler

                PUBWEAK USB0_H_F_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
USB0_H_F_IRQHandler
                B       USB0_H_F_IRQHandler

                PUBWEAK DMAC0_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DMAC0_IRQHandler
                B       DMAC0_IRQHandler

                PUBWEAK DMAC1_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DMAC1_IRQHandler
                B       DMAC1_IRQHandler

                PUBWEAK DMAC2_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DMAC2_IRQHandler
                B       DMAC2_IRQHandler

                PUBWEAK DMAC3_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DMAC3_IRQHandler
                B       DMAC3_IRQHandler

                PUBWEAK DMAC4_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DMAC4_IRQHandler
                B       DMAC4_IRQHandler

                PUBWEAK DMAC5_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DMAC5_IRQHandler
                B       DMAC5_IRQHandler

                PUBWEAK DMAC6_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DMAC6_IRQHandler
                B       DMAC6_IRQHandler

                PUBWEAK DMAC7_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
DMAC7_IRQHandler
                B       DMAC7_IRQHandler

                PUBWEAK FLASHIF_IRQHandler
                SECTION .text:CODE:REORDER:NOROOT(1)
FLASHIF_IRQHandler
                B       FLASHIF_IRQHandler




                PUBWEAK Dummy
                SECTION .text:CODE:REORDER:NOROOT(1)
Dummy
                B       Dummy

                END
