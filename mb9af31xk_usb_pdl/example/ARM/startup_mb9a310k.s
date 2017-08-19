;/*******************************************************************************
;*                                                                              *
;*   Abstract    : This file contains interrupt vector and startup code.        *
;*                                                                              *
;*   Functions   : Reset_Handler                                                *
;*                                                                              *
;*   Target      : Cypress FM microcontrollers                                  *
;*                                                                              *
;*   Environment : KEIL µVISION	                                                *
;*                                                                              *
;*   Distribution: The file is distributed "as is," without any warranty        *
;*                 of any kind.                                                 *
;*                                                                              *
;********************************************************************************
;/*******************************************************************************
;* Copyright (C) 2013-2016, Cypress Semiconductor Corporation or a              *
;* subsidiary of Cypress Semiconductor Corporation.  All rights reserved.       *
;*                                                                              *
;* This software, including source code, documentation and related              *
;* materials ("Software"), is owned by Cypress Semiconductor Corporation or     *
;* one of its subsidiaries ("Cypress") and is protected by and subject to       *
;* worldwide patent protection (United States and foreign), United States       *
;* copyright laws and international treaty provisions. Therefore, you may use   *
;* this Software only as provided in the license agreement accompanying the     *
;* software package from which you obtained this Software ("EULA").             *
;*                                                                              *
;* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,     *
;* non-transferable license to copy, modify, and compile the                    *
;* Software source code solely for use in connection with Cypress's             *
;* integrated circuit products.  Any reproduction, modification, translation,   *
;* compilation, or representation of this Software except as specified          *
;* above is prohibited without the express written permission of Cypress.       *
;*                                                                              *
;* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                         *
;* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                         *
;* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                 *
;* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                              *
;* PARTICULAR PURPOSE. Cypress reserves the right to make                       *
;* changes to the Software without notice. Cypress does not assume any          *
;* liability arising out of the application or use of the Software or any       *
;* product or circuit described in the Software. Cypress does not               *
;* authorize its products for use in any products where a malfunction or        *
;* failure of the Cypress product may reasonably be expected to result in       *
;* significant property damage, injury or death ("High Risk Product"). By       *
;* including Cypress's product in a High Risk Product, the manufacturer         *
;* of such system or application assumes all risk of such use and in doing      *
;* so agrees to indemnify Cypress against all liability.                        *
;*******************************************************************************/

; Stack Configuration
;  Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>

Stack_Size      EQU     0x00001000

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; Heap Configuration
;  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>

Heap_Size       EQU     0x00000800

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

; Numbered IRQ handler vectors				
				
                DCD     CSV_IRQHandler		; IRQ #0
                DCD     SWDT_IRQHandler		; IRQ #1
                DCD     LVD_IRQHandler		; IRQ #2
                DCD     MFT0_2_WFG_DTIF_IRQHandler		; IRQ #3
                DCD     EXINT0_7_IRQHandler		; IRQ #4
                DCD     EXINT8_31_IRQHandler		; IRQ #5
                DCD     DT_QPRC0_2_IRQHandler		; IRQ #6
                DCD     MFS0_RX_IRQHandler		; IRQ #7
                DCD     MFS0_TX_IRQHandler		; IRQ #8
                DCD     MFS1_RX_IRQHandler		; IRQ #9
                DCD     MFS1_TX_IRQHandler		; IRQ #10
                DCD     Dummy		; IRQ #11
                DCD     Dummy		; IRQ #12
                DCD     MFS3_RX_IRQHandler		; IRQ #13
                DCD     MFS3_TX_IRQHandler		; IRQ #14
                DCD     Dummy		; IRQ #15
                DCD     Dummy		; IRQ #16
                DCD     MFS5_RX_IRQHandler		; IRQ #17
                DCD     MFS5_TX_IRQHandler		; IRQ #18
                DCD     Dummy		; IRQ #19
                DCD     Dummy		; IRQ #20
                DCD     Dummy		; IRQ #21
                DCD     Dummy		; IRQ #22
                DCD     PPG00_02_04_08_10_12_16_18_20_IRQHandler		; IRQ #23
                DCD     TIM_WC_RTC_IRQHandler		; IRQ #24
                DCD     ADC0_IRQHandler		; IRQ #25
                DCD     ADC1_IRQHandler		; IRQ #26
                DCD     Dummy		; IRQ #27
                DCD     MFT0_2_FRT_IRQHandler		; IRQ #28
                DCD     MFT0_2_ICU_IRQHandler		; IRQ #29
                DCD     MFT0_2_OCU_IRQHandler		; IRQ #30
                DCD     BT0_7_IRQHandler		; IRQ #31
                DCD     Dummy		; IRQ #32
                DCD     Dummy		; IRQ #33
                DCD     USB0_F_IRQHandler		; IRQ #34
                DCD     USB0_H_F_IRQHandler		; IRQ #35
                DCD     Dummy		; IRQ #36
                DCD     Dummy		; IRQ #37
                DCD     DMAC0_IRQHandler		; IRQ #38
                DCD     DMAC1_IRQHandler		; IRQ #39
                DCD     DMAC2_IRQHandler		; IRQ #40
                DCD     DMAC3_IRQHandler		; IRQ #41
                DCD     DMAC4_IRQHandler		; IRQ #42
                DCD     DMAC5_IRQHandler		; IRQ #43
                DCD     DMAC6_IRQHandler		; IRQ #44
                DCD     DMAC7_IRQHandler		; IRQ #45
                DCD     Dummy		; IRQ #46
                DCD     FLASHIF_IRQHandler		; IRQ #47


__Vectors_End

__Vectors_Size 	EQU 	__Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

;No FPU                LDR.W R0, =0xE000ED88            ; CPACR is located at address 0xE000ED88
;No FPU                LDR R1, [R0]                     ; Read CPACR
;No FPU                ORR R1, R1, #(0xF << 20)         ; Set bits 20-23 to enable CP10 and CP11 coprocessors
;No FPU                STR R1, [R0]                     ; Write back the modified value to the CPACR

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  CSV_IRQHandler    [WEAK]
                EXPORT  SWDT_IRQHandler    [WEAK]
                EXPORT  LVD_IRQHandler    [WEAK]
                EXPORT  MFT0_2_WFG_DTIF_IRQHandler    [WEAK]
                EXPORT  EXINT0_7_IRQHandler    [WEAK]
                EXPORT  EXINT8_31_IRQHandler    [WEAK]
                EXPORT  DT_QPRC0_2_IRQHandler    [WEAK]
                EXPORT  MFS0_RX_IRQHandler    [WEAK]
                EXPORT  MFS0_TX_IRQHandler    [WEAK]
                EXPORT  MFS1_RX_IRQHandler    [WEAK]
                EXPORT  MFS1_TX_IRQHandler    [WEAK]
                EXPORT  MFS3_RX_IRQHandler    [WEAK]
                EXPORT  MFS3_TX_IRQHandler    [WEAK]
                EXPORT  MFS5_RX_IRQHandler    [WEAK]
                EXPORT  MFS5_TX_IRQHandler    [WEAK]
                EXPORT  PPG00_02_04_08_10_12_16_18_20_IRQHandler    [WEAK]
                EXPORT  TIM_WC_RTC_IRQHandler    [WEAK]
                EXPORT  ADC0_IRQHandler    [WEAK]
                EXPORT  ADC1_IRQHandler    [WEAK]
                EXPORT  MFT0_2_FRT_IRQHandler    [WEAK]
                EXPORT  MFT0_2_ICU_IRQHandler    [WEAK]
                EXPORT  MFT0_2_OCU_IRQHandler    [WEAK]
                EXPORT  BT0_7_IRQHandler    [WEAK]
                EXPORT  USB0_F_IRQHandler    [WEAK]
                EXPORT  USB0_H_F_IRQHandler    [WEAK]
                EXPORT  DMAC0_IRQHandler    [WEAK]
                EXPORT  DMAC1_IRQHandler    [WEAK]
                EXPORT  DMAC2_IRQHandler    [WEAK]
                EXPORT  DMAC3_IRQHandler    [WEAK]
                EXPORT  DMAC4_IRQHandler    [WEAK]
                EXPORT  DMAC5_IRQHandler    [WEAK]
                EXPORT  DMAC6_IRQHandler    [WEAK]
                EXPORT  DMAC7_IRQHandler    [WEAK]
                EXPORT  FLASHIF_IRQHandler    [WEAK]

                
                EXPORT  Dummy	          [WEAK]

CSV_IRQHandler		; IRQ #0
SWDT_IRQHandler		; IRQ #1
LVD_IRQHandler		; IRQ #2
MFT0_2_WFG_DTIF_IRQHandler		; IRQ #3
EXINT0_7_IRQHandler		; IRQ #4
EXINT8_31_IRQHandler		; IRQ #5
DT_QPRC0_2_IRQHandler		; IRQ #6
MFS0_RX_IRQHandler		; IRQ #7
MFS0_TX_IRQHandler		; IRQ #8
MFS1_RX_IRQHandler		; IRQ #9
MFS1_TX_IRQHandler		; IRQ #10
MFS3_RX_IRQHandler		; IRQ #13
MFS3_TX_IRQHandler		; IRQ #14
MFS5_RX_IRQHandler		; IRQ #17
MFS5_TX_IRQHandler		; IRQ #18
PPG00_02_04_08_10_12_16_18_20_IRQHandler		; IRQ #23
TIM_WC_RTC_IRQHandler		; IRQ #24
ADC0_IRQHandler		; IRQ #25
ADC1_IRQHandler		; IRQ #26
MFT0_2_FRT_IRQHandler		; IRQ #28
MFT0_2_ICU_IRQHandler		; IRQ #29
MFT0_2_OCU_IRQHandler		; IRQ #30
BT0_7_IRQHandler		; IRQ #31
USB0_F_IRQHandler		; IRQ #34
USB0_H_F_IRQHandler		; IRQ #35
DMAC0_IRQHandler		; IRQ #38
DMAC1_IRQHandler		; IRQ #39
DMAC2_IRQHandler		; IRQ #40
DMAC3_IRQHandler		; IRQ #41
DMAC4_IRQHandler		; IRQ #42
DMAC5_IRQHandler		; IRQ #43
DMAC6_IRQHandler		; IRQ #44
DMAC7_IRQHandler		; IRQ #45
FLASHIF_IRQHandler		; IRQ #47

Dummy


                B       .

                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB
                
                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit
                
                ELSE
                
                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, = Heap_Mem
                LDR     R1, = (Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem + Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END
