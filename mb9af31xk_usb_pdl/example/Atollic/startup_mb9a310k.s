/*******************************************************************************
*                                                                              *
*   Abstract    : This file contains interrupt vector and startup code.        *
*                                                                              *
*   Functions   : Reset_Handler                                                *
*                                                                              *
*   Target      : Cypress FM microcontrollers                                  *
*                                                                              *
*   Environment : Atollic TrueSTUDIO(R)                                        *
*                                                                              *
*   Distribution: The file is distributed "as is," without any warranty        *
*                 of any kind.                                                 *
*                                                                              *
*******************************************************************************/

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

  .syntax unified
  .thumb

  .global Reset_Handler
  .global InterruptVector
  .global Default_Handler

  /* Linker script definitions */
  /* start address for the initialization values of the .data section */
  .word _sidata
  /* start address for the .data section */
  .word _sdata
  /* end address for the .data section */
  .word _edata
  /* start address for the .bss section */
  .word _sbss
  /* end address for the .bss section */
  .word _ebss

/**
**===========================================================================
**  Program - Reset_Handler
**  Abstract: This code gets called after reset.
**===========================================================================
*/
  .section  .text.Reset_Handler,"ax", %progbits
  .type Reset_Handler, %function
Reset_Handler:
  /* Set stack pointer */
  ldr r0,=_estack
  mov sp, r0
  
  /* START NO FPU IN THIS DEVICE
  ldr     r0, =0xe000ed88           
  ldr     r1,[r0]
  ldr     r2, =0xf00000
  orr     r1,r1,r2
  str     r1,[r0]
  END NO FPU IN THIS DEVICE */
  
  /* Branch to SystemInit function */
  bl SystemInit

  /* Copy data initialization values */
  ldr r1,=_sidata
  ldr r2,=_sdata
  ldr r3,=_edata
  b cmpdata
CopyLoop:
  ldr r0, [r1]
  adds r1, r1, #4
  str r0, [r2]
  adds r2, r2, #4
cmpdata:
  cmp r2, r3
  blt CopyLoop

  /* Clear BSS section */
  movs r0, #0
  ldr r2,=_sbss
  ldr r3,=_ebss
  b cmpbss
ClearLoop:
  str r0, [r2]
  adds r2, r2, #4
cmpbss:
  cmp r2, r3
  blt ClearLoop

  /* Call static constructors */
  bl __libc_init_array

  /* Branch to main */
  bl main

  /* If main returns, branch to Default_Handler. */
  b Default_Handler

  .size  Reset_Handler, .-Reset_Handler

/**
**===========================================================================
**  Program - Default_Handler
**  Abstract: This code gets called when the processor receives an
**    unexpected interrupt.
**===========================================================================
*/
  .section  .text.Default_Handler,"ax", %progbits
Default_Handler:
  b  Default_Handler

  .size  Default_Handler, .-Default_Handler

/**
**===========================================================================
**  Interrupt vector table
**===========================================================================
*/
  .section .isr_vector,"a", %progbits
InterruptVector:
  .word _estack                   /* 0 - Stack pointer */
  .word Reset_Handler             /* 1 - Reset */
  .word NMI_Handler               /* 2 - NMI  */
  .word HardFault_Handler         /* 3 - Hard fault */
  .word MemManage_Handler         /* 4 - Memory management fault */
  .word BusFault_Handler          /* 5 - Bus fault */
  .word UsageFault_Handler        /* 6 - Usage fault */
  .word 0                         /* 7 - Reserved */
  .word 0                         /* 8 - Reserved */
  .word 0                         /* 9 - Reserved */
  .word 0                         /* 10 - Reserved */
  .word SVC_Handler               /* 11 - SVCall */
  .word DebugMonitor_Handler      /* 12 - Reserved for Debug */
  .word 0                         /* 13 - Reserved */
  .word PendSV_Handler            /* 14 - PendSV */
  .word SysTick_Handler           /* 15 - Systick */

  /* External Interrupts */
  .word CSV_IRQHandler            /* IRQ #0 */
  .word SWDT_IRQHandler           /* IRQ #1 */
  .word LVD_IRQHandler            /* IRQ #2 */
  .word MFT0_2_WFG_DTIF_IRQHandler /* IRQ #3 */
  .word EXINT0_7_IRQHandler       /* IRQ #4 */
  .word EXINT8_31_IRQHandler      /* IRQ #5 */
  .word DT_QPRC0_2_IRQHandler     /* IRQ #6 */
  .word MFS0_RX_IRQHandler        /* IRQ #7 */
  .word MFS0_TX_IRQHandler        /* IRQ #8 */
  .word MFS1_RX_IRQHandler        /* IRQ #9 */
  .word MFS1_TX_IRQHandler        /* IRQ #10 */
  .word Default_Handler           /* IRQ #11 */
  .word Default_Handler           /* IRQ #12 */
  .word MFS3_RX_IRQHandler        /* IRQ #13 */
  .word MFS3_TX_IRQHandler        /* IRQ #14 */
  .word Default_Handler           /* IRQ #15 */
  .word Default_Handler           /* IRQ #16 */
  .word MFS5_RX_IRQHandler        /* IRQ #17 */
  .word MFS5_TX_IRQHandler        /* IRQ #18 */
  .word Default_Handler           /* IRQ #19 */
  .word Default_Handler           /* IRQ #20 */
  .word Default_Handler           /* IRQ #21 */
  .word Default_Handler           /* IRQ #22 */
  .word PPG00_02_04_08_10_12_16_18_20_IRQHandler /* IRQ #23 */
  .word TIM_WC_RTC_IRQHandler     /* IRQ #24 */
  .word ADC0_IRQHandler           /* IRQ #25 */
  .word ADC1_IRQHandler           /* IRQ #26 */
  .word Default_Handler           /* IRQ #27 */
  .word MFT0_2_FRT_IRQHandler     /* IRQ #28 */
  .word MFT0_2_ICU_IRQHandler     /* IRQ #29 */
  .word MFT0_2_OCU_IRQHandler     /* IRQ #30 */
  .word BT0_7_IRQHandler          /* IRQ #31 */
  .word Default_Handler           /* IRQ #32 */
  .word Default_Handler           /* IRQ #33 */
  .word USB0_F_IRQHandler         /* IRQ #34 */
  .word USB0_H_F_IRQHandler       /* IRQ #35 */
  .word Default_Handler           /* IRQ #36 */
  .word Default_Handler           /* IRQ #37 */
  .word DMAC0_IRQHandler          /* IRQ #38 */
  .word DMAC1_IRQHandler          /* IRQ #39 */
  .word DMAC2_IRQHandler          /* IRQ #40 */
  .word DMAC3_IRQHandler          /* IRQ #41 */
  .word DMAC4_IRQHandler          /* IRQ #42 */
  .word DMAC5_IRQHandler          /* IRQ #43 */
  .word DMAC6_IRQHandler          /* IRQ #44 */
  .word DMAC7_IRQHandler          /* IRQ #45 */
  .word Default_Handler           /* IRQ #46 */
  .word FLASHIF_IRQHandler        /* IRQ #47 */



/**
**===========================================================================
**  Weak interrupt handlers redirected to Default_Handler. These can be
**  overridden in user code.
**===========================================================================
*/
  .weak NMI_Handler
  .thumb_set NMI_Handler, Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler, Default_Handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler, Default_Handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler, Default_Handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler, Default_Handler

  .weak SVC_Handler
  .thumb_set SVC_Handler, Default_Handler

  .weak DebugMonitor_Handler
  .thumb_set DebugMonitor_Handler, Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler, Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler, Default_Handler
  
/* IRQ #0 */
  .weak CSV_IRQHandler
  .thumb_set CSV_IRQHandler, Default_Handler

/* IRQ #1 */
  .weak SWDT_IRQHandler
  .thumb_set SWDT_IRQHandler, Default_Handler

/* IRQ #2 */
  .weak LVD_IRQHandler
  .thumb_set LVD_IRQHandler, Default_Handler

/* IRQ #3 */
  .weak MFT0_2_WFG_DTIF_IRQHandler
  .thumb_set MFT0_2_WFG_DTIF_IRQHandler, Default_Handler

/* IRQ #4 */
  .weak EXINT0_7_IRQHandler
  .thumb_set EXINT0_7_IRQHandler, Default_Handler

/* IRQ #5 */
  .weak EXINT8_31_IRQHandler
  .thumb_set EXINT8_31_IRQHandler, Default_Handler

/* IRQ #6 */
  .weak DT_QPRC0_2_IRQHandler
  .thumb_set DT_QPRC0_2_IRQHandler, Default_Handler

/* IRQ #7 */
  .weak MFS0_RX_IRQHandler
  .thumb_set MFS0_RX_IRQHandler, Default_Handler

/* IRQ #8 */
  .weak MFS0_TX_IRQHandler
  .thumb_set MFS0_TX_IRQHandler, Default_Handler

/* IRQ #9 */
  .weak MFS1_RX_IRQHandler
  .thumb_set MFS1_RX_IRQHandler, Default_Handler

/* IRQ #10 */
  .weak MFS1_TX_IRQHandler
  .thumb_set MFS1_TX_IRQHandler, Default_Handler

/* IRQ #13 */
  .weak MFS3_RX_IRQHandler
  .thumb_set MFS3_RX_IRQHandler, Default_Handler

/* IRQ #14 */
  .weak MFS3_TX_IRQHandler
  .thumb_set MFS3_TX_IRQHandler, Default_Handler

/* IRQ #17 */
  .weak MFS5_RX_IRQHandler
  .thumb_set MFS5_RX_IRQHandler, Default_Handler

/* IRQ #18 */
  .weak MFS5_TX_IRQHandler
  .thumb_set MFS5_TX_IRQHandler, Default_Handler

/* IRQ #23 */
  .weak PPG00_02_04_08_10_12_16_18_20_IRQHandler
  .thumb_set PPG00_02_04_08_10_12_16_18_20_IRQHandler, Default_Handler

/* IRQ #24 */
  .weak TIM_WC_RTC_IRQHandler
  .thumb_set TIM_WC_RTC_IRQHandler, Default_Handler

/* IRQ #25 */
  .weak ADC0_IRQHandler
  .thumb_set ADC0_IRQHandler, Default_Handler

/* IRQ #26 */
  .weak ADC1_IRQHandler
  .thumb_set ADC1_IRQHandler, Default_Handler

/* IRQ #28 */
  .weak MFT0_2_FRT_IRQHandler
  .thumb_set MFT0_2_FRT_IRQHandler, Default_Handler

/* IRQ #29 */
  .weak MFT0_2_ICU_IRQHandler
  .thumb_set MFT0_2_ICU_IRQHandler, Default_Handler

/* IRQ #30 */
  .weak MFT0_2_OCU_IRQHandler
  .thumb_set MFT0_2_OCU_IRQHandler, Default_Handler

/* IRQ #31 */
  .weak BT0_7_IRQHandler
  .thumb_set BT0_7_IRQHandler, Default_Handler

/* IRQ #34 */
  .weak USB0_F_IRQHandler
  .thumb_set USB0_F_IRQHandler, Default_Handler

/* IRQ #35 */
  .weak USB0_H_F_IRQHandler
  .thumb_set USB0_H_F_IRQHandler, Default_Handler

/* IRQ #38 */
  .weak DMAC0_IRQHandler
  .thumb_set DMAC0_IRQHandler, Default_Handler

/* IRQ #39 */
  .weak DMAC1_IRQHandler
  .thumb_set DMAC1_IRQHandler, Default_Handler

/* IRQ #40 */
  .weak DMAC2_IRQHandler
  .thumb_set DMAC2_IRQHandler, Default_Handler

/* IRQ #41 */
  .weak DMAC3_IRQHandler
  .thumb_set DMAC3_IRQHandler, Default_Handler

/* IRQ #42 */
  .weak DMAC4_IRQHandler
  .thumb_set DMAC4_IRQHandler, Default_Handler

/* IRQ #43 */
  .weak DMAC5_IRQHandler
  .thumb_set DMAC5_IRQHandler, Default_Handler

/* IRQ #44 */
  .weak DMAC6_IRQHandler
  .thumb_set DMAC6_IRQHandler, Default_Handler

/* IRQ #45 */
  .weak DMAC7_IRQHandler
  .thumb_set DMAC7_IRQHandler, Default_Handler

/* IRQ #47 */
  .weak FLASHIF_IRQHandler
  .thumb_set FLASHIF_IRQHandler, Default_Handler



  .end
