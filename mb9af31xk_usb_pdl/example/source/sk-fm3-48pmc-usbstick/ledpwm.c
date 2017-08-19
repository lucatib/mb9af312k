/**************************************************************************
* Copyright (C)2011 Spansion LLC. All Rights Reserved . 
*
* This software is owned and published by: 
* Spansion LLC, 915 DeGuigne Dr. Sunnyvale, CA  94088-3453 ("Spansion").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND 
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software constitutes driver source code for use in programming Spansion's 
* Flash memory components. This software is licensed by Spansion to be adapted only 
* for use in systems utilizing Spansion's Flash memories. Spansion is not be 
* responsible for misuse or illegal use of this software for devices not 
* supported herein.  Spansion is providing this source code "AS IS" and will 
* not be responsible for issues arising from incorrect user implementation 
* of the source code herein.  
*
* SPANSION MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE, 
* REGARDING THE SOFTWARE, ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED 
* USE, INCLUDING, WITHOUT LIMITATION, NO IMPLIED WARRANTY OF MERCHANTABILITY, 
* FITNESS FOR A  PARTICULAR PURPOSE OR USE, OR NONINFRINGEMENT.  SPANSION WILL 
* HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT, NEGLIGENCE OR 
* OTHERWISE) FOR ANY DAMAGES ARISING FROM USE OR INABILITY TO USE THE SOFTWARE, 
* INCLUDING, WITHOUT LIMITATION, ANY DIRECT, INDIRECT, INCIDENTAL, 
* SPECIAL, OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA, SAVINGS OR PROFITS, 
* EVEN IF SPANSION HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.  
*
* This software may be replicated in part or whole for the licensed use, 
* with the restriction that this Copyright notice must be included with 
* this software, whether used in part or whole, at all times.  
*******************************************************************************/
/** \file ledpwm.c
 **
 ** for SK-FM3-48PMC-USBSTICK
 **
 ** History:
 **   2012-02-03  1.0  MSc  First version for SK-FM3-48PMC-USBSTICK
 **   2012-07-17  1.1  MSc  base_type.h -> base_types.h
 **   2012-08-31  1.2  MSc  support for L3 rename ppg.c --> ledpwm.c
 **   2014-03-11  1.3  MSc  base_type_l3.h -> base_types.h
 *****************************************************************************/
#include "ledpwm.h"


/******************************************************************************/
/* Global variable definitions                                                */
/******************************************************************************/
#if (LEDPWM_USE_L3 == 1)
stc_bt_config_t stcBtConfig0 ;
stc_bt_config_t stcBtConfig2 ;
stc_bt_config_t stcBtConfig4 ;
#endif

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

#if (LEDPWM_USE_L3 == 0)
void LedPwm_Init(void)
{
  /*Enable peripherial ports to be used with PWM*/
  FM3_GPIO->PFR3   = 0x5400;
  FM3_GPIO->EPFR04 = 0x00080008;
  FM3_GPIO->EPFR05 = 0x00000008;
	
  /* bit11:RTGEN=0b0 reboot by triger disable */
  /* bit10:PMSK=0b0 output mask disable */
  /* bit9,8:EGS1,0=0b00 triger input none  */
  /* bit7:T32=0b0 16bit mode */
  /* bit6~4:FMD2~0=0b001 PWM timer mode */
  /* bit3:OSEL=0b0 output level normal */
  /* bit2:MDSE=0b0 output mode continue */
  FM3_BT0_PWM->TMCR = 0x3010;
  FM3_BT2_PWM->TMCR = 0x3010;
  FM3_BT4_PWM->TMCR = 0x3010;
  
  /* bit0+TMCR.bit14~12:CKS3~0=0b0011 count clock prescaler PCLK1/128 */
  FM3_BT0_PWM->TMCR2 = 0x00;
  FM3_BT2_PWM->TMCR2 = 0x00;
  FM3_BT4_PWM->TMCR2 = 0x00;
  
 /* bit6:TGIE=0b0 triger interrupt disable */
 /* bit5:DTIE=0b0 duty match interrupt disable */
 /* bit4:UDIE=0b0 under flow interrupt disable */
  FM3_BT0_PWM->STC   = 0x00;
  FM3_BT2_PWM->STC   = 0x00;
  FM3_BT4_PWM->STC   = 0x00;  
  
  /* Down count*/
  LedPwm_SetCycle(0,PWM_CYCLE);
  LedPwm_SetCycle(2,PWM_CYCLE);
  LedPwm_SetCycle(4,PWM_CYCLE);
  
  /* bit1:CTEN=0b1 down counter enable */
  /* bit0:STRG=0b1 started by software */
  FM3_BT0_PWM->TMCR |= 0x03;
  FM3_BT2_PWM->TMCR |= 0x03;
  FM3_BT4_PWM->TMCR |= 0x03;
}

void LedPwm_SetDuty(uint8_t u8Channel, uint8_t u8Duty)
{
  switch(u8Channel)
  {
  case 0:
      FM3_BT0_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PA = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PA = 1;
      }
      break;
  case 1:
      FM3_BT1_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PB = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PB = 1;
      }
      break;
  case 2:
      FM3_BT2_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PC = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PC = 1;
      }
      break;
  case 3:
      FM3_BT3_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PD = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PD = 1;
      }
      break;
  case 4:
      FM3_BT4_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PE = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PE = 1;
      }
      break;
  case 5:
      FM3_BT5_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PF = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PF = 1;
      }
      break;
  default:
    break;
  }
}

void LedPwm_SetCycle(uint8_t u8Channel, uint8_t u8Cycle)
{
  switch(u8Channel)
  {
  case 0:
      FM3_BT0_PWM->PCSR = u8Cycle;
      break;
  case 1:
      FM3_BT1_PWM->PCSR = u8Cycle;
      break;
  case 2:
      FM3_BT2_PWM->PCSR = u8Cycle;
      break;
  case 3:
      FM3_BT3_PWM->PCSR = u8Cycle;
      break;
  case 4:
      FM3_BT4_PWM->PCSR = u8Cycle;
      break;
  case 5:
      FM3_BT5_PWM->PCSR = u8Cycle;
      break;
  default:
    break;
  }
}

#else /* (LEDPWM_USE_L3 == 1) */

void LedPwm_Init(void)
{
    /*Enable peripherial ports to be used with PWM*/
    FM3_GPIO->PFR3   = 0x5400;
    FM3_GPIO->EPFR04 = 0x00080008;
    FM3_GPIO->EPFR05 = 0x00000008;
    
    stcBtConfig0.enMode = BtPwm;          
    stcBtConfig0.enPrescaler = BtIntClkDiv128;    
    stcBtConfig0.bRestart = TRUE;      
    stcBtConfig0.bOutputLow = FALSE;        
    stcBtConfig0.bOutputInvert = FALSE;    
    stcBtConfig0.bTimer32 = FALSE;  // PWM has no 32 bit mode!
    stcBtConfig0.bOneShot = FALSE;           
    stcBtConfig0.enExtTriggerMode = BtTriggerDisable;       
    stcBtConfig0.u16Cycle = PWM_CYCLE;       
    stcBtConfig0.u16Duty = 0;           
    stcBtConfig0.bTriggerIrqEnable = FALSE;
    stcBtConfig0.bUnderflowIrqEnable = FALSE; 
    stcBtConfig0.bDutyIrqEnable = FALSE;     
    stcBtConfig0.pfnCallback0 = NULL; // trigger irq callback
    stcBtConfig0.pfnCallback1 = NULL; // underflow irq callback
    stcBtConfig0.pfnCallback2 = NULL; // duty irq callback
    Bt_ResetIoSel();
    if (Ok == Bt_Init((stc_btn_t*)&BT0, &stcBtConfig0))
    {
        Bt_Enable((stc_btn_t*)&BT0) ;
        Bt_Trigger((stc_btn_t*)&BT0) ;
    }

    stcBtConfig2.enMode = BtPwm;          
    stcBtConfig2.enPrescaler = BtIntClkDiv128;    
    stcBtConfig2.bRestart = TRUE;      
    stcBtConfig2.bOutputLow = FALSE;        
    stcBtConfig2.bOutputInvert = FALSE;    
    stcBtConfig2.bTimer32 = FALSE;  // PWM has no 32 bit mode!
    stcBtConfig2.bOneShot = FALSE;           
    stcBtConfig2.enExtTriggerMode = BtTriggerDisable;       
    stcBtConfig2.u16Cycle = PWM_CYCLE;       
    stcBtConfig2.u16Duty = 0;           
    stcBtConfig2.bTriggerIrqEnable = FALSE;
    stcBtConfig2.bUnderflowIrqEnable = FALSE; 
    stcBtConfig2.bDutyIrqEnable = FALSE;     
    stcBtConfig2.pfnCallback0 = NULL; // trigger irq callback
    stcBtConfig2.pfnCallback1 = NULL; // underflow irq callback
    stcBtConfig2.pfnCallback2 = NULL; // duty irq callback
    Bt_ResetIoSel();
    if (Ok == Bt_Init((stc_btn_t*)&BT2, &stcBtConfig2))
    {
        Bt_Enable((stc_btn_t*)&BT2) ;
        Bt_Trigger((stc_btn_t*)&BT2) ;
    }
    
    stcBtConfig4.enMode = BtPwm;          
    stcBtConfig4.enPrescaler = BtIntClkDiv128;    
    stcBtConfig4.bRestart = TRUE;      
    stcBtConfig4.bOutputLow = FALSE;        
    stcBtConfig4.bOutputInvert = FALSE;    
    stcBtConfig4.bTimer32 = FALSE;  // PWM has no 32 bit mode!
    stcBtConfig4.bOneShot = FALSE;           
    stcBtConfig4.enExtTriggerMode = BtTriggerDisable;       
    stcBtConfig4.u16Cycle = PWM_CYCLE;       
    stcBtConfig4.u16Duty = 0;           
    stcBtConfig4.bTriggerIrqEnable = FALSE;
    stcBtConfig4.bUnderflowIrqEnable = FALSE; 
    stcBtConfig4.bDutyIrqEnable = FALSE;     
    stcBtConfig4.pfnCallback0 = NULL; // trigger irq callback
    stcBtConfig4.pfnCallback1 = NULL; // underflow irq callback
    stcBtConfig4.pfnCallback2 = NULL; // duty irq callback
    Bt_ResetIoSel();
    if (Ok == Bt_Init((stc_btn_t*)&BT4, &stcBtConfig4))
    {
        Bt_Enable((stc_btn_t*)&BT4) ;
        Bt_Trigger((stc_btn_t*)&BT4) ;
    }
}

void LedPwm_SetDuty(uint8_t u8Channel, uint8_t u8Duty)
{
  switch(u8Channel)
  {
  case 0:
      FM3_BT0_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PA = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PA = 1;
      }
      break;
  case 1:
      FM3_BT1_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PB = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PB = 1;
      }
      break;
  case 2:
      FM3_BT2_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PC = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PC = 1;
      }
      break;
  case 3:
      FM3_BT3_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PD = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PD = 1;
      }
      break;
  case 4:
      FM3_BT4_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PE = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PE = 1;
      }
      break;
  case 5:
      FM3_BT5_PWM->PDUT = u8Duty;
      if (u8Duty == 0)
      {
          bFM3_GPIO_PFR3_PF = 0;
      }
      else
      {
          bFM3_GPIO_PFR3_PF = 1;
      }
      break;
  default:
    break;
  }
}

void LedPwm_SetCycle(uint8_t u8Channel, uint8_t u8Cycle)
{
  switch(u8Channel)
  {
  case 0:
      FM3_BT0_PWM->PCSR = u8Cycle;
      break;
  case 1:
      FM3_BT1_PWM->PCSR = u8Cycle;
      break;
  case 2:
      FM3_BT2_PWM->PCSR = u8Cycle;
      break;
  case 3:
      FM3_BT3_PWM->PCSR = u8Cycle;
      break;
  case 4:
      FM3_BT4_PWM->PCSR = u8Cycle;
      break;
  case 5:
      FM3_BT5_PWM->PCSR = u8Cycle;
      break;
  default:
    break;
  }
}

#endif
