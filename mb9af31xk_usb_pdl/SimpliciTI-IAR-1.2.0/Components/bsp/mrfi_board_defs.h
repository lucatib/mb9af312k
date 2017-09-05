/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007 Texas Instruments Incorporated.  All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Board definition file.
 *   Target : Texas Instruments MSP-EXP430FG4618
 *            "MSP430FG4618/F2013 Experimenter Board"
 *   Radios : CC2500, CC1100, CC1101
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

#ifndef MRFI_BOARD_DEFS_H
#define MRFI_BOARD_DEFS_H

/* ------------------------------------------------------------------------------------------------
 *                                           Includes
 * ------------------------------------------------------------------------------------------------
 */
 
#include "bsp.h"
#include "pp_utils.h"

/* ------------------------------------------------------------------------------------------------
 *                                           Defines
 * ------------------------------------------------------------------------------------------------
 */

//extern BSP_ISR_FUNCTION( MRFI_GpioIsr, 0 );

/* ------------------------------------------------------------------------------------------------
 *                                        Radio Selection
 * ------------------------------------------------------------------------------------------------
 */
#if (!defined MRFI_CC1100) && \
    (!defined MRFI_CC1101) && \
    (!defined MRFI_CC1100E_470) && \
    (!defined MRFI_CC1100E_950) && \
    (!defined MRFI_CC2500) && \
    (!defined MRFI_CC2420)
#error "ERROR: A compatible radio must be specified for the EXP461x board."
/*
 *  Since the EXP461x board can support several different radios, the installed
 *  radio must be specified with a #define.  It is best to do this at the
 *  project level.  However, if only one radio will ever be used, a #define
 *  could be placed here, above this error check.
 */
#endif

//Power section
#define PIN_NR_TXON		 				
#define PIN_NR_RXON		 				
#define PIN_NR_SW1ON	 				
#define PIN_NR_SW2ON	 				
#define PIN_NR_PON						

#define MRFI_PU_ON() 					  				
#define MRFI_PU_OFF() 				

#define MRFI_POWER_TX_MODE()  										
#define MRFI_POWER_RX_MODE()			
#define MRFI_POWER_IDLE_MODE()	 		


#define Mrfi_DelayUsec(usec) 		BSP_Delay(usec)

/* ------------------------------------------------------------------------------------------------
 *                                      GDO0 Pin Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define __mrfi_GDO0_BIT__                   
#define __mrfi_GDO0_PORT__                   	
#define __mrfi_GDO0_INT__                   

#define MRFI_CONFIG_GDO0_PIN_AS_INPUT()			Gpio1pin_InitIn( GPIO1PIN_P02, Gpio1pin_InitPullup( 1 ) );																		
#define MRFI_GDO0_PIN_IS_HIGH()           	(Gpio1pin_Get(GPIO1PIN_P02)==1)			
#define MRFI_GDO0_PIN_IS_LOW()             	(Gpio1pin_Get(GPIO1PIN_P02)==0)			

#define MRFI_CONFIG_GDO0_PIN_AS_OUTPUT()		Gpio1pin_InitOut( GPIO1PIN_P02, Gpio1pin_InitVal( 1 ) );
#define MRFI_GDO0_DRIVE_HIGH()       				Gpio1pin_Put( GPIO1PIN_P02, 1 );			
#define MRFI_GDO0_DRIVE_LOW()        				Gpio1pin_Put( GPIO1PIN_P02, 0 );	

#define MRFI_ENABLE_GDO0_INT()              //st(WIU->MR|=BV(__mrfi_GDO0_INT__);)
#define MRFI_DISABLE_GDO0_INT()             //st(WIU->MR&=~BV(__mrfi_GDO0_INT__);)
#define MRFI_GDO0_INT_IS_ENABLED()          FALSE	//(WIU->MR&BV(__mrfi_GDO0_INT__))

#define MRFI_CLEAR_GDO0_INT_FLAG()							//st(WIU->PR=BV(__mrfi_GDO0_INT__);)
#define MRFI_GDO0_INT_FLAG_IS_SET()         		//(WIU->PR&BV(__mrfi_GDO0_INT__)) 
#define MRFI_CONFIG_GDO0_RISING_EDGE_INT()    	///* atomic operation */

void __inline MRFI_CONFIG_GDO0_FALLING_EDGE_INT(void){}
	
/* ------------------------------------------------------------------------------------------------
 *                                      GDO2 Pin Configuration
 * ------------------------------------------------------------------------------------------------
 */

#define __mrfi_GDO2_BIT__               
#define __mrfi_GDO2_PORT__              
#define __mrfi_GDO2_INT__               

#define MRFI_CONFIG_GDO2_PIN_AS_INPUT()					//Gpio1pin_InitIn( GPIO1PIN_P3A, Gpio1pin_InitPullup( 1 ) );

#define MRFI_GDO2_PIN_IS_HIGH()     						FALSE	//(Gpio1pin_Get(GPIO1PIN_P3A)==1)		    			
//#define MRFI_GDO2_INT_VECTOR          				INFIX( PORT, __mrfi_GDO2_PORT__, _VECTOR )
										 
#define MRFI_ENABLE_GDO2_INT()          				//st(WIU->MR |= BV(__mrfi_GDO2_INT__);)
#define MRFI_DISABLE_GDO2_INT()         				//st(WIU->MR &=~BV(__mrfi_GDO2_INT__);)
#define MRFI_GDO2_INT_IS_ENABLED()      				FALSE	//(WIU->MR&BV(__mrfi_GDO2_INT__))

#define MRFI_CLEAR_GDO2_INT_FLAG()      				//st(WIU->PR=BV(__mrfi_GDO2_INT__);)		   	
#define MRFI_GDO2_INT_FLAG_IS_SET()     				//(WIU->PR&BV(__mrfi_GDO2_INT__))
#define MRFI_CONFIG_GDO2_RISING_EDGE_INT()			//

void __inline MRFI_CONFIG_GDO2_FALLING_EDGE_INT(void){}

/* ------------------------------------------------------------------------------------------------
 *                                      SPI Configuration
 * ------------------------------------------------------------------------------------------------
 */

/* CSn Pin Configuration */
#define __mrfi_SPI_CSN_GPIO_BIT__       		
#define MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT()   		Gpio1pin_InitOut( GPIO1PIN_P00, Gpio1pin_InitVal( 1u ) );	//Default High
#define MRFI_SPI_DRIVE_CSN_HIGH()       					Gpio1pin_Put( GPIO1PIN_P00, 1 );			
#define MRFI_SPI_DRIVE_CSN_LOW()        					Gpio1pin_Put( GPIO1PIN_P00, 0 );	 	
#define MRFI_SPI_CSN_IS_HIGH()          					(Gpio1pin_Get( GPIO1PIN_P00) == 1)	

/* SCLK Pin Configuration */
#define __mrfi_SPI_SCLK_GPIO_BIT__         
#define MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT()			SetPinFunc_SCK0_0();
#define MRFI_SPI_DRIVE_SCLK_HIGH()          			
#define MRFI_SPI_DRIVE_SCLK_LOW()           			

/* SI Pin Configuration */
#define __mrfi_SPI_SI_GPIO_BIT__
#define MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT()				SetPinFunc_SOT0_0();			
#define MRFI_SPI_DRIVE_SI_HIGH()            			
#define MRFI_SPI_DRIVE_SI_LOW()             			

/* SO Pin Configuration */
#define __mrfi_SPI_SO_GPIO_BIT__        				
#define MRFI_SPI_CONFIG_SO_PIN_AS_INPUT()					SetPinFunc_SIN0_0();
#define MRFI_SPI_SO_IS_HIGH()           					(Gpio1pin_Get( GPIO1PIN_P21) == 1)

/*
 *  Radio SPI Specifications
 * -----------------------------------------------
 *    Max SPI Clock   :  10 MHz
 *    Data Order      :  MSB transmitted first
 *    Clock Polarity  :  low when idle
 *    Clock Phase     :  sample leading edge
 */

/* SPI Port Configuration */
//#define MRFI_SPI_CONFIG_PORT()                	st( P4SEL |= BV(__mrfi_SPI_SCLK_GPIO_BIT__) |  \
//                                                  BV(__mrfi_SPI_SI_GPIO_BIT__)   |  \
//                                                  BV(__mrfi_SPI_SO_GPIO_BIT__); )

#define MRFI_SPI_WRITE_BYTE(x)											st(																					\
																												Mfs_Csio_EnableFunc(CsioCh0, CsioTx); 	\
																												Mfs_Csio_SendData(CsioCh0, x, FALSE); 	\
																										)
																										
//#define MRFI_SPI_READ_BYTE()											FM3_MFS3_CSIO->RDR

#define MRFI_SPI_WAIT_DONETX()											st(																													\
																												while(TRUE != Mfs_Csio_GetStatus(CsioCh0, CsioTxIdle));	\
																												Mfs_Csio_DisableFunc(CsioCh0, CsioTx); 									\
																										)
#define MRFI_SPI_WAIT_DONERX() 											
#define MRFI_SPI_WAIT_DONE() 											
//#define MRFI_SPI_PURGERX()												st(int dummy;while((FM3_MFS3_CSIO->SSR & (SSR_RDRF)) != 0) dummy = FM3_MFS3_CSIO->RDR;)

/* initialization */	 	
#define MRFI_SPI_INIT()															CsioMasterInit();							        
#define MRFI_SPI_DEINIT() 	   											//

/* SPI critical section macros */
typedef bspIState_t mrfiSpiIState_t;
#define MRFI_SPI_ENTER_CRITICAL_SECTION(x)  	BSP_ENTER_CRITICAL_SECTION(x)	
#define MRFI_SPI_EXIT_CRITICAL_SECTION(x)   	BSP_EXIT_CRITICAL_SECTION(x)											

#define MRFI_SPI_IS_INITIALIZED()     				(1)

#ifdef MRFI_USE_SYNC_PIN_GDO2 
	#define MRFI_CONFIG_ENABLE_SYNC_INT()				MRFI_ENABLE_GDO2_INT()          
	#define MRFI_CONFIG_DISABLE_SYNC_INT()				MRFI_DISABLE_GDO2_INT()  
#else 	
	#define MRFI_CONFIG_ENABLE_SYNC_INT()				MRFI_ENABLE_GDO0_INT()          
	#define MRFI_CONFIG_DISABLE_SYNC_INT()				MRFI_DISABLE_GDO0_INT()   	 
#endif 


#ifdef MRFI_USE_SYNC_PIN_GDO2 
	#define MRFI_SYNC_PIN_IS_HIGH()                     MRFI_GDO2_PIN_IS_HIGH()
	#define MRFI_SYNC_PIN_INT_IS_ENABLED()              MRFI_GDO2_INT_IS_ENABLED()
	#define MRFI_ENABLE_SYNC_PIN_INT()                  st(if(rx_isr_context==false) MRFI_ENABLE_GDO2_INT();)
	#define MRFI_CLEAR_SYNC_PIN_INT_FLAG()              MRFI_CLEAR_GDO2_INT_FLAG()
	#define MRFI_SYNC_PIN_INT_FLAG_IS_SET()             MRFI_GDO2_INT_FLAG_IS_SET()
	#define MRFI_CONFIG_SYNC_PIN_FALLING_EDGE_INT()     MRFI_CONFIG_GDO2_FALLING_EDGE_INT()
	#define MRFI_CONFIG_SYNC_PIN_AS_PAPD_SIGNAL()       mrfiSpiWriteReg(IOCFG2, MRFI_GDO_PA_PD)
	#define MRFI_CONFIG_SYNC_PIN_AS_SYNC_SIGNAL()       mrfiSpiWriteReg(IOCFG2, MRFI_GDO_SYNC)

	//Cpu related
	#define MRFI_CONFIG_SYNC_PIN_AS_INPUT()						MRFI_CONFIG_GDO2_PIN_AS_INPUT()
	#define MRFI_DISABLE_SYNC_PIN_INT()								MRFI_DISABLE_GDO2_INT()

 #else
 
 	#define MRFI_SYNC_PIN_IS_HIGH()                     MRFI_GDO0_PIN_IS_HIGH()
	#define MRFI_SYNC_PIN_INT_IS_ENABLED()              MRFI_GDO0_INT_IS_ENABLED()
	#define MRFI_ENABLE_SYNC_PIN_INT()                  st(if(rx_isr_context==false) MRFI_ENABLE_GDO0_INT();)
	#define MRFI_CLEAR_SYNC_PIN_INT_FLAG()              MRFI_CLEAR_GDO0_INT_FLAG()
	#define MRFI_SYNC_PIN_INT_FLAG_IS_SET()             MRFI_GDO0_INT_FLAG_IS_SET()
	#define MRFI_CONFIG_SYNC_PIN_FALLING_EDGE_INT()     MRFI_CONFIG_GDO0_FALLING_EDGE_INT()
	#define MRFI_CONFIG_SYNC_PIN_AS_PAPD_SIGNAL()       mrfiSpiWriteReg(IOCFG0, MRFI_GDO_PA_PD)
	#define MRFI_CONFIG_SYNC_PIN_AS_SYNC_SIGNAL()       mrfiSpiWriteReg(IOCFG0, MRFI_GDO_SYNC)

	//Cpu related
	#define MRFI_CONFIG_SYNC_PIN_AS_INPUT()					MRFI_CONFIG_GDO0_PIN_AS_INPUT()
	#define MRFI_DISABLE_SYNC_PIN_INT()							MRFI_DISABLE_GDO0_INT()

#endif
/**************************************************************************************************
 */
#endif
