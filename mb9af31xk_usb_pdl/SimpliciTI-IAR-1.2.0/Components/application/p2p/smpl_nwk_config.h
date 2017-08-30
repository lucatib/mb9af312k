/**************************************************************************************************
  Filename:       smpl_nwk_config.dat
  Revised:        $Date: 2009-02-07 14:21:07 -0700 (Sat, 07 Feb 2009) $
  Revision:       $Revision: 19010 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Customer Configuration for overall network.

  Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.

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

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS�
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


/* max hop count */
#define MAX_HOPS 2

/* max hops away from and AP. Keeps hop count and therefore replay
 * storms down for sending to and from polling End Devices. Also used
 * when joining since the EDs can't be more than 1 hop away.
 */
#define MAX_HOPS_FROM_AP 1

/* Maximum size of Network application payload. Do not change unless
 * protocol changes are reflected in different maximum network
 * application payload size.
 */
#define MAX_NWK_PAYLOAD 50

/* Maximum size of application payload */
#define MAX_APP_PAYLOAD 50

/* default Link token */
#define DEFAULT_LINK_TOKEN 0x01020304

/* default Join token */
#define DEFAULT_JOIN_TOKEN 0x05060708

#define FREQUENCY_AGILITY

//Special frequency hopping based on agility + hopping
#define FREQUENCY_HOPPING_ASAC

//Massimo numero di hops consecutivi tra 2 hosts prima di tornare all'agility 
#define MAX_HOPS_CONSECUTIVE 20	

//Define per abilitare la gestione di mini-pacchetti ricevuti in contemporanea da + host
#define ENABLE_MULTIPEER_CACHE

#define HOPS_DELAY_BASE 10

/* Remove 'x' corruption to enable application autoacknowledge support. Requires extended API as well */
#define APP_AUTO_ACK

/* Remove 'x' corruption to enable Extended API */
#define EXTENDED_API

/* Remove 'x' corruption to enable security. */
#define xSMPL_SECURE

/* Remove 'x' corruption to enable NV object support. */
#define xNVOBJECT_SUPPORT

/* Remove 'x' corruption to enable software timer. */
#define xSW_TIMER

/* Remove 'x' corruption to enable frequency hopping. */
#define xFREQUENCY_HOPPING

#define BSP_TIMER_USED BSP_TIMER_A3

/* Remove 'x' corruption to make this device the reference clock. */
#define xNWK_PLL_REFERENCE_CLOCK

/* causes leds to blink in 00 -> 01 -> 11 -> 10 -> 00 rotation when FHSS enabled */
#define xNWK_PLL_SHOW_LOCATION_INDICATORS

//Network noise sensitivity on change channel
#define INTERFERNCE_THRESHOLD_DBM (-70)
