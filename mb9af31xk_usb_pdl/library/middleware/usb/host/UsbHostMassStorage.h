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
/******************************************************************************/
/** \file UsbHostMassStorage.h
 **
 ** A detailed description is available at 
 ** @link UsbHostMassStorageGroup USB Host Mass Storage Module description @endlink
 **
 ** History:
 **   - 2010-03-30    1.0  MSc  First version  (works with 16FX)
 **   - 2011-08-11    1.1  MSc  Several Bug fixes
 **   - 2011-08-24    1.2  MSc  Timeout Bug fixes
 **   - 2013-10-14    1.3  MSc  PDL support added
 **   - 2014-09-03    1.4  MSc  Deinit routine fixed
 **                             Switched to dynamic driver registering
 **   - 2015-04-20    1.5  MSCH replacel NULL -> 0 for ErrorHandler((uint16_t)0, &clrEponly);
 **   - 2015-05-04    1.6  MSCH deinitialization added after unsuccessful init
 **   - 2015-07-21    1.7  MSCH correct initialization for all variables added
 **   - 2015-09-04    1.8  MSCH Usb_WaitHook() added
 *****************************************************************************/

#ifndef __USBHOSTMASSSTORAGE_H__
#define __USBHOSTMASSSTORAGE_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "usb.h"

#ifndef USBHOSTMASSSTORAGE_ENABLED
    #define USBHOSTMASSSTORAGE_ENABLED OFF
#endif
     
#if ((FM_PERIPHERAL_USB_HOST_ENABLED == ON) && (USBHOSTMASSSTORAGE_ENABLED == ON))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
    
/**
 ******************************************************************************
 ** \defgroup UsbHostMassStorageGroup USB Host Middleware: Mass Storage
 **
 ** Provided functions of USB Host Mass Storage module:
 ** 
 ** Storage API:
 ** - UsbHostMassStorage_StorageApiRead()
 ** - UsbHostMassStorage_StorageApiWrite()
 ** - UsbHostMassStorage_StorageApiReadCapacity()
 ** - UsbHostMassStorage_StorageApiInit()
 ** - UsbHostMassStorage_StorageApiModeSense()
 **
 ** Other API:
 ** - UsbHostMassStorage_DriverActive()
 **
 ** Following procedures are used for the UsbHostClassSupervisor:
 ** - UsbHostMassStorage_DeinitHandler()
 ** - UsbHostMassStorage_InitHandler()
 **
 ** Used to connect USB hard drives to the MCU
 **
 ******************************************************************************/
//@{
    
#include "UsbHost.h"
#include "diskio.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define USBHOSTMASSSTORAGE_VERSION  0140
#define USBHOSTMASSSTORAGE_DATE     20140903
    
#define USBMASSSTORAGE_FLAG_TRANSFERDONE     (1<<0)
#define USBMASSSTORAGE_FLAG_TRANSFERDONE_IN  (1<<1)
#define USBMASSSTORAGE_FLAG_TRANSFERDONE_OUT (1<<2)

#define FORMAT_UNIT							0x04
#define INQUIRY_MASS_STORAGE				0x12
#define MODE_SELECT							0x55
#define MODE_SENSE							0x5A
#define PREVENT_ALLOW_MEDIUM_REMOVAL		0x1E
#define READ_MASS_STORAGE  				    0x28
#define READ_MASS_STORAGE_CAPACITY			0x25	
#define READ_FORMAT_CAPACITIES				0x23
#define MASS_STORAGE_REQUEST_SENSE			0x03
#define REZERO								0x01
#define SEEK10								0x2B
#define SEND_DIAGNOSTIC						0x1D
#define START_STOP_UNIT						0x1B
#define TEST_UNIT_READY						0x00
#define VERIFY								0x2F
#define WRITE_MASS_STORAGE   				0x2A
#define WRITE12								0xAA
#define WRITE_AND_VERIFY					0x2E

#define CBW_TAG                       0x1A2B3C4D 

#define LITTLEENDIAN 1
#define BIGENDIAN 2
#define ENDIANTYPE BIGENDIAN

#define LENDIAN_BUFFER_TO_U32(pu8Buffer)\
(uint32_t)( (uint32_t)*(uint8_t*)(pu8Buffer)                                   \
         |  ((((uint32_t)*(((uint8_t*)(pu8Buffer))+1)) <<  8  ) & 0xFF00UL    )\
         |  ((((uint32_t)*(((uint8_t*)(pu8Buffer))+2)) << 16  ) & 0xFF0000UL  )\
         |  ((((uint32_t)*(((uint8_t*)(pu8Buffer))+3)) << 24  ) & 0xFF000000UL)\
         )

#define U32_TO_LENDIAN_BUFFER(pu8Buffer,u32Data)\
    { *((pu8Buffer))  =(uint8_t)((uint32_t)(u32Data)      );\
      *((pu8Buffer)+1)=(uint8_t)((uint32_t)(u32Data) >>  8);\
      *((pu8Buffer)+2)=(uint8_t)((uint32_t)(u32Data) >> 16);\
      *((pu8Buffer)+3)=(uint8_t)((uint32_t)(u32Data) >> 24);\
    }

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
typedef struct cbw
{
   uint32_t dCBWSignature;
   uint32_t dCBWTag;
   uint32_t dCBWDataTransferLength;
   uint8_t  bCBWFlags;
   uint8_t  bCBWLUN;
   uint8_t  bCBWCBLength;
   uint8_t  CBWCB[16];
} cbw_t;

typedef struct csw
{
   uint32_t CSWSignatur;
   uint32_t CSWTag;
   uint32_t CSWDataResidue;
   uint8_t CSWStatus;
} csw_t;

// Response struct
typedef struct InquiryResponse
{
   uint8_t Peripheral;
   uint8_t Removable;
   uint8_t Version;
   uint8_t ResponseDataFormat;
   uint8_t AdditionalLength;
   uint8_t Sccstp;
   uint8_t Bqueetc;
   uint8_t CmdQue;
   uint8_t VendorID[8];
   uint8_t ProductID[16];
   uint8_t ProductRev[4];
} InquiryResponse_t;


// 8 byte capacity value buffer
typedef struct read_capacity
{
   uint32_t SectorCount;
   uint32_t SectorSize;
} read_capacity_t;


// Functions
typedef struct RequestSenseResponse
{
    uint16_t ResponseCode :7;
    uint16_t VALID :1;
    uint8_t Obsolete;
    uint16_t SenseKey :4;
    uint16_t Resv :1;
    uint16_t ILI :1;
    uint16_t EOM :1;
    uint16_t FILEMARK :1;
    uint32_t Information;
    uint8_t AddSenseLen;
    uint32_t CmdSpecificInfo;
    uint8_t ASC;
    uint8_t ASCQ;
    uint8_t FRUC;
    uint8_t SenseKeySpecific[3];
} RequestSenseResponse_t;

typedef struct StorageInfo
{
    uint8_t STORAGE_CMD;
    uint16_t  BUFFER_SIZE;
    uint8_t*  BUFFER;
    uint16_t  IN_EP;
    uint16_t  OUT_EP;
    uint32_t BLOCK_START;
    uint8_t BLOCK_CNT;
    uint16_t  STATUS;
    uint8_t SEND_CBW;
    uint8_t GET_CSW;
    uint8_t CSW_STATUS;
    uint16_t  BYTES_TRANSFERRED;
    uint16_t  ERROR_CODE;
    uint8_t MAX_LUN;
} StorageInfo_t;


typedef struct inquiry_response
{
    uint8_t Peripheral;
    uint8_t Removable;
    uint8_t Version;
    uint8_t ResponseDataFormat;
    uint8_t AdditionalLength;
    uint8_t Sccstp;
    uint8_t Bqueetc;
    uint8_t CmdQue;
    uint8_t VendorID[8];
    uint8_t ProductID[16];
    uint8_t ProductRev[4];
} inquiry_response_t;

typedef struct ModeSenseResponse
{  
    uint8_t ModeDataLength;   
    uint8_t MediumType;     
    uint8_t Reserved1     :4;
    uint8_t DPOFUA        :1;
    uint8_t Reserved2     :2;
    uint8_t WriteProtect  :1; 
    uint8_t BlockDescriptorLength;
} ModeSenseResponse_t;

typedef struct StorageReadWrite
{
    uint8_t data[512];
} StorageReadWrite_t;


// Struct for write data
typedef struct StorageWrite
{
    uint8_t data[10];
} StorageWrite_t;


/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/



/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

/* Storage API */
int UsbHostMassStorage_StorageApiRead(BYTE *buff, DWORD sector, BYTE count);
int UsbHostMassStorage_StorageApiWrite(BYTE *buff, DWORD sector, BYTE count);
int UsbHostMassStorage_StorageApiReadCapacity(void (*Buff));
int UsbHostMassStorage_StorageApiInit(void);
int UsbHostMassStorage_StorageApiModeSense(void);
/* END Storage API */

void UsbHostMassStorage_RegisterDriver(void);
void UsbHostMassStorage_Init(void);
boolean_t UsbHostMassStorage_DeinitHandler(stc_usbn_t* pstcUsb);
boolean_t UsbHostMassStorage_InitHandler(stc_usbn_t* pstcUsb,uint8_t* pu8Configuration, uint32_t u32Length);
void UsbHostMassStorage_BulkInCompletion(stc_usbn_t* pstcUsbHandle);
void UsbHostMassStorage_BulkOutCompletion(stc_usbn_t* pstcUsbHandle);
void UsbHostMassStorage_InTransferDone(void);
void UsbHostMassStorage_OutTransferDone(void);
void UsbHostMassStorage_CheckForCompletion(volatile uint8_t *NAKDetected);
StorageInfo_t* UsbHostMassStorage_GetGlobalStorageInfo(void);
int16_t UsbHostMassStorage_StorageCommandSend(StorageInfo_t *pstcStorageInfo);
int16_t UsbHostMassStorage_SendCBW(StorageInfo_t *pstcStorageInfo);
int16_t UsbHostMassStorage_Transmit_MSDC_Data(uint8_t *pu8Buffer, StorageInfo_t *pstcStorageInfo);
int16_t UsbHostMassStorage_Transmit_Data(StorageInfo_t *pstcStorageInfo);
int16_t UsbHostMassStorage_GetCSW(StorageInfo_t *pstcStorageInfo);
int16_t UsbHostMassStorage_CheckCSW(StorageInfo_t *pstcStorageInfo);
int16_t UsbHostMassStorage_BulkOnlyMSReset(void);
int16_t UsbHostMassStorage_StorageStatus(void);
int16_t UsbHostMassStorage_ResetRecovery(void);
int16_t UsbHostMassStorage_ClearInEndpoint(void);
int16_t UsbHostMassStorage_ClearOutEndpoint(void);
void UsbHostMassStorage_GetEPStatus(uint8_t* InEpStatus, uint8_t* OutEpStatus);
int UsbHostMassStorage_TestUnitReady(StorageInfo_t *pstcStorageInfo);

void MassStorageWaitForSetupComplete(void);

boolean_t UsbHostMassStorage_DriverActive(void);

#ifdef __cplusplus
}
#endif

//@} // UsbHostMassStorageGroup

#endif /* (USE_USB_HOST == 1) */
#endif /* __USBMASSSTORAGE_H__*/
