#include <rtl.h>
#include <91x_lib.h>
#include <string.h>

#include "nvmem.h"
#include "EEprom.h"
#include "highcoms.h"
#include "radio_drv.h"
#include "..\..\ASAC_LIB\Consolle\Consolle.h"
							
extern addr_t build_address(uint32_t matr_node);
extern void checkChangeChannel(void);
extern tipoEEprom    	EEprom;
extern tipoRadioBase 	RadioBase;
extern void DISPSTATUS_update_RF(void);
//------------------------------------------------------
__task void  STI_RF_Task(void){
#define PersistenzaTaccheRF 100				// persisteza visualizzazione segnale RF [100ms]
u16 presc;
u16 stato;
	while(1){		
		checkChangeChannel();   
		 	
		if (RadioBase.RssiLev>190)
            stato=5;                         /* Icona 1: 0=Non visibile; 1=Spenta; 2=Search; 3..5=Tacche segnale RF   */
        else if (RadioBase.RssiLev>175)
            stato=4;                         /* Icona 1: 0=Non visibile; 1=Spenta; 2=Search; 3..5=Tacche segnale RF   */
        else if (RadioBase.RssiLev>160)
            stato=3;                         /* Icona 1: 0=Non visibile; 1=Spenta; 2=Search; 3..5=Tacche segnale RF   */
		else
            stato=2;                         /* Icona 1: 0=Non visibile; 1=Spenta; 2=Search; 3..5=Tacche segnale RF   */
        
		if (stato!=2){
			presc=PersistenzaTaccheRF;
			DISPSTS.RfSignal=stato;
			DISPSTATUS_update_RF();        
		}else{
			if (--presc<1){
				presc=PersistenzaTaccheRF;		
				DISPSTS.RfSignal=stato;
				DISPSTATUS_update_RF();        
			}
		}

		RadioBase.RssiLev=0;
		
		os_dly_wait(10);	//100 msec base..	//os_tsk_pass();      
	}
}
//------------------------------------------------------
void STI_RF_Open(u16 VirtualCom){
    RFPK.TermNr     =Setup.NumTerm;     /* Numero Terminale (Solo Mode: 1,3,4,5,6)      */
    RFPK.Protect    =Setup.RF.Protect;  /* Codice di protezione                         */
	RFPK.Ripetizioni=EEprom.Ripetizioni; 	/* Numero di ripetizioni: 0=Illimitato     (Solo Mode: 5,6,7) */

	memset(&RadioFh,0,sizeof(RadioFh));

    STI_open();
}
//------------------------------------------------------
void STI_RF_Close(void){
   	STI_close();
}
//------------------------------------------------------
void STI_RF_GestCommand(char* cmd, char* risp, u8 cmdlen){
//
}
