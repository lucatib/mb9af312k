#include <rtl.h>
#include <91x_lib.h> 

#include "radio_drv.h"
#include "nvmem.h"
#include "highcoms.h"
#include "protoc.h"
#include "EEprom.h"

extern RF_DRV 	 			rf_drv;
extern tipoEEprom  			EEprom;
extern tipoRFPK   			RFPK;
extern tipoMsgComm  		MsgComm;
extern tipoSetup			Setup;

extern void checkChangeChannel(void);

//------------------------------------------------------
__task void STI_RF_Task(void){
	while(1){	
		checkChangeChannel();
		os_dly_wait(10);  
	}
}
//------------------------------------------------------
void STI_RF_Open(u16 VirtualCom){  
    //RFPK.NumRdy     =MODE_NMES_BIT +1;  /* Contatore di numero di Pacchetto READY  (ad un valore non ammesso) */
    //RFPK.CntReSend  =0;                 /* Azzera numero di messaggi ritrasmessi x accettare il cambio antenna */
    //RFPK.NackRq     =FALSE;             /* True=Terminale richiede NAK; False=No            */
    //RFPK.NumMes     =MODE_NMES_BIT +1;  /* Contatore di numero di Pacchetto MESC   (ad un valore non ammesso) */
    RFPK.RetrieTx   =0;                 /* Contatore di ripetizioni Trasm.      */

    RFPK.MsgFlag    =FALSE;             /* True=Messaggio arrivato da HOST in attesa di essere Conferato dal Tiddy; False=NO    */
    RFPK.MaskeraTimerTask=FALSE;        /* True=Gli eventi di RF_COMMAND_TIMER sono inibiti; FALSE= la segnalazione EVENTO RF_COMMAND_TIMER è attiva */    

    /* -------< Coefficienti Modulo ("AT+ASACMOD=") >------     */
    RFPK.Mode       =EEprom.Mode;       /* 0=Idle; 1..3=Diag Rf fissa; 4..6=Diag Pacchetti; 10=Modo Periferica; 11=Modo Nor     */ 
    RFPK.bkMode     =!RFPK.Mode;        /* Backup di Mode (x rilevare le variazioni)    */

    RFPK.Protect    =EEprom.Protect;    /* Codice di protezione                         */
    RFPK.TermNr     =EEprom.TermNr;     /* Numero di terminale                          */

    RFPK.NorNr      =EEprom.NorNr;      /* Numero di Nor      (solo Mode: 11)           */
    RFPK.Valore     =EEprom.Valore;     /* Valore 0=ZERO,1=UNO, 2=Pattern PseudoRandom (Solo Mode: 3) */
    RFPK.Ripetizioni=EEprom.Ripetizioni; /* Numero di ripetizioni: 0=Illimitato     (Solo Mode: 5,6,7) */

    /* -------< Coefficienti Channel ("AT+ASACCHANNEL=") >------     */
    RFPK.FhMode     =EEprom.FhMode;     /* Modo funzionamento modulo                    */    
    RFPK.ChannelRF  =EEprom.ChannelRF;  /* Canale Radio-Solo se senza Frequency Hopping */

	STI_open();	//Call common part
}
//------------------------------------------------------
void STI_RF_Close(void){
	STI_close();
}
//------------------------------------------------------
void STI_RF_GestCommand(char* cmd, char* risp, u8 cmdlen){
 	u16 i;
	u16 uiAdd;
	u16 uiData;
	char* p;

    // Pulisce preventivamente la risposta
    risp[0]=0;

	if (!cmdlen) return; 
	
	/*--------------------------------------------------------------------------*/
//AT    Comando invio dati verso un Host
    /*--------------------------------------------------------------------------*/
	if (cmdlen>9){ 
	    if ((strncmp(cmd,"AT@",3)==0) || 
			(strncmp(cmd,"at@",3)==0)){
		    for (i=3; i<cmdlen && (cmd[i]!=','); i++);
			if (i<8) return;
	        
			RFPK.TermNr=atol(cmd+3);        // ricava la matricola del device
	        RF_send_message(RFPK.TermNr, &cmd[i + 1], cmdlen - (i + 1));
	        return;
	    }         
	}

	/*--< Passa in maiuscolo >-- */
    ucase(cmd);
		    
    //--------------------------------------------------------------
    //  -------< Valutazione comandi 'RUNTIME' del NOR>--------- 
    //--------------------------------------------------------------
    if (cmd[0] == 0x1B){ 
				    
        /* -------------------------------------------------------- */
        /*                          COMANDI ESCAPE                  */
        /* -------------------------------------------------------- */
		if (cmdlen<2) return;

        switch(cmd[1]){
        
                /****************************************************************************
                **
                **  Comandi EEprom:
                **        Per la lettura/scrittura e cancellazione della EEPROM
                **
                **  Read        : R<add>
                **  Write       : W<add>,<data>
                **  Erase       : E<add>
                **  EraseAll    : A
                **  EraseEnable : +
                **  EraseDisable: -
                *****************************************************************************/
                case 5:                  /* ^E: Comandi EEPROM */

					if (cmdlen<3) return;

                    switch(cmd[2]){
                        case 'R':                    /* -----<COMANDO READ>--------    */
                            p=cmd+3;
                            uiAdd=atoi((const char *)p);
                            uiData=EEPROM_read(uiAdd);
                            sprintf(risp,"%cRD_EE(%2d)=>%4X",MonitorMsg, uiAdd, uiData);
                            return;
            
                        case 'W':                    /* -----<COMANDO WRITE>-------    */
                            p=cmd+3;
                            uiAdd=atoi((const char *)p);
                            while(*p++ !=',');
                            uiData=atoi(p);
                            EEPROM_ewen();
                            EEPROM_write_w(uiAdd,uiData);
                            EEPROM_ewds();
                              
                            sprintf(risp,"%cWR_EE(%2d)<=%4X",MonitorMsg,uiAdd, uiData);
                            return;
            
                        case 'E':                    /* -----<COMANDO Erase>--------    */
                            p=cmd+3;
                            uiAdd=atoi(p);
                            EEPROM_ewen();
                            EEPROM_erase_w(uiAdd );
                            EEPROM_ewds();
            
                            sprintf(risp,"%cErase(%2d)",MonitorMsg,uiAdd);
                            return;
            
                        case 'A':                    /* -----<COMANDO Erase All>----    */
                            EEPROM_ewen();
                            EEPROM_eral_w();
                            EEPROM_ewds();
            
                            sprintf(risp,"%cAll Erased ! ",MonitorMsg);
                            return;
            
                        case '+':                    /* -----<COMANDO Erase Enable>----    */
                            EEPROM_ewen();
                            sprintf(risp,"%cErase Enable",MonitorMsg);
                            return;
            
                        case '-':                    /* -----<COMANDO Erase Disable>----    */
                            EEPROM_ewds();
                            sprintf(risp,"%cErase Disable",MonitorMsg);
                            return;
            
                        default:
                            sprintf(risp,"%cEEPROM Command:Error",MonitorMsg);
                            
                    } /* endswitch '^E' */
                    return;
    
                /****************************************************************************
                **
                **  Comandi Lifetime (Timer Vita):
                **        Per accedere ai parametri di sistema dalla EEPROM
                **
                **  Matricola   : M
                **  Scritt. Matr: S<matricola>
                **  Timer life  : L
                ****************************************************************************/
                case 'V':                /* 'V' Comandi Lifetime (Timer Vita)*/

					if (cmdlen<3) return;

                    /* ---< Controllo comando >--- */
                    switch(cmd[2]){
            
                        case 'M':                    /* -----<COMANDO Lettura MATRICOLA>----    */
                            EEprom.MATRICOLA  = EEPROM_read(MATRT_LOC_H);  /* Leggo il parametro   */
                            EEprom.MATRICOLA  <<=16;
                            EEprom.MATRICOLA  += EEPROM_read(MATRT_LOC_L);  /* Leggo il parametro   */
            
                            sprintf(risp,"%c£%08lu",MonitorMsg,EEprom.MATRICOLA);
                            return;
            
                        case 'S':                    /* -----<COMANDO Scrittura MATRICOLA>----    */
                            p=cmd+3;
            
                            EEprom.MATRICOLA =atol(p);
                            EEPROM_ewen();                       /* Abilita scrittura eeprom             */
                            uiData=EEprom.MATRICOLA & 0xFFFF;
                            EEPROM_erase_w(MATRT_LOC_L);    EEPROM_write_w(MATRT_LOC_L,uiData); /* LSB  */
                            uiData=EEprom.MATRICOLA >> 16;
                            EEPROM_erase_w(MATRT_LOC_H);    EEPROM_write_w(MATRT_LOC_H,uiData);     /* MSB  */
                            EEPROM_ewds();  
                            return;
            
                        case 'L':                    /* -----<COMANDO Lettura Timer VITA>----    */
                            sprintf(risp,"%cà%08lX-%08lX-%08lX",MonitorMsg
                                                               ,(EEPROM_read(WAKE_LOC_H)<<16)+EEPROM_read(WAKE_LOC_L)
                                                               ,(EEPROM_read(OFF_LOC_H)<<16) +EEPROM_read(OFF_LOC_L)
                                                               ,(EEPROM_read(TIME_LOC_H)<<16)+EEPROM_read(TIME_LOC_L));
                             return;
            
                        default:
                            sprintf(risp,"%cLifeTime Command:Error",MonitorMsg);
                    } /* endswitch 'V' */
                    return;

                /****************************************************************************
                **
                **  Comandi Nor
                **        Per programmare i principali parametri NOR 
                **
                **  Lettura: NG   
                **              NGX = Numero di terminali
                **              NGN = Numero di Nor
                **              NGP = Codice di protezione
                **  Scritt.: NS
                **              "NS"                          'Comando Nor Set
                **              <MaxTerm>,  Massimo numero di terminali installati
                **              <ModoCode>, Modo codifica 1=HAMMING
                **              <Antenna>,  1=Destra;
                **              <NorNr>,    Numero di Nor: 0=Seriale
                **              <Protect>,  Codice di protezione impianto
                **              <SizePreambolo>, Numero caratteri preambolo
                **              <FH>,       Modo Frequency Hopping: 1=Attivo su 20 ch al secondo
                **              <NorCom> ,  Nor a cui è connesso: 0=CELLA 0
                **              <Mode>,     0=Rs232; 1=Lan; 2=Wireless
                ****************************************************************************/
                case 'N':                /* 'N': Comandi NOR */
                    /* ---< Controllo comando >--- */

					if (cmdlen<3) return;

                    switch(cmd[2]){
                        
                        // COMANDO --- GET ------
                        case 'G':
                            /* ---< Controllo parametro >--- */
                            switch(cmd[3]){
                                case 'X':  // Numero di terminali
                                    sprintf(risp,"%c%u",MonitorMsg,EEprom.TermNr);
                                    break;
                                case 'N':   // Numero di NOR
                                    sprintf(risp,"%c%u",MonitorMsg,EEprom.NorNr);
                                    break;
                                case 'P':   // Codice di protezione ditta
                                    sprintf(risp,"%c%u",MonitorMsg,EEprom.Protect);
                                    break;
                            }
                            break;
                        
                        // COMANDO --- SET ------
                        case 'S':
                            //NS16,1,1,0,2,25,1,0,0,
                            //  Cmd$ = "NS"                          'Comando Nor Set
                            i=3;
                            //  Cmd$ = Cmd$ & txtMaxTerm.Text & ","   '<MaxTerm>  Massimo numero di terminali installati
//                          EEprom.TermNr = atoi((const char *)(cmd+i));         /* <MaxTerm>  Massimo numero di terminali installati   */
                            Skip_Sep_param(cmd,&i);

                            // Cmd$ = Cmd$ & "1,"                    '<ModoCode> Modo codifica 1=HAMMING
                            Skip_Sep_param(cmd,&i);

                            //  Cmd$ = Cmd$ & "1,"                    '<Antenna>  1=Destra;
                            Skip_Sep_param(cmd,&i);

                            //  Cmd$ = Cmd$ & txtSetNorNr.Text & ","  '<NorNr>    Numero di Nor: 0=Seriale
//                          EEprom.NorNr = atoi((const char *)(cmd+i));         /* <NorNr>    Numero di Nor: 0=Seriale                  */
                            Skip_Sep_param(cmd,&i);

                            //  Cmd$ = Cmd$ & txtSetProtect.Text & "," '<Protect>  Codice di protezione impianto
                            EEprom.Protect = atoi((const char *)(cmd+i));       /* '<Protect>  Codice di protezione impianto            */
                            Skip_Sep_param(cmd,&i);

                            //  Cmd$ = Cmd$ & "25,"                   '<SizePreambolo> Numero caratteri preambolo
                            Skip_Sep_param(cmd,&i);

                            //  Cmd$ = Cmd$ & "1,"                    '<FH>       Modo Frequency Hopping: 1=Attivo su 20 ch al secondo
                            Skip_Sep_param(cmd,&i);

                            //  Cmd$ = Cmd$ & txtSetNorComm.Text & "," '<NorCom>  Nor a cui è connesso: 0=CELLA 0
                            Skip_Sep_param(cmd,&i);

                            //  Cmd$ = Cmd$ & Mode & ","              '<Mode>     0=Rs232; 1=Lan; 2=Wireless
                            Skip_Sep_param(cmd,&i);

                            RADIO_off();                   // Spegne per non interferire con la EEPROM

                            /* -------< Salvataggio in EEPROM >----- */
                            tsk_lock();                       /* ----< BLOCCA INTERRUPT DI TASK >--- */
                             //EEPROM_save_cache();
                            EEPROM_ewen();                       /* Abilita scrittura eeprom             */
							EEPROM_erase_w(EE_Protect_LOC);    EEPROM_write_w(EE_Protect_LOC,EEprom.Protect);
                            EEPROM_ewds();  
                            tsk_unlock();                     /* ----< SBLOCCA INTERRUPT DI TASK >--- */
                            sprintf(risp,"+ASACSAVE: Saved!");
                            RFPK.bkMode=RF_MODE_CMD_IDLE;      // Forza la reimpostazione della modalità
                            Nor.ResetReq=TRUE;                   /* True=Richiesta di Reset; False=Esaudita      */
                            break;
                    }
                    return;
    

    
               case 'R':                /* 'R' Richiesta di RESET   */
                    Nor.ResetReq=TRUE;                   /* True=Richiesta di Reset; False=Esaudita      */
                    return;

              
        } /* endswich 'ESC' */
        
        /* Se il comando ESC non è gestito: ESCE DAL MONITOR */
        return;
    } /* end if */

    


    /*--------------------------------------------------------------------------*/
//MODE              /* --< Controlla i comandi di gestione Modalità >--*/
    /*--------------------------------------------------------------------------*/
	if (cmdlen>11){
	    if (strncmp((const char *)cmd,"AT+ASACMOD=",11)==0){
	            i=11;
	            /* --------------------< MODE >---------------------------  */
	            /* Impara la nuova modalità di funzionamento:  0=Invalid; 1=dle; 2=Diagnostico-Tx fissa.; 3=Diagnostico=Tx pacchetti; 4=Diagnostico=Tx->Rx; 5=Diagnostico=Rx->Tx */
	            RFPK.Mode=atoi((const char *)(cmd+i));             
	
	            switch(RFPK.Mode){
	                /* --------------------< IDLE >---------------------------  */
	                case RF_MODE_CMD_IDLE:
	                    sprintf(risp,"+ASACMOD:IDLE MODE");
	                    break;
	
	                /* ----------------< RX FISSO >--------------------------- */
	                case RF_MODE_CMD_RXFIX:
	                    sprintf(risp,"+ASACMOD:RX");
	                    break;
	
	                /* ----------------< TX FISSO >--------------------------- */
	                case RF_MODE_CMD_TXFIX:
	                    Skip_Sep_param(cmd,&i);
	                    RFPK.Valore= atoi((const char *)(cmd+i));             /* Valore 0=ZERO,1=UNO, 2=Pattern PseudoRandom  */
	
	                    sprintf(risp,"+ASACMOD:TX=%u",RFPK.Valore);
	                    break;
	
	
	                /* ----------------< TX PACCHETTI >----------------------- */
	                case RF_MODE_CMD_TXPACK:
	                case RF_MODE_CMD_TXPACKRX:
	                case RF_MODE_CMD_RXPACKTX:
	
	                    Skip_Sep_param(cmd,&i);
	                    RadioPack.Protect= atoi((const char *)(cmd+i));         /* Codice cript                                 */
	                    Skip_Sep_param(cmd,&i);
	                    RadioPack.TermNr= atoi((const char *)(cmd+i));          /* Numero Terminale (Solo Mode: 1,3,4,5,6)      */
	                    Skip_Sep_param(cmd,&i);
	                    RFPK.Ripetizioni= atoi((const char *)(cmd+i));          /* Numero di ripetizioni: 0=Illimitato     (Solo Mode: 4,5,6) */
	                    Skip_Sep_param(cmd,&i);
	                    myStrCpy((unsigned char *)RFPK.Testo,(unsigned char *)(cmd+i));                         /* Testo del messaggio diagnostico (Solo Mode: 4,5,6) */
	                    
	                    sprintf(risp,"+ASACMOD:TX PACK-r.f.u.-Code=%u, N.T.=%u, R=%u, [%s]",RadioPack.Protect
	                                                                                       ,RadioPack.TermNr
	                                                                                       ,RFPK.Ripetizioni
	                                                                                       ,RFPK.Testo);
	                    break;
	
	                /* --------------------< MODULE >-------------------------  */
	                case RF_MODE_CMD_MODULE:
	                    Skip_Sep_param(cmd,&i);
	                    RFPK.Protect= atoi((const char *)(cmd+i));         /* Codice cript                                 */
	                    Skip_Sep_param(cmd,&i);
	                    RFPK.TermNr= atoi((const char *)(cmd+i));          /* Numero Terminale (Solo Mode: 1,3,4,5,6)      */
	                    Skip_Sep_param(cmd,&i);
	                    Setup.RF.u16_protocolType = atoi((const char *)(cmd+i));     /* 0=Nessuna (Nor=passante; Modulo=seriale pura);   */
	                    // tipo di protocollo ....da fare!!
	                    sprintf(risp,"+ASACMOD:MODULE - Code=%u, Nt=%u, Prot=%u",RFPK.Protect
	                                                                          ,RFPK.TermNr
	                                                                          ,Setup.RF.u16_protocolType);
	                    break;
	
	                /* --------------------< TYPHOON MODULE >-------------------------  */
	                case RF_MODE_CMD_TYPHMOD:
	                    Skip_Sep_param(cmd,&i);
	                    RFPK.Protect= atoi((const char *)(cmd+i));         /* Codice cript                                 */
	                    Skip_Sep_param(cmd,&i);
	                    RFPK.TermNr= atoi((const char *)(cmd+i));          /* Numero Terminale (Solo Mode: 1,3,4,5,6)      */
	                    Skip_Sep_param(cmd,&i);
	                    Setup.RF.u16_protocolType= atoi((const char *)(cmd+i));     /* 0=Nessuna (Nor=passante; Modulo=seriale pura);   */
	                    // tipo di protocollo ....da fare!!
	                    sprintf(risp,"+ASACMOD:TYPHMOD - Code=%u, Nt=%u, Prot=%u",RFPK.Protect
	                                                                          ,RFPK.TermNr
	                                                                          ,Setup.RF.u16_protocolType);
	                    break;
	
	                /* --------------------< NOR >----------------------------  */
	                case RF_MODE_CMD_NOR:
	                    Skip_Sep_param(cmd,&i);
	                    RFPK.Protect= atoi((const char *)(cmd+i));         /* Codice cript                                 */
	                    Skip_Sep_param(cmd,&i);
	                    RFPK.NorNr = atoi((const char *)(cmd+i));          /* Numero NOR                                   */
	                    Skip_Sep_param(cmd,&i);
	                    Setup.RF.u16_protocolType= atoi((const char *)(cmd+i));     /* 0=Nessuna (Nor=passante; Modulo=seriale pura);   */
	                    sprintf(risp,"+ASACMOD:NOR - Code=%u, Nt=%u, Prot=%u" ,RFPK.Protect
	                                                                        ,RFPK.NorNr
	                                                                        ,Setup.RF.u16_protocolType);
	                    break;
	
	                /* --------------------< INVALID >------------------------  */
	                default:
	                    sprintf(risp,"+ASACMODE:: INVALID MODE");
	                    break;
	            }/* end switch */
	            return;
	    }
	}
    /*--------------------------------------------------------------------------*/
//RADIO             /* --< Controlla i comandi di correzione radio  >--*/
    /*--------------------------------------------------------------------------*/
	if (cmdlen>13){
	    if (strncmp(cmd,"AT+ASACRADIO=",13)==0){
	            i=13;                  
	            // Reinizializza tutta la strutta RadioBase con i valori di EEPROM
	            RADIO_off();  
	            // Acquisisce i parametri dal comando 
	            RadioBase.OffsetCalPll=atoi((const char *)(cmd+i)); Skip_Sep_param(cmd,&i);/* Correzione della frequenza                   */
	            RadioBase.PowerPa  = atoi((const char *)(cmd+i)); Skip_Sep_param(cmd,&i);   /* Livello di potenza in trasmissione           */
	            RadioBase.PowerRaise=atoi((const char *)(cmd+i)); Skip_Sep_param(cmd,&i);   /* Fattore correzione Rampa TX                  */
	            RadioBase.SzPreamb = atoi((const char *)(cmd+i)); Skip_Sep_param(cmd,&i);   /* Numero di caratteri preambolo                */
	            RadioBase.RssiBase = atoi((const char *)(cmd+i)); Skip_Sep_param(cmd,&i);   /* Soglia RSSI                                  */
	            RadioBase.Deviaz   = atoi((const char *)(cmd+i)); Skip_Sep_param(cmd,&i);   /* Soglia DEVIAZIONE*/
	            RadioBase.BandWidth= atoi((const char *)(cmd+i)); Skip_Sep_param(cmd,&i);   /* Codice di filtro banda */
	            sprintf(risp,"+ASACRADIO:Coef=%i, Pwr=%u, Ramp=%u, SizePr=%u, Squelch=%u Dev=%u, BW=%u"
	                                                                ,RadioBase.OffsetCalPll
	                                                                ,RadioBase.PowerPa  
	                                                                ,RadioBase.PowerRaise
	                                                                ,RadioBase.SzPreamb
	                                                                ,RadioBase.RssiBase 
	                                                                ,RadioBase.Deviaz
	                                                                ,RadioBase.BandWidth
	                                                                 );
	            RFPK.bkMode=RF_MODE_CMD_IDLE;   // Forza la reimpostazione della modalità
	                                            // In questo modo avverà la riprogrammazione della Radio
	            return;
	    }
	}

    /*--------------------------------------------------------------------------*/
//CHANNEL     /* --< Controlla i comandi di impostazione Canale e modulazione >--*/
    /*--------------------------------------------------------------------------*/
	if (cmdlen>15){
	    if (strncmp(cmd,"AT+ASACCHANNEL=",15)==0){
	            i=15;                  
	            RFPK.ChannelRF=atoi((const char *)(cmd+i));  /*  0..83: Canale fisso;100=Freq. hopping 20 canali; 101=Freq. hopping 70 canali (diagnostica);102=Freq. hopping 2 canali estremi (0,83 x diagnostica)*/
	            Skip_Sep_param(cmd,&i);
	            RFPK.FhMode=atoi((const char *)(cmd+i));     /* 0= Hamming+MF a 333Kbps (compatibile NOR24);=r.f.u*/
	            
	            // Gestione modulazione :
	            if (RFPK.FhMode==FH_MODE_20CH){         /* Modalità FREQUENZA 20 CH */
	                RFPK.ChannelRF=40;
	            }
	            if (RFPK.FhMode==FH_MODE_70CH){         /* Modalità FREQUENZA 80 CH */
	                RFPK.ChannelRF=40;
	            }
	            if (RFPK.FhMode==FH_MODE_0_83){         /* Modalità FREQUENZA 0.83  */
	                RFPK.ChannelRF=40;
	            }
	            // Gestione canalizzazione speciale:
	            if (RFPK.ChannelRF==100){ // Se il canale = 100 :
	                RFPK.FhMode=FH_MODE_20CH;       /* Modalità FREQUENZA 20 CH */
	                RFPK.ChannelRF=40;
	            }
	
	            if (RFPK.ChannelRF==101){ // Se il canale = 101 :
	                RFPK.FhMode=FH_MODE_70CH;       /* Modalità FREQUENZA 70 CH */
	                RFPK.ChannelRF=40;
	            }
	
	            if (RFPK.ChannelRF==102){ // Se il canale = 100 :
	                RFPK.FhMode=FH_MODE_0_83;       /* Modalità FREQUENZA 0.83  */
	                RFPK.ChannelRF=40;
	            }
	            sprintf(risp,"+ASACCHANNEL:Ch=%u, Mod=%u",RFPK.ChannelRF,RFPK.FhMode );
	#if 0
	            ASAC_RADIO_pll_set_channel(RFPK.ChannelRF);      /* Cambia la sintonia     */         
	#endif 
	            RFPK.bkMode=RF_MODE_CMD_IDLE;      // Forza la reimpostazione della modalità
	            return;
	        }         
	}
    /*--------------------------------------------------------------------------*/
//CREADCC2500PARAM       /* --< Legge i registri del CC2500  >--*/
    /*--------------------------------------------------------------------------*/
#if 0
	if (cmdlen>23){
	    if (strncmp(cmd,"AT+ASACREADCC2500PARAM=",23)==0){
            i=23;                  
            uiData=atoi((const char *)(cmd+i));             /* Modalità di funzionamento:  0=Invalid; 1=dle; 2=Diagnostico-Tx fissa.; 3=Diagnostico=Tx pacchetti; 4=Diagnostico=Tx->Rx; 5=Diagnostico=Rx->Tx */
            // Impartisce il comando di lettura accedendo all'indirizzo + 0x80                
            i=send_cmd_ChipCon(uiData+0x80 ,0x0000,NULL);		           
            sprintf(risp,"RAD%cCC2500%c%u%c%u%cArray[]=%u  Status=%X",1,1,uiData,1,i & 0x00FF,1,ParTabCC[uiData],i>>8);
 
            return;
	    }
	}
#endif

   /*--------------------------------------------------------------------------*/
//CC2500 WRITE PARAM    /* --< Fornisce tutti parametri del CC2500 >--*/
    /*--------------------------------------------------------------------------*/
    /* Legge tutti i  parametri operativi */
#if 0
	if (cmdlen>24){
	    if (strncmp(cmd,"AT+ASACWRITECC2500PARAM=",24)==0){
	 
	            i=24;                  
	            uiData=atoi((const char *)(cmd+i));             /* Modalità di funzionamento:  0=Invalid; 1=dle; 2=Diagnostico-Tx fissa.; 3=Diagnostico=Tx pacchetti; 4=Diagnostico=Tx->Rx; 5=Diagnostico=Rx->Tx */
	            Skip_Sep_param(cmd,&i);
	            ParTabCC[uiData]=atoi((const char *)(cmd+i));   /* Numero di caratteri preambolo                */
	    
	            sprintf(risp,"RAD%cCC2500%c%u%c%u",1,1,uiData,1,ParTabCC[uiData]);
	    
	            send_cmd_ChipCon(uiData ,ParTabCC[uiData],NULL);		
	            return;
	    }
	}
#endif   
    /*--------------------------------------------------------------------------*/
//ATSAVE     /* --< Comando di Salvataggio parametri nella EEPROM
    /*--------------------------------------------------------------------------*/
	if (cmdlen>=11){
	    if (strncmp(cmd,"AT+ASACSAVE",11)==0){
	            
	            /* -------< Coefficienti Radio ("AT+ASACRADIO=") >-----     */
	            EEprom.OffsetCalPll=RadioBase.OffsetCalPll;  /* Correzione della frequenza                   */
	            EEprom.PowerPa   = RadioBase.PowerPa;        /* Livello di potenza in trasmissione           */
	            EEprom.PowerRaise= RadioBase.PowerRaise;     /* Fattore correzione Rampa TX                  */
	            EEprom.SzPreamb  = RadioBase.SzPreamb;       /* Numero di caratteri preambolo                */
	            EEprom.RssiBase  = RadioBase.RssiBase;       /* Soglia RSSI                                  */
	            EEprom.Deviaz    = RadioBase.Deviaz;         /* Soglia DEVIAZIONE*/
	            EEprom.BandWidth = RadioBase.BandWidth;      /* Codice di filtro banda */
	        
	            /* -------< Coefficienti Modulo ("AT+ASACMOD=") >------     */
	            EEprom.Mode   = RFPK.Mode;      /* 0=Idle; 1..3=Diag Rf fissa; 4..6=Diag Pacchetti; 10=Modo Periferica; 11=Modo Nor     */ 
	            EEprom.Protect= RFPK.Protect;   /* Codice di protezione                         */
	            EEprom.TermNr = RFPK.TermNr;    /* Numero di terminale                          */
	            EEprom.NorNr  = RFPK.NorNr;     /* Numero di Nor      (solo Mode: 11)           */
	            EEprom.Valore = RFPK.Valore;    /* Valore 0=ZERO,1=UNO, 2=Pattern PseudoRandom (Solo Mode: 3) */
	            EEprom.Ripetizioni= RFPK.Ripetizioni;    /* Numero di ripetizioni: 0=Illimitato     (Solo Mode: 5,6,7) */
	
	            /* -------< Coefficienti Channel ("AT+ASACCHANNEL=") >------     */
	            EEprom.FhMode   =RFPK.FhMode;   /* Modo funzionamento modulo                    */    
	            EEprom.ChannelRF=RFPK.ChannelRF;/* Canale Radio-Solo se senza Frequency Hopping */
	
	            RADIO_off();                   // Spegne per non interferire con la EEPROM
	            /* -------< Salvataggio in EEPROM >----- */
	            tsk_lock();                       /* ----< BLOCCA INTERRUPT DI TASK >--- */
	            EEPROM_save_cache();
	            tsk_unlock();                     /* ----< SBLOCCA INTERRUPT DI TASK >--- */
	            sprintf(risp,"+ASACSAVE: Saved!");
	            RFPK.bkMode=RF_MODE_CMD_IDLE;      // Forza la reimpostazione della modalità
	
	            Nor.ResetReq=TRUE;              // Richiede il boot
	            return;
	    }
	}
    /*--------------------------------------------------------------------------*/
//AT LOAD    /* --< Comando di Rilettura parametri dalla EEPROM
    /*--------------------------------------------------------------------------*/
	if (cmdlen>=11){
	    if (strncmp(cmd,"AT+ASACLOAD",11)==0){
	
	            RADIO_off();                   // Spegne per non interferire con la EEPROM
	
	            tsk_lock();                     /* ----< BLOCCA INTERRUPT DI TASK >--- */
	            /* -------< Lettura EEPROM >----- */
	            EEPROM_load_cache();
	            tsk_unlock();                   /* ----< SBLOCCA INTERRUPT DI TASK >--- */
	
	            /* -------< Coefficienti Radio ("AT+ASACRADIO=") >-----     */
	            RADIO_init();
	            Set_Cpu_Speed_Rf();             // Cambia il clock, ma fa perdere il task usb!!!
	            RADIO_on();
	
	            /* -------< Coefficienti Modulo ("AT+ASACMOD=") >------     */
	            RFPK.Mode  =EEprom.Mode;        /* 0=Idle; 1..3=Diag Rf fissa; 4..6=Diag Pacchetti; 10=Modo Periferica; 11=Modo Nor     */ 
	            RFPK.bkMode=!RFPK.Mode;         /* Backup di Mode (x rilevare le variazioni     */
	        #ifdef TIDDY_BOARD           // Definito=Usa il target del NOR 20
	            RFPK.Protect=Setup.RF.Protect;  /* Codice di protezione                         */
	            RFPK.TermNr= Setup.NumTerm;     /* Numero Terminale (Solo Mode: 1,3,4,5,6)      */            
	        #else
	            RFPK.Protect=EEprom.Protect;    /* Codice di protezione                         */
	            RFPK.TermNr=EEprom.TermNr;      /* Numero di terminale                          */
	        #endif    
	            RFPK.NorNr =EEprom.NorNr;        /* Numero di Nor      (solo Mode: 11)           */
	            RFPK.Valore=EEprom.Valore;      /* Valore 0=ZERO,1=UNO, 2=Pattern PseudoRandom (Solo Mode: 3) */
	            RFPK.Ripetizioni=EEprom.Ripetizioni; /* Numero di ripetizioni: 0=Illimitato     (Solo Mode: 5,6,7) */
	            sprintf(RFPK.Testo,"The quick joung fox jump over the lazy dog");
	        
	            /* -------< Coefficienti Channel ("AT+ASACCHANNEL=") >------     */
	            RFPK.FhMode   =EEprom.FhMode;    /* Modo funzionamento modulo                    */    
	            RFPK.ChannelRF=EEprom.ChannelRF; /* Canale Radio-Solo se senza Frequency Hopping */
	
	            RFPK.bkMode=RF_MODE_CMD_IDLE;      // Forza la reimpostazione della modalità
	            sprintf(risp,"+ASACLOAD: Reloaded!");
	            return;
	    }
	}

    /*--------------------------------------------------------------------------*/
//ATINFO     /* --< Comando di INFO >--*/
    /*--------------------------------------------------------------------------*/
	if (cmdlen>=9){
		if (strncmp(cmd,"AT+ASACI?",9)==0){
		    sprintf(risp,"+ASACI? -CHANNEL:(Ch=%u, Mod=%u); -MODE:(Mode=%u, Code=%u, Nt=%u); -RADIO:(Coef=%i, Pwr=%u, Ramp=%u, SizePr=%u, Squelch=%u, Dev=%u, BW=%u, Prot=%u, HVers=%i)"     
		                ,RFPK.ChannelRF ,RFPK.FhMode        ,RFPK.Mode ,RFPK.Protect ,RFPK.TermNr,   
						RadioBase.OffsetCalPll
		                ,RadioBase.PowerPa  
		                ,RadioBase.PowerRaise
		                ,RadioBase.SzPreamb
		                ,RadioBase.RssiBase 
		                ,RadioBase.Deviaz
		                ,RadioBase.BandWidth
						,Setup.RF.u16_protocolType
						,EEprom.HVersion
		                 );
		        return;
		}
	}

#ifdef  Nor2xBootMode       // Nor2xBootMode = TRUE ---->  BOOT!
    /*--------------------------------------------------------------------------*/
//AT SET BOOT /* --< Comando di attivazione modo Boot >--*/ (solo NOR o Tyhoon)
    /*--------------------------------------------------------------------------*/
	if (cmdlen>=14){    
		if (strncmp(cmd,"AT+ASACSETBOOT",14)==0){
			sprintf(risp,"AT+ASACSETBOOT!"); 
			
			RADIO_off();                   // Spegne per non interferire con la EEPROM
			
			tsk_lock();                     /* ----< BLOCCA INTERRUPT DI TASK >--- */
			
			/* --------------------------------------------------------------   */
			/* Significato della locazione EE_BOOT_REQ:                         */
			/*  FFFF=Nessun applicativo installato: necessatrio Boot            */
			/*  0000=Applicativo correttamente configurato                      */
			/*  0001..FFFE= Applicativo ha richiesto il Boot                    */
			/* --------------------------------------------------------------   */
			SetBootMode(1);
			tsk_unlock();                   /* ----< SBLOCCA INTERRUPT DI TASK >--- */
			
			Nor.ResetReq=TRUE;              // Richiede il boot
			return;
		}
	}
#endif               

    /*--------------------------------------------------------------------------*/
//AT SET PROTECT /* --< Imposta temporaneamente il Cript CODE >--
    /*--------------------------------------------------------------------------*/
    if (cmdlen>18){
	    if (strncmp(cmd,"AT+ASACSETPROTECT=",18)==0){
	        i=18;
	        /* --------------------< SET PROTECT>---------------------------  */
	        /* Impara la nuova modalità di funzionamento:  0=Invalid; 1=dle; 2=Diagnostico-Tx fissa.; 3=Diagnostico=Tx pacchetti; 4=Diagnostico=Tx->Rx; 5=Diagnostico=Rx->Tx */
	        RFPK.Protect=atoi((const char *)(cmd+i));             
	        sprintf(risp,"AT+ASACSETPROTECT=%u",RFPK.Protect); 
	        return;
	    }
	}

    /*--------------------------------------------------------------------------*/
//AT DEF PROTECT /* --< Ritorna il Cript CODE al valore di default (riletto da EEPROM) >--
    /*--------------------------------------------------------------------------*/
    if (cmdlen>=18){
	    if (strncmp(cmd,"AT+ASACDEFPROTECT!",18)==0){
	        RFPK.Protect=EEprom.Protect;    /* Codice di protezione                         */
	        sprintf(risp,"AT+ASACDEFPROTECT=%u",RFPK.Protect); 
	        return;
	    }
	}

    /*--------------------------------------------------------------------------*/
//AT GET PROTECT /* --< Ritorna il Cript CODE Corrente >--
    /*--------------------------------------------------------------------------*/
    if (cmdlen>=18){
	    if (strncmp(cmd,"AT+ASACGETPROTECT?",18)==0){
	        sprintf(risp,"AT+ASACGETPROTECT=%u",RFPK.Protect); 
	        return;
	    }
	}

    /*--------------------------------------------------------------------------*/
//AT    
    /*--------------------------------------------------------------------------*/
	if (cmdlen>=7){
	    if (strncmp(cmd,"AT+ASAC",7)==0){
	        sprintf(risp,"+ASAC:OK");
	        return;
	    }         
	}       
    return;
}
