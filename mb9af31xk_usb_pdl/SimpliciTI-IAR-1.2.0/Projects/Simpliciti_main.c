/* Includes ------------------------------------------------------------------*/

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk.h"
#include "nwk_frame.h"
#include "nwk_ping.h"
#include "nwk_QMgmt.h"

#include "crc.h"
#include "rtl.h"

#include "string.h"
#include "lowradio.h"
#include "nvmem.h"

#include "radio_drv.h"
#include "mrfi_board_defs.h"

#define PROTO_NR_RAWRETRY			4
#define PROTO_NR_RETRY				6
#define RX_ACK_TOUT					20		//10msec
#define RX_PING_TOUT				5		//5msec

//Enable piggybacking of info request
#define PIGGYBACK_INFOREQ
#define STI_CRCKEY					0xA5A5	//Local init of CRC

//Freq. Agility
//Samples sensitivity
#define SSIZE    					12
#define IN_A_ROW  					3

//#define SENSE_BEFORE_CHANGE					//Define this to sense noise before change channel
#ifdef SENSE_BEFORE_CHANGE
	#define TICKS_BEFORE_SENSE_CHNL		200	   			//2 seconds before try change channel
#else
	#define TICKS_BEFORE_SENSE_CHNL		40	   			//.2 seconds before try change channel	
#endif

//Define campo info di protocollo
#define MASK_PKT_DATA  		0x1000
#define MASK_PKT_LAST 		0x2000
//
#define MASK_PKT_INFOREQ  	0x4000
#define MASK_PKT_INFOACK  	0x8000
//
#define MASK_ERR_QFULL		0x0100
#define MASK_ERR_CRCFAIL	0x0200
#define MASK_ERR_NRWRONG	0x0400
#define MASK_ERR_NEEDTIME	0x0800
//
#define MASK_PKT_INFO 		0xF000
#define MASK_PKT_ERROR 		0x0F00
#define MASK_PKT_INDEX 		0x000F
#define MASK_PKT_CHNL 		0x00F0
//
#define PORT_LOCAL			0x3D
#define PORT_REMOTE			0x3E

#ifdef TIDDY_BUILD_SIMPLICITY
	#define MAX_SIZE_RFQUEUE		2
#else
	#define MAX_SIZE_RFQUEUE		8
#endif 

//Externs
extern RF_DRV 				rf_drv;
extern uint8_t 				sInit_done;
 
extern tipoRadioBase   		RadioBase;
extern tipoEEprom    		EEprom;
extern tipoSetup    		Setup;

extern unsigned int  		TIMER_SCA;         	   	//Tick timer
extern volatile uint8_t 	sTid;					//Signal ping sequence
extern volatile bool 		rx_isr_context;			//Isr sync management
extern const 	uint16_t 	CrcTab[256]; 			//CRC table

extern uint8_t STI_COMMS_queue_level(void);
extern __task void STI_RF_Task(void);

static u16 STI_scanChannels(addr_t dst_address);
static u16 STI_raw_send_msg(addr_t dst_address, char* msg,u32 len);
//-----------------------------------------------------------------
//Startup has logical channel 0 -> look at channel table
static 	uint8_t 			sChannel = 0;
volatile uint32_t 			sPeerFrameTick=0;

//RF task management
static 	linkID_t 			listenlinkID;
uint8_t 					rf_opened=false;	

static OS_TID				rf_tskid=NULL;
OS_TID						tsr_tskid=NULL;

//Mutext handling on send/recv messages 
volatile OS_MUT 			send_mtx;
//-----------------------------------------------------------------
//Data decoded
static volatile uint16_t  	ack_rx_data;				//This is global info received during info packets			
//-----------------------------------------------------------------

typedef struct _QPACKET{
	uint8_t  	pkt_qmsg[MAX_SIZE_PACKET]; 	//Payload applicativo
	uint16_t	pkt_qlen;					//Lunghezza payload		
	uint16_t   	pkt_okmask;					//Maschera sottopacchetti ricevuti
	uint32_t 	pkt_time;					//Ticktime
	uint8_t		pkt_nr;
	addr_t 		pkt_addr;					//Indirizzo sorgente
}QPACKET;

//Gestione coda minipacchetti 
static QPACKET pkt_queue[MAX_SIZE_RFQUEUE];
//static volatile glbl_pkt_qid;
static volatile uint8_t		pkt_hopscount;				//Number of actual consecutive hops..	
static volatile uint8_t		pkt_basechan;				//Remember base chan


#ifdef SENSE_BEFORE_CHANGE
	static int8_t sSample[SSIZE];	/* auto-channel-change policy ... */
#endif 

extern u8 CC2500_PartN;                 //0x30  CC2500 part number                             
extern u8 CC2500_Vers;	                //0x31  Current version number   
//-----------------------------------------------------------------
//Note: this function builds a node based on Id plant and Serial node
addr_t build_address(uint32_t matr_node){
	addr_t local_addr;
	
	//Note: matr: 20000 - 85500
	matr_node -= 20000; 
	local_addr.addr[0]= 10;			//Local network 
	local_addr.addr[1]= EEprom.Protect;
	local_addr.addr[2]= (matr_node & 0xFF00)>>8;
	local_addr.addr[3]= (matr_node & 0xFF);

	return local_addr;
}

uint32_t build_mat(addr_t* srcaddr){	
	uint32_t mmatr=20000;	
	mmatr+=srcaddr->addr[3];
	mmatr+=(((uint32_t)srcaddr->addr[2])<<8);
	return mmatr;
}
//-----------------------------------------------------------------
//Hopping codificato a salti di +/- 8 max
#ifdef FREQUENCY_HOPPING_ASAC
uint8_t __inline handle_sethopping(uint8_t nxt_hop){
	freqEntry_t 	freq;

	//if (nxt_hop){
	freq.logicalChan = RadioFh.ChannelRF;		
	//First hop case.. 
	if (freq.logicalChan<4){
		pkt_basechan = freq.logicalChan;
		freq.logicalChan=3;
		pkt_hopscount=0;	
	}else{
		pkt_hopscount++;
		if (pkt_hopscount>MAX_HOPS_CONSECUTIVE){
			freq.logicalChan=pkt_basechan;
			SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &freq);
			RadioFh.ChannelRF=freq.logicalChan;	
			return FALSE;
		}	
	}		
	//Decode jump..
	freq.logicalChan = freq.logicalChan + (nxt_hop + 1);			
	if (freq.logicalChan > 24 ){
		freq.logicalChan = 3 + ( freq.logicalChan - 24);   
	}
	//						
	SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &freq);
	RadioFh.ChannelRF=freq.logicalChan;								
	//}	
	return TRUE;	//Hopping case
}
 
//Valuta un hop relativo al canale attuale di max +- 8 posti
uint8_t __inline handle_evalhopping(void){
	return (MRFI_RandomByte()>>4);		//0..15
}
#endif
//-----------------------------------------------------------------
u16 STI_RF_get_messageNT(uint32_t* matr,uint32_t* nterm,uint8_t *ptrMsg, uint16_t *MsgSize){
	u16 result;

	os_mut_wait(RFPK.Qmtx,0xFFFF);
	if (RFPK.Buff_H_W_idx==RFPK.Buff_H_R_idx){
		result=FALSE;
	}else{
		*nterm=RFPK.Buff_H_Msg_NT[RFPK.Buff_H_R_idx];			
		*matr=build_mat(&RFPK.Buff_H_Msg_ADDR[RFPK.Buff_H_R_idx]);	  
		*MsgSize=RFPK.Buff_H_Msg_Len[RFPK.Buff_H_R_idx];
		memcpy(ptrMsg,(void*)&RFPK.Buff_H_Msg[RFPK.Buff_H_R_idx],RFPK.Buff_H_Msg_Len[RFPK.Buff_H_R_idx]);					
		ptrMsg[RFPK.Buff_H_Msg_Len[RFPK.Buff_H_R_idx]]=0;	
		RFPK.Buff_H_R_idx=((RFPK.Buff_H_R_idx + 1)&RF_MAX_PACKET_MASK);//%RF_MAX_PACKET_QUEUE;
		result=TRUE;
		sPeerFrameTick =TIMER_SCA;	  				//Update activity counter
	}	
	os_mut_release(RFPK.Qmtx);	
	return result;
}
//-----------------------------------------------------------------
u16 STI_RF_get_message(u32* matr, char *ptrMsg, u16 *MsgSize){
	u16 result;

	os_mut_wait(RFPK.Qmtx,0xFFFF);
	if (RFPK.Buff_H_W_idx==RFPK.Buff_H_R_idx){
		result=FALSE;
	}else{	
		*matr=build_mat(&RFPK.Buff_H_Msg_ADDR[RFPK.Buff_H_R_idx]); //RFPK.Buff_H_Msg_NT[RFPK.Buff_H_R_idx];	  
		*MsgSize=RFPK.Buff_H_Msg_Len[RFPK.Buff_H_R_idx];
		memcpy(ptrMsg,(void*)&RFPK.Buff_H_Msg[RFPK.Buff_H_R_idx],RFPK.Buff_H_Msg_Len[RFPK.Buff_H_R_idx]);					
		ptrMsg[RFPK.Buff_H_Msg_Len[RFPK.Buff_H_R_idx]]=0;			
		RFPK.Buff_H_R_idx=(RFPK.Buff_H_R_idx + 1)&RF_MAX_PACKET_MASK;
		result=TRUE;
	}
	os_mut_release(RFPK.Qmtx);
	return result;
}
//-----------------------------------------------------------------
u16 STI_RF_sended_message(void){return 1;}
u16 STI_RF_send_message_space(void){return 1;}
void STI_RF_Send_Flush(void){
	//Flush coda rx
	os_mut_wait(RFPK.Qmtx,0xFFFF);
	RFPK.Buff_H_W_idx = 0;
	RFPK.Buff_H_R_idx = 0;
	//Flush coda rx basso livello
	memset((void*)&pkt_queue,0,sizeof(pkt_queue));
	os_mut_release(RFPK.Qmtx);	
}
//-----------------------------------------------------------------
static __inline uint16_t crc_ccitt_byte(uint16_t crc, const u8 c){
 	return (crc >> 8) ^ CrcTab[(crc ^ c) & 0xff];
}
static uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len){
 	while (len--)
 		crc = crc_ccitt_byte(crc, *buffer++);
 	return crc;
}

//-----------------------------------------------------------------
//Note: Nterm can be nr. of node to communicate with NOR
// 		Nterm can be matr. to comm with another node	
u16 STI_RF_send_message(u32 matr, char *ptrMsg, u32 uLenMess){
	addr_t 		dst_addr;
	uint16_t 	rc,result;
	uint16_t 	mcrc;
	int 		i,k;

 	//Critical section to handle sending message
	os_mut_wait(send_mtx,0xFFFF);
		
	sPeerFrameTick =TIMER_SCA;	  				//Update activity counter

	//Maintain compatibility with bullshit
	if (!uLenMess){ 
		uLenMess=strlen(ptrMsg);
	}

	if ((matr>=20000)&&(matr<=85500)){			//This is another matr. node!!
		dst_addr=build_address(matr);
		RFPK.Buff_T_Msg[0][0]=Setup.NumTerm;	//Keep 4 compatibility 	
	}
	else{ 										//This is nor			
		dst_addr=build_address(Setup.RF.u32_accessPointSN[0]);	
		RFPK.Buff_T_Msg[0][0]=Setup.NumTerm;	// (NTerm&0xFF);
	}
	memcpy(&RFPK.Buff_T_Msg[0][1],ptrMsg,uLenMess);
	
	//Calcolo 2 byte di crc16
	mcrc = crc_ccitt(STI_CRCKEY, (void*)RFPK.Buff_T_Msg[0],(uLenMess + 1));
	RFPK.Buff_T_Msg[0][uLenMess + 1]=mcrc>>8;
	RFPK.Buff_T_Msg[0][uLenMess + 2]=mcrc&0xFF;
	
	result = FALSE;
	for (i=0;i<PROTO_NR_RETRY;i++){	   	 
		rc = STI_raw_send_msg(dst_addr,RFPK.Buff_T_Msg[0],(uLenMess + 1 + 2));	//CRC + NR Term
		switch(rc){
			case 0: 
				result= TRUE;
				goto exit_loop;				
				break;
			case 1: 
				for (k=0;(k<3)&&(!STI_scanChannels(dst_addr));k++);
				os_dly_wait(2);
				break;
			case 2: 
				os_dly_wait(2);	//Leave time to work target (Time req) then exit
				result= TRUE;	   	
				goto exit_loop;
				break;
			case 3: 
				os_dly_wait(2);	//Leave time to work target (Qfull) then resend!!
				break;
			default: 
				os_dly_wait(2);	
				break;								
		}
	}	 

exit_loop:
	os_mut_release(send_mtx);
	return result;
}
//-----------------------------------------------------------------
#ifdef PIGGYBACK_INFOREQ
//Capisco il numero di minipack da spedire
static uint8_t STI_get_nrminipackets(uint32_t send_len){
	uint8_t mindex=0;

	while (1){
		mindex++;
		if (send_len>(MAX_APP_PAYLOAD-2))
			send_len-=(MAX_APP_PAYLOAD-2); 			
		else
			return mindex;
	}
	return 0;	
}								
//Partendo dall'ultimo minipack controllo se e' stato acknowledged..
static uint8_t STI_get_lastminipacket(uint16_t send_mask,uint8_t send_nr){
	uint8_t i;
	for (i=(send_nr-1); i>=0; i--){
		if ((send_mask & (0x0001<<i))==0) 
			return i;			
	}
	return 0;	//Should never happen!!	
}
#endif
//Track result values  1= no ack 2= ack	recv 0= OK
u16 STI_raw_send_msg(addr_t dst_address, char* msgptr,u32 msglen){
	uint8_t  		chunklen;
	ioctlRawSend_t  send_ioctl; 

	int i,j;

#ifdef PIGGYBACK_INFOREQ
	int send_last;
	int send_nr;
#endif 

	uint8_t			result;
			
	uint8_t* 		tmp_ptr;
	uint32_t  		tmp_len;
	uint32_t 		count;

	uint16_t		send_mask;
			
	uint8_t 		mmsg[MAX_APP_PAYLOAD]; 	//Chunk built
	int				mlen;
	uint16_t 		minfo;

#ifndef PIGGYBACK_INFOREQ
	uint8_t			mreq[2]={0,(MASK_PKT_INFOREQ>>8)};
#endif 
	
	smplStatus_t	rc;

#ifdef FREQUENCY_HOPPING_ASAC
    uint8_t			nxt_hop;
#endif 
	
	result=1;

	//Preset iocontrol 
	send_ioctl.addr = (addr_t *)&dst_address;
	//send_ioctl.msg  = mmsg;		//Ptr to message
	//send_ioctl.len  = mlen;
	send_ioctl.port = PORT_REMOTE;
	send_ioctl.forced = 0;
	
	//Mask of acknowledged packets	
	send_mask=0;

#ifdef PIGGYBACK_INFOREQ	//NR minipackets to send...		
	send_nr = STI_get_nrminipackets(msglen);
#endif

	for (i=0;i<PROTO_NR_RAWRETRY;i++){	

		//------------------------------------------- 
		j=0;
		tmp_ptr=msgptr;
		tmp_len=msglen;
		
#ifdef PIGGYBACK_INFOREQ	//Eval last packet index to send(0 based)
		send_last = STI_get_lastminipacket(send_mask,send_nr);
#endif
		//reset ack status 4 every loop
		ack_rx_data = 0;
			
		//Loop of forced send minipackets 
		while(tmp_len){
			//Size minipackets
			chunklen=(tmp_len>(MAX_APP_PAYLOAD-2))?(MAX_APP_PAYLOAD-2):tmp_len;
			
			//Packet wasn't already managed?
			if ((send_mask & (0x0001<<j))==0){			 	
				minfo = (j | MASK_PKT_DATA);
				//if (j==0)				
				//	minfo|= MASK_PKT_FIRST;	
							 
				if (tmp_len<=chunklen)	
					minfo|= MASK_PKT_LAST;

#ifdef PIGGYBACK_INFOREQ	//Piggyback inforequest on last packet
	   			if (j==send_last)	
					minfo|= MASK_PKT_INFOREQ;	
#endif 
				//Little endian info 
				mmsg[0]=minfo & MASK_PKT_INDEX;
				mmsg[1]=((minfo&0xFF00)>>8);
				memcpy(&mmsg[2],tmp_ptr,chunklen);

				mlen =(chunklen + 2); 
				send_ioctl.len = mlen;
				send_ioctl.msg = mmsg;
					
				rc = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send_ioctl);
			}			
			
			tmp_ptr+=chunklen; 
			tmp_len-=chunklen;
						
			j++;
		}

#ifndef PIGGYBACK_INFOREQ	
		//Build info request	
		send_ioctl.msg = mreq;
		send_ioctl.len = sizeof(mreq);
		rc = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send_ioctl);	
#endif

 		//Wait info ack..
		count = (RX_ACK_TOUT * 1000)/MRFI_MAX_DELAY_US;
  		while(count && (!ack_rx_data)){
			BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
			os_tsk_pass();
			count--;			
     	}

		if (ack_rx_data){	
			//Analizzo risposta	(c'e' stata!!)	 			
			result=0xFF;			
			if (ack_rx_data & MASK_ERR_QFULL){
				result=3;	//Se la coda RF del destinatario e' piena, e' meglio uscire con err
				goto exit_loop;
			}
			else if (ack_rx_data & MASK_ERR_CRCFAIL){
				send_mask =0;	//Rimando tutti i minipack
			}
			else if (ack_rx_data & MASK_ERR_NRWRONG){
				send_mask = ack_rx_data & MASK_PKT_INDEX;
			}
			else{	
				send_mask = ack_rx_data & MASK_PKT_INDEX;
				
				//Tutti i minipack arrivati?
	 			if (send_mask == ((0x0001<<j)-1)){
					if (ack_rx_data & MASK_ERR_NEEDTIME)
						result= 2; 	//Richiesta di delay..
					else
						result= 0;

					//Se sono arrivati tutti i minipack a destinazione
					//provo a vedere se mi viene segnalato un cambio canale
#ifdef FREQUENCY_HOPPING_ASAC
					nxt_hop = (ack_rx_data & MASK_PKT_CHNL)>>4;
					if (!handle_sethopping(nxt_hop))   	//Se l'host torna ai canali base...
						 os_dly_wait(HOPS_DELAY_BASE);	
#endif
					goto exit_loop;
				}				
			}
			os_dly_wait(1);
		}
			
next_loop:	
	}//End for 
	//result=result;
		 
exit_loop:
	return result;	
}
//-----------------------------------------------------------------
//Viene inviata all'host la response sullo stato dei minipack in coda
void __inline handle_response(uint8_t pkt_qid,uint8_t nxt_hop){
	//Data to be sent outside interrupt context
	ioctlRawSend_t    	tx_ioctl;
	uint8_t  			tx_data[2];	

	//handle ack payload to sender
	tx_data[0]= (uint8_t)(pkt_queue[pkt_qid].pkt_okmask & MASK_PKT_INDEX);
	//embed eventual hopping info
	tx_data[0]|= (nxt_hop <<4);
	//ifo payload
	tx_data[1]= (((pkt_queue[pkt_qid].pkt_okmask & 0xFF00) | MASK_PKT_INFOACK) >>8);

	tx_ioctl.addr = &pkt_queue[pkt_qid].pkt_addr;
	tx_ioctl.msg  = tx_data;
	tx_ioctl.len  = sizeof(tx_data);
	tx_ioctl.port = PORT_REMOTE;

#if 0 //ndef NOR_BUILD_SIMPLICITY
	tx_ioctl.forced = 0;
#endif
	tx_ioctl.forced = 1;	//In tiddy ack is directed in interrupt mode
	SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &tx_ioctl);
}
//-----------------------------------------------------------------
//Gestione del pacchetto di richiesta accodamento e stato
//Se tutto ok viene accodato il paccketto RF ricostruito dai minipackets e ritorna TRUE
uint8_t __inline handle_qpacket(uint8_t pkt_qid){
	uint16_t 			eval_crc;
	uint16_t 			real_crc;
	uint8_t				result;
	
	result=FALSE;
		
	if (pkt_queue[pkt_qid].pkt_qlen==0)
		goto bypass_enqueue;

	if ((pkt_queue[pkt_qid].pkt_okmask & MASK_PKT_INDEX) != ((0x01<<pkt_queue[pkt_qid].pkt_nr)-1)){
		pkt_queue[pkt_qid].pkt_okmask |= MASK_ERR_NRWRONG;
		goto bypass_enqueue;
	}else{  
		pkt_queue[pkt_qid].pkt_okmask &= ~MASK_ERR_NRWRONG;			 
	}

	//Do eval CRC
	eval_crc = crc_ccitt(STI_CRCKEY,pkt_queue[pkt_qid].pkt_qmsg,pkt_queue[pkt_qid].pkt_qlen - 2);
	real_crc = (pkt_queue[pkt_qid].pkt_qmsg[pkt_queue[pkt_qid].pkt_qlen - 2])<<8;
	real_crc |= pkt_queue[pkt_qid].pkt_qmsg[pkt_queue[pkt_qid].pkt_qlen - 1];
	
	if (real_crc==eval_crc){
		if (((RFPK.Buff_H_W_idx + 1)&RF_MAX_PACKET_MASK) != RFPK.Buff_H_R_idx){
			//Do enqueue
			RFPK.Buff_H_Msg_Len[RFPK.Buff_H_W_idx]=pkt_queue[pkt_qid].pkt_qlen - 2 - 1;	//Tolta la CRC e NTerm	
							 
			memcpy(RFPK.Buff_H_Msg[RFPK.Buff_H_W_idx],(void*)&pkt_queue[pkt_qid].pkt_qmsg[1],RFPK.Buff_H_Msg_Len[RFPK.Buff_H_W_idx]);
			RFPK.Buff_H_Msg_NT[RFPK.Buff_H_W_idx]=pkt_queue[pkt_qid].pkt_qmsg[0];			//build_mat((addr_t*)pinfo->peerAddr);
			memcpy(&RFPK.Buff_H_Msg_ADDR[RFPK.Buff_H_W_idx].addr,&pkt_queue[pkt_qid].pkt_addr,sizeof(addr_t));	//build_mat((addr_t*)pinfo->peerAddr);
			RFPK.Buff_H_W_idx=((RFPK.Buff_H_W_idx + 1)&RF_MAX_PACKET_MASK);

#if 0		//TODO: verify if necessary this!!	
			//If queue becomes full.. let me time to work response
			if (((RFPK.Buff_H_W_idx + 1)&RF_MAX_PACKET_MASK) == RFPK.Buff_H_R_idx) 
				pkt_queue[pkt_qid].pkt_okmask|= MASK_ERR_NEEDTIME;
#endif 
			
			result=TRUE;  	//Packet was enqueued
		}
		else{//pkt_queue[pkt_qid].pkt_okmask=0; 	//Full queue
			pkt_queue[pkt_qid].pkt_okmask|= MASK_ERR_QFULL;							
		}
	}
	else{//pkt_queue[pkt_qid].pkt_okmask=0;		//Invalidate ok mask .. crc failure
		pkt_queue[pkt_qid].pkt_okmask|= MASK_ERR_CRCFAIL; 
	}

bypass_enqueue:
#ifdef NOR_BUILD_SIMPLICITY
	#if 0
		//Nor signals need of time to send data from serial
		if (STI_COMMS_queue_level()){		
			pkt_queue[pkt_qid].pkt_okmask|= MASK_ERR_NEEDTIME;		
		}
	#endif 
#endif
	return result;	
}
//-----------------------------------------------------------------
uint8_t sCallBack(linkID_t port){
	uint16_t			pkt_mask;
	uint8_t				pkt_len;
	uint16_t 			pkt_index;
	uint16_t 			pkt_info;
	uint16_t 			pkt_qmempos;
	//
	uint8_t  			rx_msg[MAX_APP_PAYLOAD];	//Single minipack in stack..
	addr_t				rx_addr;
	//	
	uint8_t 			pkt_qid,pkt_qidtout;	
	//
	ioctlRawReceive_t 	recv_ioctl;
	uint8_t				pkt_enq;
	uint8_t				nxt_hop;
		
	//-------------------------------------------------------------
	recv_ioctl.port = PORT_REMOTE;		//Listening port	
    recv_ioctl.msg  = rx_msg;			//Payload
    recv_ioctl.addr = &rx_addr;			//Filled by ioctl	   		
	
	//Get the packet in every case	(from lower queue....)
	if (SMPL_SUCCESS != SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &recv_ioctl)){
		goto cb_exit;
	}	
	//Sanity check 
	if (recv_ioctl.len<2){ 
		goto cb_exit;
	}		
	//-------------------------------------------------------------	
	//Deserializzo da network a uint16 	
	pkt_info = ((uint16_t)rx_msg[1]<<8)|((uint16_t)rx_msg[0]& 0x00FF);
	pkt_len	 = (recv_ioctl.len-2);
	//-------------------------------------------------------------
	//This is an ack response from last packet sent (no data inside)
	if ((pkt_info & MASK_PKT_INFOACK) && (pkt_len==0)){		
		ack_rx_data = pkt_info;				
		goto cb_exit;							  			 		
	}
	//-------------------------------------------------------------
#ifdef ENABLE_MULTIPEER_CACHE 
	//Cache multi host
	//Determine packet queue for standard packets
	for (pkt_qid=0,pkt_qidtout=MAX_SIZE_RFQUEUE;pkt_qid<MAX_SIZE_RFQUEUE; pkt_qid++){						
		if ((pkt_queue[pkt_qid].pkt_addr.addr[2]==rx_addr.addr[2]) &&
			(pkt_queue[pkt_qid].pkt_addr.addr[3]==rx_addr.addr[3])){					//Address ok (gli altri byte sono filtrati da protocollo)						
			break;
		}
		else{
			if ((TIMER_SCA - pkt_queue[pkt_qid].pkt_time) > 10){						//Persistence elapsed (100msec)
				if (pkt_qidtout==MAX_SIZE_RFQUEUE)										//Assign first free position 
					pkt_qidtout = pkt_qid;					
			}
		}
	}		

	if (pkt_qid==MAX_SIZE_RFQUEUE){
		if (pkt_qidtout==MAX_SIZE_RFQUEUE){
			goto cb_exit;	//Errore: non c'e' spazio in cache indir.
		}
		else{	 
			pkt_qid = pkt_qidtout;
			memcpy((void*)&pkt_queue[pkt_qid].pkt_addr,(void*)&rx_addr,sizeof(addr_t));
			pkt_queue[pkt_qid].pkt_okmask=0;	//Init mask
			pkt_queue[pkt_qid].pkt_qlen=0;
			pkt_queue[pkt_qid].pkt_nr=0;			
		}	
	}else{	//Trovato indir. in cache
		if ((TIMER_SCA - pkt_queue[pkt_qid].pkt_time) > RX_ACK_TOUT){
			pkt_queue[pkt_qid].pkt_okmask=0;	//Init mask
			pkt_queue[pkt_qid].pkt_qlen=0;
			pkt_queue[pkt_qid].pkt_nr=0;	
		}	
	}	
#else
	//Forzo acquisizione indirizzo e pacchetto di coda 0
	pkt_qid=0; 
	memcpy((void*)&pkt_queue[pkt_qid].pkt_addr,(void*)&rx_addr,sizeof(addr_t));
#endif 

	pkt_queue[pkt_qid].pkt_time = TIMER_SCA;
		
	//-------------------------------------------------------------
	if ((pkt_info & MASK_PKT_DATA)&&(pkt_len!=0)){
		pkt_index=(pkt_info & MASK_PKT_INDEX);
		if (pkt_index>((MAX_SIZE_PACKET/MAX_APP_PAYLOAD)-1)){	
			goto cb_exit;
		}					
		pkt_mask = (0x0001<<pkt_index);
		//C'era gia' un pacchetto? resetto stato logico		
		if (pkt_queue[pkt_qid].pkt_okmask & pkt_mask){ 
			pkt_queue[pkt_qid].pkt_okmask=0;		//Erase state machine on dup pack
			pkt_queue[pkt_qid].pkt_qlen=0;
			pkt_queue[pkt_qid].pkt_nr=0;
		}
		
		pkt_queue[pkt_qid].pkt_nr++;
		pkt_queue[pkt_qid].pkt_okmask |= pkt_mask;
		pkt_qmempos = pkt_index * (MAX_APP_PAYLOAD - 2);	//Every packet transports data + 2 info bytes				
		memcpy(&pkt_queue[pkt_qid].pkt_qmsg[pkt_qmempos],&rx_msg[2],pkt_len);
						
		//Last minipacket contains len & nr packet info
		if (pkt_info & MASK_PKT_LAST){ 
			pkt_queue[pkt_qid].pkt_qlen = pkt_qmempos + pkt_len;
		}			 
	}

	//-------------------------------------------------------------	
	if (pkt_info & MASK_PKT_INFOREQ){ //&& (pkt_len==0)){		
#if 0 //def NOR_BUILD_SIMPLICITY
		glbl_pkt_qid = pkt_qid;
		MRFI_DISABLE_SYNC_PIN_INT( );
		rx_isr_context = true;		
		isr_evt_set(0x0001,tsr_tskid);
#endif 	

		nxt_hop=0;
		pkt_enq=handle_qpacket(pkt_qid);   	//Enqueue and sign success reception

#ifdef FREQUENCY_HOPPING_ASAC
		if (pkt_enq){						//Handle change channel as soon as possible
			nxt_hop = handle_evalhopping();	
		}
#endif 				
		handle_response(pkt_qid,nxt_hop);	//Send back info response
#ifdef FREQUENCY_HOPPING_ASAC
		if (pkt_enq){
			handle_sethopping(nxt_hop);					
		}
#endif 
	}																												  	

cb_exit:
	return TRUE;
}
//-----------------------------------------------------------------
void STI_init(void){
	addr_t lAddr;//={10,0,0,1};
	
	BSP_Init();
	os_mut_init(send_mtx);		//Init mutex cc2500 


    // Controlla che tipo di chip è installato:
    if (send_cmd_ChipCon(cc2500_PARTNUM,0xFF,NULL)==0xFFFF){// Se nessun chip è installato:
        RadioBase.RFInstalled=FALSE;                // FALSE=RF non installata; TRUE=Rf installata
    }else{
        RadioBase.RFInstalled=TRUE;                 // FALSE=RF non installata; TRUE=Rf installata
    }

	CC2500_PartN = send_cmd_ChipCon(cc2500_PARTNUM,0xFF,NULL);  //0x30  CC2500 part number                             
    CC2500_Vers  = send_cmd_ChipCon(cc2500_VERSION,0xFF,NULL);	//0x31  Current version number  


	//Clear main structure before anything..
	memset((void*)&RadioBase,0,sizeof(RadioBase));
	memset((void*)&RFPK,0,sizeof(tipoRFPK));
	memset(pkt_queue,0,sizeof(pkt_queue)); 
	os_mut_init(RFPK.Qmtx);
				          
	lAddr= build_address(EEprom.MATRICOLA);
	//Setup Node Address 
	SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
	//It fails without access point	..
	SMPL_Init(sCallBack);	
	RadioBase.RFInstalled=TRUE;
}
//-----------------------------------------------------------------
void __task Rx_Tsr(void){			     
	while(1){
		os_evt_wait_or(0x0001, 0xFFFF);	//Shared event		

#if	0//def NOR_BUILD_SIMPLICITY
		handle_qpacket(glbl_pkt_qid);
		rx_isr_context = false;
		MRFI_ENABLE_SYNC_PIN_INT( );
#endif
	}
}
//-----------------------------------------------------------------
//Simplicity standard
//Common function used even in nor and tiddy
void STI_open(void){
	addr_t 			src_addr;	//=build_address(70001);
	smplStatus_t	rc;

	memset(&src_addr,0,sizeof(addr_t));
	rc=SMPL_Commission(&src_addr,PORT_REMOTE,PORT_LOCAL,&listenlinkID);

	if (tsr_tskid){
		os_tsk_delete(tsr_tskid);
		tsr_tskid=NULL;	
	}

 	tsr_tskid=os_tsk_create(Rx_Tsr,4);	

	SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_AWAKE, 0);
	SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);		
	
	if (rf_tskid){
		os_tsk_delete(rf_tskid);
		rf_tskid=NULL;	
	}
	rf_opened=true;
 	
	rf_tskid=os_tsk_create(STI_RF_Task,4);	
}
//-----------------------------------------------------------------
//Simplicity standard
void STI_close(void){
	connInfo_t* 	pinfo=0;

	if (rf_tskid){ 	
		os_tsk_delete(rf_tskid);
		rf_tskid=NULL;	
	}

	if (rf_opened){
		SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_SLEEP, 0);
		rf_opened=false;

		pinfo = nwk_getConnInfo(listenlinkID);
		nwk_freeConnection(pinfo);
	} 
 	 
	if (tsr_tskid){
		os_tsk_delete(tsr_tskid);
		tsr_tskid=NULL;	
	}
}

//-----------------------------------------------------------------
#ifdef FREQUENCY_AGILITY
	static u16 STI_scanChannels(addr_t dst_address){
		uint8_t      	msg[MAX_PING_APP_FRAME];
		smplStatus_t	rc;
		//connInfo_t   *pCInfo   = nwk_getConnInfo(lid);
		uint8_t      	radioState = MRFI_GetRadioState();
		int i,count;
		uint8_t      	done     = 0;
		freqEntry_t  	curChan,nextChan;
		union{
	    	ioctlRawSend_t    send;
	    	ioctlRawReceive_t recv;
	  	} ioctl_info;
	
		//if (!pCInfo) 	
		//	return FALSE;
	#if 0
		/* quit if we serviced an app RECV frame (in 2 seconds)*/
		if (sPeerFrameTick > TIMER_SCA){	//Ticks Overflow happened!!
			if (((0xFFFFFFFF - sPeerFrameTick) + TIMER_SCA)<TICKS_BEFORE_SENSE_CHNL) 
				return;		
		}else{
			if ((TIMER_SCA - sPeerFrameTick)<TICKS_BEFORE_SENSE_CHNL)
			 	return; 		
		} 
	#endif 
		
		nwk_getChannel(&curChan);
	
	    for (i=0; i<NWK_FREQ_TBL_SIZE_AGILITY && !done; i++){
			if (i!=curChan.logicalChan){
				
				//nwk_setChannel(i);
				nextChan.logicalChan = i;
				SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &nextChan);		
				RadioFh.ChannelRF=i;
	
				ioctl_info.send.addr = (addr_t *)&dst_address;
				ioctl_info.send.msg  = msg;
				ioctl_info.send.len  = sizeof(msg);
				ioctl_info.send.port = SMPL_PORT_PING;
				ioctl_info.send.forced = 0;

				/* fill in msg */
				msg[PB_REQ_OS] = PING_REQ_PING;
				msg[PB_TID_OS] = sTid;
	
				SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &ioctl_info.send);
				
				ioctl_info.recv.port = SMPL_PORT_PING;
				ioctl_info.recv.msg  = msg;
				ioctl_info.recv.addr = 0;
							
				NWK_CHECK_FOR_SETRX(radioState);
				
    			count = (RX_PING_TOUT*1000) / MRFI_MAX_DELAY_US;	
    			do{
					rc=SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &ioctl_info.recv);
      				if (rc != SMPL_SUCCESS)
						BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
					else
        				break;	
					os_tsk_pass();
    			} while (count--);
				
				NWK_CHECK_FOR_RESTORE_STATE(radioState);
	
				if (rc==SMPL_SUCCESS){
					sTid++;	//Signal to ping handler
					done   = 1;
				}
			}
		}
		
		if (done){
			return TRUE;
		}else{
			//nwk_setChannel(curChan.logicalChan);
			SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &curChan);	
			RadioFh.ChannelRF=curChan.logicalChan;
			return FALSE;
		}
	}
	
	static void set_nextchannel(void){
		freqEntry_t freq;
	
		os_mut_wait(send_mtx,0xFFFF);
	
		if (++sChannel >= NWK_FREQ_TBL_SIZE_AGILITY)	
			sChannel = 0;		
		freq.logicalChan = sChannel;
		SMPL_Ioctl(IOCTL_OBJ_FREQ, IOCTL_ACT_SET, &freq);
		RadioFh.ChannelRF=sChannel;		
	
		os_mut_release(send_mtx);
	}
	
	void checkChangeChannel(void){
		/* quit if we serviced an app RECV frame (in 2 seconds)*/
		if (sPeerFrameTick > TIMER_SCA){	//Ticks Overflow happened!!
			if (((0xFFFFFFFF - sPeerFrameTick) + TIMER_SCA)<TICKS_BEFORE_SENSE_CHNL) 
				return;		
		}else{
			if ((TIMER_SCA - sPeerFrameTick)<TICKS_BEFORE_SENSE_CHNL)
			 	return; 		
		}
	
	#ifdef SENSE_BEFORE_CHANGE
		int8_t inARow = 0;
		uint8_t i;
		memset(sSample, 0x0, SSIZE);
		for (i=0; i<SSIZE; ++i){	
			SMPL_Ioctl(IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RSSI, (void *)&sSample[i]);
			if (sSample[i] > INTERFERNCE_THRESHOLD_DBM){
			  if (++inARow == IN_A_ROW){
					set_nextchannel();
			    	break;
			  }
			}
			else{
			  inARow = 0;
			}
		
			BSP_DELAY_USECS(200);
		}
	#else //Force change channel
		set_nextchannel();
	#endif 
		//Update time of change channel
		sPeerFrameTick = TIMER_SCA;	
	  	return;
	}
#else
	static u16 STI_scanChannels(addr_t dst_address){}
	static void set_nextchannel(void){}
	void checkChangeChannel(void){}
#endif 