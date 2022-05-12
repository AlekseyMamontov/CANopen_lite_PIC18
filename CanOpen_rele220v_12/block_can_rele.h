/* 
 * File:   block_can__rele.h
 * Author: aleksey
 *
 * Created on January 23, 2022, 9:45 PM
 */

#ifndef BLOCK_CAN__RELE_H
#define	BLOCK_CAN__RELE_H

#include "mcc_generated_files/ecan.h"


#ifdef	__cplusplus
extern "C" {
#endif

	
#define  SDO_Transmit 0x580
#define  SDO_Receive   0x600

#define  PD1_Transmit 0x180 
#define  PD1_Receive   0x200

#define  PD2_Transmit 0x280 
#define  PD2_Receive   0x300

#define  PD3_Transmit 0x380 
#define  PD3_Receive   0x400

#define  PD4_Transmit 0x480 
#define  PD4_Receive   0x500

#define CAN_Emergency  0x80

#define CanOpen_sync	   0x80
#define CanOpen_TimeStamp        0x100
#define CanOpen_NMT_control       0x00
#define CanOpen_Heartbeat           0x700

/* NMT command  000_DLC_CS_NodeID
 * DLC = 2 byte
 * CS  =  command (1 byte)
 * NodeID = 01h-7fh address
*/	
#define Start_Remote_Node     0x01
#define Stop_Remote_Node      0x02	
#define Enter_Pre_Operational 0x80
#define Reset_Application         0x81
#define Reset_Communication 0x82	
	
#define  STOPPED                 0x04
#define  OPERATIONAL         0x05
#define  PRE_OPERATIONAL 0x7F	

#define _BOOLEAN 01	
#define UNSIGNED8 05
#define UNSIGNED16 06
#define UNSIGNED32  07

/*Object Dictionary Object Definitions cia301*/
/*function*/
#define OD_DOMAIN 2
/**/
#define OD_DEFTYPE 5
	
/*struct example  21 PDO mapping*/
#define OD_DEFSTRUCT 6
	
/*single uint8 ...32, boolean*/
#define OD_VAR 7
	
/*massiv  sub-index uint8_t, (u16 or u32) */
#define OD_ARRAY 8
struct OD_array_n{
	
uint8_t sub_index;
void *	  value;

};	

/*massiv  sub-index uint8_t and */
#define OD_RECORD 9	
	

union OD_data {
uint8_t     u8;
uint16_t u16;
uint32_t u32;
void *  value;
void (*func)(void *, void *);
};


struct OD_table{
// 1000..	
uint16_t	index;
//char       *name;
uint8_t      object;
uint8_t	type;
//union OD_data value;
void*	data;
uint8_t	attr;
//struct OD_table *next;
//struct OD_table *prev;
};
/*
attr
#define RW - Read and write access
#define WO- Write only
#define RO - Read only
#define CONST- Read only, Data is constant
 
 */
// for OD_table .... .data
void(*OD_functoin)(void*,void*);

//receive 1600h & transmit 1A00h
#define PDO_MAPPING 21
struct PDO_mapping{
	
uint8_t     sub_index ;
uint32_t   *object;

};
//RPDO 1400h , TPDO 1800h
#define PDO_COMM  20
struct PDO_comm{
	
uint8_t	sub_index ;
uint32_t	cob_id ;
uint8_t	Transmission_type;
uint16_t	Inhibit_time;
uint8_t	none;
uint16_t	event_timer;

};

// RxSDO 580h +id , TxSDO 600 + id 
#define SDO_PARAMETER 22
struct SDO_comm{
	
uint8_t	sub_index;
uint32_t	cob_id_client;
uint32_t	cob_id_server;
uint8_t	node_id;

};
#define IDENTITY 23
struct OD_IDENTITY{
uint8_t sub_index;
uint32_t	*value;
};


	
//  Set relays CanOpen _PD1
	
 void Set_Rele(uint16_t data){  
        LATB = (data&0b11111111);
        if(data&0b000100000000){Rele9_LAT = 1;  }else{Rele9_LAT = 0;};
        if(data&0b001000000000){Rele10_LAT = 1;}else{Rele10_LAT = 0;};
        if(data&0b010000000000){Rele11_LAT = 1;}else{Rele11_LAT = 0;};
        if(data&0b100000000000){Rele12_LAT = 1;}else{Rele12_LAT = 0;};
 };	


// read  address (Node_ID)   74h165 
 
uint8_t Read_addr_CAN(){
   uint8_t addr = 0;
   uint8_t bit_addr =0;
   
     PL_165_SetHigh() ;
     SCK_165_SetHigh();
     
     // zachelka addr.
     
     PL_165_SetLow() ;
     __delay_ms(5);
     PL_165_SetHigh() ;
     
     // cycle 8 bit
     
     for(uint8_t b=0;b<8;b++){
         
         bit_addr = Data_addr_GetValue()&1;
         bit_addr <<= b;
         addr |=bit_addr; 
         SCK_165_SetLow();
         __delay_ms(5);
         SCK_165_SetHigh();
     };
     return addr;
 };	
	
 // Can_init_modul 
 
static void Default_Handler(void){};
 
void CanOpen_ECAN_Initialize(uint8_t id)
{	
   uint16_t   per_filter;
	
    CANCON = 0x80;
    while (0x80 != (CANSTAT & 0xE0)); // wait until ECAN is in config mode
    
    /* Mode 0 */
    ECANCON = 0x00;
    
    /* Initialize CAN I/O */
    CIOCON = 0x20;
    
    /**
    Mask and Filter definitions
    ........................................................    
    CAN ID		ID Type		Mask				Filter		Buffer    
    ........................................................    
    0x80		SID		Acceptance Mask 0		Filter 0	RXB0
    0x0		SID		Acceptance Mask 0		Filter 1	RXB0
    0x200 +ID	SID		Acceptance Mask 1		Filter 2	RXB1
    0x300 +ID	SID		Acceptance Mask 1		Filter 3	RXB1
    0x600 +ID	SID		Acceptance Mask 1		Filter 4	RXB1
    0x100		SID		Acceptance Mask 1		Filter 5	RXB1
    ........................................................
    */
 
/**    
    Initialize Receive Masks
*/   
    
    RXM0EIDH = 0xFF;
    RXM0EIDL = 0xFF;
    RXM0SIDH = 0xFF;
    RXM0SIDL = 0xE3;
    
    
    RXM1EIDH = 0xFF;
    RXM1EIDL = 0xFF;
    RXM1SIDH = 0x1F;
    RXM1SIDL = 0xE3;
 
    /**
    Initialize Receive Filters
    */ 
    
    // SYNC
    
    RXF0EIDH = 0x00;
    RXF0EIDL = 0x00;
    RXF0SIDH = 0x10;
    RXF0SIDL = 0x00;
    
    // NMT
    
    RXF1EIDH =0x00;
    RXF1EIDL = 0x00;
    RXF1SIDH =0x00;
    RXF1SIDL = 0x00;
    
    // filter cob_ID
    
    per_filter = id; 
    per_filter <<=5;
 
    RXF2EIDH = 0x00;
    RXF2EIDL =  0x00;
    RXF2SIDH = (per_filter&0xFF00)>>8;
    RXF2SIDL  = (per_filter&0x00FF);
    
    //filter PD2
    
    per_filter = (PD2_Receive + id); 
    per_filter <<=5;
    
    RXF3EIDH = 0x00;
    RXF3EIDL = 0x00;
    RXF3SIDH = (per_filter&0xFF00)>>8;
    RXF3SIDL =  (per_filter&0x00FF);
    
    //filter SDO
    
    per_filter =(SDO_Receive + id);
    per_filter <<=5;
    
    RXF4EIDH = 0x00;
    RXF4EIDL =  0x00;
    RXF4SIDH =(per_filter&0xFF00)>>8;
    RXF4SIDL = (per_filter&0x00FF);
    
    // TIME
    
    RXF5EIDH = 0x00;
    RXF5EIDL =  0x00;
    RXF5SIDH = 0x20;
    RXF5SIDL =  0x00;

    /**
    Initialize CAN Timings
    */
    
  	/**
        Baud rate: 500kbps
        System frequency: 8000000
        ECAN clock frequency: 8000000
        Time quanta: 8
        Sample point: 1-1-4-2
        Sample point: 75%
	*/ 
    
    BRGCON1 = 0x00;
    BRGCON2 = 0x98;
    BRGCON3 = 0x01;
    
    ECAN_SetRXB0InterruptHandler(Default_Handler);
    PIR5bits.RXB0IF = 0;
    PIE5bits.RXB0IE = 1;
    
    ECAN_SetRXB1InterruptHandler(Default_Handler);
    PIR5bits.RXB1IF = 0;
    PIE5bits.RXB1IE = 1;
    
    ECAN_SetWakeUpInterruptHandler(Default_Handler);
    PIR5bits.WAKIF = 0;
    PIE5bits.WAKIE = 1;
    
    CANCON = 0x00;
    while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode   
    
}

static uint32_t convertReg2ExtendedCANid(uint8_t tempRXBn_EIDH, uint8_t tempRXBn_EIDL, uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) 
{
    uint32_t returnValue = 0;
    uint32_t ConvertedID = 0;
    uint8_t CAN_standardLo_ID_lo2bits;
    uint8_t CAN_standardLo_ID_hi3bits;

    CAN_standardLo_ID_lo2bits = (uint8_t)(tempRXBn_SIDL & 0x03);
    CAN_standardLo_ID_hi3bits = (uint8_t)(tempRXBn_SIDL >> 5);
    ConvertedID = (uint32_t)(tempRXBn_SIDH << 3);
    ConvertedID = ConvertedID + CAN_standardLo_ID_hi3bits;
    ConvertedID = (ConvertedID << 2);
    ConvertedID = ConvertedID + CAN_standardLo_ID_lo2bits;
    ConvertedID = (ConvertedID << 8);
    ConvertedID = ConvertedID + tempRXBn_EIDH;
    ConvertedID = (ConvertedID << 8);
    ConvertedID = ConvertedID + tempRXBn_EIDL;
    returnValue = ConvertedID;    
    return (returnValue);
}

static uint32_t convertReg2StandardCANid(uint8_t tempRXBn_SIDH, uint8_t tempRXBn_SIDL) 
{
    uint32_t returnValue = 0;
    uint32_t ConvertedID;
    //if standard message (11 bits)
    //EIDH = 0 + EIDL = 0 + SIDH + upper three bits SIDL (3rd bit needs to be clear)
    //1111 1111 111
    ConvertedID = (uint32_t)(tempRXBn_SIDH << 3);
    ConvertedID = ConvertedID + (uint32_t)(tempRXBn_SIDL >> 5);
    returnValue = ConvertedID;
    return (returnValue);
}

static void convertCANid2Reg(uint32_t tempPassedInID, uint8_t canIdType, uint8_t *passedInEIDH, uint8_t *passedInEIDL, uint8_t *passedInSIDH, uint8_t *passedInSIDL) 
{
    uint8_t wipSIDL = 0;

    if (canIdType == dEXTENDED_CAN_MSG_ID_2_0B) {

        //EIDL
        *passedInEIDL = 0xFF & tempPassedInID; //CAN_extendedLo_ID_TX1 = &HFF And CAN_UserEnter_ID_TX1
        tempPassedInID = tempPassedInID >> 8; //CAN_UserEnter_ID_TX1 = CAN_UserEnter_ID_TX1 >> 8

        //EIDH
        *passedInEIDH = 0xFF & tempPassedInID; //CAN_extendedHi_ID_TX1 = &HFF And CAN_UserEnter_ID_TX1
        tempPassedInID = tempPassedInID >> 8; //CAN_UserEnter_ID_TX1 = CAN_UserEnter_ID_TX1 >> 8

        //SIDL
        //push back 5 and or it
        wipSIDL = 0x03 & tempPassedInID;
        tempPassedInID = tempPassedInID << 3; //CAN_UserEnter_ID_TX1 = CAN_UserEnter_ID_TX1 << 3
        wipSIDL = (0xE0 & tempPassedInID) + wipSIDL;
        wipSIDL = (uint8_t)(wipSIDL + 0x08); // TEMP_CAN_standardLo_ID_TX1 = TEMP_CAN_standardLo_ID_TX1 + &H8
        *passedInSIDL = (uint8_t)(0xEB & wipSIDL); //CAN_standardLo_ID_TX1 = &HEB And TEMP_CAN_standardLo_ID_TX1

        //SIDH
        tempPassedInID = tempPassedInID >> 8;
        *passedInSIDH = 0xFF & tempPassedInID;
    } 
    else //(canIdType == dSTANDARD_CAN_MSG_ID_2_0B)
    {
        *passedInEIDH = 0;
        *passedInEIDL = 0;
        tempPassedInID = tempPassedInID << 5;
        *passedInSIDL = 0xFF & tempPassedInID;
        tempPassedInID = tempPassedInID >> 8;
        *passedInSIDH = 0xFF & tempPassedInID;
    }
}
	
uint8_t CanOpen_receive(uCAN_MSG *tempCanMsg, uCAN_MSG *tempCanMsg1)
{
    uint8_t returnValue = 0;

         //check which buffer the CAN message is in
    if (RXB0CONbits.RXFUL != 0) //CheckRXB0
    {
        if ((RXB0SIDL & 0x08) == 0x08) //If Extended Message
        {
            //message is extended
            tempCanMsg->frame.idType = (uint8_t) dEXTENDED_CAN_MSG_ID_2_0B;
            tempCanMsg->frame.id     = convertReg2ExtendedCANid(RXB0EIDH, RXB0EIDL, RXB0SIDH, RXB0SIDL);
        } 
        else 
        {
            //message is standard
            tempCanMsg->frame.idType = (uint8_t) dSTANDARD_CAN_MSG_ID_2_0B;
            tempCanMsg->frame.id     = convertReg2StandardCANid(RXB0SIDH, RXB0SIDL);
        }

        tempCanMsg->frame.dlc   =  RXB0DLC;
        tempCanMsg->frame.data0 = RXB0D0;
        tempCanMsg->frame.data1 = RXB0D1;
        tempCanMsg->frame.data2 = RXB0D2;
        tempCanMsg->frame.data3 = RXB0D3;
        tempCanMsg->frame.data4 = RXB0D4;
        tempCanMsg->frame.data5 = RXB0D5;
        tempCanMsg->frame.data6 = RXB0D6;
        tempCanMsg->frame.data7 = RXB0D7;
        RXB0CONbits.RXFUL = 0;
        returnValue = 1;
    } 
   
    
    if (RXB1CONbits.RXFUL != 0) //CheckRXB1
	    
    {
        if ((RXB1SIDL & 0x08) == 0x08) //If Extended Message
        {
            //message is extended
            tempCanMsg1->frame.idType = (uint8_t) dEXTENDED_CAN_MSG_ID_2_0B;
            tempCanMsg1->frame.id     = convertReg2ExtendedCANid(RXB1EIDH, RXB1EIDL, RXB1SIDH, RXB1SIDL);
        }
        else
        {
            //message is standard
            tempCanMsg1->frame.idType = (uint8_t) dSTANDARD_CAN_MSG_ID_2_0B;
            tempCanMsg1->frame.id     = convertReg2StandardCANid(RXB1SIDH, RXB1SIDL);
        }

        tempCanMsg1->frame.dlc   = RXB1DLC;
        tempCanMsg1->frame.data0 = RXB1D0;
        tempCanMsg1->frame.data1 = RXB1D1;
        tempCanMsg1->frame.data2 = RXB1D2;
        tempCanMsg1->frame.data3 = RXB1D3;
        tempCanMsg1->frame.data4 = RXB1D4;
        tempCanMsg1->frame.data5 = RXB1D5;
        tempCanMsg1->frame.data6 = RXB1D6;
        tempCanMsg1->frame.data7 = RXB1D7;
        RXB1CONbits.RXFUL = 0;
        returnValue = 2;
    }
    return (returnValue);
}

/* Search OD_table and return *element*/
struct OD_table *OD_search_N (struct OD_table *tab, uint16_t num){
uint16_t  cycle = 0;
   while(1){
	if ( (tab+cycle)->index == num || num == 0) {break;}else{cycle++;}
	};
return (tab+cycle);
};
/**/


#define OK_OD_read8      command = 0x4F;break;
#define OK_OD_read16    command = 0x4B;break;
#define OK_OD_read32    command = 0x43;break;

#define ERORR_OD_read                          0x80
#define ERORR_sub_index	0x06090011
#define ERORR_no_object	0x06020000
#define ERRR_ALL_OD_table	0x08000000

void OD_read_data(struct OD_table *tab,uCAN_MSG *msg){

uint32_t dump = 0;

uint8_t 
command  =  ERORR_OD_read,
sub_index = msg->frame.data3,
n_bayt;

struct OD_array_n	*arr;
struct PDO_mapping	*pdo_map;
struct PDO_comm	 *pdo_com;
struct SDO_comm	 *sdo_com;


switch (tab->type){
	
	case OD_VAR:
		
	   if(tab->object == UNSIGNED8)  {dump += *((uint8_t *)(tab->data)); OK_OD_read8}
	   if(tab->object == UNSIGNED16){dump +=*((uint16_t *)(tab->data));OK_OD_read16}
	   if(tab->object == UNSIGNED32){dump =   *((uint32_t *)(tab->data));OK_OD_read32}
	   
	   dump = ERORR_no_object;
	   
	break;

	 case OD_ARRAY:
		
	   arr = (struct OD_array_n *)(tab->data);	
	   
	   if(arr->sub_index < sub_index ){dump = ERORR_sub_index; break;};
	   if(!sub_index){sub_index --;}else{dump += arr->sub_index;OK_OD_read8}
	   
		switch(tab->object){
		case UNSIGNED8:  dump += *(((uint8_t*)(arr->value))+(sub_index));  OK_OD_read8
		case UNSIGNED16:dump +=*(((uint16_t *)(arr->value))+(sub_index));OK_OD_read16
		case UNSIGNED32:dump = *(((uint32_t *)(arr->value))+(sub_index));  OK_OD_read32
		default: dump = ERORR_no_object; break;
		};

	break;
	
	case OD_DEFSTRUCT:
		
		switch(tab->object){
		
		   case PDO_COMM:
			
		   pdo_com = (struct PDO_comm *) tab->data;
		   
		   if (pdo_com->sub_index < sub_index) {ERORR_sub_index;break;};

			switch(sub_index){
			case 0: dump += pdo_com->sub_index; OK_OD_read8
			case 1: dump    = pdo_com->cob_id; OK_OD_read32	
			case 2: dump += pdo_com->Transmission_type; OK_OD_read8
			case 3: dump += pdo_com->Inhibit_time;OK_OD_read16
			case 5: dump += pdo_com->event_timer;OK_OD_read16
			default:  dump = ERORR_no_object; break;
			}; 
		  break;	
		
		  case SDO_PARAMETER:
			
		   sdo_com = (struct SDO_comm *) tab->data;
		   
		   if (sdo_com->sub_index < sub_index) {ERORR_sub_index;break;};
		   
		   	switch(sub_index){
			case 0: dump += sdo_com->sub_index; OK_OD_read8
			case 1: dump    = sdo_com->cob_id_client;OK_OD_read32	
			case 2: dump    = sdo_com->cob_id_server;OK_OD_read32
			case 3: dump += sdo_com->node_id;OK_OD_read8
			default:dump = ERORR_no_object; break;
			}; 
		 break;
		
		case  PDO_MAPPING:

		    pdo_map = (struct PDO_mapping *) tab->data;
		    
		    if (pdo_map->sub_index < sub_index) {ERORR_sub_index;break;};
		    if (!sub_index){sub_index--;}else{dump += pdo_map->sub_index;OK_OD_read8};
		    
		    dump =*(((uint32_t *)(pdo_map->object))+(sub_index)); OK_OD_read32

		case  IDENTITY:
			
		    pdo_map = (struct PDO_mapping *) tab->data;
		    if (pdo_map->sub_index < sub_index) {ERORR_sub_index;break;};
		    if (!sub_index){sub_index--;}else{dump += pdo_map->sub_index;OK_OD_read8};
		    
		    dump =*(((uint32_t *)(pdo_map->object))+(sub_index)); OK_OD_read32
		  
		default:  
		dump = ERORR_no_object; 
		break;
		};
	
	break;
		
	case OD_DEFTYPE:
		
	break;
	
	case  OD_DOMAIN:
	break;
	

	};
msg->frame.data0 = command;
msg->frame.data4 =  dump&0xff;
msg->frame.data5 = (dump&0xff00) >>8;
msg->frame.data6 = (dump&0xff0000)>>16;
msg->frame.data7 = (dump&0xff000000)>>24;
msg->frame.dlc = 8;
CAN_transmit(msg);
};

/**/ 
#define TSTAMP 1
#define RxPDO1 2
#define RxPDO2 3
#define RxPDO3 4
#define RxPDO4 5
#define rxSDO    6
#define NMT_err 7
#define SDO_read 0x40
#define SDO_save 0x20
#define SDO_data_u8   0x0F
#define SDO_data_u16 0x0B
#define SDO_data_u32 0x03

void read_rx_block_OD(uCAN_MSG *msg,struct OD_table *tab){

uint8_t   erorr   = 0;	
uint8_t   comm = ((msg->frame.id) >> 8) & 0x07 ;
uint16_t num = 0;
uint8_t   sub_index;

if (comm== rxSDO){
	
          num= msg->frame.data2;
          num <<= 8;
          num |= msg->frame.data1;
          sub_index = msg->frame.data3;
	  
  switch (msg->frame.data0){
	  
      case SDO_read:
	      
	 tab = OD_search_N( tab, num); 
	 if(tab->index == 0){erorr = 1; break;}
	 //If(tab->object == UNSIGNED8) msg->frame.data4 =   
	 break;
	      
  };
			
		
		
		
	//tab = OD_search_N( tab, 1200);

};



	


};


#ifdef	__cplusplus
}
#endif

#endif	/* BLOCK_CAN__RELE_H */

