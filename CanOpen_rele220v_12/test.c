#include <stdint.h> 
#include <stdio.h>

	
#define  SDO_Transmit 0x580
#define  SDO_Receive  0x600

#define  PD1_Transmit 0x180
#define  PD2_Transmit 0x280 
#define  PD3_Transmit 0x380
#define  PD4_Transmit 0x480 
	
#define  PD1_Receive 0x200
#define  PD2_Receive 0x300
#define  PD3_Receive 0x400
#define  PD4_Receive 0x500

#define CAN_Emergency 0x80

#define CanOpen_sync	0x80
#define CanOpen_TimeStamp	0x100
#define CanOpen_NMT_control	0x00
#define CanOpen_Heartbeat	0x700

/* NMT command  000_DLC_CS_NodeID
 * DLC = 2 byte
 * CS  =  command (1 byte)
 * NodeID = 01h-7fh address
*/	
#define Start_Remote_Node	0x01
#define Stop_Remote_Node	0x02	
#define Enter_Pre_Operational	0x80
#define Reset_Application	0x81
#define Reset_Communication	0x82	
	
#define  STOPPED		0x04
#define  OPERATIONAL	0x05
#define  PRE_OPERATIONAL	0x7F	

// deftype	
	
#define boolean		0x01
#define INT8		0x02
#define INT16		0x03
#define INT32		0x04
#define UINT8		0x05
#define UINT16		0x06
#define UINT32		0x07
#define REAL32		0x08
#define VISIBLE_STRING	0x09
#define OCTET_STRING	0x0A
#define UNICODE_STRING	0x0B
#define TIME_OF_DAY	0x0C
#define TIME_DIFFERENCE	0x0D

#define DOMAIN		0x0F
#define INT24		0x10
#define REAL64		0x11
#define INT40		0x12
#define INT48		0x13
#define INT56		0x14
#define INT64		0x15
#define UINT24		0x16

#define UINT40		0x18
#define UINT48		0x19
#define UINT56		0x1A
#define UINT64		0x1B

#define PDO_COMM	0x20
#define PDO_MAPPING 0x21
#define SDO_PARAMETER	0x22
#define IDENTITY		0x23
	
/*Object Dictionary TYPE Definitions cia301*/
#define OD_NULL 		0
#define OD_DOMAIN 		2
#define OD_RECORD 		9
#define OD_DEFSTRUCT	6
#define OD_DEFTYPE 	5
#define OD_VAR 		7	
#define OD_ARRAY 		8

/*Attribute*/	
#define _CONST	0
#define RO	0b01
#define WO	0b10
#define RW	0b11	
	
/**/
	
#define PDO_DISABLED	0x80000000 // 0b1000 0000 0000 0000
#define PDO_is_transmit	0x80 	   //0b1000 0000
#define PDO_calculate_n	0x700 // 0b111 0000 0000
#define PDO_mask_addr	0x7FF // 0b0111 1111 1111
	
	
/**/	
#define OK_OD_read8		0x4F
#define OK_OD_read16	0x4B
#define OK_OD_read32	0x43
	
#define OD_save_u8		0x2F
#define OD_save_u16		0x2B
#define OD_save_u32		0x23

#define OK_OD_save		0x60
#define ERORR_command	0x80		
	

#define ERORR_no_save			0x06010002
#define ERORR_no_correct_data	0x06090031

#define ERROR_sub_index		0x06090011
#define ERROR_no_object		0x06020000
#define ERROR_no_open		0x06010000
#define ERROR_ALL_OD_table	0x08000000

	

//////////////////////////    OD_TABLE      //////////////////////////

/* OD_sub_index*/
struct OD_subindex{
	
uint8_t	object;	/* VAR,DEFTYPE,ARRAY,DEFSTRUCT,DOMAIN,RECORD*/
uint8_t	type;	/*_BOOLEAN,INTENGER8 ....UNSIGNED32,PDO_MAP.. etc. */
uint8_t	attr;	/* RO,RW,WO ...*/
uint8_t	n_data; /* sizeOf()*/
void*	data;	/*(void*) INTENGER8 ...UNSIGNED32 *, void (*func)(*,*)*/
};

/* OD_index*/
struct OD_table{
uint16_t			index;
struct OD_subindex* subindex;
};


/////////////////////    PDO COMMUNICATION   ////////////////////////

#define MAX_OBJECT_PDO 4
#define MAX_ELEMENT_PDO 8

//mapping receive 1600h & transmit 1A00h

union map_data{
	struct{
	uint8_t  nbit; 
	uint8_t  sub_index;
	uint16_t index;
	}		 		parameter;
	uint32_t 		data32;
};



struct PDO_mapping{	
uint8_t  			sub_index;
union map_data 		data[MAX_ELEMENT_PDO];
/*hidden quick access to objects*/
struct OD_subindex* object[MAX_ELEMENT_PDO];
uint8_t 			nbit[MAX_ELEMENT_PDO]; 
};
	
//RPDO 1400h , TPDO 1800h
struct PDO_communication{
	
uint8_t		sub_index ;
uint32_t	cob_id ;
uint8_t		Transmission_type;
uint16_t	Inhibit_time;
uint8_t		none; 	
uint16_t	event_timer;
};


struct PDO_object{
	union {
		struct{
		unsigned init: 1;
		unsigned message:1;
		}bits;
		uint8_t byte;
	}					        status;
	struct PDO_communication	comm;
	struct PDO_mapping			map;
	uint8_t 					buffer[8];
	};


////////////////////   SDO COMMUNICATION    ///////////////////

//SDO block 1200
struct SDO_block{
uint8_t				status;
uint8_t				command;
uint16_t			index; 
uint8_t				sub_index;
struct OD_subindex*	object;
uint8_t*			data;
uint8_t				n_byte;	
};

// RxSDO 580h +id , TxSDO 600 + id 

struct SDO_communication{
	
uint8_t		sub_index;
uint32_t	cob_id_client;
uint32_t	cob_id_server;
uint8_t		node_id;

/*hidden quick access to objects*/
struct
SDO_block* sdo_upload;
uint8_t 	n_sdo_upload;
struct 
SDO_block* sdo_download;
uint8_t		n_sdo_download;
};


////////////////////// NODE CAN ////////////////////////

/*Node CAN*/
struct CAN_node{

uint8_t mode;	/*pre-orintal etc.*/

/*search OD_table */	
struct OD_table*	first_od_table;
struct OD_table*	current_od_table;
struct OD_subindex*	current_subindex;
uint32_t 			error_search;	
uint16_t			search_index; 
uint8_t				search_sub_index;
struct OD_table*	
(*Search_index)(struct CAN_node* node);

/*SDO protocol segment*/	

struct
SDO_communication* sdo;
uint8_t 		n_sdo;

/*PDO protocol */

/*RPDO*/
struct PDO_object*	rxpdo[MAX_OBJECT_PDO];
uint8_t				n_rxpdo;

/*TPDO*/
struct PDO_object*	txpdo[MAX_OBJECT_PDO];
uint8_t				n_txpdo;


};

//--------------------------------------------------------------------------------

/* Search OD_table and return struct OD_table *
 * complete simple list iteration */

struct OD_table *OD_search_index(struct OD_table *tab,uint16_t index){	
 
 uint8_t err = 1;	

	while  (tab->index <= index && tab->index !=0 ){ 		
		if (tab->index == index){err = 0;break;};
		tab++;}
	if (err) tab = NULL;
		
 return tab;
 }

/* Search OD_subindex and return struct OD_subindex *
 * complete simple list iteration */
 
struct OD_subindex *OD_search_subindex(struct OD_table *index,uint8_t subindex){

  struct  OD_subindex *sub = index->subindex;		

      if(sub->object == OD_ARRAY || sub->object == OD_DEFSTRUCT ||sub->object == OD_RECORD ){
		if(*((uint8_t*)sub->data) >= subindex){sub+=subindex;}else{sub = NULL;};
	  };
	  
return sub;
};

/* Search OD_subindex and return struct OD_subindex */

struct OD_subindex* OD_search_object(struct OD_table *tab,uint16_t index, uint16_t subindex, uint32_t *error){
	
	struct OD_subindex* sub = NULL;
	uint32_t err = 0;

	tab = OD_search_index(tab,index);
	if (tab != NULL){ 	
		if (tab->subindex != NULL && tab->index != 0){	
			sub = OD_search_subindex(tab,subindex);
			if (sub == NULL){err = ERROR_sub_index;};
		}else{err = ERROR_no_object;};
	}else{err = ERROR_no_object;}
		
	if (error != NULL) *error = err;	
		
return sub;	
};

/* litlle-endian*/

#define DEFMAXSIZE 4

//-----------------------------------------------
uint8_t OD_read_deftype_element(struct OD_subindex *sub,void* buf,uint8_t len){

	for(uint8_t i = 0; i< len; i++){	
	     *((uint8_t*)buf+i) = (i < sub->n_data)? *((uint8_t*)sub->data +i) : 0;	
	}
};
//----------------------------------------------------------
uint8_t OD_save_deftype_element(struct OD_subindex *sub,void* buf){
	
	for(uint8_t i = 0; i < (sub->n_data); i++){
	   *((uint8_t*)sub->data +i) = *((uint8_t*)buf+i);	
	};
};

//-----------------------------------------------------------

uint8_t PDO_init_object(struct CAN_node *node,struct PDO_object *pdo){

	uint8_t err = 0;
		
	if (!(pdo->comm.cob_id & PDO_DISABLED)) return 1;
	if (node->mode != PRE_OPERATIONAL) return 1;
	
	for (uint8_t i=0;i < pdo->map.sub_index ;i++){
		
		 pdo->map.object[i] = OD_search_object(node->first_od_table,
											pdo->map.data[i].parameter.index,
											pdo->map.data[i].parameter.sub_index,
											&node->error_search); 
		if (pdo->map.object[i] == NULL) err=1; 
	};
return err;};


/* status  
 * bit0 - init
 * bit1 - message
 * 
 *  -> */

uint8_t rxPDOwork (struct CAN_node *node,struct PDO_object *pdo){
	
	
	/* if disаbled */
	if (pdo->comm.cob_id & PDO_DISABLED){
	
	 if (pdo->status.bits.init != 1) pdo->status.byte = 0x01;
			
	}else{
	
	/* if enabled and status init = 1 -> init pdo*/
		if (pdo->status.bits.init == 1){
		
			if(PDO_init_object(node,pdo)){		 //error init	
				pdo->comm.cob_id |= PDO_DISABLED;//set bit PDO_DISABLED 
				pdo->status.byte = 0x01;	
			}else{pdo->status.bits.init = 0;};   // i'ts ok. 
		};
		
	/* if enabled and message = 1 -> */	
		
		if (pdo->status.bits.message == 1){
			
			
			
			pdo->status.bits.message == 0;
		};
								
	};
};










//-----------------------------------------------------------------------------------

/*Search_CAN_Node index ->node->current_od_table*/
struct OD_table *CAN_node_search_index(struct CAN_node *node){
return node->current_od_table = OD_search_index(node->first_od_table,node->search_index);
}
//Search_CAN_Node subindex->node->current_subindex//
struct OD_subindex *CAN_node_search_subindex(struct CAN_node *node){
return node->current_subindex = OD_search_subindex(node->current_od_table,node->search_sub_index);
}

/* Search OD_index&subindex and return struct OD_subindex */
struct OD_subindex * OD_Search_table_object(struct CAN_node *node){
	node->current_od_table = node->first_od_table;
	return node->current_subindex = OD_search_object(  
		node->current_od_table,
		node->search_index,
		node->search_sub_index,
		&node->error_search);
};
//-----------------------------------------------------------------------------------




/////////////////////////////////////////Rele CAN cia 401 ////////////////////////////////////////

/*
N1000_DeviceType
*  
Bit 0..15     Device profile number = 401 d
Bit 16  1 b = digital input(s) implemented   0 b = not implemented
Bit 17  1 b = digital output(s) implemented  0 b = not implemented
Bit 18  1 b = analogue input(s) implemented  0 b = not implemented
Bit 19  1 b = analogue output(s) implemented 0 b = not implemented
Bit 20 to Bit 22 Reserved 0
Bit 23	1 b = Device-specific PDO mapping is supported 0 b = Pre-defined, generic PDO mapping is supported (see 6.2.4 to 6.2.11)
Bit 24..31 
	00h	No specific function-
	01h	JoystickAppendix A
	02h	JoystickAppendix A
	03h	JoystickAppendix A
	04 h to FF h Reserved
NOTE
Any combination of digital/analogue, inputs and outputs is allowed; one of the bits 16 to 19
*/

const uint32_t N1000_DeviceType;

/*
 6.2.3 Object 1029 h : Error behavior
The object specifies to which state an I/O module shall be set, when a communication error,
output error or input error is detected. The following values are defined, all others are
reserved:
00 h = transit to NMT pre-operational (only if the current NMT state is operational) state
01 h = remain in current NMT state
02 h = transit to NMT stopped state
Sub-Index 02h
Description Output error
Sub-Index 03h
Description Input error
 */
uint8_t N1029_Error_behavior_Output;
uint8_t N1029_Error_behavior_Input;


/*
6.2.4 RPDO 1 (digital outputs)
This RPDO receives the values of up to 64 digital outputs.
INDEX 1400 h
RxPDO1  communication parameter
*Conditional: Mandatory, 
if M-bit  in object 1000 h is set to 0b
   bit 17 in object 1000 h is set to 1b

INDEX1600 h
RxPDO1  mapping parameter
-
Sub-Index 00h
Value range   01 h to 08 h
Default value 01 h to 08 h
-
Sub-Index 01h
Description1 st application object
Value rangeSee /CiA301/
Default value 6200 01 08 h
.....
Sub-Index 08 h
Value rangeSee /CiA301/
Default value 6200 08 08 h

*/
struct PDO_object 
RxPDO1 = {
	
	.status = 0,
	.comm = {
		.sub_index = 2,
		.cob_id = 0x80000000,
		.Transmission_type = 0xFF,
		},
	.map = {	
		.sub_index = 2,
		.data ={
				{.data32 = 0x62000108},
				{.data32 = 0x62000208},
				{.data32 = 0x00000000},
				{.data32 = 0x00000000},
		},
	},
		
},

/*RxPDO2 (analogue outputs) 6411 01 10 h
  RxPDO3 (additional analogue outputs) 6411 05 10 h
  RxPDO4 (additional analogue outputs)
	  This RPDO receives asynchronously the 16-bit values of maximum 4 analogue outputs to the
  module. The default transmission type shall be 255.  
 */


/*
 TPDO 1 (digital inputs)
 
 INDEX1800 h
 Conditional: Mandatory, 
if M-bit in object 1000 h is set to 0 b
  bit 16 in object 1000 h is set to 1 b
  * 
Sub-Index 0
Value range 02h to 05h
Sub-Index 01h
{0000 0180 h, 4000 0180 h } + node-ID
Sub-Index 02h
Transmission type default value 255d
Sub-Index 03h
Inhibit time default value 0
Sub-Index 05h
Event timer  default value 0
 
INDEX 1A00h
Sub-Index 00h
Value range   01 h to 08 h
Default value 01 h to 08 h
Sub-Index 01h
Description1 st application object
Value rangeSee /CiA301/
Default value 6000 01 08 h
.....
Sub-Index 08 h
Value rangeSee /CiA301/
Default value 6000 08 08 h


 */

TxPDO1 = {
	
	.status = 0,
	.comm = {
		.sub_index = 5,
		.cob_id = 0x80000000,
		.Transmission_type = 0xFF,
		.Inhibit_time = 0, 	
		.event_timer = 0,
		},
	.map = {	
		.sub_index = 1,
		.data ={
				{.data32 = 0x60000108},
				{.data32 = 0x00000000},
				{.data32 = 0x00000000},
				{.data32 = 0x00000000},
		},
	},
		
};		
/*
TxPDO2 (analogue inputs)
TxPDO3 (additional analogue inputs)
TxPDO4 (additional analogue inputs)
This TPDO transmits event-driven the 16-bit values of maximum 4 analogue inputs. By default
the interrupt source (6423 h object) shall be disabled. If one of the mapped analogue input
changes its value and 6423 h object is enabled, the PDO is transmitted immediately. If an
analogue interrupt condition is enabled, the PDO is transmitted only if this interrupt condition
is fulfilled. If more than one interrupt condition is enabled; the PDO is transmitted if one of
these conditions is fulfilled.
*/

/*For TxPDO	
Object 6000 h : Read input 8-bit
Object 6002 h : Polarity input 8-bit  1 = input inverted 0 = input not inverted
Object 6003 h : Filter constant input 8-bit  1 = enabled 0 = disabled
Object 6005 h : Global interrupt enable digital 8-bit
				TRUE = global interrupt enabled
				FALSE = global interrupt disabled
Object 6006 h : Interrupt mask any change 8-bit		  
Object 6007 h : Interrupt mask low-to-high 8-bit
				1 = interrupt enabled
				0 = interrupt disabled
Object 6008 h : Interrupt mask high-to-low 8-bit
				1 = interrupt enabled
				0 = interrupt disabled
				  
Input bit 1 to 128 to read input bit 897 to 1024
---
Object 6020 h to 6027 h : Read input 
Object 6030 h to 6037 h : Polarity 
Object 6038 h to 603Fh  : Filter constant 
Object 6050 h to 6057 h : Interrupt mask input bit any change 
Object 6060 h to 6067 h : Interrupt mask input low-to-high 
Object 6070 h to 6077 h : Interrupt mask input high-to-low 

16 bit 
Object 6100h Read input 16-bit ....  6108h  See.. 8bit
32 bit
Object 6100h Read input 32-bit ....  6108h  See.. 8bit

*/

uint8_t 
gpio_input,
gpio_input_old;


/*
RxPDO
Object 6200 h : Write output 8-bit
Object 6202 h : Change polarity output 8-bit
Object 6206 h : Error mode output 8-bit
 1 = output value shall take the pre-defined condition specified in 6207 h object
 0 = output value shall be kept if an error occurs
Object 6207 h : Error value output 8-bit
 0 = output shall be set to ‘0’ in case of fault, if 6206 h object is enabled
 1 = output shall be set to ‘1’ in case of fault, if 6206 h object is enabled
Object 6208 h : Filter mask output 8-bit
 1 = output shall be set to the received output value
 0 = don’t care, the received output value is neglected for the appropriated output channel, the
     old output value shall be kep

16 bit 
Object 6300h Read input 16-bit .... 6308 See.. 8bit

32 bit
Object 6320h Read input 32-bit .... 6320 See.. 8bit

Object 6220 h to 6227 h : Write output bit 1 to 128 to write output bit 897 to 1024
Object 6240 h to 6247 h : Change polarity output bit
Object 6250 h to 6257 h : Error mode output lines
Object 6260 h to 6267 h : Error value output bit
Object 6270 h to 6277 h : Filter mask output bit
*/

union{
/*	
	struct{
		unsigned b1: 1;
		unsigned b2: 1;
		unsigned b3: 1;
		unsigned b4: 1;
		unsigned b5: 1;
		unsigned b6: 1;
		unsigned b7: 1;
		unsigned b8: 1;
		unsigned b9: 1;
		unsigned b10: 1;
		unsigned b11: 1;
		unsigned b12: 1;			
		}	 pin; */
	struct{
	uint8_t B;
	uint8_t C;
	}		 port8bit;
	uint16_t port16bit; 		
	}nGpio;








uint32_t N1005_COB_ID_SYNCMessage;
uint32_t N1006_communicationCyclePeriod;
uint32_t N1007_synchronousWindowLength;
uint32_t N1012_COB_IDTimeStampObject;
uint32_t N1014_COB_ID_EMCY;
uint16_t N1015_inhibitTimeEMCY;
uint8_t  N1016_consumerHeartbeatTime_sub0;
uint32_t N1016_consumerHeartbeatTime[1];
uint16_t N1017_producerHeartbeatTime;
struct {
        uint8_t  sub_index;
        uint32_t vendor_ID;
        uint32_t productCode;
        uint32_t revisionNumber;
        uint32_t serialNumber;
    }N1018_Identity;


/*PDO_COMMUNICTION
uint8_t		sub_index ;
uint32_t	cob_id ;
uint8_t		Transmission_type;
uint16_t	Inhibit_time;
uint8_t		none; 	
uint16_t	event_timer;
};
 
 */
	
struct OD_subindex 



N1000 ={OD_VAR,	  UINT8,	RO,sizeof(N1000_DeviceType),(void*)&N1000_DeviceType},

N1400[]={
	{OD_DEFSTRUCT,UINT8,	RO,sizeof(RxPDO1.comm.sub_index),(void*)&RxPDO1.comm.sub_index},
	{OD_VAR,	  UINT32,	RW,sizeof(RxPDO1.comm.cob_id),(void*)&RxPDO1.comm.cob_id},				
	{OD_VAR,	  UINT8,	RW,sizeof(RxPDO1.comm.Transmission_type),(void*)&RxPDO1.comm.Transmission_type},
},
N1600[]={
	{OD_ARRAY,	  UINT8,	RO,sizeof(RxPDO1.map.sub_index),(void*)&RxPDO1.map.sub_index},
	{OD_VAR,	  UINT32,	RW,4,(void*)&RxPDO1.map.data[0].data32},				
	{OD_VAR,	  UINT32,	RW,4,(void*)&RxPDO1.map.data[1].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&RxPDO1.map.data[2].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&RxPDO1.map.data[3].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&RxPDO1.map.data[4].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&RxPDO1.map.data[5].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&RxPDO1.map.data[6].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&RxPDO1.map.data[7].data32},
},

N1800[]={
	{OD_DEFSTRUCT,UINT8,	RO,sizeof(TxPDO1.comm.sub_index),		(void*)&TxPDO1.comm.sub_index},
	{OD_VAR,	  UINT32,	RW,sizeof(TxPDO1.comm.cob_id),	 		(void*)&TxPDO1.comm.cob_id},				
	{OD_VAR,	  UINT8,	RW,sizeof(TxPDO1.comm.Transmission_type),(void*)&TxPDO1.comm.Transmission_type},
	{OD_VAR,	  UINT16,	RW,sizeof(TxPDO1.comm.Inhibit_time),	(void*)&TxPDO1.comm.Inhibit_time},
	{OD_VAR,	  UINT8,	RO,sizeof(TxPDO1.comm.none),			(void*)&TxPDO1.comm.none},
	{OD_VAR,	  UINT16,	RW,sizeof(TxPDO1.comm.event_timer),		(void*)&TxPDO1.comm.event_timer},
},
N1A00[]={
	{OD_ARRAY,	  UINT8,	RO,sizeof(TxPDO1.map.sub_index),(void*)&TxPDO1.map.sub_index},
	{OD_VAR,	  UINT32,	RW,4,(void*)&TxPDO1.map.data[0].data32},				
	{OD_VAR,	  UINT32,	RW,4,(void*)&TxPDO1.map.data[1].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&TxPDO1.map.data[2].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&TxPDO1.map.data[3].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&TxPDO1.map.data[4].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&TxPDO1.map.data[5].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&TxPDO1.map.data[6].data32},
	{OD_VAR,	  UINT32,	RW,4,(void*)&TxPDO1.map.data[7].data32},
};




struct OD_table MAP_Block_Rele[]={
{0x1000,&N1000},
{0x1400,N1400},
{0x1600,N1600},
{0x1800,N1800},
{0x1A00,N1A00},



{0x0000,NULL},
};



struct CAN_node  BlockRele12 ={
	
	.mode = 0,
	.first_od_table = MAP_Block_Rele,
	.current_od_table = MAP_Block_Rele,
	
	.rxpdo = {&RxPDO1},
	.n_rxpdo = 1,
	.txpdo = {&TxPDO1},
	.n_txpdo = 1,
	
	
};












void main(){
	
	
	
nGpio.port16bit = 0x00fe;	
	printf(" code --%x\n",nGpio.port16bit);
	printf(" code --%x\n",nGpio.port8bit.B);
	printf(" code --%x\n",nGpio.port8bit.C);
	printf(" code --%x\n",nGpio.pin.b1);
	

	
/*

BlockRele12
struct OD_table *n = Block_RELE.current_od_table;
struct  OD_subindex *s;
uint32_t error_code = 0;
uint8_t buf[6]={0x44,0x33,0x22,0x11};
   
	printf("*n->nsub --%x\n", *((uint8_t*)n->subindex->data));

	n=Block_RELE.current_od_table = OD_search_index (Block_RELE.current_od_table,0x1000);

	if (n != NULL)printf("current ---%x\n", Block_RELE.current_od_table->nsub);
	if (n != NULL){ printf("*n->nsub--%x\n",n->nsub);
					s = OD_search_subindex(n,0);
					if(s!= NULL){printf("*s->data 32bit--%x\n",*((uint32_t*)s->data));}
	
	};

	Block_RELE.current_od_table = Block_RELE.first_od_table;
	
	printf("init_current--%x\n", Block_RELE.current_od_table->nsub);

	n = Block_RELE.current_od_table;

	printf("init_*n----%x\n", n->nsub);

	s = OD_search_element(n,0x1400,1,&error_code);

	if(s != NULL)printf("*s->data 32bit--%x\n",*((uint32_t*)s->data));

	printf("first--%x\n", Block_RELE.first_od_table->nsub);
	if (n != NULL)printf("current ---%x\n", Block_RELE.current_od_table->nsub);
	if (n != NULL)printf("*n->nsub--%x\n",n->nsub);
	
	printf("error code --%x\n",error_code);
	
	
	OD_save_deftype_element(s,buf);
	
	printf("*s->data 32bit--%x\n",*((uint32_t*)s->data));

	buf[0] = 0x55; buf[1] = 0xAA;
	printf("buf 32bit--%x\n",*((uint32_t*)buf));

	OD_read_deftype_element(s,buf,4);
	printf("read buf 32bit--%x\n",*((uint32_t*)buf));

	s = OD_search_element(n,0x1400,2,&error_code);
	OD_read_deftype_element(s,buf,4);
	printf("read8bit buf 32bit--%x\n",*((uint32_t*)buf));
	
	buf[0] = 0x55; buf[1] = 0xAA;
	OD_save_deftype_element(s,buf);
	printf("save 8bit buf--%x\n", Id_Type);






printf("0--%x\n",ob.bit.b0);
printf("1--%x\n",ob.bit.b1);
printf("2--%x\n",ob.bit.b2);
printf("3--%x\n",ob.bit.b3);
printf("4--%x\n",ob.bit.b4);
printf("5--%x\n",ob.bit.b5);
printf("6--%x\n",ob.bit.b6);
printf("7--%x\n",);

//printf("void -%x\n",(uint32_t*)ob.data);

ob.data = (void*)&ob.u32;

printf("u8---%x\n",ob.u8 );
printf("u16--%x\n",ob.u16 );
printf("u32-%x\n",ob.u32 );
printf("void -%lx\n",*(uint64_t*)ob.data);

*/
};


