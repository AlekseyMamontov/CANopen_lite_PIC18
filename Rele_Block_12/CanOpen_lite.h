/*  Created on 7 03 2023 ?., 20:39
 */

#ifndef CANOPEN_LITE_H
#define	CANOPEN_LITE_H

#ifdef	__cplusplus
extern "C" {
#endif


#include <stdint.h> 
#include <stdio.h>

	
#define  txSDO  0x580
#define  rxSDO  0x600

#define  txPDO1 0x180
#define  txPDO2 0x280 
#define  txPDO3 0x380
#define  txPDO4 0x480 
	
#define  rxPDO1 0x200
#define  rxPDO2 0x300
#define  rxPDO3 0x400
#define  rxPDO4 0x500



// deftype	
	
#define boolean         0x01
#define INT8            0x02
#define INT16           0x03
#define INT32           0x04
#define UINT8           0x05
#define UINT16          0x06
#define UINT32          0x07
#define REAL32          0x08
#define VISIBLE_STRING	0x09
#define OCTET_STRING	0x0A
#define UNICODE_STRING	0x0B
#define TIME_OF_DAY     0x0C
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

#define PDO_COMM        0x20
#define PDO_MAPPING     0x21
#define SDO_PARAMETER	0x22
#define IDENTITY	0x23
	
/*Object Dictionary TYPE Definitions cia301*/
#define OD_NULL 0
/* Large variable amount of data e.g. executable program code */
#define OD_DOMAIN 2
/*A multiple data field object where the data fields may be any combination of
simple variables. Sub-index 0 is of UNSIGNED8 and sub-index 255 is of
UNSIGNED32 and therefore not part of the RECORD data */
#define OD_RECORD 9
/* Defines a new record type e.g. the PDO mapping structure at 21h*/
#define OD_DEFSTRUCT 6
/* Denotes a type definition such as a BOOLEAN, UNSIGNED16, FLOAT and so on*/
#define OD_DEFTYPE 5
/* A single value such as anUNSIGNED8, BOOLEAN, FLOAT,INTEGER16, VISIBLE STRING etc.*/
#define OD_VAR  7	
/* A multiple data field object where each data field is a simple variable of
the SAME basic data type e.g. array of UNSIGNED16 etc. Sub-index 0 is
of UNSIGNED8 and therefore not part of the ARRAY data*/
#define OD_ARRAY 8

/* code sub_index_FF*/ 
    
const uint32_t subindex_FF[]={

#define _OD_NULL 0
0x00000000,
#define UINT8_OD_VAR 1
0x00050007,
#define UINT16_OD_VAR 2
0x00060007,
#define UINT24_OD_VAR 3
0x00160007,
#define UINT32_OD_VAR 4
0x00070007,
#define UINT8_OD_ARRAY 5
0x00050008,
#define UINT16_OD_ARRAY 6
0x00060008,
#define UINT24_OD_ARRAY 7
0x00160008,
#define UINT32_OD_ARRAY 8
0x00070008,
#define PDO_COMM_OD_DEFSTRUCT 9
0x00200006,
#define PDO_MAPPING_OD_DEFSTRUCT 10
0x00210006,
#define SDO_PARAMETER_OD_DEFSTRUCT 11
0x00220006,
#define IDENTITY_DEFSTRUCT 12
0x00230006,
//...pic 256 byte block ram (4x64)..
};
    
    
    
/*Attribute*/	

#define _CONST	0
#define RO      0b0001
#define WO      0b0010
#define RW      0b0011
#define NO_MAP  0b0100


/* structure */
	
#define SDO_request  0
#define MAP_info     1
	
struct Data_Object{
    
    uint8_t     request_type;
    uint8_t     attribute;
    uint8_t     nbit;
    uint8_t     sub_index;
    uint8_t     sub_index_ff;
    void*       data_object;
    void*       rw_object;
    
};

struct OD_Object{
    
    uint16_t index;
    void*     data;
    void (*data_func)(struct Data_Object*);
    
};

struct one_type_array{
    
    uint8_t sub_index;
    void*   array;
    
};

struct record_arr_object{
    
    uint8_t  sub_index;
    uint8_t* nbit;    //arr[subindex] = {0x08,0x20....0x10};
    void**   array; //arr[subindex];*uint8,*uint?_t .....
    
};  

struct string_object{
    
    uint8_t*    text;
    uint8_t*    text_buffer;
    uint8_t     n_byte;; 
    uint8_t     cond_sdo;
    
};



struct map_info{
    
	uint8_t  nbit; 
	uint8_t  sub_index;
	uint16_t index;
    
};

union  map_data{
    
    uint32_t        data32;
    struct map_info info;
      
};

#define MAX_MAP_DATA 8

/*
 union  cond {
	
    char stat;
    
    struct{
	    
    unsigned sync        : 1; // 0x01 sync
    unsigned new_msg     : 1; // 0x02 new message rxPDO & new send txPDO
    unsigned inhibit_time: 1; // 0x04 0 - pause is over & 1 - set start pause(timer)
    unsigned event_timer : 1; // 0x08 0- off event_timer & 1 - set start event_timer
    unsigned event_txpdo : 1; // 0x10 1 - there is a change to send a message
    unsigned init_pdo    : 1; // 0x20 1 - xPDO init   
    unsigned rx_tx       : 1; // 0x40 0 - rxPDO object, 1 - txPDO
    unsigned lock        : 1; // 0x80 1 - Lock
    
    }flag;     
};
due to the fact that the XC8 is buggy */

#define setSync		*(pdo->cond) |=0x01
#define clrSync		*(pdo->cond) &=0xFE
#define checkSync	*(pdo->cond) & 0x01

#define setNew_msg	*(pdo->cond) |=0x02
#define clrNew_msg	*(pdo->cond) &=0xFD
#define checkNew_msg	*(pdo->cond) & 0x02

#define setInhibit_time   *(pdo->cond) |=0x04
#define clrInhibit_time   *(pdo->cond) &=0xFB
#define checkInhibit_time *(pdo->cond) & 0x04

#define setEvent_timer    *(pdo->cond) |=0x08
#define clrEvent_timer    *(pdo->cond) &=0xF7
#define checkEvent_timer  *(pdo->cond) & 0x08

#define setEvent_txpdo    *(pdo->cond) |=0x10
#define clrEvent_txpdo    *(pdo->cond) &=0xEF
#define checkEvent_txpdo  *(pdo->cond) & 0x10

#define setInit_pdo     *(pdo->cond) |=0x20
#define clrInit_pdo     *(pdo->cond) &=0xDF
#define checkInit_pdo	*(pdo->cond) & 0x20

#define setTx		*(pdo->cond) |=0x40
#define setRX		*(pdo->cond) &=0xBF
#define checkRxTx	*(pdo->cond) & 0x40

#define setLock		*(pdo->cond) |=0x80
#define clrLock		*(pdo->cond) &=0x7F
#define checkLock	*(pdo->cond) & 0x80


// ROM memory

struct PDO_Object{
    
    uint8_t*	cond;
    uint32_t*	cob_id ;
    
    uint8_t*	sub_index ;
    uint8_t*	Transmission_type;
    uint8_t*	Sync_start_value;
    uint8_t *   counter_sync;
    
    uint16_t*	Inhibit_time; 	// n x 100ms
    uint16_t*	counter_Inhibit_time; // 0 <-- (Inhibit_time --)
       
    uint16_t*	Event_timer;	// n x 100ms
    uint16_t*	counter_Event_timer;  // 0 <-- (Event_timer--)
    
    // mapping
    
    uint8_t*	sub_index_map;
    uint32_t*   map;	      // massiv[MAX_MAP_DATA]
    uint8_t**	map_addr_obj; // massiv[MAX_MAP_DATA]
    uint8_t*    n_byte_pdo_map;

    struct
    OD_Object*	OD_Object_list; // 
    
    // Buffer
    
    uint8_t*    buffer;

    // function
    
    void   (*init_pdo)(struct PDO_Object* pdo);
    void   (*build_map)(struct PDO_Object* pdo);
    void   (*process_rxpdo)(struct PDO_Object *pdo);
    void   (*process_txpdo)(struct PDO_Object *pdo);
     
};  

// ROM memory
struct SDO_Object{

    uint8_t*	cond; 
	
    uint8_t*	sub_index;
    uint8_t*	node_id;
    uint32_t*	cob_id_client;
    uint32_t*	cob_id_server;
  
};


/*  struct msg  */

// for no 8bit
// #pragma pack(push, 1)

typedef union {
    
    struct {
        uint8_t idType;
        uint32_t id;         
        uint8_t dlc;   
        uint8_t data0; 
        uint8_t data1;
        uint8_t data2; 
        uint8_t data3; 
        uint8_t data4; 
        uint8_t data5; 
        uint8_t data6; 
        uint8_t data7;
	
    } can_frame;
    
    struct {
	    
        uint8_t  idType; 
        uint32_t id;	   
        uint8_t  dlc;	  
        uint8_t  cmd;	  
        uint16_t index;    
        uint8_t  subindex; 
        uint8_t  data0;
	uint8_t  data1;
	uint8_t  data2;
	uint8_t  data3;
       
    }frame_sdo;
    
    uint8_t array[14];
    
}CanOpen_Msg;

//  for no 8bit
//  #pragma pack(pop) 

#define COB_ID      array[1]&0x7f
#define MAX_SDO_OBJECT 1
#define MAX_PDO_OBJECT 8 //4x2
#define MAX_OBJ_TIMER MAX_MAP_DATA+(MAX_MAP_DATA/2)+1

struct _CanOpen{

    uint8_t*                  id; /*id */
    uint8_t*                mode; /*pre-orintal etc.*/

    CanOpen_Msg*    current_msg;

    uint8_t  (*init)(uint8_t id);
    uint8_t  (*receiving_message)(CanOpen_Msg *msg);
    uint8_t  (*sending_message)(CanOpen_Msg *msg);
    void     (*pdo_timer)(void);

    struct OD_Object*    map;
    
    struct PDO_Object**  pdo;
    struct SDO_Object**  sdo;

    uint16_t*     pdo_timers;  // !! -not quite right
    uint8_t*     n_obj_timer;
    
};

/* --------------- ERROR ----------------*/

#define OK_SAVE         0x60
#define RESPONSE_ERROR  0x80		

const uint32_t error_msg[]={

#define NO_ERROR         0
0x00000000,
#define ERROR_NO_ACCESS  1
0x06010000,
#define ERROR_NO_READ    2
0x06010001,
#define ERROR_NO_SAVE    3
0x06010002, 
#define ERROR_NO_OBJECT  4     
0x06020000,
#define ERROR_OBJECT_PDO 5    
0x06040041,
#define ERROR_SIZE_PDO   6   
0x06040042,
#define ERROR_LEN_OBJECT 7
0x06070010,
#define ERROR_bLEN_OBJECT 8   
0x06070012,
#define ERROR_sLEN_OBJECT 9  
0x06070013,
#define ERROR_SUB_INDEX	  10	
0x06090011,
#define ERROR_SYSTEM      11
0x06040047,
#define ERROR_EQUIPMENT   12
0x06060000,
#define ERROR_TOGGLE_BIT  13
0x05030000,
#define ERROR_SDO_SERVER  14
0x05040001,
#define ERROR_DATA        15
0x06090030,
#define ERROR_BIG_DATA_OBJ 16
0x06090031,
#define ERROR_SMALL_DATA_OBJ  17
0x06090032,
#define ERROR_ALL_OD_TABLE 18
0x08000000,
#define ERROR_OBJ_DICT    19
0x80000023,
};


/*---------- function  OD_TABLE  ------------*/


/*index 0000 -  0xFFFF -> the end)*/

struct OD_Object* OD_search_index(uint16_t index, struct OD_Object* tab){   
    while (tab->index < index){tab++;} 
    tab = tab->index == index?tab:NULL;
    return tab;}

struct OD_Object* 
	OD_search_msg_index(CanOpen_Msg *msg, struct OD_Object* tab){
return  OD_search_index((msg->frame_sdo.index),tab);}

struct OD_Object* 
	OD_search_map_index(CanOpen_Msg *msg, struct OD_Object* tab){
	uint16_t index = msg->frame_sdo.data3;
	index <<= 8;
	index |= msg->frame_sdo.data2;
return  OD_search_index(index,tab);}
	
/* copy block  */

void copy_data (uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
    
    if(!wdata || !rdata)return;
    nbit >>=3;
    for(uint8_t i=0;i<nbit;i++){*(wdata + i) = *(rdata+i);}
    
};

void copy_data_sdo(uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
	
	if(!wdata || !rdata)return;
	switch(nbit){
	case 0x20:*(wdata++) = *(rdata++);
	case 0x18:*(wdata++) = *(rdata++);
	case 0x10:*(wdata++) = *(rdata++);
	case 0x08:*(wdata) = *(rdata);break;
	default:break;};
};


uint8_t* copy_rdata_answer (uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
    
    if(!wdata || !rdata)return wdata;
    nbit >>=3;
    for(uint8_t i=0;i<nbit;i++){*(wdata + i) = *(rdata+i);}
    return (rdata+nbit);
    
};

uint8_t* copy_wdata_answer (uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
    
    if(!wdata || !rdata)return wdata;
    nbit >>=3;
    for(uint8_t i=0;i<nbit;i++){*(wdata + i) = *(rdata+i);}
    return (wdata+nbit);
    
};	
	

void copy_xPDO(uint8_t* wdata,uint8_t* rdata,uint8_t dlc){
    
    for(uint8_t i= 0; i< MAX_MAP_DATA ;i++){
        *(wdata+i)= i < dlc?*(rdata+i):0;}
    
};


/* compare block  */
uint8_t compare_bytes(uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
	
	if(!wdata || !rdata)  return 0;
	nbit >>=3;
	for(uint8_t i=0;i<nbit;i++){
        if(*(wdata + i) != *(rdata+i)) return 0;
	}
	return 0x01;
};







/*----------   function PDO communication  -------------*/

#define PDO_DISABLED	0x80000000 // 0b1000 0000 0000 0000
#define PDO_is_TX       0x80 	   //0b1000 0000
#define PDO_calculate_n	0x700 // 0b111 0000 0000
#define PDO_mask_addr	0x7FF // 0b0111 1111 1111


#define MAX_MAP_NBIT MAX_MAP_DATA*8 //warning 8bit x n_byte




void build_map_objects(struct PDO_Object* pdo){

     if(!pdo)return;
   
     uint8_t sum_nbit = 0,sub_i = 0, subindex_map = *pdo->sub_index_map,
	     *addr_obj,
	     **map_obj = pdo->map_addr_obj;
     
     struct Data_Object d_obj;
     struct OD_Object* obj;   
     union  map_data* map;
           
     if(subindex_map > MAX_MAP_DATA) subindex_map = MAX_MAP_DATA;
    
     while(sub_i < subindex_map){
	         
	map = (union map_data* )pdo->map + sub_i;
	if(map->info.nbit == 0 || map->info.index == 0) break;
	
	obj = OD_search_index(map->info.index,pdo->OD_Object_list);
        if(obj == NULL) break;
	
	d_obj.request_type = MAP_info;
	d_obj.data_object = obj;
	d_obj.sub_index = map->info.sub_index;
	
	obj->data_func(&d_obj);
	
	if(d_obj.rw_object == NULL || d_obj.attribute & NO_MAP ||
	   map->info.nbit != d_obj.nbit) break;
	
	/* check ro and wo*/
	if(checkRxTx){ if(!(d_obj.attribute&RO)) break; //tx ->read memory?
	}else{ if(!(d_obj.attribute&WO)) break; } // rx -> save_mem ?
	

        sum_nbit += map->info.nbit;
        if(sum_nbit > MAX_MAP_NBIT) break;
	
	addr_obj = (uint8_t*)d_obj.rw_object;
	
       	switch(d_obj.nbit){
		
	case 0x20:*map_obj++ = addr_obj++;
	case 0x18:*map_obj++ = addr_obj++;
	case 0x10:*map_obj++ = addr_obj++;
	case 0x08:*map_obj++ = addr_obj;break;
	default:return;break;}; 
       
        sub_i++;
      };
      
      if(!sub_i)setLock; // subindex map == 0? pdo lock
      
      *pdo->sub_index_map = sub_i;
      *pdo->n_byte_pdo_map = sum_nbit >>3;
      
};


void process_the_RxPDO_message(struct PDO_Object* pdo){
    
    if(!pdo)return;
    
    uint8_t   num = 0, 
	      **addr  = pdo->map_addr_obj, 
	      *buffer = pdo->buffer;
    
    while(num < (*pdo->n_byte_pdo_map)){ 
	    
	    **(addr+num) = *(buffer+num);
	    num++;
    
    }
};


void copy_rxPDO_message_to_array(CanOpen_Msg* msg,struct PDO_Object* pdo){
    
    if(msg->can_frame.dlc < *(pdo->n_byte_pdo_map)) return; 
    copy_xPDO(pdo->buffer,(uint8_t*)&msg->can_frame.data0,*(pdo->n_byte_pdo_map)); //or 8?
    setNew_msg; 
    
};


void process_the_TxPDO_message(struct PDO_Object* pdo){
    
    if(!pdo)return;
    
    uint8_t   num = 0, 
	      **addr  = pdo->map_addr_obj, 
	      *buffer = pdo->buffer;
    
    while(num < (*pdo->n_byte_pdo_map)){ 
	    
	    *(buffer+num) = **(addr+num);
	    num++;
    
    }   
};

void copy_txPDO_array_to_message(CanOpen_Msg* msg,struct PDO_Object* pdo){
    
    copy_xPDO ((uint8_t*)&msg->can_frame.data0, pdo->buffer, *(pdo->n_byte_pdo_map));// or 8?
    
};

void start_Inhibit_time(struct PDO_Object *pdo){
	
	if(pdo->counter_Inhibit_time && *pdo->Inhibit_time){	    	
		*pdo->counter_Inhibit_time = *pdo->Inhibit_time;    
		setInhibit_time ;} 
};

void start_Event_timer(struct PDO_Object *pdo){
	
	if(pdo->counter_Event_timer && *pdo->Event_timer){    
		*pdo->counter_Event_timer = *pdo->Event_timer;    
		setEvent_timer;} 
};


void init_xPDO(struct PDO_Object* pdo){
    
    if(!pdo)return;
    
    setLock;
    clrNew_msg;
    
    if(*pdo->cob_id&PDO_DISABLED)return;
    
    if(pdo->build_map == NULL)return;
    
    pdo->build_map(pdo);
    if(*pdo->sub_index_map == 0)return;
    
    switch(*pdo->Transmission_type){
    
	case 0xFF: start_Event_timer(pdo);
	case 0xFE: start_Inhibit_time(pdo); break;
	default: if(*pdo->Transmission_type > 0xF0) return;
	         *pdo->counter_sync = *pdo->Transmission_type;
		 break;
    };

    clrLock;
};



void pdo_object_type(struct PDO_Object *pdo){
    
    if(!pdo)return;
    
    if(checkLock) return;
    if((checkInit_pdo) && pdo->init_pdo) pdo->init_pdo(pdo);
    
    switch(*pdo->Transmission_type){
 
/*PDO transmission is sent if the PDO data was changed by at least 1 bit.
  interval before sending PDO = inhibit_time * 100ms;
 */
        case 0xFF:
      
            if(checkRxTx){//txpdo == 1
         
                if(!(checkEvent_txpdo)) break; 
		
                if(checkInhibit_time){	
			if(pdo->counter_Inhibit_time && 
			   *pdo->counter_Inhibit_time) break;
		   clrInhibit_time;}
		
                if(checkEvent_timer){
			if(pdo->counter_Event_timer &&
			   *pdo->counter_Event_timer) break;
		   clrEvent_timer;}
               
                if(pdo->process_txpdo)pdo->process_txpdo(pdo);
                
                clrEvent_txpdo;
                
		start_Inhibit_time(pdo);
		start_Event_timer (pdo);
		
                setNew_msg;
                
            }else{ //rxpdo = 0
                
                if(!(checkNew_msg))break;
                if(pdo->process_rxpdo)pdo->process_rxpdo(pdo);
                clrNew_msg;
            }
      
        break;
        
/* Cyclically  
 * Transfer PDO ,period  = inhibit_time * 100ms;
   Receive PDOs, on the other hand, are parsed immediately upon receipt.
 */
        case 0xFE: 
            
            if(checkRxTx){  //txpdo
         
                if(checkInhibit_time){	
			if(pdo->counter_Inhibit_time && 
			   *pdo->counter_Inhibit_time) break;
		   clrInhibit_time;}
		
                if(pdo->process_txpdo)pdo->process_txpdo(pdo);
				
		start_Inhibit_time(pdo);
	
                setNew_msg;
                
            }else{ //rxpdo 
               
                if(!(checkNew_msg))break;
                if(pdo->process_rxpdo)pdo->process_rxpdo(pdo);
                clrNew_msg;
            }
            
        break; 
        
/* SYNC-Message
 0 - send only when an event occurs and SYNC
 
 */
        case 0x00:
            
             if(checkRxTx){//txpdo
           
                if(!(checkSync))break;
                if(!(checkEvent_txpdo))break;
		
                if(pdo->process_txpdo)pdo->process_txpdo(pdo);
                
		clrSync;
                clrEvent_txpdo;
                setNew_msg;
                
            }else{ //rxpdo
         
                if(!(checkSync))break; 
                if(!(checkNew_msg))break; // +pdo->cond.flag.sync = 0;
		
                if(pdo->process_rxpdo)pdo->process_rxpdo(pdo);
                
		clrNew_msg;
                clrSync;
                
            }
            
        break;  
        
    //1 -240 sent cyclically on SYNC 
        
        default:
            
            if(*pdo->Transmission_type > 0xF0) break;
            
            if(checkRxTx){
                
                //txpdo       
                if(!(checkSync))break;
		
                if(pdo->process_txpdo)pdo->process_txpdo(pdo);
                
		clrSync;
                *pdo->counter_sync = *pdo->Transmission_type;
                setNew_msg;
                
            }else{  //rxpdo
                
                if(!(checkSync))break; // sync == 0 ?
                if(!(checkNew_msg))break;
		
                if(pdo->process_rxpdo)pdo->process_rxpdo(pdo);
                
		clrNew_msg;
                clrSync;
                *pdo->counter_sync = *pdo->Transmission_type;
                
            }
        break;         
    } 
};	
	
	
	
	/* ----------------  SDO COMMUNICATION -----------------*/


#define RESPONSE_1b  0x4f
#define RESPONSE_2b  0x4B
#define RESPONSE_3b  0x47
#define RESPONSE_4b  0x43

#define GET_1b        0x2F
#define GET_2b        0x2B
#define GET_3b        0x27
#define GET_4b        0x23

#define READ_REQUEST  0x40
#define WRITE_REQUEST 0x20

#define SUB_INDEX_FF  0x0FF 


uint8_t check_sdo_command_for_writing(CanOpen_Msg *msg,uint8_t nbit){
    
    uint8_t dlc = 4+(nbit >> 3);     
    uint8_t cmd = 8-dlc;
    cmd <<=2;
    cmd |=0x23;
    if(msg->frame_sdo.cmd != cmd) return ERROR_SDO_SERVER;
    if(msg->frame_sdo.dlc < dlc) return ERROR_SMALL_DATA_OBJ;
    msg->frame_sdo.cmd = OK_SAVE;
    msg->frame_sdo.dlc = 4;
    return 0;
    
} 

uint8_t check_sdo_command_for_reading(CanOpen_Msg *msg,uint8_t nbit){
    
    if(msg->frame_sdo.cmd != READ_REQUEST) return ERROR_SDO_SERVER;
    if(msg->frame_sdo.dlc < 4) return ERROR_SMALL_DATA_OBJ;
    uint8_t dlc = 4+(nbit >> 3);
    msg->frame_sdo.dlc = dlc;
    uint8_t cmd = 8-dlc;
    cmd <<=2;
    msg->frame_sdo.cmd = cmd |0x43;   
    return 0;
    
}


	
/*------------ function Object proccesing -------*/


void single_object(struct Data_Object *obj){
   
    uint8_t  nbit,*wdata,*rdata;
    CanOpen_Msg* msg;
    
    if(!obj)return;
    nbit = obj->nbit;
    
    switch(obj->request_type){
        
       case SDO_request:
           
            msg = obj->rw_object;
            if(!msg) break;
            uint8_t error = ERROR_SDO_SERVER;
            wdata = &msg->frame_sdo.data0;//block data (4byte) uint8_t*
	    
 /*Read OD_object  msg <- addr_object*/   
	    
            if(msg->frame_sdo.cmd == READ_REQUEST){//read
		    
                if(obj->attribute&RO){		
			
                  error = check_sdo_command_for_reading(msg,nbit); 
                  
		  if(!error){ 
                    switch(msg->frame_sdo.subindex){    
                        case 0: rdata =(uint8_t*) obj->data_object;
				break;              
                        case 0xFF:
                            rdata = (uint8_t*)&subindex_FF[obj->sub_index_ff];
                            nbit = 0x20;
                            msg->frame_sdo.cmd = RESPONSE_4b;
                            msg->frame_sdo.dlc = 8;
				break;         
                        default:error = ERROR_SUB_INDEX;break;}
		    };
		  }else error = ERROR_NO_READ;	
		
/*Write_OD_object   msg -> addr_object*/	
		
            }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){ 
	
		 if(obj->attribute&WO){ 
			 
		   error = check_sdo_command_for_writing(msg,nbit);//ok save
		   
		   if(!error){  
		      if(msg->frame_sdo.subindex == 0){
			  rdata = wdata;
                          wdata = (uint8_t*)obj->data_object;	    
		     }else error = ERROR_SUB_INDEX;}
		   
		}else error = ERROR_NO_SAVE;
		 
           };
	    
           if(error){ rdata = (uint8_t*)&error_msg[error];
                      nbit = 0x20;
                      msg->frame_sdo.cmd = RESPONSE_ERROR;
                      msg->frame_sdo.dlc = 0x08;}
	    
          copy_data_sdo(wdata,rdata,nbit);
	  
          break;      
            
       /*response data_object | answer rw_data  *addr_objecta,nbit,attr;NULL - no object.
          * for fast processing mapping*/ 
	  
       case MAP_info:
		  /*
           if(obj->sub_index == 0xFF){
           obj->rw_object = (uint8_t*)&subindex_FF[obj->sub_index_ff];
           obj->nbit = 0x20;obj->attribute = RO;};
           */
            obj->rw_object = obj->sub_index?NULL: obj->data_object;
	    
	  break;
        
     default:break;}
    
};	
	
void one_type_array_object(struct Data_Object *obj){
	
    if(!obj)return;
	
    uint8_t *wdata,*rdata,nbit = obj->nbit;;
    CanOpen_Msg* msg;
    
    struct one_type_array* array =(struct one_type_array*) obj->data_object;
    if(!array) return;
    
    switch(obj->request_type){
		
       case SDO_request:
	       
            msg = obj->rw_object;
            if(!msg)return;
            uint8_t error = ERROR_SDO_SERVER;
            wdata =(uint8_t*) &msg->frame_sdo.data0;
	    
 /*Read OD_object  msg <- addr_object*/ 
	    
            if(msg->frame_sdo.cmd == READ_REQUEST){ 
		    
                if(obj->attribute&RO){
			
                  error = check_sdo_command_for_reading(msg,nbit);
		
                  if(!error){ 
			  
                    switch(msg->frame_sdo.subindex){
                        case 0:rdata = &array->sub_index;nbit  = 0x08;
			       msg->frame_sdo.cmd = RESPONSE_1b;
			       msg->frame_sdo.dlc = 5;
			       break;              
                        case 0xFF:
                            rdata = (uint8_t*)&subindex_FF[obj->sub_index_ff];
                            nbit = 0x20;
                            msg->frame_sdo.cmd = RESPONSE_4b;
                            msg->frame_sdo.dlc = 8;
			       break;         
                        default: if(array->sub_index < (msg->frame_sdo.subindex))
							error = ERROR_SUB_INDEX;break;
			         rdata = (uint8_t*)array->array;
			         rdata = rdata + ((nbit>>3)*((msg->frame_sdo.subindex)-1));
			         break;}
		 };
               }else error = ERROR_NO_READ;
		
/*Write_OD_object   msg -> addr_object*/			
		     
           }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){   //write
		    
		if(obj->attribute&WO){
			
		error = check_sdo_command_for_writing(msg,nbit);
		
		  if(!error){ 
			  
		    switch(msg->frame_sdo.subindex){    
                       case 0:error = ERROR_NO_SAVE;break;              
                       default: if(array->sub_index < (msg->frame_sdo.subindex))
							error = ERROR_SUB_INDEX;break;
			        rdata = wdata;
				wdata = (uint8_t*)array->array;
				wdata = wdata + ((nbit>>3)*((msg->frame_sdo.subindex)-1));

			break;
		    }
		  }
		}else error = ERROR_NO_SAVE;
	   
	   };
	    
          if(error){  rdata = (uint8_t*)&error_msg[error];
                      nbit = 0x20;
                      msg->frame_sdo.cmd = RESPONSE_ERROR;
                      msg->frame_sdo.dlc = 0x08;}
	    
          copy_data_sdo(wdata,rdata,nbit);            
          
	  break;         
            
        case MAP_info:
        
	   if(!obj->data_object){obj->rw_object= NULL;return;}
	     
	      switch(obj->sub_index){
	          case 0: rdata = &array->sub_index;nbit = 0x08;obj->attribute = RO;
		             break;
              /*    case 0xFF: rdata = (uint8_t*)&subindex_FF[obj->sub_index_ff]; 
                             nbit = 0x20;
                             obj->attribute = RO;
			     break;               */
	          default: if(obj->sub_index > array->sub_index){rdata=NULL;nbit=0;break;}
		           rdata = (uint8_t*)array->array;
		           nbit = obj->nbit;
		           rdata = rdata?rdata + ((nbit>>3)*((obj->sub_index)-1)):NULL;
			   break;}
	      
	  obj->rw_object = rdata;
	  obj->nbit = nbit;
	  break;
	    
        default:break;}
};

/* bit,attribute,data.type*/
//#define OBJ_ATTR(b,a,f,t) obj->nbit = b;obj->attribute=a;obj->sub_index_ff = (f << 8)| t ;

/* 1 byte object*/

void ro_object_1byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x08;obj->attribute=RO;obj->sub_index_ff = UINT8_OD_VAR;
	single_object(obj);}

void rw_object_1byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x08;obj->attribute=RW;obj->sub_index_ff = UINT8_OD_VAR;
	single_object(obj);}

void ro_array_1byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x08;obj->attribute=RO;obj->sub_index_ff = UINT8_OD_ARRAY;	
	one_type_array_object(obj);}

void rw_array_1byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x08;obj->attribute=RW;obj->sub_index_ff = UINT8_OD_ARRAY;	
	one_type_array_object(obj);}

/* 2 byte object*/

void ro_object_2byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x10;obj->attribute=RO;obj->sub_index_ff = UINT16_OD_VAR;
	single_object(obj);}

void rw_object_2byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x10;obj->attribute=RW;obj->sub_index_ff = UINT16_OD_VAR;
	single_object(obj);}

void ro_array_2byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x10;obj->attribute=RO;obj->sub_index_ff = UINT16_OD_ARRAY;	
	one_type_array_object(obj);}

void rw_array_2byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x10;obj->attribute=RW;obj->sub_index_ff = UINT16_OD_ARRAY;	
	one_type_array_object(obj);}

/* 3 byte object*/

void ro_object_3byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x18;obj->attribute=RO;obj->sub_index_ff = UINT24_OD_VAR;
	single_object(obj);}

void rw_object_3byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x18;obj->attribute=RW;obj->sub_index_ff = UINT24_OD_VAR;
	single_object(obj);}

void ro_array_3byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x18;obj->attribute=RO;obj->sub_index_ff = UINT24_OD_ARRAY;	
	one_type_array_object(obj);}

void rw_array_3byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x18;obj->attribute=RW;obj->sub_index_ff = UINT24_OD_ARRAY;	
	one_type_array_object(obj);}

/* 4 byte object*/

void ro_object_4byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x20;obj->attribute=RO;obj->sub_index_ff = UINT32_OD_VAR;
	single_object(obj);}

void rw_object_4byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x20;obj->attribute=RW;obj->sub_index_ff = UINT32_OD_VAR;
	single_object(obj);}

void ro_array_4byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x20;obj->attribute=RO;obj->sub_index_ff = UINT32_OD_ARRAY;	
	one_type_array_object(obj);}

void rw_array_4byte(struct Data_Object *obj){
	if(!obj)return;
	obj->nbit = 0x20;obj->attribute=RW;obj->sub_index_ff = UINT32_OD_ARRAY;	
	one_type_array_object(obj);}



/* ------------------------- pdo_object ------------------------ */

void pdo_object(struct Data_Object *obj){
    
  if(!obj)return;
  
  uint8_t error = 0, nbit = 0x20, *wdata,*rdata;
  CanOpen_Msg *msg;  
  struct PDO_Object *pdo = (struct PDO_Object *)obj->data_object;
    
  
  switch(obj->request_type){
		
    case SDO_request:

        msg = (CanOpen_Msg *)obj->rw_object;
        if(!msg)break;
	wdata = (uint8_t*)&msg->frame_sdo.data0;
	
/*Read OD_object  msg <- addr_object*/ 	
	
        if(msg->frame_sdo.cmd == READ_REQUEST){	 //read
		
          if(obj->attribute&RO){    
            if(msg->frame_sdo.dlc < 4) error = ERROR_SMALL_DATA_OBJ;
            if(!pdo) error = ERROR_SYSTEM;                
            if(!error){
	      
              switch(msg->frame_sdo.subindex){                 
                case 0:rdata = (uint8_t*)pdo->sub_index;nbit = 0x08;break;
                case 1:rdata = (uint8_t*)pdo->cob_id;break;
                case 2:rdata = (uint8_t*)pdo->Transmission_type; nbit = 0x08;break;
                case 3:rdata = (uint8_t*)pdo->Inhibit_time; nbit = 0x10;break;                    
                case 5:rdata = (uint8_t*)pdo->Event_timer; nbit = 0x10;break;
                case 0xFF:rdata = (uint8_t*)&subindex_FF[obj->sub_index_ff];break;
                default: error = ERROR_SUB_INDEX;break;}}    
          }else error = ERROR_NO_READ;
	    
	if(!error)error = check_sdo_command_for_reading(msg,nbit); 
	  
/*Write_OD_object   msg -> addr_object*/		  
	    
       }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){ // write
	  
	  
	  
	if(obj->attribute&WO){
		  
          rdata = wdata;  
		  
          if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ;
          if(!pdo || pdo->map) error = ERROR_SYSTEM;
          if(!error){    	  
	      
	//uint8_t lock = pdo->cob_id&PDO_DISABLED?1:0;
            switch(msg->frame_sdo.subindex){
                
        // sub-index
                
             case 0: error = ERROR_NO_SAVE;break;
            
        // sub-id 32bit
	    
             case 1:
                
              if(msg->frame_sdo.cmd != GET_4b){error = ERROR_SDO_SERVER;break;}  
              if(msg->frame_sdo.dlc < 8){error = ERROR_SMALL_DATA_OBJ;break;}
              
	      wdata = (uint8_t*)pdo->cob_id;
	      //nbit = 0x20;
	      
	      // correct id  cob_id&0x80000fff
	      *rdata	&=0xff;
	      *(rdata+1)&=0x0f;
	      *(rdata+2)&=0x00;
	      *(rdata+3)&=0x80;
	      
              if(checkLock){ // lock == 1? 
                 
	        if(!(checkRxTx)){	//Rx ?
		     
	           if(*rdata != *wdata ||
		      *(rdata+1) != *(wdata+1)){error = ERROR_NO_SAVE;break;}
		}

              }else{ // lock == 0 ? 
              
		   if(*rdata != *wdata ||
		      *(rdata+1) != *(wdata+1)){error = ERROR_NO_SAVE;break;}
		   
	      };
           
              setInit_pdo;
        
              break;
               
        // Transmission_type 8bit
	       
            case 2: 
                
              if(!(checkLock)){error = ERROR_NO_SAVE;break;} 
              if(msg->frame_sdo.cmd != GET_1b){error = ERROR_SDO_SERVER;break;}  
              if(msg->frame_sdo.dlc < 5){error = ERROR_SMALL_DATA_OBJ;break;}
            
                 wdata = pdo->Transmission_type; nbit = 0x08;
                 setInit_pdo;

              break;
                
        // Inhibit_time; 16bit
            case 3: 
                
              if(!(checkLock)){error = ERROR_NO_SAVE;break;}
                if(msg->frame_sdo.cmd != GET_2b){error = ERROR_SDO_SERVER;break;} 
                if(msg->frame_sdo.dlc < 6){error = ERROR_SMALL_DATA_OBJ;break;} 
              
                  wdata =(uint8_t*)pdo->Inhibit_time; nbit = 0x10;
		  setInit_pdo;
	     
             break;
                
        // Event_timer 16bit
            case 5:  
                
              if(!(checkLock)){error = ERROR_NO_SAVE;break;} 
                if(msg->frame_sdo.cmd != GET_2b){error = ERROR_SDO_SERVER;break;} 
                if(msg->frame_sdo.dlc < 6){error = ERROR_SMALL_DATA_OBJ;break;}
              
                  wdata = (uint8_t*)pdo->Event_timer; nbit = 0x10;
		  setInit_pdo;
                                 
             break;
                 
            default:error = ERROR_SUB_INDEX;break;}}
	  
	    if(!error){msg->frame_sdo.cmd = OK_SAVE;
		       msg->frame_sdo.dlc = 4;
		       nbit=0;}else{wdata=rdata;}
	  
	  }else error = ERROR_NO_SAVE;	  
        }else error = ERROR_SDO_SERVER;
	
          if(error){rdata = (uint8_t*)&error_msg[error];
                    nbit = 0x20;
                    msg->frame_sdo.cmd = RESPONSE_ERROR;
                    msg->frame_sdo.dlc = 0x08;}
	    
         copy_data_sdo(wdata,rdata,nbit);        
        
	 break; 
           
    case MAP_info:
        
          switch(obj->sub_index){
                case 0: obj->rw_object = pdo->sub_index;nbit = 0x08;break;
                case 1: obj->rw_object = pdo->cob_id;nbit = 0x20;break;       
                case 2: obj->rw_object = pdo->Transmission_type;nbit = 0x08;break;
                case 3: obj->rw_object = pdo->Inhibit_time;nbit = 0x10;break;       
                case 5: obj->rw_object = pdo->Event_timer;nbit = 0x10;break;
                /*
                case 0xff:obj->rw_object = &subindex_FF[obj->sub_index_ff]; 
                          nbit = 0x20;obj->attribute = RO;break; */
                
                default:obj->rw_object = NULL;nbit = 0x00;break;}
           obj->nbit = nbit;
           break;
    };  
};


void ro_pdo_object(struct Data_Object *obj){
	
    if(!obj)return;
	obj->nbit = 0x20;obj->attribute=RO;obj->sub_index_ff = PDO_COMM_OD_DEFSTRUCT;
	pdo_object(obj);
}


void rw_pdo_object(struct Data_Object *obj){
	
	if(!obj)return;
	obj->nbit = 0x20;obj->attribute=RW;obj->sub_index_ff = PDO_COMM_OD_DEFSTRUCT;
	pdo_object(obj);
}


/*------------------------- pdo_map object ------------------------------*/

void map_object(struct Data_Object *obj){
    
  if(!obj)return;
  CanOpen_Msg *msg;
  uint8_t nbit = 0x20, error = 0,*wdata,*rdata;
  struct PDO_Object *pdo = (struct PDO_Object *)obj->data_object;

  switch(obj->request_type){		
    
    case SDO_request:
        
      msg = (CanOpen_Msg *)obj->rw_object;
      if(!msg)return;
      wdata = (uint8_t*)&msg->frame_sdo.data0;
      
/*Read OD_object  msg <- addr_object*/ 	     
      
      if(msg->frame_sdo.cmd == READ_REQUEST){
	      
        if(obj->attribute&RO){

           if(msg->frame_sdo.subindex > MAX_MAP_DATA) error = ERROR_SUB_INDEX;
           if(msg->frame_sdo.dlc < 4) error = ERROR_SMALL_DATA_OBJ;
           if(!pdo) error = ERROR_SYSTEM;         
           if(!error){ 
		   
              switch(msg->frame_sdo.subindex){
                case 0: rdata= pdo->sub_index_map;nbit=0x08;break;
                case 0xFF:rdata=(uint8_t*)&subindex_FF[obj->sub_index_ff];;break;       
                default: rdata = (uint8_t*)pdo->map +((msg->frame_sdo.subindex)-1);
                         break;}
	  }        
       }else error = ERROR_NO_READ;
	
/*Write_OD_object   msg -> addr_object*/	           
	
    }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){
	    
        if(obj->attribute&WO){
	       
	   rdata = wdata;	
		
           if(msg->frame_sdo.subindex >MAX_MAP_DATA) error = ERROR_SUB_INDEX;
           if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ; 
           if(!pdo) error = ERROR_SYSTEM;
	   if(!(checkLock)) error = ERROR_NO_SAVE;
           if(!error){
		    
               switch(msg->frame_sdo.subindex){  
                  
	         case 0: if(msg->frame_sdo.cmd != GET_1b){error = ERROR_SDO_SERVER;break;}
			 //if(*rdata > MAX_MAP_DATA) {error = ERROR_OBJECT_PDO;break;}
			 if(*rdata > MAX_MAP_DATA) *rdata = MAX_MAP_DATA;  
			 wdata = pdo->sub_index_map;
			 break;
                 default: 
			if(msg->frame_sdo.dlc < 8){error = ERROR_SMALL_DATA_OBJ;break;};
			if(msg->frame_sdo.cmd != GET_4b){error = ERROR_SDO_SERVER; break;}
			if(*rdata == 0){error = ERROR_SMALL_DATA_OBJ;break;} // check nbit==0?
			if(*rdata > 0x40){error = ERROR_BIG_DATA_OBJ;break;}
			wdata = (uint8_t*)pdo->map +(*(pdo->sub_index_map)-1);
			break;}
	  }
	   
	  if(!error){msg->frame_sdo.cmd = OK_SAVE;
		     msg->frame_sdo.dlc = 4;
		     setInit_pdo; // init PDO = check_map
		     
	 }else{wdata=rdata;} 
	      
       }else error = ERROR_NO_SAVE;
		
    }else error = ERROR_SDO_SERVER;
      
     if(error){rdata = (uint8_t*)&error_msg[error];
              nbit = 0x20;
              msg->frame_sdo.cmd = RESPONSE_ERROR;
              msg->frame_sdo.dlc = 0x08;}
	    
      copy_data_sdo(wdata,rdata,nbit);
    break;       

    case MAP_info:
	    
      switch(obj->sub_index){
        case 0:obj->rw_object = &(pdo->sub_index_map);
               obj->nbit = 0x08;break;
        default: if(obj->sub_index <=MAX_MAP_DATA){
                    obj->rw_object = pdo->map+((obj->sub_index) -1);
                    //obj->nbit = 0x20;
                }else{obj->rw_object = NULL;obj->nbit = 0;};
        break;}
      
     break;}
}

void ro_map_object(struct Data_Object *obj){
	
	if(!obj)return;
	obj->attribute=RO;obj->sub_index_ff = PDO_MAPPING_OD_DEFSTRUCT;
	map_object(obj);
}

void rw_map_object(struct Data_Object *obj){
	
	if(!obj)return;
	obj->attribute=RW;obj->sub_index_ff = PDO_MAPPING_OD_DEFSTRUCT;
	map_object(obj);
}

/*------------------------- sdo_map object ------------------------------*/

void sdo_object(struct Data_Object *obj){
    
    
      
}
void ro_sdo_object(struct Data_Object *obj){
	
	if(!obj)return;
	obj->attribute=RO|NO_MAP;obj->sub_index_ff = SDO_PARAMETER_OD_DEFSTRUCT;
        sdo_object(obj);}

void rw_sdo_object(struct Data_Object *obj){
	
	if(!obj)return;
	obj->attribute=RW|NO_MAP;obj->sub_index_ff = SDO_PARAMETER_OD_DEFSTRUCT;
        sdo_object(obj);

;}

	
/* ------------------ STRUCT CAN NODE ------------------*/

/* ------------NULL-> function obiect call -----------*/


void NOP_call(struct Data_Object *obj){};


/*---------------  NMT_control ----------------*/

/* NMT command  000_DLC_CS_NodeID
 * DLC = 2 byte
 * CS  =  command (1 byte)
 * NodeID = 01h-7fh address
*/	

#define START_REMOTE_NODE       0x01
#define STOP_REMOTE_NODE        0x02	
#define ENTER_PRE_OPERATIONAL	0x80
#define RESET_APPLICATION       0x81
#define RESET_COMMUNICATION     0x82	
	
#define BOOT			0x00
#define STOPPED			0x04
#define OPERATIONAL		0x05
#define PRE_OPERATIONAL		0x7F

void NMT_message_processing(struct _CanOpen *node){
    
        if (((node->current_msg->can_frame.id)&0x7F) == 0){
             
            if((node->current_msg->can_frame.data1) == 0 ||
               (node->current_msg->can_frame.data1) == *node->id){
                        
                switch((node->current_msg->can_frame.data0)){
                        
                    case START_REMOTE_NODE:     *node->mode = OPERATIONAL;break;
                    case STOP_REMOTE_NODE:      *node->mode = STOPPED ;break;  
                    case ENTER_PRE_OPERATIONAL: *node->mode = PRE_OPERATIONAL;break;
                    case RESET_APPLICATION:     *node->mode = BOOT; break;
                    case RESET_COMMUNICATION:   *node->mode = BOOT; break;
                    default:break;                               
                };
            };                    
        };
};
/*---------------------- Sync -----------------------*/

void SYNC_message_processing(struct _CanOpen *node){  
    
     struct PDO_Object* pdo;
    
     for(uint8_t n=0; n<MAX_PDO_OBJECT; n++){
	     
	pdo = *(node->pdo + n);
	    
      if(  pdo == NULL ) continue;
      if( *pdo->Transmission_type > 240)continue;
      if( *pdo->counter_sync)*pdo->counter_sync = *(pdo->counter_sync) -1;
      if( *pdo->counter_sync == 0) setSync; 
    
    };   
};
    
/*--------------------- Time ------------------------*/

void TIME_message_processing(struct _CanOpen *node){
    
    
    
};


/*--------------------- RxPDO ------------------------*/

void rxPDO_message_processing(uint8_t code,struct _CanOpen *node){
    
    if(*(node->mode) != OPERATIONAL) return;
    if(code >= MAX_PDO_OBJECT)	return;
    struct PDO_Object* pdo = *(node->pdo + code);
    if((checkRxTx) || (checkLock))return; // txPDO? -> return;
 
    
    copy_rxPDO_message_to_array(node->current_msg,pdo);
    
};

/*-------------------  RxSDO  -----------------------*/
    
void rxSDO_message_processing(uint8_t code,struct _CanOpen* node){// client -> server 
    
    struct SDO_Object* sdo = *(node->sdo + code);
    struct OD_Object*  tab;
    
    if(*node->mode != OPERATIONAL || *node->mode != PRE_OPERATIONAL) return;
    if(*sdo->cob_id_server&PDO_DISABLED) return;
    if(node->current_msg->frame_sdo.id != *sdo->cob_id_server) return;
    
    tab =  OD_search_msg_index(node->current_msg,node->map);
    if(!tab || !(tab->data_func) || !(tab->data)) return;
    
    struct Data_Object info;
    
    info.request_type = SDO_request;
    info.rw_object = node->current_msg;
    info.data_object = tab->data;
     
    tab->data_func(&info);
       
    node->current_msg->frame_sdo.id = *sdo->cob_id_client;
    node->sending_message(node->current_msg); // server -> client
    
};    
   
/*---------------- message_processing Node ----------------*/

// --- function code ------

#define NMT         0b0000
#define SYNC        0b0001
#define EMERGENCY   0b0001
#define TIME_STAMP  0b0010

#define TPDO1       0b0011
#define RPDO1       0b0100
#define TPDO2       0b0101
#define RPDO2       0b0110
#define TPDO3       0b0111
#define RPDO3       0b1000
#define TPDO4       0b1001
#define RPDO4       0b1010
#define TxSDO       0b1011
#define RxSDO       0b1100
#define NMT_ERROR   0b1110

#define COB_ID      array[1]&0x7f



void NODE_message_processing(struct _CanOpen* node){

    if((node->receiving_message(node->current_msg)) == 0) return;
    uint8_t fun_code = ((node->current_msg->can_frame.id)&0x780) >> 7;
    
    switch(fun_code){
    
        case NMT: NMT_message_processing(node);break;
        case SYNC:SYNC_message_processing(node);break;
        case TIME_STAMP:break;
        
        case RPDO1:rxPDO_message_processing(1,node);break;
        case RPDO2:rxPDO_message_processing(3,node);break;
        case RPDO3:rxPDO_message_processing(5,node);break;
        case RPDO4:rxPDO_message_processing(7,node);break;
        case RxSDO:rxSDO_message_processing(0,node);break;
        default:break;   
    };
};

/*-----------    processing PDO objects    ---------- */

void NODE_processing_pdo_objects(struct _CanOpen* node){
	
    struct PDO_Object* pdo;	

    if(*(node->mode) != OPERATIONAL) return;
    
    for(uint8_t i=0;i<MAX_PDO_OBJECT;i++){
       
	pdo = *(node->pdo + i);    
	    
        if(pdo != NULL){
	       
           pdo_object_type(pdo);
           
           if((checkLock) && (checkNew_msg)){// tx?
               
              node->current_msg->frame_sdo.id = *pdo->cob_id; 
              node->current_msg->frame_sdo.dlc = *pdo->n_byte_pdo_map;
              node->current_msg->frame_sdo.idType = 1; //dSTANDARD_CAN_MSG_ID_2_0B;
              copy_txPDO_array_to_message(node->current_msg,pdo);
              node->sending_message(node->current_msg);// while()?
              clrNew_msg;
              
           };          
       } ///else break;
   }; 
   
};

void NODE_init_objects_xCanOpen(struct _CanOpen* node){

   *node->mode = BOOT;
   struct PDO_Object* pdo;
   struct SDO_Object* sdo;
   
   for(uint8_t i=0;i<MAX_PDO_OBJECT;i++){ 
	   
	pdo = *(node->pdo+i);   
	   
       if(pdo != NULL) {
	       
           *pdo->cob_id |= *node->id;
	    pdo->init_pdo(pdo);
   
       }    
   };
   
   for(uint8_t i=0;i<MAX_SDO_OBJECT;i++){
	   
	   sdo = *(node->sdo+i);
	   
       if(sdo != NULL) {
	       
           *sdo->cob_id_client |= *node->id;
	   *sdo->cob_id_server |= *node->id;

       }    
   };
   
   
};

/*
 example timer
 
 void timer(){
  for (uint8_t i=0; i< xCanOpen.n_obj_timer; i++){
	if(...xCanOpen.timers[i])...xCanOpen.timers[i] --;
 };

*/
	
	
	
	
	
	

#ifdef	__cplusplus
}
#endif

#endif	/* CANOPEN_LITE_H */

