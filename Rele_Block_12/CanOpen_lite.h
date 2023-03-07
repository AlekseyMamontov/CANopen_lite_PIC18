/*  Created on 7 03 2023 ?., 20:39
 */

#ifndef CANOPEN_LITE_H
#define	CANOPEN_LITE_H

#ifdef	__cplusplus
extern "C" {
#endif


#include <stdint.h> 
#include <stdio.h>

#include "CANOPEN.h"

	
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

/*Attribute*/	

#define _CONST	0
#define RO      0b001
#define WO      0b010
#define RW      0b011
#define NO_MAP  0b10000000


/* structure */
	
struct Data_Object{
    
    uint8_t     request_type;
    uint8_t     attribute;
    uint8_t     nbit;
    uint8_t     sub_index;
    uint32_t    sub_index_ff;
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

struct PDO_Mapping {
    
    uint8_t           sub_index;
    union map_data*   map;
    
    // quick access to the map
    struct OD_Object*  node_map;
    void **	  quick_mapping;
    
    //union map_data    map[MAX_MAP_DATA];
    //void *            quick_mapping[MAX_MAP_DATA];
};

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
*/


struct PDO_Object{
    
    uint8_t	cond;
    
    // visible block
    
    uint8_t	sub_index ;
    uint8_t	Transmission_type;
    uint8_t     Sync_start_value;
    
    uint32_t	cob_id ;
    
    uint16_t	Inhibit_time; 	// n x 100ms
    uint16_t	Event_timer;	// n x 100ms
    
    // hidden block 
    
    uint16_t*	counter_Inhibit_time; // 0 <-- (Inhibit_time --)
    uint16_t*	counter_Event_timer;  // 0 <-- (Event_timer--)
    
    uint8_t     counter_sync;
        
    uint8_t     n_byte_pdo_map;
    struct PDO_Mapping* pdo_map;
    
    // function
    
    struct func_pdo  *func;
     
    // Buffer 
    
    uint8_t*      buffer;
    
};  

struct func_pdo{
    
    void   (*init_pdo)(struct PDO_Object* pdo);
    void   (*check_map)(struct PDO_Object* pdo);
    void   (*process_rxpdo)(struct PDO_Object *pdo);
    void   (*process_txpdo)(struct PDO_Object *pdo);
    
};


struct SDO_Object{
	
    uint32_t	cob_id_client;
    uint32_t	cob_id_server;
    uint8_t	node_id;
    uint8_t	sub_index;
    
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
        union{
		uint8_t  data8;
		uint16_t data16;
		uint24_t data24;
		uint32_t data32;
        struct map_info  map;
	
       }data;
       
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

    uint8_t                  id; /*id */
    uint8_t                mode; /*pre-orintal etc.*/

    CanOpen_msg*    current_msg;

    uint8_t  (*init)(uint8_t id);
    uint8_t  (*receiving_message)(CanOpen_Msg *msg);
    uint8_t  (*sending_message)(CanOpen_Msg *msg);
    void     (*pdo_timer)(void);

    struct OD_Object*   map;
    
    struct PDO_Object**  pdo;
    struct SDO_Object**  sdo;

    uint16_t*     pdo_timers;  // !! -not quite right
    uint8_t  n_obj_timer;
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

#define ERR_MSG(err) msg->frame_sdo.data.data32 = error_msg[err];\
                     msg->frame_sdo.cmd = RESPONSE_ERROR;\
                     msg->frame_sdo.dlc = 8;
                     


/*---------- function  OD_TABLE  ------------*/


/*index 0000 -  0xFFFF -> the end)*/

struct OD_Object* OD_search_index(uint16_t index, struct OD_Object* tab){   
    while (tab->index < index){tab++;} 
    return tab = tab->index == index?tab:NULL;};

inline struct OD_Object* OD_search_msg_index(CanOpen_Msg *msg, struct OD_Object* tab){
    return  OD_search_index((msg->frame_sdo.index),tab);}

inline struct OD_object* OD_search_map_index(CanOpen_Msg *msg, struct OD_object* tab){
    return  OD_search_index(msg->frame_sdo.data.map.index,tab);}
	
	
void copy_data (uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
    
    if(!wdata || !rdata)return;
    nbit >>=3;
    for(uint8_t i=0;i<nbit;i++){*(wdata + i) = *(rdata+i);}
    
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
	

inline void copy_xPDO(uint8_t* wdata,uint8_t* rdata,uint8_t dlc){
    
    for(uint8_t i= 0; i< MAX_MAP_DATA ;i++){
        *(wdata+i)= i < dlc?*(rdata+i):0;}
    
};



/*----------   function PDO communication  -------------*/

#define PDO_DISABLED	0x80000000 // 0b1000 0000 0000 0000
#define PDO_is_TX       0x80 	   //0b1000 0000
#define PDO_calculate_n	0x700 // 0b111 0000 0000
#define PDO_mask_addr	0x7FF // 0b0111 1111 1111


#define MAX_MAP_NBIT MAX_MAP_DATA*8 //warning 8bit x n_byte


void map_object_check(struct PDO_Object* pdo){
   
   if(!pdo)return;
   
   uint8_t sum_nbit = 0, subindex = 0;
   union map_data* map = pdo->pdo_map->map;
   
   
   if(pdo->pdo_map->sub_index > MAX_MAP_DATA)
                    pdo->pdo_map->sub_index = MAX_MAP_DATA;

   while(subindex < (pdo->pdo_map->sub_index)){
       
       if(!(map + subindex)->info.index||
          !(map + subindex)->info.nbit ||
          !*((pdo->pdo_map->quick_mapping)+subindex) ) break;
       
       sum_nbit += (map + subindex)->info.nbit;
       if(sum_nbit > MAX_MAP_NBIT) break; 
       subindex ++;
   };
   pdo->n_byte_pdo_map = sum_nbit >>3;
   pdo->pdo_map->sub_index = subindex;
   
};

void process_the_RxPDO_message(struct PDO_Object* pdo){
    
    if(!pdo)return;
    
    struct PDO_Mapping* pdomap = pdo->pdo_map;
    uint8_t   num = 0, subindex = pdomap->sub_index,
	      sum_nbit = 0, nbit, *addr, *buffer= pdo->buffer;
              
    if(!subindex) return;
    if(subindex > MAX_MAP_DATA) subindex = MAX_MAP_DATA;
    
    while(num < subindex){
	 
	 nbit =((pdomap->map)+num)->info.nbit; 
	 sum_nbit += nbit;   
         if(sum_nbit > MAX_MAP_NBIT) break;
	 addr = *((pdomap->quick_mapping)+num);
         if(!addr) break;
	 buffer = copy_rdata_answer (addr,buffer,nbit); // object <- buffer
         num++;
	 
    };    
};

void copy_rxPDO_message_to_array(CanOpen_Msg* msg,struct PDO_Object* pdo){
    
    if(msg->can_frame.dlc < pdo->n_byte_pdo_map) return; 
    copy_xPDO(pdo->buffer,(uint8_t*)&msg->can_frame.data0,pdo->n_byte_pdo_map); //or 8?
    pdo->cond |= 0x02; // 2bit new message
    
};


void process_the_TxPDO_message(struct PDO_Object* pdo){
    
    if(!pdo)return;
    
    struct PDO_Mapping* pdomap = pdo->pdo_map;
    uint8_t   num = 0, subindex = pdomap->sub_index,
	      sum_nbit = 0, nbit, *addr, *buffer = pdo->buffer;
    
    if(!subindex) return;
    if(subindex > MAX_MAP_DATA) subindex = MAX_MAP_DATA;
    
        while(num < subindex){
	 
	 nbit =((pdomap->map)+num)->info.nbit; 
	 sum_nbit += nbit;   
         if(sum_nbit > MAX_MAP_NBIT) break;
	 addr = *((pdomap->quick_mapping)+num);
         if(!addr) break;
	 buffer = copy_wdata_answer (buffer,addr,nbit);// object -> buffer
         num++;
	 
    };   
};

void copy_txPDO_array_to_message(CanOpen_Msg* msg,struct PDO_Object* pdo){
    
    copy_xPDO ((uint8_t*)&msg->can_frame.data0, pdo->buffer, pdo->n_byte_pdo_map);// or 8?
    
};

inline void start_Inhibit_time(struct PDO_Object *pdo){
	
	if(pdo->counter_Inhibit_time && pdo->Inhibit_time){	    	
		*pdo->counter_Inhibit_time = pdo->Inhibit_time;    
		pdo->cond |= 0x40 ;} //set 1 Inhibit
};

inline void start_Event_timer(struct PDO_Object *pdo){
	
	if(pdo->counter_Event_timer && pdo->Event_timer){    
		*pdo->counter_Event_timer = pdo->Event_timer;    
		pdo->cond |= 0x08;} // set 
};


void init_xPDO(struct PDO_object* pdo){
    
    if(!pdo)return;
    
    pdo->cond.flag.lock = 1;
    pdo->cond.flag.new_msg = 0;
    
    if(pdo->cob_id&PDO_DISABLED)return;
    
    pdo->func->check_map(pdo);
    if(pdo->pdo_map->sub_index == 0)return;
    
    switch(pdo->Transmission_type){
    
	case 0xFF: start_Event_timer(pdo);
	case 0xFE: start_Inhibit_time(pdo); break;
	default: if(pdo->Transmission_type > 0xF0) return;
	         pdo->cond.flag.sync = pdo->Transmission_type;
		 break;
    };

    pdo->cond.flag.lock = 0;
};


struct func_pdo func_default={
    
    .init_pdo = init_xPDO,
    .check_map = map_object_check,
    .process_rxpdo = process_the_RxPDO_message,
    .process_txpdo = process_the_TxPDO_message,

};

void pdo_object_type(struct PDO_object *pdo){
    
    if(!pdo)return;
    
    if(pdo->cond.flag.lock) return;
    if(pdo->cond.flag.init_pdo)pdo->func->init_pdo(pdo);
    
    switch(pdo->Transmission_type){
 
/*PDO transmission is sent if the PDO data was changed by at least 1 bit.
  interval before sending PDO = inhibit_time * 100ms;
 */
        case 0xFF:
      
            if(pdo->cond.flag.rx_tx){
         //txpdo
                if(!pdo->cond.flag.event_txpdo)break;
		
                if(pdo->cond.flag.inhibit_time){	
			if(pdo->counter_Inhibit_time && 
			   *pdo->counter_Inhibit_time) break;
		   pdo->cond.flag.inhibit_time = 0;}
		
                if(pdo->cond.flag.event_timer){
			if(pdo->counter_Event_timer &&
			   *pdo->counter_Event_timer) break;
		   pdo->cond.flag.event_timer = 0;}
               
                if(pdo->func->process_txpdo)pdo->func->process_txpdo(pdo);
                
                pdo->cond.flag.event_txpdo=0;
                
		start_Inhibit_time(pdo);
		start_Event_timer (pdo);
		
                pdo->cond.flag.new_msg = 1;
                
            }else{
         //rxpdo       
                if(pdo->cond.flag.new_msg == 0)break;
                if(pdo->func->process_rxpdo)pdo->func->process_rxpdo(pdo);
                pdo->cond.flag.new_msg = 0;
            }
      
        break;
        
/* Cyclically  
 * Transfer PDO ,period  = inhibit_time * 100ms;
   Receive PDOs, on the other hand, are parsed immediately upon receipt.
 */
        case 0xFE: 
            
            if(pdo->cond.flag.rx_tx){
         //txpdo
                if(pdo->cond.flag.inhibit_time){	
			if(pdo->counter_Inhibit_time && 
			   *pdo->counter_Inhibit_time) break;
		   pdo->cond.flag.inhibit_time = 0;}
		
                if(pdo->func->process_txpdo)pdo->func->process_txpdo(pdo);
				
		start_Inhibit_time(pdo);
	
                pdo->cond.flag.new_msg = 1;
                
            }else{
         //rxpdo       
                if(pdo->cond.flag.new_msg == 0)break;
                if(pdo->func->process_rxpdo)pdo->func->process_rxpdo(pdo);
                pdo->cond.flag.new_msg = 0;
            }
            
        break; 
        
/* SYNC-Message
 0 - send only when an event occurs and SYNC
 
 */
        case 0x00:
            
             if(pdo->cond.flag.rx_tx){
         //txpdo  
                if(!pdo->cond.flag.sync)break;
                if(!pdo->cond.flag.event_txpdo)break;
		
                if(pdo->func->process_txpdo)pdo->func->process_txpdo(pdo);
                
		pdo->cond.flag.sync = 0;
                pdo->cond.flag.event_txpdo = 0;
                pdo->cond.flag.new_msg = 1;
                
            }else{
         //rxpdo
                if(!pdo->cond.flag.sync)break; 
                if(!pdo->cond.flag.new_msg)break; // +pdo->cond.flag.sync = 0;
		
                if(pdo->func->process_rxpdo)pdo->func->process_rxpdo(pdo);
                
		pdo->cond.flag.new_msg = 0;
                pdo->cond.flag.sync = 0;
                
            }
            
        break;  
        
    //1 -240 sent cyclically on SYNC 
        
        default:
            
            if(pdo->Transmission_type > 0xF0) break;
            
            if(pdo->cond.flag.rx_tx){
                
                //txpdo       
                if(!pdo->cond.flag.sync)break;
		
                if(pdo->func->process_txpdo)pdo->func->process_txpdo(pdo);
                
		pdo->cond.flag.sync = 0;
                pdo->counter_sync = pdo->Transmission_type;
                pdo->cond.flag.new_msg = 1;
                
            }else{
                //rxpdo
                if(!pdo->cond.flag.sync)break; 
                if(!pdo->cond.flag.new_msg)break;
		
                if(pdo->func->process_rxpdo)pdo->func->process_rxpdo(pdo);
                
		pdo->cond.flag.new_msg = 0;
                pdo->cond.flag.sync = 0;
                pdo->counter_sync = pdo->Transmission_type;
                
            }
        break;         
    } 
};	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	

#ifdef	__cplusplus
}
#endif

#endif	/* CANOPEN_LITE_H */

