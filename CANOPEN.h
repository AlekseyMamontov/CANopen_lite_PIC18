/* 
 * File:   CANopen.h
 * Author: Oleksii Mamontov
 * Created on  2022 
 */

#ifndef CANOPEN_H
#define	CANOPEN_H

	
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
#define IDENTITY		0x23
	
/*Object Dictionary TYPE Definitions cia301*/
#define OD_NULL 		0
/* Large variable amount of data e.g. executable program code */
#define OD_DOMAIN 		2
/*A multiple data field object where the data fields may be any combination of
simple variables. Sub-index 0 is of UNSIGNED8 and sub-index 255 is of
UNSIGNED32 and therefore not part of the RECORD data */
#define OD_RECORD 		9
/* Defines a new record type e.g. the PDO mapping structure at 21h*/
#define OD_DEFSTRUCT	6
/* Denotes a type definition such as a BOOLEAN, UNSIGNED16, FLOAT and so on*/
#define OD_DEFTYPE      5
/* A single value such as anUNSIGNED8, BOOLEAN, FLOAT,INTEGER16, VISIBLE STRING etc.*/
#define OD_VAR          7	
/* A multiple data field object where each data field is a simple variable of
the SAME basic data type e.g. array of UNSIGNED16 etc. Sub-index 0 is
of UNSIGNED8 and therefore not part of the ARRAY data*/
#define OD_ARRAY 		8

/*Attribute*/	

#define _CONST	0
#define RO      0b001
#define WO      0b010
#define RW      0b011
#define NO_MAP  0b10000000


/* structure */


struct map_info{
    
	uint8_t  nbit; 
	uint8_t  sub_index;
	uint16_t index;
    
};
	
struct data_object{
    
    void* data_object;
    void* rw_object;
    uint8_t request_type;
    uint8_t attribute;
    uint8_t nbit;
    uint8_t sub_index;
    uint32_t sub_index_ff;
    
};

struct OD_object{
    
    uint16_t index;
    void* data;
    void (*func_data)(struct data_object *obj);
    
};

struct one_type_array{
    
    uint8_t sub_index;
    void*   array;
    
};

struct arr_object{
    
    uint8_t  sub_index;
    uint8_t* nbit;
    void*    array;
    
};    
    
union map_data{
    
    struct map_info info;
	uint32_t        data32;
    
};

#define MAX_MAP_DATA 8

struct PDO_mapping {
    
    uint8_t           sub_index;
    union map_data    map[MAX_MAP_DATA];
    // quick access to the map
    void *            quick_mapping[MAX_MAP_DATA];
    struct OD_object* node_map;
    
};

union cond {
    
    struct{
    uint8_t sync       : 1; // sync
    uint8_t new_msg    : 1; // new message rxPDO
    uint8_t timer_event: 1; // 1 - the timer has worked
    uint8_t pause_send : 1; // 1 - pause is over
    uint8_t event_txpdo: 1; // 1 - there is a change to send a message
    uint8_t init_pdo   : 1; // 1 - xPDO init   
    uint8_t rx_tx      : 1; // 0 - rxPDO object, 1 - txPDO
    uint8_t lock       : 1; // 1 - Lock
   }       flag; 
   
    uint8_t stat;
    
};





struct PDO_object{
    
    union cond  cond;
    
    // visible block
    
    uint8_t		sub_index ;
    uint8_t		Transmission_type;
    uint8_t     Sync_start_value;
    
    uint32_t	cob_id ;
    
    uint16_t	Inhibit_time; 	
    uint16_t	Event_timer;
    
    // quick access to the structure map
    
    struct PDO_mapping* pdo_map;
    
    uint8_t     n_byte_pdo_map; // == msg->dlc? map_object_check()
    uint8_t     counter_sync;
    
    // function
    struct func_pdo  *func;
  //void *       node_parent;
    
    // Buffer 
    uint8_t      data[MAX_MAP_DATA];
    
};  

struct func_pdo{
    
    void   (*init_xpdo)(struct PDO_object* pdo);
    void   (*process_map)(struct PDO_object* pdo);
    void   (*process_rxpdo)(struct PDO_object *pdo);
    void   (*process_txpdo)(struct PDO_object *pdo);
    void   (*start_Inhibit_timer)(struct PDO_object *pdo);
    void   (*start_event_timer)(struct PDO_object *pdo);
    
};



struct SDO_object{
	
    uint8_t		sub_index;
    uint32_t	cob_id_client;
    uint32_t	cob_id_server;
    uint8_t		node_id;

};

union cob_id{
    
    uint32_t id;
    struct{
           uint8_t     id: 7;
           uint8_t f_code: 4;
           uint8_t       : 5;
           uint8_t       : 8;
           uint8_t       : 5;
           uint8_t frame : 1;
           uint8_t       : 1;
           uint8_t valid : 1;              
            }pdo;     
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
        struct 
        map_info map;
       }         data;  			
    }frame_sdo;
    
    struct {
        uint8_t  idType; 
        union {
            uint32_t id;
          struct{
            uint16_t func_id;
            uint8_t  none;
            uint8_t  status;
                 }       _id;
          struct{
            uint8_t id_node: 7;
            uint8_t  func: 4;
            uint8_t      : 5;
            uint8_t      : 8;
            uint8_t      : 5;
            uint8_t  ext : 1;
            uint8_t      : 1;
            uint8_t  lock: 1;                                
                 }      bit_id;        
              }  cob;	   
        uint8_t  dlc;	  
        uint8_t data0; 
        uint8_t data1;
        uint8_t data2; 
        uint8_t data3; 
        uint8_t data4; 
        uint8_t data5; 
        uint8_t data6; 
        uint8_t data7;  			
    }frame_pdo;
    
    uint8_t array[14];
    
}CanOpen_msg;

//  for no 8bit
//  #pragma pack(pop) 

#define COB_ID      array[1]&0x7f

#define MAX_PDO_OBJECT 8 //4x2

struct xCanOpen{

    uint8_t                  id; /*id */
    uint8_t                mode; /*pre-orintal etc.*/

    CanOpen_msg*    current_msg;

    uint8_t  (*init)(uint8_t id);
    uint8_t  (*receiving_message)(CanOpen_msg *msg);
    uint8_t  (*sending_message)(CanOpen_msg *msg);

    struct OD_object*   map;
    
    struct PDO_object*  pdo[MAX_PDO_OBJECT];
    struct SDO_object*  sdo[1];

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
                     msg->frame_sdo.dlc = 8;\
                     


/*---------- function  OD_TABLE  ------------*/

void (*Object_call)(struct data_object *obj);

/*index 0000 -  0xFFFF -> the end)*/

struct OD_object* OD_search_index(uint16_t index, struct OD_object* tab){   
    while (tab->index < index){tab++;} 
    return tab = tab->index == index?tab:NULL;};

struct OD_object* OD_search_msg_index(CanOpen_msg *msg, struct OD_object* tab){
return  OD_search_index((msg->frame_sdo.index),tab);}

struct OD_object* OD_search_map_index(CanOpen_msg *msg, struct OD_object* tab){
return  OD_search_index(msg->frame_sdo.data.map.index,tab);}


void copy_data (uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
    
    if(!wdata || !rdata)return;
    nbit >>=3;
    for(uint8_t i=0;i<nbit;i++){*(wdata + i) = *(rdata+i);}
    
};

void* copy_data_answer (uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
    
    if(!wdata || !rdata)return wdata;
    nbit >>=3;
    for(uint8_t i=0;i<nbit;i++){*(wdata + i) = *(rdata+i);}
    return (wdata+nbit);
    
};


/*----------   function PDO communication  -------------*/

#define PDO_DISABLED	0x80000000 // 0b1000 0000 0000 0000
#define PDO_is_TX       0x80 	   //0b1000 0000
#define PDO_calculate_n	0x700 // 0b111 0000 0000
#define PDO_mask_addr	0x7FF // 0b0111 1111 1111


#define MAX_MAP_NBIT MAX_MAP_DATA*8 //warning 8bit x n_byte


void map_object_check(struct PDO_object* pdo){
   
   uint8_t sum_nbit = 0, sub_index = 0;
   
   if(pdo->pdo_map->sub_index > MAX_MAP_DATA)
                    pdo->pdo_map->sub_index = MAX_MAP_DATA;

   while(sub_index < (pdo->pdo_map->sub_index)){
       
       if(!(pdo->pdo_map->map[sub_index].info.index)||
          !(pdo->pdo_map->map[sub_index].info.nbit )||
          !(pdo->pdo_map->quick_mapping[sub_index])) break;
       
       sum_nbit += pdo->pdo_map->map[sub_index].info.nbit;
       if(sum_nbit > MAX_MAP_NBIT) break; 
       sub_index ++;
   };
   pdo->n_byte_pdo_map = sum_nbit >>3;
   pdo->pdo_map->sub_index = sub_index;
   
};

void process_the_RxPDO_message(struct PDO_object* pdo){
    
    uint8_t  sum_nbit = 0,index =0,addr=0;
    uint8_t* data = pdo->data;
    
    if(!(pdo->pdo_map->sub_index))return;
    if(pdo->pdo_map->sub_index > MAX_MAP_DATA)
                    pdo->pdo_map->sub_index = MAX_MAP_DATA;
    
    while(index < pdo->pdo_map->sub_index){
    
        sum_nbit += pdo->pdo_map->map[index].info.nbit;
        if(sum_nbit > MAX_MAP_NBIT) break;
        if(!(pdo->pdo_map->quick_mapping[index])) break;
        
            switch(pdo->pdo_map->map[index].info.nbit){   
                case 0x08: *((uint8_t*)(pdo->pdo_map->quick_mapping[index])) = 
                            *(data+addr);addr++;break;
                case 0x10: *((uint16_t*)(pdo->pdo_map->quick_mapping[index])) = 
                            *((uint16_t*)(data+addr));addr +=2;break;
                case 0x18: *((uint24_t*)(pdo->pdo_map->quick_mapping[index])) = 
                            *((uint24_t*)(data+addr));addr +=3;break;
                case 0x20: *((uint32_t*)(pdo->pdo_map->quick_mapping[index])) = 
                            *((uint32_t*)(data+addr));addr +=4;break;
                default: return; break;
                
        }; index++;
    };
    
};

void process_the_TxPDO_message(struct PDO_object* pdo){
    
    uint8_t  sum_nbit = 0,index =0,addr=0;
    uint8_t* data = pdo->data;
    
    if(!(pdo->pdo_map->sub_index))return;
    if(pdo->pdo_map->sub_index > MAX_MAP_DATA)
                    pdo->pdo_map->sub_index = MAX_MAP_DATA;
    
    while(index < pdo->pdo_map->sub_index){
    
        sum_nbit += pdo->pdo_map->map[index].info.nbit;
        if(sum_nbit > MAX_MAP_NBIT) break;
        if(!(pdo->pdo_map->quick_mapping[index])) break;
        
            switch(pdo->pdo_map->map[index].info.nbit){   
                case 0x08: *(data+addr) = 
                            *((uint8_t*)(pdo->pdo_map->quick_mapping[index]));
                            addr++;break;
                case 0x10: *((uint16_t*)(data+addr))=
                            *((uint16_t*)(pdo->pdo_map->quick_mapping[index]));
                            addr +=2;break;
                case 0x18: *((uint24_t*)(data+addr))=
                           *((uint24_t*)(pdo->pdo_map->quick_mapping[index]));
                            addr +=3;break;
                case 0x20: *((uint32_t*)(data+addr))=
                           *((uint32_t*)(pdo->pdo_map->quick_mapping[index])); 
                            addr +=4;break;
                default: return; break;
                
        }; index++;
    };
    
};

void process_the_PDO_message(struct PDO_object* pdo){
    
    if(pdo->cond.flag.rx_tx){
        
        pdo->func->process_txpdo(pdo);
   }else 
        pdo->func->process_rxpdo(pdo);
    
};

inline void copy_xPDO(uint8_t* wdata,uint8_t* rdata,uint8_t dlc){
    
    for(uint8_t i= 0; i< MAX_MAP_DATA ;i++){
        *(wdata+i)= i < dlc?*(rdata+i):0;}
    
};


void copy_rxPDO_message_to_array(CanOpen_msg* msg,struct PDO_object* pdo){
    
    if(msg->can_frame.dlc < pdo->n_byte_pdo_map) return;
    copy_xPDO (pdo->data,&msg->frame_pdo.data0,pdo->n_byte_pdo_map); //or 8?
    pdo->cond.flag.new_msg = 1; // new message
    
};

void copy_txPDO_array_to_message(CanOpen_msg* msg,struct PDO_object* pdo){
    
    copy_xPDO (&msg->frame_pdo.data0,pdo->data,msg->frame_pdo.dlc);// or 8?
    
};

void init_xPDO(struct PDO_object* pdo){
    
    if(!pdo)return;
    
    if(pdo->cob_id&PDO_DISABLED){ pdo->cond.flag.lock = 1; return;}
    pdo->cond.flag.lock = 0;
    
    map_object_check(pdo);
    
    if(pdo->Transmission_type > 0xFE){ pdo->cond.flag.sync =0;return;}
    
    if(pdo->Inhibit_time){
        pdo->cond.flag.pause_send = 0;
        pdo->func->start_Inhibit_timer(pdo);}
    
    if(pdo->Event_timer){
        pdo->cond.flag.timer_event =0;
        pdo->func->start_event_timer(pdo);}
    
};



void pdo_object_type(struct PDO_object *pdo){
    
    if(!pdo)return;
    if(pdo->cond.flag.lock) return;

    switch(pdo->Transmission_type){
 
/*PDO transmission is sent if the PDO data was changed by at least 1 bit.
  interval before sending PDO = inhibit_time * 100ms;
 */
        case 0xFF:
      
            if(pdo->cond.flag.rx_tx){
         //txpdo
                pdo->cond.flag.new_msg = 0;
                if(!pdo->cond.flag.event_txpdo)break;
                if(!pdo->cond.flag.pause_send)break;
                if(pdo->Event_timer && !pdo->cond.flag.timer_event)break;
               
                if(pdo->func->process_txpdo)pdo->func->process_txpdo(pdo);
                
                pdo->cond.flag.event_txpdo=0;
                pdo->cond.flag.pause_send = 0;
                pdo->cond.flag.timer_event =0;
                pdo->cond.flag.new_msg = 1;
                
                pdo->func->start_Inhibit_timer(pdo);
                pdo->func->start_event_timer(pdo);
                
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
                pdo->cond.flag.new_msg = 0;
                if(!pdo->cond.flag.pause_send)break;        
                if(pdo->func->process_txpdo)pdo->func->process_txpdo(pdo);
                pdo->cond.flag.pause_send = 0;
                //...start pause timer if inhibit_time !=0
                pdo->func->start_Inhibit_timer(pdo);
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
                pdo->cond.flag.new_msg = 0; 
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
                
                pdo->cond.flag.new_msg = 0;  //txpdo       
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

#define SDO_ANSWER_1b   msg->frame_sdo.cmd = RESPONSE_1b;\
                        msg->frame_sdo.dlc = 5;\

#define SDO_ANSWER_2b   msg->frame_sdo.cmd = RESPONSE_2b;\
                        msg->frame_sdo.dlc = 6;\
                         
#define SDO_ANSWER_3b   msg->frame_sdo.cmd = RESPONSE_3b;\
                        msg->frame_sdo.dlc = 7;\
                         
#define SDO_ANSWER_4b   msg->frame_sdo.cmd = RESPONSE_4b;\
                        msg->frame_sdo.dlc = 8;\

#define SDO_SAVE_OK     msg->frame_sdo.cmd = OK_SAVE;\
                        msg->frame_sdo.dlc = 4;\



uint8_t check_sdo_command_for_writing(CanOpen_msg *msg,uint8_t nbit){
    
    uint8_t dlc = 4+(nbit >> 3);     
    uint8_t cmd = 8-dlc;
    cmd <<=2;
    cmd |=0x23;
    if(msg->frame_sdo.cmd != cmd) return ERROR_SDO_SERVER;
    if(msg->frame_sdo.dlc < dlc) return ERROR_SMALL_DATA_OBJ;
    SDO_SAVE_OK
    return 0;
    
} 

uint8_t check_sdo_command_for_reading(CanOpen_msg *msg,uint8_t nbit){
    
    if(msg->frame_sdo.cmd != READ_REQUEST) return ERROR_SDO_SERVER;
    if(msg->frame_sdo.dlc < 4) return ERROR_SMALL_DATA_OBJ;
    uint8_t dlc = 4+(nbit >> 3);
    msg->frame_sdo.dlc = dlc;
    msg->frame_sdo.cmd = 8-dlc;
    msg->frame_sdo.cmd <<=2;
    msg->frame_sdo.cmd |= 0x43;   
    return 0;
    
}


/*------------ function Object proccesing -------*/

#define SDO_request       0
#define MAP_txpdo_request 1
#define MAP_rxpdo_request 2
#define MAP_info          3

void single_object(struct data_object *obj){
   
    uint8_t  nbit,*wdata,*rdata;
    CanOpen_msg* msg;
    if(!obj)return;
    
    switch(obj->request_type){
        
       case SDO_request:
           
            msg = obj->rw_object;
            if(!msg) return;
            uint8_t error = ERROR_SDO_SERVER;
            wdata = &msg->frame_sdo.data.data8;
            //read
            if(msg->frame_sdo.cmd == READ_REQUEST){	
                if(obj->attribute&RO){
                error = check_sdo_command_for_reading(msg,obj->nbit);
                if(!error){ 
                    switch(msg->frame_sdo.subindex){
                        case 0:rdata =(uint8_t*) obj->data_object;
                               nbit  = obj->nbit;break;              
                        case 0xFF:rdata = (uint8_t*)&(obj->sub_index_ff);
                                  nbit = 0x20; SDO_ANSWER_4b;break;         
                       default:error = ERROR_SUB_INDEX;break;};		
                 }}else error = ERROR_NO_READ;	
            //write    
            }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){
				if(obj->attribute&WO){
				error = check_sdo_command_for_writing(msg,obj->nbit);
				if(!error){ 
                    switch(msg->frame_sdo.subindex){
                       case 0:rdata = wdata;
                              wdata = (uint8_t*)obj->data_object;
							  nbit  = obj->nbit;break;       
                       case 0xFF:rdata = (uint8_t*)&(obj->sub_index_ff);
                                 nbit = 0x20; SDO_ANSWER_4b;break;         
                        default:error = ERROR_SUB_INDEX;break;};	
				}}else error = ERROR_NO_SAVE;
		  }          
          if(error){  rdata = (uint8_t*)&error_msg[error];
                      nbit = 0x20;
                      msg->frame_sdo.cmd = RESPONSE_ERROR;
                      msg->frame_sdo.dlc = 0x08;}
          copy_data(wdata,rdata,nbit);  
          break;         
          
        case MAP_txpdo_request: 
            if(!(obj->attribute & RO)) break;
            if(obj->sub_index) break;
            copy_data(obj->rw_object,obj->data_object,obj->nbit);break;
            
        case MAP_rxpdo_request: 
            if(!(obj->attribute & WO)) break;
            if(obj->sub_index) break;
            copy_data(obj->data_object,obj->rw_object,obj->nbit);break;
            
       /*response data_object | answer rw_data  *addr_objecta,nbit,attr;NULL - no object.
          * for fast processing mapping*/     
        case MAP_info:
            obj->rw_object = obj->sub_index?NULL: obj->data_object;break;
        default:break;
     }
    
};

void one_type_array_object(struct data_object *obj){
	
    uint8_t nbit,*wdata,*rdata,error;
    CanOpen_msg* msg;
    if(!obj)return;
    struct one_type_array* array = obj->data_object;
    if(!array) return;
    
    switch(obj->request_type){
		
       case SDO_request:
            msg = obj->rw_object;
            if(!msg)return;
            error = ERROR_SDO_SERVER;
            wdata =(uint8_t*) &msg->frame_sdo.data;
     //read
            if(msg->frame_sdo.cmd == READ_REQUEST){	
                if(obj->attribute&RO){
                error = check_sdo_command_for_reading(msg,obj->nbit);
                if(!error){ 
                    switch(msg->frame_sdo.subindex){
                        case 0:rdata = &array->sub_index;;
                               nbit  = 0x08;SDO_ANSWER_1b;break;              
                        case 0xFF:rdata = (uint8_t*)&(obj->sub_index_ff);
                                  nbit = 0x20; SDO_ANSWER_4b;break;         
                        default: if(array->sub_index < (msg->frame_sdo.subindex))
											error = ERROR_SUB_INDEX;break;
                                nbit = obj->nbit;
								rdata = (uint8_t*)array->array;
								rdata = rdata + ((nbit>>3)*((msg->frame_sdo.subindex)-1));
                        break;}		
                 }
             }else error = ERROR_NO_READ;	
    //write    
            }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){
				if(obj->attribute&WO){
				error = check_sdo_command_for_writing(msg,obj->nbit);
				if(!error){ 
                    switch(msg->frame_sdo.subindex){
                       case 0:error = ERROR_NO_SAVE;break;       
                       case 0xFF:rdata = (uint8_t*)&(obj->sub_index_ff);
                                 nbit = 0x20; SDO_ANSWER_4b;break;         
                       default:if(array->sub_index < (msg->frame_sdo.subindex))
												error = ERROR_SUB_INDEX;break;
								rdata = wdata;
								wdata = (uint8_t*)array->array;
								wdata = wdata + ((nbit>>3)*((msg->frame_sdo.subindex)-1));		
						;}      	
				}
			 }else error = ERROR_NO_SAVE;
		  }          
          if(error){rdata = (uint8_t*)&error_msg[error];
                      nbit = 0x20;
                      msg->frame_sdo.cmd = RESPONSE_ERROR;
                      msg->frame_sdo.dlc = 0x08;}
          copy_data(wdata,rdata,nbit);            
          break;         
         
        /*not a quick method read write mapping -> func_data 
          data_object rw_object  sub_index & nbit*/ 
          
        case MAP_txpdo_request:
        
            if(!(obj->attribute & RO))break;
            rdata = array->array;
            if(!rdata) break;
            switch(obj->sub_index){
                case 0:rdata =&array->sub_index;nbit=0x08;break; 
                case 0xFF: rdata=(uint8_t*)&(obj->sub_index_ff);nbit=0x20;break;
                default:if(array->sub_index < obj->sub_index){nbit= 0;break;};
                        nbit = obj->nbit;
                        rdata=rdata + ((nbit>>3)*((obj->sub_index)-1));
                    break;}
			copy_data(obj->rw_object,rdata,nbit);		
            break;
            
        case MAP_rxpdo_request: 
        
            if(!(obj->attribute & WO)) return;
            wdata =(uint8_t*) array->array;
            if(!wdata) break;
            wdata = (!(obj->sub_index)|| obj->sub_index == 0xFF)? NULL : 
					wdata + ((nbit>>3)*((obj->sub_index)-1));
                    nbit = obj->nbit;
			copy_data(wdata,obj->rw_object,nbit);		
            break;
  
         /*response data_object | answer rw_data  *addr_objecta,nbit,attr;NULL - no object.
          * for fast processing mapping*/ 
            
        case MAP_info:
        
			if(!obj->data_object){obj->rw_object= NULL;return;} 
			switch(obj->sub_index){
				 case 0: rdata = &array->sub_index;nbit = 0x08;obj->attribute = RO;break;
				 case 0xFF : rdata = (uint8_t*)&(obj->sub_index_ff); nbit = 0x20;obj->attribute = RO;break;
				 default: if(obj->sub_index > array->sub_index) rdata=NULL;nbit=0;break;
						  rdata = (uint8_t*)array->array;
						   nbit = obj->nbit;
						  rdata = rdata?rdata + ((nbit>>3)*((obj->sub_index)-1)):NULL;
			};
			obj->rw_object = rdata;obj->nbit = nbit;return;
            break;       
      default:break;
     }
    
};

/* bit,attribute,data.type*/
#define OBJ_ATTR(b,a,f,t) obj->nbit = b;obj->attribute=a;obj->sub_index_ff = (f << 8)| t ;

/* 1 byte object*/
void ro_object_1byte(struct data_object *obj){
     OBJ_ATTR(0x08,RO,UINT8,OD_VAR) single_object(obj);}
void rw_object_1byte(struct data_object *obj){
     OBJ_ATTR(0x08,RW,UINT8,OD_VAR) single_object(obj);}
void ro_array_1byte(struct data_object *obj){
     OBJ_ATTR(0x08,RW,UINT8,OD_ARRAY); one_type_array_object(obj);}
void rw_array_1byte(struct data_object *obj){
     OBJ_ATTR(0x08,RW,UINT8,OD_ARRAY); one_type_array_object(obj);}

/* 2 byte object*/
void ro_object_2byte(struct data_object *obj){
     OBJ_ATTR(0x10,RO,UINT16,OD_VAR) single_object(obj);}
void rw_object_2byte(struct data_object *obj){
     OBJ_ATTR(0x10,RW,UINT16,OD_VAR) single_object(obj);}
void ro_array_2byte(struct data_object *obj){
     OBJ_ATTR(0x10,RO,UINT16,OD_ARRAY); one_type_array_object(obj);}
void rw_array_2byte(struct data_object *obj){
     OBJ_ATTR(0x10,RW,UINT16,OD_ARRAY); one_type_array_object(obj);}

/* 3 byte object*/
void ro_object_3byte(struct data_object *obj){
     OBJ_ATTR(0x18,RO,UINT24,OD_VAR) single_object(obj);}
void rw_object_3byte(struct data_object *obj){
     OBJ_ATTR(0x18,RW,UINT24,OD_VAR) single_object(obj);}
void ro_array_3byte(struct data_object *obj){
     OBJ_ATTR(0x18,RO,UINT24,OD_ARRAY); one_type_array_object(obj);}
void rw_array_3byte(struct data_object *obj){
     OBJ_ATTR(0x18,RW,UINT24,OD_ARRAY); one_type_array_object(obj);}

/* 4 byte object*/
void ro_object_4byte(struct data_object *obj){
     OBJ_ATTR(0x20,RO,UINT32,OD_VAR) single_object(obj);}
void rw_object_4byte(struct data_object *obj){
     OBJ_ATTR(0x20,RW,UINT32,OD_VAR) single_object(obj);}
void ro_array_4byte(struct data_object *obj){
     OBJ_ATTR(0x20,RO,UINT32,OD_ARRAY); one_type_array_object(obj);}
void rw_array_4byte(struct data_object *obj){
     OBJ_ATTR(0x20,RW,UINT32,OD_ARRAY); one_type_array_object(obj);}



/* ------------------------- pdo_object ------------------------ */

void pdo_object(struct data_object *obj){
    
  if(!obj)return;
  uint8_t error;
  CanOpen_msg *msg;
  struct PDO_object *pdo = (struct PDO_object *)obj->data_object;
    
  switch(obj->request_type){
		
    case SDO_request:
        
        error = ERROR_SDO_SERVER;
        msg = obj->rw_object;
        if(!msg)return;
 //read
        if(msg->frame_sdo.cmd == READ_REQUEST){	
          if(obj->attribute&RO){    
            error = 0;
            if(msg->frame_sdo.dlc < 4) error = ERROR_SMALL_DATA_OBJ;
            if(!pdo) error = ERROR_SYSTEM;                
            if(!error){      
              switch(msg->frame_sdo.subindex){                 
                case 0:msg->frame_sdo.data.data8  = pdo->sub_index;SDO_ANSWER_1b break;
                case 1:msg->frame_sdo.data.data32 = pdo->cob_id;SDO_ANSWER_4b break;
                case 2:msg->frame_sdo.data.data8  = pdo->Transmission_type;SDO_ANSWER_1b break;
                case 3:msg->frame_sdo.data.data16 = pdo->Inhibit_time;SDO_ANSWER_2b break;                    
                case 5:msg->frame_sdo.data.data16 = pdo->Event_timer;SDO_ANSWER_2b break;
                case SUB_INDEX_FF:msg->frame_sdo.data.data32 =(PDO_COMM << 8)|OD_DEFSTRUCT ;
                                  SDO_ANSWER_4b break;
                default: error = ERROR_SUB_INDEX;break;}}    
          }else error = ERROR_NO_READ;
// write
       }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){ 
          if(obj->attribute&WO){
          error = 0;      
          if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ;
          if(!pdo) error = ERROR_SYSTEM;
          if(!error){      
            uint8_t lock = pdo->cob_id&PDO_DISABLED?1:0;
            switch(msg->frame_sdo.subindex){
                
        // sub-index
                
            case 0: error = ERROR_NO_SAVE;break;
            
        // sub-id 32bit
            case 1:
                
              if(msg->frame_sdo.cmd != GET_4b){error = ERROR_SDO_SERVER;break;}  
              if(msg->frame_sdo.dlc < 8){error = ERROR_SMALL_DATA_OBJ;break;}
              if(!pdo->cond.flag.lock){error = ERROR_NO_SAVE;break;}
                      // do the right processing!!! 
                 pdo->cob_id = msg->frame_sdo.data.data32&0x800007FF;
                 pdo->cond.flag.lock = pdo->cob_id&PDO_DISABLED?1:0;
                 if(!pdo->cond.flag.lock)pdo->cond.flag.init_pdo = 1;
        
               break;
               
        // Transmission_type 8bit
            case 2: 
                
              if(!pdo->cond.flag.lock){error = ERROR_NO_SAVE;break;} 
              if(msg->frame_sdo.cmd != GET_1b){error = ERROR_SDO_SERVER;break;}  
                 if(msg->frame_sdo.dlc < 5){error = ERROR_SMALL_DATA_OBJ;break;}
            
                 pdo->Transmission_type = msg->frame_sdo.data.data8;
                 
                 pdo->cond.flag.init_pdo = 1;

                break;
                
        // Inhibit_time; 16bit
            case 3: 
                
              if(!pdo->cond.flag.lock){error = ERROR_NO_SAVE;break;}
                if(msg->frame_sdo.cmd != GET_2b){error = ERROR_SDO_SERVER;break;} 
                if(msg->frame_sdo.dlc < 6){error = ERROR_SMALL_DATA_OBJ;break;} 
              
                  pdo->Inhibit_time = msg->frame_sdo.data.data16;                 
            
                break;
                
        // Event_timer 16bit
            case 5:  
                
              if(!pdo->cond.flag.lock){error = ERROR_NO_SAVE;break;} 
                if(msg->frame_sdo.cmd != GET_2b){error = ERROR_SDO_SERVER;break;} 
                if(msg->frame_sdo.dlc < 6){error = ERROR_SMALL_DATA_OBJ;break;}
              
                  pdo->Event_timer = msg->frame_sdo.data.data16;                
                                 
                 break;
                 
            default:error = ERROR_SUB_INDEX;break;}}       
            }else error = ERROR_NO_SAVE;
          if(!error){SDO_SAVE_OK;}
        }
         if(error)ERR_MSG(error);
        
           break; 
           
    case MAP_txpdo_request:
           break;
           
    case MAP_rxpdo_request:
           break;
           
    case MAP_info:
        
          switch(obj->sub_index){
                case 0: obj->rw_object = &(pdo->sub_index);error = 0x08;break;
                case 1: obj->rw_object = &(pdo->cob_id);error = 0x20;break;       
                case 2: obj->rw_object = &(pdo->Transmission_type);error = 0x08;break;
                case 3: obj->rw_object = &(pdo->Inhibit_time);error = 0x10;break;       
                case 5: obj->rw_object = &(pdo->Event_timer);error = 0x10;break;
                default:obj->rw_object = NULL; error = 0x00;break;}
           obj->nbit = error;
           break;
    };  
};


void ro_pdo_object_(struct data_object *obj){obj->attribute=RO;pdo_object(obj);}
void rw_pdo_object_(struct data_object *obj){obj->attribute=RW;pdo_object(obj);}


/*------------------------- pdo_map object ------------------------------*/

void map_object(struct data_object *obj){
    
  if(!obj)return;
  CanOpen_msg *msg;
  uint8_t lock,error;
  struct PDO_object *pdo = (struct PDO_object *)obj->data_object;

  switch(obj->request_type){		
    
    case SDO_request:
        
      msg = (CanOpen_msg *)obj->rw_object;
      if(!msg)return;
      error = ERROR_SDO_SERVER;
// read      
      if(msg->frame_sdo.cmd == READ_REQUEST){	
        if(obj->attribute&RO){
           error = 0;
           if(msg->frame_sdo.subindex > MAX_MAP_DATA) error = ERROR_SUB_INDEX;
           if(msg->frame_sdo.dlc < 4) error = ERROR_SMALL_DATA_OBJ;
           if(!pdo) error = ERROR_SYSTEM;         
           if(!error){ 
              switch(msg->frame_sdo.subindex){
                case 0:  msg->frame_sdo.data.data8  = pdo->pdo_map->sub_index;
                         SDO_ANSWER_1b;break;
                case 0xFF: msg->frame_sdo.data.data32 = (PDO_MAPPING << 8)| OD_DEFSTRUCT;
                           SDO_ANSWER_4b break;       
                default: msg->frame_sdo.data.data32 = 
                         pdo->pdo_map->map[(msg->frame_sdo.subindex)-1].data32;
                         SDO_ANSWER_4b ; break;}}        
       }else error = ERROR_NO_READ;
// write               
    }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){ 
       if(obj->attribute&WO){
         error = 0; 
         if(msg->frame_sdo.subindex >MAX_MAP_DATA) error = ERROR_SUB_INDEX;
         if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ; 
         if(!pdo) error = ERROR_SYSTEM;  
         if(!error){
            if(pdo->cond.flag.lock){  
              switch(msg->frame_sdo.subindex){
              case 0:
                 if(msg->frame_sdo.cmd != GET_1b){error = ERROR_SDO_SERVER;break;}              
                 pdo->pdo_map->sub_index = msg->frame_sdo.data.data8<=MAX_MAP_DATA?
                 msg->frame_sdo.data.data8:MAX_MAP_DATA;
                 map_object_check(pdo); //test->map
                 break;
              default: 
                 if(msg->frame_sdo.dlc < 8){error = ERROR_SMALL_DATA_OBJ;break;};
                 if(msg->frame_sdo.cmd != GET_4b){error = ERROR_SDO_SERVER; break;}
                 if(!(msg->frame_sdo.data.map.nbit)){error = ERROR_SMALL_DATA_OBJ;break;}
          
                   struct OD_object *tab; 
                   void (*object_call)(struct data_object *obj);
                        
                   tab = OD_search_map_index(msg,pdo->pdo_map->node_map);
                   if(!tab || !tab->func_data || !tab->data){
                                            error = ERROR_NO_OBJECT;break;}                       
                   struct data_object  info;
                   info.sub_index = msg->frame_sdo.data.map.sub_index;
                   info.data_object = tab->data;
                   info.request_type = MAP_info;
                   object_call = tab->func_data;
                   object_call (&info);
                   if(info.rw_object == NULL){error = ERROR_SUB_INDEX;break;}
                   if(!(info.attribute&NO_MAP)){ error = ERROR_OBJECT_PDO;break;}
                   
                   /*
                    if(pdo->cob_id&PDO_is_TX){
                       if(!(info.access&RO)){error = ERROR_OBJECT_PDO;break;} 
                    }else if(!(info.access&WO)){error = ERROR_OBJECT_PDO;break;}
                   */
                   
                   if(pdo->cond.flag.rx_tx){ //tx = 1
                        if(!(info.attribute&RO)){error = ERROR_OBJECT_PDO;break;}
                  }else if(!(info.attribute&WO)){error = ERROR_OBJECT_PDO;break;}
                      
                   if(info.nbit != msg->frame_sdo.data.map.nbit){
                                          error = ERROR_LEN_OBJECT;break;}    
                   pdo->pdo_map->map[(msg->frame_sdo.subindex)-1].data32=
                       msg->frame_sdo.data.data32 ;      
                   pdo->pdo_map->quick_mapping[(msg->frame_sdo.subindex)-1] = 
                           info.rw_object;
                   
                   map_object_check(pdo); //test mapping object
                   
                break;}  
        }else{error = ERROR_NO_SAVE;};
      }  
   }}
    if(error)ERR_MSG(error);               
    break;       
    
    case MAP_txpdo_request:
      break;
      
    case MAP_rxpdo_request:
      break;
      
    case MAP_info:       
      switch(obj->sub_index){
        case 0:obj->rw_object = &(pdo->pdo_map->sub_index);
               obj->nbit = 0x08;break;
        default: if(obj->sub_index <=MAX_MAP_DATA){
                    obj->rw_object = &(pdo->pdo_map->map[(obj->sub_index)-1].data32);
                    obj->nbit = 0x20;
                }else{obj->rw_object = NULL;obj->nbit = 0;};
        break;}
     break;};
}

void ro_map_object_(struct data_object *obj){obj->attribute=RO;pdo_object(obj);}
void rw_map_object_(struct data_object *obj){obj->attribute=RW;pdo_object(obj);}


/* ------------------ STRUCT CAN NODE ------------------*/

#define n_FUNC_COMMAND 0x0f


/* ------------ function obiect call -----------*/


void NOP_call(uint8_t code,void* data){};


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
	
#define  BOOT               0x00
#define  STOPPED            0x04
#define  OPERATIONAL        0x05
#define  PRE_OPERATIONAL	0x7F

void NMT_message_processing(struct xCanOpen *node){
    
        if ((0x7F & node->current_msg->can_frame.id) == 0){
             
            if(node->current_msg->can_frame.data1 == 0 ||
               node->current_msg->can_frame.data1 == node->id){
                        
                switch(node->current_msg->can_frame.data0){
                        
                    case START_REMOTE_NODE:     node->mode = OPERATIONAL;break;
                    case STOP_REMOTE_NODE:      node->mode = STOPPED ;break;  
                    case ENTER_PRE_OPERATIONAL: node->mode = PRE_OPERATIONAL;break;
                    case RESET_APPLICATION:     node->mode = BOOT; break;
                    case RESET_COMMUNICATION:   node->mode = BOOT; break;
                    default:break;                               
                };
            };                    
        };
};
/*---------------------- Sync -----------------------*/

void SYNC_message_processing(struct xCanOpen *node){  
    
    for(uint8_t n=0; n<MAX_PDO_OBJECT; n++){
      if(node->pdo[n] == NULL) continue;
      if(node->pdo[n]->Transmission_type > 240)continue;
      if(node->pdo[n]->counter_sync)node->pdo[n]->counter_sync--;
      if(node->pdo[n]->counter_sync == 0) node->pdo[n]->cond.flag.sync = 1; 
    };   
};
    
/*--------------------- Time ------------------------*/

void TIME_message_processing(void* data){
    
    struct xCanOpen *node = (struct xCanOpen *)data;
    
};


/*--------------------- RxPDO ------------------------*/

void rxPDO_message_processing(uint8_t code,struct xCanOpen *node){
    
    struct PDO_object* pdo = node->pdo[code];
    
    if(code > 7) return;
    if(node->mode != OPERATIONAL) return;
    if(pdo->cond.flag.lock) return;
    
    copy_rxPDO_message_to_array(node->current_msg,pdo);
    
};

/*-------------------  RxSDO  -----------------------*/
    
void rxSDO_message_processing(uint8_t code,struct xCanOpen* node){// client -> server 
    
    void (*object_call)(struct data_object *);;
    struct SDO_object* sdo = node->sdo[code];
    struct OD_object*  tab;
    
    if(node->mode != OPERATIONAL || node->mode != PRE_OPERATIONAL) return;
    if(sdo->cob_id_server&PDO_DISABLED) return;
    if(node->current_msg->frame_sdo.id != sdo->cob_id_server) return;
    
    tab =  OD_search_msg_index(node->current_msg,node->map);
    if(!tab || !(tab->func_data) || !(tab->data)) return;
    
    struct data_object info;
    info.request_type = SDO_request;
    info.data_object = tab->data;
    object_call = tab->func_data;
    
    object_call(&info);
    
    node->current_msg->frame_sdo.id = sdo->cob_id_server;
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



void NODE_message_processing(struct xCanOpen* node){

    if(node->receiving_message(node->current_msg) == 0) return;
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

void Processing_pdo_objects(struct xCanOpen* node){

    if(node->mode != OPERATIONAL) return;
    
   for(uint8_t i=0;i<MAX_PDO_OBJECT;i++){
       
       if(node->pdo[i] != NULL){     
           pdo_object_type(node->pdo[i]);
           // tx?
           if(node->pdo[i]->cond.flag.rx_tx && node->pdo[i]->cond.flag.new_msg){
               
              node->current_msg->frame_pdo.cob.id = node->pdo[i]->cob_id; 
              node->current_msg->frame_pdo.dlc = node->pdo[i]->n_byte_pdo_map;
              node->current_msg->frame_pdo.idType = dSTANDARD_CAN_MSG_ID_2_0B;
              copy_txPDO_array_to_message(node->current_msg,node->pdo[i]);
              node->sending_message(node->current_msg);// while()?
              node->pdo[i]->cond.flag.new_msg = 0;
              
           };
       };
   }; 
   
};

void pdo_objects_add_id(struct xCanOpen* node){

   for(uint8_t i=0;i<MAX_PDO_OBJECT;i++){  
       if(node->pdo[i] != NULL) {
           node->pdo[i]->cob_id |= node->id;
           node->pdo[i]->cond.flag.init_pdo = 1;
       } 
   };
   
};




#endif	/* CANOPEN_H */

/*
switch(obj->request_type){
		
       case SDO_request:
            uint8_t error = ERROR_SDO_SERVER;
            if(msg->frame_sdo.cmd == READ_REQUEST){	
                if(obj->attribute&RO){    
            
                    
                    
                    
                }else error = ERROR_NO_READ;
                
            }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){ 
                if(obj->attribute&WO){
                    
                    
                    
                    
                }else error = ERROR_NO_SAVE;   
            }
                    
           break;       
       case MAP_read_request:
           break;
       case MAP_write_request:
           break;
       case MAP_info:
           break;
    };

 
 */