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

struct Info_Object{    
    uint8_t nbit;
    uint8_t access;     
    /* request type -> sub_index
     * answer type = 0  no sub_index */   
    uint8_t sub_type;
    /* object_code = 0 -no data */
    uint8_t obj_code;
};
	
/**/
	
#define PDO_DISABLED	0x80000000 // 0b1000 0000 0000 0000
#define PDO_is_transmit	0x80 	   //0b1000 0000
#define PDO_calculate_n	0x700 // 0b111 0000 0000
#define PDO_mask_addr	0x7FF // 0b0111 1111 1111


#define FUNC_MSG        0x01
#define FUNC_SDO_READ   0x02
#define FUNC_SDO_SAVE   0x03
#define FUNC_SYSTEM     0x04



//////////////////// ERROR ////////////////

#define OK_SAVE         0x60
#define RESPONSE_ERROR  0x80		

uint32_t error_msg[]={

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
#define ERROR_NO_CORRECT  11   
0x06090031,
#define ERROR_SYSTEM      12
0x06040047,
#define ERROR_EQUIPMENT   13
0x06060000,
#define ERROR_TOGGLE_BIT  14
0x05030000,
#define ERROR_SDO_SERVER  15
0x05040001,
#define ERROR_DATA        16
0x06090030,
#define ERROR_BIG_DATA_OBJ 17
0x06090031,
#define ERROR_SMALL_DATA_OBJ  18
0x06090032,
#define ERROR_ALL_OD_TABLE 19
0x08000000,
#define ERROR_OBJ_DICT    20
0x80000023,
};

#define ERR_MSG(err) msg->frame_sdo.data.data32 = error_msg[err];\
                     msg->frame_sdo.cmd = RESPONSE_ERROR;\
                     msg->frame_sdo.dlc = 8;\
                     

// ---------------- struct msg --------------------	

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
	    } data;  			
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


/*----------------  OD_TABLE  --------------------*/

struct OD_object{
    uint16_t index;
    void* data;
    void (*func_data)(CanOpen_msg *msg,void *obj);
};

/*index 0000 -  0xFFFF -> the end)*/
struct 
OD_object* OD_search_index(CanOpen_msg *msg, struct OD_object* tab){
    
    uint16_t index = msg->frame_sdo.index;
    
    while (tab->index != index){ 
        if(tab->index > index){tab = NULL; break;}
        tab++;}
    
 return tab;};





/*--------------PDO COMMUNICATION-----------------*/

 
struct map_info{
	uint8_t  nbit; 
	uint8_t  sub_index;
	uint16_t index;
	};
union map_data{
    struct map_info info;
	uint32_t        data32;
};

#define MAX_MAP_DATA 8

struct PDO_mapping {
    uint8_t                sub_index;
    union map_data         map[MAX_MAP_DATA];
    // quick access to the map
    void *                 quick_mapping[MAX_MAP_DATA];
    struct OD_object*      node_map;
};

struct PDO_object{

uint32_t	cob_id ;
uint16_t	Inhibit_time; 	
uint16_t	event_timer;
uint8_t		Transmission_type;
uint8_t		none;
uint8_t		sub_index ;

// quick access to the structure map
struct 
PDO_mapping* pdo_map;

uint8_t     status;
uint8_t     data[8];
void *      node_parent;
};

#define PDO_INIT 01



/////////////////   SDO COMMUNICATION   ////////////

struct SDO_object{
	
uint8_t		sub_index;
uint32_t	cob_id_client;
uint32_t	cob_id_server;
uint8_t		node_id;
};

#define RESPONSE_1b  0x4f
#define RESPONSE_2b  0x4B
#define RESPONSE_3b  0x47
#define RESPONSE_4b  0x43

#define GET_1b  		0x2F
#define GET_2b  		0x2B
#define GET_3b  		0x27
#define GET_4b  		0x23

#define READ_REQUEST  0x40
#define WRITE_REQUEST 0x20

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

/* ------------- STRUCT CAN NODE ------------*/

struct xCanOpen{

uint8_t              cob_id; /*id */
uint8_t                mode; /*pre-orintal etc.*/

CanOpen_msg*    current_msg;

struct OD_object *      map;

struct PDO_object*   pdo[8];
struct SDO_object*   sdo[2];

uint8_t Sync_object [8];


void  (*object_call[8])(uint8_t ,void*);
};

/* ------------- FUNCTION OBJECT ------------*/
/*
 * function skeleton
 * void name ((CanOpen_msg *msg,void *obj){
 * 
 * f(msg != NULL){
 * 
 * .....answer code...
 * 
 * }else{
 * 
 *  struct Info_Object *info = (struct Info_Object*)obj;
 *  
 *   if(obj){
 *      if(sub_data)
 *      ....
 *      info->type = ;
 *      info->sub_code = ;
 *      info->nbit = ;
 *      info->access = ;
 *      ....
 *      };
 * };
 * 
 * 
 * -----------------------  1 byte  ------------------------*/

void ro_object_1b(CanOpen_msg *msg,void *obj){
    
    if(msg){
       uint8_t error = ERROR_NO_CORRECT;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = ERROR_NO_SAVE; 
       if(msg->frame_sdo.cmd == READ_REQUEST) error = 0;
       if(!obj) error = ERROR_SYSTEM;
       if(error){ERR_MSG(error);return;}
       msg->frame_sdo.data.data8 = *((uint8_t *)obj);
       SDO_ANSWER_1b    
    }else{  
        if(obj){ 
            struct Info_Object *info = (struct Info_Object*)obj;
            info->sub_type = info->sub_type==0?UINT8:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x08;
            info->access = RO;          
       };  
    };   
};

void wo_object_1b(CanOpen_msg *msg,void *obj){

    if(msg){
        uint8_t error = ERROR_NO_CORRECT;
        if(msg->frame_sdo.cmd == READ_REQUEST) error = ERROR_NO_READ;
        if(msg->frame_sdo.cmd == GET_1b) error = 0;
        if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ;
        if(!obj) error = ERROR_SYSTEM;
        if(error){ERR_MSG(error);return;};
        *((uint8_t *)obj) = msg->frame_sdo.data.data8;  
        SDO_SAVE_OK   
    }else{
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;  
            info->sub_type = info->sub_type==0?UINT8:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x08;
            info->access = WO;
        };   
    };   
};

void rw_object_1b(CanOpen_msg *msg,void *obj){
     
    if(msg){
        switch(msg->frame_sdo.cmd){
            case READ_REQUEST: ro_object_1b(msg,obj); break;
            case GET_1b:wo_object_1b(msg,obj);break;
            default: ERR_MSG(ERROR_NO_CORRECT);break; 
        };
    }else{
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;  
            info->sub_type = info->sub_type==0?UINT8:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x08;
            info->access = RW;
        };   
    };   
};
/*-----------------------  2 byte  ------------------------*/
void ro_object_2b(CanOpen_msg *msg,void *obj){
    
    if(msg){
       uint8_t error = ERROR_NO_CORRECT;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = ERROR_NO_SAVE; 
       if(msg->frame_sdo.cmd == READ_REQUEST) error = 0;
       if(!obj) error = ERROR_SYSTEM;
       if(error){ERR_MSG(error);return;}
       msg->frame_sdo.data.data16 = *((uint16_t *)obj);
       SDO_ANSWER_2b    
    }else{  
        if(obj){ 
            struct Info_Object *info = (struct Info_Object*)obj;
            info->sub_type = info->sub_type==0?UINT16:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x10;
            info->access = RO;          
       };  
    };   
};

void wo_object_2b(CanOpen_msg *msg,void *obj){

    if(msg){
        uint8_t error = ERROR_NO_CORRECT;
        if(msg->frame_sdo.cmd == READ_REQUEST) error = ERROR_NO_READ;
        if(msg->frame_sdo.cmd == GET_2b) error = 0;
        if(msg->frame_sdo.dlc < 6) error = ERROR_SMALL_DATA_OBJ;
        if(!obj) error = ERROR_SYSTEM;
        if(error){ERR_MSG(error);return;};
        *((uint16_t *)obj) = msg->frame_sdo.data.data16;
        SDO_SAVE_OK  
    }else{
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;  
            info->sub_type = info->sub_type==0?UINT16:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x10;
            info->access = WO;
        };   
    };   
};

void rw_object_2b(CanOpen_msg *msg,void *obj){
     
    if(msg){
        switch(msg->frame_sdo.cmd){
            case READ_REQUEST: ro_object_2b(msg,obj); break;
            case GET_2b:wo_object_2b(msg,obj);break;
            default: ERR_MSG(ERROR_NO_CORRECT);break; 
        };
    }else{
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;  
            info->sub_type = info->sub_type==0?UINT16:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x10;
            info->access = RW;
        };   
    };   
};
/*-----------------------  3 byte  ------------------------*/
void ro_object_3b(CanOpen_msg *msg,void *obj){
    
    if(msg){
       uint8_t error = ERROR_NO_CORRECT;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = ERROR_NO_SAVE; 
       if(msg->frame_sdo.cmd == READ_REQUEST) error = 0;
       if(!obj) error = ERROR_SYSTEM;
       if(error){ERR_MSG(error);return;}
       msg->frame_sdo.data.data24 = *((uint24_t *)obj);
       SDO_ANSWER_3b    
    }else{  
        if(obj){ 
            struct Info_Object *info = (struct Info_Object*)obj;
            info->sub_type = info->sub_type==0?UINT24:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x18;
            info->access = RO;          
       };  
    };   
};

void wo_object_3b(CanOpen_msg *msg,void *obj){

    if(msg){
        uint8_t error = ERROR_NO_CORRECT;
        if(msg->frame_sdo.cmd == READ_REQUEST) error = ERROR_NO_READ;
        if(msg->frame_sdo.cmd == GET_3b) error = 0;
        if(msg->frame_sdo.dlc < 7) error = ERROR_SMALL_DATA_OBJ;
        if(!obj) error = ERROR_SYSTEM;
        if(error){ERR_MSG(error);return;};
        *((uint24_t *)obj) = msg->frame_sdo.data.data24;
        SDO_SAVE_OK     
    }else{
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;  
            info->sub_type = info->sub_type==0?UINT24:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x18;
            info->access = WO;
        };   
    };   
};

void rw_object_3b(CanOpen_msg *msg,void *obj){
     
    if(msg){
        switch(msg->frame_sdo.cmd){
            case READ_REQUEST: ro_object_3b(msg,obj); break;
            case GET_3b:wo_object_3b(msg,obj);break;
            default: ERR_MSG(ERROR_NO_CORRECT);break; 
        };
    }else{
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;  
            info->sub_type = info->sub_type==0?UINT24:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x18;
            info->access = RW;
        };   
    };   
};
/*-----------------------  4 byte  ------------------------*/
void ro_object_4b(CanOpen_msg *msg,void *obj){
    
    if(msg){
       uint8_t error = ERROR_NO_CORRECT;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = ERROR_NO_SAVE; 
       if(msg->frame_sdo.cmd == READ_REQUEST) error = 0;
       if(!obj) error = ERROR_SYSTEM;
       if(error){ERR_MSG(error);return;}
       msg->frame_sdo.data.data32 = *((uint32_t *)obj);
       SDO_ANSWER_4b    
    }else{  
        if(obj){ 
            struct Info_Object *info = (struct Info_Object*)obj;
            info->sub_type = info->sub_type==0?UINT32:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x20;
            info->access = RO;          
       };  
    };   
};

void wo_object_4b(CanOpen_msg *msg,void *obj){

    if(msg){
        uint8_t error = ERROR_NO_CORRECT;
        if(msg->frame_sdo.cmd == READ_REQUEST) error = ERROR_NO_READ;
        if(msg->frame_sdo.cmd == GET_4b) error = 0;
        if(msg->frame_sdo.dlc < 8) error = ERROR_SMALL_DATA_OBJ;
        if(!obj) error = ERROR_SYSTEM;
        if(error){ERR_MSG(error);return;};
        *((uint32_t *)obj) = msg->frame_sdo.data.data32;
        SDO_SAVE_OK     
    }else{
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;  
            info->sub_type = info->sub_type==0?UINT32:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x20;
            info->access = WO;
        };   
    };   
};

void rw_object_4b(CanOpen_msg *msg,void *obj){
     
    if(msg){
        switch(msg->frame_sdo.cmd){
            case READ_REQUEST: ro_object_4b(msg,obj); break;
            case GET_4b:wo_object_4b(msg,obj);break;
            default: ERR_MSG(ERROR_NO_CORRECT);break; 
        };
    }else{
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;  
            info->sub_type = info->sub_type==0?UINT32:0;
            info->obj_code = OD_VAR;
            info->nbit = 0x20;
            info->access = RW;
        };   
    };   
};


/* ----------------- array data ----------------------- */


/* --------------- pdo_object ------------------------ */

void ro_pdo_object(CanOpen_msg *msg,void *obj){
    
    
    if(msg){
       struct PDO_object *pdo = (struct PDO_object *)obj;
       uint8_t error = ERROR_NO_CORRECT;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = ERROR_NO_SAVE;
       if(msg->frame_sdo.cmd == READ_REQUEST) error = 0;
       if(msg->frame_sdo.dlc < 4) error = ERROR_SMALL_DATA_OBJ;
       if(!pdo) error = ERROR_SYSTEM;
       if(!error){      
           switch(msg->frame_sdo.subindex){                 
                case 0:msg->frame_sdo.data.data8  = pdo->sub_index;SDO_ANSWER_1b break;
                case 1:msg->frame_sdo.data.data32 = pdo->cob_id;SDO_ANSWER_4b break;
                case 2:msg->frame_sdo.data.data8  = pdo->Transmission_type;SDO_ANSWER_1b break;
                case 3:msg->frame_sdo.data.data16 = pdo->Inhibit_time;SDO_ANSWER_2b break;                    
                case 5:msg->frame_sdo.data.data16 = pdo->event_timer;SDO_ANSWER_2b break;    
            default: error = ERROR_SUB_INDEX;break;}}
       if(error)ERR_MSG(error);
    }else{  
        if(obj){ 
            struct Info_Object *info = (struct Info_Object*)obj;
            info->obj_code = OD_DEFSTRUCT;
            info->access = RO;   
            switch(msg->frame_sdo.subindex){
                case 0:case 2: info->nbit = 0x08;info->sub_type = UINT8;break;
                case 3:case 5: info->nbit = 0x10;info->sub_type = UINT16;break;
                case 1: info->nbit = 0x20;info->sub_type = UINT32;break;
                default:info->nbit = 0x0; 
                        info->sub_type = 0;
                        info->access = 0;
                        break;}          
       };  
    };   
};

void wo_pdo_object(CanOpen_msg *msg,void *obj){
      
    if(msg){
       struct PDO_object *pdo = (struct PDO_object *)obj; 
       uint8_t error = ERROR_NO_CORRECT;
       if(msg->frame_sdo.cmd == READ_REQUEST) error = ERROR_NO_READ;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = 0;
       if(msg->frame_sdo.dlc < 4) error = ERROR_SMALL_DATA_OBJ;
       if(!pdo) error = ERROR_SYSTEM;
       if(!error){      
       uint8_t lock = pdo->cob_id&0x80000000?1:0;
       switch(msg->frame_sdo.subindex){
        // sub-index
        case 0: error = ERROR_NO_SAVE;break;
        // sub-id 32bit
        case 1:                                    
            if(msg->frame_sdo.cmd != GET_4b){error = ERROR_NO_CORRECT;break;}  
            if(msg->frame_sdo.dlc < 8){error = ERROR_SMALL_DATA_OBJ;break;}
            if(lock){ /// do the right processing!!! 
                    pdo->cob_id = msg->frame_sdo.data.data32&0x800007FF;
                    if((pdo->cob_id&0x80000000) == 0) pdo->status |= PDO_INIT;    
           }else{error = ERROR_NO_SAVE;}            
        break;
        // Transmission_type 8bit
        case 2:   
            if(lock){  
                if(msg->frame_sdo.cmd != GET_1b){error = ERROR_NO_CORRECT;break;}  
                if(msg->frame_sdo.dlc < 5){error = ERROR_SMALL_DATA_OBJ;break;}
                pdo->Transmission_type = msg->frame_sdo.data.data8;
           }else{error = ERROR_NO_SAVE;}; 
        break;
        // Inhibit_time; 16bit
        case 3: 
            if(lock){ 
                if(msg->frame_sdo.cmd != GET_2b){error = ERROR_NO_CORRECT;break;} 
                if(msg->frame_sdo.dlc < 6){error = ERROR_SMALL_DATA_OBJ;break;}           
                pdo->Inhibit_time = msg->frame_sdo.data.data16;                 
           }else{error = ERROR_NO_SAVE;}
        break;
        // Event_timer 16bit
        case 5:   
            if(lock){ 
                if(msg->frame_sdo.cmd != GET_2b){error = ERROR_NO_CORRECT;break;} 
                if(msg->frame_sdo.dlc < 6){error = ERROR_SMALL_DATA_OBJ;break;} 
                pdo->event_timer = msg->frame_sdo.data.data16;                
            }else{error = ERROR_NO_SAVE;};                   
        break;
        default:
           error = ERROR_SUB_INDEX;                      
        break;}}
       if(error){ERR_MSG(error)}else{SDO_SAVE_OK;}
   }else{  
        if(obj){ 
            struct Info_Object *info = (struct Info_Object*)obj;
            info->obj_code = OD_DEFSTRUCT;
            info->access = WO;
            switch(msg->frame_sdo.subindex){
                case 0:case 2: info->nbit = 0x08;info->sub_type = UINT8;break;
                case 3:case 5: info->nbit = 0x10;info->sub_type = UINT16;break;
                case 1: info->nbit = 0x20;info->sub_type = UINT32;break;
                default:info->nbit = 0x0; 
                        info->sub_type = 0;
                        info->access = 0;
                        break;}         
       };  
    };   
};

void rw_pdo_object(CanOpen_msg *msg,void *obj){

    
    if(msg){
         struct PDO_object *pdo = (struct PDO_object *)obj;
         if(msg->frame_sdo.cmd == READ_REQUEST){
              ro_pdo_object(msg,obj);
        }else{wo_pdo_object(msg,obj);}    
    }else{
          struct Info_Object *info = (struct Info_Object*)obj;
            ro_pdo_object(msg,obj);
            info->access = RW;             
   }
}
/*------------------- pdo_map -------------*/

void ro_map_object(CanOpen_msg *msg,void *obj){
    if(msg){
       struct PDO_object *pdo = (struct PDO_object *)obj; 
       uint8_t error = ERROR_NO_CORRECT;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = ERROR_NO_SAVE;
       if(msg->frame_sdo.cmd == READ_REQUEST) error = 0;
       if(msg->frame_sdo.dlc < 4) error = ERROR_SMALL_DATA_OBJ;
       if(!pdo) error = ERROR_SYSTEM;
       if(!error){
            if(msg->frame_sdo.subindex == 0){
                msg->frame_sdo.data.data8  = pdo->pdo_map->sub_index;
                SDO_ANSWER_1b return;}
            if(msg->frame_sdo.subindex <= MAX_MAP_DATA){
                msg->frame_sdo.data.data32 = 
                        pdo->pdo_map->map[(msg->frame_sdo.subindex)-1].data32;
                SDO_ANSWER_4b return;}
            error = ERROR_SUB_INDEX;}
       ERR_MSG(error);
    }else{   
        if(obj){  
            struct Info_Object *info = (struct Info_Object*)obj;
                info->obj_code = OD_ARRAY; 
                info->access = RO;
            if(msg->frame_sdo.subindex <= MAX_MAP_DATA){
                info->nbit = info->sub_type==0?0x08:0x20;
                info->sub_type = info->sub_type==0?UINT8:UINT32; 
            }else{info->nbit = 0; 
                  info->sub_type = 0;
                  info->access = 0;}
        };
    }
};

void rw_rpdo_map_object(CanOpen_msg *msg,void *obj){

 if(msg){
    if(msg->frame_sdo.cmd == READ_REQUEST){ro_map_object(msg,obj);return;}
    
    struct PDO_object *pdo = (struct PDO_object *)obj;
    uint8_t error = ERROR_NO_CORRECT;;
    uint8_t lock = pdo->cob_id&0x80000000?1:0; 
    
    if((msg->frame_sdo.cmd&0xE0) == 0x20) error = 0;
    if(msg->frame_sdo.subindex <=MAX_MAP_DATA) error = ERROR_SUB_INDEX;
    if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ; 
    if(!pdo) error = ERROR_SYSTEM;  
    if(!error){
        if(lock){  
            switch(msg->frame_sdo.subindex){
                case 0:pdo->pdo_map->sub_index = 
                        msg->frame_sdo.data.data8<=MAX_MAP_DATA?
                            msg->frame_sdo.data.data8:MAX_MAP_DATA; 
                            break;
                default: if(msg->frame_sdo.dlc < 8){
                            error = ERROR_SMALL_DATA_OBJ;break;};
                            if(msg->frame_sdo.cmd == GET_4b){
                            error = ERROR_NO_CORRECT; break;}
                            
                            struct OD_object *map;
                            void (*object_call)(CanOpen_msg * ,void* );
                            
                    
                break;}
        }else{error = ERROR_NO_SAVE;};
    };
 if(error){ERR_MSG(error)}else{SDO_SAVE_OK;}
 }else{}
           
};
 



//////////// function obiect call ////////////






void NOP_call(uint8_t code,void* data){};


//-------------NMT_control----------------------

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

void NMT_control(uint8_t code,void* data){
    
   
    struct xCanOpen *node = (struct xCanOpen *)data;
    
        if ((0x7F & node->current_msg->can_frame.id) == 0){
             
            if(node->current_msg->can_frame.data1 == 0 ||
               node->current_msg->can_frame.data1== node->cob_id){
                        
                switch(node->current_msg->can_frame.data0){
                        
                    case START_REMOTE_NODE:
                      node->mode = OPERATIONAL;   
                    break;
                  
                    case STOP_REMOTE_NODE:
                      node->mode = STOPPED ;
                    break;
                      
                    case ENTER_PRE_OPERATIONAL:
                      node->mode = PRE_OPERATIONAL;
                    break;
                   
                    case RESET_APPLICATION:
                       node->mode = BOOT;  
                    break; 
                    
                    case RESET_COMMUNICATION:                       
                         node->mode = BOOT;
                    break;
                    
                    default:
                    break;                               
                };
            };                    
        };

};
//---------------------------------------------


//
void SYNC_control(uint8_t code,void* data){};
//
void TIME_control(uint8_t code,void* data){};


////////////////////// xPDO COMMUNICATION /////////////////

/* NodeID
 * 0...11 11-bit CAN-ID
 * 0...28 29-bit CAN-ID
 * 29     frame
 * 30     reserved
 * 
 * 31     valid 0 PDO exists / is valid
                1 PDO does not exist / is not valid
*/	

#define NEW_MSG_PDO 0x01

#define RxPDO1 RPDO1-3

void RPDO1_call(uint8_t code,void* data){
    
    struct xCanOpen *node = (struct xCanOpen *)data;
    uint8_t lock = node->pdo[RxPDO1]->cob_id&0x80000000?1:0;
    uint32_t * data32;
    uint8_t  error = 0;
     
    switch(code){
    
        case FUNC_MSG:
            
            if (node->mode != OPERATIONAL) break;
            if ( lock) break;            
            if ( node->current_msg->can_frame.id != 
                (node->pdo[RxPDO1]->cob_id & 0x7ff)) break;
            
            for(uint8_t i= 0;i<8;i++){
                
            node->pdo[RxPDO1]->data[i] = 
                    i < (node->current_msg->can_frame.dlc)?
                            node->current_msg->array[i+7]:0;};
                            
            node->pdo[RxPDO1]->status =
                    node->pdo[RxPDO1]->status | NEW_MSG_PDO;
                       
         break;
                
            
        case FUNC_SYSTEM:
            
           if(lock) break;
           
           switch(node->pdo[RxPDO1]->Transmission_type){
               
               case 0xFE:
                   break;
                   
               case 0xFF:
                   break;
                   
               default:
              
                   if (node->pdo[RxPDO1]->Transmission_type > 0xF0) break;
                   
                   
                   
                   
               
           };
           
         break;         
         
        default:
        break;
            
    };
    
};




void RPDO2_call(uint8_t code,void* data){};
void RPDO3_call(uint8_t code,void* data){};
void RPDO4_call(uint8_t code,void* data){};

//Tx
void TPDO1_call(uint8_t code,void* data){};
void TPDO2_call(uint8_t code,void* data){};
void TPDO3_call(uint8_t code,void* data){};
void TPDO4_call(uint8_t code,void* data){};

//Rx
void RxSDO1_call(uint8_t code,void* data){};
//Tx
void TxSDO2_call(uint8_t code,void* data){};





#endif	/* CANOPEN_H */

