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

struct obj_info{
    void *  object; // -> (object->data) <- object+sub_index
    uint8_t sub_nbit;// -> sub_index ; <-nbit
    uint8_t access;
};

struct map_info{
	uint8_t  nbit; 
	uint8_t  sub_index;
	uint16_t index;
	};	


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


//////////////////// ERROR ////////////////

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
                     


/*----------------  OD_TABLE  --------------------*/

struct OD_object{
    uint16_t index;
    void* data;
    void (*func_data)(CanOpen_msg *msg,void *obj);
};

/*index 0000 -  0xFFFF -> the end)*/
struct 
OD_object* OD_search_index(uint16_t index, struct OD_object* tab){   
    while (tab->index < index){tab++;} 
    return tab = tab->index == index?tab:NULL;};

struct OD_object* OD_search_msg_index(CanOpen_msg *msg, struct OD_object* tab){
return  OD_search_index((msg->frame_sdo.index),tab);}

struct OD_object* OD_search_map_index(CanOpen_msg *msg, struct OD_object* tab){
return  OD_search_index(msg->frame_sdo.data.map.index,tab);}



/*--------------PDO COMMUNICATION-----------------*/

#define PDO_DISABLED	0x80000000 // 0b1000 0000 0000 0000
#define PDO_is_TX	0x80 	   //0b1000 0000
#define PDO_calculate_n	0x700 // 0b111 0000 0000
#define PDO_mask_addr	0x7FF // 0b0111 1111 1111

#define RXPDO 0
#define TXPDO 1

#define MAX_MAP_DATA 8
#define MAX_MAP_NBIT MAX_MAP_DATA*8 //warning 8bit x n_byte


#define PDO_INIT    0b00000001
#define NEW_MSG_PDO 0b00000010

union map_data{
    struct map_info info;
	uint32_t        data32;
};
struct PDO_mapping {
    uint8_t           sub_index;
    union map_data    map[MAX_MAP_DATA];
    // quick access to the map
    void *            quick_mapping[MAX_MAP_DATA];
    struct OD_object* node_map;
};
struct PDO_object{   
uint8_t     type; // RX = 0,TX = 1
uint8_t     status; 

// visible block
uint8_t		sub_index ;
uint8_t		Transmission_type;
uint32_t	cob_id ;
uint16_t	Inhibit_time; 	
uint16_t	event_timer;
// quick access to the structure map

struct 
PDO_mapping* pdo_map;
void *      node_parent;
void (*process_map)(struct PDO_object*);
uint8_t     data[8];
};

struct one_type_array{
    uint8_t sub_index;
    void*   array;
};

struct arr_object{
    uint8_t sub_index;
    uint8_t nbit;
    void*   array;
};








void map_object_processing(struct PDO_object* pdo){
   
   uint8_t sum_nbit = 0, sub_index = 0;
   
   if(pdo->pdo_map->sub_index > MAX_MAP_DATA)
                    pdo->pdo_map->sub_index = MAX_MAP_DATA;

   while(sub_index < pdo->pdo_map->sub_index){      
       if(!(pdo->pdo_map->map[sub_index].info.index)||
          !(pdo->pdo_map->map[sub_index].info.nbit )||
          !(pdo->pdo_map->quick_mapping[sub_index])) break;
       sum_nbit += pdo->pdo_map->map[sub_index].info.nbit;
       if(sum_nbit > MAX_MAP_NBIT) break; 
       sub_index ++;
   };
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
    if(pdo->type == TXPDO)process_the_TxPDO_message(pdo);
    else process_the_RxPDO_message(pdo);
};



void copy_rxPDO_message_to_array(CanOpen_msg* msg,struct PDO_object* pdo){
    if(!(msg->can_frame.dlc)) return;
    for(uint8_t i= 0; i< MAX_MAP_DATA ;i++){
        pdo->data[i] = i < msg->can_frame.dlc?msg->array[i+7]:0;}
    pdo->status = pdo->status | NEW_MSG_PDO;
};


/* ---------------------- SDO COMMUNICATION -----------------*/

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

struct data_object{
    void* data_object;
    void* rw_object;
    uint8_t request_type;
    uint8_t attribute;
    uint8_t nbit;
    uint8_t sub_index;
    uint32_t sub_index_ff;
};
void copy_data (uint8_t* wdata, uint8_t* rdata, uint8_t nbit){
    if(!wdata || !rdata)return;
    nbit >>=3;
    for(uint8_t i=0;i<nbit;i++){*(wdata + i) = *(rdata+i);}
};

void single_object_processing(struct rw_object *obj){



};







uint8_t check_sdo_command_for_writing(CanOpen_msg *msg,uint8_t nbit){
    if(!msg) return ERROR_SYSTEM;
    uint8_t dlc = 4+(nbit >> 3);
    uint8_t cmd = ((8-dlc)<<2)|0x23;
    if(msg->frame_sdo.cmd != cmd) return ERROR_SDO_SERVER;
    if(msg->frame_sdo.dlc < dlc) return ERROR_SMALL_DATA_OBJ;
    SDO_SAVE_OK
    return 0;
} 
uint8_t check_sdo_command_for_reading(CanOpen_msg *msg,uint8_t nbit){
    if(!msg) return ERROR_SYSTEM;
    if(msg->frame_sdo.cmd != READ_REQUEST) return ERROR_SDO_SERVER;
    if(msg->frame_sdo.dlc < 4) return ERROR_SMALL_DATA_OBJ;
    uint8_t dlc = 4+(nbit >> 3);
    msg->frame_sdo.dlc = dlc;
    msg->frame_sdo.cmd = ((8-dlc)<<2)|0x43;   
    return 0;
}

void single_object_response(struct obj_info *info,uint8_t attr,uint8_t nbit){ 
    info->object = info->sub_nbit == 0?info->object:NULL;
    info->sub_nbit = info->object != NULL? nbit:0;
    info->access = attr;};
    
void one_type_array_response(struct obj_info *info,uint8_t attr,uint8_t nbit){ 
    
    struct one_type_array *arr = info->object;
    uint8_t len; 
    if(!arr) return;
    if(info->sub_nbit > arr->sub_index){
       info->object = NULL;info->sub_nbit = 0;return;}
    
    switch(arr->sub_index){
        case 0:info->object = &(arr->sub_index);
               info->sub_nbit = 0x08;info->access = RO;break;
        default:
            len = (info->sub_nbit)-1;
            switch(nbit){
             case 0x08:info->object = ((uint8_t *)arr->array)+len;break;   
             case 0x10:info->object = ((uint16_t *)arr->array)+len;break;
             case 0x18:info->object = ((uint24_t *)arr->array)+len;break;
             case 0x20:info->object = ((uint32_t *)arr->array)+len;break;
             default: info->object = NULL;break;}
    };    
    info->access = attr; info->sub_nbit = nbit;
};



#define SDO_request 0
#define MAP_read_request 1
#define MAP_write_request 2
#define MAP_info 3

void single_object(struct data_object *obj){
    
    if(!obj)return;
    switch(obj->request_type){
       case SDO_request:
            CanOpen_msg *msg = obj->rw_object;
            uint8_t *wdata,*rdata,nbit,error;
            wdata = &msg->frame_sdo.data;
            //read
            if(msg->frame_sdo.cmd == READ_REQUEST){	
				if(obj->attribute&RO){
				error = check_sdo_command_for_reading(msg,obj->nbit);
				if(!error){ 
                    switch(msg->frame_sdo.subindex){
                       case 0:rdata = obj->data_object;
                              nbit  = obj->nbit;break;              
                       case 0xFF:rdata = &(obj->sub_index_ff);
                                 obj->nbit = 0x20; SDO_ANSWER_4b;break;         
                       default:error = ERROR_SUB_INDEX;break;};		
                 }}else error = ERROR_NO_READ;	
            //write    
            }else if((msg->frame_sdo.cmd&0xE0) == WRITE_REQUEST){
				if(obj->attribute&WO){
				error = check_sdo_command_for_writing(msg,obj->nbit);
				if(!error){ 
                    switch(msg->frame_sdo.subindex){
                       case 0:rdata = wdata;
                              wdata = obj->data_object;
							  nbit  = obj->nbit;break;       
                       case 0xFF:rdata = &(obj->sub_index_ff);
                                 obj->nbit = 0x20; SDO_ANSWER_4b;break;         
                        default:error = ERROR_SUB_INDEX;break;};	
				}}else error = ERROR_NO_SAVE;
		  }else error = ERROR_SDO_SERVER;
                    
          if(error){obj->data_object = (error_msg + error);
                      obj->nbit = 0x20;
                      msg->frame_sdo.cmd = RESPONSE_ERROR;
                      msg->frame_sdo.dlc = 0x08;}
                      
          copy_data(wdata,rdata,nbit);break;         
          
        case MAP_read_request:
            if(!(obj->attribute & RO)) break;
            copy_data(obj->rw_object,obj->data_object,obj->nbit);break;
        case MAP_write_request:
            if(!(obj->attribute & WO)) break;
            copy_data(obj->data_object,obj->rw_object,obj->nbit);break;
        case MAP_info:
            obj->rw_object = obj->sub_index?NULL: obj->data_object;
     }
};

void ro_object_1byte(struct data_object *obj){
     obj->nbit = 0x08;obj->sub_index_ff = (UINT8 << 8)| OD_VAR ;
     single_object(obj);}



void ro_object_1b(CanOpen_msg *msg,void *obj){
    
    if(msg){
       uint8_t error = ERROR_SYSTEM;
       if(obj) error = check_sdo_command_for_reading(msg,0x08);
       if(!error){
           switch(msg->frame_sdo.subindex){
               case 0: msg->frame_sdo.data.data8 = *((uint8_t *)obj); break;
               case 0xFF: msg->frame_sdo.data.data32 = (UINT8 << 8)| OD_VAR ; 
                          SDO_ANSWER_4b; break;
               default:error = ERROR_SUB_INDEX;break;}
       }          
       if(error) ERR_MSG(error);   
    }else if(obj)single_object_response(obj,RO,0x08);   
};

void wo_object_1b(CanOpen_msg *msg,void *obj){

    if(msg){
        uint8_t error = ERROR_SYSTEM;
        if(obj) error = check_sdo_command_for_writing(msg,0x08);
        if(!error && !(msg->frame_sdo.subindex)){
            *((uint8_t *)obj) = msg->frame_sdo.data.data8;return;
        }else error = ERROR_SUB_INDEX;
        ERR_MSG(error);            
    }else if(obj) single_object_response(obj,WO,0x08);   
};

void rw_object_1b(CanOpen_msg *msg,void *obj){
     
    if(msg){
        switch(msg->frame_sdo.cmd){
            case READ_REQUEST: ro_object_1b(msg,obj); break;
            case GET_1b:wo_object_1b(msg,obj);break;
            default: ERR_MSG(ERROR_SDO_SERVER);break; 
        };
    }else if(obj)single_object_response(obj,RW,0x08);      
};
/*-----------------------  2 byte  ------------------------*/

void ro_object_2b(CanOpen_msg *msg,void *obj){
    
    if(msg){
       uint8_t error = ERROR_SYSTEM;
       if(obj) error = check_sdo_command_for_reading(msg,0x10);
       if(!error){
           switch(msg->frame_sdo.subindex){
               case 0: msg->frame_sdo.data.data16 = *((uint16_t *)obj); break;
               case 0xFF: msg->frame_sdo.data.data32 = (UINT16 << 8)| OD_VAR ; 
                          SDO_ANSWER_4b; break;
               default:error = ERROR_SUB_INDEX;break;}
       }          
       if(error) ERR_MSG(error);    
    }else if(obj)single_object_response(obj,RO,0x10);   
};

void wo_object_2b(CanOpen_msg *msg,void *obj){

   if(msg){
        uint8_t error = ERROR_SYSTEM;
        if(obj) error = check_sdo_command_for_writing(msg,0x10);
        if(!error && !(msg->frame_sdo.subindex)){
            *((uint16_t *)obj) = msg->frame_sdo.data.data16;return;
        }else error = ERROR_SUB_INDEX;
        ERR_MSG(error);  
    }else if(obj)single_object_response(obj,WO,0x10);   
};

void rw_object_2b(CanOpen_msg *msg,void *obj){
     
    if(msg){
        switch(msg->frame_sdo.cmd){
            case READ_REQUEST: ro_object_2b(msg,obj); break;
            case GET_2b:wo_object_2b(msg,obj);break;
            default: ERR_MSG(ERROR_SDO_SERVER);break; 
        };
    }else if(obj)single_object_response(obj,RW,0x10);   
};
/*-----------------------  3 byte  ------------------------*/

void ro_object_3b(CanOpen_msg *msg,void *obj){
    
   if(msg){
       uint8_t error = ERROR_SYSTEM;
       if(obj) error = check_sdo_command_for_reading(msg,0x18);
       if(!error){
           switch(msg->frame_sdo.subindex){
               case 0: msg->frame_sdo.data.data24 = *((uint24_t *)obj); break;
               case 0xFF: msg->frame_sdo.data.data32 = (UINT24 << 8)| OD_VAR ; 
                          SDO_ANSWER_4b; break;
               default:error = ERROR_SUB_INDEX;break;}
       }          
       if(error) ERR_MSG(error);    
    }else if(obj)single_object_response(obj,RO,0x18);
};

void wo_object_3b(CanOpen_msg *msg,void *obj){

    if(msg){
        uint8_t error = ERROR_SYSTEM;
        if(obj) error = check_sdo_command_for_writing(msg,0x18);
        if(!error && !(msg->frame_sdo.subindex)){
            *((uint24_t *)obj) = msg->frame_sdo.data.data24;return;
        }else error = ERROR_SUB_INDEX;
        ERR_MSG(error);    
    }else if(obj)single_object_response(obj,WO,0x18);   
 };

void rw_object_3b(CanOpen_msg *msg,void *obj){
     
    if(msg){
        switch(msg->frame_sdo.cmd){
            case READ_REQUEST: ro_object_3b(msg,obj); break;
            case GET_3b:wo_object_3b(msg,obj);break;
            default: ERR_MSG(ERROR_SDO_SERVER);break; 
        };
    }else if(obj)single_object_response(obj,RW,0x18);  
};
/*-----------------------  4 byte  ------------------------*/

void ro_object_4b(CanOpen_msg *msg,void *obj){
    
    if(msg){
       uint8_t error = ERROR_SYSTEM;
       if(obj) error = check_sdo_command_for_reading(msg,0x20);
       if(!error){
           switch(msg->frame_sdo.subindex){
               case 0: msg->frame_sdo.data.data32 = *((uint32_t *)obj); break;
               case 0xFF: msg->frame_sdo.data.data32 = (UINT32 << 8)| OD_VAR ;break;
               default:error = ERROR_SUB_INDEX;break;}}          
       if(error) ERR_MSG(error);    
    }else if(obj)single_object_response(obj,RO,0x20);   
};

void wo_object_4b(CanOpen_msg *msg,void *obj){

    if(msg){
        uint8_t error = ERROR_SYSTEM;
        if(obj) error = check_sdo_command_for_writing(msg,0x20);
        if(!error && !(msg->frame_sdo.subindex)){
            *((uint32_t *)obj) = msg->frame_sdo.data.data32;return;
        }else error = ERROR_SUB_INDEX;
        ERR_MSG(error);     
    }else if(obj)single_object_response(obj,WO,0x20);   
};

void rw_object_4b(CanOpen_msg *msg,void *obj){
     
    if(msg){
        switch(msg->frame_sdo.cmd){
            case READ_REQUEST: ro_object_4b(msg,obj); break;
            case GET_4b:wo_object_4b(msg,obj);break;
            default: ERR_MSG(ERROR_SDO_SERVER);break; 
        };
    }else if(obj)single_object_response(obj,RW,0x20);   
};


/* ----------------------- array data ---------------------- */


void ro_object_1b_array(CanOpen_msg *msg,void *obj){
    
    if(msg){
       struct one_type_array *arr = obj; 
       uint8_t error = ERROR_SYSTEM;
       if(obj) error = check_sdo_command_for_reading(msg,0x08);
       if(!error){
           switch(msg->frame_sdo.subindex){
               case 0: msg->frame_sdo.data.data8 = arr->sub_index; break;
               case 0xFF: msg->frame_sdo.data.data32 = (UINT8 << 8)| OD_ARRAY ; 
                          SDO_ANSWER_4b; break;
               default:
                 if(msg->frame_sdo.subindex > arr->sub_index) error = ERROR_SUB_INDEX;break;
                 msg->frame_sdo.data.data8=
                         *(((uint8_t*)arr->array)+((msg->frame_sdo.subindex)-1));
               break;}
       }          
       if(error) ERR_MSG(error);   
    }else if(obj)one_type_array_response(obj,RO,0x08);   
};


// or ?




void ro_array_object(CanOpen_msg *msg,void *obj){

if(msg){
    struct arr_object *arr = (struct arr_object*)obj;
    
    uint8_t error = ERROR_SYSTEM;
    uint8_t  *b8=(uint8_t*)arr->array;
    uint16_t *b16=(uint16_t*)arr->array;
    uint24_t *b24=(uint24_t*)arr->array;
    uint32_t *b32=(uint32_t*)arr->array;
    
    if(arr) error = check_sdo_command_for_reading(msg,arr->nbit);
      if(!error){
        switch(arr->sub_index){
          case 0:msg->frame_sdo.data.data8 = arr->sub_index;SDO_ANSWER_1b break; 
          default:
            if(msg->frame_sdo.subindex > arr->sub_index)error = ERROR_SUB_INDEX;break;
            switch(arr->nbit){
                case 0x08:msg->frame_sdo.data.data8=*(b8+((msg->frame_sdo.subindex)-1));break;    
                case 0x10:msg->frame_sdo.data.data16 = *(b16 + ((msg->frame_sdo.subindex)-1));break;
                case 0x18:msg->frame_sdo.data.data24 = *(b24 + ((msg->frame_sdo.subindex)-1));break;
                case 0x20:msg->frame_sdo.data.data32 = *(b32 + ((msg->frame_sdo.subindex)-1));break;
                default: error = ERROR_LEN_OBJECT; break;};
          break;}} 
    if(error)ERR_MSG(error);
}else{
      if(obj){  
       struct obj_info *info = (struct obj_info*)obj;
       struct arr_object *arr = (struct arr_object*)info->object;
       uint8_t  *b8=(uint8_t*)arr->array;
       uint16_t *b16=(uint16_t*)arr->array;
       uint24_t *b24=(uint24_t*)arr->array;
       uint32_t *b32=(uint32_t*)arr->array;
       switch(info->sub_nbit){
          case 0:  info->object = &(arr->sub_index);
                   info->sub_nbit = 0x08;info->access = RO;break;
          default: if(info->sub_nbit > arr->sub_index){info->object = NULL;break;}
          
             switch(arr->nbit){
                   case 0x08:info->object = b8+((info->sub_nbit)-1);break;   
                   case 0x10:info->object = b16+((info->sub_nbit)-1);break;
                   case 0x18:info->object = b24+((info->sub_nbit)-1);break;
                   case 0x20:info->object = b32+((info->sub_nbit)-1);break;
                   default: info->object = NULL;break;}
             
               if(info->object){info->access = WO;info->sub_nbit = arr->nbit;
               }else{info->sub_nbit = 0;info->access = 0;};       
       break;}
}}};

void wo_array_object(CanOpen_msg *msg,void *obj){

if(msg){
    struct arr_object *arr = (struct arr_object*)obj;
    
    uint8_t error = ERROR_SDO_SERVER;
    uint8_t  *b8=(uint8_t*)arr->array;
    uint16_t *b16=(uint16_t*)arr->array;
    uint24_t *b24=(uint24_t*)arr->array;
    uint32_t *b32=(uint32_t*)arr->array;
    
    if((msg->frame_sdo.cmd&0xE0) == 0x20) error = 0;
    if(msg->frame_sdo.cmd == READ_REQUEST) error = ERROR_NO_READ;
    if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ;
    if(!arr) error = ERROR_SYSTEM;
      if(!error){
        switch(arr->sub_index){
          case 0:error = ERROR_NO_SAVE;break; 
          default:
               if(msg->frame_sdo.subindex > arr->sub_index)error = ERROR_SUB_INDEX;break;
                 switch(arr->nbit){
                   case 0x08:*(b8+((msg->frame_sdo.subindex)-1))=msg->frame_sdo.data.data8;break;   
                   case 0x10:if(msg->frame_sdo.dlc < 6) error = ERROR_SMALL_DATA_OBJ;break;
                             *(b16 + ((msg->frame_sdo.subindex)-1)) = msg->frame_sdo.data.data16;break;
                   case 0x18:if(msg->frame_sdo.dlc < 7) error = ERROR_SMALL_DATA_OBJ;break;
                             *(b24 + ((msg->frame_sdo.subindex)-1)) = msg->frame_sdo.data.data24;break; 
                   case 0x20:if(msg->frame_sdo.dlc < 8) error = ERROR_SMALL_DATA_OBJ;break;
                             *(b32 + ((msg->frame_sdo.subindex)-1)) = msg->frame_sdo.data.data32;break;
                   default: error = ERROR_LEN_OBJECT; break;};
         break;}   
    }; if(error){ERR_MSG(error)}else{SDO_SAVE_OK};
}else{
      if(obj){  
       struct obj_info *info = (struct obj_info*)obj;
       struct arr_object *arr = (struct arr_object*)info->object;
       uint8_t  *b8=(uint8_t*)arr->array;
       uint16_t *b16=(uint16_t*)arr->array;
       uint24_t *b24=(uint24_t*)arr->array;
       uint32_t *b32=(uint32_t*)arr->array;
       
       switch(info->sub_nbit){
          case 0:  info->object = &(arr->sub_index);
                   info->sub_nbit = 0x08;info->access = RO;break;
          default: if(info->sub_nbit > arr->sub_index){info->object = NULL;}
          
             switch(arr->nbit){
                   case 0x08:info->object = b8+((info->sub_nbit)-1);break;   
                   case 0x10:info->object = b16+((info->sub_nbit)-1);break;
                   case 0x18:info->object = b24+((info->sub_nbit)-1);break;
                   case 0x20:info->object = b32+((info->sub_nbit)-1);break;
                   default: info->object = NULL;break;}
             
               if(info->object){info->access = WO;info->sub_nbit = arr->nbit;
               }else{info->sub_nbit = 0;info->access = 0;};       
       break;}
}}};

void rw_array_object(CanOpen_msg *msg,void *obj){
     
    if(msg){
        if(msg->frame_sdo.cmd == READ_REQUEST){ 
            ro_array_object(msg,obj);
        }else if((msg->frame_sdo.cmd&0xE0) == 0x20){wo_array_object(msg,obj);
        }else{ERR_MSG(ERROR_SDO_SERVER);}
    }else{
        if(obj){ 
            struct obj_info *info = (struct obj_info*)obj;
            wo_array_object(msg,obj);
            info->access = RW;
        };   
    };   
};


/* ------------------------- pdo_object ------------------------ */

void ro_pdo_object(CanOpen_msg *msg,void *obj){
    
    
    if(msg){
       struct PDO_object *pdo = (struct PDO_object *)obj;
       uint8_t error = ERROR_SDO_SERVER;
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
                case SUB_INDEX_FF:msg->frame_sdo.data.data32 =
                                  (PDO_COMM >>8)|OD_DEFSTRUCT ;SDO_ANSWER_4b break;
            default: error = ERROR_SUB_INDEX;break;}}
       if(error)ERR_MSG(error);
    }else{  
        if(obj){ 
            
            struct obj_info *info = (struct obj_info*)obj;
            struct PDO_object *pdo = (struct PDO_object *)info->object; 
            switch(info->sub_nbit){
                case 0: info->object = &(pdo->sub_index);info->sub_nbit = 0x08;break;
                case 1: info->object = &(pdo->cob_id);info->sub_nbit = 0x20;break;       
                case 2: info->object = &(pdo->Transmission_type);info->sub_nbit = 0x08;break;
                case 3: info->object = &(pdo->Inhibit_time);info->sub_nbit = 0x10;break;       
                case 5: info->object = &(pdo->event_timer);info->sub_nbit = 0x10;break;
                
                default:info->object = NULL;info->sub_nbit = 0x00;break;}        
            info->access = info->object != NULL?RO:0;
       };  
    };   
};

void wo_pdo_object(CanOpen_msg *msg,void *obj){
      
    if(msg){
       struct PDO_object *pdo = (struct PDO_object *)obj; 
       uint8_t error = ERROR_SDO_SERVER;
       if(msg->frame_sdo.cmd == READ_REQUEST) error = ERROR_NO_READ;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = 0;
       if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ;
       if(!pdo) error = ERROR_SYSTEM;
       if(!error){      
       uint8_t lock = pdo->cob_id&0x80000000?1:0;
       switch(msg->frame_sdo.subindex){
        // sub-index
        case 0: error = ERROR_NO_SAVE;break;
        // sub-id 32bit
        case 1:                                    
            if(msg->frame_sdo.cmd != GET_4b){error = ERROR_SDO_SERVER;break;}  
            if(msg->frame_sdo.dlc < 8){error = ERROR_SMALL_DATA_OBJ;break;}
            if(lock){ /// do the right processing!!! 
                    pdo->cob_id = msg->frame_sdo.data.data32&0x800007FF;
                    if((pdo->cob_id&0x80000000) == 0) pdo->status |= PDO_INIT;    
           }else{error = ERROR_NO_SAVE;}            
        break;
        // Transmission_type 8bit
        case 2:   
            if(lock){  
                if(msg->frame_sdo.cmd != GET_1b){error = ERROR_SDO_SERVER;break;}  
                if(msg->frame_sdo.dlc < 5){error = ERROR_SMALL_DATA_OBJ;break;}
                pdo->Transmission_type = msg->frame_sdo.data.data8;
           }else{error = ERROR_NO_SAVE;}; 
        break;
        // Inhibit_time; 16bit
        case 3: 
            if(lock){ 
                if(msg->frame_sdo.cmd != GET_2b){error = ERROR_SDO_SERVER;break;} 
                if(msg->frame_sdo.dlc < 6){error = ERROR_SMALL_DATA_OBJ;break;}           
                pdo->Inhibit_time = msg->frame_sdo.data.data16;                 
           }else{error = ERROR_NO_SAVE;}
        break;
        // Event_timer 16bit
        case 5:   
            if(lock){ 
                if(msg->frame_sdo.cmd != GET_2b){error = ERROR_SDO_SERVER;break;} 
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
            
            struct obj_info *info = (struct obj_info*)obj;
            struct PDO_object *pdo = (struct PDO_object *)info->object;
            uint8_t sub_index = info->sub_nbit;
            switch(sub_index){
                case 0: info->object = &(pdo->sub_index);info->sub_nbit = 0x08;break;
                case 1: info->object = &(pdo->cob_id);info->sub_nbit = 0x20;break;       
                case 2: info->object = &(pdo->Transmission_type);info->sub_nbit = 0x08;break;
                case 3: info->object = &(pdo->Inhibit_time);info->sub_nbit = 0x10;break;       
                case 5: info->object = &(pdo->event_timer);info->sub_nbit = 0x10;break;
                
                default:info->object = NULL;info->sub_nbit = 0x00;info->access = 0;
                        break;}
            if(info->object != NULL){info->access = sub_index?WO:RO;}
       };  
    };   
};

void rw_pdo_object(CanOpen_msg *msg,void *obj){
   
    if(msg){
         struct PDO_object *pdo = (struct PDO_object *)obj;
         if(msg->frame_sdo.cmd == READ_REQUEST){
              ro_pdo_object(msg,obj);
        }else{wo_pdo_object(msg,obj);}  
         
   }else{ struct obj_info *info = (struct obj_info*)obj;
          struct PDO_object *pdo = (struct PDO_object *)info->object;
          uint8_t sub_index = info->sub_nbit;
          switch(sub_index){
            case 0: info->object = &(pdo->sub_index);info->sub_nbit = 0x08;break;
            case 1: info->object = &(pdo->cob_id);info->sub_nbit = 0x20;break;       
            case 2: info->object = &(pdo->Transmission_type);info->sub_nbit = 0x08;break;
            case 3: info->object = &(pdo->Inhibit_time);info->sub_nbit = 0x10;break;       
            case 5: info->object = &(pdo->event_timer);info->sub_nbit = 0x10;break;
            default:info->object = NULL;info->sub_nbit = 0;info->access = 0;break;}
            if(info->object != NULL){info->access = sub_index?RW:RO;}
   }
}
/*------------------------- pdo_map object ------------------------------*/

void ro_map_object(CanOpen_msg *msg,void *obj){
    if(msg){
       struct PDO_object *pdo = (struct PDO_object *)obj; 
       uint8_t error = ERROR_SDO_SERVER;
       if((msg->frame_sdo.cmd&0xE0) == 0x20) error = ERROR_NO_SAVE;
       if(msg->frame_sdo.cmd == READ_REQUEST) error = 0;
       if(msg->frame_sdo.subindex > MAX_MAP_DATA) error = ERROR_SUB_INDEX;
       if(msg->frame_sdo.dlc < 4) error = ERROR_SMALL_DATA_OBJ;
       if(!pdo) error = ERROR_SYSTEM;
       if(!error){ switch(msg->frame_sdo.subindex){
                   case 0:msg->frame_sdo.data.data8  = pdo->pdo_map->sub_index;
                          SDO_ANSWER_1b;break;
                   case SUB_INDEX_FF:msg->frame_sdo.data.data32 =
                          (PDO_MAPPING >>8)| OD_DEFSTRUCT;SDO_ANSWER_4b break;       
                   default:  msg->frame_sdo.data.data32 = 
                        pdo->pdo_map->map[(msg->frame_sdo.subindex)-1].data32;
                        SDO_ANSWER_4b ; break;
                    };return;
       };
       ERR_MSG(error);
    }else{   
        if(obj){  
            struct obj_info *info = (struct obj_info*)obj;
            struct PDO_object *pdo = (struct PDO_object *)info->object;
            switch(info->sub_nbit){
                case 0:info->object = &(pdo->pdo_map->sub_index);
                       info->sub_nbit = 0x08;info->access = RO;break;
                default: if(info->sub_nbit <=MAX_MAP_DATA){
                    info->object = &(pdo->pdo_map->map[info->sub_nbit-1].data32);
                    info->sub_nbit = 0x20;info->access = RO;break;
                }else{info->object = NULL;info->sub_nbit = 0;info->access = 0;};
                break;
            }
        };
    }
};

void rw_rpdo_map_object(CanOpen_msg *msg,void *obj){

 if(msg){
    if(msg->frame_sdo.cmd == READ_REQUEST){ro_map_object(msg,obj);return;}
    
    struct PDO_object *pdo = (struct PDO_object *)obj;
    uint8_t error = ERROR_SDO_SERVER;
    uint8_t lock = pdo->cob_id&0x80000000?1:0; 
    
    if((msg->frame_sdo.cmd&0xE0) == 0x20) error = 0;
    if(msg->frame_sdo.subindex >MAX_MAP_DATA) error = ERROR_SUB_INDEX;
    if(msg->frame_sdo.dlc < 5) error = ERROR_SMALL_DATA_OBJ; 
    if(!pdo) error = ERROR_SYSTEM;  
    if(!error){
      if(lock){  
        switch(msg->frame_sdo.subindex){
          case 0:if(msg->frame_sdo.cmd != GET_1b){error = ERROR_SDO_SERVER;break;}              
                 pdo->pdo_map->sub_index = msg->frame_sdo.data.data8<=MAX_MAP_DATA?
                 msg->frame_sdo.data.data8:MAX_MAP_DATA;
                 map_object_processing(pdo); //test->map
                 break;
          default: if(msg->frame_sdo.dlc < 8){error = ERROR_SMALL_DATA_OBJ;break;};
                   if(msg->frame_sdo.cmd != GET_4b){error = ERROR_SDO_SERVER; break;}
                   if(!(msg->frame_sdo.data.map.nbit)){error = ERROR_SMALL_DATA_OBJ;break;}
          
                   struct OD_object *tab; 
                   void (*object_call)(CanOpen_msg * ,void* );
                        
                   tab = OD_search_map_index(msg,pdo->pdo_map->node_map);
                   if(!tab || !tab->func_data || !tab->data){
                                            error = ERROR_NO_OBJECT;break;}                       
                   struct obj_info info;
                   info.sub_nbit = msg->frame_sdo.data.map.sub_index;
                   info.object = tab->data;
                   object_call = tab->func_data;
                   object_call (NULL,&info);
                   if(info.object == NULL){error = ERROR_SUB_INDEX;break;}
                   if(!(info.access&NO_MAP)){ error = ERROR_OBJECT_PDO;break;}
                   
                   /*
                    if(pdo->cob_id&PDO_is_TX){
                       if(!(info.access&RO)){error = ERROR_OBJECT_PDO;break;} 
                    }else if(!(info.access&WO)){error = ERROR_OBJECT_PDO;break;}
                   */
                   
                   if(pdo->type == RXPDO){ 
                       if(!(info.access&WO)){error = ERROR_OBJECT_PDO;break;} 
                   }else if(pdo->type == TXPDO){
                       if(!(info.access&RO)){error = ERROR_OBJECT_PDO;break;}
                   }else{error = ERROR_SYSTEM;break;};
                      
                   if(info.sub_nbit != msg->frame_sdo.data.map.nbit){
                                          error = ERROR_LEN_OBJECT;break;}    
                   pdo->pdo_map->map[(msg->frame_sdo.subindex)-1].data32=
                       msg->frame_sdo.data.data32 ;      
                   pdo->pdo_map->quick_mapping[(msg->frame_sdo.subindex)-1] = 
                           info.object;
                   
                   map_object_processing(pdo); //test mapping object
                   
                break;}
        }else{error = ERROR_NO_SAVE;};
    };
 if(error){ERR_MSG(error)}else{SDO_SAVE_OK;}
 }else{
     if(obj){  
            struct obj_info *info = (struct obj_info*)obj;
            struct PDO_object *pdo = (struct PDO_object *)info->object;
            switch(info->sub_nbit){
                case 0:info->object = &(pdo->pdo_map->sub_index);
                       info->sub_nbit = 0x08;info->access = RW;break;
                default: if(info->sub_nbit <=MAX_MAP_DATA){
                    info->object = &(pdo->pdo_map->map[info->sub_nbit-1].data32);
                    info->sub_nbit = 0x20;info->access = RW;break;
                }else{info->object = NULL;info->sub_nbit = 0;info->access = 0;};
                break;
            }
        };
    }
           
};

/* ------------------ STRUCT CAN NODE ------------------*/

#define n_FUNC_COMMAND 0x0f
#define MAX_SYNC_OBJECT 8

struct xCanOpen{

uint8_t              cob_id; /*id */
uint8_t                mode; /*pre-orintal etc.*/

CanOpen_msg*    current_msg;

uint8_t  (*init)(uint8_t id);
uint8_t  (*receiving_message)(CanOpen_msg *msg);
uint8_t  (*sending_message)(CanOpen_msg *msg);

struct OD_object*   map; 
struct PDO_object*  pdo[8];
struct SDO_object*  sdo[1];

uint8_t Sync_object [MAX_SYNC_OBJECT];

void  (*func_call[n_FUNC_COMMAND])(uint8_t ,void*);
};

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
               node->current_msg->can_frame.data1 == node->cob_id){
                        
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
    for(uint8_t n=0; n<MAX_SYNC_OBJECT; n++){
      if(node->Sync_object[n]) node->Sync_object[n]--;
    };
};
    
/*--------------------- Time ------------------------*/

void TIME_message_processing(uint8_t code,void* data){
    struct xCanOpen *node = (struct xCanOpen *)data;
};


/*--------------------- RxPDO ------------------------*/

void rxPDO_message_processing(uint8_t code,struct xCanOpen *node){
    
    struct PDO_object* pdo = node->pdo[code];
    
    if(code > 7) return;
    if(node->mode != OPERATIONAL) return;
    if(pdo->cob_id&0x80000000) return;
    
    copy_rxPDO_message_to_array(node->current_msg,pdo);
    pdo->status |= NEW_MSG_PDO;
};

/*-------------------  RxSDO  -----------------------*/
    
void rxSDO_message_processing(uint8_t code,struct xCanOpen* node){// client -> server 
    
    void (*object_call)(CanOpen_msg * ,void* );
    struct SDO_object* sdo = node->sdo[code];
    struct OD_object*  tab;
    
    if(node->mode != OPERATIONAL || node->mode != PRE_OPERATIONAL) return;
    if(sdo->cob_id_server&0x80000000) return;
    if(node->current_msg->frame_sdo.id != sdo->cob_id_server) return;
    
    tab =  OD_search_msg_index(node->current_msg,node->map);
    if(!tab)return;
    if(!(tab->func_data) || !(tab->data)) return;
    
    object_call = tab->func_data;
    object_call(node->current_msg,tab->data);
    
    node->current_msg->frame_sdo.id = sdo->cob_id_server;
    node->sending_message(node->current_msg); // server -> client
};    
   
/*---------------------- message_processing Node ----------------------*/

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

void Processing_pdo_objects(struct xCanOpen* node){};







#endif	/* CANOPEN_H */

