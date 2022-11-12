/* 
 * File:   CANopen.h
 * Author: oleksii
 *
 * Created on  2022 ?., 20:46
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

// ------- function code -----------------

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
#define RO      0b01
#define WO      0b10
#define RW      0b11	
	
/**/
	
#define PDO_DISABLED	0x80000000 // 0b1000 0000 0000 0000
#define PDO_is_transmit	0x80 	   //0b1000 0000
#define PDO_calculate_n	0x700 // 0b111 0000 0000
#define PDO_mask_addr	0x7FF // 0b0111 1111 1111
	
	

//----------------  OD_TABLE  -----------------------------

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
// ------------------------- fast od table-------------------

typedef union {
    void*  data;
    void*  (*record)(void *);
}sub_data;

struct fast_table_index{
    uint16_t index;
    sub_data sub_index;
};

// ---------------------------------------------------------------------

struct PDO_object{
    
//uint8_t		sub_index ;
    
uint32_t	cob_id ;
uint8_t		Transmission_type;
uint16_t	Inhibit_time;
// uint8_t		none; 	
uint16_t	event_timer;
// quick access to the structure map
uint8_t     status;
uint8_t     data[8];
//void *    pdo_map;
};

union map_data{
	struct{
	uint8_t  nbit; 
	uint8_t  sub_index;
	uint16_t index;
	}		 parameter;
	uint32_t data32;
};

struct xPDO_map {
    uint8_t         sub_index;
    union map_data  map[8];
    // quick access to the map
    void *          hidden_map[8];    
};


struct SDO_object{
	
uint8_t		sub_index;
uint32_t	cob_id_client;
uint32_t	cob_id_server;
uint8_t		node_id;
};
// --------------- union -----------------

#define COB_ID      array[1]&0x7f

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



///////////////////////////////


struct xCanOpen{

uint8_t              cob_id; /*id */
uint8_t                mode; /*pre-orintal etc.*/

CanOpen_msg*    current_msg;

struct PDO_object*   pdo[8];
struct SDO_object*   sdo[2];


void  (*object_call[8])(uint8_t ,void*);

};

//////////// function obiect call ////////////

#define FUNC_MSG        0x01
#define FUNC_SDO_READ   0x02
#define FUNC_SDO_SAVE   0x03
#define FUNC_SYSTEM     0x04

#define RESPONSE_1b  0x4f
#define RESPONSE_2b  0x4B
#define RESPONSE_3b  0x47
#define RESPONSE_4b  0x43

#define GET_1b  		0x2F
#define GET_2b  		0x2B
#define GET_4b  		0x23

#define OK_SAVE         0x60

#define RESPONSE_ERROR  0x80		
	

#define ERORR_NO_SAVE		0x06010002
#define ERROR_OBJECT        0x06070010
#define ERORR_NO_CORRECT    0x06090031

#define ERROR_SUB_INDEX		0x06090011
#define ERROR_no_object		0x06020000
#define ERROR_no_open		0x06010000
#define ERROR_ALL_OD_table	0x08000000





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


//------------- xPDO object ----------------------

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
            
        case FUNC_SDO_READ:
            
            if (node->mode != OPERATIONAL || 
                node->mode != PRE_OPERATIONAL) break;
            
            node->current_msg->frame_sdo.data.data32 = 0;
                 
            switch(node->current_msg->frame_sdo.subindex){ 
                
                case 0:  
                node->current_msg->frame_sdo.data.data8 = 0x05;
                node->current_msg->frame_sdo.cmd = RESPONSE_1b;
                node->current_msg->frame_sdo.dlc = 5;
                    break;
                case 1:
                node->current_msg->frame_sdo.data.data32 =
                        node->pdo[RxPDO1]->cob_id;
                node->current_msg->frame_sdo.cmd = RESPONSE_4b;
                node->current_msg->frame_sdo.dlc = 8;
                    break;
                case 2:
                node->current_msg->frame_sdo.data.data8 =
                        node->pdo[RxPDO1]->Transmission_type;
                node->current_msg->frame_sdo.cmd = RESPONSE_1b;
                node->current_msg->frame_sdo.dlc = 5;
                    break;
                case 3:
                node->current_msg->frame_sdo.data.data16 =
                        node->pdo[RxPDO1]->Inhibit_time;
                node->current_msg->frame_sdo.cmd = RESPONSE_2b;
                node->current_msg->frame_sdo.dlc = 6;
                
                    break;                    
                case 5:
                node->current_msg->frame_sdo.data.data16 =
                        node->pdo[RxPDO1]->event_timer;
                node->current_msg->frame_sdo.cmd = RESPONSE_2b;
                node->current_msg->frame_sdo.dlc = 6;
                    break;
                    
                default:
                 node->current_msg->frame_sdo.cmd = RESPONSE_ERROR;
                 node->current_msg->frame_sdo.data.data32 = ERROR_SUB_INDEX;
                 node->current_msg->frame_sdo.dlc = 8;
                break;
                    
            }; 
 
         break;
            
        case FUNC_SDO_SAVE:
            
             if (node->mode != OPERATIONAL || 
                 node->mode != PRE_OPERATIONAL) break; 
             
              switch(node->current_msg->frame_sdo.subindex){ 
                    
                case 0 :
                   error = 1;  
                   node->current_msg->frame_sdo.data.data32 = ERORR_NO_SAVE;   
                   break;
                        
                case 1: // sub-id 32b               
                    
                    if(node->current_msg->frame_sdo.cmd != GET_4b && 
                       node->current_msg->frame_sdo.dlc != 8){
                       error = 1; 
                       node->current_msg->frame_sdo.data.data32= ERROR_OBJECT;   
                       break;}
                                   
                    if((node->current_msg->can_frame.id&0x7FF) == 
                        (node->pdo[RxPDO1]->cob_id & 0x7ff)){
                    
                            node->pdo[RxPDO1]->cob_id =
                            node->current_msg->can_frame.id&0x800007FF;        
                     
                    }else{
                            error = 1;
                            node->current_msg->frame_sdo.data.data32 = 
                            ERORR_NO_SAVE;}                
                    break;
                    
                case 2: //Transmission_type   1b  
                    
                    if (lock){ 
                        
                         if(node->current_msg->frame_sdo.cmd != GET_1b && 
                            node->current_msg->frame_sdo.dlc >= 5){
                            error = 1; 
                            node->current_msg->frame_sdo.data.data32=
                            ERORR_NO_CORRECT;
                            break;}
                                
                            node->pdo[RxPDO1]->Transmission_type = 
                                    node->current_msg->frame_sdo.data.data8;
                    
                            }else{
                            error = 1;
                            node->current_msg->frame_sdo.data.data32 = 
                            ERORR_NO_SAVE;
                            }; 

                    break;
                    
                case 3: //Inhibit_time; 2b
                    
                     if (lock){ 
                        
                         if(node->current_msg->frame_sdo.cmd != GET_2b && 
                            node->current_msg->frame_sdo.dlc >= 6){
                            error = 1; 
                            node->current_msg->frame_sdo.data.data32=
                            ERORR_NO_CORRECT;
                            break;}
                                
                            node->pdo[RxPDO1]->Inhibit_time = 
                                    node->current_msg->frame_sdo.data.data16;
                    
                            }else{
                            error = 1;
                            node->current_msg->frame_sdo.data.data32 = 
                            ERORR_NO_SAVE;
                            };                   
                            
                    break;
                case 5: // event_timer  2b
                     
                    if (lock){ 
                        
                         if(node->current_msg->frame_sdo.cmd != GET_2b && 
                            node->current_msg->frame_sdo.dlc >= 6){
                            error = 1; 
                            node->current_msg->frame_sdo.data.data32=
                            ERORR_NO_CORRECT;
                            break;}
                                
                            node->pdo[RxPDO1]->event_timer = 
                                    node->current_msg->frame_sdo.data.data16;
                    
                            }else{
                            error = 1;
                            node->current_msg->frame_sdo.data.data32 = 
                            ERORR_NO_SAVE;
                            };                   
                    break;
                    
                default:
                 error=1;
                 node->current_msg->frame_sdo.data.data32 = ERROR_SUB_INDEX;                      
                break;
                    
            };  
             
            if(error){
                 node->current_msg->frame_sdo.cmd = RESPONSE_ERROR;
                 node->current_msg->frame_sdo.dlc = 8;
                }else{
                 node->current_msg->frame_sdo.cmd=OK_SAVE;
                 node->current_msg->frame_sdo.dlc = 4;
                };               
         break;    
            
        case FUNC_SYSTEM:
            
           if(lock) break;
           
           switch(node->pdo[RxPDO1]->Transmission_type){
               
               case 0xFE:
                   break;
                   
               case 0xFF:
                   break;
               
               
               
               
               
               
               
               
               
               
               
           
           
           
           
           
           
           
           
           
           
           
           
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

