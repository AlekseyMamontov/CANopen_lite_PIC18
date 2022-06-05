#include <stdint.h> 
#include <stdio.h>

	
#define  SDO_Transmit 0x580
#define  SDO_Receive   0x600

#define  PD1_Transmit 0x180
#define  PD2_Transmit 0x280 
#define  PD3_Transmit 0x380
#define  PD4_Transmit 0x480 
	
#define  PD1_Receive 0x200
#define  PD2_Receive 0x300
#define  PD3_Receive 0x400
#define  PD4_Receive	 0x500

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

#define PDO_communication	0x20
#define PDO_mapping	0x21
#define SDO_parameter	0x22
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
	
#define rxPDO_disabled	0x80000000 // 0b1000 0000 0000 0000
#define PDO_is_transmit	0x80 //0b1000 0000
#define PDO_calculate_n	0x700 // 0b111 0000 0000
#define PDO_mask_addr	0x7FF // 0b0111 1111 1111
	
	
/**/	
#define OK_OD_read8	0x4F
#define OK_OD_read16	0x4B
#define OK_OD_read32	0x43
	
#define OD_save_u8		0x2F
#define OD_save_u16		0x2B
#define OD_save_u32		0x23

#define OK_OD_save		0x60
#define ERORR_command	0x80		
	

#define ERORR_no_save	0x06010002
#define ERORR_no_correct_data	0x06090031

#define ERROR_sub_index	0x06090011
#define ERROR_no_object	0x06020000
#define ERROR_no_open	0x06010000
#define ERROR_ALL_OD_table	0x08000000

	



struct OD_subindex{
/*VAR,DEFTYPE,ARRAY,DEFSTRUCT,DOMAIN,RECORD*/	
uint8_t	object;
/*_BOOLEAN,INTENGER8 ....UNSIGNED32,PDO_MAP.. etc. */
uint8_t	type;
uint8_t	attr;
/* sizeOf()*/
uint8_t	n_byte_data;
/*(void*) INTENGER8 ...UNSIGNED32 *, void (*func)(*,*)*/
void*	data;
};
/* OD_index*/
struct OD_table{
uint16_t	index;
uint8_t		nsub;
struct OD_subindex*	subindex;
};

/* SDO segment block */

struct SDO_block{
uint8_t				command;
uint16_t			index; 
uint8_t				sub_index;
struct OD_subindex*	object;
uint8_t*			data;
uint8_t				n_byte;	
};
/* PDO segment block */
struct PDO_block{
struct OD_subindex*	object;
uint8_t				n_object;
uint8_t 			buffer[8];	
};


/*Node CAN*/
struct CAN_node{


/*pre-orintal etc.*/
uint8_t mode;

/*search */	
struct OD_table*	first_od_table;
struct OD_table*	current_od_table;
struct OD_subindex*	current_subindex;
uint32_t 			error_search;	
uint16_t			search_index; 
uint8_t				search_sub_index;
struct OD_table*	
(*Search_index)(struct CAN_node* node);

/*SDO protocol segment*/	

/*upload*/
struct SDO_block* sdo_upload;
uint8_t 		n_sdo_upload;
/*download*/
struct SDO_block* sdo_download;
uint8_t			n_sdo_download;

/*PDO protocol */

/*RPDO*/
struct PDO_block*	rpdo;
uint8_t				n_rpdo;

/*TPDO*/
struct PDO_block*	tpdo;
uint8_t				n_tpdo;


};

/* Search OD_table and return struct OD_table *
 * complete simple list iteration */

struct OD_table *OD_search_index(struct OD_table *tab,uint16_t index){	
 uint8_t err = 1;
 	
	while  (tab->index <= index && tab->index !=0 ){ 		
		if (tab->index == index){err = 0;break;}
		tab++;
	};
	if (err) tab = NULL;
	
 return tab;};

//Search_CAN_Node index ->node->current_od_table//
struct OD_table *CAN_node_search_index(struct CAN_node *node){
return node->current_od_table = OD_search_index(node->current_od_table,node->index);
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
//Search_CAN_Node subindex->node->current_subindex//
struct OD_subindex *CAN_node_search_subindex(struct CAN_node *node){
return node->current_subindex = OD_search_subindex(node->current_od_table,node->sub_index);
}

/* Search OD_subindex and return struct OD_subindex */
struct OD_subindex* OD_search_element(struct OD_table *tab,uint16_t index, uint16_t subindex, uint32_t *error){

	struct OD_subindex* sub = NULL;

	if ((tab = OD_search_index(tab,index)) != NULL){ 

		if (tab->subindex != NULL && tab->index != 0){
			
				if ((sub = OD_search_subindex(tab,subindex)) == NULL){*error = ERROR_sub_index;};
		
		}else{*error = ERROR_no_object;};
				
	}else{*error = ERROR_no_object;}
		
return sub;	
};

/* Search OD_index&subindex and return struct OD_subindex */
struct OD_subindex * OD_Search_table_element(struct CAN_node *node){
	node->current_od_table = node->first_od_table;
	return node->current_subindex = OD_search_element(  
		node->current_od_table,
		node->search_index,
		node->search_sub_index,
		&node->error_search);
};


/* litlle-endian*/

#define DEFMAXSIZE 4

uint8_t OD_read_deftype_element(struct OD_subindex *sub,void* buf, uint8_t len){

	for(uint8_t i = 0; i< len; i++){	
	     *((uint8_t*)buf+i) = (i < sub->n_byte_data)? *((uint8_t*)sub->data +i) : 0;	
	}
} ;

uint8_t OD_save_deftype_element(struct OD_subindex *sub, void* buf){
	
	for(uint8_t i = 0; i < (sub->n_byte_data); i++){
	   *((uint8_t*)sub->data +i) = *((uint8_t*)buf+i);	
	};
};






































//////////////////////////////////////////test////////////////////////////////////////
/*
struct OD_subindex{	
uint8_t	object;
uint8_t	type;
uint8_t	attr;
uint8_t	n_bayt_data;
void*	data;
};
*/
uint8_t Id_test = 0x55;

struct OD_subindex N1000[]={{OD_VAR,UINT8,RW,sizeof(Id_test),(void*)&Id_test}};


uint8_t  Id_subindex = 2;
uint32_t Id_Node = 0x555;
uint8_t  Id_Type = 0xAA;


struct OD_subindex 
N1400[]={
	{OD_DEFSTRUCT,UINT8,RO,1,(void*)&Id_subindex},
	{OD_VAR,UINT32,RW,sizeof(Id_Node),(void*)&Id_Node},				
	{OD_VAR,UINT8,RW,sizeof(Id_test),(void*)&Id_Type},
};

/* OD_index
struct OD_table{
uint16_t	index;
uint8_t		nsub;
struct OD_subindex*	subindex;
};
*/
struct OD_table MAP_RELE[]={
{0x1000,1,N1000},
{0x1400,2,N1400},
{0x1401,3,NULL},
{0x0000,0,NULL},
};



struct CAN_node  Block_RELE={

.first_od_table = MAP_RELE,
.current_od_table = MAP_RELE,

};



void main(){


struct OD_table *n = Block_RELE.current_od_table;
struct  OD_subindex *s;
uint32_t error_code = 0;
uint8_t buf[6]={0x44,0x33,0x22,0x11};
   
	printf("*n->nsub --%x\n", n->nsub);

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





/*

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


