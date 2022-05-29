/* 
 * File:   CANopenLite.h
 * Author: aleksey
 *
 * Created on 25 ??? 2022 ?., 14:38
 */

#ifndef CANOPENLITE_H
#define	CANOPENLITE_H

#ifdef	__cplusplus
extern "C" {
#endif
	
#define  SDO_Transmit 0x580
#define  SDO_Receive 0x600

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

#define _BOOLEAN	01
#define INTENGER8   02
#define INTENGER16  03
#define INTENGER32  04	
#define UNSIGNED8	 05
#define UNSIGNED16	 06
#define UNSIGNED32	 07

#define OD_NULL 		0
#define OD_DOMAIN 		2
#define OD_RECORD 		9
#define OD_DEFSTRUCT	6
#define OD_DEFTYPE 		5
#define OD_VAR 			7	
#define OD_ARRAY 		8	

#define ERROR_sub_index 0x06090011
#define ERROR_no_object 0x06020000
#define ERROR_no_open   0x06010000
#define ERROR_ALL_OD_table 0x08000000

struct OD_table{
	
/* index */	
uint16_t	index;
/*sub_index*/
uint8_t	sub_index;

/* NULL 0/  OD_DOMAIN 2 / OD_RECORD 9 /OD_DEFSTRUCT 6
OD_DEFTYPE 5 / OD_VAR 7 / OD_ARRAY 8  */
uint8_t	object;
/*
BOOLEAN		01
INTENGER8   02
INTENGER16  03
INTENGER32  04	
UNSIGNED8	05
UNSIGNED16	06
UNSIGNED32	07
 */
uint8_t	type;
/*
BOOLEAN	   *;
INTENGER8  *;
INTENGER16 *;
INTENGER32 *;	
UNSIGNED8  *;
UNSIGNED16 *;
UNSIGNED32 *;
void (*func)(*,*);
 */
void*	data;

/* lsizeOf()*/
uint8_t	n_bayt;

//struct OD_table *next;
//struct OD_table *prev;
};



struct CAN_Node{
	
/*search */	
struct OD_table *first_od_table;
struct OD_table *current_od_table;
uint32_t	error_search;	
uint16_t	index; 
uint8_t 	sub_index;


/**/	
	


};


/* Search OD_table and return struct OD_table *
 * complete simple list iteration */

struct OD_table *OD_serch_index0(struct OD_table *tab,uint16_t index){
	
  uint8_t err = 1;	
  
	while  (tab->index <= index){ 
		if (tab->index == 0)break;   
		if (tab->index == index){
			if(tab->sub_index == 0)err=0;//its ok 	
		break;}
	tab++;};
	if (err) tab = NULL;
	
 return tab;};
 
 /*version 1*/
 
struct OD_table *OD_search_sub_index(struct OD_table *tab,uint8_t sub_index){
	
	if (tab->object == OD_VAR ||  tab->object == OD_DEFTYPE ) return tab;
	if (tab->data != NULL && *((uint8_t*)tab->data) < sub_index ){
		
		tab += sub_index;
		
	 }else{tab = NULL;};	 
	 
return tab;}

/* version  2*/

struct OD_table *OD_search_subindex(struct OD_table *tab,uint8_t sub_index){
	
  uint16_t index = tab->index;	
  uint8_t err = 1;
  
	while(tab->index == index){
	       if  (tab->sub_index == sub_index){err=0; break;}
	       tab++;};
	       
	if (err) tab = NULL;  	
		
return tab;};

/* Search OD_table and return struct OD_table *
 * complete simple list iteration
 * Metod 1
 * */

uint8_t OD_search_N( struct CAN_Node *node){

struct OD_table *tab = node->current_od_table;
uint16_t index = node->index;
uint8_t   sub_index = node->sub_index;
uint8_t   error =2;

    while  (tab->index <= index){ 
	
	if (tab->index == 0)break;   
	if (tab->index == index){
	      if (tab->sub_index != sub_index){error = 1;}else{error=0; break;}; //its ok
	};
	
	tab++;
    };
    switch(error){
	case 0:node->error_search= 0;break;
	case 1:node->error_search= ERROR_sub_index;break;
	default : node->error_search= ERROR_no_object;tab =NULL;break;
   };		
node->current_od_table = tab;		
return error;
}

/* Search OD_table  */

struct OD_table *
OD_search_all( struct CAN_Node *node, uint16_t index,uint8_t sub_index){
	node->index = index;
	node->sub_index = sub_index;
	node->current_od_table = node->first_od_table;
	OD_search_N (node); 	
	return node->current_od_table;
};


/* Search OD_table and return struct OD_table *
 * complete simple list iteration
 * Metod 2
 * */

uint8_t OD_search__num (struct OD_table **ntab,uint16_t index,uint8_t sub_index ){

struct OD_table *tab = *ntab;

uint8_t   error =2;

    while  (tab->index <= index){ 	
	if (tab->index == 0)break;   
	if (tab->index == index){
	      if (tab->sub_index != sub_index){error = 1;
	      }else{error=0; break;}; //its ok
	};	
	tab++;
    };
    
    *ntab = tab ;		
	
return error;}












#ifdef	__cplusplus
}
#endif

#endif	/* CANOPENLITE_H */

