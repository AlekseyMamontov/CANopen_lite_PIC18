/* 
 * File:   Triac_block_12.h
 * Author: Oleksii_Mamontov
 * triac unit for 12 outputs 220v
 */

#ifndef TRIAC_BLOCK_12_H
#define	TRIAC_BLOCK_12_H

#include "CanOpen_lite.h"


#ifdef	__cplusplus
extern "C" {
#endif
 
/* DS-401*/  

// pic18 256 byte ram

struct RAM_to_EEPROM{
 
 // rxPDO1 4+1+2+2+1+4x8 = 42 byte
 
 uint32_t rxpdo1_cob_id;
 uint8_t  rxpdo1_Transmission;
 uint16_t rxpdo1_inhibit_time;
 uint16_t rxpdo1_event_timer;
 uint8_t  rxpdo1_sub_index_map;
 uint32_t rxpdo1_map_data[MAX_MAP_DATA];
 //uint8_t* rxpdo_map_addr_obj[MAX_MAP_DATA];

 // txPDO1 4+1+2+2+1+4x8 = 42 byte
 
 uint32_t txpdo1_cob_id;
 uint8_t  txpdo1_Transmission;
 uint16_t txpdo1_inhibit_time;
 uint16_t txpdo1_event_timer;
 uint8_t  txpdo1_sub_index_map;
 uint32_t txpdo1_map_data[MAX_MAP_DATA];
 
 
 
 // 6000  2 byte
 
 uint8_t polary_input[1];  //6002h
 uint8_t filter_input[1];  //6003h
 
 // 6200  6 byte
 
 uint8_t port_output[2] ;  //6200h
 uint8_t polary_output[2]; //6202h
 uint8_t filter_output[2]; //6203h
 

 };

static struct _CanOpen  Triac_rele;
struct RAM_to_EEPROM    Triac_rele_ram;
static struct OD_Object OD_Triac_rele[]; // OD_table    

/* 401d (0x191) | 16th Bit Digital input | 17th Bit Digital output */

const uint32_t

 N1000_Device_Type = 30191,
 N1008_Device_name =  0,
 N1009_Hard_version = 0,
 N100A_Soft_version = 0;


 uint8_t  
        
 // 0 - port B, 1 - port C
        
 port_output[2]    ={0x00,0x00},//6200h
 polary_output[2]  ={0x00,0x00},//6202h
 filter_output[2]  ={0xFF,0x0F},//6208h
        
 // AC220V portA.5 
        
 port_input[1]   = {0},  //6000h
 polary_input[1] = {0},  //6002h
 filter_input[1] = {0};  //6003h 


 struct one_type_array
 
 N6200_output = { .sub_index = 2, .array = port_output},
 N6202_output = { .sub_index = 2, .array = polary_output},
 N6208_output = { .sub_index = 2, .array = filter_output},
         
 N6000_input = { .sub_index = 1, .array = port_input},
 N6002_input = { .sub_index = 1, .array = polary_input},
 N6003_input = { .sub_index = 1, .array = filter_input};        
         

/*------------------- Error -----------------------*/

uint8_t  N1001_Error_register = 0;   
uint32_t data_error[5];
struct 
one_type_array N1003_Error = { .sub_index = 5, .array = data_error };
 
/*------------------ rxPDO1 -----------------------*/
// ram

uint8_t rxpdo1_cond = 0x20,
	rxpdo1_subindex = 0x05,
	rxpdo1_counter_sync = 0,
	rxpdo1_n_byte_pdo_map = 0,
	rxpdo1_buffer[MAX_MAP_DATA]={},
       *rxpdo_map_addr_obj[MAX_MAP_DATA] = {};
	
uint16_t rxpdo1_counter_inhibit_time = 0,
	 rxpdo1_counter_event_timer = 0;
	


// rom
static struct PDO_Object rx_pdo_0x200={

    .cond = &rxpdo1_cond,
    .cob_id = &Triac_rele_ram.rxpdo1_cob_id,
    
    .sub_index = &rxpdo1_subindex,
    .Transmission_type = &Triac_rele_ram.rxpdo1_Transmission,
    .Sync_start_value = NULL,
    .counter_sync = &rxpdo1_counter_sync,
    
    .Inhibit_time = &Triac_rele_ram.rxpdo1_inhibit_time, 	// n x 100ms
    .counter_Inhibit_time = &rxpdo1_counter_inhibit_time, // 0 <-- (Inhibit_time --)
       
    .Event_timer = &Triac_rele_ram.rxpdo1_event_timer,	// n x 100ms
    .counter_Event_timer= &rxpdo1_counter_event_timer,  // 0 <-- (Event_timer--)
    
    // mapping
    
    .sub_index_map = &Triac_rele_ram.rxpdo1_sub_index_map,
    .map = Triac_rele_ram.rxpdo1_map_data,	      // massiv[MAX_MAP_DATA]
    .map_addr_obj = rxpdo_map_addr_obj,		      // massiv[MAX_MAP_DATA]
    .n_byte_pdo_map = &rxpdo1_n_byte_pdo_map,

    .OD_Object_list = OD_Triac_rele, // 
    
    // Buffer
    
    .buffer = rxpdo1_buffer,

    // function
    
    .init_pdo = init_xPDO,
    .build_map = build_map_objects,
    .process_rxpdo = process_the_RxPDO_message,
    .process_txpdo = process_the_TxPDO_message,
};


/******---------- txPDO1 ---------------*********/
// ram

uint8_t txpdo1_cond = 0x60, // setInit,setTx
	txpdo1_subindex = 0x05,
	txpdo1_counter_sync = 0x0,
	txpdo1_buffer[MAX_MAP_DATA]={},
	txpdo1_n_byte_pdo_map = 0, 
	*txpdo_map_addr_obj[MAX_MAP_DATA] = {};
	
uint16_t txpdo1_counter_inhibit_time = 0,
	 txpdo1_counter_event_timer = 0;


// rom

struct PDO_Object tx_pdo_0x180={

    .cond = &txpdo1_cond,
    .cob_id = &Triac_rele_ram.txpdo1_cob_id,
    
    .sub_index = &txpdo1_subindex,
    .Transmission_type = &Triac_rele_ram.txpdo1_Transmission,
    .Sync_start_value = NULL,
    .counter_sync = &txpdo1_counter_sync,
    
    .Inhibit_time = &Triac_rele_ram.txpdo1_inhibit_time, 	// n x 100ms
    .counter_Inhibit_time = &txpdo1_counter_inhibit_time, // 0 <-- (Inhibit_time --)
       
    .Event_timer = &Triac_rele_ram.txpdo1_event_timer,	// n x 100ms
    .counter_Event_timer= &txpdo1_counter_event_timer,  // 0 <-- (Event_timer--)
    
    // mapping
    
    .sub_index_map = &Triac_rele_ram.txpdo1_sub_index_map,
    .map = Triac_rele_ram.txpdo1_map_data,	      // massiv[MAX_MAP_DATA]
    .map_addr_obj = txpdo_map_addr_obj, // massiv[MAX_MAP_DATA]
    .n_byte_pdo_map = &txpdo1_n_byte_pdo_map,

    .OD_Object_list = OD_Triac_rele, // 
    
    // Buffer
    
    .buffer = txpdo1_buffer,

    // function
    
    .init_pdo = init_xPDO,
    .build_map = build_map_objects,
    .process_rxpdo = process_the_RxPDO_message,
    .process_txpdo = process_the_TxPDO_message,
};





/******* SDO  *********/

struct SDO_Object sdo_Triac_rele={

  
    
};   
    
   
/* =============== NODE _CANopen ================ */

//struct OD_object OD_Triac[2];

/*
struct _CanOpen{

    uint8_t*                  id; 
    uint8_t*                mode; 

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
 
 */





 struct PDO_Object* xPDO[8] ={&tx_pdo_0x180, &rx_pdo_0x200,NULL};
 
 uint8_t node_id = 0,
	 node_mode = BOOT;
 
 CanOpen_Msg node_msg;
 
 
static
struct _CanOpen Triac_rele = {
	
	.id = &node_id,
	.mode = &node_mode,
	.current_msg = &node_msg,
	.pdo = xPDO,

};    
 

// OD_table

    static struct OD_Object OD_Triac_rele[18]={

    {0x1000,(void*)&N1000_Device_Type,   ro_object_4byte},
    {0x1001,(void*)&N1001_Error_register,ro_object_1byte},
    {0x1003,(void*)&N1003_Error,        ro_array_4byte},
    {0x1008,(void*)&N1008_Device_name,  ro_object_4byte},
    {0x1009,(void*)&N1009_Hard_version, ro_object_4byte},
    {0x100A,(void*)&N100A_Soft_version, ro_object_4byte},
    
    {0x1200,(void*)&sdo_Triac_rele,ro_sdo_object},
    
    {0x1400,(void*)&rx_pdo_0x200, rw_pdo_object},
    {0x1600,(void*)&rx_pdo_0x200, rw_map_object},
    
    {0x1800,(void*)&tx_pdo_0x180, rw_pdo_object},
    {0x1A00,(void*)&tx_pdo_0x180, ro_map_object},
   
    {0x6000,(void*)&N6000_input,ro_array_1byte},
    {0x6002,(void*)&N6002_input,rw_array_1byte},
    {0x6003,(void*)&N6003_input,rw_array_1byte},    
      
    {0x6200,(void*)&N6200_output,rw_array_1byte},
    {0x6202,(void*)&N6202_output,rw_array_1byte},
    {0x6208,(void*)&N6208_output,rw_array_1byte},

    {0xffff,NULL,NULL},
        
};     
   
    
/****  work with GPIO    
*****  read  address (Node_ID) chip 74h165 *******/

uint8_t Read_addr_CAN(void){
	
   uint8_t addr = 0;
   uint8_t bit_addr =0;
   
     PL_165_SetHigh() ;
     SCK_165_SetHigh();
      
     PL_165_SetLow() ;
     __delay_ms(5);
     PL_165_SetHigh() ;
     
     // cycle 8 bit
     
     for(uint8_t b=0;b<8;b++){
         
         bit_addr = DATA_165_GetValue()&1;
         bit_addr <<= b;
         addr |=bit_addr; 
         SCK_165_SetLow();
         __delay_ms(5);
         SCK_165_SetHigh();
     };
     return addr;
 };	


 
 
 
 
 /*
 output 
 Rele 1-8  portB 0-7 
 Rele 9-12 portC 0-3
 StatusLEd portA.3
   
 input
 AC220V portA.5 
 
*/
 
void GPIO_processing(){ 
    
     uint8_t 
     mask = filter_output[0],       
     data = (port_output[0]^polary_output[0])&mask;
     
     
     if(data != (LATB&mask)){
        
         data |=(LATB&(~mask));
         LATB = data; 
         
     };
     
     mask = filter_output[1]&0x0F;
     data = (port_output[1]^polary_output[1])&mask;
     
    if(data != (LATC&mask)){
        
         data |=(LATC&(~mask));
         LATC = data; 
         
     };
     
     data = AC220V_GetValue()?1:0;
     data = (data^polary_input[0])&filter_input[0];
     
     if(data != port_input[0]){
         
         port_input[0]= data;
         //tx_pdo_0x180.cond.flag.event_txpdo = 1;
         
     };    
 }

/******** irq *********************************/

void irq_Triac_rele_100us(void){

   if(rxpdo1_counter_inhibit_time)rxpdo1_counter_inhibit_time --;
   if(rxpdo1_counter_event_timer) rxpdo1_counter_event_timer--;
   if(txpdo1_counter_inhibit_time)txpdo1_counter_inhibit_time --;
   if(txpdo1_counter_event_timer) txpdo1_counter_event_timer--;
   
};

void irq_default(){};

/********   *modification can init ************/

 void CANOPEN_ECAN_Initialize(uint8_t id)
{
    uint16_t cob_id = id;
             cob_id <<= 5;
    uint8_t  cob_id_h =  (cob_id&0xFF00)>>8;
    uint8_t  cob_id_l =  (cob_id&0x00FF);

    CANCON = 0x80;
    while (0x80 != (CANSTAT & 0xE0)); // wait until ECAN is in config mode

    /* Mode 2 */
    ECANCONbits.MDSEL = 0x02;

    /* Initialize CAN I/O */
    CIOCON = 0x20;
    
    /*Configure Generic Buffers to be Transmit or Receive*/
    BSEL0 = 0x00;
    
    /* Initialize Receive Masks */
    RXM0EIDH = 0xFF;RXM0EIDL = 0xFF;RXM0SIDH = 0xFF;RXM0SIDL = 0xE3; 
    RXM1EIDH = 0xFF;RXM1EIDL = 0xFF;RXM1SIDH = 0xFF;RXM1SIDL = 0xE3;
    
    /* Enable Filters */
    RXFCON0 = 0xFF; RXFCON1 = 0x00;
    
    /* Assign Filters to Masks */
    MSEL0 = 0x50;
    MSEL1 = 0x55;
    MSEL2 = 0x00;
    MSEL3 = 0x00;
    
    /* Initialize Receive Filters */
    RXF0EIDH = 0x00; RXF0EIDL = 0x00;
    RXF0SIDH = 0x00; RXF0SIDL = 0x00; // 0x00
    
    RXF1EIDH = 0x00; RXF1EIDL = 0x00;
    RXF1SIDH = 0x10; RXF1SIDL = 0x00; // 0x80
    
    // 0x200+cob_id PDO1   
    RXF2EIDH = 0x00; RXF2EIDL = 0x00;
    RXF2SIDH = 0x40|cob_id_h; 
    RXF2SIDL = cob_id_l;
    
    // 0x300+cob_id PDO2
    RXF3EIDH = 0x00; RXF3EIDL = 0x00;
    RXF3SIDH = 0x60|cob_id_h;
    RXF3SIDL = cob_id_l; 
    
    // 0x400+cob_id PDO3
    RXF4EIDH = 0x00; RXF4EIDL = 0x00;
    RXF4SIDH = 0x80|cob_id_h;
    RXF4SIDL = cob_id_l; 
    
    // 0x500+cob_id PDO4
    RXF5EIDH = 0x00; RXF5EIDL = 0x00;
    RXF5SIDH = 0xA0|cob_id_h;
    RXF5SIDL = cob_id_l; 
    
    // 0x600+cob_id rxSDO
    RXF6EIDH = 0x00; RXF6EIDL = 0x00;
    RXF6SIDH = 0xC0|cob_id_h;
    RXF6SIDL = cob_id_l;
    
    // 0x700 NMT error
    RXF7EIDH = 0x00;RXF7EIDL = 0x00;
    RXF7SIDH = 0xE0|cob_id_h;
    RXF7SIDL = cob_id_l;  
    
    RXF8EIDH =  0x00; RXF8EIDL =  0x00; RXF8SIDH = 0x00; RXF8SIDL =  0x00;
    RXF9EIDH =  0x00; RXF9EIDL =  0x00; RXF9SIDH = 0x00; RXF9SIDL =  0x00;
    RXF10EIDH = 0x00; RXF10EIDL = 0x00;RXF10SIDH = 0x00; RXF10SIDL = 0x00;
    RXF11EIDH = 0x00; RXF11EIDL = 0x00;RXF11SIDH = 0x00; RXF11SIDL = 0x00;
    RXF12EIDH = 0x00; RXF12EIDL = 0x00;RXF12SIDH = 0x00; RXF12SIDL = 0x00; 
    RXF13EIDH = 0x00; RXF13EIDL = 0x00;RXF13SIDH = 0x00; RXF13SIDL = 0x00;  
    RXF14EIDH = 0x00; RXF14EIDL = 0x00;RXF14SIDH = 0x00; RXF14SIDL = 0x00;
    
    //mask
    RXF15EIDH = 0xFF;RXF15EIDL = 0xFF;RXF15SIDH = 0xFF;RXF15SIDL = 0xE3;
    /**
    Initialize CAN Timings
    */
    
   /**
	Baud rate: 500kbps
	System frequency: 16000000
    ECAN clock frequency: 16000000
	Time quanta: 8
	Sample point: 1-1-4-2
	Sample point: 75%
	*/
    
    BRGCON1 = 0x01;
    BRGCON2 = 0x98;
    BRGCON3 = 0x01;

    // Generate interrupt when one receive buffer remains
    ECANCONbits.FIFOWM = 1;
    
    ECAN_SetWatermarkInterruptHandler(irq_default);
    PIR5bits.FIFOWMIF = 0;
    PIE5bits.FIFOWMIE = 1;
    
    ECAN_SetWakeUpInterruptHandler(irq_default);
    PIR5bits.WAKIF = 0;
    PIE5bits.WAKIE = 1;
    
    CANCON = 0x00;
    while (0x00 != (CANSTAT & 0xE0)); // wait until ECAN is in Normal mode

}

 
#ifdef	__cplusplus
}
#endif

#endif	/* TRIAC_BLOCK_12_H */

