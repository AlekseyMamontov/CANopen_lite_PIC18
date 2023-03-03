/* 
 * File:   Triac_block_12.h
 * Author: Oleksii_Mamontov
 * triac unit for 12 outputs 220v
 */

#ifndef TRIAC_BLOCK_12_H
#define	TRIAC_BLOCK_12_H

#include "CANOPEN.h"


#ifdef	__cplusplus
extern "C" {
#endif
 
  
    
    

uint32_t

/* 401d (0x191) | 16th Bit Digital input | 17th Bit Digital output */

 N1000_Device_Type = 30191,
 N1008_Device_name = 0,
 N1009_Hard_version = 0,
 N100A_Soft_version = 0;


 /* DS-401*/   
    
 uint8_t  
        
 // 0 - port B, 1 - port C
        
 output_port[2]    ={0x00,0x00},//6200h
 polary_output[2]  ={0x00,0x00},//6202h
 filter_output[2]  ={0xFF,0xFF},//6208h
        
 // AC220V portA.5 
        
 input_port[1]   = {0},  //6000h
 polary_input[1] = {0},  //6002h
 filter_input[1] = {0};  //6003h 


 struct one_type_array
 
 N6200_output = { .sub_index = 2, .array =output_port },
 N6202_output = { .sub_index = 2, .array =polary_output},
 N6208_output = { .sub_index = 2, .array =filter_output},
         
 N6000_input = { .sub_index = 1, .array = input_port},
 N6002_input = { .sub_index = 1, .array = polary_input},
 N6003_input = { .sub_index = 1, .array = filter_input};        
         

// Error

uint8_t  N1001_Error_register = 0;   
uint32_t data_error[5];
struct 
one_type_array N1003_Error = { .sub_index = 5, .array = data_error };






// OD_table
static struct OD_object OD_Triac_rele[];



/******************* PDO objects ***************
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
 
 struct PDO_mapping {
    
    uint8_t           sub_index;
    union map_data    map[MAX_MAP_DATA];
    // quick access to the map
    void *            quick_mapping[MAX_MAP_DATA];
    struct OD_object* node_map;
    
};
 
 
 
 **/ 
struct func_pdo func_rele={
    
    .init_xpdo = init_xPDO,
    .process_map = map_object_check,
    .process_rxpdo = process_the_RxPDO_message,
    .process_txpdo = process_the_TxPDO_message,
 
    
};
 
/******* rxPDO1  *********/

struct PDO_mapping map_rxpdo1 = {

    .sub_index = 2,
    .map = {{0x08010062},
            {0x08020062},
            {0x00000000},
            },
    .quick_mapping = {(void*)&output_port[0],
                      (void*)&output_port[1],
                      NULL},
                      
    .node_map = OD_Triac_rele,
    
};


struct PDO_object rx_pdo1={

    .cond ={.stat = 0x20}, // initPDO
    .Transmission_type = 0xFF,
    .cob_id = rxPDO1,
    .sub_index = 5,
    .pdo_map = &map_rxpdo1,
    .n_byte_pdo_map = 2,
    .counter_sync = 0,
    .func = &func_rele,
    .data ={0},
};


/******* txPDO1  *********/

struct PDO_mapping map_txpdo1 = {

    .sub_index = 1,
    
    .map = {{0x08010060},
            {0x00000000},
            },
    .quick_mapping = {(void*)&input_port[0],
                      NULL},
                      
    .node_map = OD_Triac_rele,
    
};


struct PDO_object tx_pdo1={
    
    .cond = {.stat = 0x20}, // initPDO
    .Transmission_type = 0xFF,
    .cob_id = txPDO1,
    .sub_index = 5,
    .pdo_map = &map_rxpdo1,
    .Event_timer = 0,
    .Inhibit_time = 0,
    .n_byte_pdo_map = 1,
    .func = &func_rele,
    .data = {0},

};


/******* SDO  *********/

struct SDO_object sdo_Triac_rele={

    .cob_id_client = rxSDO,
    .cob_id_server = txSDO,
    .node_id = 0,
    .sub_index = 3,
    
};   
    
   
//struct OD_object OD_Triac[2];



static
struct xCanOpen Triac_rele = {
    
.pdo = {
        &rx_pdo1,// 200 + cob_id
        &tx_pdo1,// 180 + cob_id
        },
       
.sdo = {&sdo_Triac_rele},

.map = OD_Triac_rele,

};    
 

// OD_table
    static struct OD_object OD_Triac_rele[18]={

    {0x1000,(void*)&N1000_Device_Type, ro_object_4byte},
    {0x1001,(void*)&N1001_Error_register,ro_object_1byte},
    {0x1003,(void*)&N1003_Error,        ro_array_4byte},
    {0x1008,(void*)&N1008_Device_name,  ro_object_4byte},
    {0x1009,(void*)&N1009_Hard_version, ro_object_4byte},
    {0x100A,(void*)&N100A_Soft_version, ro_object_4byte},
    
    {0x1200,(void*)&sdo_Triac_rele,ro_sdo_object},
    
    {0x1400,(void*)&rx_pdo1, rw_pdo_object},
    {0x1600,(void*)&rx_pdo1, rw_map_object},
    
    {0x1800,(void*)&tx_pdo1, rw_pdo_object},
    {0x1A00,(void*)&tx_pdo1, ro_map_object},
   
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
     data = (output_port[0]^polary_output[0])&mask;
     
     
     if(data != (LATB&mask)){
        
         data |=(LATB&(~mask));
         LATB = data; 
         
     };
     
     mask = filter_output[1]&0x0F;
     data = (output_port[1]^polary_output[1])&mask;
     
    if(data != (LATC&mask)){
        
         data |=(LATC&(~mask));
         LATC = data; 
         
     };
     
     data = AC220V_GetValue()?1:0;
     data = (data^polary_input[0])&filter_input[0];
     
     if(data != input_port[0]){
         
         input_port[0]= data;
         tx_pdo1.cond.flag.event_txpdo = 1;
         
     };    
 }

/******** irq *********************************/

void irq_default(void){};


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

