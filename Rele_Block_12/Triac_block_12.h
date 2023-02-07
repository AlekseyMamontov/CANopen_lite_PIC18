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
 
 /* DS-401*/   
    
uint8_t    
// 0 - port B, 1 - port C
output_port[2]    ={0,0},      //6200h
output_port_old[2]={0,0},
polary_output[2]  ={0,0},      //6202h
mask_output[2]    ={0xFF,0xFF},//6208h
        
        
gpio_input = 0,         //6000h
gpio_polary_input = 0,  //6002h
gpio_mask_input   = 0;  //6003h  
    
    
const 
uint32_t

 /* 401d (0x191) | 16th Bit Digital input | 17th Bit Digital output */

 N1000_Device_Type = 30191,
 N1008_Device_name = 0,
 N1009_Hardware_version = 0,
 N100A_Software_version = 0;

// ERROR

uint8_t  N1001_Error_register = 0;   
uint32_t data_error[5];
struct 
one_type_array N1003_Error = { .sub_index = 5,
                               .array = data_error };

 

struct OD_object OD_Triac_rele[2]={

    //{.index. .data, .func_data },
    {0x1000,&N1000_Device_Type,ro_object_4byte},
};


// rxPDO1

struct PDO_mapping map_rxpdo1={

    .sub_index = 1,
    
    


};

struct PDO_object rx_pdo1={

    .cond = 0,
    .Transmission_type = 0xFF,
    .cob_id = 0x200,
    .sub_index = 5,
    .pdo_map = &map_rxpdo1,

};


struct func_pdo func_rele={
    
    .init_xpdo = init_xPDO,
    .process_map = map_object_check,
    .process_rxpdo = process_the_RxPDO_message,
    .process_txpdo = process_the_TxPDO_message,
    .start_Inhibit_timer=0,
    .start_event_timer=0,
    
};




struct PDO_object tx_pdo1={


};

struct SDO_object sdo_rx_tx;   
    
    
struct xCanOpen Triac_rele = {
.pdo = {
        &rx_pdo1,// 180 + cob_id
        &tx_pdo1,// 200 + cob_id
        },
       
.sdo = {&sdo_rx_tx},
};    
 
     
    
    
    
    
    
    
    
    
    
/*    
 read  address (Node_ID) chip 74h165
*/

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

void irq_default(void){};

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


 void GPIO_processing(){
 
     uint8_t 
     mask = mask_output[0],        
     data = (output_port[0]^polary_output[0])&mask;
     
     
     if(data != (LATB&mask)){
        
         data |=(LATB&(~mask));
         LATB = data; 
         
     };
     
     mask = mask_output[1]&0x0F;
     data = (output_port[1]^polary_output[1])&mask;
     
    if(data != (LATC&mask)){
        
         data |=(LATC&(~mask));
         LATC = data; 
         
     };
 
 };
 
 
 

/* structure triac_block*/




 
 
 
 
 
 
 
 
 
 
#ifdef	__cplusplus
}
#endif

#endif	/* TRIAC_BLOCK_12_H */

