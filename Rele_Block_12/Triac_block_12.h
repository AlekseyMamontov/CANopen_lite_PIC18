/* 
 * File:   Triac_block_12.h
 * Author: Oleksii_Mamontov
 * triac unit for 12 outputs 220v
 */

#ifndef TRIAC_BLOCK_12_H
#define	TRIAC_BLOCK_12_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    
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
       
    RXF2EIDH = 0x00; RXF2EIDL = 0x00;
    RXF2SIDH = 0x40|cob_id_h; RXF2SIDL = cob_id_l; // 0x200+cob_id PDO1
    
    RXF3EIDH = 0x00; RXF3EIDL = 0x00;
    RXF3SIDH = 0x60|cob_id_h; RXF3SIDL = cob_id_l; // 0x300+cob_id PDO2
    
    RXF4EIDH = 0x00; RXF4EIDL = 0x00;
    RXF4SIDH = 0x80|cob_id_h; RXF4SIDL = cob_id_l; // 0x400+cob_id PDO3
    
    RXF5EIDH = 0x00; RXF5EIDL = 0x00;
    RXF5SIDH = 0xA0|cob_id_h; RXF5SIDL = cob_id_l; // 0x500+cob_id PDO4
    
    RXF6EIDH = 0x00; RXF6EIDL = 0x00;
    RXF6SIDH = 0xC0|cob_id_h; RXF6SIDL = cob_id_l; // 0x600+cob_id rxSDO
    
    RXF7EIDH = 0x00;RXF7EIDL = 0x00;
    RXF7SIDH = 0xE0|cob_id_h; RXF7SIDL = cob_id_l;  // 0x700 NMT error
    
    RXF8EIDH =  0x00; RXF8EIDL =  0x00; RXF8SIDH = 0x00; RXF8SIDL =  0x00;
    RXF9EIDH =  0x00; RXF9EIDL =  0x00; RXF9SIDH = 0x00; RXF9SIDL =  0x00;
    RXF10EIDH = 0x00; RXF10EIDL = 0x00;RXF10SIDH = 0x00; RXF10SIDL = 0x00;
    RXF11EIDH = 0x00; RXF11EIDL = 0x00;RXF11SIDH = 0x00; RXF11SIDL = 0x00;
    RXF12EIDH = 0x00; RXF12EIDL = 0x00;RXF12SIDH = 0x00; RXF12SIDL = 0x00; 
    RXF13EIDH = 0x00; RXF13EIDL = 0x00;RXF13SIDH = 0x00; RXF13SIDL = 0x00;  
    RXF14EIDH = 0x00; RXF14EIDL = 0x00;RXF14SIDH = 0x00; RXF14SIDL = 0x00;
    
    
    RXF15EIDH = 0xFF;RXF15EIDL = 0xFF;
    RXF15SIDH = 0xFF;RXF15SIDL = 0xE3; //mask

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
 

 void GPIO_processing(){};
 
 
 
///////////////////////////////////////////////////////////////////



struct OD_object map[]={
};
 
 
struct PDO_object 

rx_pdo1,tx_pdo1,
rx_pdo2,tx_pdo2,
rx_pdo3,tx_pdo3,
rx_pdo4,tx_pdo4;

struct SDO_object 

sdo_rx_tx;

 /*
struct xCanOpen{

uint8_t              cob_id; 
uint8_t                mode; 

CanOpen_msg*    current_msg;

uint8_t  (*receiving_message)(CanOpen_msg *msg);
uint8_t  (*sending_message)(CanOpen_msg *msg);

struct OD_object*   map; 
struct PDO_object*  pdo[8];
struct SDO_object*  sdo[2];

uint8_t Sync_object [MAX_SYNC_OBJECT];

void  (*func_call[n_FUNC_COMMAND])(uint8_t ,void*);

};*/
struct xCanOpen Triac_rele = {
.pdo = {
        &rx_pdo1,// 180 + cob_id
        &tx_pdo1,// 200 + cob_id
        &rx_pdo2,
        &tx_pdo2,
        &rx_pdo3,
        &tx_pdo3,
        &rx_pdo4,
        &tx_pdo4,
       },
       
.sdo = {&sdo_rx_tx},
};    
 
 
 
 
 
 
 
 
 
 
 
 
#ifdef	__cplusplus
}
#endif

#endif	/* TRIAC_BLOCK_12_H */

