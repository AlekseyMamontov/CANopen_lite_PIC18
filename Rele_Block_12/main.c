/* CAN rele block*/

#include "mcc_generated_files/mcc.h"
#include "CanOpen_lite.h"
#include "Triac_block_12.h"





void main(void)
{

    uCAN_MSG        *msg = (uCAN_MSG*)Triac_rele.current_msg;
    uint8_t         status_msg;
    
    *Triac_rele.id = Read_addr_CAN()&0x7F;
    *Triac_rele.mode = BOOT;
    
    Triac_rele_ram.rxpdo1_cob_id = *Triac_rele.id + rxPDO1;
    Triac_rele_ram.rxpdo1_Transmission = 0xFF;
    Triac_rele_ram.rxpdo1_event_timer = 1;
    Triac_rele_ram.rxpdo1_inhibit_time = 0;
    
    
    
    Triac_rele_ram.txpdo1_cob_id = *Triac_rele.id + txPDO1;
    
    
    
    
    
    
  
    //pdo_objects_add_id(&Triac_rele); //node.id + pdo[].cobid
    
    
    CANOPEN_ECAN_Initialize(*Triac_rele.id);
    
    // Enable the Global Interrupts
    
    INTERRUPT_GlobalInterruptEnable();
    
    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();
   
    // -------------- boot init ----------------
    
     msg->frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
     msg->frame.id =     0x700 + *(Triac_rele.id);
     msg->frame.dlc =    1;
     msg->frame.data0 =  0;   
                
    while(CAN_transmit(msg) == 0){};
    
    *Triac_rele.mode = PRE_OPERATIONAL;
    
    
    while (1){
              
       NODE_message_processing(&Triac_rele);      
       NODE_processing_pdo_objects(&Triac_rele);
       GPIO_processing();
       
    };     
 };           

 
 
/**
 End of File
*/