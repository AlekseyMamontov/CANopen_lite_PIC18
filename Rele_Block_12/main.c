/* CAN rele block*/

#include "mcc_generated_files/mcc.h"
#include "CANOPEN.h"
#include "Triac_block_12.h"





void main(void)
{

    uCAN_MSG        msg;
    uint8_t         status_msg;
    
    Triac_rele.cob_id = Read_addr_CAN()&0x7F;
    Triac_rele.mode = BOOT;
    Triac_rele.current_msg =(CanOpen_msg*)&msg;
  
    rx_pdo1.cob_id = PDO_DISABLED + Triac_rele.cob_id;
    rx_pdo2.cob_id = PDO_DISABLED + Triac_rele.cob_id;
    rx_pdo1.cob_id = PDO_DISABLED + Triac_rele.cob_id;
    rx_pdo1.cob_id = PDO_DISABLED + Triac_rele.cob_id;
    
    tx_pdo1.cob_id = PDO_DISABLED + Triac_rele.cob_id;
    tx_pdo1.cob_id = PDO_DISABLED + Triac_rele.cob_id;
    tx_pdo1.cob_id = PDO_DISABLED + Triac_rele.cob_id;
    tx_pdo1.cob_id = PDO_DISABLED + Triac_rele.cob_id;    
    
    CANOPEN_ECAN_Initialize(Triac_rele.cob_id);
    
    // Enable the Global Interrupts
    
    INTERRUPT_GlobalInterruptEnable();
    
    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();
   
    // -------------- boot init ----------------
    
     msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
     msg.frame.id =     0x700 + Triac_rele.cob_id;
     msg.frame.dlc =    1;
     msg.frame.data0 =  0;   
                
    while(CAN_transmit(&msg) == 0){};
    
    Triac_rele.mode = PRE_OPERATIONAL; 
    
    while (1){
              
       NODE_message_processing(&Triac_rele); 
       GPIO_processing();
       Processing_pdo_objects(&Triac_rele);

    };     
 };           

/**
 End of File
*/