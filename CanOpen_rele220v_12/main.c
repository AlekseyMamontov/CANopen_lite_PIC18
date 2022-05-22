/**
 Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F25K80
        Driver Version    :  2.00
*/

#include "mcc_generated_files/mcc.h"
#include "block_can_rele.h"


#define NMT_send          0b01
#define PDO_SDO_send 0b10
 uCAN_MSG  msg_tx;
 uCAN_MSG  msg_rx_buf_0;
 uCAN_MSG  msg_rx_buf_1;
 
 uint8_t  status_msg = 0;
 uint8_t  status_msg1 = 0;

 uint32_t temp_d ;
 uint8_t   status_command = 0;
 uint8_t   timer_counter1 = 0;
 uint8_t   timer_end = 0;
 uint8_t   clock_test = 0;
 uint8_t   addr_block_rele;
 
 uint8_t STATUS_rele_block;
 
 
 
void RXB0_message_N (void){status_msg  = 1;}; 
void RXB0_msg(void){status_msg  |= NMT_send;};
void RXB1_msg(void){status_msg  |= PDO_SDO_send;};

  
     
          
 
 void Block_status(){
  if (timer_counter1 <40){
      timer_counter1++;
  }else{
      timer_counter1=0;
      timer_end= 1;};
  };
  
  
  
//  CIA 301  VAR
  
/*  1001_Error_,bit n,
   0 Generic error
   1 Current
   2 Voltage
   3 Temperature
   4 Communication error
   5 Device profile specific
   6 Reserved (always 0)
   7 Manufacturer specific*/
 
 uint8_t     
 N1001_Error_Registr = 0,
 N100D_Life_Time_Factor = 0,
 /* 1029, 0 = Pre-Operational, 1 = No State Change, 2 = Stop
  * sub-index:
    1 - Communication Error
    2 - Output Error
    3 - Input Error  */     
 N1029_Communication_Error_value[4] = {0};

uint16_t 
N100C_Guard_Time = 0,
N1015_Inhibit_Time_EMCY = 0,
N1017_Producer_heartbeat_time = 0;
 
 
 uint32_t   
 N1003_error_field = 0, 
 /* N1005, bit n, 
   30 Generate
        0: Device does not generate SYNC message
        1: Device generates SYNC message
    29 Frame Not supported, always set to 0.
    28. . . 11 29 bit ID Not supported, always set to 0.
    10. . . 0 11 bit ID 11 bit COB-ID.*/
 N1005_cob_id_sync = 0,
 N1006_Comm_cycle_period = 0,
 N1007_Sync_window_len = 0,
 /* N1010, sub-index:
    1 - save all parameters
    2 - save communication parameters
    3 - save application parameters
    4 - save manufacturer defined parameters
     save 65h_76h_61h_73h (evas),
     load 64h_61h_6Fh_6Ch (daol) */        
 N1010_value[4] = {0},
  /* N1011, sub-index:
    1 - restore all default parameters
    2 - restore Index 1000h - 1FFFh
    3 - restore Index 6000h - 9FFFh */   
 N1011_value[3] = {0},             
 N1014_cob_id_EMCY = 0,
  /*N1016, bit n,
    31. . . 24 Reserved 
    23. . . 16 Node ID Heartbeat Producer Node ID
    15. . . 0 Heartbeat time Time in 1ms    */
 N1016_Consumer_heartbeat_time_value={0},
 /*N1018 , sub-index:
   1 - The Vendor ID
   2 - Product code
   3 - Revision number
   4 - Serial number*/                
 N1018_value[4] ={0};
               

 const 
 uint32_t    
 N1000_Device_Type = 30191,
 N1008_Device_name = 0,
 N1009_Hardware_version = 0,
 N100A_Software_version = 0;

 // CIA 401

uint32_t
pdo1_map_output[2] ={0x63000110,0}, //for  PDO 1600 
pdo1_map_input [2] = {0x60000101,0}; //for  PDO 1A00

uint16_t 
gpio_output = 0,
gpio_output_old = 0,
gpio_polary_output = 0,
gpio_mask_output   = 0;
uint8_t   
gpio_input = 0, 
gpio_polary_input = 0,
gpio_mask_input   = 0;


 
// STRUCTURE

struct SDO_comm   N1200; 

struct OD_identity
N1018_Identity_Object ={.sub_index = 4};

struct OD_array_n   

N1010_Store_Parameters ={.sub_index = 3, .value = N1010_value}, 
N1011_Restore_Default_param = {.sub_index = 3, .value = N1011_value}, 
N1029_Error_Behaviour ={.sub_index = 3, .value = N1029_Communication_Error_value};


struct PDO_comm   
N1400={.sub_index = 3},
N1800={.sub_index = 5}; 

struct PDO_mapping   
N1600={.sub_index = 1, .object = pdo1_map_output,},  
N1A00={.sub_index = 1, .object = pdo1_map_input};

 
 struct OD_table Table_can_rele[]={

{0x1000, OD_VAR,UNSIGNED32, &N1000_Device_Type,attr_RO},
{0x1029, OD_ARRAY,UNSIGNED32,&N1029_Communication_Error_value,0},
{0x1200, OD_DEFSTRUCT, SDO_PARAMETER, &N1200,attr_RO},
{0x1400, OD_DEFSTRUCT, PDO_COMM,           &N1400,0},
{0x1600, OD_DEFSTRUCT, PDO_MAPPING,      &N1600,0},
{0x1800, OD_DEFSTRUCT, PDO_COMM,           &N1800,0},
{0x1A00, OD_DEFSTRUCT, PDO_MAPPING,      &N1A00,0},
// bit8 input 
{0x6000, OD_VAR, UNSIGNED8, &gpio_input,0},
// bit8 input polarity
{0x6002, OD_VAR, UNSIGNED8, &gpio_polary_input,0},
// bit8 input filters
{0x6003, OD_VAR, UNSIGNED8, &gpio_mask_input,0},
// bit output 0..12
{0x6220, OD_VAR, _BOOLEAN, NULL,0},
// bit polarity output 0-12
{0x6240, OD_VAR,  _BOOLEAN, NULL,0},
// bit filters output 0-12
{0x6270, OD_VAR,  _BOOLEAN, NULL,0},
// 16bit output 0..12
{0x6300, OD_VAR, UNSIGNED16, &gpio_output,0},
// 16bit output 0..12
{0x6302, OD_VAR, UNSIGNED16, &gpio_polary_output ,0},
// 16bit output 0..12
{0x6308, OD_VAR, UNSIGNED16, &gpio_mask_output,0},
{0},
 };
 
 
void main(void)
{
    // Initialize the device
    SYSTEM_Initialize();
    
    uint8_t  
    Node_ID = (Read_addr_CAN()&0x7F);
    if(Node_ID == 0) Node_ID= 1;
    CanOpen_ECAN_Initialize(Node_ID);
    
    // SDO protocol
    N1200.cob_id_client = SDO_Receive + Node_ID;
    N1200.cob_id_server= SDO_Transmit + Node_ID;
    N1200.node_id = Node_ID;
    // RPDO
    N1400.cob_id = PD1_Receive + Node_ID;
    N1400.Transmission_type = 0xff;
    //TPDO
    N1800.cob_id = PD1_Transmit + Node_ID;
    N1800.Transmission_type = 0xff;
    N1800.event_timer = 0;
    N1800.Inhibit_time = 0;
   
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts
    // Use the following macros to:
    
    ECAN_SetRXB0InterruptHandler(RXB0_message_N);
    ECAN_SetRXB1InterruptHandler(RXB0_message_N);
    TMR1_SetInterruptHandler(Block_status);
    
    // Enable the Global Interrupts
     INTERRUPT_GlobalInterruptEnable(); 
    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();


    
    //  send message Bootup 700 + Node_ID
    
        msg_tx.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
        msg_tx.frame.id         = CanOpen_Heartbeat+ Node_ID;
        msg_tx.frame.dlc       = 1;
        msg_tx.frame.data0  = 0;
     
        CAN_transmit(&msg_tx); 
        
        STATUS_rele_block = PRE_OPERATIONAL;
        
        
        while(1){
       
      
         if(status_msg !=0 ) status_msg1 = CanOpen_receive (&msg_rx_buf_0,&msg_rx_buf_1);
           
          if(status_msg1 != 0 ){
          
          }
        
        
        
        
       switch(STATUS_rele_block){
       
       
           case STOPPED:
       
      break; 
       
       
           case OPERATIONAL: 
       
       break;
       
       
           case PRE_OPERATIONAL:
        
               
       break;
               
      
       
       
     }; 
     
        
        
    

        
   
};       
        
}
/**
 * 
 *    
   uCAN_MSG  msg2;
 *   TMR1_StartTimer(); 
  while (1) {
      
 
        // Add your application code
        
           
        if(status_command == 1){
            
          if ( CAN_receive (&msg2) != 0){
          
                if(msg2.frame.id == (PD1_Receive+Node_ID) ){ 
                    
                    cycle_pr = 0;
                    cycle_pr |= (msg2.frame.data0&0x0f); 
                    cycle_pr =  cycle_pr << 8;
                    cycle_pr |= (msg2.frame.data1);
                    Set_Rele(cycle_pr);
                    status_command =2 ;  
                    
                }else{               
                    
                     msg.frame.data0 = msg2.frame.id&0xFF;
                     msg.frame.data1 = (msg2.frame.id&0xFF00)>>8;
                     msg.frame.data2 = (msg2.frame.id&0xFF0000)>>16;
                     msg.frame.data3 = (msg2.frame.id&0xFF000000)>>24;                   
                     msg.frame.id = 0x700;
                     msg.frame.dlc = 4;
                     CAN_transmit(&msg); 
                     Rele1_Toggle();
                     
                };  
                    
                    
              };  
        };

        if(status_command == 2){
   
            
             msg.frame.id = PD1_Transmit + Node_ID;
             msg.frame.data0 = LATB;
             msg.frame.data1 = LATC&0x0F;
             msg.frame.dlc = 2;
             CAN_transmit( &msg); 
             status_command = 0;
             
       };
        
       if(timer_end ==1){
           timer_end = 0;
             msg.frame.id = 0x700+Node_ID;
             msg.frame.data0 = clock_test;
             msg.frame.data1 = timer_counter1;
             msg.frame.data2 = status_command;         
             msg.frame.dlc = 3;
             CAN_transmit(&msg); 
             Led_info_Toggle() ;
       };
       

    };
 * 
 * 
 * 
 End of File
*/