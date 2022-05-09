/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This header file provides APIs for driver for .
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F25K80
        Driver Version    :  2.11
    The generated drivers are tested against the following:
        Compiler          :  XC8 2.31 and above
        MPLAB 	          :  MPLAB X 5.45	
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/**
  Section: Included Files
*/

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set Data_addr aliases
#define Data_addr_TRIS                 TRISAbits.TRISA0
#define Data_addr_LAT                  LATAbits.LATA0
#define Data_addr_PORT                 PORTAbits.RA0
#define Data_addr_ANS                  ANCON0bits.ANSEL0
#define Data_addr_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define Data_addr_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define Data_addr_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define Data_addr_GetValue()           PORTAbits.RA0
#define Data_addr_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define Data_addr_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define Data_addr_SetAnalogMode()      do { ANCON0bits.ANSEL0 = 1; } while(0)
#define Data_addr_SetDigitalMode()     do { ANCON0bits.ANSEL0 = 0; } while(0)

// get/set SCK_165 aliases
#define SCK_165_TRIS                 TRISAbits.TRISA1
#define SCK_165_LAT                  LATAbits.LATA1
#define SCK_165_PORT                 PORTAbits.RA1
#define SCK_165_ANS                  ANCON0bits.ANSEL1
#define SCK_165_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define SCK_165_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define SCK_165_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define SCK_165_GetValue()           PORTAbits.RA1
#define SCK_165_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define SCK_165_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define SCK_165_SetAnalogMode()      do { ANCON0bits.ANSEL1 = 1; } while(0)
#define SCK_165_SetDigitalMode()     do { ANCON0bits.ANSEL1 = 0; } while(0)

// get/set PL_165 aliases
#define PL_165_TRIS                 TRISAbits.TRISA2
#define PL_165_LAT                  LATAbits.LATA2
#define PL_165_PORT                 PORTAbits.RA2
#define PL_165_ANS                  ANCON0bits.ANSEL2
#define PL_165_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define PL_165_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define PL_165_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define PL_165_GetValue()           PORTAbits.RA2
#define PL_165_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define PL_165_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define PL_165_SetAnalogMode()      do { ANCON0bits.ANSEL2 = 1; } while(0)
#define PL_165_SetDigitalMode()     do { ANCON0bits.ANSEL2 = 0; } while(0)

// get/set Led_info aliases
#define Led_info_TRIS                 TRISAbits.TRISA3
#define Led_info_LAT                  LATAbits.LATA3
#define Led_info_PORT                 PORTAbits.RA3
#define Led_info_ANS                  ANCON0bits.ANSEL3
#define Led_info_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define Led_info_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define Led_info_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define Led_info_GetValue()           PORTAbits.RA3
#define Led_info_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define Led_info_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define Led_info_SetAnalogMode()      do { ANCON0bits.ANSEL3 = 1; } while(0)
#define Led_info_SetDigitalMode()     do { ANCON0bits.ANSEL3 = 0; } while(0)

// get/set Ststus_220V aliases
#define Ststus_220V_TRIS                 TRISAbits.TRISA5
#define Ststus_220V_LAT                  LATAbits.LATA5
#define Ststus_220V_PORT                 PORTAbits.RA5
#define Ststus_220V_ANS                  ANCON0bits.ANSEL4
#define Ststus_220V_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define Ststus_220V_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define Ststus_220V_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define Ststus_220V_GetValue()           PORTAbits.RA5
#define Ststus_220V_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define Ststus_220V_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define Ststus_220V_SetAnalogMode()      do { ANCON0bits.ANSEL4 = 1; } while(0)
#define Ststus_220V_SetDigitalMode()     do { ANCON0bits.ANSEL4 = 0; } while(0)

// get/set Rele1 aliases
#define Rele1_TRIS                 TRISBbits.TRISB0
#define Rele1_LAT                  LATBbits.LATB0
#define Rele1_PORT                 PORTBbits.RB0
#define Rele1_WPU                  WPUBbits.WPUB0
#define Rele1_ANS                  ANCON1bits.ANSEL10
#define Rele1_SetHigh()            do { LATBbits.LATB0 = 1; } while(0)
#define Rele1_SetLow()             do { LATBbits.LATB0 = 0; } while(0)
#define Rele1_Toggle()             do { LATBbits.LATB0 = ~LATBbits.LATB0; } while(0)
#define Rele1_GetValue()           PORTBbits.RB0
#define Rele1_SetDigitalInput()    do { TRISBbits.TRISB0 = 1; } while(0)
#define Rele1_SetDigitalOutput()   do { TRISBbits.TRISB0 = 0; } while(0)
#define Rele1_SetPullup()          do { WPUBbits.WPUB0 = 1; } while(0)
#define Rele1_ResetPullup()        do { WPUBbits.WPUB0 = 0; } while(0)
#define Rele1_SetAnalogMode()      do { ANCON1bits.ANSEL10 = 1; } while(0)
#define Rele1_SetDigitalMode()     do { ANCON1bits.ANSEL10 = 0; } while(0)

// get/set Rele2 aliases
#define Rele2_TRIS                 TRISBbits.TRISB1
#define Rele2_LAT                  LATBbits.LATB1
#define Rele2_PORT                 PORTBbits.RB1
#define Rele2_WPU                  WPUBbits.WPUB1
#define Rele2_ANS                  ANCON1bits.ANSEL8
#define Rele2_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define Rele2_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define Rele2_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define Rele2_GetValue()           PORTBbits.RB1
#define Rele2_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define Rele2_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define Rele2_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define Rele2_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define Rele2_SetAnalogMode()      do { ANCON1bits.ANSEL8 = 1; } while(0)
#define Rele2_SetDigitalMode()     do { ANCON1bits.ANSEL8 = 0; } while(0)

// get/set Rele3 aliases
#define Rele3_TRIS                 TRISBbits.TRISB2
#define Rele3_LAT                  LATBbits.LATB2
#define Rele3_PORT                 PORTBbits.RB2
#define Rele3_WPU                  WPUBbits.WPUB2
#define Rele3_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define Rele3_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define Rele3_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define Rele3_GetValue()           PORTBbits.RB2
#define Rele3_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define Rele3_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define Rele3_SetPullup()          do { WPUBbits.WPUB2 = 1; } while(0)
#define Rele3_ResetPullup()        do { WPUBbits.WPUB2 = 0; } while(0)

// get/set Rele4 aliases
#define Rele4_TRIS                 TRISBbits.TRISB3
#define Rele4_LAT                  LATBbits.LATB3
#define Rele4_PORT                 PORTBbits.RB3
#define Rele4_WPU                  WPUBbits.WPUB3
#define Rele4_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define Rele4_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define Rele4_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define Rele4_GetValue()           PORTBbits.RB3
#define Rele4_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define Rele4_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define Rele4_SetPullup()          do { WPUBbits.WPUB3 = 1; } while(0)
#define Rele4_ResetPullup()        do { WPUBbits.WPUB3 = 0; } while(0)

// get/set Rele5 aliases
#define Rele5_TRIS                 TRISBbits.TRISB4
#define Rele5_LAT                  LATBbits.LATB4
#define Rele5_PORT                 PORTBbits.RB4
#define Rele5_WPU                  WPUBbits.WPUB4
#define Rele5_ANS                  ANCON1bits.ANSEL9
#define Rele5_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define Rele5_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define Rele5_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define Rele5_GetValue()           PORTBbits.RB4
#define Rele5_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define Rele5_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define Rele5_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define Rele5_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define Rele5_SetAnalogMode()      do { ANCON1bits.ANSEL9 = 1; } while(0)
#define Rele5_SetDigitalMode()     do { ANCON1bits.ANSEL9 = 0; } while(0)

// get/set Rele6 aliases
#define Rele6_TRIS                 TRISBbits.TRISB5
#define Rele6_LAT                  LATBbits.LATB5
#define Rele6_PORT                 PORTBbits.RB5
#define Rele6_WPU                  WPUBbits.WPUB5
#define Rele6_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define Rele6_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define Rele6_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define Rele6_GetValue()           PORTBbits.RB5
#define Rele6_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define Rele6_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define Rele6_SetPullup()          do { WPUBbits.WPUB5 = 1; } while(0)
#define Rele6_ResetPullup()        do { WPUBbits.WPUB5 = 0; } while(0)

// get/set Rele7 aliases
#define Rele7_TRIS                 TRISBbits.TRISB6
#define Rele7_LAT                  LATBbits.LATB6
#define Rele7_PORT                 PORTBbits.RB6
#define Rele7_WPU                  WPUBbits.WPUB6
#define Rele7_SetHigh()            do { LATBbits.LATB6 = 1; } while(0)
#define Rele7_SetLow()             do { LATBbits.LATB6 = 0; } while(0)
#define Rele7_Toggle()             do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define Rele7_GetValue()           PORTBbits.RB6
#define Rele7_SetDigitalInput()    do { TRISBbits.TRISB6 = 1; } while(0)
#define Rele7_SetDigitalOutput()   do { TRISBbits.TRISB6 = 0; } while(0)
#define Rele7_SetPullup()          do { WPUBbits.WPUB6 = 1; } while(0)
#define Rele7_ResetPullup()        do { WPUBbits.WPUB6 = 0; } while(0)

// get/set Rele8 aliases
#define Rele8_TRIS                 TRISBbits.TRISB7
#define Rele8_LAT                  LATBbits.LATB7
#define Rele8_PORT                 PORTBbits.RB7
#define Rele8_WPU                  WPUBbits.WPUB7
#define Rele8_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define Rele8_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define Rele8_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define Rele8_GetValue()           PORTBbits.RB7
#define Rele8_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define Rele8_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define Rele8_SetPullup()          do { WPUBbits.WPUB7 = 1; } while(0)
#define Rele8_ResetPullup()        do { WPUBbits.WPUB7 = 0; } while(0)

// get/set Rele9 aliases
#define Rele9_TRIS                 TRISCbits.TRISC0
#define Rele9_LAT                  LATCbits.LATC0
#define Rele9_PORT                 PORTCbits.RC0
#define Rele9_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define Rele9_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define Rele9_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define Rele9_GetValue()           PORTCbits.RC0
#define Rele9_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define Rele9_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)

// get/set Rele10 aliases
#define Rele10_TRIS                 TRISCbits.TRISC1
#define Rele10_LAT                  LATCbits.LATC1
#define Rele10_PORT                 PORTCbits.RC1
#define Rele10_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define Rele10_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define Rele10_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define Rele10_GetValue()           PORTCbits.RC1
#define Rele10_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define Rele10_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)

// get/set Rele11 aliases
#define Rele11_TRIS                 TRISCbits.TRISC2
#define Rele11_LAT                  LATCbits.LATC2
#define Rele11_PORT                 PORTCbits.RC2
#define Rele11_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define Rele11_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define Rele11_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define Rele11_GetValue()           PORTCbits.RC2
#define Rele11_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define Rele11_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)

// get/set Rele12 aliases
#define Rele12_TRIS                 TRISCbits.TRISC3
#define Rele12_LAT                  LATCbits.LATC3
#define Rele12_PORT                 PORTCbits.RC3
#define Rele12_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define Rele12_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define Rele12_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define Rele12_GetValue()           PORTCbits.RC3
#define Rele12_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define Rele12_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)

// get/set RC6 procedures
#define RC6_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define RC6_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define RC6_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define RC6_GetValue()              PORTCbits.RC6
#define RC6_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define RC6_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)

// get/set RC7 procedures
#define RC7_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define RC7_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define RC7_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define RC7_GetValue()              PORTCbits.RC7
#define RC7_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define RC7_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/