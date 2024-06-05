
/****** ASEN 4/5067 Lab 6 ******************************************************
 * Author: Joaquin Ramirez
 * Date  : 12/8
 *
 * Updated for XC8
 * 
 * Description
 * 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

#include <xc.h>
#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
const char LCDRow1[] = {0x80,'I','N','I','T',0x00};  //const puts into prog memory
char LCD_dist[] = {0x80,0,0,0,0,0x00}; 
unsigned int Alive_count = 0;
double t_conv = 0.250; 
int j; 
char disp_dist[20]; 
int sbus_bitmap[23]; 
int throttle; 
unsigned char sbus_frame[25]; 
double deriv_dist; 
char send_bit = 1; 
char timeHigh; 
char timeLow; 
unsigned char motorData[22] = {0};
double tot_dist; 

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main
void HR04Handler(void); 
void addToMotor(unsigned char data[22], int motorIndex, int value);

/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
     Initial();                 // Initialize everything
     LATDbits.LATD6 = 1;        
     
      while(1) {
          
            __delay_ms(3); 
//          if (send_bit){
            send_bit = 0;       // Signify not to send another signal until echo
            LATDbits.LATD7 = 1; 
            __delay_us(10); 
            LATDbits.LATD7 = 0; 
//          }
          
          
//          __delay_ms(100);
          DisplayV(disp_dist);      // display distance 
          
//        sprintf(sbus_frame, "%c%c%f%c", 0x00,0x00,motorData,0x0F);  
        
          deriv_dist = 0.3*tot_dist; 
//          __delay_ms(6);    /    // 6 ms in between transmissions
        // Put mainline code here
     }
}

/******************************************************************************
 * Initial()
 *
 * This subroutine performs all initializations of variables and registers.
 * 
 ******************************************************************************/
void Initial() {
    // Configure the IO ports
    TRISAbits.TRISA1 = 0;       // 
    TRISD  = 0b00001111;
    LATDbits.LATD6 = 1; 
    
    // Configure the LCD pins for output. Defined in LCDRoutinesEasyPic.h
    LCD_RS_TRIS   = 0;              // Register Select Control line
    LCD_E_TRIS    = 0;              // Enable control line 
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD data is only on the upper nibble RB0:3
                                    // Redundant to line above RB 4:5 for control
                                    // RB 6:7 set as inputs for other use, not used by LCD
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero
    
    //Initialize for distance sensor
    TRISB = 0b10000000;         // Only 7 as echo
    TRISDbits.TRISD7 = 0;       // Trig
    LATBbits.LATB6 = 0;
    LATDbits.LATD7 = 0;         // Make sure trigger is 0 
    TRISFbits.TRISF5 = 0;         // Test LED
    
    // SBUS Serial Transmit Config -----UART.init
    // Set Baudrate and other config parameters. 
//    SPBRGH1 = 0x00;                 // SPBRG set to 9 for 100,000 baud rate - ignore high 
    SPBRG1 = 0x09;                  // 
    TXSTA1 = 0b00100101;
    RCSTA1 = 0b10010000;            // Configure RCSTA1<7> as input for EUSART1
    BAUDCON1 = 0b01000000; 

    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);

    // Initializing TMR0
    T0CON = 0b00001000;             // 16-bit, Fosc / 4, no pre/post scale timer
    TMR0L = 0x40; 
    TMR0H = 0xA2;
    
    
    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    INTCON2bits.TMR0IP = 1;         // Assign high priority to TMR0 interrupt
    
//    INTCON2bits.RBIP = 0;           // Set low pri to RB interrupt.
//    INTCONbits.RBIE = 1;            // Enable PORTB global interrupts
    
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts

    T0CONbits.TMR0ON = 1;           // Turning on TMR0
    sprintf(sbus_frame, "%c%c%c%c", 0x0F,motorData,0x00,0x00);
    sbus_frame[23] = 0x00; 
    sbus_frame[24] = 0x00; 
    sbus_frame[0] = 0x0F;
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
        if(INTCONbits.TMR0IF) {
            TMR0handler(); 
            continue; 
        }
        // Save temp copies of WREG, STATUS and BSR if needed.
        break;      // Supports RETFIE automatically
    }
}	// Supports retfie FAST automatically

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until TMR0IF is clear.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) 
{
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
        if(INTCONbits.RBIF) {
            HR04Handler(); 
            continue; 
        }
        // Save temp copies of WREG, STATUS and BSR if needed.
        break;      // Supports RETFIE automatically
    }
}

/******************************************************************************
 * Function to add value to motor control
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/

void addToMotor(unsigned char data[23], int motorIndex, int value) {
    // Calculate the byte index and bit offset for the specified motor
    int byteIndex = (motorIndex - 1) / 2;
    int bitOffset = (motorIndex - 1) % 2 * 11;

    // Assuming motor values are stored as little-endian 11-bit chunks
    unsigned short* motorValue = (unsigned short*)&data[byteIndex];
    *motorValue += (value << bitOffset);

    // If the addition causes overflow, you may need to handle it accordingly
    // For simplicity, this example does not handle overflow.
}

/******************************************************************************
 * HR04Handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/
void HR04Handler() {
    if (PORTBbits.RB7){           // Change from zero to one
        T0CONbits.TMR0ON = 1;           // Turn on the timer
    }else{                              // Compare and calculate distance
        T0CONbits.TMR0ON = 0;           // Turn off the timer
        INTCONbits.RBIE = 0;            // Avoid race condition
        tot_dist = (((TMR0H << 8) + TMR0L)*.25)/148;           //tot_time is an int
        send_bit = 1;                   // Initiate another trigger
        INTCONbits.RBIE = 1;            // Reenable interrupts on PORTB
        sprintf(disp_dist, "%cD=%.1f %c",0x80, tot_dist,0x00); 
        TMR0H = 0; 
        TMR0L = 0; 
    }        
    INTCONbits.RBIF = 0;        // Reset flag

}

/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/
void TMR0handler() {
    // Initiate transmit
    
    //          addToMotor(motorData, 3, 200);
    for (j = 0; j< 25; j++){
        while(PIR1bits.TX1IF == 0){}
        TXREG1 = sbus_frame[j]; 
    }
    // min 1100 us to 1900 us 
    
    
    LATFbits.LATF5 = ~LATFbits.LATF5; 
    
    TMR0L = 0x40;
    TMR0H = 0xA2; 
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}
