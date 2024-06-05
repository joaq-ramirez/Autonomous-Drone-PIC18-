
/****** ASEN 4/5067 Lab 6 ******************************************************
 * Author: Joaquin Ramirez
 * Date  : 12/8
 *
 * Updated for XC8
 * 
 * Description
 * 
 * This code autonomously controls the drone using a distance sensor input and 
 * PWM outputs to the drone's ESCs. 
 * 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR 
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
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF, ECCPMX = PORTE

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
char sbus_frame[25]; 
char send_bit = 1; 
char timeHigh; 
char timeLow; 
unsigned char motorData[23] = {0};
double tot_dist; 
unsigned int low_ccp;
unsigned int high_ccp;
double old_dist; 
double Kp = 0.065;  // Proportional gain
double Kd = 0.0001; // Derivative gain
double targetDistance = 30.0; // Target distance from the object in cm
double prevError = 0.0; // Initial error
double error = 0;
double dt; 
double mass = 0.8;         // kg
double t_force = 1.5;        // N? 
unsigned int prev_pwm;
char f_mode = 'T'; 
double controlPwm; 
/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Initial(void);         // Function to initialize hardware and interrupts
void TMR0handler(void);     // Interrupt handler for TMR0, typo in main
void HR04Handler(void); 
void addToMotor(unsigned char data[22], int motorIndex, int value);
void CCP1handler(void); 
void CCP2handler(void);
void ESCinit(void); 
double computeControlForce(double error, double prevError); 
/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
    Initial();                 // Initialize everything
    ESCinit();                 // Initialize ESC
//    T0CONbits.TMR0ON = 1;      // Turn on timer for PD control
    T4CONbits.TMR4ON = 1; 
    LATDbits.LATD6 = 1;     
    f_mode = 'T'; 
      while(1) {
          
        send_bit = 0;       // Signify not to send another signal until echo
        LATDbits.LATD7 = 1; 
        __delay_us(10); 
        LATDbits.LATD7 = 0; 

        __delay_ms(100);
        DisplayV(disp_dist);      // display distance 
          
          
        //PD controller
        error = targetDistance - tot_dist;         
//        controlPwm = computeControlForce(error, prevError);
        prevError = error; 
        
        //Flight modes
        switch (f_mode){
            case 'T':
                LATGbits.LATG0 = 1;
                controlPwm = (40/tot_dist)*75 + 6000;
                if(tot_dist > 30){  // initiate hover
                    f_mode = 'H'; 
                    LATHbits.LATH1 = 0;
                    LATHbits.LATH2 = 1;     // Turn on LED to indicate hover  
                    controlPwm = 6000;      // assuming 2:1 lift:weight
                }
                break; 
            case 'L':
                LATGbits.LATG2 = 1;
                LATGbits.LATG1 = 0;
                targetDistance = 3;         // 3 cm for landing gear offset
                controlPwm = (tot_dist/40)*1400 + 5600;
                break; 
            case 'H':
                controlPwm = 6000;
                __delay_ms(2000);       //Temp delay for hover - should implement timer
                LATGbits.LATG0 = 0;
                LATGbits.LATG1 = 1;
                f_mode = 'L'; 
                break; 
        }
        
        //Software Killswitch 
        if(tot_dist > 60){
            controlPwm = 4500;      // default to low motor power
        } 
        
//        double controlPwm = (tot_dist/40)*2000 + 6000;
        
        // Smooth motor control 
        if(controlPwm > 9000){
            controlPwm = prev_pwm; 
        }
        
        low_ccp = 1000;
        high_ccp = controlPwm; 
        prev_pwm = controlPwm;          
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
    TRISCbits.TRISC2 = 0;       // Set as output for ECCP1
    TRISCbits.TRISC3 = 0;       // Set as output for ECCP2
    TRISGbits.TRISG0 = 0; 
    TRISGbits.TRISG1 = 0; 
    TRISGbits.TRISG2 = 0; 
    LATG = 0; 
    // SBUS Serial Transmit Config -----UART.init
    // Set Baudrate and other config parameters. 
    SPBRGH1 = 0x00;                 // SPBRG set to 9 for 100,000 baud rate - ignore high 
    SPBRG1 = 0x09;                  // 
    TXSTA1 = 0b00100100;
    RCSTA1 = 0b10010000;            // Configure RCSTA1<7> as input for EUSART1
    BAUDCON1 = 0b01000000; 

    // Initialize the LCD and print to it
    InitLCD();
    DisplayC(LCDRow1);

    // Initializing TMR0
    T0CON = 0b00001000;             // 16-bit, Fosc / 4, no pre/post scale timer
//    TMR0L = 0x40; 
//    TMR0H = 0xA2;
    TMR0L = 0x00;
    TMR0H = 0xA2; 
    
    //Initialize timers for CCPs
    T1CON = 0b00000010;
    TMR1L = 0; 
    TMR1H = 0;
    T3CON = 0b00000010;
    TMR1L = 0; 
    TMR1H = 0;
    T4CON = 0b00000010;
    TMR1L = 0; 
    TMR1H = 0;
    
    //Initialize CCP1 register- 4-7
    CCP1CON = 0b00001010;           // Generate software interrupt only
    CCPTMRS0 = 0b00000000;          // <2-0> 000 ECCP1 is based on TMR1/TMR2
    
    //Initialize CCP2 register- 4-7
    CCP2CON = 0b00001010;           // Generate software interrupt only
    CCPTMRS0 = 0b00001000;          // <2-0> 000 ECCP1 is based on TMR3/TMR4
    
    
    // Configuring Interrupts
    RCONbits.IPEN = 1;              // Enable priority levels
    
//    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
//    INTCON2bits.TMR0IP = 1;         // Assign high priority to TMR0 interrupt
    
    INTCON2bits.RBIP = 0;           // Set low pri to RB interrupt.
    INTCONbits.RBIE = 1;            // Enable PORTB global interrupts
    
    //CCP Interrupts
    IPR1bits.TMR1IP = 0;            //Low priority to timer1
    IPR3bits.CCP1IP = 1;            //High priority to ECCP1
    PIE3bits.CCP1IE = 1;            // Enable interrupts for ECCP1; 
    
    IPR2bits.TMR3IP = 0;            //Low priority to timer3
    IPR3bits.CCP2IP = 1;            //High priority to ECCP2
    PIE3bits.CCP2IE = 1;            // Enable interrupts for ECCP1;
    
//    PIE1bits.TMR1IE = 1;            // Enables the TMR1 overflow I
    
    //General Interrupts
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
//    T0CONbits.TMR0ON = 1;           // Turning on TMR0 - enable for UART
}

/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
        if (PIR3bits.CCP1IF){
            CCP1handler();
            PIR3bits.CCP1IF = 0;        // clear flag
            continue;
        }else if(PIR3bits.CCP2IF){
            CCP2handler();
            PIR3bits.CCP2IF = 0;        // clear flag
            continue;
        }
//        if(INTCONbits.TMR0IF) {
//            TMR0handler(); 
//            continue; 
//        }
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
 * Function to calculate PWM using a PD controller
 *
 * 
 ******************************************************************************/
double computeControlForce(double error, double prevError) {
    //Stop Timer4
    T4CONbits.TMR4ON = 0; 

    //Update error and dt
//    prevError = error;      //update previous error ASAP
    dt = TMR4 * 0.000000250;
    
    // Derivative term
    double D = Kd * ((error - prevError) / dt);
//    double D = 0; 
    
    
    
    //Proportional Term
    double P = Kp * error;
    
    // PD Controller output
    double output = ((P + D)/t_force)*2500 + 5500;    // PWM high to command
    
    // Edge case for "negative" thrust values
    if (output< 5500){
        output = 5500; 
    }
    
    //Restart Timer 4
    TMR4 = 0; 
    T4CONbits.TMR4ON = 1;
    
    return output;
}

/******************************************************************************
 * Function to add value to motor control
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/

void addToMotor(unsigned char data[23], int motorIndex, int value) {

    int byteIndex = (motorIndex - 1) / 2;
    int bitOffset = (motorIndex - 1) % 2 * 11;

    // Motor values are stored as little-endian 11-bit chunks
    unsigned short* motorValue = (unsigned short*)&data[byteIndex];
    *motorValue += (value << bitOffset);

}

/******************************************************************************
 * HR04Handler interrupt service routine.
 *
 * Displays and manages distance sensor data
 ******************************************************************************/
void HR04Handler() {
    if (PORTBbits.RB7){           // Change from zero to one
        T0CONbits.TMR0ON = 1;           // Turn on the timer
    }else{                              // Compare and calculate distance
        T0CONbits.TMR0ON = 0;           // Turn off the timer
        INTCONbits.RBIE = 0;            // Avoid race condition
        tot_dist = (((TMR0H << 8) + TMR0L)*.25)/148;           //tot_time is an int
        send_bit = 1;                   // Initiate another trigger
        if(tot_dist < 0 || tot_dist > 100 ){
            tot_dist = old_dist; 
        }
        sprintf(disp_dist, "%cD=%.1f %c",0x80, tot_dist,0x00); 
        
        old_dist = tot_dist;            // set old value to current for next loop
        INTCONbits.RBIE = 1;            // Reenable interrupts on PORTB
        TMR0H = 0; 
        TMR0L = 0; 
    }        
    INTCONbits.RBIF = 0;        // Reset flag

}

/******************************************************************************
 * CCP1handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/
void CCP1handler() {
//    if(PIR1bits.TMR1IF){    //check if TMR1 IF set
//        CCPR1 += 7600;  
//    }
    
    if(PORTCbits.RC2){
        CCPR1 += low_ccp;
        LATCbits.LATC2 = 0;
    }else{
        CCPR1 += high_ccp;
        LATCbits.LATC2 = 1; 
    }
           //toggle pin for PWM
    //5660 is min
    
}

/******************************************************************************
 * CCP2handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/
void CCP2handler() {
//    if(PIR1bits.TMR1IF){    //check if TMR1 IF set
//        CCPR1 += 7600;  
//    }
    
    if(PORTCbits.RC3){
        CCPR2 += low_ccp;
        LATCbits.LATC3 = 0;
    }else{
        CCPR2 += high_ccp;
        LATCbits.LATC3 = 1; 
    }
           //toggle pin for PWM
    //5660 is min
    
}

/******************************************************************************
 * ESCinit
 *
 * Initializes and calibrates ESC
 ******************************************************************************/
void ESCinit() {
    low_ccp = 1000; 
    high_ccp = 8000; 
    T1CONbits.TMR1ON = 1;           //Turn on timer 1
    T3CONbits.TMR3ON = 1;           //Turn on timer 1
    LATGbits.LATG1 = 1; 
    
    __delay_ms(6000);               // Delay 1 second
    LATGbits.LATG1 = 0;
    LATGbits.LATG0 = 1; 
    low_ccp = 1000; 
    high_ccp = 4000;
    LATGbits.LATG0 = 0; 
    __delay_ms(5000);
    
}
/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter 
 ******************************************************************************/
void TMR0handler() {
    // Initiate transmit
    
    //          addToMotor(motorData, 3, 200);
//    sprintf(sbus_frame, "%c%c%f%c", 0x00,0x00,motorData,0x0F);
//
//    for (j = 0; j< 23; j++){
//        TXREG1 = sbus_frame[j]; 
//        while(PIR1bits.TX1IF == 0){}
//    }
    // min 1100 us to 1900 us 
    
//    if(PORTFbits.RF7){
//       LATFbits.LATF5 = 0; 
//    }else{
//       LATFbits.LATF5 = 1;
//    }
    
    LATFbits.LATF5 = ~LATFbits.LATF5; 
    
    
    TMR0L = 0x40;
    TMR0H = 0xA2; 
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}
