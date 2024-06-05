;;;;;;; Lab 5 5067 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Author:	    Joaquin Ramirez
; Date:		    11/1/2023
; Target:	    PIC18F87K22
; 	    
; REFERENCES:	    Ruben Hinojosa Torres, Lara Lufkin
;		Using as reference also: main.s by Dan1138
;		lab3_orig.asm by Scott Palo, Doug Weibel, Gabe LoDolce and Trudy Schwartz
; Date (Original):  2021-06-5
; 	    
; Compiler: pic-as(v2.32)
; IDE:      MPLABX v5.50
			      
			      
; !!!!!!!!!!!!!!!IMPORTANT!!!!!!!! 
; Compiler Notes: 
; Add this line to the Compiler flags i.e
;   Right click on project name -> Properties -> pic-as Global Options -> 
;   Additional options: 
;    -Wl,-presetVec=0h,-pHiPriISR_Vec=0008h,-pLoPriISR_Vec=0018h
			      
; Description: 
; On power up execute the following sequence:
; 	RD5 ON for ~1 second then OFF
; 	RD6 ON for ~1 second then OFF
; 	RD7 ON for ~1 second then OFF
; LOOP on the following forever:
; 	Blink "Alive" LED (RD4) ON for ~1sec then OFF for ~1sec
; 	Read input from RPG (at least every 2ms) connected to pins 
;		RD0 and RD1 and mirror the output onto pins RJ2 and RJ3
; 	ASEN5067 ONLY: Read input from baseboard RD3 button and toggle the value 
;		of RD2 such that the switch being pressed and RELEASED causes 
;		RD2 to change state from ON to OFF or OFF to ON
;	NOTE: ~1 second means +/- 100msec
			      
;;;;;;;;;;;;;;;;;;;;;;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Mainline
; Loop
; Initial 	- 	Initialize ports and perform LED sequence
; WaitXXXms	- 	Subroutine to wait XXXms
; Wait1sec 	- 	Subroutine to wait 1 sec 
; Check_SW 	- 	Subroutine to check the status of RD3 button and change RD2 (ASEN5067 ONLY)
; Check_RPG	- 	Read the values of the RPG from RD0 and RD1 and display on RJ2 and RJ3	
			      
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Hardware notes ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;	RPG-A port/pin is RJ2
;	RPG-B port/pin is RJ3
// <editor-fold defaultstate="collapsed" desc="Pin Mapping">
/*
    Pin | Pin Name/Register Name
     1  | RH2/AN21/A18
     2  | RH3/AN20/A19
     3  | RE1/P2C/WR/AD9
     4  | RE0/P2D/RD/AD8
     5  | RG0/ECCP3/P3A
     6  | RG1/TX2/CK2/AN19/C3OUT
     7  | RG2/RX2/DT2/AN18/C3INA
     8  | RG3/CCP4/AN17/P3D/C3INB
     9  | MCLR/RG5
     10 | RG4/RTCC/T7CKI(Note:2)/T5G/CCP5/AN16/P1D/C3INC
     11 | VSS
     12 | VDDCORE/VCAP
     13 | RF7/AN5/SS1
     14 | RF6/AN11/C1INA
     15 | RF5/AN10/C1INB
     16 | RF4/AN9/C2INA
     17 | RF3/AN8/C2INB/CTMUI
     18 | RF2/AN7/C1OUT
     19 | RH7/CCP6(Note:3)/P1B/AN15
     20 | RH6/CCP7(Note:3)/P1C/AN14/C1INC
     21 | RH5/CCP8(Note:3)/P3B/AN13/C2IND
     22 | RH4/CCP9(Note:2,3)/P3C/AN12/C2INC
     23 | RF1/AN6/C2OUT/CTDIN
     24 | ENVREG
     25 | AVDD
     26 | AVSS
     27 | RA3/AN3/VREF+
     28 | RA2/AN2/VREF-
     29 | RA1/AN1
     30 | RA0/AN0/ULPWU
     31 | VSS
     32 | VDD
     33 | RA5/AN4/T1CKI/T3G/HLVDIN
     34 | RA4/T0CKI
     35 | RC1/SOSC/ECCP2/P2A
     36 | RC0/SOSCO/SCKLI
     37 | RC6/TX1/CK1
     38 | RC7/RX1/DT1
     39 | RJ4/BA0
     40 | RJ5/CE
     41 | RJ6/LB
     42 | RJ7/UB
     43 | RC2/ECCP1/P1A
     44 | RC3/SCK1/SCL1
     45 | RC4/SDI1/SDA1
     46 | RC5/SDO1
     47 | RB7/KBI3/PGD
     48 | VDD
     49 | OSC1/CLKI/RA7
     50 | OSC2/CLKO/RA6
     51 | VSS
     52 | RB6/KBI2/PGC
     53 | RB5/KBI1/T3CKI/T1G
     54 | RB4/KBI0
     55 | RB3/INT3/CTED2/ECCP2(Note:1)/P2A
     56 | RB2/INT2/CTED1
     57 | RB1/INT1
     58 | RB0/INT0/FLT0
     59 | RJ3/WRH
     60 | RJ2/WRL
     61 | RJ1/OE
     62 | RJ0/ALE
     63 | RD7/SS2/PSP7/AD7
     64 | RD6/SCK2/SCL2/PSP6/AD6
     65 | RD5/SDI2/SDA2/PSP5/AD5
     66 | RD4/SDO2/PSP4/AD4
     67 | RD3/PSP3/AD3
     68 | RD2/PSP2/AD2
     69 | RD1/T5CKI/T7G/PSP1/AD1
     70 | VSS
     71 | VDD
     72 | RD0/PSP0/CTPLS/AD0
     73 | RE7/ECCP2/P2A/AD15
     74 | RE6/P1B/CCP6(Note:3)/AD14
     75 | RE5/P1C/CCP7(Note:3)/AD13
     76 | RE4/P3B/CCP8(Note:3)/AD12
     77 | RE3/P3C/CCP9(Note:2,3)/REF0/AD11
     78 | RE2/P2B/CCP10(Note:2)/CS/AD10
     79 | RH0/AN23/A16
     80 | RH1/AN22/A17

Note (1) The ECCP2 pin placement depends on the CCP2MX Configuration bit 
	setting and whether the device is in Microcontroller or Extended 
	Microcontroller mode.
     (2) Not available on the PIC18F65K22 and PIC18F85K22 devices.
     (3) The CC6, CCP7, CCP8 and CCP9 pin placement depends on the 
	setting of the ECCPMX Configuration bit (CONFIG3H<1>).
*/
// </editor-fold>

;;;;;;;;;;;;;;;;;;;;;;;;;; Assembler Directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Processor Definition
PROCESSOR   18F87K22
; Radix Definition 
RADIX	DEC
; List Definition
;   C: Set the page (i.e., Column) width
;   N: Set the page length
;   X: Turn MACRO expansion on or off
; LIST	C = 160, N = 0, X = OFF
; Include File:
#include <xc.inc>

; PIC18F87K22 Configuration Bit Settings
// <editor-fold defaultstate="collapsed" desc="CONFIG Definitions">
; CONFIG1L
CONFIG  RETEN = ON            ; VREG Sleep Enable bit (Enabled)
CONFIG  INTOSCSEL = HIGH      ; LF-INTOSC Low-power Enable bit (LF-INTOSC in 
                              ;	    High-power mode during Sleep)
CONFIG  SOSCSEL = HIGH        ; SOSC Power Selection and mode Configuration bits 
			      ;	    (High Power SOSC circuit selected)
CONFIG  XINST = OFF           ; Extended Instruction Set (Disabled)

; CONFIG1H
CONFIG  FOSC = HS1            ; Oscillator (HS oscillator 
			      ;	    (Medium power, 4 MHz - 16 MHz))
CONFIG  PLLCFG = OFF          ; PLL x4 Enable bit (Disabled)
CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor (Disabled)
CONFIG  IESO = OFF            ; Internal External Oscillator Switch Over Mode 
			      ;	    (Disabled)

; CONFIG2L
CONFIG  PWRTEN = ON           ; Power Up Timer (Enabled)
CONFIG  BOREN = ON            ; Brown Out Detect (Controlled with SBOREN bit)
CONFIG  BORV = 1              ; Brown-out Reset Voltage bits (2.7V)
CONFIG  BORPWR = ZPBORMV      ; BORMV Power level (ZPBORMV instead of BORMV 
			      ;	    is selected)

; CONFIG2H
CONFIG  WDTEN = OFF           ; Watchdog Timer (WDT disabled in hardware; 
			      ;	    SWDTEN bit disabled)
CONFIG  WDTPS = 1048576       ; Watchdog Postscaler (1:1048576)

; CONFIG3L
CONFIG  RTCOSC = SOSCREF      ; RTCC Clock Select (RTCC uses SOSC)
CONFIG  EASHFT = ON           ; External Address Shift bit (Address Shifting 
			      ;	    enabled)
CONFIG  ABW = MM              ; Address Bus Width Select bits (8-bit 
			      ;	    address bus)
CONFIG  BW = 16               ; Data Bus Width (16-bit external bus mode)
CONFIG  WAIT = OFF            ; External Bus Wait (Disabled)

; CONFIG3H
CONFIG  CCP2MX = PORTC        ; CCP2 Mux (RC1)
CONFIG  ECCPMX = PORTE        ; ECCP Mux (Enhanced CCP1/3 [P1B/P1C/P3B/P3C] 
			      ;	    muxed with RE6/RE5/RE4/RE3)
; CONFIG  MSSPMSK = MSK7        ; MSSP address masking (7 Bit address masking 
			      ;	    mode)
CONFIG  MCLRE = ON            ; Master Clear Enable (MCLR Enabled, RG5 Disabled)

; CONFIG4L
CONFIG  STVREN = ON           ; Stack Overflow Reset (Enabled)
CONFIG  BBSIZ = BB2K          ; Boot Block Size (2K word Boot Block size)

; CONFIG5L
CONFIG  CP0 = OFF             ; Code Protect 00800-03FFF (Disabled)
CONFIG  CP1 = OFF             ; Code Protect 04000-07FFF (Disabled)
CONFIG  CP2 = OFF             ; Code Protect 08000-0BFFF (Disabled)
CONFIG  CP3 = OFF             ; Code Protect 0C000-0FFFF (Disabled)
CONFIG  CP4 = OFF             ; Code Protect 10000-13FFF (Disabled)
CONFIG  CP5 = OFF             ; Code Protect 14000-17FFF (Disabled)
CONFIG  CP6 = OFF             ; Code Protect 18000-1BFFF (Disabled)
CONFIG  CP7 = OFF             ; Code Protect 1C000-1FFFF (Disabled)

; CONFIG5H
CONFIG  CPB = OFF             ; Code Protect Boot (Disabled)
CONFIG  CPD = OFF             ; Data EE Read Protect (Disabled)

; CONFIG6L
CONFIG  WRT0 = OFF            ; Table Write Protect 00800-03FFF (Disabled)
CONFIG  WRT1 = OFF            ; Table Write Protect 04000-07FFF (Disabled)
CONFIG  WRT2 = OFF            ; Table Write Protect 08000-0BFFF (Disabled)
CONFIG  WRT3 = OFF            ; Table Write Protect 0C000-0FFFF (Disabled)
CONFIG  WRT4 = OFF            ; Table Write Protect 10000-13FFF (Disabled)
CONFIG  WRT5 = OFF            ; Table Write Protect 14000-17FFF (Disabled)
CONFIG  WRT6 = OFF            ; Table Write Protect 18000-1BFFF (Disabled)
CONFIG  WRT7 = OFF            ; Table Write Protect 1C000-1FFFF (Disabled)

; CONFIG6H
CONFIG  WRTC = OFF            ; Config. Write Protect (Disabled)
CONFIG  WRTB = OFF            ; Table Write Protect Boot (Disabled)
CONFIG  WRTD = OFF            ; Data EE Write Protect (Disabled)

; CONFIG7L
CONFIG  EBRT0 = OFF           ; Table Read Protect 00800-03FFF (Disabled)
CONFIG  EBRT1 = OFF           ; Table Read Protect 04000-07FFF (Disabled)
CONFIG  EBRT2 = OFF           ; Table Read Protect 08000-0BFFF (Disabled)
CONFIG  EBRT3 = OFF           ; Table Read Protect 0C000-0FFFF (Disabled)
CONFIG  EBRT4 = OFF           ; Table Read Protect 10000-13FFF (Disabled)
CONFIG  EBRT5 = OFF           ; Table Read Protect 14000-17FFF (Disabled)
CONFIG  EBRT6 = OFF           ; Table Read Protect 18000-1BFFF (Disabled)
CONFIG  EBRT7 = OFF           ; Table Read Protect 1C000-1FFFF (Disabled)

; CONFIG7H
CONFIG  EBRTB = OFF           ; Table Read Protect Boot (Disabled)
// </editor-fold>

;;;;;;;;;;;;;;;;;;;;;;;;; MACRO Definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; MACRO Definitions:

; MOVLF
; Description:
;   Move literal value to given register. 
; Input: 
;   lit: literal value
;   dest: destination 
;   access: Access bank or not. Possible values are 'a' for access bank or
;	'b' for banked memory.
  MOVLF	    MACRO   lit, dest, access
    MOVLW   lit	    ; Move literal into WREG
    BANKSEL	(dest)	; Select Bank for next file instruction
    MOVWF   BANKMASK(dest), access  ; Move WREG into destination file
  ENDM
  
;; POINT adapted from Reference: Peatman CH 7 LCD
;POINT
; Description:
;   Loads strings into table pointer. 
; Input: 
;   stringname: name of the variable containg the desired string.
  POINT	    MACRO stringname
    MOVLF high stringname, TBLPTRH, A 
    MOVLF low stringname, TBLPTRL, A
  ENDM
  
;DISPLAY
; Description:
;   Displays a given register in binary on the LCD. 
; Input: 
;   register: The register that is to be displayed on the LCD. 
  DISPLAY   MACRO register
    MOVFF register, BYTE 
    CALL ByteDisplay
  ENDM
  
  
 ;;;;;;;;;;;;;;;;;;;;;;;;; Project Sections ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  
// <editor-fold defaultstate="collapsed" desc="Project Sections">
;;;;;;;;;;;;;;;;;;;;;; Power-On-Reset entry point ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PSECT	resetVec, class = CODE, reloc = 2
resetVec:
    NOP	    ; No Operation, give time for reset to occur
    goto    main    ; Go to main after reset

;;;;;;;;;;;;;;;;;;; Interrupt Service Routine Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;
; High Priority ISR Vector Definition:
PSECT	HiPriISR_Vec, class = CODE, reloc = 2
HiPriISR_Vec:
    GOTO    HiPriISR	; Go to High Priority ISR
    
; Low Priority ISR Vector Definition:
PSECT	LoPriISR_Vec, class = CODE, reloc = 2
LoPriISR_Vec:
    GOTO    LoPriISR	; Go to Low Priority ISR
// </editor-fold>  
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
// <editor-fold defaultstate="collapsed" desc="Variables">
; Objects to be defined in Access Bank
; Examples:
PSECT	udata_acs
COUNT:	DS  1	; Reserve 1 byte for CNT in access bank at 0x000 (literal or file location)
ROLLOVR: DS 1
REG12:	DS  1	; anotha one
WREG_TEMP:	DS	1   ; Temp variables used in Low Pri ISR
STATUS_TEMP:	DS	1
BSR_TEMP:	DS	1    
TMR1X:		DS	1   ; Eight-bit extension to TMR1
CCPR1X:		DS	1   ; Eight-bit extension to CCPR1
DTIMEX:		DS	1   ; Delta time variable of half period of square wave
DTIMEH:		DS	1   ; Will copy HalfPeriod constant into these 3 registers
DTIMEL:		DS	1
D8TIMEX:		DS	1   ; Delta time variable of half period of square wave
D8TIMEH:		DS	1   ; Will copy HalfPeriod constant into these 3 registers
D8TIMEL:		DS	1

TMR3X:		DS	1   ; Eight-bit extension to TMR3
CCPR3X:		DS	1   ; Eight-bit extension to CCPR3    
DLTIMEX:		DS	1   ; Delta time variable of half period of square wave
DLTIMEH:		DS	1   ; Will copy HalfPeriod constant into these 3 registers
DLTIMEL:		DS	1
DHTIMEX:		DS	1   ; Delta time variable of half period of square wave
DHTIMEH:		DS	1   ; Will copy HalfPeriod constant into these 3 registers
DHTIMEL:		DS	1
    
DIR_RPG:	DS	1   ; Direction of RPG
RPG_TEMP:	DS	1   ; Temp variable used for RPG state
OLDPORTD:	DS	1   ; Used to hold previous state of RPG    
    
DISPSTR: DS 6	; 1 for placement, 3 for decimal nums, 1 for end 
TDEC: DS 2
    
SWSTAT:	DS  1	; Keep track of RE3 switch current state
BYTE:		DS  1	; Reserve 1 byte for BYTE in access bank
BYTESTR:	DS  10	; Reserve 10 bytes for BYTESTR in access bank

GLOBAL highnum
highnum: DS 2
GLOBAL lownum
lownum: DS 2
    

    
; Objects to be defined in Bank 1
PSECT	udata_bank1
; not used   
    
;;;;;;; Constant Strings (Program Memory) ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
PSECT romData, space = 0, class = CONST  
LCDstr:  
    DB  0x33,0x32,0x28,0x01,0x0C,0x06,0x00  ;Initialization string for LCD
    
LCDs:
    DB 0x80,'A','S','E','N',' ','5', '0', '6', '7', 0x00	    ;Write "ASEN 5067" to first line of LCD
    
BYTE_1:
    DB 0xC0,'P','W','=','1', '.', '0', '0', 'm', 's', 0x00
   
// </editor-fold>


; Program Section: All Code Starts here
PSECT	code
;;;;;;;;;;;;;;;;;;;;;;;;;;;; Mainline Code ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
main:
    RCALL    Initial	; Call to Initial Routine
mloop:
    RCALL RPG
    BRA	    mloop
   
HalfPeriod equ 800000    
Half800Period equ 3200000  
Period20 equ 19000
Period1 equ 1000
;;;;;;;;;;;;;;;;;;;;;; Initialization Routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Initial:
    ; Initialization of pins
    MOVLF 0x0F, TRISD, a    ; Set TRISD
    MOVLF 11110011B, TRISJ, a    ; Set TRISJ
    MOVLF 00001000B, TRISE, a
    MOVLF 11000000B, TRISB, A

    MOVLF 0x00,  TRISC, a		    ; Set I/O for PORTC
    CLRF    LATC, a		    ; Clear lines on PORTC
    CLRF LATD,0		    ; Turn off all LEDS on LATD
    CLRF LATJ,0		    ; Turn off ALL LEDS on LATJ       
    ;Initialize LCD
    MOVLF   00000000B,INTCON,A
    MOVLF   00001000B,T0CON,A		; Set up Timer0 for a delay of 1 s
    MOVLF   high Lilnum,TMR0H,A		; Writing binary 25536 to TMR0H / TMR0L
    MOVLF   low Lilnum,TMR0L,A		; Write high byte first, then low!
    BSF	    T0CON, 7,A			; Turn on the timer
    RCALL   InitLCD			; Initialize LCD
    POINT   LCDs			; ASEN 5067
    RCALL   DisplayC			; Display character subroutine
     
    RCALL Wait1sec	; call subroutine to wait 1 second
    BTG LATD,5,0	; Turn ON RD5
    RCALL Wait1sec	; call subroutine to wait 1 second
    BTG LATD,5,0	; Turn OFF RD5
    BTG LATD,6,0	; Turn ON RD6
    RCALL Wait1sec	; call subroutine to wait 1 second
    BTG LATD,6,0	; Turn OFF RD6
    BTG LATD,7,0	; Turn ON RD7
    RCALL Wait1sec	; call subroutine to wait 1 second
    BTG LATD,7,0	; Turn OFF RD7
    
    ;Display / PWM configuration
    MOVLF 0xB5, lownum+1, A
    MOVLF 0xC8, lownum, A 
    MOVLF 0xFC, highnum+1,A
    MOVLF 0x18, highnum, A
    
    POINT BYTE_1
    RCALL DisplayC
    
    CLRF DISPSTR, A
    MOVLF 0xC3, DISPSTR, A 
    MOVLF 0x31, DISPSTR+1, A
    MOVLF '.', DISPSTR+2, A
    MOVLF 0x30,DISPSTR+3,A
    MOVLF 0x30, DISPSTR+4,A
    CLRF DISPSTR+5, A
    CLRF REG12, A
    
    BTG LATE, 4, 0
            
    ;Configure CCP1 200 ms timer
    MOVLF   low HalfPeriod, DTIMEL, a	; Load DTIME with HalfPeriod constant
    MOVLF   high HalfPeriod, DTIMEH, a
    MOVLF   low highword HalfPeriod, DTIMEX, a
    
    ;Configure CCP1 800 ms timer
    MOVLF   low Half800Period, D8TIMEL, a	; Load DTIME with HalfPeriod constant
    MOVLF   high Half800Period, D8TIMEH, a
    MOVLF   low highword Half800Period, D8TIMEX, a
    
    MOVLF   00000010B, T1CON, a	    ; 16 bit timer, buffer H/L registers
    MOVLF   00001010B, CCP1CON, a   ; Select compare mode, software interrupt only
    MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank				
    MOVLF   00000000B, CCPTMRS0, b  ; Set TMR1 for use with ECCP1, Using BSR!
    BSF	    RCON, 7, a 		    ; Set IPEN bit <7> enables priority levels
    BCF	    IPR1, 0, a		    ; TMR1IP bit <0> assigns low priority to TMR1 interrupts
    BCF	    IPR3, 1, a		    ; CCP1IP bit<1> assign low pri to ECCP1 interrupts
    CLRF    TMR1X, a		    ; Clear TMR1X extension
    MOVLF   low highword HalfPeriod, CCPR1X, a	; Make first 24-bit compare 
						; occur quickly 16bit+8bit ext 
						; Note: 200000 (= 0x30D40)
    BSF	    PIE3, 1, a	    ; CCP1IE bit <1> enables ECCP1 interrupts
    BSF	    PIE1, 0, a	    ; TMR1IE bit <0> enables TMR1 interrupts
    BSF	    INTCON, 6, a    ; GIEL bit <6> enable low-priority interrupts to CPU
    BSF	    INTCON, 7, a    ; GIEH bit <7> enable all interrupts
    BSF	    T1CON, 0, a	    ; TMR1ON bit <0> turn on timer1    
    
    ;Configure CCP3 20 ms timer
    MOVLF   low Period20, DLTIMEL, a	; Load DTIME with HalfPeriod constant
    MOVLF   high Period20, DLTIMEH, a
    MOVLF   low highword Period20, DLTIMEX, a
    
    ;Configure CCP3 1 ms timer
    MOVLF   low Period1, DHTIMEL, a	; Load DTIME with HalfPeriod constant
    MOVLF   high Period1, DHTIMEH, a
    MOVLF   low highword Period1, DHTIMEX, a
    
    CLRF TRISE, a
    CLRF LATE, a
    MOVLF   00100000B, T3CON, a	    ; 1 bit timer, buffer H/L registers
    MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank
    MOVLF   00001010B, CCP3CON, b   ; Select compare mode, software interrupt only
    MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank			    
    MOVLF   11000000B, CCPTMRS0, b  ; Set TMR3 for use with ECCP3, Using BSR!
    BSF	    RCON, 7, a		    ; Set IPEN bit <7> enables priority levels
    BSF	    IPR2, 1, a		    ; TMR3IP bit <1> assigns high priority to TMR3 interrupts
    BSF	    IPR4, 0, a		    ; CCP3IP bit <0> assign high pri to ECCP3 interrupts
    CLRF    TMR3X, a		    ; Clear TMR1X extension
    MOVLF   low highword HalfPeriod, CCPR3X, a	; Make first 24-bit compare 
						; occur quickly 16bit+8bit ext 
						; Note: 200000 (= 0x30D40)
    BSF	    PIE4, 0, a	    ; CCP3IE bit <1> enables ECCP1 interrupts
    BSF	    PIE2, 1, a	    ; TMR3IE bit <0> enables TMR3 interrupts
    BSF	    INTCON, 6, a    ; GIEL bit <6> enable low-priority interrupts to CPU
    BSF	    INTCON, 7, a    ; GIEH bit <7> enable all interrupts
    BSF	    T3CON, 0, a	    ; TMR3ON bit <0> turn on timer1   
        
    RETURN			    ; Return to Mainline code

    ; pg 326 

;;;;;;; HiPriISR interrupt service routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

HiPriISR:                        ; High-priority interrupt service routine
;       <execute the handler for interrupt source, write your code here if needed>
;       <clear that source's interrupt flag>
    ; COnfigure CCP3 as high priority - then can either have another ccp for the 20 ms timing? Might be nice
HL2:
	BTFSS	PIR4, 0, a	; Test CCP3IF bit <0> for this interrupt
        BRA	HL3
        RCALL	CCP3handler	; Call CCP1handler for generating RC2 output
	BRA	HL2
HL3:
	BTFSS	PIR2, 1, a	; Test TMR3IF bit <0> for this interrupt
        BRA	HL4
        RCALL	TMR3handler	; Call TMR3handler for timing with CCP3
        BRA	HL2
HL4: 
        RETFIE  1	    ; Return and restore STATUS, WREG, and BSR
			    ; from shadow registers

;;;;;;; LoPriISR interrupt service routine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

LoPriISR:				; Low-priority interrupt service routine
        MOVFF	STATUS, STATUS_TEMP	; Set aside STATUS and WREG
        MOVWF	WREG_TEMP, a
	MOVFF	BSR, BSR_TEMP        
L2:
        BTFSS	PIR3, 1, a	; Test CCP1IF bit <1> for this interrupt
        BRA	L3
        RCALL	CCP1handler	; Call CCP1handler for generating RC2 output
        BRA	L2
L3:
        BTFSS	PIR1, 0, a	; Test TMR1IF bit <0> for this interrupt
        BRA	L4
        RCALL	TMR1handler	; Call TMR1handler for timing with CCP1
        BRA	L2
L4:
        MOVF	WREG_TEMP, w, a	    ; Restore WREG and STATUS
        MOVFF	STATUS_TEMP, STATUS
	MOVFF	BSR_TEMP, BSR        
        RETFIE			; Return from interrupt, reenabling GIEL  

;;;;;;;; CCP Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CCP3handler:			; First must test of TMR1IF occurred at the same time
        BTFSS	PIR2, 1, a	; If TMR3's overflow flag is set? skip to test CCP bit7
        BRA	L35		; If TMR1F was clear, branch to check extension bytes
        BTFSC	CCPR3H, 7, a	; Is bit 7 a 0? Then TMR1/CCP just rolled over, need to inc TMR1X
        BRA	L35		; Is bit 7 a 1? Then let TMR1handler inc TMR1X 
        INCF	TMR3X, f, a	; TMR1/CCP just rolled over, must increment TMR1 extension
        BCF	PIR2, 1, a	; and clear TMR1IF bit <0> flag 
				;(Since TMR1 handler was unable to and arrived here first!)
L35:
        MOVF	TMR3X, w, a	; Check whether extensions are equal
        SUBWF	CCPR3X, w, a	; by subtracting TMR1X and CCPR1X, check if 0
        BNZ	L37		; If not, branch to return
        BTG	LATC, 2, a	; Manually toggle RC2
	; switch period time from 200 ms to 800 depending if LATC is flipped. 
	BTFSS LATC, 2, a
	BRA TMRLOW
	
	MOVF	DHTIMEL, w, a	; and add half period to CCPR1 to add more pulse time
        MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank
	ADDWF	CCPR3L, f, b
        MOVF	DHTIMEH, w, a	; Add to each of the 3 bytes to get 24 bit CCP
        MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank
	ADDWFC	CCPR3H, f, b
        MOVF	DHTIMEX, w, a
        ADDWFC	CCPR3X, f, a
	BRA L37
TMRLOW: 
	MOVF	DLTIMEL, w, a	; and add half period to CCPR1 to add more pulse time
        MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank
	ADDWF	CCPR3L, f, b
        MOVF	DLTIMEH, w, a	; Add to each of the 3 bytes to get 24 bit CCP
        MOVLB   0x0F		    ; Set BSR to bank F for SFRs outside of access bank
	ADDWFC	CCPR3H, f, b
        MOVF	DLTIMEX, w, a
        ADDWFC	CCPR3X, f, a
L37:
        BCF	PIR4, 0, a	; Clear the CCP3IF bit <1> interrupt flag
        RETURN

;;;;;;;; TMR Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
TMR3handler:
        INCF	TMR3X, f, a	;Increment Timer1 extension
        BCF	PIR2, 1, a	;Clear TMR3IF flag and return to service routine
        RETURN	

;;;;;;;; CCP Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
CCP1handler:			; First must test of TMR1IF occurred at the same time
        BTFSS	PIR1, 0, a	; If TMR1's overflow flag is set? skip to test CCP bit7
        BRA	L5		; If TMR1F was clear, branch to check extension bytes
        BTFSC	CCPR1H, 7, a	; Is bit 7 a 0? Then TMR1/CCP just rolled over, need to inc TMR1X
        BRA	L5		; Is bit 7 a 1? Then let TMR1handler inc TMR1X 
        INCF	TMR1X, f, a	; TMR1/CCP just rolled over, must increment TMR1 extension
        BCF	PIR1, 0, a	; and clear TMR1IF bit <0> flag 
				;(Since TMR1 handler was unable to and arrived here first!)
L5:
        MOVF	TMR1X, w, a	; Check whether extensions are equal
        SUBWF	CCPR1X, w, a	; by subtracting TMR1X and CCPR1X, check if 0
        BNZ	L7		; If not, branch to return
        BTG	LATD, 4, a	; Manually toggle RC2
	; switch period time from 200 ms to 800 depending if LATC is flipped. 
	BTFSS LATD, 4, a
	BRA TMER800
	
	MOVF	DTIMEL, w, a	; and add half period to CCPR1 to add more pulse time
        ADDWF	CCPR1L, f, a
        MOVF	DTIMEH, w, a	; Add to each of the 3 bytes to get 24 bit CCP
        ADDWFC	CCPR1H, f, a
        MOVF	DTIMEX, w, a
        ADDWFC	CCPR1X, f, a
	BRA L7
TMER800: 
	MOVF	D8TIMEL, w, a	; and add half period to CCPR1 to add more pulse time
        ADDWF	CCPR1L, f, a
        MOVF	D8TIMEH, w, a	; Add to each of the 3 bytes to get 24 bit CCP
        ADDWFC	CCPR1H, f, a
        MOVF	D8TIMEX, w, a
        ADDWFC	CCPR1X, f, a
L7:
        BCF	PIR3, 1, a	; Clear the CCP1IF bit <1> interrupt flag
        RETURN

;;;;;;;; TMR Handler ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
TMR1handler:
        INCF	TMR1X, f, a	;Increment Timer1 extension
        BCF	PIR1, 0, a	;Clear TMR1IF flag and return to service routine
        RETURN
	
	
;;;;;;; Wait1sec subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;  Code based on lab4_example.asm
; Subroutine to wait 1 sec using Timer 0 
	
Bignum  equ     65536-62500
Wait1sec:
    ; Set TIMER0 to 1 sec delay (code based on lab4_example.asm)
    MOVLF   00000100B,T0CON,A		; Set up Timer0 for a delay of 1 s
    MOVLF   high Bignum,TMR0H,A		; Writing binary 25536 to TMR0H / TMR0L
    MOVLF   low Bignum,TMR0L,A		; Write high byte first, then low!
    
    BSF	    T0CON,7,A			;Turn on timer
sLOOP:
    BTFSS 	INTCON,2,A		    ; Read Timer0 TMR0IF rollover flag and ...
    BRA     sLOOP			    ; Loop if timer has not rolled over
    MOVLF  	high Bignum,TMR0H,A	    ; Then write the timer values into
    MOVLF  	low Bignum,TMR0L,A	    ; the timer high and low registers
    BCF  	INTCON,2,A		    ; Clear Timer0 TMR0IF rollover flag
    RETURN
    
;;;;;;; CHECKSWITCH subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Subroutine to check the status of RD3 button and change RD2 (ASEN5067 ONLY)
				
CHECKSWITCH:   
    BTFSS LATD, 7, a	; CW
    BRA CCW		; Decrement
    BRA CW		; Else increment LCD
    
CCW:	
    MOVLW 0x32
    CPFSLT DISPSTR+1, a
    BRA SET199
    
    MOVLW 0x00
    CPFSGT REG12, a
    BRA SWEND  
    DECF REG12, a   ; keep track of hardstops
    
    ;Add to PWM low and subtract from high
    MOVLW 0x0A
    ADDWF DLTIMEL,1,0
    MOVLW 0x00
    ADDWFC DLTIMEH,1,0
    
    MOVLW 0x0A
    SUBWF DHTIMEL,1,0
    MOVLW 0x00
    SUBWFB DHTIMEH,1,0
    
    ;Add to LCD display
    DECF    DISPSTR+4, A	;subtract from 3rd digit in decimal num
    MOVLW   0x30
    CPFSLT  DISPSTR+4, a
    BRA	    DISPBITS		; Display
    
    MOVLW 0x39
    MOVWF   DISPSTR+4, a	; reset to 9
    DECF    DISPSTR+3, A	; decrement from higher 
    MOVLW 0x30
    CPFSLT  DISPSTR+3,A		
    BRA DISPBITS		; Display if not less than 0x30
    
CW:    
    INCF REG12, a	; Keep track of hardstop
    MOVLW 0x64
    CPFSLT REG12, a
    BRA SET2

    ; Add to PWM high and subtract from low
    MOVLW 0x0A
    ADDWFC DHTIMEL,1,0
    MOVLW 0x00
    ADDWFC DHTIMEH,1,0
    
    MOVLW 0x0A
    SUBWF DLTIMEL,1,0
    MOVLW 0x00
    SUBWFB DLTIMEH,1,0
    
    ;Add to LCD display
    INCF DISPSTR+4, A	;add to 3rd digit in decimal num
    MOVLW 0x39
    CPFSGT  DISPSTR+4, a
    BRA DISPBITS		; Display 
    
    MOVLW 0x30		    ; Reset to 0
    MOVWF DISPSTR+4, a
    INCF DISPSTR+3, A	    ; Increment the 2nd digit
    MOVLW 0x39
    CPFSGT DISPSTR+3, a
    BRA DISPBITS
    
SET2:
    ; 2000
    MOVLF 0xD0, DHTIMEL,A
    MOVLF 0x07,DHTIMEH,A
    ; 18000
    MOVLF 0x50, DLTIMEL,A
    MOVLF 0x46,DLTIMEH,A
    
    MOVLF 0x32, DISPSTR+1, A
    MOVLF 0x30, DISPSTR+3, A
    MOVLF 0x30, DISPSTR+4, A
    BRA DISPBITS
SET199:
    MOVLW 0x63
    MOVWF REG12, a
  
    ; 18010 
    MOVLF 0x5A, DLTIMEL,A
    MOVLF 0x46,DLTIMEH,A
    ;1990
    MOVLF 0xC6, DHTIMEL,A
    MOVLF 0x07,DHTIMEH,A
    
    MOVLF 0x31, DISPSTR+1, A
    MOVLF 0x39, DISPSTR+3, A
    MOVLF 0x39, DISPSTR+4, A
DISPBITS:  
    LFSR 0, DISPSTR
    RCALL DisplayV
SWEND: 
    RETURN	

;;;;;;; T50 subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; T50 modified version of T40 taken from Reference: Peatman CH 7 LCD
; Pause for 50 microseconds or 50/0.25 = 200 instruction cycles.
; Assumes 16/4 = 4 MHz internal instruction rate (250 ns)
; rcall(2) + movlw(1) + movwf(1) + COUNT*3 - lastBNZ(1) + return(2) = 200 
; Then COUNT = 195/3
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        
T50:
        MOVLW  195/3          ;Each loop L4 takes 3 ins cycles
        MOVWF  COUNT,A		    
Lten:
        DECF  COUNT,F,A
        BNZ	Lten
        RETURN
    
;;;;;;;;DisplayC subroutine;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; DisplayC taken from Reference: Peatman CH7 LCD
; This subroutine is called with TBLPTR containing the address of a constant
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position hex to ASCII.
; This subroutine expects a normal one-byte cursor-positioning code, 0xhh, and
; a null byte at the end of the string 0x00
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

DisplayC:
        BCF   LATB,4,A		    ;Drive RS pin low for cursor positioning code
        TBLRD*			    ;Get byte from string into TABLAT
        MOVF  TABLAT,F,A	    ;Check for leading zero byte
        BNZ	Loop5
        TBLRD+*			    ;If zero, get next byte
Loop5:
	MOVLW 0xF0
	ANDWF LATB,F,A		    ;Clear RB0:RB3, which are used to send LCD data
        BSF   LATB,5,A		    ;Drive E pin high
        MOVF TABLAT,W,A		    ;Move byte from table latch to working register
	ANDLW 0xF0		    ;Mask to get only upper nibble
	SWAPF WREG,W,A		    ;swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	IORWF PORTB,W,A		    ;Mask to include the rest of PORTB
	MOVWF LATB,A		    ;Send upper nibble out to LATB
        BCF   LATB,5,A		    ;Drive E pin low so LCD will accept nibble
	
	MOVLW 0xF0
	ANDWF LATB,F,A		    ;Clear RB0:RB3, which are used to send LCD data
        BSF   LATB,5,A		    ;Drive E pin high again
        MOVF TABLAT,W,A		    ;Move byte from table latch to working register
	ANDLW 0x0F		    ;Mask to get only lower nibble
	IORWF PORTB,W,A		    ;Mask to include the rest of PORTB
	MOVWF LATB,A		    ;Send lower nibble out to LATB
        BCF   LATB,5,A		    ;Drive E pin low so LCD will accept nibble
        RCALL T50		    ;Wait 50 usec so LCD can process
	
        BSF   LATB,4,A		    ;Drive RS pin high for displayable characters
        TBLRD+*			    ;Increment pointer, then get next byte
        MOVF  TABLAT,F,A	    ;Is it zero?
        BNZ	Loop5
        RETURN
	
;;;;;;; DisplayV subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; DisplayV taken from Reference: Peatman CH7 LCD
; This subroutine is called with FSR0 containing the address of a variable
; display string.  It sends the bytes of the string to the LCD.  The first
; byte sets the cursor position.  The remaining bytes are displayed, beginning
; at that position.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;	

DisplayV:
        BCF     LATB,4,A	    ;Drive RS pin low for cursor positioning code
Loop6:
	MOVLW 0xF0
	ANDWF LATB,F,A		    ;Clear RB0:RB3, which are used to send LCD data
        BSF   LATB,5,A		    ;Drive E pin high
        MOVF INDF0,W,A		    ;Move byte from FSR to working register
	ANDLW 0xF0		    ;Mask to get only upper nibble
	SWAPF WREG,W,A		    ;swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	IORWF PORTB,W,A		    ;Mask to include the rest of PORTB
	MOVWF LATB,A		    ;Send upper nibble out to LATB
        BCF   LATB,5,A		    ;Drive E pin low so LCD will accept nibble
	
	MOVLW 0xF0
	ANDWF LATB,F,A		    ;Clear RB0:RB3, which are used to send LCD data
        BSF   LATB,5,A		    ;Drive E pin high again
        MOVF INDF0,W,A		    ;Move byte from table latch to working register
	ANDLW 0x0F		    ;Mask to get only lower nibble
	IORWF PORTB,W,A		    ;Mask to include the rest of PORTB
	MOVWF LATB,A		    ;Send lower nibble out to LATB
        BCF   LATB,5,A		    ;Drive E pin low so LCD will accept nibble
        RCALL T50		    ;Wait 50 usec so LCD can process
	  
        BSF   LATB,4,A		    ;Drive RS pin high for displayable characters
        MOVF  PREINC0,W,A	    ;Increment pointer, then get next byte
        BNZ   Loop6
        RETURN	
    
    
;;;;;; ByteDisplay subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Display whatever is in BYTE as a binary number.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
ByteDisplay:
        POINT   BYTE_1			    ;Display "BYTE="
        RCALL   DisplayC
        LFSR    0,BYTESTR+8
LBD:
        CLRF  WREG,A
        RRCF  BYTE,F,A			    ;Move bit into carry
        RLCF  WREG,F,A			    ;and from there into WREG
        IORLW 0x30			    ;Convert to ASCII
        MOVWF POSTDEC0,A		    ; and move to string
        MOVF  FSR0L,W,A			    ;Done?
        SUBLW low BYTESTR
        BNZ	LBD

        LFSR    0,BYTESTR		    ;Set pointer to display string
        MOVLF   0xC4,BYTESTR,A		    ;Add cursor-positioning code
        CLRF    BYTESTR+9,A		    ;and end-of-string terminator
        RCALL   DisplayV
        RETURN    
    
    
;;;;;;; InitLCD  subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;This subroutine was taken from lab4_example_picas.asm
; Subroutine to read the values of the RPG and display on RJ2 and RJ3    
    
InitLCD:
        MOVLF  10,COUNT,A	    ; Wait 0.1 second for LCD to power up
Loop3:
        RCALL  Wait10ms		    ; Call wait10ms 10 times to 0.1 second
        DECF  COUNT,F,A
        BNZ	Loop3
        BCF     LATB,4,A	    ; RS=0 for command mode to LCD
        POINT   LCDstr		    ; Set up table pointer to initialization string
        TBLRD*			    ; Get first byte from string into TABLAT
Loop4:
	CLRF LATB,A		    ; First set LATB to all zero	
        BSF   LATB,5,A		    ; Drive E high - enable LCD
	MOVF TABLAT,W,A		    ; Move byte from program memory into working register
	ANDLW 0xF0		    ; Mask to get only upper nibble
	SWAPF WREG,W,A		    ; Swap so that upper nibble is in right position to move to LATB (RB0:RB3)
	IORWF PORTB,W,A		    ; Mask with the rest of PORTB to retain existing RB7:RB4 states
	MOVWF LATB,A		    ; Update LATB to send upper nibble
        BCF   LATB,5,A		    ; Drive E low so LCD will process input
        RCALL Wait10ms		    ; Wait ten milliseconds
	
	CLRF LATB,A		    ; Reset LATB to all zero	    
        BSF  LATB,5,A		    ; Drive E high
        MOVF TABLAT,W,A		    ; Move byte from program memory into working register
	ANDLW 0x0F		    ; Mask to get only lower nibble
	IORWF PORTB,W,A		    ; Mask lower nibble with the rest of PORTB
	MOVWF LATB,A		    ; Update LATB to send lower nibble
        BCF   LATB,5,A		    ; Drive E low so LCD will process input
        RCALL Wait10ms		    ; Wait ten milliseconds
        TBLRD+*			    ; Increment pointer and get next byte
        MOVF  TABLAT,F,A	    ; Check if we are done, is it zero?
        BNZ	Loop4
        RETURN

;;;;;;;; LoopTime subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; This subroutine waits for Timer0 to complete its ten millisecond count
;; sequence. It does so by waiting for sixteen-bit Timer0 to roll over. To obtain
;; a period of 10ms/250ns = 40000 clock periods, it needs to remove
;; 65536-40000 or 25536 counts from the sixteen-bit count sequence.  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
Lilnum  equ     65536-40000

Wait10ms:
        BTFSS 	INTCON,2,A		    ; Read Timer0 TMR0IF rollover flag and ...
        BRA     Wait10ms		    ; Loop if timer has not rolled over
        MOVLF  	high Lilnum,TMR0H,A	    ; Then write the timer values into
        MOVLF  	low Lilnum,TMR0L,A	    ; the timer high and low registers
        BCF  	INTCON,2,A		    ; Clear Timer0 TMR0IF rollover flag
        RETURN
	
;;;;;; RPG subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; Credit: This subroutine modified from Peatman book Chapter 8 - RPG
; This subroutine deciphers RPG changes into values for RPG direction DIR_RPG of 0, +1, or -1.
; DIR_RPG = +1 for CW change, 0 for no change, and -1 for CCW change.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
RPG:
        CLRF	DIR_RPG, a	; Clear for "no change" return value.
        MOVF	PORTD, w, a	; Copy PORTD into W.
        MOVWF	RPG_TEMP, a	;  and RPG_TEMP.
        XORWF	OLDPORTD, w, a	; Check for any change?
        ANDLW	00000011B	; Masks just the RPG pins          
        BZ  L8		; If zero, RPG has not moved, ->return
			; But if the two bits have changed then...
	; Form what a CCW change would produce.          	
	RRCF	OLDPORTD, w, a	; Rotate right once into carry bit   
	BNC	L9	; If no carry, then bit 0 was a 0 -> branch to L9
        BCF	WREG, 1, a	; Otherwise, bit 0 was a 1. Then clear bit 1
				; to simulate what a CCW change would produce
        BRA	L10	; Branch to compare if RPG actually matches new CCW pattern in WREG
L9:
        BSF	WREG, 1, a  ; Set bit 1 since there was no carry
			    ; again to simulate what CCW would produce
L10:			    ; Test direction of RPG
        XORWF	RPG_TEMP, w, a	; Did the RPG actually change to this output?
        ANDLW	00000011B	; Masks the RPG pins
        BNZ	L11		; If not zero, then branch to L11 for CW case
        DECF	DIR_RPG, f, a	; If zero then change DIR_RPG to -1, must be CCW. 
	BCF	LATD,7,a
	RCALL CHECKSWITCH
        BRA	L8		; Done so branch to return
L11:	; CW case 
        INCF	DIR_RPG, f, a	; Change DIR_RPG to +1 for CW.
	BSF	LATD,7,a
	RCALL CHECKSWITCH
L8:
        MOVFF	RPG_TEMP, OLDPORTD  ; Save current RPG state as OLDPORTD
        RETURN
	
;;;;;;; End of Program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;		
	
	
    END     resetVec		    ; End program, return to reset vector ;;;;;;; ASEN 4-5067 Lab3 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

