/*
 * File:   main.c
 * Author: Javier Lima
 *
 * Created on 28 de abril 2018, 08:26 PM
 */

//#include "C:\Program Files (x86)\Microchip\xc32\v1.44\pic32mx\include\p32xxxx.h"
//#include "C:\Program Files (x86)\Microchip\xc32\v1.44\pic32mx\include\xc.h"
#include <string.h>
#include <stdio.h>
//#include <stdlib.h>
#include <p32xxxx.h>
#include <stdlib.h>

#include "../sys/attribs.h"
// PIC32MX250F128B Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON           // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = ON            // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
#pragma config FUSBIDIO = OFF           // USB USID Selection (Controlled by Port Function)
#pragma config FVBUSONIO = OFF          // USB VBUS ON Selection (Controlled by Port Function)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_12        // USB PLL Input Divider (12x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SYSTEM CLOCK 40MHz
// Peripheral Clock 40MHz
#include <xc.h>

#define SHORT_DELAY (39062)
#define LONG_DELAY	(39062)

#define BDOS LATBbits.LATB10
#define BUNO LATBbits.LATB11
#define AUNO LATBbits.LATB13
#define ADOS LATBbits.LATB14

void IniciarUart(void);
void EnviarCaracter(char c);
char RecibirCaracter(void);
void EnviarCadena(char *c);
char *RecibirCadena(char *s);
void IntTimer1(void);
void IntUart1(void);
void IntCNA(void);
void SentidoLlantas(char b);
void Velocidad(char b);

int td = 0;
char prueba;
char charIn;
char cadenaIn[512];
char cadenaOut[512];
char cad1[] = "sijkds\n";
char cad2[] = "NO funciono";
char prueba2[] = "prueba\n\0";
int Puerto1;
int Puerto2;
int i = 0;

char mensaje[4];
char temp1;
int contador = 0;
int bandera = 0;

void main(void) {
    
    //CLOCK INTERNO CON 40kHz en PBCLK
    
    OSCTUNbits.TUN  = 0b000000;         // Center Freq. Oscilator at 8MHz
    OSCCONbits.PLLODIV = 0b001;         // PLL out divided by 2
    OSCCONbits.FRCDIV = 0b000;          // FRC divided by 1
    OSCCONbits.SOSCRDY = 0b0;           // Secondary Osc is warming or turned off
    OSCCONbits.PBDIVRDY = 0b1;          // PBDIV bits can be written
    OSCCONbits.PBDIV = 0b00;            // PBCLK is SYSCLK divided by 1
    OSCCONbits.PLLMULT = 0b101;         // Clock is multiplied by 20
    OSCCONbits.COSC = 0b001;            // FRCPLL
    OSCCONbits.NOSC = 0b001;            // FRCPLL
    OSCCONbits.CLKLOCK = 0b0;           // Clock and PLL can be modify
    OSCCONbits.ULOCK = 0b0;             // USB is out of lock or is disabled
    OSCCONbits.SLOCK = 0b0;             // PLL Lock Status
    OSCCONbits.SLPEN = 0b0;             // Will enter in idle mode when WAIT is executed
    OSCCONbits.CF = 0b0;                // No clock failure has been detected
    OSCCONbits.UFRCEN = 0b0;            // Use the primary Osc or USB PLL as the USB clock source
    OSCCONbits.SOSCEN = 0b0;            // Disable secondary oscilator
    OSCCONbits.OSWEN = 0b0;             // Oscilator switch is complete  
    
    //CONFIGURACIÓN DE ENTRADAS Y SALIDAS
    
    ANSELA = 0;                         // Pines Digitales Puerto A
    ANSELB = 0;                         // Pines Digitales Puerto B
    
    TRISAbits.TRISA0 = 1;               // Pin 2 como entrada - ENC DERECHA A
    TRISAbits.TRISA1 = 1;               // Pin 3 como entrada - ENC DERECHA B
    TRISBbits.TRISB0 = 1;               // Pin 4 como entrada - ENC IZQUIERDA A
    TRISBbits.TRISB1 = 1;               // Pin 5 como salida - ENC IZQUIERDA B
    TRISBbits.TRISB2 = 1;               // Pin 6 como entrada - UART RECEIVER
    TRISBbits.TRISB3 = 0;               // Pin 7 como salida - UART TRANSMITER
    TRISAbits.TRISA2 = 0;               // Pin 9 como salida -
    TRISAbits.TRISA3 = 0;               // Pin 10 como salida - 
    TRISBbits.TRISB4 = 0;               // Pin 11 como salida - 
    TRISAbits.TRISA4 = 0;               // Pin 12 como salida - 
    TRISBbits.TRISB7 = 0;               // Pin 16 como salida - 
    TRISBbits.TRISB8 = 0;               // Pin 17 como salida - SERVO MOTOR
    TRISBbits.TRISB9 = 0;               // Pin 18 como salida - DRIVER PWM B
    TRISBbits.TRISB10 = 0;               // Pin 21 como salida - DRIVER B2
    TRISBbits.TRISB11 = 0;               // Pin 22 como salida - DRIVER B1
    TRISBbits.TRISB13 = 0;               // Pin 24 como salida - DRIVER A1
    TRISBbits.TRISB14 = 0;               // Pin 25 como salida - DRIVER A2
    TRISBbits.TRISB15 = 0;               // Pin 26 como salida - DRIVER PWM A
    
    // RE-MAPEO DE PINES
    
    CFGCONbits.IOLOCK = 0b0;            // IOLock is disabled.
    
    U1RXRbits.U1RXR = 0b0100;           // Se asigna RPB2 a U1RX - Receiver Uart - pin 6
    RPB3Rbits.RPB3R = 0b0001;           // Se asigna U1TX a RPB3 - Transmiter Uart - pin 7
    
    RPB8Rbits.RPB8R = 0b0101;           // Se asigna OC2 a RPB8 - Servo - pin 17
    RPB9Rbits.RPB9R = 0b0101;           // Se asigna OC3 a RPB9 - Motor PWM B - pin 18
    RPB10Rbits.RPB10R = 0b0000;         // No se asigna nada - Motor Bin2  - pin 21
    RPB11Rbits.RPB11R = 0b0000;         // No se asigna nada - Motor Bin1 - pin 22
    RPB13Rbits.RPB13R = 0b0000;         // No se asigna nada - Motor Ain1 - pin 24
    RPB14Rbits.RPB14R = 0b0000;         // No se asigna nada - Motor Ain2 - pin 25
    RPB15Rbits.RPB15R = 0b0101;         // Se asigna OC1 a RPB15 - Motor PWM A - pin 26
    
    // CONFIGURACIÓN DE UART 1
    
    U1BRG = 86;                         //baud rate=115200 a Fpb = 40 MHZ with BRGH = 1

    U1MODEbits.ON = 0b1;                // UART Enabled
    U1MODEbits.SIDL = 0b0;
    U1MODEbits.IREN = 0b0;
    U1MODEbits.RTSMD = 0b1;             // Simplex Mode
    U1MODEbits.UEN = 0b00;
    U1MODEbits.WAKE = 0b0;
    U1MODEbits.LPBACK = 0b0;
    U1MODEbits.ABAUD = 0b0;
    U1MODEbits.RXINV = 0b0;
    U1MODEbits.BRGH = 0b1;              // High Speed Mode
    U1MODEbits.PDSEL = 0b00;            // No parity
    U1MODEbits.STSEL = 0b0;             // 1 stop bit   
    
    U1STAbits.URXISEL = 0b00;
    U1STAbits.URXEN = 1;                // habilitar recepcion
    U1STAbits.UTXEN = 1;                // habilitar transmision
    
    // SE CONFIGURA TIMER 1
    
    T1CONbits.ON = 0b0;                 // Timer 1 is disabled
    T1CONbits.SIDL = 0b0;
    T1CONbits.TWDIS = 0b0;
    T1CONbits.TWIP = 0b0;
    T1CONbits.TGATE = 0b0;
    T1CONbits.TCKPS = 0b11;             // Prescaler at 1:256
    T1CONbits.TSYNC = 0b0;
    T1CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source  
    
    //TIMER 2 ES PARA CONTROLAR LOS MOTORES
    
    T2CONbits.ON = 0b0;                 // Timer 2 is disabled
    T2CONbits.SIDL = 0b0;
    T2CONbits.TGATE = 0b0;
    T2CONbits.TCKPS = 0b100;            // Prescaler at 1:16
    T2CONbits.T32 = 0b0;
    T2CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
    //TIMER 3 ES PARA CONTROLAR AL SERVO
    
    T3CONbits.ON = 0b0;                 // Timer 3 is disabled
    T3CONbits.SIDL = 0b0;
    T3CONbits.TGATE = 0b0;
    T3CONbits.TCKPS = 0b100;            // Prescaler at 1:16
    T3CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
    //PWM 1 PARA CONTROLAR MOTOR A
    
    OC1CONbits.ON = 0b0;                // OC1 is disabled
    OC1CONbits.SIDL = 0b0;
    OC1CONbits.OC32 = 0b0;
    OC1CONbits.OCFLT = 0b0;
    OC1CONbits.OCTSEL = 0b0;            // Timer 2 is the clock source
    OC1CONbits.OCM = 0b110;             // Estandar PWM
    
    //PWM 2 PARA CONTROLAR SERVO
    
    OC2CONbits.ON = 0b0;                // OC2 is disabled
    OC2CONbits.SIDL = 0b0;
    OC2CONbits.OC32 = 0b0;
    OC2CONbits.OCFLT = 0b0;
    OC2CONbits.OCTSEL = 0b1;            // Timer 3 is the clock source
    OC2CONbits.OCM = 0b110;             // Estandar PWM
    
    //PWM 3 PARA CONTROLAR MOTOR B
    
    OC3CONbits.ON = 0b0;                // OC3 is disabled
    OC3CONbits.SIDL = 0b0;
    OC3CONbits.OC32 = 0b0;
    OC3CONbits.OCFLT = 0b0;
    OC3CONbits.OCTSEL = 0b0;            // Timer 2 is the clock source
    OC3CONbits.OCM = 0b110;             // Estandar PWM
    
    // Definición de periodos de Timers
    
    PR1 = 39062;                        // Periodo de 250mS
    PR2 = 5000;                         // Freq de 500Hz (Motores)
    PR3 = 50000;                        // Periodo de 20mS (Servo) 2500-5000 / 0° - 180°
    
    // Definición de Duty Cycle de PWM's
    
    OC1RS = 0;
    OC2RS = 44000;                      // PR3 * 85%
    OC3RS = 0;                      
    
    //INTERRUPTS CONFIG
    
    //Interrupción Timer 1
    IPC1bits.T1IP = 0b111;              // Prioridad de Interrupción 7
    IPC1bits.T1IS = 0b11;               // Sub-Prioridad de Interrupción 3
    IFS0bits.T1IF = 0b0;                // Interruption Flag to 0
    IEC0bits.T1IE = 0b1;                // Timer 1 Interrupt Enabled
    
    //Interrupción UART
    IPC8bits.U1IP = 0b111;              // Prioridad de Interrupción 7
    IPC8bits.U1IS = 0b11;               // Sub-Prioridad de Interrupción 3
    IFS1bits.U1RXIF = 0b0;              // Interruption Flag of Receiver to 0
    IFS1bits.U1TXIF = 0b0;              // Interruption Flag of Transmiter to 0
    IEC1bits.U1RXIE = 0b1;              // UART Receiver 1 Interrupt Enabled
    IEC1bits.U1TXIE = 0b1;              // UART Transmiter 1 Interrupt Enabled
    
    //Interrupción Change Control Port A y B
    IPC8bits.CNIP = 0b111;              // Prioridad de Interrupción 7
    IPC8bits.CNIS = 0b11;               // Sub-Prioridad de Interrupción 3
    CNENAbits.CNIEA0 = 0b1;             // Change Control pin A0 Interrupt Enabled
    CNPUAbits.CNPUA0 = 0b0;             // Deshabilitar PULL UP en A0
    CNPDAbits.CNPDA0 = 0b1;             // Habilitar PULL DOWN en A0
    CNENAbits.CNIEA1 = 0b1;             // Change Control pin A1 Interrupt Enabled
    CNPUAbits.CNPUA1 = 0b0;             // Deshabilitar PULL UP en A1
    CNPDAbits.CNPDA1 = 0b1;             // Habilitar PULL DOWN en A1
    CNENBbits.CNIEB0 = 0b1;             // Change Control pin B0 Interrupt Enabled
    CNPUBbits.CNPUB0 = 0b0;             // Deshabilitar PULL UP en B0
    CNPDBbits.CNPDB0 = 0b1;             // Habilitar PULL DOWN en B0
    CNENBbits.CNIEB1 = 0b1;             // Change Control pin B1 Interrupt Enabled
    CNPUBbits.CNPUB1 = 0b0;             // Deshabilitar PULL UP en B1
    CNPDBbits.CNPDB1 = 0b1;             // Habilitar PULL DOWN en B1
    
    CNCONAbits.ON = 0b1;                // Change Control Notice ON portA
    CNCONBbits.ON = 0b1;                // Change Control Notice ON portB
    IFS1bits.CNAIF = 0b0;               // Interruption Flag of PORTA to 0
    IFS1bits.CNBIF = 0b0;               // Interruption Flag of PORTB to 0
    IEC1bits.CNAIE = 0b1;               // Change Control PortA Interrupt Enabled
    IEC1bits.CNBIE = 0b1;               // Change Control PortB Interrupt Enabled
    
    INTCONbits.MVEC = 0b1;              // Interrupt Controller for Multi-Vectored Mode
    
    //Enable peripherals
    
    OC1CONbits.ON = 0b1;                // OC1 is enabled
    OC2CONbits.ON = 0b1;                // OC2 is enabled
    OC3CONbits.ON = 0b1;                // OC3 is enabled  
    T1CONbits.ON = 0b1;                 // Timer 1 is enabled
    T2CONbits.ON = 0b1;                 // Timer 2 is enabled
    T3CONbits.ON = 0b1;                 // Timer 3 is enabled
    T1CONbits.TON = 0b1;
   
    __builtin_enable_interrupts();      // Se habilitan las interrupciones
    
    
    while(1)
    {    
        if(bandera == 2){
            EnviarCadena(mensaje);
            //SentidoLlantas(mensaje[1]);
            //Velocidad(mensaje[2]); 
            
            bandera = 0;
        }     
    }
    return;
}

//void __ISR (#vector, )
void __ISR (4, IPL7SOFT) IntTimer1(void)
{
    //LATBINV = 0x0001;
    //LATBbits.LATB0 = ~PORTBbits.RB0;    // Invertir estado del pin
    //LATBbits.LATB1 = ~PORTBbits.RB1;    // Invertir estado del pin
    
	IFS0bits.T1IF = 0;                  // Resetear bandera de interrupción
}

void __ISR (32, IPL7SOFT) IntUart1(void)
{
    if(IFS1bits.U1RXIF){                // Se ejecuta si se recibe un dato
        charIn = U1RXREG;

        if(charIn == '-'){
            bandera = 1;
            contador = 0;
        }

        if (bandera == 1 & charIn != '-'){
            if(charIn != '\n' && contador < 4){
                mensaje[contador++] = charIn;
            }else{
                bandera = 2;
            }
        }

        IFS1bits.U1RXIF = 0b0;
    }else if(IFS1bits.U1TXIF){          // Se ejecuta si se termina de enviar un dato
        
        
        IFS1bits.U1TXIF = 0b0;
    }
}

// Interrupción para detectar cambio de estado en A0, A1, B0 y B1 para encoders.
void __ISR (34, IPL7SOFT) IntCN(void)
{
    if(CNSTATAbits.CNSTATA0 && PORTAbits.RA0){
        LATAINV = 0x0004;                        //Invierte el estado del pin RA2
    }
    if(CNSTATAbits.CNSTATA1 && PORTAbits.RA1){
        LATAINV = 0x0008;                        //Invierte el estado del pin RA3
    }
    if(CNSTATBbits.CNSTATB0 && PORTBbits.RB0){
        LATBINV = 0x0010;                        //Invierte el estado del pin RB4
    }
    if(CNSTATBbits.CNSTATB1 && PORTBbits.RB1){
        LATAINV = 0x0010;                        //Invierte el estado del pin RA4
    }
    
	IFS1bits.CNAIF = 0b0;                   // Resetear bandera de interrupción
    IFS1bits.CNBIF = 0b0;                   // Resetear bandera de interrupción
}

//ENVIAR CARACTER POR SERIAL
void EnviarCaracter(char c) {
    //U1STAbits.URXEN=0;//deshabilitar recepcion
    //U1STAbits.UTXEN=1;//habilitar transmision
    U1TXREG = c;
    while (!U1STAbits.TRMT); //espera a que termine la transimision
}

//ENVIAR STRING POR SERIAL
void EnviarCadena(char p[]){
    for(i = 0; i <= 3; i++){
        EnviarCaracter(p[i]); 
    }
}

void SentidoLlantas(char b){
    
    switch(b){
        case '1':
            BDOS = 0;
            BUNO = 1;
            AUNO = 0;
            ADOS = 1;
            EnviarCaracter('F');
            break;
        case '2':
            BDOS = 1;
            BUNO = 0;
            AUNO = 0;
            ADOS = 1;
            EnviarCaracter('L');
            break;
        case '3':
            BDOS = 0;
            BUNO = 1;
            AUNO = 1;
            ADOS = 0;
            EnviarCaracter('R');
            break;
        case '4':
            BDOS = 1;
            BUNO = 0;
            AUNO = 1;
            ADOS = 0;
            EnviarCaracter('B');
            break;
    }
}

int porcent = 500;

void Velocidad(char b){
    
    switch(b){
        case '0':
            OC1RS = 0;
            OC3RS = 0;
            break;
        case '1':
            OC1RS = 500;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        case '2':
            OC1RS = 1000;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        case '3':
            OC1RS = 1500;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        case '4':
            OC1RS = 2000;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        case '5':
            OC1RS = 2500;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        case '6':
            OC1RS = 3000;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        case '7':
            OC1RS = 3500;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        case '8':
            OC1RS = 4000;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        case '9':
            OC1RS = 4500;
            OC3RS = OC1RS - OC1RS/porcent;
            break;
        default:
            break;
    }
}