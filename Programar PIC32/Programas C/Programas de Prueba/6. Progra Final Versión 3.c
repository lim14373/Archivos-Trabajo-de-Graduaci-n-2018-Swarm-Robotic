/*
 * File:   main.c
 * Author: Javier Lima
 *
 * Created on 28 de abril 2018, 08:26 PM
 */

//#define Usuario "Megaproyecto"          // Nombre del Router
//#define LargoUs 12                      // Cantidad de caracteres del Usuario, sin contar comillas
//#define Contrasena "ABC1234567"         // Contraseña del Router
//#define LargoCn 10                      // Cantidad de caracteres de la Contraseña
#define Usuario "TURBONETT_0B6287"      // Nombre del Router
#define LargoUs 16                      // Cantidad de caracteres del Usuario, sin contar comillas
#define Contrasena "7EB8E43D08"         // Contraseña del Router
#define LargoCn 10                      // Cantidad de caracteres de la Contraseña
#define Puerto "5400"                   // Puerto TCP
#define LargoPt 4                       // Cantidad de caracteres del Puerto

#include <string.h>
#include <stdio.h>
#include <p32xxxx.h>
#include <stdlib.h>
#include <math.h>
#include "../sys/attribs.h"

// PIC32MX250F128B Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow multiple reconfigurations)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow multiple reconfigurations)
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

#define BDOS 0x0400                     // Driver Motor B2 - RB10 - Pin 21
#define BUNO 0x0800                     // Driver Motor B1 - RB11 - Pin 22
#define AUNO 0x2000                     // Driver Motor A1 - RB13 - Pin 24
#define ADOS 0x4000                     // Driver Motor A2 - RB14 - Pin 25

#define LED1 0x0010                     // Led 1 - RA4 - Pin 12
#define LED2 0x0020                     // Led 2 - RB5 - Pin 14
#define LED3 0x0080                     // Led 3 - RB7 - Pin 16

void EnviarCaracter(char c);
void EnviarCadena(char c[], int a);
void IntTimer1(void);
void IntTimer4(void);
void IntTimer5(void);
void IntUart1(void);
void IntCN(void);
void SentidoLlantas(void);
void Velocidad(void);
void EnviarPasos(void);
void EnviarSentidos(void);
void EnviarTiempos(void);
void Radar(void);
void InicializarESP(void);
void EnviarDatos(char p);

char charIn;
int j = 0;
int k = 0;
int PasosDerF = 0;
int PasosIzqF = 0;
int PasosDerB = 0;
int PasosIzqB = 0;
int contTiemposIzq = 0;
int contTiemposDer = 0;
int DifTiempos = 0;
long TiempoActual = 0;
long TiempoPasadoDer = 0;
long TiempoPasadoIzq = 0;
long PromTiempos = 0;
int TiemposPasosDer[] = {0,0,0,0,0,0};
int TiemposPasosIzq[] = {0,0,0,0,0,0};
char SentidoActualIzq = 'f';
char SentidoActualDer = 'f';

char smsR[50];
int contR = 0;
int Revision = 0;

int dmil = 0;
int mil = 0;
int cen = 0;
int dec = 0;
int uni = 0;
int var;

int PasosESP = 1;
int InitESP = 1;

int blink = 0;

int ContarSonico1 = 0;
int ContarSonico2 = 0;
int TiempoSonico1 = 0;
int TiempoSonico2 = 0;
int Mandar = 0;
int Grados = 146;
int g = 0;

int Rutina = 0;

//Variables de prueba

int Enviando = 0;

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
    TRISBbits.TRISB0 = 1;               // Pin 4 como entrada - ENC IZQUIERDA B
    TRISBbits.TRISB1 = 1;               // Pin 5 como entrada - ENC IZQUIERDA A
    TRISBbits.TRISB2 = 1;               // Pin 6 como entrada - UART RECEIVER
    TRISBbits.TRISB3 = 0;               // Pin 7 como salida - UART TRANSMITER
    TRISAbits.TRISA2 = 1;               // Pin 9 como entrada - MAXSONAR 1 
    TRISAbits.TRISA3 = 1;               // Pin 10 como entrada - MAXSONAR 2
    TRISBbits.TRISB4 = 1;               // Pin 11 como entrada - MAXSONAR 3
    TRISAbits.TRISA4 = 0;               // Pin 12 como salida - LED 1
    TRISBbits.TRISB5 = 0;               // Pin 14 como salida - LED 2
    TRISBbits.TRISB7 = 0;               // Pin 16 como salida - LED 3
    TRISBbits.TRISB8 = 0;               // Pin 17 como salida - SERVO MOTOR
    TRISBbits.TRISB9 = 0;               // Pin 18 como salida - DRIVER PWM B
    TRISBbits.TRISB10 = 0;              // Pin 21 como salida - DRIVER B2
    TRISBbits.TRISB11 = 0;              // Pin 22 como salida - DRIVER B1
    TRISBbits.TRISB13 = 0;              // Pin 24 como salida - DRIVER A1
    TRISBbits.TRISB14 = 0;              // Pin 25 como salida - DRIVER A2
    TRISBbits.TRISB15 = 0;              // Pin 26 como salida - DRIVER PWM A
    
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
    
    U1BRG = 4;                          // BaudRate = 2000000 at Fpb = 40 MHZ with BRGH = 1
    
    U1MODEbits.PDSEL = 0b00;            // No parity
    U1MODEbits.ON = 0b0;                // UART Disabled
    U1MODEbits.SIDL = 0b0;
    U1MODEbits.IREN = 0b0;
    U1MODEbits.RTSMD = 0b1;             // Simplex ModeA
    U1MODEbits.UEN = 0b00;
    U1MODEbits.WAKE = 0b0;
    U1MODEbits.LPBACK = 0b0;
    U1MODEbits.ABAUD = 0b0;
    U1MODEbits.RXINV = 0b0;
    U1MODEbits.BRGH = 0b1;              // High Speed Mode
    U1MODEbits.STSEL = 0b0;             // 1 stop bit   
    
    U1STAbits.URXISEL = 0b00;           // Se realiza una interrupción por cada caracter recibido
    U1STAbits.URXEN = 1;                // habilitar recepcion
    U1STAbits.UTXEN = 1;                // habilitar transmision
    
    // SE CONFIGURA TIMER 1
    
    T1CONbits.ON = 0b0;                 // Timer 1 is disabled
    T1CONbits.SIDL = 0b0;
    T1CONbits.TWDIS = 0b0;
    T1CONbits.TWIP = 0b0;
    T1CONbits.TGATE = 0b0;
    T1CONbits.TCKPS = 0b00;             // Prescaler at 1:1
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
    T3CONbits.TCKPS = 0b011;            // Prescaler at 1:8
    T3CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
    //TIMER 4 ES PARA MEDIR LOS SENSORES DE DISTANCIA CADA 50ms
    
    T4CONbits.ON = 0b0;                 // Timer 4 is disabled
    T4CONbits.SIDL = 0b0;
    T4CONbits.TGATE = 0b0;
    T4CONbits.TCKPS = 0b101;            // Prescaler at 1:32
    T4CONbits.T32 = 0b0;
    T4CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
    //TIMER 5 ES PARA MEDIR EL TIEMPO DE LA SEÑAL PWM DE SENSORES DE DISTANCIA
    
    T5CONbits.ON = 0b0;                 // Timer 5 is disabled
    T5CONbits.SIDL = 0b0;
    T5CONbits.TGATE = 0b0;
    T5CONbits.TCKPS = 0b000;            // Prescaler at 1:1
    T5CONbits.TCS = 0b0;                // Internal Peripheral Clock as Timer Clock Source
    
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
    // FreqPWM = FreqPB/[(PRx+1)*(TMR Prescale Value)]
    
    PR1 = 3999;                         // Cada 100uS se activa la interrupción del T1
    PR2 = 4999;                         // Freq de 500Hz (Motores)
    PR3 = 16666;                        // Freq de 300Hz (Servo) 4000-11300 / 0° - 140°
    PR4 = 62499;                        // Freq de 20Hz (Lectura de MaxSonar)
    PR5 = 998;                          // Freq de 40kHz (Conteo de tiempo de PWM MaxSonar)
    
    // Definición de Duty Cycle de PWM's
    
    OC1RS = 0;
    OC2RS = 4000;                       // 24% of PR3 to 67.8% of PR3 for Servo Traxxas
    OC3RS = 0;                      
    
    //INTERRUPTS CONFIG
    
    //Interrupción Timer 1
    IPC1bits.T1IP = 0b111;              // Prioridad de Interrupción 7
    IPC1bits.T1IS = 0b11;               // Sub-Prioridad de Interrupción 3
    
    //Interrupción Timer 4
    IPC4bits.T4IP = 0b111;              // Prioridad de Interrupción 7
    IPC4bits.T4IS = 0b11;               // Sub-Prioridad de Interrupción 3
    
    //Interrupción Timer 5
    IPC5bits.T5IP = 0b111;              // Prioridad de Interrupción 7
    IPC5bits.T5IS = 0b11;               // Sub-Prioridad de Interrupción 3
    
    //Interrupción UART
    IPC8bits.U1IP = 0b111;              // Prioridad de Interrupción 7
    IPC8bits.U1IS = 0b11;               // Sub-Prioridad de Interrupción 3
    
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
    CNENAbits.CNIEA2 = 0b0;             // Change Control pin A2 Interrupt Disabled
    CNPUAbits.CNPUA2 = 0b0;             // Deshabilitar PULL UP en A2
    CNPDAbits.CNPDA2 = 0b1;             // Habilitar PULL DOWN en A2
    CNENAbits.CNIEA3 = 0b0;             // Change Control pin A3 Interrupt Disabled
    CNPUAbits.CNPUA3 = 0b0;             // Deshabilitar PULL UP en A3
    CNPDAbits.CNPDA3 = 0b1;             // Habilitar PULL DOWN en A3
    
    CNCONAbits.ON = 0b1;                // Change Control Notice ON portA
    CNCONBbits.ON = 0b1;                // Change Control Notice ON portB
    
    //HABILITACIÓN DE MÓDULOS
    
    OC1CONbits.ON = 0b1;                // OC1 is enabled
    OC2CONbits.ON = 0b1;                // OC2 is enabled
    OC3CONbits.ON = 0b1;                // OC3 is enabled  
    T1CONbits.ON = 0b1;                 // Timer 1 is enabled
    T1CONbits.TON = 0b1;
    T2CONbits.ON = 0b1;                 // Timer 2 is enabled
    T3CONbits.ON = 0b1;                 // Timer 3 is enabled
    T4CONbits.ON = 0b1;                 // Timer 4 is enabled
    T5CONbits.ON = 0b1;                 // Timer 5 is enabled
    U1MODEbits.ON = 0b1;                // UART Enabled
    
    // HABILITACIÓN DE INTERRUPCIONES
    
    IEC0bits.T1IE = 0b1;                // Timer 1 Interrupt Enabled
    IEC1bits.U1RXIE = 0b1;              // UART 1 Receiver Interrupt Enabled
    IEC1bits.CNAIE = 0b1;               // Change Control PortA Interrupt Enabled
    IEC1bits.CNBIE = 0b1;               // Change Control PortB Interrupt Enabled
    
    // INHABILITACIÓN DE INTERRUPCIONES
    
    IEC0bits.T4IE = 0b0;                // Timer 4 Interrupt Disabled
    IEC0bits.T5IE = 0b0;                // Timer 5 Interrupt Disabled
    
    INTCONbits.MVEC = 0b1;              // Interrupt Controller for Multi-Vectored Mode
    
    // LIMPIEZA DE BANDERAS DE INTERRUPCIÓN
    
    IFS0bits.T1IF = 0b0;                // Interruption Flag of Timer 1 to 0
    IFS0bits.T4IF = 0b0;                // Interruption Flag of Timer 4 to 0
    IFS0bits.T5IF = 0b0;                // Interruption Flag of Timer 5 to 0
    IFS1bits.U1RXIF = 0b0;              // Interruption Flag of UART 1 Receiver to 0
    IFS1bits.CNAIF = 0b0;               // Interruption Flag of PORTA to 0
    IFS1bits.CNBIF = 0b0;               // Interruption Flag of PORTB to 0
    
    // LIMPIEZA DE SALIDAS
    
    LATACLR = LED1;
    LATBCLR = LED2;
    LATBCLR = LED3;
    
    // Llantas hacia adelante
    
    LATBCLR = BDOS;
    LATBSET = BUNO;
    LATBCLR = AUNO;
    LATBSET = ADOS;
   
    __builtin_enable_interrupts();      // Se habilitan las interrupciones
    
    InicializarESP();
    
    while(1)
    {
        if(Revision == 2){
            switch (smsR[0]){
                case 's':
                    SentidoLlantas();
                    break;
                case 'v':
                    Velocidad();
                    break;
                case 'd':
                    EnviarSentidos();
                    break;
                case 'p':
                    EnviarPasos();
                    break;
                case 't':
                    EnviarTiempos();
                    break;
                case 'r':
                    Radar();
                    break;
                case 'c':
                    EnviarDatos('c');
                    break;
                default:
                    EnviarDatos('e');
                    break;
            }
            Revision = 0;
        }
        
        switch(Rutina){
            case 1:
                switch(blink){
                    case 1:
                        LATACLR = LED1;
                        LATBCLR = LED2;
                        LATBCLR = LED3;
                        break;
                    case 2500:
                        LATASET = LED1;
                        LATBSET = LED2;
                        LATBSET = LED3;
                        break;
                    case 5000:
                        LATACLR = LED1;
                        LATBCLR = LED2;
                        LATBCLR = LED3;
                        break;
                    case 7500:
                        LATASET = LED1;
                        LATBSET = LED2;
                        LATBSET = LED3;
                        break;
                    case 10000:
                        LATACLR = LED1;
                        LATBCLR = LED2;
                        LATBCLR = LED3;
                        break;
                    case 12500:
                        LATASET = LED1;
                        LATBSET = LED2;
                        LATBSET = LED3;
                        break;
                    case 15000:
                        LATACLR = LED1;
                        LATBCLR = LED2;
                        LATBCLR = LED3;
                        Rutina = 2;
                        blink = 0;
                        break;
                    default:
                        break;
                }
                break;
            case 2:
                switch(blink){
                    case 1:
                        LATACLR = LED1;
                        LATBCLR = LED2;
                        LATBCLR = LED3;
                        break;
                    case 47550:
                        LATASET = LED1;
                        break;
                    case 47900:
                        LATACLR = LED1;
                        break;
                    case 48250:
                        LATBSET = LED2;
                        break;
                    case 48600:
                        LATBCLR = LED2;
                        break;
                    case 48950:
                        LATBSET = LED3;
                        break;
                    case 49300:
                        LATBCLR = LED3;
                        break;
                    case 49650:
                        LATASET = LED1;
                        break;
                    case 50000:
                        blink = 0;
                        break;
                    default:
                        break;
                }
                break;
        }
    }
    return;
}

void EnviarDatos(char p){
    
    switch(p){
        case 'e':
            EnviarCadena("AT+CIPSEND=0,7\r\n",16);
            Enviando = 1;
            while(Enviando == 1){
            }
            EnviarCadena("Error\r\n",7);
            break;
        case 'c':
            EnviarCadena("AT+CIPSEND=0,9\r\n",16);
            Enviando = 1;
            while(Enviando == 1){
            }
            EnviarCadena("Robot 1\r\n",9);
            break;
        default:
            EnviarCadena("AT+CIPSEND=0,4\r\n",16);
            Enviando = 1;
            while(Enviando == 1){
            }
            EnviarCadena("OK\r\n",4);
            break;
    }
    
}

// Interrupción para detectar cambio de estado en A0, A1, B0 y B1 para encoders.
void __ISR (34, IPL7SOFT) IntCN(void)
{
    if(CNSTATAbits.CNSTATA0 && PORTAbits.RA0){   //Se activa en el rising del pin 2 - Enc 1A
        
        if(SentidoActualDer == 'f'){
            PasosDerF++;
        }else{
            PasosDerB++;
        }
        
        DifTiempos = TiempoActual - TiempoPasadoDer;
        
        TiempoPasadoDer = TiempoActual;
        
        if(DifTiempos < 65000){
            TiemposPasosDer[contTiemposDer++] = DifTiempos;
        }else{
            TiemposPasosDer[contTiemposDer++] = 65000;
        }
        
        if(contTiemposDer == 6){
            contTiemposDer = 0;
        }
        
    }
    if(CNSTATAbits.CNSTATA1 && PORTAbits.RA1){   //Se activa en el rising del pin 3 - Enc 1B
        
        if(PORTAbits.RA0){
            SentidoActualDer = 'b';              //Llanta derecha gira hacia atrás
        }else{
            SentidoActualDer = 'f';              //Llanta derecha gira hacia adelante
        }
        
    }
    if(CNSTATBbits.CNSTATB0 && PORTBbits.RB0){   //Se activa en el rising del pin 4 - Enc 2B
        
        if(PORTBbits.RB1){
            SentidoActualIzq = 'f';              //Llanta derecha gira hacia atrás
        }else{
            SentidoActualIzq = 'b';              //Llanta derecha gira hacia adelante
        }

    }
    if(CNSTATBbits.CNSTATB1 && PORTBbits.RB1){   //Se activa en el rising del pin 5 - Enc 2A
        
        if(SentidoActualIzq == 'f'){
            PasosIzqF++;
        }else{
            PasosIzqB++;
        }
        
        DifTiempos = TiempoActual - TiempoPasadoIzq;
        
        TiempoPasadoIzq = TiempoActual;
        
        if(DifTiempos < 65000){
            TiemposPasosIzq[contTiemposIzq++] = DifTiempos;
        }else{
            TiemposPasosIzq[contTiemposIzq++] = 65000;
        }
        
        if(contTiemposIzq == 6){
            contTiemposIzq = 0;
        }
    }
    
    
    if(CNSTATAbits.CNSTATA2){
        if(PORTAbits.RA2){                              //Se activa en el rising del pin 9 - MaxSonar 1
            ContarSonico1 = 0;
        }else{                                          //Se activa en el falling del pin 9 - MaxSonar 1
            TiempoSonico1 = ContarSonico1;
        }
    }
    
    if(CNSTATAbits.CNSTATA3){
        if(PORTAbits.RA3){                              //Se activa en el rising del pin 10 - MaxSonar 2
            ContarSonico2 = 0;
        }else{                                          //Se activa en el falling del pin 10 - MaxSonar 2
            TiempoSonico2 = ContarSonico2;
        }
    }
    
	IFS1bits.CNAIF = 0b0;                   // Resetear bandera de interrupción
    IFS1bits.CNBIF = 0b0;                   // Resetear bandera de interrupción
}

// Interrupción del Timer 4 se activa cada 50ms
void __ISR (16, IPL7SOFT) IntTimer4(void)
{
    
    Mandar = 1;
	IFS0bits.T4IF = 0;                  // Resetear bandera de interrupción
}

// Interrupción del Timer 5 se activa cada 25us
void __ISR (20, IPL7SOFT) IntTimer5(void)
{    
    ContarSonico1++;
    ContarSonico2++;
    
	IFS0bits.T5IF = 0;                  // Resetear bandera de interrupción
}

void Radar(void){
    
    EnviarCadena("AT+CIPSEND=0,1168\r\n",19);
    Enviando = 1;
    while(Enviando == 1){
    }
    
    OC2RS = 4000;
    
    __builtin_disable_interrupts();
    
    IEC0bits.T4IE = 0b1;                // Timer 4 Interrupt Enabled
    IEC0bits.T5IE = 0b1;                // Timer 5 Interrupt Enabled
    
    IEC1bits.CNAIE = 0b0;               // Change Control PortA Interrupt Disabled
    CNENAbits.CNIEA2 = 0b1;             // Change Control pin A2 Interrupt Enabled
    CNENAbits.CNIEA3 = 0b1;             // Change Control pin A3 Interrupt Enabled
    IEC1bits.CNAIE = 0b1;               // Change Control PortA Interrupt Enabled
    
    IFS0bits.T4IF = 0b0;                // Interruption Flag of Timer 4 to 0
    IFS0bits.T5IF = 0b0;                // Interruption Flag of Timer 5 to 0
    IFS1bits.CNAIF = 0b0;               // Interruption Flag of PORTA to 0
    
    __builtin_enable_interrupts();
    
    Grados = 0;
    
    blink = 0;
    
    while(Grados < 146){
        if(Mandar == 1){
            Mandar = 0;                 
            
            dmil = TiempoSonico1;
            mil = dmil/1000;
            dmil -= mil*1000;
            cen = dmil/100;
            dmil -= cen*100;
            dec = dmil/10;
            dmil -= dec*10;
            uni = dmil;
            
            dmil = TiempoSonico2;
            
            U1TXREG = mil+48;
            while (!U1STAbits.TRMT);                //espera a que termine la transimision
            U1TXREG = cen+48;
            while (!U1STAbits.TRMT);                //espera a que termine la transimision
            U1TXREG = dec+48;
            while (!U1STAbits.TRMT);                //espera a que termine la transimision
            U1TXREG = uni+48;
            while (!U1STAbits.TRMT);                //espera a que termine la transimision
            
            mil = dmil/1000;
            dmil -= mil*1000;
            cen = dmil/100;
            dmil -= cen*100;
            dec = dmil/10;
            dmil -= dec*10;
            uni = dmil;
            
            U1TXREG = mil+48;
            while (!U1STAbits.TRMT);                //espera a que termine la transimision
            U1TXREG = cen+48;
            while (!U1STAbits.TRMT);                //espera a que termine la transimision
            U1TXREG = dec+48;
            while (!U1STAbits.TRMT);                //espera a que termine la transimision
            U1TXREG = uni+48;
            while (!U1STAbits.TRMT);                //espera a que termine la transimision
            
            Grados++;
        }
        
        switch(blink){
            case 1:
                LATACLR = LED1;
                LATBCLR = LED2;
                LATBCLR = LED3;
                break;
            case 500:
                LATASET = LED1;
                break;
            case 1000:
                LATACLR = LED1;
                break;
            case 1500:
                LATBSET = LED2;
                break;
            case 2000:
                LATBCLR = LED2;
                break;
            case 2500:
                LATBSET = LED3;
                break;
            case 3000:
                blink = 0;
                break;
            default:
                break;
        }
    }         
    
    blink = 0;
    
    TiempoSonico1 = 0;
    TiempoSonico2 = 0;
    
    OC2RS = 4000;
    
    __builtin_disable_interrupts();
    
    IEC0bits.T4IE = 0b0;                // Timer 4 Interrupt Disabled
    IEC0bits.T5IE = 0b0;                // Timer 5 Interrupt Disabled
    
    IEC1bits.CNAIE = 0b0;               // Change Control PortA Interrupt Disabled
    CNENAbits.CNIEA2 = 0b0;             // Change Control pin A2 Interrupt Disabled
    CNENAbits.CNIEA3 = 0b0;             // Change Control pin A3 Interrupt Disabled
    IEC1bits.CNAIE = 0b1;               // Change Control PortA Interrupt Enabled
    
    IFS0bits.T4IF = 0b0;                // Interruption Flag of Timer 4 to 0
    IFS0bits.T5IF = 0b0;                // Interruption Flag of Timer 5 to 0
    IFS1bits.CNAIF = 0b0;               // Interruption Flag of PORTA to 0
    
    __builtin_enable_interrupts();
    
}

// Interrupción del Timer 1 se activa cada 0.1ms
void __ISR (4, IPL7SOFT) IntTimer1(void)
{
    
    if(InitESP == 1){
        blink++;
    }
    
    if(Grados < 146){
        g++;
        if(g == 10){
            g = 0;
            OC2RS++;  
        }
    }
    
    TiempoActual++;
    
	IFS0bits.T1IF = 0;                  // Resetear bandera de interrupción
}

// Interrupción del UART Transmisión y Recepción
void __ISR (32, IPL7SOFT) IntUart1(void)
{
    charIn = U1RXREG;
    
    if(Enviando == 1 && charIn == '>'){
        Enviando = 0;
    }
    
    if(InitESP == 0){
        if(charIn != '\n' && charIn != '\r'){
            smsR[contR++] = charIn;
        }else if(charIn == '\n'){
            Revision = 1;
            contR = 0;
        }        
    }else if (Revision == 1){
        if(charIn != '\n' && charIn != '\r' && contR < 10){
            smsR[contR++] = charIn;
        }else if(charIn == '\n'){
            Revision = 2;
        }
    }else if(charIn == ':'){
        Revision = 1;
        contR = 0;
    }

    IFS1bits.U1RXIF = 0b0;
}

//ENVIAR CARACTER POR SERIAL
void EnviarCaracter(char c) {
    __builtin_disable_interrupts();
    U1TXREG = c;
    while (!U1STAbits.TRMT);                //espera a que termine la transimision
    __builtin_enable_interrupts();
}

//ENVIAR STRING POR SERIAL
void EnviarCadena(char p[], int a){
    for(k = 0; k < a; k++){
        EnviarCaracter(p[k]); 
    }
}

//Modifica el sentido en el que giran las llantas
void SentidoLlantas(void){
    
    switch(smsR[1]){
        case '1':            // Llantas hacia adelante
            LATBCLR = BDOS;
            LATBSET = BUNO;
            LATBCLR = AUNO;
            LATBSET = ADOS;
            break;
        case '2':            // Llanta derecha hacia adelante, izquierda hacia atrás
            LATBSET = BDOS;
            LATBCLR = BUNO;
            LATBCLR = AUNO;
            LATBSET = ADOS;
            break;
        case '3':            // Llanta derecha hacia atrás, izquierda hacia adelante
            LATBCLR = BDOS;
            LATBSET = BUNO;
            LATBSET = AUNO;
            LATBCLR = ADOS;
            break;
        case '4':            // Llantas hacia atrás
            LATBSET = BDOS;
            LATBCLR = BUNO;
            LATBSET = AUNO;
            LATBCLR = ADOS;
            break;
        default:
            break;
    }    
}

// Modifica la velocidad de la llanta derecha o izquierda
void Velocidad(void){
     
    var = (smsR[2]-48)*1000 + (smsR[3]-48)*100 + (smsR[4]-48)*10 + smsR[5] - 48;
    
    if(var > 4500){
        var = 4500;
    }else if(var < 0){
        var = 0;
    }
    
    switch(smsR[1]){
        case 'd':
            OC1RS = var;
            break;
        case 'i':
            OC3RS = var;
            break;
        default:
            break;
    }    
}

//Envía los pasos recorridos por las llantas
void EnviarPasos(void){
    
    EnviarCadena("AT+CIPSEND=0,20\r\n",17);
    Enviando = 1;
    while(Enviando == 1){
    }
    
    dmil = PasosDerF/10000;
    PasosDerF -= dmil*10000;
    mil = PasosDerF/1000;
    PasosDerF -= mil*1000;
    cen = PasosDerF/100;
    PasosDerF -= cen*100;
    dec = PasosDerF/10;
    PasosDerF -= dec*10;
    uni = PasosDerF;
    PasosDerF -= uni;
    
    EnviarCaracter(dmil+48);
    EnviarCaracter(mil+48);
    EnviarCaracter(cen+48);
    EnviarCaracter(dec+48);
    EnviarCaracter(uni+48);
    
    dmil = PasosIzqF/10000;
    PasosIzqF -= dmil*10000;
    mil = PasosIzqF/1000;
    PasosIzqF -= mil*1000;
    cen = PasosIzqF/100;
    PasosIzqF -= cen*100;
    dec = PasosIzqF/10;
    PasosIzqF -= dec*10;
    uni = PasosIzqF;
    PasosIzqF -= uni;
    
    EnviarCaracter(dmil+48);
    EnviarCaracter(mil+48);
    EnviarCaracter(cen+48);
    EnviarCaracter(dec+48);
    EnviarCaracter(uni+48);
    
    dmil = PasosDerB/10000;
    PasosDerB -= dmil*10000;
    mil = PasosDerB/1000;
    PasosDerB -= mil*1000;
    cen = PasosDerB/100;
    PasosDerB -= cen*100;
    dec = PasosDerB/10;
    PasosDerB -= dec*10;
    uni = PasosDerB;
    PasosDerB -= uni;
    
    EnviarCaracter(dmil+48);
    EnviarCaracter(mil+48);
    EnviarCaracter(cen+48);
    EnviarCaracter(dec+48);
    EnviarCaracter(uni+48);
    
    dmil = PasosIzqB/10000;
    PasosIzqB -= dmil*10000;
    mil = PasosIzqB/1000;
    PasosIzqB -= mil*1000;
    cen = PasosIzqB/100;
    PasosIzqB -= cen*100;
    dec = PasosIzqB/10;
    PasosIzqB -= dec*10;
    uni = PasosIzqB;
    PasosIzqB -= uni;
    
    EnviarCaracter(dmil+48);
    EnviarCaracter(mil+48);
    EnviarCaracter(cen+48);
    EnviarCaracter(dec+48);
    EnviarCaracter(uni+48);    
}

//Envía en qué sentido están girando las llantas
void EnviarSentidos(void){
    
    EnviarCadena("AT+CIPSEND=0,3\r\n",16);
    Enviando = 1;
    while(Enviando == 1){
    }
   
    EnviarCaracter(SentidoActualDer);
	EnviarCaracter(' ');
    EnviarCaracter(SentidoActualIzq);
}

void EnviarTiempos(void){
    
    EnviarCadena("AT+CIPSEND=0,10\r\n",17);
    Enviando = 1;
    while(Enviando == 1){
    }
    
    PromTiempos = 0;
    
    for(j = 0; j <6; j++){
        PromTiempos += TiemposPasosDer[j];
    }
    
    PromTiempos /= 6;
    
    dmil = PromTiempos/10000;
    PromTiempos -= dmil*10000;
    mil = PromTiempos/1000;
    PromTiempos -= mil*1000;
    cen = PromTiempos/100;
    PromTiempos -= cen*100;
    dec = PromTiempos/10;
    PromTiempos -= dec*10;
    uni = PromTiempos;
    PromTiempos -= uni;
    
    EnviarCaracter(dmil+48);
    EnviarCaracter(mil+48);
    EnviarCaracter(cen+48);
    EnviarCaracter(dec+48);
    EnviarCaracter(uni+48);
    
    for(j = 0; j <6; j++){
        PromTiempos += TiemposPasosIzq[j];
    }
    
    PromTiempos /= 6;
    
    dmil = PromTiempos/10000;
    PromTiempos -= dmil*10000;
    mil = PromTiempos/1000;
    PromTiempos -= mil*1000;
    cen = PromTiempos/100;
    PromTiempos -= cen*100;
    dec = PromTiempos/10;
    PromTiempos -= dec*10;
    uni = PromTiempos;
    PromTiempos -= uni;
    
    EnviarCaracter(dmil+48);
    EnviarCaracter(mil+48);
    EnviarCaracter(cen+48);
    EnviarCaracter(dec+48);
    EnviarCaracter(uni+48);    
}

// Configuración para Inicializar WIFI ESP
void InicializarESP(void){
    
    LATASET = LED1;
    LATBSET = LED2;
    LATBSET = LED3; 
    EnviarCadena("AT+RST\r\n",8);                               //Resetea Módulo Wifi
    PasosESP = 1;
    contR = 0;
    InitESP = 0;
    
    while(InitESP == 0){
        if(Revision == 1){
            switch(PasosESP){            
                case 1:
                    if(smsR[0]=='r' && smsR[1]=='e' && smsR[2]=='a' && smsR[3]=='d' && smsR[4]=='y'){
                        smsR[0] = '@';
                        Revision = 0;
                        EnviarCadena("ATE0\r\n",6);                 //Desactivar echo de serial
                        PasosESP++;
                    }
                    break;

                case 2:
                    if(smsR[0]=='O' && smsR[1]=='K'){
                        smsR[0] = '@';
                        Revision = 0;
                        EnviarCadena("AT+CWMODE_CUR=1\r\n",17);     //Modo cliente en comunicación WIFI
                        PasosESP++;
                    }
                    break;

                case 3:
                    if(smsR[0]=='O' && smsR[1]=='K'){
                        smsR[0] = '@';
                        Revision = 0;
                        EnviarCadena("AT+CWJAP_CUR=\"",14);         //Conectarse a la red
                        EnviarCadena(Usuario,LargoUs);
                        EnviarCadena("\",\"",3);
                        EnviarCadena(Contrasena,LargoCn);
                        EnviarCadena("\"\r\n",3);
                        PasosESP++;
                    }
                    break;

                case 4:
                    if(smsR[0]=='O' && smsR[1]=='K'){
                        smsR[0] = '@';
                        Revision = 0;
                        EnviarCadena("AT+CIPMUX=1\r\n",13);            //Multiple Conection
                        PasosESP++;
                    }
                    break;
                case 5:
                    if(smsR[0]=='O' && smsR[1]=='K'){
                        smsR[0] = '@';
                        Revision = 0;
                        EnviarCadena("AT+CIPSERVER=1,",15);            //Iniciar server en el puerto designado
                        EnviarCadena(Puerto, LargoPt);
                        EnviarCadena("\r\n",2);
                        PasosESP++;
                    }
                    break;
                case 6:
                    if(smsR[0]=='O' && smsR[1]=='K'){
                        smsR[0] = '@';
                        Revision = 0;
                        InitESP = 1;
                        PasosESP = 0;
                        Rutina = 1;
                    }
                default:
                    break;
            }
        }
    }
}