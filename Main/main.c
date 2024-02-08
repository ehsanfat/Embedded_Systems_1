// DSPIC30F4011 Configuration Bit Settings
// 'C' source line config statements
// FOSC
#pragma config FPR = XT          // Primary Oscillator Mode (XTL)
#pragma config FOS = FRC           // Oscillator Source (Internal Fast RC)
#pragma config FCKSMEN = CSW_FSCM_OFF// Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16   // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512  // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF       // Watchdog Timer (Enabled)

// FBORPOR
#pragma config FPWRT = PWRT_64  // POR Timer Value (64ms)
#pragma config BODENV = BORV20  // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON  // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI// Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI// High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN// PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN  // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF      // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF  // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD       // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
/*
 * File:   main.c
 * Author:  Ehsan Fatemi 
 *
 * Created on October 3, 2023, 12:33 PM
 */

//Include the essential libraries
#include "xc.h"
#include "p30F4011.h"
#include <stdio.h>
#include <string.h>

// Peripheral Settings
#define LED_PIN LATBbits.LATB0
#define LED_PIN2 LATBbits.LATB1
#define BUTTON_PIN PORTDbits.RD0

// Timer Settings 
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3

// Clock Settings
#define CRYSTALFREQUENCY 7372800

// UART Settings
#define UART_BUFFER_SIZE 16
char uartReadBuffer[UART_BUFFER_SIZE];
char uartWRiteBuffer[UART_BUFFER_SIZE];
volatile uint8_t UART_ReadBufferIndex = 0;
char charRecv;
int charRecvNum=0;
int uartRead_allowed = 0;
int uartWrite_allowed = 0;
int bufferClean_allowed = 0;

// LCD Settings
#define cursorStart1  0x80
#define cursorEnd1  0x8F
#define cursorStart2  0xC0
#define cursorEnd2  0xCF

char temp[20];

// Functions Declaration

// Configuration Functions
void peripheralConfig(void);
void spiConfig(void);
void interruptConfig(void);
void uartConfig(void);

// Timer Functions
void tmr_setup_period(int timer, int ms);
void tmr_wait_period(int timer);
void tmr_wait_ms(int timer, int ms);

// LCD Functions
void lcdWrite(const char *str, int writeAddress);
void lcdUpdate();

// UART Functions
void bufferEmpty();
void uartRead(void);
void uartWrite();

// User Functions
void algorithm();

int main(void) {
    // Configure the peripherals
    peripheralConfig();
    spiConfig();
    uartConfig();
    interruptConfig();
    
     // Wait 1000 milliseconds to allow the LCD to begin normal operation
    tmr_wait_ms(TIMER1, 1000);
    
    // Configure Timer2 to adjust the frequency of while loop to 100 Hz
    tmr_setup_period(TIMER1, 10);
    
    // Configure Timer3 for handling the bouncing issue
    tmr_setup_period(TIMER3, 200);
    T3CONbits.TON = 0; // disable timer
   
    while(1){
           // Run the Algorithm (It takes 7 ms to execute the algorithm)
            algorithm();
           // Visualize Data
            lcdUpdate();
           // Adjust the frequency of while loop to 100 Hz (Waits for 3ms)
            tmr_wait_period(TIMER1); 
            
            // Allow to write on UART
            if(uartWrite_allowed){
                uartWrite();
                uartWrite_allowed = 0;
            }
            
            // Allow to clean the UART buffer
            if(bufferClean_allowed){
              bufferEmpty();
              charRecvNum = 0;
              bufferClean_allowed = 0;
            }
    }
    return 0;
}

// User Function
void algorithm(){
   tmr_wait_ms(TIMER2, 7);
}

// Function to configure LED & Push button
void peripheralConfig(void){
    TRISBbits.TRISB0 = 0;  // Output for LED
    TRISBbits.TRISB1 = 0;  // Output for LED
    TRISDbits.TRISD0 = 1;  // Input for BUTTON
}

// Function to configure SPI
void spiConfig(void){
     // SPI configuration
    SPI1CONbits.MSTEN = 1; // master mode
    SPI1CONbits.MODE16 = 0; // 8?bit mode
    SPI1CONbits.PPRE = 3; // 1:1 primary prescaler
    SPI1CONbits.SPRE = 3; // 5:1 secondary prescaler
    SPI1STATbits.SPIEN = 1; // enable SPI 
}

// Function to configure Interrupts
void interruptConfig(void){
    IEC0bits.INT0IE = 1; // enable INT0 interrupt!
    IEC1bits.INT1IE = 1; // enable INT1 interrupt!
    
    IEC0bits.T3IE = 1; // enable the Timer3 interrupt!
    
    U2MODEbits.UARTEN = 1; //Enable UART 2 interrupt
    U2STAbits.URXISEL = 0b00;
    IFS1bits.U2RXIF = 0; 
    IEC1bits.U2RXIE = 1;
}

// Function to configure UART2
void uartConfig(void)
{
    U2BRG = 11; // (7372800 / 4) / (16 * 9600) ? 1
    U2MODEbits.UARTEN = 1; // enable UART
    U2STAbits.UTXEN = 1;
}

// Function to setup timer 1/2/3 for periodic wait
void tmr_setup_period(int timer, int ms)
{
    uint32_t tcount = (CRYSTALFREQUENCY/4/256)*(ms/1000.0) -1; // fill the PR1 register with the proper number of clocks
    
    if(timer == TIMER1)
    {
        TMR1 = 0; // reset timer counter
        PR1 = tcount;
        
        T1CONbits.TCKPS = 0b11; // set the prescaler to 8 
        
        T1CONbits.TON = 1; // starts the timer! 
    }
    
    if(timer == TIMER2)
    {
        TMR2 = 0; // reset timer counter
        PR2 = tcount;
        
        T2CONbits.TCKPS = 0b11; // set the prescaler to 8 
        T2CONbits.TON = 1; // starts the timer!       
    }
    
    if(timer == TIMER3)
    {
        TMR3 = 0; // reset timer counter
        PR3 = tcount;
        
        T3CONbits.TCKPS = 0b11; // set the prescaler to 8 
        T3CONbits.TON = 1; // starts the timer!       
    }
}

// Function to setup timer 1/2/3 for periodic wait
void tmr_wait_period(int timer)
{
    if(timer == TIMER1)
    {
       while(!IFS0bits.T1IF); 
       IFS0bits.T1IF = 0;
    }
       
    if(timer == TIMER2)
    {
       while(!IFS0bits.T2IF); 
       IFS0bits.T2IF = 0;
    } 
    
    if(timer == TIMER3)
    {
       while(!IFS0bits.T3IF); 
       IFS0bits.T3IF = 0;
    }  
}

// Function to setup timer 1/2 to waits for ms
void tmr_wait_ms(int timer, int ms)
{
    tmr_setup_period(timer, ms);
    tmr_wait_period(timer);
}

// Function to write a string on the LCD
void lcdWrite(const char *str, int writeAddress) {
    //Write uartReadBuffer
    SPI1BUF = writeAddress; 
    tmr_wait_ms(TIMER1, 1);
    while (*str) {
        //Write the character
        SPI1BUF = *str; // send the ?x? character
        while(SPI1STATbits.SPITBF == 1); // wait until not full 
        str++;
    }
}

// Function to write all the user's data on the LCD
void lcdUpdate(){
  lcdWrite(uartReadBuffer, cursorStart1);
  strcpy(uartWRiteBuffer,"Char Recv:");
  sprintf(temp, "%d", charRecvNum);
  strcat(uartWRiteBuffer,temp);
  strcat(uartWRiteBuffer, "  ");
  lcdWrite(uartWRiteBuffer, cursorStart2);
}

// Function to clear the LCD
void bufferEmpty(){
    strcpy(uartReadBuffer, "                ");
    UART_ReadBufferIndex = 0;
}

// Function to read from UART
void uartRead(void) {
    if (U2STAbits.URXDA) {
        char receivedChar = U2RXREG;

        // Check for buffer overflow
        if (UART_ReadBufferIndex + 1 <= UART_BUFFER_SIZE) {
            // If special character received
            if(receivedChar == '\r' || receivedChar == '\n'){
                UART_ReadBufferIndex = 0;
                bufferEmpty(); 
            }else{
                // If normal character received
                uartReadBuffer[UART_ReadBufferIndex] = receivedChar; // Save the character in the buffr
                UART_ReadBufferIndex = UART_ReadBufferIndex + 1; // Increase the buffer's index
                charRecvNum = charRecvNum + 1; // If normal character received
            }
        }else{
            // If buffer overflow happened
            bufferEmpty(); // Clear the buffer
            UART_ReadBufferIndex = 0; // Reset the buffer's index
            uartReadBuffer[UART_ReadBufferIndex] = receivedChar; // Save the 17th character in the buffer
            UART_ReadBufferIndex = UART_ReadBufferIndex + 1; // Increase the buffer's index
            charRecvNum = charRecvNum + 1; // If normal character received
        }
    }
}

// Function to write on UART
void uartWrite(){
    while(!U1STAbits.TRMT); // Waits till ongoing transmission finished
    
    // Now we can write the new data
    char temp[5];
    sprintf(temp, "%d", charRecvNum);
    for(int i=0;i<strlen(temp);i++){
        char c = temp[i];
        if(c !='\0')
           U2TXREG = c;
    }
}

// Interrupt Service Routine (ISR) for INT0
void __attribute__ ((__interrupt__ , __auto_psv__)) _INT0Interrupt(){
    IEC0bits.INT0IE = 0; // disable interrupt button
    T3CONbits.TON = 1;   // enable the timer 2 for handling the bouncing
}

// Interrupt Service Routine (ISR) for INT1
void __attribute__ ((__interrupt__ , __auto_psv__)) _INT1Interrupt(){
    
    IEC1bits.INT1IE = 0; // reset interrupt flag
    bufferClean_allowed = 1;
    IFS1bits.INT1IF = 0; // reset interrupt flag
    IEC1bits.INT1IE = 1; 
}

// Interrupt Service Routine (ISR) for UART_READ
void __attribute__ ((__interrupt__ , __auto_psv__)) _U2RXInterrupt(){
    IEC1bits.U2RXIE = 0;
    uartRead();
    IFS1bits.U2RXIF = 0; // // reset interrupt flag
    IEC1bits.U2RXIE = 1;
}

// Interrupt Service Routine (ISR) for handling the bounce of S5 push button
void __attribute__ ((__interrupt__ , __auto_psv__)) _T3Interrupt() {
    
    IFS0bits.T3IF = 0;
            
    // Check button status
    if(PORTDbits.RD0 != 0){
        // Activate the process
        uartWrite_allowed = 1;
    }
    T3CONbits.TON = 0; // Disable timer
    TMR3 = 0; // Reset counter for the timer because we don't know when this interrupt happens
    IFS0bits.INT0IF = 0;
    IEC0bits.INT0IE = 1; // Enable interrupt firing for button
}

