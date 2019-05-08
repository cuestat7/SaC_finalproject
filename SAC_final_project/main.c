#include <msp430.h> 
#include <stdint.h>

volatile float Volttemp;
volatile float Desiredvolt;
volatile float Desiredtemp;
volatile float Currenttemp;
volatile float Error;
volatile float Lasterror;
volatile float Proportion;
volatile float Integral;
volatile float Derivative;
volatile float PID;
// Low pass filter function for PTAT input
volatile float Filtervolt;
float LPF_Beta = 0.95; // 0<ß<1
// PID coefficients
volatile float Kp = 25;
volatile float Ki = 0.0408;
volatile float Kd = 0.2416;
int uart_count = 0;
uint16_t displaytemp;


void configureUART()
{
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    P4SEL |= BIT5;                                       // Enables RX and TX buffer
    P4SEL |= BIT4;
    UCA1CTL1 |= UCSWRST;                      // Clears the UART control register 1
    UCA1CTL1 |= UCSSEL_2;                     // Sets SMCLK
    UCA1BR0 = 104;                            // For baud rate of 9600
    UCA1BR1 = 0;                              // For baud rate of 9600

    UCA1MCTL |= UCBRS_2;                      // set modulation pattern to high on bit 1 & 5
    UCA1CTL1 &= ~UCSWRST;                     // initialize USCI
    UCA1IE |= UCRXIE;                         // enable USCI_A1 RX interrupt
    UCA1IFG &= ~UCRXIFG;                      // clears interrupt flags
}

void configurePWM()
{
   // Sets P1.2 as the output pin
   P1DIR |= BIT2;                                        // Sets P1.2 as output driver for pwm for fan speed control
   P1SEL |= BIT2;                                        // Selects the port 1.2 as the timer A output
   TA0CTL = TASSEL_1 | MC_1 | TACLR;                     // Sets timerA_0 to SMCLK, up-mode, clears the register
   TA0CCR0 = 255;                                        // Sets CCR0 max pwm
   TA0CCR1 = 0;                                          // Sets CCR1 to initial value of 0% Duty Cycle
   TA0CCTL1 = OUTMOD_7;                                  // Output mode 7 reset/set
}


void configureADC()
{
    P6SEL = 0x1F;                             // Enable A/D channel inputs
    ADC12CTL0 = ADC12ON+ADC12MSC+ADC12SHT0_2; // Turn on ADC12, set sampling time
    ADC12CTL1 = ADC12SHP+ADC12CONSEQ_1;       // Use sampling timer, single sequence
    ADC12MCTL0 = ADC12INCH_0;                 // ref+=AVcc, channel = A0
    ADC12MCTL1 = ADC12INCH_1;                 // ref+=AVcc, channel = A1
    ADC12MCTL2 = ADC12INCH_2;                 // ref+=AVcc, channel = A2
    ADC12MCTL3 = ADC12INCH_3;                 // ref+=AVcc, channel = A3, end seq.
    ADC12MCTL4 = ADC12INCH_4 + ADC12EOS;
    ADC12IE = 0x10;                           // Enable ADC12IFG.4
    ADC12CTL0 |= ADC12ENC;                    // Enable conversions
}

int updatePWM(void)
{
           Filtervolt = Filtervolt - (LPF_Beta * (Filtervolt - ADC12MEM4));
           Desiredvolt = ADC12MEM2; // Set point voltage - not yet converted to Temperature in celcius
           //Filtervolt = ADC12MEM4;
           Desiredtemp = 10*((Desiredvolt/4096)*5);
           //Desiredtemp = 35;
           Currenttemp = 55*((Filtervolt/4096)*3.3);
           Error = Currenttemp - Desiredtemp;


          P1OUT ^= BIT0;
          Proportion = Kp * Error;
          Integral += Ki * (Error * 0.025);
          Derivative = Kd * ((Lasterror - Error)/0.025);

          PID = (Proportion + Integral + Derivative);


           if (PID > 255)  PID = 255;
           if (PID < 0)   PID = 0;
           else if (0 < PID < 255) PID = PID;

           uart_count += 1;
              if(uart_count%100 == 0)
              {
                  //displaytemp = Desiredtemp;
                  UCA1TXBUF = Desiredtemp;
              }
              else if(uart_count%50 == 0)
              {
                 // displaytemp = Currenttemp;
                  UCA1TXBUF = Currenttemp;
              }

           Lasterror = Error;
           return PID;
}


int main(void)
{
    //UCSCTL4 = SELA_0;                                    // Enables UART ACLK (32.768 kHz signal)
    WDTCTL = WDTPW | WDTHOLD;                            // Stop watchdog timer


    updatePWM();
    configureUART();
    configurePWM();
    configureADC();
while(1)
    {
            ADC12CTL0 |= ADC12SC;
            __bis_SR_register(GIE);              // Enables Global Interrupt - ADC/UART interrupt support
            __no_operation();

    }
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12ISR (void)
#else
#error Compiler not supported!
#endif
{
    TA0CCR1 = updatePWM();
}



