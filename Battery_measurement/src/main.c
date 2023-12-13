#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
/* Voltage didiver
*   R1: 2.7k
*   R2: 1k
*/
#define ADC_PIN0 6

uint16_t adc_read(uint8_t adc_channel);
float voltagecalc(void);

unsigned int adcval;
int digitalVolt = 0;
float Volt, totalvolt;

int main(void){
    uart_init();
    io_redirect();
    ADMUX = (1<<REFS0); // Select Vref = AVcc
    //set prescaler to 128 and turn on the ADC module
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
    
    while(1){
        printf("%f\n", totalvolt);
    }
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0x00;        // Clear any previously used channel
    ADMUX |= adc_channel; // set the desired channel
    ADCSRA |= (1<<ADSC);  // Start a conversion
    while ( (ADCSRA & (1<<ADSC))); // Wait for the conversion to complete
    return ADC;
}

float voltagecalc(void){
    digitalVolt = adc_read(ADC_PIN0);
    Volt = (float)digitalVolt/1024*11.98;
    totalvolt = Volt/3.7;
    return totalvolt;
}