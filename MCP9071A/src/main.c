#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

#define ADC_PIN0 0

uint16_t adc_read(uint8_t adc_channel);
float tempCalc(void);

unsigned int adcval;
const float vout0 = 400; // Output voltage at 0ºC is 400mV *Given in datasheet
const float tc = 19.5;   // mv/ºC *Given in datasheet
float ta = 20;     // ambient temperature 20ºC
float vout_avg, vout;

int main(void){
    uart_init();
    io_redirect();
    ADMUX = (1<<REFS0); // Select Vref = AVcc
    //set prescaler to 128 and turn on the ADC module
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
    
    while(1){
        tempCalc();
    }
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0x00;        // Clear any previously used channel
    ADMUX |= adc_channel; // Set the desired channel
    ADCSRA |= (1<<ADSC);  // Start a conversion
    while ((ADCSRA & (1<<ADSC))); // Wait for the conversion to complete
    return ADC;
}

float tempCalc(void){
    vout_avg = 0;
    for(int i = 0; i < 1023; i++){
        vout = adc_read(ADC_PIN0) * (5000/1024.0);
        vout_avg = vout_avg + vout;
    }
    vout = vout_avg/1024;
    ta = (vout - vout0) / tc;

    printf("Temp:%fºC   V:%fv\n", ta, vout);
    _delay_ms(1000);
    return 0;
} 