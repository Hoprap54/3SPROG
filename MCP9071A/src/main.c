#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"
// Sensor transfer function -> Vout = T_C * T_A + V_0ºC *Given in datasheet
#define ADC_PIN0 0

uint16_t adc_read(uint8_t adc_channel);
float tempCalc(void);

unsigned int adcval;
const float v_0 = 400;   // Output voltage at 0ºC is 400mV *Given in datasheet
const float tc = 19.5;   // mv/ºC *Given in datasheet
float ta = 20;  // Ambient temperature 20ºC
float vout;

int main(void){
    uart_init();
    io_redirect();
    ADMUX = (1<<REFS0); // Select Vref = AVcc
    // Set prescaler to 128 and turn on the ADC module
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
    
    while(1){
        vout = adc_read(ADC_PIN0) * (5000/1023);
        ta = (vout - v_0) / tc;

        printf("Temp:%fºC   V:%fmV\n", ta, vout);
        _delay_ms(1000);
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
    vout = adc_read(ADC_PIN0) * (5000/1023);
    ta = (vout - v_0) / tc;

    printf("Temp:%fºC   V:%fmV\n", ta, vout);
    _delay_ms(1000);
    return 0;
}