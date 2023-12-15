#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

// Sensor transfer function -> Vout = T_C * T_A + V_0ºC *Given in datasheet
#define ADC_PIN0 0

uint16_t adc_read(uint8_t adc_channel);
float tempCalc(void);

const float v_0 = 400;   // Output voltage at 0ºC is 400mV *Given in datasheet
const float tc = 19.5;   // mv/ºC *Given in datasheet
float ta = 20;  // Ambient temperature 20ºC
float vout;
int value;

int main(void){
    uart_init();
    io_redirect();

    DDRC &= (1<<DDC0);

    ADMUX = (1<<REFS0) | (1<<ADLAR); // Select Vref = AVcc and Shift the result to the left
    // Set prescaler to 128 and turn on the ADC module
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

    while(1){
        tempCalc();
        _delay_ms(1000);
    }
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX |= adc_channel;
    ADCSRA |= (1<<ADSC);         // Start a conversion
    while((ADCSRA & (1<<ADSC))); // Wait for the conversion to complete
    
    value = ADCH;
    printf("ADCH:%d ", value);
	return(value); 
}

float tempCalc(void){
    vout = adc_read(ADC_PIN0) * (5000.0/1024);
    ta = (vout - v_0) / tc;

    printf("Temp:%.2f ºC   V:%.2fmV\n", ta, vout);
    _delay_ms(1000);
    return 0;
}