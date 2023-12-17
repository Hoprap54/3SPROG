/*
 * Optocoupler.c
 *
 * Created: 13/12/2023
 * Author : Ali
 */ 

#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

/*
// Optocoupler 1 (Motor 1) Definitions
#define OPTO_PIN1 PB1  // D9 corresponds to pin PB1
#define WHEEL_CIRCUMFERENCE1 0.06754 // Example in meters
#define SEGMENTS_PER_REVOLUTION1 12

// Optocoupler 2 (Motor 2) Definitions
#define OPTO_PIN2 PC6 // A0 corresponds to pin PC0
#define WHEEL_CIRCUMFERENCE2 0.06754
#define SEGMENTS_PER_REVOLUTION2 12

#define DEBOUNCE_TIME 10 // 10 milliseconds debounce time

void delayMilliseconds(int ms) {
    while (ms--) {
        _delay_ms(1);
    }
}

void initOptoPins() {
    DDRB &= ~(1 << OPTO_PIN1); // Set OPTO_PIN1 as input with pull-up resistor
    PORTB |= (1 << OPTO_PIN1);

    DDRC &= ~(1 << OPTO_PIN2); // Set OPTO_PIN2 as input with pull-up resistor
    PORTC |= (1 << OPTO_PIN2); 
}

int readOptoPinWithDebounce(uint8_t pin, volatile uint8_t *port) {
    int stableState = (*port & (1 << pin)) ? 1 : 0;
    delayMilliseconds(DEBOUNCE_TIME);
    int currentState = (*port & (1 << pin)) ? 1 : 0;
    return (currentState == stableState) ? currentState : -1;
}

void calculateAndPrintSpeed(uint16_t pulseCount, uint32_t timeElapsed, float wheelCircumference, uint8_t segmentsPerRevolution, const char* motorLabel) {
    float revolutions = (float)pulseCount / segmentsPerRevolution;
    float distance = revolutions * wheelCircumference;
    float speed = distance / (timeElapsed / 1000.0); 
    printf("%s Pulse Count: %u\n", motorLabel, pulseCount);
    printf("%s Speed: %f m/s\n", motorLabel, speed);
}

int main() {
    uart_init();
    io_redirect();
    initOptoPins();

    uint16_t pulseCount1 = 0, pulseCount2 = 0;
    uint32_t timeElapsed = 0;
    int lastOptoState1 = 1, lastOptoState2 = 1;

    while (1) {
        int currentOptoState1 = readOptoPinWithDebounce(OPTO_PIN1, &PINB);
        if(currentOptoState1 != -1 && (lastOptoState1 == 1 && currentOptoState1 == 0)) {
            pulseCount1++;
        }
        lastOptoState1 = currentOptoState1;

        int currentOptoState2 = readOptoPinWithDebounce(OPTO_PIN2, &PINC);
        if(currentOptoState2 != -1 && (lastOptoState2 == 1 && currentOptoState2 == 0)) {
            pulseCount2++;
        }
        lastOptoState2 = currentOptoState2;

        delayMilliseconds(100); 
        timeElapsed += 100;

        // Inside the main loop
        if (timeElapsed >= 1000) {
        calculateAndPrintSpeed(pulseCount1, timeElapsed, WHEEL_CIRCUMFERENCE1, SEGMENTS_PER_REVOLUTION1, "Motor 1");
        calculateAndPrintSpeed(pulseCount2, timeElapsed, WHEEL_CIRCUMFERENCE2, SEGMENTS_PER_REVOLUTION2, "Motor 2");
         pulseCount1 = 0;
         pulseCount2 = 0;
         timeElapsed = 0;
}



        if (timeElapsed >= 1000) {
            calculateAndPrintSpeed(pulseCount1, timeElapsed, WHEEL_CIRCUMFERENCE1, SEGMENTS_PER_REVOLUTION1);
            calculateAndPrintSpeed(pulseCount2, timeElapsed, WHEEL_CIRCUMFERENCE2, SEGMENTS_PER_REVOLUTION2);
            pulseCount1 = 0;
            pulseCount2 = 0;
            timeElapsed = 0;
        }
        
    }

    return 0;
}*/

void PWM_MotorR(unsigned char);
void PWM_MotorL(unsigned char duty);

int main(void){
    uart_init();
    while (1){
        PWM_MotorR(100);     // Select speed from 1-255. + number + speed
        PWM_MotorL(100);     // Select speed from 1-255. + number + speed
    }
}

/*TIMER0*/
inline void PWM_MotorR(unsigned char duty){ 
    DDRD |= (1 << PB2);     // Declare this ENA pin as output     
    TCCR0A |= (1<<WGM01) | (1<<WGM00);  // Fast PWM configuration
    TCCR0A |= (1<<COM0A1);              // Non inverting output

    TCCR0B |= (1<<CS02) | (1<<CS00);    // 1024 Prescaler
    OCR0A = duty; // Set the value to be compared
}

/*TIMER1*/
void PWM_MotorL(unsigned char duty){
    DDRB |= (1 << PB3);     // Declare this ENA pin as output     

    TCCR1A |= (1<<WGM11) | (1<<WGM10);  // Fast PWM configuration
    TCCR1B |= (1<<WGM13) | (1<<WGM12);
    TCCR1A |= (1<<COM1A1);              // Non inverting output

    TCCR1B |= (1<<CS12) | (1<<CS10);    // 1024 Prescaler
    OCR1A = duty; // Set the value to be compared
}