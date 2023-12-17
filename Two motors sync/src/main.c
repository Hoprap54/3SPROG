#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"

#define ENA DDB3
#define IN1 PB5
#define IN2 PB0
#define IN3 DD7
#define IN4 PB4
#define ENB DDB2

void PWM_MotorR(unsigned char);
void PWM_MotorL(unsigned char duty);

int main(void){
    uart_init();
    /* I/O */
    DDRD |= (1<<IN3);
    DDRB |= (1<<IN1) | (1<<IN2) | (1<<IN4) | (1<<ENA) | (1<<ENB);

    /* Set a direction in the motors */
    PORTD |= (1<<IN3);
    PORTB |= (1<<IN1) ;
    PORTB &= ~(1<<IN2) | ~(1<<IN4);

    while (1){
        // PWM_MotorR(100);     // Select speed from 1-255. + number + speed
        PWM_MotorR(100);     // Select speed from 1-255. + number + speed
        PWM_MotorL(250);     // Select speed from 1-255. + number + speed
    }
}

/*TIMER1*/
void PWM_MotorL(unsigned char duty){
    TCCR1A |= (1<<WGM11) | (1<<WGM10);  // Fast PWM configuration
    TCCR1B |= (1<<WGM13) | (1<<WGM12);
    TCCR1A |= (1<<COM1B1);              // Non inverting output

    TCCR1B |= (1<<CS12) | (1<<CS10);    // 1024 Prescaler
    OCR1B = duty; // Set the value to be compared
}

/*TIMER2*/
void PWM_MotorR(unsigned char duty){ 
    TCCR2A |= (1<<WGM21) | (1<<WGM20);  // Fast PWM configuration
    TCCR2A |= (1<<COM2A1);              // Non inverting output
    TCCR2B |= (1<<WGM22);

    TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20);    // 1024 Prescaler
    OCR2A = duty; // Set the value to be compared
}