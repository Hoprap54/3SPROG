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



// Optocoupler 1 (Motor 1) Definitions
#define OPTO_PIN1 PB1  // D9 corresponds to pin PB1
#define WHEEL_CIRCUMFERENCE1 0.06754 // In meters
#define SEGMENTS_PER_REVOLUTION1 12 // The amount of holes in the wheel

// Optocoupler 2 (Motor 2) Definitions
#define OPTO_PIN2 PC0  // A0 corresponds to pin PC0
#define WHEEL_CIRCUMFERENCE2 0.06754
#define SEGMENTS_PER_REVOLUTION2 12

#define DEBOUNCE_TIME 10 // 10 milliseconds debounce time



/*
Custom delay function, 
that uses a while loop to create a delay in milliseconds.
*/

void delayMilliseconds(int ms) {
    while (ms--) { // ms decrements by one.
        _delay_ms(1); // Delay by 1 millisecond
    }
}

void initOptoPins() {
    DDRB &= ~(1 << OPTO_PIN1); // Set bit corresponding to OPTO_PIN1 to 0 (input).
    PORTB |= (1 << OPTO_PIN1); // Turn on the pull-up resistor for bit corresponding to OPTO_PIN1.

    DDRC &= ~(1 << OPTO_PIN2); // Set bit corresponding to OPTO_PIN2 to 0 (input).
    PORTC |= (1 << OPTO_PIN2); // Turn on the pull-up resistor for bit corresponding to OPTO_PIN2.
}



/*
Function takes two parameters. The pointer is to make the function more versatile, 
meaning it can be used with more ports (PORTB and PORTC).
*/
int readOptoPinWithDebounce(uint8_t pin, volatile uint8_t *port) {
    /*
    Isolates the bit in '*port', which corresponds to 'pin'.
    if the bit is 1 (HIGH state), the expression evaluates to a 1 value (true).
    if the bit is 0 (LOW state), the expression evaluates to a 0 value (false).
    the ternary operator is used to check the result.
    If its true (1) then the stablestate is set to 1.
    If its false (0) then the stablestate is set to 0.
    */
    int stableState = (*port & (1 << pin)) ? 1 : 0; // Used to eliminate false triggers, using debouncing.
    delayMilliseconds(DEBOUNCE_TIME); // a Sensors  state like an optocoupler, can change rapidly, meaning the signal can "bounce". "bouncing" means that the signal can fluctuate between high and low states, before settling.
    int currentState = (*port & (1 << pin)) ? 1 : 0; // if both stablestate and currentstate is the same, then we have a reliable HIGH or LOW state
    return (currentState == stableState) ? currentState : -1; // checks if both states are the same. Uses ternary operator to use if-else statement. if true, return currentstate, if false return -1.
}



/* 
The function is used to calculate the distance and speed, and to also print it. 
We use a total of 5 parameters. Pulsecount and segmentsperrevolution are used to calculate how much the wheel has been spun.
Calculate the distance the robot has gone, by multiplying how much the wheel has spun with the wheels circumference.
The speed formula is speed = distance * time, but considering the microcontroller usually takes time in milliseconds, 
we have to divide the time we got with 1000.
We use motorlabel to see what motor we are currently finding the distance and speed of.
*/
void calculateAndPrintSpeed(uint16_t pulseCount, uint32_t timeElapsed, float wheelCircumference, uint8_t segmentsPerRevolution, const char* motorLabel) {
    float revolutions = (float)pulseCount / segmentsPerRevolution; // Finding the amount the wheel has spun.
    float distance = revolutions * wheelCircumference; // Finding the distance the robot has traveled. 
    float speed = distance / (timeElapsed / 1000.0); // Finding the current speed of the robot.
    printf("%s Pulse Count: %u\n", motorLabel, pulseCount); // Prints the motor being used, and the amount of pulsecounts detected.
    printf("%s Speed: %f m/s\n", motorLabel, speed); // Prints the motor being used, and the speed of that motor.
}




int main() {
    uart_init(); // Open the communication to the microcontroller.
	io_redirect(); // Redirect input and output to the communication.
    initOptoPins(); // Initialize pins used for optocoupler.

    uint16_t pulseCount1 = 0, pulseCount2 = 0; // Used to count the number of pulses detected, using an unsigned integer 16 bits data specifier.
    uint32_t timeElapsed = 0; // Used to see the amount of time in milliseconds that has past.
    int lastOptoState1 = 1, lastOptoState2 = 1; // used to detect the changes in optocoupler signals. Assumes the optocoupler starts in a non-triggered state.

    while (1) {
        int currentOptoState1 = readOptoPinWithDebounce(OPTO_PIN1, &PINB); // Reads the state of the optocoupler, for when its connected to OPTO_PIN1 on port.
        /*
        Here we have a pulse detection logic.
        The if statement checks two conditions.
        Checks what the returned value is in the variable currentoptostate.
        - if currentoptostate is -1, then the condition is true, 
          but the logic NOT operator inverts it into a false, making it skip the if statement.
        - if currentoptostate is 1 or 0, then the condition is false, which makes the NOT operator,
          make it into a true condition.
        Checks if a high-to-low transition has happened which indicates a pulse detection.
        - lastOptoState1 == 1, checks if the current state of the lastoptostate1 is 1.
        - currentOptoState1 == 0, checks if the current state of the currentoptostate1 is 0.
        - if both conditions are true, then a high-to-low transition has happened.
        */
        if(currentOptoState1 != -1 && (lastOptoState1 == 1 && currentOptoState1 == 0)) {
            pulseCount1++; // Pulsecount gets incremented by 1, meaning a pulsecount has been detected.
        }
        lastOptoState1 = currentOptoState1; // Updates the 'lastoptostate1' with the current value from 'currentoptostate1'. obstructed = LOW = 0 and unobstructed = HIGH = 1.

        // Doing the same thing, for the second optocoupler.
        int currentOptoState2 = readOptoPinWithDebounce(OPTO_PIN2, &PINC);
        if(currentOptoState2 != -1 && (lastOptoState2 == 1 && currentOptoState2 == 0)) {
            pulseCount2++;
        }
        lastOptoState2 = currentOptoState2; 

        delayMilliseconds(100); // Creating a 100 milliseconds delay in the loop.
        timeElapsed += 100; // Increments the timeelapsed by 100 milliseconds.

        // Inside the main loop
        if (timeElapsed >= 1000) { // When a second has passed, this if statement will start.
        calculateAndPrintSpeed(pulseCount1, timeElapsed, WHEEL_CIRCUMFERENCE1, SEGMENTS_PER_REVOLUTION1, "Motor 1"); // Sends all the needed values to calculate the distance and speed and says what motor we are using..
        calculateAndPrintSpeed(pulseCount2, timeElapsed, WHEEL_CIRCUMFERENCE2, SEGMENTS_PER_REVOLUTION2, "Motor 2"); 
         pulseCount1 = 0; // Resets the pulsecount for the next cycle.
         pulseCount2 = 0;
         timeElapsed = 0; // Resets the time for the next cycle.
}
    }

    return 0;
}
