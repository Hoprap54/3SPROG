/*
 * UltrasonicSensor.c
 *
 * Created: 12/12/2023
 * Author : Ali
 */ 

#define F_CPU 16000000UL



// Ultrasonic sensor 1
#define TRIG_PIN PD5 // Assuming you're using pin D5 for Trig - Common trigpin for all ultrasonic sensor
#define ECHO_PIN1 PD2 // Assuming you're using pin D2 for Echo

// Ultrasonic sensor 2
#define ECHO_PIN2 PD3

// Ultrasonic sensor 3
#define ECHO_PIN3 PD4

// Ultrasonic sensor 4
#define ECHO_PIN4 PD6


//Header files
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "usart.h"


// Function to set trigpin to output and echopin to input
void initializeSensor() {
/*
    Create a bitmask by making the bit corresponding to TRIG_PIN (D5) into a 1 (output).
    Logic bitwise [|=] operator, is used to make sure only the bit corresponding
    to the TRIG_PIN (D5) gets set to 1.
    DDRD is to make sure the bitmask happens in PORT D REGISTER.
*/
    DDRD |= (1 << TRIG_PIN); // Set Trig pin as output
/*
    Create a bitmaske, by making corresponding bits to the ECHO_PINs, set to 1 (output).
    The bitwise OR operator is used to make sure all of ECHO_PINs are set to 1.
    The bitwise NOT operator is used to invert the entire bitmask, every 1 gets turned into 0 and every 0 gets turned into 1.
    The logic bitwise [&=] operator is used, to make sure the bitwise NOT operator does not affect the entire bitmask.
    The only bits that will change are the corresponding bits to the ECHO_pins.
*/
    DDRD &= ~((1 << ECHO_PIN1) | (1 << ECHO_PIN2) | (1 << ECHO_PIN3) | (1 << ECHO_PIN4)); // Set Echo pins as input
}





/*
    Creating a function named 'getduration'. The function returns a value of data type 'long'. 
    'uint8_t echopin' is the parameter for the function.
    it has a single argument named 'echopin', which has a data type of 'uint8_t' (unsigned integer of size 8 bits).
    'echopin' represents the pin number of the echo signal from the ultrasonic sensor, which will be read.
*/
long getduration(uint8_t echoPin) {

    // Variable is used to measure the duration of the echosignal.
    long count = 0;

    // Threshhold to make sure the program doesnt get stuck in an infinite loop. 
    // In essence it works as a timeout limit.
    const long maxCount = 30000; // Adjust as necessary


/*
    '1 << TRIG_PIN' creates a bitmask, by using the bitwise left shift operation.
    The bitmask created, where only the bit corresponding to TRIG_PIN, gets set to high,
    without changing the state of the other bits in the bitmask.
*/ 
    PORTD |= (1 << TRIG_PIN); // Send trigger pulse. Trigger the sensor - ON.
    _delay_us(10); //delay made to make sure eight pulses get sent out.
/*
    '1 << TRIG_PIN'. Startes from the right and goes to the left, all the way to bit, 
    corresponding to TRIG_PIN and sets its state to 1 (output).
    The bitwise NOT operator is used to invert the state of the bit to 0 (input).
    The compound assignment operator is used to make sure none of the other bits,
    except the bit corresponding to TRIG_PIN changes.
    It all happens in PORTD
*/
    PORTD &= ~(1 << TRIG_PIN); // turned OFF


/*
    we use the echopin to create a bitmask that will make the PIND focus on a specific bit, 
    specifically the bit currently corresponding to that echopin. 
    the PIND startes out in a LOW state also meaning 0, and while the echopin is set to 1, 
    the bitwise AND operator makes sure the statement inside gets set to 0 AND 0, 
    while the logic NOT operator turns it into a 1 AND 1, making the while loop true. 
    The while loops keeps running and it keeps checking whether PIND is LOW meaning 0 the moment it becomes HIGH meaning 1, 
    the statement inside becomes 1 AND 1 making whats inside true, 
    but the logic NOT operator turns it into a 0 AND 0 making the while loop false, and it exits.
*/
    // Wait for echo to start
    while(!(PIND & (1 << echoPin))) {
        //If the count gets higher than the maxcount, then the inside of the if statement gets printed. count will increment by 1, each time its run.
        if (count++ > maxCount) {
            printf("Timeout waiting for echo start on pin %d\n", echoPin); // Echopin, belongs to the sensor, which is calling the function.
            return -1; // Return -1 to indicate a timeout (no echo detected).
        }
    }

    count = 0; // Reset count for measuring the high time of the echo signal

    // Measure the length of the high signal
    while(PIND & (1 << echoPin)) {
        if (count++ > maxCount) {
            printf("Timeout waiting for echo end on pin %d\n", echoPin);
            return -1; // Timeout, echo too long
        }
    }

    return count;
}



// Calculate distance
double calculateDistance(long duration) {
    return duration * 0.034 / 2; // Speed of sound in cm/us
}



// Main function for ultrasonic sensor
void ultrasonicsensor() {
    while (1) {
        // Sensor 1
        long duration1 = getduration(ECHO_PIN1);
        double distance1 = calculateDistance(duration1);
        printf("Sensor 1 Duration: %ld, Distance: %.2f cm\n", duration1, distance1);

        _delay_ms(500); // Delay between sensor readings

        // Sensor 2
        long duration2 = getduration(ECHO_PIN2);
        double distance2 = calculateDistance(duration2);
        printf("Sensor 2 Duration: %ld, Distance: %.2f cm\n", duration2, distance2);

        _delay_ms(500); // Delay before the next set of measurements

        // Sensor 3
        long duration3 = getduration(ECHO_PIN3);
        double distance3 = calculateDistance(duration3);
        printf("Sensor 3 Duration: %ld, Distance: %.2f cm\n", duration3, distance3);

        _delay_ms(500); // Delay before the next set of measurements

        // Sensor 4
        long duration4 = getduration(ECHO_PIN4);
        double distance4 = calculateDistance(duration4);
        printf("Sensor 4 Duration: %ld, Distance: %.2f cm\n", duration4, distance4);

        _delay_ms(500); // Delay before the next set of measurements

    }
}



int main() {
    uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the communication
    initializeSensor();
    ultrasonicsensor();
    return 0;
}



