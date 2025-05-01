/*
 * FinalProject.c
 *
 * Created: 4/28/2025 3:55:00 PM
 * Author : Adrian Alvarez & Seth Bolen
 */

#define F_CPU 16000000UL  // 16 MHz clock
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <Ultrasonic.h>

// Define UART baud rate
#define BAUD 9600               // Baud rate for serial communication
#define MYUBRR F_CPU/16/BAUD-1  // Baud rate register calculation

#define RELAY_PIN1 PD5     // Relay control pin 1
#define BUTTON1_PIN PB3    // First pushbutton input (was relay before)
#define BUTTON2_PIN PB2    // Second pushbutton input (new pin)
#define RELAY_PIN2 PD4     // Relay control pin 2 (was button before)
#define LED_PIN    PB5     // Built-in "L" LED pin

#define TRIG_PIN PB4 // Trigger pin
#define ECHO_PIN PD2 // Echo pin

// global variables for sensor
uint8_t distance = 0;
uint8_t diagnostics = 0;
volatile uint16_t pulse;
volatile uint8_t iIRC = 0;
volatile int f_wdt = 1;

///////////////////////////////////////////////
// Setups everything
///////////////////////////////////////////////
void setup(void) {
    // Set relay pins and LED pin as outputs
    DDRD |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2);// 
	DDRB |= (1 << LED_PIN);

    // Set button pin as input
    DDRB &= ~(1 << BUTTON1_PIN);
	DDRB &= ~(1 << BUTTON2_PIN);

	// Initializes sensor
	init_ultrasonic();

    // Enable pull-up resistor on button pin
    PORTB |= (1 << BUTTON1_PIN);
	PORTB |= (1 << BUTTON2_PIN);

    // Turn on built-in LED steady
    PORTB |= (1 << LED_PIN);

    // Make sure relays are OFF at startup
    PORTD |= ~((1 << RELAY_PIN1) | (1 << RELAY_PIN2));	
}

///////////////////////////////////////////////
// For serial monitor
///////////////////////////////////////////////
void uart_init(unsigned int ubrr) {
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1<<TXEN0);      // Enable transmitter
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // 8-bit data
}

// Transmit a single character over UART
void uart_transmit(char data) {
	while (!(UCSR0A & (1 << UDRE0))); // Wait until transmit buffer is empty
	UDR0 = data;                      // Load data into the UART data register
}

// Send a string over UART
void uart_print(const char* str) {
	while (*str) {             // Loop through each character until null terminator
		uart_transmit(*str++); // Transmit current character and move to next
	}
}

// Convert a float to string and print over UART
void uart_print_float(float num) {
	char buffer[16];                         // Buffer to hold converted float string
	dtostrf(num, 5, 2, buffer);              // Convert float to string with 2 decimal places
	uart_print(buffer);                      // Send the resulting string over UART
}

///////////////////////////////////////////////
// Main loop
///////////////////////////////////////////////
void loop(void) {
    // Wait for button1 press
    if (!(PINB & (1 << BUTTON1_PIN))) {
        _delay_ms(50);  // debounce delay

        // Confirm button still pressed after debounce
        if (!(PINB & (1 << BUTTON1_PIN))) {

            // Turn ON relays immediately
			 PORTD &= ~((1 << RELAY_PIN1));
        

            // Wait for button release before accepting another press
            while (!(PINB & (1 << BUTTON1_PIN))) {
                _delay_ms(10);
            }

            _delay_ms(50); // optional debounce after release
        }
    }
	
	// Wait for button2 press
	if (!(PINB & (1 << BUTTON2_PIN))) {
		 _delay_ms(50);  // debounce delay

		 // Confirm button still pressed after debounce
		 if (!(PINB & (1 << BUTTON2_PIN))) {
			 PORTD &= ~((1 << RELAY_PIN2));
			 
			 // Wait for button release before accepting another press
			 while (!(PINB & (1 << BUTTON2_PIN))) {
				 _delay_ms(10);
			 }
			 _delay_ms(50); // optional debounce after release
		 }
	}
	
	// for sensor
	distance = getDistance_main(&diagnostics);
	uart_print("Distance: ");      // Print label
	uart_print_float(distance);    // Print measured distance
	uart_print(" cm\r\n");         // Print units and newline
	_delay_ms(100);                // Wait 100 ms before next reading

	PORTD |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2); // Make relays go high (turns off relay) (MAYBE, LOOKUP)
}

///////////////////////////////////////////////
// Stuff for sensor, look at Github
///////////////////////////////////////////////
ISR(INT0_vect)
{
	switch (iIRC)
	{
		case 0: //when logic changes from LOW to HIGH
		{
			iIRC = 1;
			TCCR1B |= (1<<CS11);
			break;
		}
		case 1:
		{
			/* reset iIRC */
			iIRC = 0;
			/* stop counter */
			TCCR1B &= ~(1<<CS11);
			/* assign counter value to pulse */
			pulse = TCNT1;
			/* reset counter */
			TCNT1=0;
			break;
		}
	}
}

/* *****************************************************************
Name:		Watchdog Interrupt
Inputs:		none
Outputs:	f_wdt
Description:wakes up processor after internal timer limit reached (8 sec)
******************************************************************** */
ISR(WDT_vect)
{
	/* set the flag. */
	if(f_wdt == 0)
	{
		f_wdt = 1;
	}
	//else there is an error -> flag was not cleared
}

///////////////////////////////////////////////
// Main
///////////////////////////////////////////////
int main(void) {
	uart_init(MYUBRR);                // Initialize UART with calculated baud rate
    setup();
    while (1) {
        loop();
    }
	for (int i = 0; i < 10; i++) { // 10 blinks = 5 sec
		PORTB ^= (1 << LED_PIN); // Toggle LED
		_delay_ms(250);
	}
}