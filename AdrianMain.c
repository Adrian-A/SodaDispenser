/*
 * FinalProject.c
 *
 * Created: 4/28/2025 3:55:00 PM
 * Author : Adrian Alvarez & Seth Bolen
 */
// Library for Sensor: https://github.com/Ovidiu22/HC-SR04/tree/main/src

#define F_CPU 16000000UL  // 16 MHz clock
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <Ultrasonic.h>

// Define UART baud rate
#define BAUD 9600               // Baud rate for serial communication
#define MYUBRR F_CPU/16/BAUD-1  // Baud rate register calculation

#define RELAY_PIN1 PB4     // Relay control pin 1
#define BUTTON1_PIN PB3    // First pushbutton input (was relay before)
#define BUTTON2_PIN PB2    // Second pushbutton input (new pin)
#define RELAY_PIN2 PB1     // Relay control pin 2 (was button before)
#define LED_PIN    PB5     // Built-in "L" LED pin
#define TRIG_PIN PD2 // Trigger pin
#define ECHO_PIN PD3 // Echo pin

//long duration, distance;
//char buffer[7];
uint8_t distance = 0;
uint8_t diagnostics = 0;
volatile uint16_t pulse;
volatile uint8_t iIRC = 0;
volatile int f_wdt = 1;

void setup(void) {
    // Set relay pins and LED pin as outputs
    DDRB |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2) | (1 << LED_PIN);

    // Set button pin as input
    DDRB &= ~(1 << BUTTON1_PIN);
	DDRB &= ~(1 << BUTTON2_PIN);

	// Set trigger as output and echo as input
	//DDRD |= (1 << TRIG_PIN);
	//DDRD &= ~(1 << ECHO_PIN);
	init_ultrasonic();

    // Enable pull-up resistor on button pin
    PORTB |= (1 << BUTTON1_PIN);
	PORTB |= (1 << BUTTON2_PIN);

    // Turn on built-in LED steady
    PORTB |= (1 << LED_PIN);

    // Make sure relays are OFF at startup
    PORTB |= ~((1 << RELAY_PIN1) | (1 << RELAY_PIN2));	
}

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

// Initialize Timer1 for pulse measurement
void timer1_init() {
	TCCR1A = 0;                  // Normal mode (no waveform generation)
	TCCR1B = (1 << CS11);        // Start Timer1 with prescaler = 8 (1 tick = 0.5 µs at 16 MHz)
}

uint16_t pulseIn() {
	//uart_print("Debug1");
	//// Wait for ECHO to go HIGH
	//while (!(PIND & (1 << ECHO_PIN)));
//
	//uart_print("Debug2");
	//// Start timing
	//TCNT1 = 0;
//
	//// Wait for ECHO to go LOW
	//uart_print("Debug3");
	//while (PIND & (1 << ECHO_PIN));
//
	//uart_print("Debug4");
	//return TCNT1;
	
	uint16_t timeout = 38000; // 38 ms timeout = 76000 timer ticks at 0.5 µs per tick

	// Wait for ECHO to go HIGH (start of echo pulse)
	while (!(PIND & (1 << ECHO_PIN))) {
		if (--timeout == 0) return 0; // Timeout waiting for HIGH
	}

	// Start timing
	TCNT1 = 0;

	// Reset timeout for HIGH duration
	timeout = 76000; // Max echo high time in ticks (38 ms / 0.5 µs)

	// Wait for ECHO to go LOW (end of echo pulse)
	while (PIND & (1 << ECHO_PIN)) {
		if (--timeout == 0) return 0; // Timeout waiting for LOW
	}

	return TCNT1; // Duration of HIGH pulse (in 0.5 µs units)
}

void sendPulse(void){
	// Clear the trigger pin
	PORTD &= ~(1 << TRIG_PIN); // Set trigger pin LOW
	_delay_us(2); // Wait 2 microseconds
	
	// Set the trigger pin high
	PORTD |= (1 << TRIG_PIN); // Set trigger pin HIGH
	_delay_us(10); // Wait for 10 microseconds
	PORTD &= ~(1 << TRIG_PIN); // Set trigger pin LOW
	
	//uart_print("Debug");
	_delay_ms(10);
	
	//TCCR1B |= (1 << CS11); // Start Timer1 with prescaler 8
	//duration = pulseIn(); // Read echo duration
	//TCCR1B &= ~(1 << CS11); // Stop Timer1
	//float distance = (duration*340)/2000;
	//distance = (duration*343)/(2*100); // Convert to distance in cm (avoids float)
	//dtostrf(distance, 4, 2, buffer);
}

void loop(void) {
    // Wait for button1 press
    if (!(PINB & (1 << BUTTON1_PIN))) {
        _delay_ms(50);  // debounce delay

        // Confirm button still pressed after debounce
        if (!(PINB & (1 << BUTTON1_PIN))) {

            // Turn ON relays immediately
			 PORTB &= ~((1 << RELAY_PIN1));
        

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
			 PORTB &= ~((1 << RELAY_PIN2));
			 
			 // Wait for button release before accepting another press
			 while (!(PINB & (1 << BUTTON2_PIN))) {
				 _delay_ms(10);
			 }
			 _delay_ms(50); // optional debounce after release
		 }
	}
	
	//uart_print("Hello");
	distance = getDistance_main(&diagnostics);
	// Print distance over UART
	uart_print("Distance: ");      // Print label
	uart_print_float(distance);    // Print measured distance
	uart_print(" cm\r\n");         // Print units and newline
	
	_delay_ms(100);                // Wait 100 ms before next reading
	//if (distance <= 4){
		//// Print distance over UART
		//uart_print("Distance: ");      // Print label
		//uart_print_float(distance);    // Print measured distance
		//uart_print(" cm\r\n");         // Print units and newline
//
		//_delay_ms(100);                // Wait 100 ms before next reading
	//}
	
	PORTB |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2); // Make relays go high (turns off relay) (MAYBE, LOOKUP)
}

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

int main(void) {
	uart_init(MYUBRR);                // Initialize UART with calculated baud rate
	timer1_init();                    // Initialize Timer1 for measuring pulse width
    setup();
    while (1) {
        loop();
    }
	for (int i = 0; i < 10; i++) { // 10 blinks = 5 sec
		PORTB ^= (1 << LED_PIN); // Toggle LED
		_delay_ms(250);
	}
}
