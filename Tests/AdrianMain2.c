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
#include <utils.h>
#include <spi.h>
#include <mfrc522.h>

// Define UART baud rate
#define BAUD 9600               // Baud rate for serial communication
#define MYUBRR F_CPU/16/BAUD-1  // Baud rate register calculation

#define RELAY_PIN1 PD5     // Relay control pin 1
#define BUTTON1_PIN PC4    // First pushbutton input (was relay before)
#define BUTTON2_PIN PC5    // Second pushbutton input (new pin)
#define RELAY_PIN2 PD4     // Relay control pin 2 (was button before)
#define LED_PIN    PB5     // Built-in "L" LED pin

#define TRIG_PIN PB0 // Trigger pin
#define ECHO_PIN PD2 // Echo pin

// global variables for sensor
uint8_t distance = 0;
uint8_t diagnostics = 0;
volatile uint16_t pulse;
volatile uint8_t iIRC = 0;
volatile int f_wdt = 1;

// global variables for rfid
uint8_t SelfTestBuffer[64];
uint8_t byte;
uint8_t str[MAX_LEN];
volatile uint8_t uid_match = 0;
volatile uint8_t distance_ok = 0;
//volatile uint8_t rfid_ok = 0;

///////////////////////////////////////////////
// Setups everything
///////////////////////////////////////////////
void setup(void) {
    // Set relay pins and LED pin as outputs
    DDRD |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2);// 
	DDRB |= (1 << LED_PIN);

    // Set button pin as input
    DDRC &= ~(1 << BUTTON1_PIN);
	DDRC &= ~(1 << BUTTON2_PIN);

	init_rfid();
	// Initializes sensor
	init_ultrasonic();
	// Initializes RFID
	// FIX: THIS CAUSES THE SENSOR TO NOT WORK BECAUSE OF TIMER STUFF
	//init_rfid();

    // Enable pull-up resistor on button pin
    PORTC |= (1 << BUTTON1_PIN);
	PORTC |= (1 << BUTTON2_PIN);

    // Turn on built-in LED steady
    PORTB |= (1 << LED_PIN);

    // Make sure relays are OFF at startup
    //PORTD |= ~((1 << RELAY_PIN1) | (1 << RELAY_PIN2));	
	
	PORTD |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2);
}

///////////////////////////////////////////////
// Setups RFID
///////////////////////////////////////////////
void init_rfid(){	
	spi_init();
	//_delay_ms(1000);
	
	//init reader
	mfrc522_init();
	
	byte = mfrc522_read(ComIEnReg);
	mfrc522_write(ComIEnReg,byte|0x20);
	byte = mfrc522_read(DivIEnReg);
	mfrc522_write(DivIEnReg,byte|0x80);
	
	//_delay_ms(1500);
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
	distance = getDistance_main(&diagnostics);

	// Check and set distance_ok flag
	if (distance <= 4) {
		distance_ok = 1;
	} 
	else {
		distance_ok = 0;
	}
	
	uart_print("Distance: ");      // Print label
	uart_print_float(distance);    // Print measured distance
	uart_print(" cm\r\n");         // Print units and newline

	// RFID check - updates rfid_ok flag inside
	if (uid_match == 0){
		check_rfid();
	}

	// Only check buttons if both conditions are met
	while (distance_ok && uid_match) {
		uart_print("In if");
		uart_print("\r\n");
		
		distance = getDistance_main(&diagnostics);

		uart_print("Distance: ");      // Print label
		uart_print_float(distance);    // Print measured distance
		uart_print(" cm\r\n");         // Print units and newline

		// Check and set distance_ok flag
		if (distance <= 4) {
			distance_ok = 1;
		}
		else {
			distance_ok = 0;
		}

		// --- BUTTON 1 CHECK ---
		if (!(PINC & (1 << BUTTON1_PIN))) {
			uart_print("Button 1");
			uart_print("\r\n");
			
			// Debounce
			//_delay_ms(50);
			if (!(PINC & (1 << BUTTON1_PIN))) {
				// Activate relay 1
				PORTD &= ~(1 << RELAY_PIN1);
				uart_print("Relay 1 ON\r\n");

				// Wait for button release
				while (!(PINC & (1 << BUTTON1_PIN))) {
					_delay_ms(10);
				}
			}
		}

		// --- BUTTON 2 CHECK ---
		if (!(PINC & (1 << BUTTON2_PIN))) {
			uart_print("Button 2");
			uart_print("\r\n");
			
			// Debounce
			//_delay_ms(50);
			if (!(PINC & (1 << BUTTON2_PIN))) {
				// Activate relay 2
				PORTD &= ~(1 << RELAY_PIN2);
				uart_print("Relay 2 ON\r\n");

				// Wait for button release
				while (!(PINC & (1 << BUTTON2_PIN))) {
					_delay_ms(10);
				}
			}
		}
		
		PORTD |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2);
	}

	_delay_ms(100); // Polling interval
}

void check_rfid(){
	uart_print("In RFID");
	uart_print("\r\n");
	
	byte = mfrc522_request(PICC_REQALL, str);
	uid_match = 0;  // Reset match flag each scan

	if (byte == CARD_FOUND) {
		uart_print("Card found 1");
		uart_print("\r\n");
		byte = mfrc522_get_card_serial(str);
		if (byte == CARD_FOUND) {
			uart_print("Card found 2");
			uart_print("\r\n");
			// Check for matching UID: 61 24 18 02 5F
			if (str[0] == 0x61 && str[1] == 0x24 && str[2] == 0x18 && str[3] == 0x02 && str[4] == 0x5F) {
				uart_print("Card match");
				uart_print("\r\n");
				uid_match = 1;
			}
			_delay_ms(2500);  // Wait before next read attempt
		}
	}

	_delay_ms(1000);  // General polling delay
}


/* *****************************************************************
Name:		Ultrasonic Interrupt
Inputs:		none
Outputs:	internal timer
Description:calculates elapsed time of a measurement
******************************************************************** */
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
