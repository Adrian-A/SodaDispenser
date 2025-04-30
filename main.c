/*
 * FinalProject.c
 *
 * Created: 4/28/2025 3:55:00 PM
 * Author : Adrian Alvarez & Seth Bolen
 */

#define F_CPU 16000000UL  // 16 MHz clock
#include <avr/io.h>
#include <util/delay.h>


#define RELAY_PIN1 PB4     // Relay control pin 1
#define BUTTON1_PIN PB3    // First pushbutton input (was relay before)
#define BUTTON2_PIN PB2    // Second pushbutton input (new pin)
#define RELAY_PIN2 PB1     // Relay control pin 2 (was button before)
#define LED_PIN    PB5     // Built-in "L" LED pin

void setup(void) {
    // Set relay pins and LED pin as outputs
    DDRB |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2) | (1 << LED_PIN);

    // Set button pin as input
    DDRB &= ~(1 << BUTTON1_PIN);
	DDRB &= ~(1 << BUTTON2_PIN);

    // Enable pull-up resistor on button pin
    PORTB |= (1 << BUTTON1_PIN);
	PORTB |= (1 << BUTTON2_PIN);

    // Turn on built-in LED steady
    PORTB |= (1 << LED_PIN);

    // Make sure relays are OFF at startup
    PORTB |= ~((1 << RELAY_PIN1) | (1 << RELAY_PIN2));
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
	PORTB |= (1 << RELAY_PIN1) | (1 << RELAY_PIN2);
}

int main(void) {
    setup();
    while (1) {
        loop();
    }
	for (int i = 0; i < 10; i++) { // 10 blinks = 5 sec
		PORTB ^= (1 << LED_PIN); // Toggle LED
		_delay_ms(250);
	}
}
