/*
 * FinalProject.c
 *
 * Created: 4/28/2025 3:55:00 PM
 * Author : Adrian Alvarez & Seth Bolen
 */ 

#define F_CPU 16000000UL  // 16 MHz clock
#include <avr/io.h>
#include <util/delay.h>

#define RELAY_PIN PB3  // Use PORTB Pin 3

void setup(void) {
    // Set RELAY_PIN as output
    DDRB |= (1 << RELAY_PIN);
}

void loop(void) {
    // Turn on relay (set pin high)
    PORTB |= (1 << RELAY_PIN);
    _delay_ms(5000);

    // Turn off relay (set pin low)
    PORTB &= ~(1 << RELAY_PIN);
    _delay_ms(5000);
}

int main(void) {
    setup();
    while (1) {
        loop();
    }
}
