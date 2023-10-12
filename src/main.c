#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define PWM_OUTPUT PD3
#define P1 PD2 // On/Off

#define BOUNCE_DELAY 6 // ms

volatile uint8_t flag = 0;

ISR(INT0_vect)
{
    _delay_ms(BOUNCE_DELAY);
    if (!(PIND & (1 << P1)))
    {
        flag = 1;
    }
}

int main(void)
{
    // Timer2
    // Config compare TOP and BOTTOM to width pulse
    OCR2A = 125;
    OCR2B = 50;

    // Fast PWM mode
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2B1);
    TCCR2B = (1 << WGM22);

    // Config pre-scaler
    TCCR2B |= (1 << CS22); // N=64

    // External interrup
    EICRA |= (1 << ISC01); // set INT0 to trigger on ANY logic change
    EIMSK |= (1 << INT0);  // Turns on INT0

    sei();

    // Config inputs pin
    DDRD &= (1 << P1);
    PORTD = (1 << P1);

    // Config output pin to PWM
    DDRD |= (1 << PWM_OUTPUT);

    // Config PORTB as output to display
    DDRB = 0xFF;

    while (1)
    {
        if (flag)
            PORTB |= (1 << PORTB0);
    }

    return 0;
}