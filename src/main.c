#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

int main(void)
{
    // Config compare TOP and BOTTOM to width pulse
    OCR2A = 125;
    OCR2B = 50;

    // Config Timer0 Fast PWM mode
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2B1);
    TCCR2B = (1 << WGM22);

    // Config pre-scaler
    TCCR2B |= (1 << CS22); // N=64

    // Config output pin to PWM
    DDRD = (1 << PD3);

    while (1)
    {
    }

    return 0;
}