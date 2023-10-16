#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

// -------------------- DISPLAY LCD --------------------
// Pins of control
#define RS eS_PORTB0
#define EN eS_PORTB1

// Pins of data
#define D4 eS_PORTB2
#define D5 eS_PORTB3
#define D6 eS_PORTB4
#define D7 eS_PORTB5

#include <lcd_328P.h>
// -------------------- DISPLAY LCD --------------------

#define PWM_OUTPUT PD3
#define P1 PD2 // On/Off
#define P2 PD1 // Config mode
#define P3 PD0 // Velocity

#define BOUNCE_DELAY 6 // ms

volatile uint8_t flag = 0;
volatile uint8_t config_flag = 1;
char buffer[16]; // Character array that stores strings (16 = Number of LCD rows)

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
    DDRD &= ~((1 << P1) | (1 << P2) | (1 << P3));

    // Config output pin to PWM
    DDRD |= (1 << PWM_OUTPUT);

    // Port to display data
    DDRB = 0xFF;

    Lcd4_Init();
    Lcd4_Clear();

    sprintf(&buffer[0], "Welcome.");
    Lcd4_Set_Cursor(1, 0);
    Lcd4_Write_String(buffer);

    while (1)
    {
    }

    return 0;
}