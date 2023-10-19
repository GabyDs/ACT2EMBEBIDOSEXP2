#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>

// -------------------- DISPLAY LCD --------------------
// pins of control
#define RS eS_PORTB0
#define EN eS_PORTB1

// pins of data
#define D4 eS_PORTB2
#define D5 eS_PORTB3
#define D6 eS_PORTB4
#define D7 eS_PORTB5

#include <lcd_328P.h>
// -------------------- DISPLAY LCD --------------------

#define PWM_OUTPUT PD3
#define P1 PD2 // on/off motor
#define P2 PD1 // entert to config mode
#define P3 PD0 // switch between T1 and T2

#define RV1 PC0
#define RV2 PC1

#define BOUNCE_DELAY 6 // ms

volatile uint8_t on_off_flag = 0; // 1 -> ON / 0 -> OFF
volatile uint8_t config_T = 1;    // 1 -> T1 / 0 -> T2

char buffer[16]; // character array that stores strings (16 = Number of LCD rows)

uint8_t last_config_state = 1;
uint8_t config_state;
uint8_t last_on_off_state = 1;
uint8_t on_off_state;
uint8_t last_time_select_state = 1;
uint8_t time_select_state;

void configuration_mode();
void home_display();

ISR(INT0_vect)
{
    _delay_ms(BOUNCE_DELAY);
    if (!(PIND & (1 << P1)))
        on_off_flag = 1;
}

int main(void)
{

    // timer to PWM signal
    OCR2A = 125; // register that allows for modifying frecuency of the PWM signal
    OCR2B = 50;  // register that allows for modifying the pulse width of the PWM signal

    // fast PWM mode
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2B1);
    TCCR2B = (1 << WGM22);

    // config pre-scaler
    TCCR2B |= (1 << CS22); // N=64

    // external interrup
    EICRA |= (1 << ISC01); // set INT0 to trigger on ANY logic change
    EIMSK |= (1 << INT0);  // turns on INT0

    sei();

    // ADC config
    DIDR0 |= (1 << ADC0D) | (1 << ADC1D); // deactivate the digital pin

    ADMUX = (1 << ADLAR);  // allows adjusting the data register to the left -> 16 bits
    ADMUX |= (1 << REFS0); // enables Vcc as the reference voltage

    ADCSRA = (1 << ADIE);                                 // enable interrupt
    ADCSRA |= (1 << ADATE);                               // auto triggering
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // pre-scaler 128

    // config inputs pin
    DDRD &= ~((1 << P1) | (1 << P2) | (1 << P3));
    DDRC &= ~((1 << RV1) | (1 << RV2));

    // config output pin to PWM
    DDRD |= (1 << PWM_OUTPUT) | (1 << PORTD4);
    PORTD &= ~(1 << PORTD4);

    // port to display data
    DDRB = 0xFF;

    Lcd4_Init();
    home_display();

    while (1)
    {
        config_state = (PIND & (1 << P2));
        if (config_state != last_config_state)
        {
            if (!config_state)
            {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2)))
                    configuration_mode();
            }
        }
        last_config_state = config_state;
    }

    return 0;
}

void configuration_mode()
{
    // ADCSRA |= (1 << ADEN) | (1 << ADSC); // initiates the conversion

    last_config_state = 0;

    while (1)
    {
        if (on_off_flag)
            on_off_flag = 0;

        // read pin P2 and wait for a debounce time to exit configuration mode
        config_state = (PIND & (1 << P2));
        if (config_state != last_config_state)
        {
            if (!config_state)
            {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2)))
                {
                    home_display();
                    break;
                }
            }
        }
        last_config_state = config_state;

        // read pin P3 and wait for a debounce time to switch config_T
        time_select_state = (PIND & (1 << P3));
        if (time_select_state != last_time_select_state)
        {
            if (!time_select_state)
            {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P3)))
                    config_T ^= 1;
            }
        }
        last_time_select_state = time_select_state;

        _delay_ms(BOUNCE_DELAY);
        sprintf(&buffer[0], "Modo T%d", config_T);
        Lcd4_Set_Cursor(1, 0);
        Lcd4_Write_String(buffer);
    }
}

void home_display()
{
    Lcd4_Clear();
    sprintf(&buffer[0], "Welcome");
    Lcd4_Set_Cursor(1, 0);
    Lcd4_Write_String(buffer);
}