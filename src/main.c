#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>

// ------------------
// pins of control
#define RS eS_PORTB1
#define EN eS_PORTB0

// pins of data
#define D4 eS_PORTB2
#define D5 eS_PORTB3
#define D6 eS_PORTB4
#define D7 eS_PORTB5

#include <lcd_328P.h>
// ------------------

// ------------------
#define P1 PD2 // ON/OFF
#define P2 PD1 // Config mode
#define P3 PD0 // Switch T1/T2

#define BOUNCE_DELAY 6

#define PWM_PERIOD 124 // (500u / (64/(1/16M))) - 1 = 124
#define PWM_OUTPUT PD5
// ------------------

// ------------------
volatile uint8_t flag = 0;
uint8_t motor_state = 0; // 0 -> OFF / 1 -> ON
uint8_t counter = 0;
char buffer[16]; // character array that stores strings (16 = Number of LCD columns)
volatile uint8_t P2_state;
volatile uint8_t last_P2_state = 1;
volatile uint8_t P3_state;
volatile uint8_t last_P3_state = 1;
volatile uint8_t T = 1;           // 1 -> T1 / 0 -> T2
volatile float duration_up = 100; // * 100ms
volatile uint8_t duration_down = 200;
volatile uint8_t dutyCycle1 = 40;
volatile uint8_t dutyCycle2 = 95;
// ------------------

// ------------------
void start_motor();
void stop_motor();
void config_mode();
void config_mode_display();
void home_display();
// ------------------

int main()
{
    // input pin config
    DDRD &= ~((1 << P1) | (1 << P2) | (1 << P3));

    // output pin config
    DDRD |= (1 << PWM_OUTPUT);
    DDRB = 0xFF;

    // timer0 config
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // FAST PWM
    TCCR0B |= (1 << WGM02);                // TOP in OCR0A
    OCR0A = PWM_PERIOD;                    // TOP

    // timer1 config
    TCCR1B = (1 << WGM12);  // CTC mode
    TIMSK1 = (1 << OCIE1A); // Interrupt Enable
    OCR1A = 6249;           // (1+6249)(256)(1/16MHz) = 100ms

    EIMSK = (1 << INT0);  // enable external interrupt on INT0
    EICRA = (1 << ISC01); // falling edge
    sei();

    // adc config
    DIDR0 |= (1 << ADC0D) | (1 << ADC1D);                 // deactivate the digital pin
    ADMUX |= (1 << REFS0) | (1 << ADLAR);                 // AVcc reference
    ADCSRA |= (1 << ADIE);                                // enable interrupt
    ADCSRA |= (1 << ADATE);                               // auto triggering
    ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2); // pre-scaler 128

    Lcd4_Clear();
    Lcd4_Init();

    while (1)
    {
        if (flag)
        {
            if (motor_state)
                stop_motor();
            else
                start_motor();
            flag = 0;
        }

        P2_state = (PIND & (1 << P2));
        if (!motor_state && P2_state != last_P2_state)
        {
            if (!P2_state)
            {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2)))
                    config_mode();
            }
        }
        last_P2_state = P2_state;

        _delay_ms(10);
        home_display();
    }

    return 0;
}

ISR(INT0_vect)
{
    _delay_ms(BOUNCE_DELAY);
    if (!(PIND & (1 << P1)))
        flag = 1;
}

ISR(TIMER1_COMPA_vect)
{
    counter++;

    switch (motor_state)
    {
    case 1:
        if (counter >= duration_up)
        {
            OCR0B = dutyCycle2 / 100.0 * PWM_PERIOD;
            motor_state = 2;
            counter = 0;
        }
        break;
    case 2:
        if (counter >= duration_down)
        {
            OCR0B = dutyCycle1 / 100.0 * PWM_PERIOD;
            motor_state = 1;
            counter = 0;
        }
        break;
    default:
        break;
    }
}

ISR(ADC_vect)
{
    if (ADMUX & (1 << MUX0))
    {
        if (T)
            duration_up = 100 + (ADCH / 255.0) * 100;
        else
            duration_down = 100 + (ADCH / 255.0) * 100;
    }
    else
    {
        if (T)
            dutyCycle1 = 40 + (ADCH / 255.0) * 55;
        else
            dutyCycle2 = 40 + (ADCH / 255.0) * 55;
    }
    ADMUX ^= (1 << MUX0);
}

void start_motor()
{
    motor_state = 1;

    // PWM
    TCCR0A |= (1 << COM0B1);             // non-inverting mode
    TCCR0B |= (1 << CS00) | (1 << CS01); // prescaler N=64
    // OCR0B = 50;                          // BOTTOM (duty cycle)
    OCR0B = dutyCycle1 / 100.0 * PWM_PERIOD;
    // CTC
    TCCR1B |= (1 << CS12); // prescaler 256
}

void stop_motor()
{
    motor_state = 0;
    counter = 0;

    // PWM
    TCCR0A &= ~(1 << COM0B1);               // desconected
    TCCR0B &= ~((1 << CS00) | (1 << CS01)); // prescaler N=0
    // CTC
    TCCR1B &= ~(1 << CS12); // prescaler N=0.
    TCNT1 = 0;
}

void config_mode()
{
    ADCSRA |= (1 << ADEN) | (1 << ADSC); // initiates the conversion
    last_P2_state = 0;

    while (1)
    {
        if (flag)
            flag = 0;

        // read pin P2 and wait for a debounce time to exit configuration mode
        P2_state = (PIND & (1 << P2));
        if (P2_state != last_P2_state)
        {
            if (!P2_state)
            {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2)))
                    break;
            }
        }
        last_P2_state = P2_state;

        P3_state = (PIND & (1 << P3));
        if (P3_state != last_P3_state)
        {
            if (!P3_state)
            {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P3)))
                    T ^= 1;
            }
        }
        last_P3_state = P3_state;

        _delay_ms(BOUNCE_DELAY);
        config_mode_display();
    }
    ADCSRA &= ~(1 << ADEN); // stop the conversion
    home_display();
}

void config_mode_display()
{
    if (T)
    {
        sprintf(&buffer[0], "T %d: %.1fs   ", 1, duration_up / 10.0);
        Lcd4_Set_Cursor(1, 0);
        Lcd4_Write_String(buffer);
        sprintf(&buffer[0], "C.U %d: %2d%%     ", 1, dutyCycle1);
        Lcd4_Set_Cursor(2, 0);
        Lcd4_Write_String(buffer);
    }
    else
    {
        // configuring T2
        sprintf(&buffer[0], "T %d: %.1fs   ", 2, duration_down / 10.0);
        Lcd4_Set_Cursor(1, 0);
        Lcd4_Write_String(buffer);
        sprintf(&buffer[0], "C.U %d: %2d%%     ", 2, dutyCycle2);
        Lcd4_Set_Cursor(2, 0);
        Lcd4_Write_String(buffer);
    }
}

void home_display()
{
    if (motor_state)
    {
        sprintf(&buffer[0], "Motor: ON      ");
        Lcd4_Set_Cursor(1, 0);
        Lcd4_Write_String(buffer);
        sprintf(&buffer[0], "Status: TIME %d", motor_state);
        Lcd4_Set_Cursor(2, 0);
        Lcd4_Write_String(buffer);
    }
    else
    { // motor is off
        sprintf(&buffer[0], "Motor: OFF     ");
        Lcd4_Set_Cursor(1, 0);
        Lcd4_Write_String(buffer);
        sprintf(&buffer[0], "Status: PAUSE  ");
        Lcd4_Set_Cursor(2, 0);
        Lcd4_Write_String(buffer);
    }
}