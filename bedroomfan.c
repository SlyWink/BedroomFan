#define F_CPU 1200000L

#include <avr/io.h>
#include <util/delay.h>

#define SERIAL_DEBUG_DDR DDRB
#define SERIAL_DEBUG_PORT PORTB
#define SERIAL_DEBUG_PIN PB1

#include "dbginclude.c"

#define PIN_FAN_PWM   PB0
#define PIN_SPEED_POT PB2
#define PIN_IN_PROBE  PB3
#define PIN_OUT_PROBE PB4

#define MUX_SPEED_POT ((0<<MUX1) | (1<<MUX0))
#define MUX_IN_PROBE  ((1<<MUX1) | (0<<MUX0))
#define MUX_OUT_PROBE ((1<<MUX1) | (1<<MUX0))

#define PWM_MAX 0x94 // Limit max current to 1A (of 2.7A)
#define PWM_MIN 0x10
#define PWM_RANGE (PWM_MAX - PWM_MIN + 1)

#define POT_MAX 0xC3 // Right limit
#define POT_MIN 0x45 // Left limit
#define POT_RANGE (POT_MAX - POT_MIN + 1)

#define ADC_READ_COUNT 4


void Fan_Speed(uint8_t p_pwm) {

// < 0xC0 pour tomber < 1,5A
// = 0x50 fonctionnement normal
// ~ 0x20 fonctionnement silencieux

  OCR0A = (p_pwm < PWM_MAX) ? p_pwm : PWM_MAX ;
}


uint8_t Pot_2_Pwm(uint8_t l_pot) {
  uint16_t l_tmp ;

  l_tmp = (l_pot - POT_MIN) * PWM_RANGE ;
  return (PWM_MIN + (uint8_t) (l_tmp / POT_RANGE)) ;
}


uint8_t ADC_Read(uint8_t p_mux) {
  uint16_t l_sum ;
  uint8_t l_count ;

  // Clear MUX1 & MUX0 in ADMUX
  ADMUX &= ~(_BV(MUX1)|_BV(MUX0)) ;
  ADMUX |= p_mux ;
  l_sum = 0 ;
  // Read ADC several times
  for (l_count=0 ; l_count<ADC_READ_COUNT ; l_count++) {
    ADCSRA |= _BV(ADSC) ; // Start conversion
    while (ADCSRA & _BV(ADSC)) ; // Wait conversion to complete
    l_sum += ADCH ;
  }
  return (uint8_t)(l_sum / ADC_READ_COUNT) ;
}


int main(void) {
  uint8_t l_pot ;
  uint8_t l_prev ;
  uint8_t l_pwm ;

  OSCCAL = 0x56;

  DDRB = _BV(PIN_FAN_PWM) ; //Output
  PORTB = (0<<PIN_FAN_PWM) | _BV(PIN_SPEED_POT) ;

  Serial_Debug_Init() ;

  // PWM init
  TCCR0A = ((1<<COM0A1) | (1<<COM0A0)) | ((0<<WGM01) | (1<<WGM00)) ;
  TCCR0B = (0<<WGM02) | ((0<<CS02) | (0<<CS01) | (1<<CS00)) ;

  // ADC init
  ADMUX = _BV(ADLAR) ;
  ADCSRA = _BV(ADEN) | ((1<<ADPS2) | (0<<ADPS1) | (0>>ADPS0));
  // AJOUTER INIT ADC


  l_prev = 0 ;
  for(;;) {

   do {
     l_pot = ADC_Read(MUX_SPEED_POT) ;
   } while (l_pot == l_prev) ;
   l_pwm = Pot_2_Pwm(l_pot) ;
   Fan_Speed(l_pwm) ;
   Serial_Debug_Send(l_pot) ; _delay_ms(1000) ;
   Serial_Debug_Send(l_pwm) ; _delay_ms(1000) ;
//   _delay_ms(10) ;
  }
}
