#define DEBUG

#define F_CPU 1200000L

#include <avr/io.h>
#include <util/delay.h>

#ifdef DEBUG
  #define SERIAL_DEBUG_DDR DDRB
  #define SERIAL_DEBUG_PORT PORTB
  #define SERIAL_DEBUG_PIN PB1
  #include "dbginclude.c"
#endif

#define PIN_FAN_PWM   PB0
#define PIN_SPEED_POT PB2
#define PIN_SENSOR    PB3

#define MUX_SPEED_POT ((0<<MUX1) | (1<<MUX0))
#define MUX_SENSOR    ((1<<MUX1) | (1<<MUX0))

#define PWM_MAX 0x94 // Limit max current to 1A (of 2.7A)
#define PWM_MIN 0x10
#define PWM_RANGE (PWM_MAX - PWM_MIN + 1)
#define PWM_STEP 40

#define POT_MAX 0xC3 // Right limit
#define POT_MIN 0x45 // Left limit
#define POT_RANGE (POT_MAX - POT_MIN + 1)

#define ADC_READ_COUNT 4

#define SENSOR_ZERO 0x7F

void Fan_Speed(uint8_t p_new_pwm) {
  static uint8_t l_old_pwm = 0 ;
  uint8_t l_pwm ;

  // Progressive PWM to avoid massive current load
  if (p_new_pwm > l_old_pwm) {
    for (l_pwm=l_old_pwm ; l_pwm<p_new_pwm ; l_pwm++)
      if (l_pwm >= PWM_MIN) { OCR0A = l_pwm ; _delay_ms(PWM_STEP) ; }
  } else if (p_new_pwm < l_old_pwm) {
    for (l_pwm=l_old_pwm ; l_pwm>p_new_pwm ; l_pwm--)
      if (l_pwm >= PWM_MIN) { OCR0A = l_pwm ; _delay_ms(PWM_STEP) ; }
  }
  OCR0A = p_new_pwm ;
  l_old_pwm = p_new_pwm ;
}


uint8_t Pot_2_Pwm(uint8_t l_pot) {
  uint16_t l_tmp ;

  l_tmp = (l_pot - POT_MIN) * PWM_RANGE ;
  return (PWM_MIN + (uint8_t) (l_tmp / POT_RANGE)) ;
}


uint8_t ADC_Read(uint8_t p_mux) {
  uint16_t l_sum ;
  uint8_t l_count ;

  ADMUX &= ~(_BV(MUX1)|_BV(MUX0)) ; // Clear MUX1 & MUX0 in ADMUX
  ADMUX |= p_mux ; // Select MUX pin
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
  uint8_t l_pwm_max, l_pwm ;
  uint8_t l_sensor ;

  DDRB = _BV(PIN_FAN_PWM) ; //Output
  PORTB = (0<<PIN_FAN_PWM) | _BV(PIN_SPEED_POT) ;

#ifdef DEBUG
  OSCCAL = 0x56;
  Serial_Debug_Init() ;
#endif

  // PWM init
  TCCR0A = ((1<<COM0A1) | (1<<COM0A0)) | ((0<<WGM01) | (1<<WGM00)) ;
  TCCR0B = (0<<WGM02) | ((0<<CS02) | (0<<CS01) | (1<<CS00)) ;

  // ADC init
  ADMUX = _BV(ADLAR) ;
  ADCSRA = _BV(ADEN) | ((1<<ADPS2) | (0<<ADPS1) | (0>>ADPS0));


  for(;;) {
    l_pwm_max = Pot_2_Pwm(ADC_Read(MUX_SPEED_POT)) ;
    l_sensor = ADC_Read(MUX_SENSOR) ;

    switch (l_sensor) {
      case 0 ... SENSOR_ZERO:
        l_pwm = 0 ;
        break ;
      case SENSOR_ZERO+1 ... SENSOR_ZERO+2:
        l_pwm = PWM_MIN + (l_pwm_max - PWM_MIN) / 4 ;
        break ;
      case SENSOR_ZERO+3 ... SENSOR_ZERO+4:
        l_pwm = PWM_MIN + (l_pwm_max - PWM_MIN) / 2 ;
        break ;
      default:
        l_pwm = l_pwm_max ;
    }
    Fan_Speed(l_pwm) ;

#ifdef DEBUG
    Serial_Debug_Send(0xF0) ; _delay_ms(500) ;
    Serial_Debug_Send(l_sensor) ; _delay_ms(1000) ;
    Serial_Debug_Send(0xF1) ; _delay_ms(500) ;
    Serial_Debug_Send(l_pwm_max) ; _delay_ms(1000) ;
    Serial_Debug_Send(0xF2) ; _delay_ms(500) ;
    Serial_Debug_Send(l_pwm) ; _delay_ms(1000) ;
#endif
  }
}
