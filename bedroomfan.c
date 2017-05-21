#define F_CPU 1200000L

#include <avr/io.h>
#include <util/delay.h>

#define SERIAL_DEBUG_DDR DDRB
#define SERIAL_DEBUG_PORT PORTB
#define SERIAL_DEBUG_PIN PB4

#include "dbginclude.c"

#define PIN_FAN PB0


#define PWM_MAX 0xF0

void Fan_Speed(uint8_t p_pwm) {

// < 0xC0 pour tomber < 1,5A
// = 0x50 fonctionnement normal
// ~ 0x20 fonctionnement silencieux

  OCR0A = (p_pwm < PWM_MAX) ? p_pwm : PWM_MAX ;
}


int main(void) {
  uint8_t l_pwm ;
  #define INC_PWM 0x10
//  uint8_t l_osccal = 0 ;

  OSCCAL = 0x56;

  DDRB = _BV(PIN_FAN) ; //Output
  PORTB = (0<<PIN_FAN) ; //Low

  Serial_Debug_Init() ;

  TCCR0A = ((1<<COM0A1) | (1<<COM0A0)) | ((0<<WGM01) | (1<<WGM00)) ;
  TCCR0B = (0<<WGM02) | ((0<<CS02) | (0<<CS01) | (1<<CS00)) ;

  for(;;) {

    for(l_pwm=0 ; l_pwm<PWM_MAX ; l_pwm+=INC_PWM) {
      Serial_Debug_Send(l_pwm) ;
      Fan_Speed(l_pwm);
      _delay_ms(5000) ;
    }
    for(l_pwm=PWM_MAX ; l_pwm>0 ; l_pwm-=INC_PWM) {
      Serial_Debug_Send(l_pwm) ;
      Fan_Speed(l_pwm);
      _delay_ms(5000) ;

    }
/*
OSCCAL=l_osccal;
Serial_Debug_Send(0b10101010);
_delay_ms(1000);
Serial_Debug_Send(0b01010101);
_delay_ms(1000);
Serial_Debug_Send(l_osccal);
_delay_ms(1000);
l_osccal++;
*/
  }
}
