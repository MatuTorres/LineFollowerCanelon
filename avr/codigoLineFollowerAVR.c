#include "avrlnf.h"  // Incluye el archivo de encabezado
#include <avr/io.h>
#include <util/delay.h>

uint8_t *ledPort = &DDRB;
uint8_t led = PB5;

int main(void) 
{
  *ledPort |= (1 << led);

  while (1) 
  {
    *ledPort |= (1 << led);
    _delay_ms(500);

    *ledPort &= ~(1 << led);
    _delay_ms(500);
  }
}
