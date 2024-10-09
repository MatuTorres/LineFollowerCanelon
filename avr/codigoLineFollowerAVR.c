#include "avrlnf.h"  // Incluye el archivo de encabezado
#include <avr/io.h>
#include <util/delay.h>

/*
Para el nombramiento de las variables, queda implicito que:
- Si no se aclara, la variable indica el pin en el que se encuentra determinado componente
- Si la variable termina en _DDR esa variable modifica el DDR (Data Direction Register) del componente que se aclara
- Si la variable termina en _timer esa variable modifica el PWM de ese componente

basicamente si la variable tiene una terminacion se debe empezar con '*' al llamarla para modificar directamente el registro que modifica
ese pin (Ejemplo: *led_DDR |= (1 << led);)
*/

uint8_t *led_DDR = &DDRB;
uint8_t led = PB5;

uint8_t *bot_DDR = &DDRB;
uint8_t bot = PB0;

uint8_t *enableM1_DDR = &DDRD;
uint8_t enableM1 = PD4;

uint8_t *enableM2_DDR = &DDRB;
uint8_t enableM2 = PB1;

uint8_t *PWMM1A_DDR = &DDRB;
uint8_t PWMM1A = PB2;

uint8_t *PWMM1B_DDR = &DDRB;
uint8_t PWMM1A = PB3;

uint8_t *PWMM1A_DDR = &DDRD;
uint8_t PWMM1A = PD3;

uint8_t *PWMM1A_DDR = &DDRD;
uint8_t PWMM1A = PD5;


int main(void) 
{
  while (1) 
  {
    
  }
}
