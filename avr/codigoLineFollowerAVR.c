#include "avrlnf.h" //Libreria para poder subir el codigo desde la IDE de arduino, por eso los otros dos archivos
#include <avr/io.h>
#include <util/delay.h>

// Declaración de funciones porque en C es necesario para tener main arriba del todo
void configureIO(void); // Función para configurar entradas y salidas

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
uint8_t PWMM1B = PB3;

uint8_t *PWMM2A_DDR = &DDRD;
uint8_t PWMM2A = PD3;

uint8_t *PWMM2B_DDR = &DDRD;
uint8_t PWMM2B = PD5;

// Variables y constantes de los sensores
const uint8_t sensorCount = 6;
uint16_t sensorValues[6]; //Por alguna razon no compila si pongo 'sensorCount' en vez de el numero literal
int position = 0;
int error = 0;
const int errorCount = 5;

// Constantes para el PID
double Kp = 0;
double Ki = 0;
double Kd = 0;

// Variables para el PID
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double cumError = 0, rateError;
double lastError = 0;

// Variables y constantes de los motores
double motVel = 0;
const double vel = 73; // Valor base para la velocidad del motor 
const double velMin = 0; // Valor minimo para la velocidad del motor, solo usar en casos de debugging
double m1Vel = 0;
double m2Vel = 0;
const int ajuste = 13;

// Variables para los botones
int prevState = 0;
int flag = 0; // Si flag es 1, estan los dos motores al palo
unsigned long timePressed = 0;
unsigned long startedPressing = 0;
int ledState = 0;
unsigned long lastBlink = 0;

// Variables para el giroscopio
double cumTheta = 0; // Integral de la velocidad angular (angulo recorrido), theta es una letra griega

int main(void) 
{
  configureIO();
  while (1) 
  {
    
  }
}

void configureIO(void) // Función para configurar entradas y salidas
{
  *led_DDR |= (1 << led);
  *bot_DDR &= ~(1 << bot);
  *enableM1_DDR |= (1 << enableM1);
  *enableM2_DDR |= (1 << enableM2);
  *PWMM1A_DDR |= (1 << PWMM1A);
  *PWMM1B_DDR |= (1 << PWMM1B);
  *PWMM2A_DDR |= (1 << PWMM2A);
  *PWMM2B_DDR |= (1 << PWMM2B);
}
