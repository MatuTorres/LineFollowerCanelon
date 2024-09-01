### A pedido del público...
# Explicación "Código Marche"
Así vemos lo que está bien y lo que está mal.

## Declaración de pines y valores iniciales

```
#define LED13 13 // NO CAMBIA
#define Bot 8 // NO CAMBIA
#define Bot1 2 // NO CAMBIA
#define LedOn 13 // NO CAMBIA
#define PWMM1B 11
#define PWMM1A 10
#define ENM1 4
#define ENM2 9
#define PWMM2A 3
#define PWMM2B 5
```

Empieza inicializando los pines, nada que aclarar acá, lo único en el diseño habría que cambiar para que LedOn y LED13 sean distintos, pero no es necesario.

PWMMnx siendo ‘n’ el número de motor y ‘x’ la dirección del motor, conexiones al puente H.

ENMn siendo ‘n’ el número de motor son los enables que habilitan los motores.

```
#define CALIBRATION_SENTINEL 0x55  // Un valor específico para detectar si hay datos
```

Constante para temas de guardar calibración en la EEPROM, **no es necesario**

```
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int umbral = 500;
int position = 0;
int error = 0;
```
Crea un **objeto** ```qtr``` que son los 8 sensores (el objeto es el conjunto entero).

El array ```sensorValues``` guarda los valores de los sensores

El ```umbral``` es lo que distingue entre blanco y negro, despues se vuelve a calcular con los valores de la calibración por lo que no queda en 500.

La posición es la ubicación del auto, valor de 0 a 7000 (definido por la librería ```QTRSensors.h```), el error es la distancia del centro de los sensores a la línea, valor entre -3500 y 3500, la línea se encuentra en 0.

Por lo que podemos tratar al error como a una función

```math
E: \mathbb{R} \rightarrow [-3500, 3500] / y = E(t)
```

*E(t)* es el error en el tiempo, no se usa *e* por el número de euler, tratar a el error como una función nos va a servir el el cálculo del PID, más adelante

Como pequeña aclaración, el dominio no seria todos los reales, ya que tenemos distintos saltos en el tiempo entre medición y medición, pero al calcular la integral del error es lo más cerca que podemos estar.

```
double mAVel = 0;
double mBVel = 0;
double motVel = 0;
double Vel = 70; // Valor maximo para la velocidad del motor 
double VelMin = 60; // Valor minimo para la velocidad del motor
bool flag = false;
bool flag2 = true;
// Variables globales adicionales para el control PID
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double cumError = 0, rateError;
double lastError = 0;
```

- ```mxVel``` es la velocidad del motor ‘X’.
- ```motVel``` es la salida del PID
- ```Vel``` es la velocidad máxima.
- ```VelMin``` es la velocidad mínima.
- Si ambas flags están en ```true```, el auto puede empezar a seguir la línea, estas se controlan con los botones.
- ```currentTime```, ```previousTime``` y ```elapsedTime``` guardan lo que su nombre indica, usados en el cálculo de la integral.
- ```cumError``` es la acumulación del error, es decir, la integral del error.
- ```rateError``` es la tasa de cambio del error, es decir, la derivada del error.
- ```lastError``` guarda el último valor del error.

```
// Parámetros del PID
double Kp = 0.3;
double Ki = 0.01;
double Kd = 0.35;
```

Constantes usadas en el cálculo del PID, se calculan a prueba y error.

## Funciones Iniciales (setup)

### Configuración inicial

```
void setup()
{
    configureSensors();
    configureMotor();
    configureIO();
    calibration();
...
}
```

Lo primero que hace el auto son estas funciones, que vamos a detallar ahora.

```
void configureSensors()
{
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
}
```

La función ```configureSensors()``` lo que hace es inicializar el array de sensores , primero los setea como analógicos, luego configura los pines a los que están conectados los emisores de los fototransistores de los sensores.

```
void configureMotor()
{
    digitalWrite(PWMM2B, LOW);
    digitalWrite(ENM1, HIGH);
    digitalWrite(PWMM1B, LOW);
    digitalWrite(ENM2, HIGH);
    analogWrite(PWMM1A, 0);
    analogWrite(PWMM2A, 0);
}
```

```configureMotor``` setea los valores iniciales de los motores.

```
// Configura los pins de entrada/salida
void configureIO()
{
    pinMode(LedOn, OUTPUT);
    pinMode(Bot, INPUT_PULLUP);
    pinMode(Bot1, INPUT_PULLUP);
    pinMode(PWMM1A, OUTPUT);
    pinMode(PWMM2A, OUTPUT);
    pinMode(ENM1, OUTPUT);
    pinMode(ENM2, OUTPUT);
    pinMode(PWMM1B, OUTPUT);
    pinMode(PWMM2B, OUTPUT);
}
```

Esta función configura todos los pines usados como entradas o salidas.

**ERROR**: Esta función es llamada luego de ponerle un valor a los motores, cuando debería ser al revés.

### Calibración

```
void calibration()
{
    pinMode(LED13, OUTPUT);
    digitalWrite(LED13, HIGH);

    for (uint16_t i = 0; i < 150; i++)
    {
        qtr.calibrate();
    }

    digitalWrite(LED13, LOW);
    Serial.begin(9600);
    printCalibration();
}
```

Aquí una de las funciones más importantes, la que se ocupa de la **calibración**, está enciende el ```LED13```, integrado en el arduino, para indicar que el auto está en proceso de calibración.

Durante la calibración, se guardan los **valores máximos y mínimos** que toman los sensores para luego calcular correctamente el umbral. *Esto último está mal)*

Al terminar de calibrar, apaga el ```LED13``` indicando que se terminó de calibrar. Y luego inicia la serial para mostrar en esta los valores mínimos y máximos que tomó cada sensor (función ```printCalibration```).

