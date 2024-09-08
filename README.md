### A pedido del público...
# Explicación "Código Marche"
Así vemos lo que está bien y lo que está mal.

PD: Outdated

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

Durante la calibración, se guardan los **valores máximos y mínimos** que toman los sensores para luego calcular correctamente el umbral. *(Esto último está mal)*

Al terminar de calibrar, apaga el ```LED13``` indicando que se terminó de calibrar. Y luego inicia la serial para mostrar en esta los valores mínimos y máximos que tomó cada sensor (función ```printCalibration```).

## Funcionamiento continuo del auto (loop)

```
void loop()
{
    //Serial.print(flag);
    //Serial.print(flag2);
    funBotones();
    readSensors();
    controlMotors();
    Serial.println();
}
```

Una vez se terminó de calibrar el auto, empieza el loop, en el cual se está comprobando constantemente el estado de los botones, lee los sensores y controla los motores.

#### Funcionamiento de los botones

```
void funBotones()
```

Es dificil explicar esta función línea por línea, pero lo que hace esta claro, trabaja con las banderas (```flag``` y ```flag2```) para controlar en que estado está el auto, si ambas son verdaderas, el auto puede empezar a funcionar, si no, tiene dos opciones.

La primera es seguir parpadeando ```LedOn``` para indicar que se puede arrancar el auto, cuando se toca ```bot1``` por 1 segundo el auto puede empezar a andar.

En el caso de ```bot```, si se mantiene apretado se ponen los motores al máximo.

#### Lectura de Sensores

```
void readSensors()
```

La función ```readSensors``` calcula la posición y el error del auto, mediante la lectura de los sensores.

```
Serial.print("Sensores: ");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
      umbral = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2; //cambiar para que los calcule en la calibracion
        if (sensorValues[i] <= umbral)
        {
            sensorValues[i] = 0;
        }
        else
        {
            sensorValues[i] = 1000;
        }
        Serial.print(sensorValues[i]); // lectura de sensores
        Serial.print('\t');
    }
````

**ERROR MUY GRAVE** los valores digitalizados nunca se usan, sino que cuando se llama a ‘readLineBlack’ se leen de vuelta los valores y los almacena en el array de sensorValues.

El último pedazo de código mostrado está al re pedo, como mucho puede servir para depurar pero todos los datos de la calibración no los usamos en ningún momento.

No es que va a modificar el funcionamiento, pero la librería ya hace automáticamente la lectura de la línea.

```
uint16_t position = qtr.readLineBlack(sensorValues);
error = position - 3500;
```

Como expliqué antes, esta función lee los valores de los sensores y calcula la posición de la línea, la cual va a ser un valor entre 0 y 7000. (Si el auto está en el primer sensor, va a retornar 0, y si está en el último 7000).

El error se calcula de esa manera ya que el centro de la línea sería 3500, si el error es positivo significa que está por encima de la línea, y si es negativo por debajo.

#### Control de Motores y Cálculo del PID

```
void controlMotors()
```

Esta función es la encargada de controlar los motores en función del error a través del sistema de control PID (Proportional-Integral-Derivative Control).

```
currentTime = millis(); // Obtener el tiempo actual
    
if (previousTime == 0) 
{  // Si es la primera vez que se ejecuta la función
  previousTime = currentTime;  // Actualizar previousTime
  return;  // Salir de la función para evitar división por cero en elapsedTime
}

elapsedTime = (double)(currentTime - previousTime) / 1000; // Calcular el tiempo transcurrido desde el cálculo anterior en segundos
```

Lo primero que hace ```controlMotors``` es obtener el tiempo actual del auto con la función ```millis()``` integrada en *Arduino*

Luego, si es la primera medición retorna la función para que en el cálculo de la derivada no se divida por cero.

**Nota importante**: En el calculo de la derivada nunca se va a dividir por cero, ya que millis nunca va a devolver cero debido a que antes ya paso toda la calibración, pero no está de mas chequear.

Luego calcula el tiempo que paso desde la última medición y lo guarda en ```elapsedTime```

```
cumError += error * elapsedTime;               // Calcular la integral del error
rateError = (error - lastError) / elapsedTime; // Calcular la derivada del error
  
motVel = Kp * error + Ki * cumError + Kd * rateError; // Calcular la salida del PID
````

Esta es la parte del cálculo del PID, la cual voy a retomar mas adelante.

```
lastError = error;          // Recordar el error actual
previousTime = currentTime; // Recordar el tiempo actual
```

En esta parte se guardan las distintas variables del PID para usarlas mas adelante al volver a calcularlo.

```
mAVel = Vel + motVel;
mBVel = Vel - motVel;
```

```mxVel```, como dije antes es la velocidad que toma el motor *x*, ahora, a uno se le suma y a otro se le resta la salida del PID, esto es ya que si uno tiene que doblar tanto para doblar, la otra rueda va a tener que frenar ese tanto para doblar, y como ```motVel``` puede ser negativo, se pueden invertir los signos que se muestran ahi.

Luego se llama a la funcion ```restrictMotorSpeed()``` que lo que hace es limitar la velocidad de los motores a valores seguros. **Nota**: A simple vista no parece tener problemas pero habría que pensar la lógica que toma esta función.

```
analogWrite(PWMM1B, mAVel);
analogWrite(PWMM2B, mBVel + ajuste);
```

En estas dos líneas se le manda lo calculado a cada motor.

#### Cálculo del PID

```math
PID = K_p · P + K_i · \int_{0}^{t_i} E(t)dt + K_d · E'(t)
```

Para empezar hay que ver que es cada termino.

##### Término Proporcional

En primer lugar tenemos el término **proporcional**, el cuál no es más que el error en sí, este término es **proporcional al error**, por lo que acá no hay ningún cálculo complejo.

```math
P = E(t) = error
```

##### Término Derivativo

Luego tenemos el término derivativo, para el cual te recomiendo ver el siguiente video: [Deducción de la Fórmula de la Derivada del Error](https://www.youtube.com/watch?v=ZYGA-PqmQr4)

Voy a resumirlo, básicamente el cálculo en nuestro código sale de la fórmula de la derivada.

```math
f'(x) = \lim_{\Delta x \to 0} \frac{f(x + \Delta x) - f(x)}{\Delta x}
```

Con esta fórmula hay un problema, en nuestro caso tenemos puntos sueltos, por lo que no tendremos la situación en la que la variacion del tiempo sea 0, por lo que la fòrmula de la derivada instantanea nos quedaría asi:

```math
E'(t_i) = \frac{E(t_i) - E(t_i - \Delta t)}{\Delta t}
 ```

En el código, $`E'(t_i)`$ es ```rateError```, $`E(t_i - \Delta t)`$ es ```lastError```, y $`\Delta t`$ es ```elapsedTime```, por lo que podemos poner esto en el código:

```
rateError = (error - lastError) / elapsedTime;
```

##### Término Integral

Para entender bien este término, les recomiendo ver el siguiente video: [Deducción de la fórmula de la integral del error](https://www.youtube.com/watch?v=I08we04KKHY)

Como antes, voy a tomar los puntos más importantes y trasladarlos acá.

En primer lugar, tenemos que entender que nuestra fórmula va a salir de la **suma de riemann**, la cual es la siguiente:

```math
A = \lim_{n \to \infty} \sum_{i=0}^n f(x_i) · \Delta x
```

La suma de riemann lo que intenta hacer es calcular el área bajo la curva $`f`$, dividiendola en $`n`$ rectángulos de base $`\Delta x`$ y evaluando la función en el $`i`$ rectángulo para conocer su altura y asi conocer el área de cada rectángulo y sumarlas, teniendo infinitos rectángulos podemos conocer el área exacta.

![image](https://github.com/user-attachments/assets/3d4c6af3-37bd-410a-bfd7-e4b8c76b3a71)

En el caso del seguidor de línea, no tenemos la situación límite de poder dividir nuestra función del error en infinitos rectángulos, por lo que nos va a quedar algo así:

```math
A = \sum_{i=0}^n E(t_i) · \Delta t
```

Para pasarlo a código, podemos decir que $`A`$ es ```cumError```, $`\sum_{i=0}^n`$ es ```+=``` ya que en la variable ya van a estar sumada toda el área anterior, $`E(t_i)`$ es ```error``` y como antes, $`\Delta t`$ es ```elapsedTime```, por lo que en el código vamos a escribir

```
cumError += error * elapsedTime;
```

Esta aproximación del área está bien, y debería funcionar, pero se podría hacer una mejor aproximación si en vez de dividir nuestro área en rectángulos la dividimos en trapecios, como se ve en la imagen.

![image](https://github.com/user-attachments/assets/c8944fef-fb97-492a-a64e-9436b8a1062a)

Ahora, nuestra fórmula pasaría a ser calculada como el área del trapecio, y no del rectángulo, por lo que nuestra nueva suma de riemann quedaría algo así:

```math
A = \sum_{i=0}^n \frac{E(t_i) + E(t_{i-1})}{2} · \Delta t
```

Ahora tenemos que pasar esta fórmula a C++, por lo que es todo igual agregando que $`E(t_{i-1})`$ es ```lastError```, la línea de código nos quedaría así:

```
cumError += ((error + lastError) / 2) * elapsedTime
```

## Guía para calcular los parámetros del PID

![image](https://github.com/user-attachments/assets/c76ef514-fede-4fed-934b-c2f266afbc11)

![image](https://github.com/user-attachments/assets/ddc4934f-7075-4f07-a9f1-977f39193e00)

![image](https://github.com/user-attachments/assets/d5b0891d-def0-4ce8-8432-21adde44a8b1)


## Contenido Útil

[Libreria QTR Sensors](https://github.com/pololu/qtr-sensors-arduino)

[VIDEO - Derivadas - El traductor de ingenieria](https://www.youtube.com/watch?v=_6-zwdrqD3U)

[VIDEO - Integrales - El traductor de ingenieria](https://www.youtube.com/watch?v=Ec-cGjh0Fr0&t=886s)

[VIDEO - PID Control - AerospaceControlsLab](https://www.youtube.com/watch?v=4Y7zG48uHRo)
