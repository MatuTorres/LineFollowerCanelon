### A pedido del público...
# Explicación "Código Marche"
Así vemos lo que está bien y lo que está mal.

### Declaración de pines y valores iniciales

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





