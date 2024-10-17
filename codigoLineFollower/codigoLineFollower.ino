#include <Arduino.h>
#include <QTRSensors.h>

// Definicion de pines
#define bot 8 
#define led 13 
#define PWMM1B 11 //Para adelante
#define PWMM1A 10 //Para atras
#define ENM1 4
#define ENM2 9
#define PWMM2A 3 //Para atras
#define PWMM2B 5 //Para adelante

// Constantes de sensado y posicion
#define SensorCount 8
#define errorCount 3 // Cantidad de errores para calcular promedio y eliminar ruido

// Constantes de los motores y velocidades
#define Vel 71 // Valor base para la velocidad del motor 
#define VelMin 0 // Valor minimo de velocidad
#define VelMax 255 // Valor maximo de velocidad

// Variables de sensado y posicion
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
int position = 0;
int error = 0;

// Velocidades
double motVel = 0; // Salida del PID
double mAVel = Vel; // Velocidad motor A
double mBVel = Vel; // Velocidad motor B

// Variables globales adicionales para el control PID
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double cumError = 0, rateError;
double lastError = 0;

// Parámetros del PID
double Kp = 0.0401; 
double Ki = 0.00145; 
double Kd = 0.0018;

// Variables para los botones
int prevState = 0;
int flag = 0; // Si flag es 1, estan los dos motores al palo
unsigned long timePressed = 0;
unsigned long startedPressing = 0;
int ledState = 0;
unsigned long lastBlink = 0;

// Esta función se encarga de la configuración inicial del sistema
void setup()
{
  configureIO();
  configureSensors();
  configureMotor();
  calibration();

  //Serial.begin(9600);
  //printCalibration();

  while (!funBotones()){} // Si retorna 0 sigue esperando, si retorna 1 el auto empieza a andar
}

// Esta función se encarga de leer los sensores y controlar los motores
void loop()
{
  readSensors();

  controlMotors();
  applySpeed();

  //printSensors();
  //printMotorSpeed();
}

// Función para el control de los botones
int funBotones()
{
  unsigned long actualTime = millis();
  int state = !digitalRead(bot);

  //Serial.println(state);
  
  if (state == 0 && prevState == 0) // No se esta apretando el boton
  {
    if (ledState == 0 && actualTime - lastBlink > 182) // En honor a blink-182
    {
      lastBlink = actualTime;
      ledState = 1;
      digitalWrite(led, HIGH);
    }
    else if (ledState == 1 && actualTime - lastBlink > 182)
    {
      lastBlink = actualTime;
      ledState = 0;
      digitalWrite(led, LOW);
    }
  }
  else if (state == 1 && prevState == 0) // Se empezo a apretar el boton
  {
    prevState = 1;
    startedPressing = actualTime;
  }
  else if (state == 1 && prevState == 1) // Se esta apretando el boton
  {
    timePressed = actualTime - startedPressing;

    if (timePressed < 2000)
    {
      digitalWrite(led, LOW);
    }
    else
    {
      digitalWrite(led, HIGH);
    }
  }
  else if (state == 0 && prevState == 1) // Se dejo de apretar el boton
  {
    prevState = 0;
    timePressed = actualTime - startedPressing;
  }
  
  if (state == 0 && timePressed != 0) // Si esta apretado seguir esperando
  {
    // Manejo de las flags
    if (timePressed < 2000 && flag == 0)
    {
      flag = 1;
      timePressed = 0;
      analogWrite(PWMM1B, 255);
      analogWrite(PWMM2B, 255);
    }
    else if (timePressed < 2000 && flag == 1)
    {
      flag = 0;
      timePressed = 0;
      analogWrite(PWMM1B, 0);
      analogWrite(PWMM2B, 0);
    }
    else if (timePressed >= 2000)
    {
      return 1;
    }
  }

  return 0;
}

// Configuración inicial de los sensores
void configureSensors()
{
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
}

// Configuración inicial del motor
void configureMotor()
{
    digitalWrite(PWMM2A, LOW);
    digitalWrite(ENM1, HIGH);
    digitalWrite(PWMM1A, LOW);
    digitalWrite(ENM2, HIGH);
    analogWrite(PWMM1B, 0);
    analogWrite(PWMM2B, 0);
}

// Proceso de calibración de los sensores
void calibration()
{
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(led, LOW);
}

// Imprime los valores de calibración
void printCalibration()
{
  Serial.print("calibration on minimum: ");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
      Serial.print(qtr.calibrationOn.minimum[i]);
      Serial.print(' ');
  }
  Serial.println();
  Serial.print("calibration on maximum: ");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
     Serial.print(qtr.calibrationOn.maximum[i]);
     Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(500);
}

// Configura los pins de entrada/salida
void configureIO()
{
    pinMode(led, OUTPUT);
    pinMode(bot, INPUT_PULLUP);
    pinMode(PWMM1A, OUTPUT);
    pinMode(PWMM2A, OUTPUT);
    pinMode(ENM1, OUTPUT);
    pinMode(ENM2, OUTPUT);
    pinMode(PWMM1B, OUTPUT);
    pinMode(PWMM2B, OUTPUT);
}


// Lee los valores de los sensores
void readSensors()
{
  error = 0;
  for (int i = 0; i < errorCount; i++)
  {
    uint16_t position = qtr.readLineWhite(sensorValues);
    error += position - 3500;
  }
  error /= errorCount;
}

void printSensors()
{
  Serial.print("Sensores: ");
  for (int i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.print("Pos: ");
  Serial.print(position);
  Serial.print('\t');
  Serial.print("Err: ");
  Serial.print(error);
  Serial.print('\t');
}

// Controla los motores basándose en los valores de los sensores
void controlMotors()
{
  currentTime = millis(); // Obtener el tiempo actual
    
  if (previousTime == 0) 
  {  // Si es la primera vez que se ejecuta la función
    previousTime = currentTime;  // Actualizar previousTime
    return;  // Salir de la función para evitar división por cero en elapsedTime
  }
  
  elapsedTime = (double)(currentTime - previousTime) / 1000; // Calcular el tiempo transcurrido desde el cálculo anterior en segundos

  //cumError += error * elapsedTime; // Calcular la integral del error
  cumError += ((error + lastError) / 2) * elapsedTime; // Esta es otra formula, teoricamente mas precisa
  rateError = (error - lastError) / elapsedTime; // Calcular la derivada del error
  
  motVel = Kp * error + Ki * cumError + Kd * rateError; // Calcular la salida del PID

  lastError = error;          // Recordar el error actual
  previousTime = currentTime; // Recordar el tiempo actual

  mAVel = Vel + motVel;
  mBVel = Vel - motVel;

  restrictMotorSpeed();
}

void applySpeed()
{
  // Aplicar velocidades
  if (mAVel >= 0) 
  {
      analogWrite(PWMM1B, mAVel);
      analogWrite(PWMM1A, 0);
  } 
  else 
  {
      analogWrite(PWMM1A, -mAVel);  // El ajuste porque los motores son distintos
      analogWrite(PWMM1B, 0);
  }

  if (mBVel >= 0) 
  {
      analogWrite(PWMM2B, mBVel);
      analogWrite(PWMM2A, 0);
  } 
  else 
  {
      analogWrite(PWMM2A, -mBVel);  // El ajuste porque los motores son distintos
      analogWrite(PWMM2B, 0);
  }
}

// Restringe la velocidad del motor a valores seguros
void restrictMotorSpeed()
{
  mAVel = constrain(mAVel, VelMin, VelMax);
  mBVel = constrain(mBVel, VelMin, VelMax);
}

// Imprime la velocidad del motor
void printMotorSpeed()
{
    Serial.print('\t');
    Serial.print("mAVel: ");
    Serial.print(mAVel);
    Serial.print('\t');
    Serial.print("mBVel: ");
    Serial.print(mBVel);
    Serial.print('\t');
    Serial.print("motVel: ");
    Serial.println(motVel);
}

