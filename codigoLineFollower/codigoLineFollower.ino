#include <Arduino.h>
#include <EEPROM.h>
#include <QTRSensors.h>

#define LED13 13 
#define Bot 8 
#define Bot1 2 
#define LedOn 13 
#define PWMM1B 11 //Para adelante
#define PWMM1A 10 //Para atras
#define ENM1 4
#define ENM2 9
#define PWMM2A 3 //Para atras
#define PWMM2B 5 //Para adel


#define CALIBRATION_SENTINEL 0x55  // Un valor específico para detectar si hay datos

// declarar funciones
void configureSensors();
void configureMotor();
void calibration();
void printCalibration();
void configureIO();
void funBotones();
void readSensors();
void controlMotors();
void restrictMotorSpeed();
void printMotorSpeed();

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int position = 0;
int error = 0;

double motVel = 0;
double Vel = 70; // Valor maximo para la velocidad del motor 
double VelMin = 0; // Valor minimo para la velocidad del motor, solo usar en casos de debugging
double mAVel = Vel;
double mBVel = Vel;

bool flag = false;
bool flag2 = true;
// Variables globales adicionales para el control PID
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double cumError = 0, rateError;
double lastError = 0;

// Parámetros del PID
double Kp = 0.04; 
double Ki = 0;
double Kd = 0.01;

//ajuste para motores distintos
int ajuste = 10;

// Esta función se encarga de la configuración inicial del sistema
void setup()
{
    configureIO();
    configureSensors();
    configureMotor();
    calibration();
}

// Esta función se encarga de leer los sensores y controlar los motores
void loop()
{
    funBotones();
    readSensors();
    controlMotors();
    Serial.println();
}

// Función para el control de los botones
void funBotones()
{
    if (digitalRead(Bot1) == LOW)
    {
        analogWrite(PWMM1A, 0);
        analogWrite(PWMM2A, 0);
        flag2 = true;
        delay(2000);
        if (digitalRead(Bot1) == LOW)
        {
            //analogWrite(PWMM1B, 255);
            //analogWrite(PWMM2B, 255);
            flag2 = true;
            flag = true;
            delay(1000);
        }
    }

    while (flag2 == true)
    {
        while (flag == true)
        {
            if (digitalRead(Bot1) == LOW)
            {
                flag = false;
            }
        }
        digitalWrite(LedOn, HIGH);
        delay(100);
        digitalWrite(LedOn, LOW);
        delay(100);

        if (digitalRead(Bot) == LOW)
        {
            flag2 = false;
            digitalWrite(LedOn, HIGH);
            while (digitalRead(Bot) == LOW)
            {
                /* espera */
            }
        }
        if (digitalRead(Bot1) == LOW)
        {
            analogWrite(PWMM1A, 0);
            analogWrite(PWMM2A, 0);
            delay(2000);

            if (digitalRead(Bot1) == LOW)
            {
                analogWrite(PWMM1A, 255);
                analogWrite(PWMM2A, 255);
                flag = true;
                delay(1000);
            }
        }
    }
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
    pinMode(LED13, OUTPUT);
    digitalWrite(LED13, HIGH);

    //const int calVel = 55; // Velocidad de motores en calibracion

    for (uint16_t i = 0; i < 150; i++)
    {
      /*if (i % 20 == 0)
      {
        analogWrite(PWMM1B, calVel);
        analogWrite(PWMM2B, calVel + ajuste);
        analogWrite(PWMM1A, 0);
        analogWrite(PWMM2A, 0);
      }
      else if (i % 10 == 0)
      {
        analogWrite(PWMM1B, 0);
        analogWrite(PWMM2B, 0);
        analogWrite(PWMM1A, calVel);
        analogWrite(PWMM2A, calVel + ajuste);
      }*/ // Todo este condicional es para calibracion automatica pero por ahora es al pedo
      qtr.calibrate();
    }
    /*
    analogWrite(PWMM1B, 0);
    analogWrite(PWMM2B, 0);
    */
    digitalWrite(LED13, LOW);
    Serial.begin(9600);
    printCalibration();
}

// Imprime los valores de calibración
void printCalibration()
{
  Serial.print("calibration on minimum: ");
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(qtr.calibrationOn.minimum[SensorCount - i]);
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


// Lee los valores de los sensores
void readSensors()
{
    //uint16_t position = qtr.readLineBlack(sensorValues);
    //error = position - 3500;
    Serial.print("Sensores: ");
    uint16_t position = qtr.readLineBlack(sensorValues);
    error = position - 3500;
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
  
  //analogWrite(PWMM1B, mAVel);
  //analogWrite(PWMM2B, mBVel + ajuste);
  //el ajuste es xq los motores son distintos
  printMotorSpeed();

  //delay(25);
}

// Restringe la velocidad del motor a valores seguros
void restrictMotorSpeed()
{
    if (mAVel < VelMin) mAVel = VelMin;
    if (mBVel < VelMin) mBVel = VelMin;
    if (mAVel > Vel) mAVel = Vel;
    if (mBVel > Vel) mBVel = Vel;
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
