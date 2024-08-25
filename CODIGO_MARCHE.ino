#include <Arduino.h>
#include <EEPROM.h>
#include <QTRSensors.h>

//NICO: led integrado al arduino
#define LED13 13
//NICO: botones ??
#define Bot 3
#define Bot1 2
#define LedOn 4
//NICO: conexiones al puente H
#define PwmA 6 //NICO: PWM motor A
#define PwmB 5 //NICO: PWM motor B
#define Ain1 7 //NICO: Input 1 motor A
#define Ain2 8 //NICO: Input 2 motor A
#define Bin1 9 //NICO: Input 1 motor B
#define Bin2 10 //NICO: Input 2 motor B

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
int umbral = 500;
int position = 0;
int error = 0;

double mAVel = 0;
double mBVel = 0;
double motVel = 0;
double Vel = 60;
double  VelMin = 0;
bool flag = false;
bool flag2 = true;
// Variables globales adicionales para el control PID
unsigned long currentTime, previousTime = 0;
double elapsedTime;
double cumError = 0, rateError;
double lastError = 0;

// Parámetros del PID
double Kp = 0.006835825;
double Ki = 0.000001503;
double Kd = 0.00083216;

void saveCalibrationToEEPROM() {
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.println("Calibracion");

        Serial.print(i * 2);
        Serial.print(" LOW: ");
        Serial.println(qtr.calibrationOn.minimum[i] & 0xFF);
        Serial.print(i * 2); 
        Serial.print(" HIGH: ");
        Serial.println((qtr.calibrationOn.minimum[i] >> 8) & 0xFF);
        
        EEPROM.write(i * 2, qtr.calibrationOn.minimum[i] & 0xFF);  // Guarda el byte bajo
        EEPROM.write(i * 2 + 1, (qtr.calibrationOn.minimum[i] >> 8) & 0xFF);  // Guarda el byte alto
        EEPROM.write((i + SensorCount) * 2, qtr.calibrationOn.maximum[i] & 0xFF);
        EEPROM.write((i + SensorCount) * 2 + 1, (qtr.calibrationOn.maximum[i] >> 8) & 0xFF);
    }
}

void loadCalibrationFromEEPROM() {
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(i * 2);
        Serial.print(" Primero: ");
        Serial.println(EEPROM.read(i * 2) | (EEPROM.read(i * 2 + 1) << 8));
        Serial.print(i * 2); 
        Serial.print(" Segundo: ");
        Serial.println(EEPROM.read((i + SensorCount) * 2) | (EEPROM.read((i + SensorCount) * 2 + 1) << 8));
        qtr.calibrationOn.minimum[i] = EEPROM.read(i * 2) | (EEPROM.read(i * 2 + 1) << 8);
        qtr.calibrationOn.maximum[i] = EEPROM.read((i + SensorCount) * 2) | (EEPROM.read((i + SensorCount) * 2 + 1) << 8);
    }
}


// Esta función se encarga de la configuración inicial del sistema
void setup()
{
    configureSensors();
    configureMotor();
    configureIO();
    calibration();
//    if (digitalRead(Bot1) == LOW)
//    {
//      digitalWrite(LedOn, HIGH);
//      calibration();
//      saveCalibrationToEEPROM();
//      EEPROM.write(0, CALIBRATION_SENTINEL);
//      digitalWrite(LedOn, LOW);
//    }else{
//        if (EEPROM.read(0) == CALIBRATION_SENTINEL) {
//          Serial.println("Entro");
//          loadCalibrationFromEEPROM();
//          Serial.println("Salio");
//      } else {
//          digitalWrite(LedOn, HIGH);
//          calibration();
//          saveCalibrationToEEPROM();
//          EEPROM.write(0, CALIBRATION_SENTINEL);
//          digitalWrite(LedOn, LOW);
//      }
//    }
    
}

// Esta función se encarga de leer los sensores y controlar los motores
void loop()
{
    funBotones();
    readSensors();
    controlMotors();
}

// Función para el control de los botones
void funBotones()
{
    if (digitalRead(Bot1) == LOW)
    {
        analogWrite(PwmA, 0);
        analogWrite(PwmB, 0);
        flag2 = true;
        delay(2000);
        if (digitalRead(Bot1) == LOW)
        {
            analogWrite(PwmA, 255);
            analogWrite(PwmB, 255);
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
            analogWrite(PwmA, 0);
            analogWrite(PwmB, 0);
            delay(2000);

            if (digitalRead(Bot1) == LOW)
            {
                analogWrite(PwmA, 255);
                analogWrite(PwmB, 255);
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
    digitalWrite(Ain1, LOW);
    digitalWrite(Ain2, HIGH);
    digitalWrite(Bin1, LOW);
    digitalWrite(Bin2, HIGH);
    analogWrite(PwmA, 0);
    analogWrite(PwmB, 0);
}

// Proceso de calibración de los sensores
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

// Imprime los valores de calibración
void printCalibration()
{
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(qtr.calibrationOn.minimum[SensorCount - i]);
        Serial.print(' ');
    }
    Serial.println();

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
    pinMode(PwmA, OUTPUT);
    pinMode(PwmB, OUTPUT);
    pinMode(Ain1, OUTPUT);
    pinMode(Ain2, OUTPUT);
    pinMode(Bin1, OUTPUT);
    pinMode(Bin2, OUTPUT);
}


// Lee los valores de los sensores
void readSensors()
{
    uint16_t position = qtr.readLineWhite (sensorValues);
    error = position - 3500;
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] <= umbral)
        {
            sensorValues[i] = 0;
        }
        Serial.print(sensorValues[i]); // lectura de sensores
        Serial.print('\t');
    }
    Serial.print(position);
    Serial.print('\t');
    Serial.print('\t');
    Serial.print(error);
}

// Controla los motores basándose en los valores de los sensores
void controlMotors()
{
    currentTime = millis(); // Obtener el tiempo actual
    
    if (previousTime == 0) {  // Si es la primera vez que se ejecuta la función
    previousTime = currentTime;  // Actualizar previousTime
    return;  // Salir de la función para evitar división por cero en elapsedTime
  }
  
  elapsedTime = (double)(currentTime - previousTime) / 1000; // Calcular el tiempo transcurrido desde el cálculo anterior en segundos


  cumError += error * elapsedTime;               // Calcular la integral del error
  rateError = (error - lastError) / elapsedTime; // Calcular la derivada del error
  
  motVel = Kp * error + Ki * cumError + Kd * rateError; // Calcular la salida del PID

    lastError = error;          // Recordar el error actual
    previousTime = currentTime; // Recordar el tiempo actual

    mAVel = Vel + motVel;
    mBVel = Vel - motVel;

    restrictMotorSpeed();

    analogWrite(PwmA, mAVel);
    analogWrite(PwmB, mBVel);

    printMotorSpeed();

    delay(25);
}

// Restringe la velocidad del motor a valores seguros
void restrictMotorSpeed()
{
    if (mAVel < VelMin)
        mAVel = VelMin;
    if (mBVel < VelMin)
        mBVel = VelMin;
    if (mAVel > Vel)
        mAVel = Vel;
    if (mBVel > Vel)
        mBVel = Vel;
}

// Imprime la velocidad del motor
void printMotorSpeed()
{
    Serial.print('\t');
    Serial.print(mAVel);
    Serial.print('\t');
    Serial.print(mBVel);
    Serial.print('\t');
    Serial.println(motVel);
}
