#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

int gxAjuste = -3;
int gyAjuste = 1;
int gzAjuste = 0;

int axAjuste = 9;
int ayAjuste = 0;
int azAjuste = 1;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Iniciar el MPU6050
  Serial.println("Inicializando el MPU6050...");
  mpu.initialize();

  // Verificar la conexión
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente.");
  } else {
    Serial.println("Error al conectar el MPU6050.");
    while (1);
  }
}

void loop() {
  // Variables para guardar los datos de aceleración y giroscopio
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t tx, ty, tz;

  // Obtener lecturas del acelerómetro
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getAcceleration(&tx, &ty, &tz);

  // Obtener lecturas del giroscopio
  mpu.getRotation(&gx, &gy, &gz);

  // Convertir los datos crudos a unidades físicas
  // Sensibilidad: ±2g para acelerómetro y ±250°/s para giroscopio
  ax = ((ax / 16384.0) * 9.81) - axAjuste;   // 16384 LSB/g para ±2g
  ay = ((ay / 16384.0) * 9.81) - ayAjuste;   // despues lo multiplico por 9.81m/s2 (gravedad de la tierra) para que de en m/s2.
  az = ((az / 16384.0) * 9.81) - azAjuste;

  gx = (gx / 131.0) - gxAjuste; // 131 LSB/°/s para ±250°/s
  gy = (gy / 131.0) - gyAjuste;
  gz /= 131.0;

  // Calculo para obtener el angulo de inclinacion en X y en Y.
  float angX = atan (tx / sqrt(pow (ay, 2) + pow(tz, 2)))*(180.0 / 3.14);
  float angY = atan (ty / sqrt(pow (ax, 2) + pow(tz, 2)))*(180.0 / 3.14);

  // Imprimir datos del acelerómetro
  Serial.print("Aceleración [X: ");
  Serial.print(ax);
  Serial.print(" Y: ");
  Serial.print(ay);
  Serial.print(" Z: ");
  Serial.print(az);
  Serial.print("]");
  Serial.print("\t");

  // Imprimir datos del giroscopio
  Serial.print("Giroscopio [X: ");
  Serial.print(gx);
  Serial.print(" Y: ");
  Serial.print(gy);
  Serial.print(" Z: ");
  Serial.print(gz);
  Serial.print("]");
  Serial.print("\t");

  // Imprimir angulos de inclinación
  Serial.print("Angulo de inclinación [X: ");
  Serial.print(angX);
  Serial.print(" Y: ");
  Serial.print(angY);
  Serial.println("]");

  delay(100); // Esperar 500 ms para la siguiente lectura
}
