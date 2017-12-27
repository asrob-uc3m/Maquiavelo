#include <QTRSensors.h>
#define NUM_SENSORS   8     // numero de sensores usados
#define TIMEOUT       2500  // esperar 2.5 ms para tener una respuesta del sensado
#define EMITTER_PIN   8     // este pin controla el led on del los sensores (enciende y apaga)
#define CENTRO        3500  // Posicion central de la linea 
#define VEL_RECT      50    //velocidad recto (CAMBIAR)
//sensores 
char S1 = A5;
char S2 = A4;
char S3 = A3;
char S4 = A2;
char S5 = A1;
char S6 = A0;
char S7 = 11;
char S8 = 12;
unsigned char pines[] = {S1,S2,S3,S4,S5,S6,S7,S8};
//aqui se pone el numero de los pines conectados a los sensores
QTRSensorsRC sensor(pines,
NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

  //variables PID
int error_anterior = 0; // Para el PID (derivativo)
int error_acumulado = 0; // Para el PID (integral)

  //asignacion motores
int pwma = 5;
int pwmb = 3;
int ai1 = 6;
int ai2 = 7;
int bi1 = 4;
int bi2 = 2;

// Inicialización
void setup() {
  
  delay(500);    
  
  for (int i = 0; i < 200; i++)  // la calibracion se lleva a cabo por 5 segundos 
  {
    sensor.calibrate();       // funcion para calibrar los sensores (quitar y poner la mano rapidamente)
  }  
  // imprime la calibracion minima de los sensores
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensor.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  
  delay(5000);
  Serial.println();
  
  // imprime la calibracion maxima de los sensores
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensor.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  
  Serial.println();
  Serial.println();
  delay(1000);

  //declaracion de pines
  pinMode(pwma,OUTPUT);
  pinMode(pwmb,OUTPUT);
  pinMode(ai1,OUTPUT);
  pinMode(ai2,OUTPUT);
  pinMode(bi1,OUTPUT);
  pinMode(bi2,OUTPUT);
  
  pinMode(S1,INPUT);
  pinMode(S2,INPUT);
  pinMode(S3,INPUT);
  pinMode(S4,INPUT);
  pinMode(S5,INPUT);
  pinMode(S6,INPUT);
  pinMode(S7,INPUT);
  pinMode(S8,INPUT);
  
  analogWrite(pwma,VEL_RECT);
  analogWrite(pwmb,VEL_RECT);
}


void loop() { 
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  int position = sensor.readLine(sensorValues);
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
  Serial.print(sensorValues[i]);
  Serial.print('\t');
  }
  Serial.println();
  Serial.println(position); 
  delay(50);
  
  digitalWrite(ai1,HIGH);
  digitalWrite(ai2,LOW);
  digitalWrite(bi1,HIGH);
  digitalWrite(bi2,LOW);
  
  //********* PID *************//                        
  
  // Ganancias del PID (PUESTO A OJO!!!)
  float kp = 0.01; // Proporcional
  float ki = 0.001; // Integral
  float kd = 0.01; // Derivativa
  
  // Queremos que position sea lo mas cercano a CENTRO
  int error = CENTRO - position;
  error_acumulado += error;
  //[-1000,1000]
  error_acumulado = min(max(-1000, error_acumulado),1000);
  
  float PID_proporcional = kp * error;
  float PID_integral = ki * error_acumulado;
  float PID_derivativo = kd * (error - error_anterior);
  
  int correccion_pid = PID_proporcional + PID_integral + PID_derivativo;
  error_anterior = error;
  
  //Suponiendo que el motor A esta a la izquierda, y que cuando vas para la izquierda el error es negativo
  //Entonces correccion_pid es negativo tambien, y entonces el motor izquierdo tiene que ir mas rapido.
  //COMPROBAR si A es izquierda
   
  int v_motor_izq = velocidad_recto - correccion_pid;
  int v_motor_der = velocidad_recto + correccion_pid;
  
  // Nunca menor que 0 ni mayor que 255
  v_motor_izq = min(max(0, v_motor_izq), 255);
  v_motor_der = min(max(0, v_motor_der), 255);
  analogWrite(pwma, v_motor_izq);
  analogWrite(pwmb, v_motor_der);

}
