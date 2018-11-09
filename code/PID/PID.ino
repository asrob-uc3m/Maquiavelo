#include <QTRSensors.h>
#define NUM_SENSORS   6     // numero de sensores usados
#define TIMEOUT       2500  // esperar 2.5 ms para tener una respuesta del sensado
#define EMITTER_PIN   12     // este pin controla el led on del los sensores (enciende y apaga)
#define CENTRO        3500  // Posicion central de la linea 
#define VEL_RECT      190    //velocidad recto (CAMBIAR)
//sensores
char S6 = 9;
char S5 = 10;
char S4 = A3;
char S3 = A2;
char S2 = A1;
char S1 = A0; //izq

unsigned char pines[] = {S1, S2, S3, S4, S5, S6};
//aqui se pone el numero de los pines conectados a los sensores
QTRSensorsRC sensor(pines,
                    NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

//variables PID
int error_anterior = 0; // Para el PID (derivativo)
int error_acumulado = 0; // Para el PID (integral)

//asignacion motores
int pwma = 11;
int pwmb = 3;
int ai1 = 2;
int ai2 = 4;
int bi1 = 5;
int bi2 = 6;

//boton
int boton = 8;

// Inicialización
void setup() {

  //declaracion de pines
  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(ai1, OUTPUT);
  pinMode(ai2, OUTPUT);
  pinMode(bi1, OUTPUT);
  pinMode(bi2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(boton, INPUT);

  //Serial.begin(9600);
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
}


void loop() {

  digitalWrite(LED_BUILTIN, HIGH);
  //while (!digitalRead(boton));
  delay(5000);
  Serial.print("Calibration...");
  for (int i = 0; i < 200; i++) {
    sensor.calibrate();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  Serial.print("GO!!!");
  digitalWrite(LED_BUILTIN, LOW);
  delay(5000);
  digitalWrite(LED_BUILTIN, HIGH);

  int position = sensor.readLine(sensorValues);


  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  position = sensor.readLine(sensorValues);
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  Serial.println(position);
  delay(500);



  digitalWrite(ai1, LOW);
  digitalWrite(ai2, HIGH);
  digitalWrite(bi1, HIGH);
  digitalWrite(bi2, LOW);


  unsigned long time;


  //********* PID *************//
  // Ganancias del PID (PUESTO A OJO!!!)
  float kp = 0.18; // Proporcional
  float ki = 0; // Integral
  float kd = 0.3; // Derivativa

  int error;


  float PID_proporcional;
  float PID_integral;
  float PID_derivativo;

  int correccion_pid;

  int v_motor_izq;
  int v_motor_der;



  while (1) {
    time = millis();

    position = sensor.readLine(sensorValues);



    // Queremos que position sea lo mas cercano a CENTRO
    error = CENTRO - position;
    error_acumulado += error;
    //[-1000,1000]
    error_acumulado = min(max(-1000, error_acumulado), 1000);

    PID_proporcional = kp * error;
    PID_integral = ki * error_acumulado;
    PID_derivativo = kd * (error - error_anterior);

    correccion_pid = PID_proporcional + PID_integral + PID_derivativo;
    error_anterior = error;

    v_motor_izq = VEL_RECT - correccion_pid;
    v_motor_der = VEL_RECT + correccion_pid;

    v_motor_izq = min(max(-255, v_motor_izq), 255);
    v_motor_der = min(max(-255, v_motor_der), 255);

    if(v_motor_izq > 0){
        digitalWrite(ai1, LOW);
        digitalWrite(ai2, HIGH);
    }
    else{
      digitalWrite(ai1, HIGH);
      digitalWrite(ai2, LOW);
    }

    if(v_motor_der > 0){
        digitalWrite(bi1, HIGH);
        digitalWrite(bi2, LOW);
    }
    else{
      digitalWrite(bi1, LOW);
      digitalWrite(bi2, HIGH);
    }
    
    
    analogWrite(pwma, abs(v_motor_izq));
    analogWrite(pwmb, abs(v_motor_der));

    while (millis() < time + 1); //1ms
  }
}
