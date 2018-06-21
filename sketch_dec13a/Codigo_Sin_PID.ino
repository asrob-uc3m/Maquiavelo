#include <QTRSensors.h>
#define NUM_SENSORS   8     // numero de sensores usados
#define TIMEOUT       2500  // esperar 2.5 ms para tener una respuesta del sensado
#define EMITTER_PIN   8     // este pin controla el led on del los sensores (enciende y apaga)
#define NEGRO         2500
#define BLANCO        500
#define CENTRO        3500  // indica que el sensor esta en el centro de la linea
int recto;

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


int pwma = 5;
int pwmb = 3;
int ai1 = 6;
int ai2 = 7;
int bi1 = 4;
int bi2 = 2;

// Inicialización
void setup() {
  
delay(500);   
recto = 1;   

for (int i = 0; i < 200; i++)  // la calibracion se lleva a cabo por 5 segundos 
{
sensor.calibrate();       // funcion para calibrar los sensores
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

analogWrite(pwma,50);
analogWrite(pwmb,50);
}

// Rutina
void loop() { 
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  unsigned int position = sensor.readLine(sensorValues);
  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
  Serial.print(sensorValues[i]);
  Serial.print('\t');
  }
  Serial.println();
  //Serial.println(position); // comment this line out if you are using raw values 
  delay(50);

  digitalWrite(ai1,HIGH);
  digitalWrite(ai2,LOW);
  digitalWrite(bi1,HIGH);
  digitalWrite(bi2,LOW);
  
  //variables control sentido

  int izqda = 0;
  int dcha = 0;
  int dcha_mas;
  int izqda_mas;
  
 if (sensorValues[0] <= BLANCO && sensorValues[1] <= BLANCO && sensorValues[2] <= BLANCO   &&  sensorValues[3] <= BLANCO && recto == 1){
    recto = 0;
    izqda = 0;
    dcha = 1;
  } else if (sensorValues[5] <= BLANCO && sensorValues[6] <= BLANCO && sensorValues[7] <= BLANCO   &&  sensorValues[4] <= BLANCO && recto == 1 ){
    recto = 0; 
    izqda = 1;
    dcha = 0; 
  } else if (sensorValues[3] > BLANCO && sensorValues[4] > BLANCO){
    recto = 1;
    izqda = 0;
    dcha = 0;
  }
  Serial.print(recto);
  Serial.print(izqda);
  Serial.print(dcha);
  
  //recto
  if (recto){
    analogWrite(pwma,50);
    analogWrite(pwmb,50);
    delay(10);
  } else if (izqda){
    analogWrite(pwma,100);
    analogWrite(pwmb,25);
    delay(10);
  }else if (dcha){
    analogWrite(pwma,25);
    analogWrite(pwmb,100);
    delay(10);
  }
}
