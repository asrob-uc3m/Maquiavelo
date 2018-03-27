//VELOCISTA DEFINITIVO 
#include <QTRSensors.h>

//DECLARACION DE ENTRADAS///////////////
//BOTONES

const int btn1 = 8;

int cruzero = 120 ;//VELOCIDAD DE CURCERO ;D   

int pwma = 3;
int ain2 = 4;
int ain1 = 2;
int pwmb = 11;
int bin2 = 7;
int bin1 = 6;

int P = 0; 
int I = 0;
int D = 0;
int LAST = 0;
float vel;

#define NUM_SENSORS   6     // NUMERO DE SENSORES USADOS
#define TIMEOUT       2500     // TIEMPO DE ESPERA QUE PONE SALIDAS EN LOW
#define EMITTER_PIN   12    

//EL 12 ES EL 12, EL 14 = A0 Y SUSECIVAMENTE  
QTRSensorsRC qtrrc((unsigned char[]) {19, 18, 17, 16, 15, 14, 11, 12},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

unsigned int position = 0;

void setup()
{

  pinMode(btn1, INPUT);
    
  pinMode(pwma,OUTPUT);
  pinMode(ain1,OUTPUT);
  pinMode(ain2,OUTPUT);
  pinMode(pwmb,OUTPUT);
  pinMode(bin1,OUTPUT);
  pinMode(bin2,OUTPUT);
  
  pinMode(13,OUTPUT);

  //Serial.begin(9600);
  delay(1500);
  digitalWrite(13, HIGH);
   for (int j = 0; j < 40; j++)  
 {                                 
                    
  qtrrc.calibrate();     //FUNCIÓN QUE CALIBRA EL SENSOR

 }
 
 digitalWrite(13, LOW); 
                                                                   
 digitalWrite(ain1,LOW);
 digitalWrite(ain2,HIGH);
 
 digitalWrite(bin1,HIGH);
 digitalWrite(bin2,LOW);
 
 analogWrite(pwma,0);
 analogWrite(pwmb,0);
}


void loop()
{
  //Serial.println(ain1);
  if (digitalRead(btn1) == 1){
    for(;;){ 
  
      qtrrc.read(sensorValues);
      position = qtrrc.readLine(sensorValues, QTR_EMITTERS_ON, 0);
      // Serial.println(position);                                                                                                  
      P = ((position)-(2500)); /// ERROR ACUMULADO
  /////FRENOS////
      if(P < -2500){
        analogWrite(pwma,150); // VELOCIDAD PARA EL MOTOR DERECHO 
        analogWrite(pwmb,90); //  VELOCIDAD PARA EL MOTOR IZQUIERDO
        digitalWrite(ain1,LOW);   ///FRENTE
        digitalWrite(ain2,HIGH);
        digitalWrite(bin1,LOW);  ///RETROCEDE
        digitalWrite(bin2,HIGH);  
      } else if (P > 2500){
        analogWrite(pwma,150); // VELOCIDAD PARA EL MOTOR DERECHO
        analogWrite(pwmb,90); //  VELOCIDAD PARA EL MOTOR IZQUIERDO
        digitalWrite(ain1,HIGH);   ///RETROCEDE
        digitalWrite(ain2,LOW);
        digitalWrite(bin1,HIGH);  ///FRENTE
        digitalWrite(bin2,LOW);

      }
        /////////////////////////
      else{
        D = (P - LAST); /// ERROR MENOS EL ERROR ANTERIOR , DERIVATIVO
        I = (P + LAST); //INTEGRAL
        vel = (P*0.18) + (D*4 ) + (I*0.001 );// para velocidad 120//////estaba en 0.0925
        if(vel > cruzero) vel = cruzero;
        if(vel < -cruzero) vel = -cruzero;

        analogWrite(pwma,cruzero - vel); // VELOCIDAD PARA EL MOTOR DERECHO
        analogWrite(pwmb,cruzero + vel); //  VELOCIDAD PARA EL MOTOR IZQUIERDO

        digitalWrite(ain1,LOW);   ///FRENTE
        digitalWrite(ain2,HIGH);
        digitalWrite(bin1,HIGH);  ///FRENTE
        digitalWrite(bin2,LOW);
        LAST = P; //ERROR ANTERIOR 
        }
}////BUCLE INFINITO
}///PRESIONO BOTON
}///FIN DEL LOOP


