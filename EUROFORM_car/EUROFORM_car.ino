/*
 * 
 *  13/02/2020
 *  created by viciopoli01
 *   
 *  email: polivico@gmail.com
 * 
*/

#include "defines.h"
#include <NewPing.h>

// HC SR 04

NewPing sonar(trigPin, echoPin, MAX_DISTANCE);

//PID parameters

double Kp = 4, Ki = 0, Kd = 1;

double SetPoint, distanza, velocit;

void setup() {


  //set the initial speed
  setupMotors();

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  distanza = prendiDistanza();

  SetPoint = 10; // 10 cm from the obstacle
  
  //turn the PID on
  
  Serial.begin(9600); // Starts the serial communication

  Serial.println("Initialized");

  delay(50);
}

void loop() {
  distanza = prendiDistanza();
  
  Serial.print("Nuova Distanza ");
  Serial.print(distanza);
  Serial.print("; ");
  
  if (distanza < 10) {
    turn_and_measure();
  } else {
    velocit = calcolaPID(distanza);
    muovi_motore(velocit);

    Serial.print("Nuova velocità ");
    Serial.print(velocit);
    Serial.println(";");    
  }
}

double prendiDistanza(){
  double d = sonar.ping_cm();
  return d<400?(d<2?0:d):400;
}

void turn_and_measure() {
  turn_sinistra(150);
  if (prendiDistanza() < 10)
    turn_and_measure();
  return;
}


/*
*
*
*
*   H-bridge
*
*
*/

double gain_sinistra, gain_destra,gain=0;

void setupMotors(){
  pinMode(EN_destra,OUTPUT);
  pinMode(EN_sinistra,OUTPUT);
  
  pinMode(L1_r,OUTPUT);
  pinMode(L2_r,OUTPUT);
  
  pinMode(L1_l,OUTPUT);
  pinMode(L2_l,OUTPUT);


  if(gain = 0){
    gain_sinistra=1;
    gain_destra=1;
  }
  if(gain<0)gain_sinistra=gain<-1?-1:gain;
  if(gain>0)gain_destra=gain<-1?-1:gain;
}

void muovi_motore(int velocit){
  if(velocit>0)
    motore_avanti(velocit);
  if(velocit<0)
    motore_avanti(-velocit);
}

void motore_avanti(int velocit){
  digitalWrite(L1_r, HIGH);
  digitalWrite(L2_r,LOW);
  
  digitalWrite(L1_l, HIGH);
  digitalWrite(L2_l,LOW);
  

  analogWrite(EN_destra,velocit*gain_destra);
  analogWrite(EN_sinistra,velocit*gain_sinistra);
}

void motore_indietro(int velocit){
  digitalWrite(L1_r, LOW);
  digitalWrite(L2_r,HIGH);
  
  digitalWrite(L1_l, LOW);
  digitalWrite(L2_l,HIGH);

  analogWrite(EN_destra,velocit);
  analogWrite(EN_sinistra,velocit);
}



void turn_sinistra(int t){
  digitalWrite(L1_r, HIGH);
  digitalWrite(L2_r,LOW);
  
  digitalWrite(L1_l, LOW);
  digitalWrite(L2_l,HIGH);

  analogWrite(EN_destra,255);
  analogWrite(EN_sinistra,255);

  delay(t);
}



/*
 * 
 * 
 *  PID
 * 
 * 
*/


int errore, precedenteErrore, cumulativoErrore=0, derivativoErrore=0;

unsigned long correnteTempo, precedenteTempo=0;
double deltaTempo;
int PID=0;
int calcolaPID(int val){
  correnteTempo = millis();
  deltaTempo=(double)(correnteTempo-precedenteTempo);
  
  errore = SetPoint-val;

  if(PID<255 && PID>-255 && ((errore>0)-(errore<0))==((precedenteErrore>0)-(precedenteErrore<0)))
    cumulativoErrore += errore;
    
  derivativoErrore = (errore - precedenteErrore)/deltaTempo;

  Serial.println(cumulativoErrore);
  

  PID = (int)(Kp*errore + Ki*cumulativoErrore + Kd*derivativoErrore);
  
  precedenteErrore = errore;
  precedenteTempo = correnteTempo;

  if(PID<0 && PID>-100)
    PID = -100;
  if(PID>0 && PID<100)
    PID = 100;
    
  if(PID>255)
    PID = 255;
  if(PID<-255)
    PID = -255;
    
  Serial.println(PID);
  
  return PID;
}
