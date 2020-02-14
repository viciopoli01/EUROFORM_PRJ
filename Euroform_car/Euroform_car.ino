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

double SetPoint, distance, Speed;

void setup() {


  //set the initial speed
  setupMotors();

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  distance = getDistance();

  SetPoint = 10; // 10 cm from the obstacle
  
  //turn the PID on
  
  Serial.begin(9600); // Starts the serial communication

  Serial.println("Initialized");

  delay(50);
}

void loop() {
  distance = getDistance();
  
  Serial.print("New Distance ");
  Serial.print(distance);
  Serial.print("; ");
  
  if (distance < 10) {
    turn_and_measure();
  } else {
    Speed = computePID(distance);
    move_motor(Speed);

    Serial.print("New Speed ");
    Serial.print(Speed);
    Serial.println(";");    
  }
}

double getDistance(){
  double d = sonar.ping_cm();
  return d<400?(d<2?0:d):400;
}

void turn_and_measure() {
  turn_left(150);
  if (getDistance() < 10)
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

double gain_left, gain_right,gain=0;

void setupMotors(){
  pinMode(EN_right,OUTPUT);
  pinMode(EN_left,OUTPUT);
  
  pinMode(L1_r,OUTPUT);
  pinMode(L2_r,OUTPUT);
  
  pinMode(L1_l,OUTPUT);
  pinMode(L2_l,OUTPUT);


  if(gain = 0){
    gain_left=1;
    gain_right=1;
  }
  if(gain<0)gain_left=gain<-1?-1:gain;
  if(gain>0)gain_right=gain<-1?-1:gain;
}

void move_motor(int Speed){
  if(Speed>0)
    motor_forward(Speed);
  if(Speed<0)
    motor_forward(-Speed);
}

void motor_forward(int Speed){
  digitalWrite(L1_r, HIGH);
  digitalWrite(L2_r,LOW);
  
  digitalWrite(L1_l, HIGH);
  digitalWrite(L2_l,LOW);
  

  analogWrite(EN_right,Speed*gain_right);
  analogWrite(EN_left,Speed*gain_left);
}

void motor_backward(int Speed){
  digitalWrite(L1_r, LOW);
  digitalWrite(L2_r,HIGH);
  
  digitalWrite(L1_l, LOW);
  digitalWrite(L2_l,HIGH);

  analogWrite(EN_right,Speed);
  analogWrite(EN_left,Speed);
}



void turn_left(int t){
  digitalWrite(L1_r, HIGH);
  digitalWrite(L2_r,LOW);
  
  digitalWrite(L1_l, LOW);
  digitalWrite(L2_l,HIGH);

  analogWrite(EN_right,255);
  analogWrite(EN_left,255);

  delay(t);
}



/*
 * 
 * 
 *  PID
 * 
 * 
*/


int error, previousError, integralError=0, derivativeError=0;

unsigned long currentTime, previousTime=0;
double deltaTime;
int PID=0;
int computePID(int val){
  currentTime = millis();
  deltaTime=(double)(currentTime-previousTime);
  
  error = SetPoint-val;

  if(PID<255 && PID>-255 && ((error>0)-(error<0))==((previousError>0)-(previousError<0)))
    integralError += error;
    
  derivativeError = (error - previousError)/deltaTime;

  Serial.println(integralError);
  

  PID = (int)(Kp*error + Ki*integralError + Kd*derivativeError);
  
  previousError = error;
  previousTime = currentTime;

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
