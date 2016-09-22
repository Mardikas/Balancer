#include <PWM.h>

#define MAX 255

void init_motors();
void setM1Speed(int speed); // Set speed for M1.
void setM2Speed(int speed); // Set speed for M2.
void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
void setM1Brake(int brake); // Brake M1. 
void setM2Brake(int brake); // Brake M2.
void setBrakes(int m1Brake, int m2Brake);


int motor1=9;
int motor1_dirA=2;
int motor1_dirB=4;
int motor1_EN=6;

int motor2=10;
int motor2_dirA=7;
int motor2_dirB=8;
int motor2_EN=12;

void setup() {
  // put your setup code here, to run once:
init_motors();

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(motor1_EN, HIGH);
digitalWrite(motor2_EN, HIGH);
 setSpeeds(255, 255);
}

void init_motors()
{
 InitTimersSafe();
 Timer1_SetFrequency(20000);
 pinMode(motor1, OUTPUT);
 pinMode(motor1_dirA, OUTPUT);
 pinMode(motor1_dirB, OUTPUT);
 pinMode(motor1_EN, OUTPUT);
 

 pinMode(motor2, OUTPUT);
 pinMode(motor2_dirA, OUTPUT);
 pinMode(motor2_dirB, OUTPUT);
 pinMode(motor2_EN, OUTPUT);
 
}

void setM1Speed(int speed){

  unsigned char reverse = 0;
  if(speed < 0){
    speed=-speed;
    reverse = 1;
  }
  if(speed>MAX){
    speed=MAX;
  }
  if(speed==0){
    digitalWrite(motor1_dirA, LOW);
    digitalWrite(motor1_dirB, LOW);
  }
  else if(reverse==1){
    digitalWrite(motor1_dirA, HIGH);
    digitalWrite(motor1_dirB, LOW);
  }
    else{
    digitalWrite(motor1_dirA, LOW);
    digitalWrite(motor1_dirB, HIGH);
  }
  pwmWrite(motor1, speed);
}

void setM2Speed(int speed){

  
  
  unsigned char reverse = 0;
  if(speed < 0){
    speed=-speed;
    reverse = 1;
  }
  if(speed>MAX){
    speed=MAX;
  }
  if(speed==0){
    digitalWrite(motor2_dirA, LOW);
    digitalWrite(motor2_dirB, LOW);
  }
  else if(reverse==1){
    digitalWrite(motor2_dirA, HIGH);
    digitalWrite(motor2_dirB, LOW);
  }
    else{
    digitalWrite(motor2_dirA, LOW);
    digitalWrite(motor2_dirB, HIGH);
  }
  pwmWrite(motor2, speed);
}

void setSpeeds(int m1Speed, int m2Speed){
   setM1Speed(m1Speed);
   setM2Speed(m2Speed);
}
void setM1Brake(int brake){
  
  if(brake<0){
  brake=-brake;
  }

  if(brake>MAX){
    brake=MAX;
  }
  pwmWrite(motor1, brake);
  digitalWrite(motor1_dirA, LOW);
  digitalWrite(motor1_dirB, LOW);
}
void setM2Brake(int brake){
  
  if(brake<0){
  brake=-brake;
  }

  if(brake>MAX){
    brake=MAX;
  }
  pwmWrite(motor2, brake);
  digitalWrite(motor2_dirA, LOW);
  digitalWrite(motor2_dirB, LOW);
}
void setBrakes(int m1Brake, int m2Brake){
  setM1Brake(m1Brake);
  setM2Brake(m2Brake);
}

