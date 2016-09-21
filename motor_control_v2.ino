#include <PWM.h>

int motor1=9;
int motor1_dirA=2;
int motor1_dirB=4;
int motor1_EN=6;

int motor2=10;
int motor2_dirA=7;
int motor2_dirB=8;
int motor2_EN=12;

void setup()
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


void loop()
{
  digitalWrite(motor1_EN, HIGH);
  digitalWrite(motor1_dirA, HIGH);
  digitalWrite(motor1_dirB, LOW);
  pwmWriteHR(motor1, 65535);

  digitalWrite(motor2_EN, HIGH);
  digitalWrite(motor2_dirA, HIGH);
  digitalWrite(motor2_dirB, LOW);
  pwmWriteHR(motor2, 15000);
}
