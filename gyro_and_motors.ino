//Somewhat ok PID settings: 9;10;500
//

#include <PWM.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define MAX 255
#define ANGLE_MECHANICAL_CALIBRATION 0.1
#define P_GAIN 18 //80;150 on somewhat twerking
#define D_GAIN 1000

MPU6050 mpu;

bool dmpReady = true;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void init_motors();
void setM1Speed(int speed); // Set speed for M1.
void setM2Speed(int speed); // Set speed for M2.
void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
void setM1Brake(int brake); // Brake M1. 
void setM2Brake(int brake); // Brake M2.
void setBrakes(int m1Brake, int m2Brake);

void gyro_init();

int motor1=9;
int motor1_dirA=2;
int motor1_dirB=4;
int motor1_EN=6;

int motor2=10;
int motor2_dirA=7;
int motor2_dirB=8;
int motor2_EN=12;

int error, P, D, last_P;
long I = 0;
int speed;

int I_gain, P_gain, D_gain;
char buff[4];
int offset;

void setup() {
  // put your setup code here, to run once:
init_motors();
   gyro_init();
   P=I=D=error=last_P=0;
P_gain=8;
D_gain=14;
I_gain=1000000;

offset=100;
   Serial.begin(9600);
}







void loop() {
  delay(1);

    if(Serial.available()>2){
      buff[0]=Serial.read();
      if(buff[0]=='p'){
        P_gain=Serial.parseInt();
        Serial.print("P_gain set to ");
        Serial.println(P_gain);
      }
      else if(buff[0]=='d'){
        D_gain=Serial.parseInt();
        Serial.print("D_gain set to ");
        Serial.println(D_gain);
      }
      else if(buff[0]=='i'){
        I_gain=Serial.parseInt();
        Serial.print("I_gain set to ");
        Serial.println(I_gain);
      }
      if(buff[0]=='o'){
        offset=Serial.parseInt();
        Serial.print("offset set to ");
        Serial.println(offset);
      }
  }
  
  // put your main code here, to run repeatedly:
      mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }

    
  digitalWrite(motor1_EN, HIGH);
  digitalWrite(motor2_EN, HIGH);

//calculate PID
  

  P=(ypr[1]-(10/offset))*100 ;
  I = I + last_P;
  D=P-last_P;
  if(P > 3 && P < 3) I = 0;
  if(I > 200) I = 200;
  if(I < -200) I = -200;
  speed = (P*P_gain + (I*100)/I_gain + D*D_gain);
  
  D=P-last_P;
  //error=(((P*P_GAIN)+(D*D_GAIN))/10);
  
  //speed=map(error, -1000, 1000, -255, 255);
  if(ypr[1]>1||ypr[1]<(-1)){
    setSpeeds(0,0);
  }
  else{
   setSpeeds(speed, speed);
  }
  last_P = P;
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

void gyro_init(){
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(115200);
    mpu.initialize();

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Gyro offsets
    mpu.setXGyroOffset(-599);
    mpu.setYGyroOffset(39);
    mpu.setZGyroOffset(15);
    mpu.setZAccelOffset(2279);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    if (!dmpReady){
      Serial.print("DMP not ready!");
      while(1);
    }


}
