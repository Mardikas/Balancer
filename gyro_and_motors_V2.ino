//Somewhat ok PID settings: 130/10, 13/20, 90/10
//Even more stable:  135; 50; 16

//way better PID settings (IT ACTUALLY STAYS UPRIGHT): 100/10; 16/40; 55/10 


//Last PID: 140/10; 16/40; 120/10
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


//ALL FUNCTIONS USED

void init_motors();
void setM1Speed(int speed); // Set speed for M1.
void setM2Speed(int speed); // Set speed for M2.
void setSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
void setM1Brake(int brake); // Brake M1. 
void setM2Brake(int brake); // Brake M2.
void setBrakes(int m1Brake, int m2Brake);

void gyro_init();

void calc_balance_PID();
void calc_setpoint_PID();

void receive_data();

//END OF FUNCTIONS


int motor1=9;
int motor1_dirA=2;
int motor1_dirB=4;
int motor1_EN=6;

int motor2=10;
int motor2_dirA=7;
int motor2_dirB=8;
int motor2_EN=12;

int motor_left=0;
int motor_right=0;

int avgPWM = 0;
int avgPWM_array[10];
int avgPWM_counter=0;



//Balancing PID related values
int error, P, D, last_P;
long I = 0;
int speed=0;
int I_gain_num = 33;
int I_gain_den = 40;
int D_gain_num = 550;
int D_gain_den = 10;
int P_gain_num= 209;//numenator
int P_gain_den=10;//denominator
int max_i=200;
int target_PWM=0;

//
//setpoint PID related values
int stp_error=0;
int last_stp_error=0;
long int stp_I=0;
int stp_P=0;
int stp_D=0;

int stp_P_gain=150;
int stp_I_gain=2000;
int stp_D_gain=150;
//

char buff[4];
float offset;
int turn;
int dir;
int turn_amount;
int direction_amount;
int remap_counter=0;
int remap_normalise_counter=0;
int remap_limit=10;
int setpoint_offset=0;

unsigned long controller_rst_timer;

void setup() {
  // put your setup code here, to run once:
init_motors();
   gyro_init();
   P=I=D=error=last_P=0;

dir=0;
turn=0;
turn_amount=100;
direction_amount=5;

//zero avgPWM
for(avgPWM_counter=0;avgPWM_counter<9;avgPWM_counter++){
  avgPWM_array[avgPWM_counter]=0;
}
avgPWM_counter=0;
   Serial.begin(9600);
   delay(2000);
}


void loop() {
    receive_data();

  // put your main code here, to run repeatedly:
      mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        while((millis()%5)!=0);
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);


        //If robot is lying down, stop the motors
        if(ypr[2]>1||ypr[2]<(-1)){
          setpoint_offset=0;
          setSpeeds(0,0);
          I=0;
        }
        //if it's not horisontal: DRIVE and BALANCE! :)
        else{
          calc_setpoint_PID();
          calc_balance_PID();
          
            setSpeeds(motor_left+turn+target_PWM, motor_right-turn+target_PWM);
        }
    }
  digitalWrite(motor1_EN, HIGH);
  digitalWrite(motor2_EN, HIGH);
}




//FUNCTIONS: 

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

void receive_data(){

      if(Serial.available()>2){
      buff[0]=Serial.read();
      if(buff[0]=='d'){
        turn=Serial.parseInt();
        turn=turn/2;
        dir=Serial.parseInt();
        target_PWM=map(dir, -512, 512, -150, 150);
      }
    else{
       
      if(buff[0]=='s'){
        buff[0]=Serial.read();
          if(buff[0]=='p'){
            P_gain_num=Serial.parseInt();
            Serial.print("P_numerator set to ");
            Serial.println(P_gain_num);
          }
          else if(buff[0]=='P'){
            P_gain_den=Serial.parseInt();
            Serial.print("P_denominator set to ");
            Serial.println(P_gain_den);
          }
          else if(buff[0]=='D'){
            D_gain_den=Serial.parseInt();
            Serial.print("D_gain_denominator set to ");
            Serial.println(D_gain_den);
          }
          else if(buff[0]=='I'){
            I_gain_den=Serial.parseInt();
            Serial.print("I_gain_denominator set to ");
            Serial.println(I_gain_den);
          }
                    else if(buff[0]=='d'){
            D_gain_num=Serial.parseInt();
            Serial.print("D_gain_numenator set to ");
            Serial.println(D_gain_num);
          }
          else if(buff[0]=='i'){
            I_gain_num=Serial.parseInt();
            Serial.print("I_gain_numenator set to ");
            Serial.println(I_gain_num);
          }
          else if(buff[0]=='s'){
            buff[0]=Serial.read();
            if(buff[0]=='p'){
              stp_P_gain=Serial.parseInt();
              Serial.print("stp_P_gain set to ");
              Serial.println(stp_P_gain);
            }
            else if(buff[0]=='i'){
              stp_I_gain=Serial.parseInt();
              Serial.print("stp_I_gain set to ");
              Serial.println(stp_I_gain);
            }
            else if(buff[0]=='d'){
              stp_D_gain=Serial.parseInt();
              Serial.print("stp_D_gain set to ");
              Serial.println(stp_D_gain);
            }
          }
          
          
      }
    }     
  }
}

void gyro_init(){
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    Serial.begin(9600);
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

void calc_balance_PID(){
  
        //calc P
        P=(ypr[2])*100+setpoint_offset;
        //P=P;
        /*if(ypr[2]<0){
          P=-P;
        }*/
        if(P>25){
          P=25;
        }
        else if(P<-25){
          P=-25;
        }
        //calc I
        I+=P;
        if(I>300) I=300;
        else if(I<-300) I=-300;
        


        //calc D
        D=P-last_P;

        //calc Total error
        error=(P*P_gain_num/P_gain_den) + (I*I_gain_num/I_gain_den) + (D*D_gain_num/D_gain_den);

        if(error>255){
          error=255;
        }
        else if(error<-255){
          error=-255;
        }
        //remember last error
        last_P=P;

        //assign total corrections to motors
        motor_right=error+dir;
        motor_left=error+dir;
}

void calc_setpoint_PID(){
  /*
  if(error<-3){
    remap_counter--;
  }
  else if(error>3){
    remap_counter++;
  }

  if(remap_counter>remap_limit){
    setpoint_offset=setpoint_offset+0.001;
    remap_counter=0;
  }
  else if(remap_counter<-remap_limit){
    setpoint_offset=setpoint_offset-0.001;
    remap_counter=0;
  }
  */
  /*
    if(avgPWM_counter>9){
      avgPWM_counter=0;
    }
    avgPWM_array[avgPWM_counter]=error;
    avgPWM_counter++;
    avgPWM=0;
    for(int i=0; i<10; i++){
      avgPWM+=avgPWM_array[i];
    }
    avgPWM=avgPWM/10;
    setpoint_offset=(avgPWM+target_PWM)/10;
    */

    /*
    int stp_error=0;
    int last_stp_error=0;
    int stp_I=0;
    int stp_P=0;*/

    if(target_PWM<0&&error<=0){
      setpoint_offset=target_PWM/50;
    }
    else if(target_PWM>0&&error>=0){
      setpoint_offset=target_PWM/50;
    }
    else{
      stp_error=(error-target_PWM);
      stp_I+=stp_error;
      stp_D=stp_error-last_stp_error;
      setpoint_offset=stp_error/stp_P_gain+stp_I/stp_I_gain+stp_D/stp_D_gain;
      last_stp_error=stp_error;
    }


}
