#include <PWM.h>
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md;
// EVERYTHING GYRO RELATED
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 24  // use pin 2 on Arduino Uno & most boards
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void gyro_init();
// END OF EVERYTHING GYRO RELATED

//////////////AVG ERROR CALCULATION STUFF////////////
#include "RunningAverage.h"
RunningAverage avg_error_array(200);
void calc_avg_error();
int avg_error;
/////////////////////////////////////////////////////
void calc_balance_PID();
void calc_setpoint_PID();
void receive_data();
void stopIfFault();
void receive_data();

char safety=0;

//Balancing PID related values
int error, P, D, last_P;
long I = 0;
int speed=0;
int I_gain_num = 3;
int I_gain_den = 1000;
int D_gain_num = 20;
int D_gain_den = 100;
int P_gain_num = 70;//numenator
int P_gain_den = 100;//denominator
int max_i = 200;
int target_speed=0;

//
//setpoint PID related values
int stp_error=0;
int last_stp_error=0;
long int stp_I=0;
int stp_P=0;
int stp_D=0;
int stp_P_gain=500;
int stp_I_gain=6600;
int stp_D_gain=150;

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
int motor_left=0;
int motor_right=0;

/////////////BEGINNING OF MOTOR STUFF/////////////
#include <Encoder.h>

Encoder motor_left_pos(38, 39);
Encoder motor_right_pos(40, 41);

int calc_motor_left_PWM();
int calc_motor_right_PWM();

int check_motor_safety();
//LEFT MOTOR STUFF
long motor_left_last_pos  = 0;
long motor_left_current_pos=0;
int motor_left_PWM=0;
int motor_left_current_speed=0;
int motor_left_target_speed=10;

int motor_left_speed_error=0;
int motor_left_P=0;
long motor_left_I=0;
int motor_left_D=0;
int motor_left_last_speed_error=0;

int motor_left_P_gain=10;
int motor_left_I_gain=1;
int motor_left_D_gain=10;

//RIGHT MOTOR STUFF
long motor_right_last_pos=0;
long motor_right_current_pos=0;
int motor_right_PWM=0;
int motor_right_current_speed=0;
int motor_right_target_speed=10;

int motor_right_speed_error=0;
int motor_right_P=0;
long motor_right_I=0;
int motor_right_D=0;
int motor_right_last_speed_error=0;

int motor_right_P_gain=10;
int motor_right_I_gain=1;
int motor_right_D_gain=10;

/////////////////END OF MOTOR STUFF//////////////

void printData();

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
  gyro_init();
  md.init();
  
  P=I=D=error=last_P=0;
  dir=0;
  turn=0;
  turn_amount=100;
  direction_amount=5;
  target_speed=0;

  Serial1.begin(9600);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    receive_data();
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
        while((millis()%5)!=0);
       

        check_safety();
        if(safety!=0){
          md.setSpeeds(0,0);
        }
        //If robot is lying down, stop the motors

        //if it's not horisontal: DRIVE and BALANCE! :)
        else{
          calc_avg_error();
          calc_setpoint_PID();
          calc_balance_PID();
          //md.setSpeeds(motor_left_target_speed+turn+target_PWM, motor_right_target_speed-turn+target_PWM);
          md.setSpeeds(motor_left_PWM, motor_right_PWM);
        }
        printData();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // blink LED to indicate activity
    }
}

void gyro_init(){
      // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-599);
    mpu.setYGyroOffset(39);
    mpu.setZGyroOffset(15);
    mpu.setZAccelOffset(2279);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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
}


void calc_balance_PID(){
  
        //calc P
        P=(-ypr[2])*100+setpoint_offset;
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
        /*
        if(I>30000) I=300;
        else if(I<-300) I=-300;
        */


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
        motor_right_target_speed=error;
        motor_left_target_speed=error;
        motor_right_PWM=calc_motor_right_PWM();
        motor_left_PWM=calc_motor_left_PWM();
}

void calc_setpoint_PID(){

      //stp_error=(error-target_PWM);
      stp_error=target_speed-((motor_right_current_speed+motor_left_current_speed)/2);
      stp_I+=stp_error;
      stp_D=stp_error-last_stp_error;
      setpoint_offset=stp_error/stp_P_gain+stp_I/stp_I_gain+stp_D/stp_D_gain;
      last_stp_error=stp_error;


}

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while(1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while(1);
  }
}

void receive_data(){

      if(Serial1.available()>2){
      buff[0]=Serial1.read();
      if(buff[0]=='d'){
        turn=Serial1.parseInt();
        turn=turn/2;
        dir=Serial1.parseInt();
        target_speed=map(dir, -512, 512, -150, 150);
      }
    else{
       
      if(buff[0]=='s'){
        buff[0]=Serial1.read();
          if(buff[0]=='p'){
            P_gain_num=Serial1.parseInt();
            Serial1.print("P_numerator set to ");
            Serial1.println(P_gain_num);
          }
          else if(buff[0]=='P'){
            P_gain_den=Serial1.parseInt();
            Serial1.print("P_denominator set to ");
            Serial1.println(P_gain_den);
          }
          else if(buff[0]=='D'){
            D_gain_den=Serial1.parseInt();
            Serial1.print("D_gain_denominator set to ");
            Serial1.println(D_gain_den);
          }
          else if(buff[0]=='I'){
            I_gain_den=Serial1.parseInt();
            Serial1.print("I_gain_denominator set to ");
            Serial1.println(I_gain_den);
          }
                    else if(buff[0]=='d'){
            D_gain_num=Serial1.parseInt();
            Serial1.print("D_gain_numenator set to ");
            Serial1.println(D_gain_num);
          }
          else if(buff[0]=='i'){
            I_gain_num=Serial1.parseInt();
            Serial1.print("I_gain_numenator set to ");
            Serial1.println(I_gain_num);
          }
          else if(buff[0]=='s'){
            buff[0]=Serial1.read();
            if(buff[0]=='p'){
              stp_P_gain=Serial1.parseInt();
              Serial1.print("stp_P_gain set to ");
              Serial1.println(stp_P_gain);
            }
            else if(buff[0]=='i'){
              stp_I_gain=Serial1.parseInt();
              Serial1.print("stp_I_gain set to ");
              Serial1.println(stp_I_gain);
            }
            else if(buff[0]=='d'){
              stp_D_gain=Serial1.parseInt();
              Serial1.print("stp_D_gain set to ");
              Serial1.println(stp_D_gain);
            }
          }
          
          
      }
    }     
  }
}

int calc_motor_right_PWM(){

  motor_right_current_pos = motor_right_pos.read();
  motor_right_current_speed=motor_right_current_pos-motor_right_last_pos;
  motor_right_last_pos=motor_right_current_pos;
  
  motor_right_speed_error=motor_right_target_speed-motor_right_current_speed;

  motor_right_P=motor_right_speed_error*motor_right_P_gain;
  motor_right_D=(motor_right_speed_error-motor_right_last_speed_error)*motor_right_D_gain;
  motor_right_I=motor_right_I+motor_right_speed_error;
  //motor_right_I=motor_right_I*motor_right_I_gain;

  //cap I
  
  if(motor_right_I>400){
    motor_right_I=400;
  }
  else if(motor_right_I<-400){
    motor_right_I=-400;
  }
  return (motor_right_P+(motor_right_I*motor_right_I_gain)+motor_right_D);
}

int calc_motor_left_PWM(){

  motor_left_current_pos = motor_left_pos.read();
  motor_left_current_speed=motor_left_current_pos-motor_left_last_pos;
  motor_left_last_pos=motor_left_current_pos;
  
  motor_left_speed_error=motor_left_target_speed-motor_left_current_speed;

  motor_left_P=motor_left_speed_error*motor_left_P_gain;
  motor_left_D=(motor_left_speed_error-motor_left_last_speed_error)*motor_left_D_gain;
  motor_left_I=motor_left_I+motor_left_speed_error;
  //motor_left_I=motor_left_I*motor_left_I_gain;

  //cap I
  
  if(motor_left_I>400){
    motor_left_I=400;
  }
  else if(motor_left_I<-400){
    motor_left_I=-400;
  }
  return (motor_left_P+(motor_left_I*motor_left_I_gain)+motor_left_D);
}

void check_safety(){
  int M1_current = md.getM1CurrentMilliamps();
  int M2_current = md.getM2CurrentMilliamps();
  safety=0;
  if(M2_current>1000&&M1_current<1000){
    safety=1;
  }
  else if(M1_current>1000&&M2_current<1000){
    safety=2;
  }
  else if(M1_current>1000&&M2_current>1000){
    safety=3;
  }
  if(ypr[2]>1||ypr[2]<(-1)){
    setpoint_offset=0;
    //md.setSpeeds(0,0);
    I=0;
    stp_I=0;
    safety=4;
  }
  /*
  if(safety>0&&safety<4){
    md.setSpeeds(0,0);
    delay(1000);
  }*/
}

void calc_avg_error(){
  avg_error_array.addValue(P);
  avg_error=avg_error_array.getAverage();
}

void printData(){

    Serial.print("avg error: ");
    Serial.println(avg_error);

}
