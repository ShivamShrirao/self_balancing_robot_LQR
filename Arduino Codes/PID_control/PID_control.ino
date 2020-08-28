#include "I2Cdev.h"
//#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <TimerOne.h>
#include <PID_v1.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

#define InL1            8                       // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            13                      // motor pin

#define buzz_pin        31                      // buzzer pin
#define Mag1            44                      // electromagnet pin
#define Mag2            46 

#define enc_pin1A        2                      // encoder pins of motor
#define enc_pin1B        12

#define enc_pin2A        3
#define enc_pin2B        11

#define MPUINTR          40

MPU6050 mpu;

bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

float pitch_angle=0,roll_angle=0;

long int encoder_val1=0;
long int encoder_val2=0;

int bzct=0;
int bzval=HIGH;

double angleSet=0;
double svdangleSet=angleSet;
double AKp = 32;
double AKd = 2;
double AKi = 50;
double Ainp=0, Aout;
PID Apid(&Ainp, &Aout, &angleSet, AKp, AKi, AKd, DIRECT);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup() {
  // put your setup code here, to run once:
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);
    attachInterrupt(digitalPinToInterrupt(enc_pin1A),count1,RISING);
    pinMode(enc_pin1B,INPUT);
    attachInterrupt(digitalPinToInterrupt(enc_pin2A),count2,RISING);
    pinMode(enc_pin2B,INPUT);

    Serial1.begin(9600);
    Serial1.flush();
    motor_init();
    BUZZ_init();
    MAG_init();

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      
      // enable Arduino interrupt detection
      attachInterrupt(digitalPinToInterrupt(MPUINTR), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
    }

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

//    Timer1.initialize(10000);
//    Timer1.attachInterrupt(driveMotor);

    Apid.SetMode(AUTOMATIC);
    Apid.SetSampleTime(9.5);
    Apid.SetOutputLimits(-255, 255);
}

void count1(){
  if(digitalRead(enc_pin1B)==LOW){
    encoder_val1--;
  }
  else if(digitalRead(enc_pin1B)==HIGH){
    encoder_val1++; 
  }
}
void count2(){
  if(digitalRead(enc_pin2B)==LOW){
    encoder_val2++;
  }
  else if(digitalRead(enc_pin2B)==HIGH){
    encoder_val2--; 
  }
}

float diameter=0.066;

float getPosition(){
    float encoder_val=(encoder_val1+encoder_val2)/2.0;
    if(abs(encoder_val1-encoder_val2)>20){
      encoder_val1=encoder_val2=encoder_val;
    }
    float dis=PI*diameter*encoder_val/270.0;
    return dis;
}

int spd,rlsp=150;
float dst;
unsigned long ptime=micros(),ctime;
void loop() 
{
  if(Serial1.available()>=18){                        // If 18 bytes received
    if(Serial1.read()==0x7E){                         // check the initializer byte
      for(int i=0;i<10;i++){
        byte discard = Serial1.read();                // discard some config bytes
      }
      int zMSB=Serial1.read();                        // read bytes for electromagnet state
      int zLSB=Serial1.read();
      if(zLSB){
        MagDrop();
        Serial.print("Drop, ");
      }
      else{
        MagPick();
        Serial.print("Pick, ");
      }
      int xMSB=Serial1.read();
      int xLSB=Serial1.read();
      int xdata = xLSB + (xMSB*256);                  // combine x values of joystick
      int yMSB=Serial1.read();
      int yLSB=Serial1.read();
      int ydata = yLSB + (yMSB*256);                  // combine y values of joystick
      if(xdata<200 && ydata>900){                     // range of x and y values for forward motion
        angleSet=2.5;
        Serial.println("Forward");
      }
      else if(xdata>400 && xdata<700 && ydata>900){   // range of x and y values for backward motion
        angleSet=-2.5;
        Serial.println("Backward");
      }
      else if(ydata<200 && xdata>900){                // range of x and y values for right motion
        motorForwardL(rlsp);
        motorBackwardR(rlsp);
        Serial.println("Right");
      }
      else if(ydata>400 && ydata<700 && xdata>900){   // range of x and y values for left motion
        motorForwardR(rlsp);
        motorBackwardL(rlsp);
        Serial.println("Left");
      }
      else{                                            // else Stop the motors
        angleSet=0;
        Serial.println("Stop");
      }
    }
  }
  // put your main code here, to run repeatedly:
  Serial.print("Y: ");Serial.print(roll_angle);
  Serial.print("\tdst: ");Serial.print(dst);
  Serial.print("\taset: ");Serial.print(angleSet);
  Serial.print("\tP: ");Serial.println(spd);
//  if(Serial.available()>0){
//    AKp=(double)Serial.parseFloat();
//    Apid.SetTunings(AKp,AKi,AKd);
//    Serial.read();
//    Serial.flush();
//  }
  if (!dmpReady) return;
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
//    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll_angle = ypr[1]*180.0/PI+2;
  }
  if(abs(roll_angle)<45)
  {
    Ainp=(double)roll_angle;
    Apid.Compute();
    spd=(int)Aout;
    int pwmm=abs(spd);
    pwmm=map(pwmm,0,255,65,255);
    if(spd>0){
      motorForwardR(pwmm);
      motorForwardL(pwmm);
    }
    else if(spd<0){
      motorBackwardR(pwmm);
      motorBackwardL(pwmm);
    }
    else{
      motorStopR(pwmm);
      motorStopL(pwmm);
    }
    if(abs(dst)>1)
    {
      encoder_val1=encoder_val2=0;
    }
  }
  else
  {
    motorStopR(0);
    motorStopL(0);
  }
}

void BUZZ_init(){                              // initialize the Buzzer pins
    pinMode(buzz_pin, OUTPUT);
    digitalWrite(buzz_pin, HIGH);
}

void motor_init(){                              // initialize the motors
    pinMode(InL1, OUTPUT);
    pinMode(InL2, OUTPUT);
    pinMode(PWML, OUTPUT);
    
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

void motorForwardL(int PWM_val)  {              // Left motor forward
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, HIGH);
}

void motorForwardR(int PWM_val)  {              // Right motor forward
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}

void motorBackwardL(int PWM_val)  {              // Left motor backward
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, HIGH);
    digitalWrite(InL2, LOW);
}

void motorBackwardR(int PWM_val)  {              // Right motor backward
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}

void motorStopL(int PWM_val)  {                  // Stop left motor
    analogWrite(PWML, PWM_val);
    digitalWrite(InL1, LOW);
    digitalWrite(InL2, LOW);
}

void motorStopR(int PWM_val)  {                  // Stop right motor
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
}

void MAG_init(){                                // Initialize electromagnet pins
    pinMode(Mag1, OUTPUT); 
    digitalWrite(Mag1, LOW);
    pinMode(Mag2, OUTPUT); 
    digitalWrite(Mag2, LOW);
}

void MagPick(void)  {                            // Activate the electromagnet
  digitalWrite(Mag1, HIGH);
}

void MagDrop(void)  {                            // Deactivate the electromagnet
  digitalWrite(Mag2, LOW);
}
