#include "I2Cdev.h"
#include "MPU6050.h"

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
#define MagF            51                      // electromagnet pin
#define MagHigh         49

#define enc_pin1A        2                      // encoder pins of motor
#define enc_pin1B        12

#define enc_pin2A        3
#define enc_pin2B        11

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int f_cut = 5;
float dT = 0.01;
float Tau = 1/(2*PI*f_cut);
float alph = 0.98;
float lowx=0,lowy=0,lowz=0,highx=0,highy=0,highz=0,ogx=0,ogy=0,ogz=0;
float glax,glay,glaz,glgx,glgy,glgz;
float pitch_angle=0,roll_angle=0;
float force=0;

long int encoder_val1=0;
long int encoder_val2=0;

int bzct=0;
int bzval=HIGH;

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
    Serial.begin(57600);
    attachInterrupt(digitalPinToInterrupt(enc_pin1A),count1,RISING);
    pinMode(enc_pin1B,INPUT);
    attachInterrupt(digitalPinToInterrupt(enc_pin2A),count2,RISING);
    pinMode(enc_pin2B,INPUT);

    Serial1.begin(9600);
    motor_init();
    BUZZ_init();
    MAG_init();

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void count1(){
  if(digitalRead(enc_pin1B)==LOW){
    encoder_val1++;
  }
  else if(digitalRead(enc_pin1B)==HIGH){
    encoder_val1--; 
  }
}
void count2(){
  if(digitalRead(enc_pin2B)==LOW){
    encoder_val2--;
  }
  else if(digitalRead(enc_pin2B)==HIGH){
    encoder_val2++; 
  }
}

float diameter=0.066;

float getPosition(){
    float encoder_val=(encoder_val1+encoder_val2)/2.0;
    if(abs(encoder_val1-encoder_val2)>20){
      encoder_val1=encoder_val2=encoder_val;
    }
    float revs=encoder_val/270.0;
    return revs*PI*diameter;
}

float K[] = {-1.7154, -2.8569, 27.0535, 4.0291};
float y[] = {0, 0, 0, 0};
float y_set[] = {0, 0, 0, 0};
unsigned long ptime=micros(),curr=0;

void loop() 
{
  // put your main code here, to run repeatedly:
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  curr=micros();
  lowpassfilter(ax,ay,az);
  highpassfilter(gx,gy,gz);
  comp_filter_roll(glax,glay,glaz,glgx,glgy,glgz);
//  comp_filter_pitch(glax,glay,glaz,glgx,glgy,glgz);
  float angle=roll_angle;
  angle=angle*PI/180.0;
  float dst=getPosition();
  dT=(curr-ptime)/1000000.0;
  ptime=curr;

  y[1]=(dst-y[0])/dT;
  y[0]=dst;
  y[3]=(angle-y[2])/dT;
  y[2]=angle;
  float frc=0;
  for(int i=0; i<4; i++){
    frc-=(y[i]-y_set[i])*K[i];
  }
  force=frc;
  driveMotor();
  Serial.print("\tDist: ");Serial.print(y[1],4);
  Serial.print("\tdt: ");Serial.print(dT,5);
  Serial.print("\tRoll: ");Serial.print(angle*180/PI);
  Serial.print("\tForce: ");Serial.println(force);
  if(abs(dst)>2)
  {
    y[0]=encoder_val1=encoder_val2=0;
  }
}

float tbv=0.085/11;
float radius=diameter/2;
float rbtbv=radius/tbv;
float srtbv=7.9/rbtbv;

void driveMotor()
{
  float vlt = rbtbv*force/2 + srtbv*y[1];
  int spd = 31.67225*abs(vlt)-123.73295;
//  int spd=(int)(force*22.5);
  spd=constrain(spd, 0, 255);
  Serial.print("vlt: ");Serial.print(vlt);
  if(vlt<0){
    motorForwardR(spd);
    motorForwardL(spd);
  }
  else if(vlt>0){
    motorBackwardR(spd);
    motorBackwardL(spd);
  }
  else{
    motorStopR(spd);
    motorStopL(spd);
  }
}

void lowpassfilter(int16_t ax,int16_t ay,int16_t az){
  float alpha = Tau/(Tau+dT);
  lowx=(1-alpha)*ax + alpha*lowx;
  glax=lowx;
  lowy=(1-alpha)*ay + alpha*lowy;
  glay=lowy;
  lowz=(1-alpha)*az + alpha*lowz;
  glaz=lowz;
}

void highpassfilter(int16_t gx,int16_t gy,int16_t gz){
  float alpha = Tau/(Tau+dT);
  highx=(1-alpha)*highx + (1-alpha)*(gx-ogx);
  ogx=gx;
  glgx=highx;
  highy=(1-alpha)*highy + (1-alpha)*(gy-ogy);
  ogy=gy;
  glgy=highy;
  highz=(1-alpha)*highz + (1-alpha)*(gz-ogz);
  ogz=gz;
  glgz=highz;
}

void comp_filter_pitch(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz){
  float acc_angle=atan2(ay,abs(az))* 180 / PI;
  float gyro_angle=(-1*gx*dT) + pitch_angle;
  pitch_angle=(1-alph)*gyro_angle + alph*acc_angle;
}

void comp_filter_roll(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz){
  float acc_angle=atan2(ax,abs(az))* 180 / PI;
  float gyro_angle=(-1*gy*dT) + roll_angle;
  roll_angle=(1-alph)*gyro_angle + alph*acc_angle;
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
    pinMode(MagF, OUTPUT); 
    digitalWrite(MagF, LOW);
    pinMode(MagHigh, OUTPUT);
    digitalWrite(MagHigh, HIGH);
}

void MagPick(void)  {                            // Activate the electromagnet
  digitalWrite(MagF, HIGH);
}

void MagDrop(void)  {                            // Deactivate the electromagnet
  digitalWrite(MagF, LOW);
}
