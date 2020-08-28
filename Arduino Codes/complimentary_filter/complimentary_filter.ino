#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int f_cut=5;
float dT = 0.01;
float Tau = 1/(2*PI*f_cut);
float alpha = Tau/(Tau+dT);
float alph = 0.03;
float lowx=0,lowy=0,lowz=0,highx=0,highy=0,highz=0,ogx=0,ogy=0,ogz=0;
float glax,glay,glaz,glgx,glgy,glgz;
float pitch_angle=0,roll_angle=0;

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
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void lowpassfilter(int16_t ax,int16_t ay,int16_t az){
  lowx=(1-alpha)*ax + alpha*lowx;
  glax=lowx;
  lowy=(1-alpha)*ay + alpha*lowy;
  glay=lowy;
  lowz=(1-alpha)*az + alpha*lowz;
  glaz=lowz;
}

void highpassfilter(int16_t gx,int16_t gy,int16_t gz){
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

void loop() 
{
  // put your main code here, to run repeatedly:
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  lowpassfilter(ax,ay,az);
  highpassfilter(gx,gy,gz);
  comp_filter_pitch(glax,glay,glaz,glgx,glgy,glgz);
  comp_filter_roll(glax,glay,glaz,glgx,glgy,glgz);
  Serial.print("Pitch:");Serial.print(pitch_angle);
  Serial.print("\tRoll:");Serial.println(roll_angle);
}
