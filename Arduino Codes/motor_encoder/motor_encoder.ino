#define InR1            7                       // motor pin
#define PWMR            2                       // PWM motor pin
#define InR2            4                       // motor pin 

#define enc_pinA        3
#define enc_pinB        11

long int encoder_val=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(enc_pinA),count,RISING);
  pinMode(enc_pinB,INPUT);
  motor_init();
}

void count(){
  if(digitalRead(enc_pinB)==LOW){
    encoder_val++;
  }
  else if(digitalRead(enc_pinB)==HIGH){
    encoder_val--; 
  }
}

void motor_init(){                              // initialize the motors
    pinMode(InR1, OUTPUT);
    pinMode(InR2, OUTPUT);
    pinMode(PWMR, OUTPUT);
}

void motorForwardR(int PWM_val)  {              // Right motor forward
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, HIGH);
}

void motorBackwardR(int PWM_val)  {              // Right motor backward
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, HIGH);
    digitalWrite(InR2, LOW);
}

void motorStopR(int PWM_val)  {                  // Stop right motor
    analogWrite(PWMR, PWM_val);
    digitalWrite(InR1, LOW);
    digitalWrite(InR2, LOW);
}

float diameter=0.066;

float getPosition(){
    float revs=encoder_val/270.0;
    return revs*PI*diameter;
}

void loop() {
  // put your main code here, to run repeatedly
  encoder_val=0;
  int spd=50;
  motorForwardR(spd);
//  spd=0;
//  while(encoder_val<=(270)){
//      Serial.println(encoder_val);
//  }
//  delay(5000);
//  spd=0;
//  float dt=getPosition();
//  Serial.print(dt,8);
//  Serial.print("\t");
//  Serial.println(encoder_val/270.0);
//  motorStopR(spd);
//  delay(500);
}
