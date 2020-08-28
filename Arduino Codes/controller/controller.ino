
#define InR1            7                       // motor pin
#define PWMR            6                       // PWM motor pin
#define InR2            4                       // motor pin 

#define InL1            13                      // motor pin
#define PWML            10                      // PWM motor pin  
#define InL2            9                       // motor pin

#define buzz_pin        31                      // buzzer pin
#define MagF            51                      // electromagnet pin
#define MagHigh         49

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.flush();
  Serial1.flush();
  motor_init();
  BUZZ_init();
  MAG_init();
}

int bzct=0;
int bzval=HIGH;
int pwr=255;

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial1.available()>=18){                        // If 18 bytes received
    if(Serial1.read()==0x7E){                         // check the initializer byte
      for(int i=0;i<10;i++){
        byte discard = Serial1.read();                // discard some config bytes
      }
      int zMSB=Serial1.read();                        // read bytes for electromagnet state
      int zLSB=Serial1.read();
//      Serial.print(zMSB);
//      Serial.print("   ");
//      Serial.print(zLSB);
//      Serial.print("   ");
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
//      Serial.print(xdata);
//      Serial.print(" , ");
//      Serial.println(ydata);
      if(xdata<200 && ydata>900){                     // range of x and y values for forward motion
        motorForwardR(pwr);
        motorForwardL(pwr);
        Serial.println("Forward");
      }
      else if(xdata>400 && xdata<700 && ydata>900){   // range of x and y values for backward motion
        motorBackwardR(pwr);
        motorBackwardL(pwr);
        if(!(bzct%12)){                               // beep the buzzer on and off
          bzval = !bzval;
          bzct=0;
        }
        digitalWrite(buzz_pin, bzval);
        bzct+=1;
        Serial.println("Backward");
      }
      else if(ydata<200 && xdata>900){                // range of x and y values for right motion
        motorBackwardL(pwr);
        motorForwardR(pwr);
        Serial.println("Right");
      }
      else if(ydata>400 && ydata<700 && xdata>900){   // range of x and y values for left motion
        motorForwardL(pwr);
        motorBackwardR(pwr);
        Serial.println("Left");
      }
      else{                                            // else Stop the motors
        motorStopR(0);
        motorStopL(0);
        digitalWrite(buzz_pin, HIGH);
        Serial.println("Stop");
      }
    }
  }
}
