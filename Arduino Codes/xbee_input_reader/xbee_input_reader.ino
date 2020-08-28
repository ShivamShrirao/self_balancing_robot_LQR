void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.flush();
  Serial1.flush();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial1.available()>=16){
    if(Serial1.read()==0x7E){
      for(int i=0;i<10;i++){
        byte discard = Serial1.read();
      }
      int xMSB=Serial1.read();
      int xLSB=Serial1.read();
      int xdata = xLSB + (xMSB*256);
      int yMSB=Serial1.read();
      int yLSB=Serial1.read();
      int ydata = yLSB + (yMSB*256);
//      Serial.print(xMSB,HEX);
//      Serial.print(", ");
//      Serial.print(xLSB,HEX);
//      Serial.print(", ");
//      Serial.print("=== ");
      Serial.print(xdata);
//      Serial.print(" |||| ");
//      Serial.print(yMSB,HEX);
//      Serial.print(", ");
//      Serial.print(yLSB,HEX);
//      Serial.print(", ");
//      Serial.print("=== ");
//      Serial.println(ydata);
    }
  }
}
