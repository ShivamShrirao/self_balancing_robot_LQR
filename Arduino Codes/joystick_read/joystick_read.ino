void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

int Xax=A0;

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(Xax));
}
