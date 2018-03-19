

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(8,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
}

void loop() {

  
  Serial.print(digitalRead(8));
  Serial.print(digitalRead(9));
  Serial.println(digitalRead(10));
}
