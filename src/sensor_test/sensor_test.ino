void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(8,INPUT);
}

int k = 0;

void loop() {
  // put your main code here, to run repeatedly:
  
  k = digitalRead(8);
  //k = analogRead(A0);
  Serial.println(k);
}
