

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(8,INPUT_PULLUP);
}

int k = 0;

void loop() {
  k = digitalRead(8);
  Serial.println(k);
}
