float wave1_val = 0.25;
float wave2_val = 0.12;
float wave3_val = 0.5;

int wave1_pin = A1;
int wave2_pin = A2;
int wave3_pin = A3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(wave1_val);
  Serial.print(",");
  Serial.print(wave2_val);
  Serial.print(",");
  Serial.println(wave3_val);
  
  wave1_val = analogRead(wave1_pin);
  wave2_val = analogRead(wave2_pin);
  wave3_val = analogRead(wave3_pin);

  
}
