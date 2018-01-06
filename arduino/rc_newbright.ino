/*
  New Bright 1/24 RC

  Connect to the 4 out pins in the main PCB to the two motors.
*/

int valF, valB, valL, valR;

void setup() {
  Serial.begin(115200);
}

void loop() {
  if(Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    char *c_str = str.c_str();
		
    if (!strncmp(c_str, "getrc", 5)) {
      valF = analogRead(1);
      valB = analogRead(2);
  
      valL = analogRead(5);
      valR = analogRead(6);
  
      Serial.print(valF);
      Serial.print(' ');
      Serial.print(valB);
      Serial.print(' ');
      Serial.print(' ');
      Serial.print(valL);
      Serial.print(' ');
      Serial.print(valR);
      Serial.print('\n');
    }
  }
}
