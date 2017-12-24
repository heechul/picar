int ch1; // Here's where we'll keep our channel values
int ch2;

void setup()
{
  pinMode(2, INPUT); 
  pinMode(3, INPUT);
  Serial.begin(115200);
}

void loop()
{
  if(Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');

    if (str == "getrc" ) {
      ch1 = pulseIn(2, HIGH, 50000); // steering (25ms timeout)
      ch2 = pulseIn(3, HIGH, 50000); // throttle (25ms timeout)

      Serial.print(ch1);
      Serial.print(' ');
      Serial.print(ch2);
      Serial.print('\n');
    }
  }
}
