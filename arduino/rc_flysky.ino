int ch1; // Here's where we'll keep our channel values
int ch3;

void setup()
{
  pinMode(4, INPUT); 
  pinMode(3, INPUT);
  Serial.begin(115200);
}

void loop()
{
  if(Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');

    if (str == "getrc" ) {
      ch1 = pulseIn(4, HIGH, 25000); // Read the pulse width of 
      ch3 = pulseIn(3, HIGH, 25000);

      Serial.print(ch1);
      Serial.print(' ');
      Serial.print(ch3);
      Serial.print('\n');
    }
  }
}
