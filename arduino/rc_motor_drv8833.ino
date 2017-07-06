// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
#include <stdio.h>

// Define the pin numbers on which the outputs are generated.
#define MOT_A1_PIN 4
#define MOT_A2_PIN 5
#define MOT_B1_PIN 6
#define MOT_B2_PIN 7

// ================================================================================
/// Set the current on a motor channel using PWM and directional logic.
/// Changing the current will affect the motor speed, but please note this is
/// not a calibrated speed control.  This function will configure the pin output
/// state and return.
///
/// \param pwm		PWM duty cycle ranging from -255 full reverse to 255 full forward
/// \param IN1_PIN	pin number xIN1 for the given channel
/// \param IN2_PIN	pin number xIN2 for the given channel

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);

  } else { // stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

void left()
{
  set_motor_pwm(255, MOT_A1_PIN, MOT_A2_PIN);
}

void right()
{
  set_motor_pwm(-255, MOT_A1_PIN, MOT_A2_PIN);
}

void center()
{
  set_motor_pwm(0, MOT_A1_PIN, MOT_A2_PIN);
}

void forward()
{
  set_motor_pwm(255, MOT_B1_PIN, MOT_B2_PIN);
}

void backward()
{
  set_motor_pwm(-255, MOT_B1_PIN, MOT_B2_PIN);
}

void nomove()
{
  set_motor_pwm(0, MOT_B1_PIN, MOT_B2_PIN);
}

void demo()
{
  nomove();
  center();
  delay(1000);
  forward();
  delay(1000);
  backward();
  delay(1000);  
  nomove();
  delay(1000);
  left();
  delay(1000);
  right();
  delay(1000);
  center();
}

void setup() 
{
  // Initialize the stepper driver control pins to output drive mode.
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  // Initialize the serial UART at 9600 bits per second.
  Serial.begin(115200);
  Serial.setTimeout(50); // 50 ms timeout. 20 HZ
  Serial.println("Motor test!");
  
  // start demo
  demo();
}


void loop() {
  uint8_t i;
  if(Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
  
    if (str == "l") {
      left();
    } else if (str == "r") {
      right();
    } else if (str == "f") {
      forward();
    } else if (str == "b") {
      backward();
    } else if (str == "c") {
      center();
    } else if (str == "s") {
      nomove();
    } else if (str == "demo") {
      Serial.println("Start demo");
      demo();
      Serial.println("End demo");
    } else {
      Serial.println("ERR:" + str);
    }
  }
}
