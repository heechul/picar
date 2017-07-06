// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!
#include <stdio.h>

#define FORWARD_PIN   6
#define BACKWARD_PIN  5
#define LEFT_PIN      3
#define RIGHT_PIN     2

void left()
{
  digitalWrite(LEFT_PIN, LOW);
  digitalWrite(RIGHT_PIN, HIGH);
}

void right()
{
  digitalWrite(RIGHT_PIN, LOW);
  digitalWrite(LEFT_PIN, HIGH);  
}

void center()
{
  digitalWrite(LEFT_PIN, HIGH);
  digitalWrite(RIGHT_PIN, HIGH);
}

void forward()
{
  digitalWrite(FORWARD_PIN, LOW);
  digitalWrite(BACKWARD_PIN, HIGH);
}

void backward()
{
  digitalWrite(BACKWARD_PIN, LOW);
  digitalWrite(FORWARD_PIN, HIGH);
}

void nomove()
{
  digitalWrite(FORWARD_PIN, HIGH);
  digitalWrite(BACKWARD_PIN, HIGH);
}

void demo()
{
  nomove();
  center();
  delay(500);
  forward();
  delay(500);
  backward();
  delay(500);  
  nomove();
  delay(500);
  left();
  delay(500);
  right();
  delay(500);
  center();
}

void setup() 
{
  Serial.begin(115200);
  // Serial.setTimeout(50); // 50 ms timeout. 20 HZ
  Serial.println("Motor test!");

  pinMode(6, OUTPUT); // forward
  pinMode(5, OUTPUT); // backward
  pinMode(3, OUTPUT); // left
  pinMode(2, OUTPUT); // right
  
  // start demo
  demo();
}


void loop_new() {
  uint8_t command;
  if(Serial.available() > 0) {
    int cmd = Serial.read();
  
    switch(cmd) {
      case 's': nomove(); break;
      case 'a': forward(); break;
      case 'z': backward(); break;

      case 'c': center(); break;
      case 'l': left(); break;
      case 'r': right(); break;

      default:
        Serial.print("Invalid command\n");
        Serial.print(cmd);
    }
  } else {
    center();
    nomove();
  }
}


void loop() {
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
