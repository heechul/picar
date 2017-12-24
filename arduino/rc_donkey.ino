#include <Servo.h>

const int throttle_intr_pin = 3;
const int steering_intr_pin = 2;

const int steering_servo_pin = 9;
const int throttle_servo_pin = 10;

Servo steer, throttle;

volatile int ch1_pulse_us, ch2_pulse_us;   // duty cycles (in us)
volatile int ch1_prev_time, ch2_prev_time; // temporal store for time

void isr_rc_ch1_rising()
{
  ch1_prev_time = micros();
  attachInterrupt(digitalPinToInterrupt(throttle_intr_pin),
                  isr_rc_ch1_falling, FALLING);
}

void isr_rc_ch1_falling()
{
  int dur = micros() - ch1_prev_time;
  if (dur > 0) /* !overflow */
    ch1_pulse_us = dur;
  attachInterrupt(digitalPinToInterrupt(throttle_intr_pin),
                  isr_rc_ch1_rising, RISING);    
}

void isr_rc_ch2_rising()
{
  ch2_prev_time = micros();
  attachInterrupt(digitalPinToInterrupt(steering_intr_pin),
                  isr_rc_ch2_falling, FALLING);
}

void isr_rc_ch2_falling()
{
  int dur = micros() - ch2_prev_time;
  if (dur > 0) /* !overflow */
    ch2_pulse_us = dur;
  attachInterrupt(digitalPinToInterrupt(steering_intr_pin),
                  isr_rc_ch2_rising, RISING);    
}

void setup()
{
  pinMode(steering_intr_pin, INPUT_PULLUP);
  pinMode(throttle_intr_pin, INPUT_PULLUP);
  
  // RC input pins
  attachInterrupt(digitalPinToInterrupt(throttle_intr_pin),
                  isr_rc_ch1_rising, RISING);
  attachInterrupt(digitalPinToInterrupt(steering_intr_pin),
                  isr_rc_ch2_rising, RISING);
  
  // servo control pins
  steer.attach(steering_servo_pin);
  throttle.attach(throttle_servo_pin);

  // serial init
  Serial.begin(115200);
  
  // initialize servo values at 1500us
  ch1_pulse_us = ch2_pulse_us = 1500; 
}

void loop()
{
  if(Serial.available() > 0) {
    String str = Serial.readStringUntil('\n');
    char *c_str = str.c_str();
		
    if (!strncmp(c_str, "getrc", 5)) {
      Serial.print(ch1_pulse_us);
      Serial.print(' ');
      Serial.print(ch2_pulse_us);
      Serial.print('\n');
		} else if (!strncmp(c_str, "setpwm", 6)) {
      int ch1, ch2;
      sscanf(c_str + 7, "%d %d", &ch1, &ch2);
      steer.writeMicroseconds(ch1);
      throttle.writeMicroseconds(ch2);
		}
	}
}
