#include <Servo.h>

Servo steer, throttle;
int ch1, ch2;

void setup()
{
	// RC input pins
	pinMode(2, INPUT); 
	pinMode(3, INPUT);

	// servo control pins
	steer.attach(9);
	throttle.attach(10);

	// serial init
	Serial.begin(115200);

	// initialize servo values at 1500us
	ch1 = ch2 = 1500; 
}

void loop()
{
	if(Serial.available() > 0) {
		String str = Serial.readStringUntil('\n');
		char *c_str = str.c_str();
		
		if (!strncmp(c_str, "getrc", 5)) {
			ch1 = pulseIn(2, HIGH, 50000); // steering (25ms timeout)
			ch2 = pulseIn(3, HIGH, 50000); // throttle (25ms timeout)
			Serial.print(ch1);
			Serial.print(' ');
			Serial.print(ch2);
			Serial.print('\n');
		} else if (!strncmp(c_str, "setpwm", 6)) {
			sscanf(c_str + 7, "%d %d", &ch1, &ch2);
			steer.writeMicroseconds(ch1);
			throttle.writeMicroseconds(ch2);
		}
	}
}
