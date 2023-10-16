#include <Arduino.h>
#include <Servo.h>

const int servo_pin = 6;
Servo servo;

const int speed = 40;

int servo_pos = 1000;
int servo_target = 1000;

void setup() {
  // put your setup code here, to run once:
  // Attach the launcher
  Serial.begin(57600);

  servo.attach(servo_pin);
  servo.writeMicroseconds(servo_pos);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available() > 0) {
    servo_target = Serial.parseInt();
    Serial.println(servo_target);
  }
  
  int dist = servo_target - servo_pos;

  if (abs(dist) < speed) servo_pos = servo_target;
  else if (dist > 0) servo_pos += speed;
  else servo_pos -= speed;

  servo.writeMicroseconds(servo_pos);

  delay(20);
}
