#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include <Servo.h>

#define SERVO 10

#define HEAD 5
#define TAIL 3
#define MAX_DISTANCE 300

NewPing head(HEAD, HEAD, MAX_DISTANCE);
NewPing tail(TAIL, TAIL, MAX_DISTANCE);

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *steer = AFMS.getMotor(1);
Adafruit_DCMotor *drive = AFMS.getMotor(2);

Servo servo;

void setup() {
  Serial.begin(9600);
  
  servo.attach(SERVO);
  AFMS.begin();
  steer->setSpeed(255);
  drive->setSpeed(100);
  steer->run(RELEASE);
  drive->run(RELEASE);
}

void loop() {
  for(int i = 0; i <= 180; i += 45) {
    servo.write(i);
    unsigned int p = head.ping();
    if(p / US_ROUNDTRIP_CM != 0) 
      Serial.print(p / US_ROUNDTRIP_CM);
    else
      Serial.print(MAX_DISTANCE);
    Serial.print(" ");
    delay(1000);
  }
  unsigned int p2 = tail.ping();
  if(p2 / US_ROUNDTRIP_CM != 0)
    Serial.println(p2 / US_ROUNDTRIP_CM);
  else
    Serial.println(MAX_DISTANCE);
  steer->run(RELEASE);
  delay(1250);
  drive->run(RELEASE);
  delay(250);
  while(Serial.available() == 0) {}
  while(Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'f') {
      drive->run(FORWARD);
    } else if (c == 'r') {
      drive->run(BACKWARD);
    } else if (c == 'a') {
      steer->run(FORWARD);
    } else if (c == 'd') {
      steer->run(BACKWARD);
    }
  }
}
