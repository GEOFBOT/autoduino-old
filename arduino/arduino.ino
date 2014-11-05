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

int comm[2] = {0,0};

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
  if(comm[0] == 1) {
    drive->run(BACKWARD);
    delay(100);
    drive->run(RELEASE);
  } else if(comm[0] == -1) {
    drive->run(FORWARD);
    delay(100);
    drive->run(RELEASE);
  } 
  if(comm[1] == 1) {
    steer->run(BACKWARD);
    delay(100);
    steer->run(RELEASE);
  } else if(comm[1] == -1) {
    steer->run(FORWARD);
    delay(100);
    steer->run(RELEASE);
  }
  comm[0] = 0;
  comm[1] = 0;
  for(int i = 0; i <= 180; i += 45) {
    servo.write(i);
    unsigned int p = head.ping();
    if(p / US_ROUNDTRIP_CM != 0) 
      Serial.print(p / US_ROUNDTRIP_CM);
    else
      Serial.print(MAX_DISTANCE);
    Serial.print(" ");
  }
  unsigned int p2 = tail.ping();
  if(p2 / US_ROUNDTRIP_CM != 0)
    Serial.println(p2 / US_ROUNDTRIP_CM);
  else
    Serial.println(MAX_DISTANCE);
  while(Serial.available() == 0) {}
  while(Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'f') {
      drive->run(FORWARD);
      comm[0] = 1;
    } else if (c == 'r') {
      drive->run(BACKWARD);
      comm[0] = -1;
    } else if (c == 'a') {
      steer->run(FORWARD);
      comm[1] = 1;
    } else if (c == 'd') {
      steer->run(BACKWARD);
      comm[1] = -1;
    }
  }
  delay(750);
}
