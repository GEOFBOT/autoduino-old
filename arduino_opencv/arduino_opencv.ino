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

int dir[2] = {0, 0};

void setup() {
  Serial.begin(9600);
  
  servo.attach(SERVO);
  AFMS.begin();
  steer->setSpeed(255);
  drive->setSpeed(80);
  steer->run(RELEASE);
  drive->run(RELEASE);
}

void loop() {
  delay(50);
  /*for(int i = 0; i <= 180; i += 45) {
    servo.write(i);
    unsigned int p = head.ping();
    Serial.print(p / US_ROUNDTRIP_CM);
    Serial.print(" ");
    delay(50);
  }
  unsigned int p2 = tail.ping();
  Serial.println(p2 / US_ROUNDTRIP_CM);
  steer->run(RELEASE);
  delay(1250);
  drive->run(RELEASE);
  delay(250);*/
  while(Serial.available() == 0) {}
  while(Serial.available() > 0) {
    char c = Serial.read();
    if (c == 's' || c == 'c') {
      if (c == 's') {
        if (dir[0] = 1) drive->run(BACKWARD);
        else if (dir[0] = -1) drive->run(FORWARD);          
        delay(25);
        drive->run(RELEASE);
        dir[0] = 0;
      } else if (c == 'c') {
        if (dir[1] = 1) drive->run(FORWARD);
        else if (dir[1] = -1) drive->run(BACKWARD);   
        delay(25);
        steer->run(RELEASE);
        dir[1] = 0;
      }
    }
    else {
      if (c == 'f') {
        drive->run(FORWARD);
        dir[0] = 1;
      } else if (c == 'r') {
        drive->run(BACKWARD);
        dir[0] = -1;
      } else if (c == 'a') {
        steer->run(FORWARD);
        dir[1] = -1;
      } else if (c == 'd') {
        steer->run(BACKWARD);
        dir[1] = 1;
      } else if (c == 'z') {
        unsigned int p = head.ping();
        if(p / US_ROUNDTRIP_CM != 0)
          Serial.println(p / US_ROUNDTRIP_CM);
        else
          Serial.println(MAX_DISTANCE);
      }
      delay(25);
    }
  }
}
