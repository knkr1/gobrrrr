#include <Ultrasonic.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

#define TRIG_PIN_LEFT 2
#define ECHO_PIN_LEFT 3
#define TRIG_PIN_RIGHT 4
#define ECHO_PIN_RIGHT 5

Ultrasonic ultrasonicLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
Ultrasonic ultrasonicRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

Servo steeringServo;

MPU6050 mpu;

double setPoint = 0;
double currentAngle = 0;
double servoOutput;

PID anglePID(&currentAngle, &servoOutput, &setPoint, 2.0, 5.0, 1.0, DIRECT);

void setup() {
    steeringServo.attach(6);
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        while (1);
    }
    setPoint = 0;
    anglePID.SetMode(AUTOMATIC);
    anglePID.SetOutputLimits(-45, 45);
}

void loop() {
    readGyroData();
    if (anglePID.Compute()) {
        int servoAngle = constrain(90 + servoOutput, 0, 180);
        steeringServo.write(servoAngle);
    }
}

void readGyroData() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    currentAngle = gx / 131.0;
}

void moveForward(int speed) {
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    analogWrite(10, speed);
    digitalWrite(11, HIGH);
    digitalWrite(12, LOW);
    analogWrite(13, speed);
}

void moveBackward(int speed) {
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
    analogWrite(10, speed);
    digitalWrite(11, LOW);
    digitalWrite(12, HIGH);
    analogWrite(13, speed);
}

void stopMotors() {
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    analogWrite(10, 0);
    digitalWrite(11, LOW);
    digitalWrite(12, LOW);
    analogWrite(13, 0);
}

int readDistanceLeft() {
    return ultrasonicLeft.read();
}

int readDistanceRight() {
    return ultrasonicRight.read();
}

void steerLeft(int angle) {
    steeringServo.write(angle);
}

void steerRight(int angle) {
    steeringServo.write(angle);
}

void centerSteering() {
    steeringServo.write(90);
}
