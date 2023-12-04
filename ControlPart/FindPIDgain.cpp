/*
Arduino 내 Differential Drive Robot에서의 모터 PID 제어 실험 코드
*/

#include <Arduino.h>
#include <PID_v1.h>

// Class representing a wheel with encoder and PID control
class Wheel {
private:
  int encoderPinA;
  int encoderPinB;
  int motorPin1;
  int motorPin2;
  int motorPinPWM;
  volatile long encoderCount;

public:
  Wheel(int pinA, int pinB, int pin1, int pin2, int pinPWM)
      : encoderPinA(pinA), encoderPinB(pinB), motorPin1(pin1), motorPin2(pin2), motorPinPWM(pinPWM), encoderCount(0) {}

  void init() {
    pinMode(encoderPinA, INPUT);
    pinMode(encoderPinB, INPUT);
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  }

  void updateEncoder() {
    if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }

  long getCount() const {
    return encoderCount;
  }

  void resetCount() {
    encoderCount = 0;
  }

  void move(double speed) {
    int direction = speed > 0 ? HIGH : LOW;
    speed = abs(speed);
    speed = min(speed, (double)MAX_PWM);

    analogWrite(motorPinPWM, (int)speed);
    digitalWrite(motorPin1, direction);
    digitalWrite(motorPin2, !direction);
  }

private:
  static void updateEncoder() {
    wheel.updateEncoder();
  }
};

// Class representing a differential drive robot with two wheels
class DifferentialDriveRobot {
private:
  Wheel leftWheel;
  Wheel rightWheel;

  double wheelbase;

  double setpointLeft;
  double inputLeft;
  double outputLeft;

  double setpointRight;
  double inputRight;
  double outputRight;

  PID PIDLeft;
  PID PIDRight;

public:
  DifferentialDriveRobot(int leftEncoderA, int leftEncoderB, int leftMotor1, int leftMotor2, int leftMotorPWM,
                          int rightEncoderA, int rightEncoderB, int rightMotor1, int rightMotor2, int rightMotorPWM,
                          double base)
      : leftWheel(leftEncoderA, leftEncoderB, leftMotor1, leftMotor2, leftMotorPWM),
        rightWheel(rightEncoderA, rightEncoderB, rightMotor1, rightMotor2, rightMotorPWM),
        wheelbase(base),
        setpointLeft(0.0), inputLeft(0.0), outputLeft(0.0),
        setpointRight(0.0), inputRight(0.0), outputRight(0.0),
        PIDLeft(&inputLeft, &outputLeft, &setpointLeft, 650.0, 20.0, 0.000001, DIRECT),
        PIDRight(&inputRight, &outputRight, &setpointRight, 650.0, 20.0, 0.000001, DIRECT) {}

  void init() {
    leftWheel.init();
    rightWheel.init();

    PIDLeft.SetMode(AUTOMATIC);
    PIDRight.SetMode(AUTOMATIC);

    PIDLeft.SetOutputLimits(-MAX_PWM, MAX_PWM);
    PIDRight.SetOutputLimits(-MAX_PWM, MAX_PWM);
  }

  void update() {
    leftWheel.updateEncoder();
    rightWheel.updateEncoder();

    double leftWheelSpeed = (leftWheel.getCount() * distancePerPulse) / (interval / 1000.0);
    double rightWheelSpeed = (rightWheel.getCount() * distancePerPulse) / (interval / 1000.0);

    leftWheel.resetCount();
    rightWheel.resetCount();

    inputLeft = leftWheelSpeed;
    inputRight = rightWheelSpeed;

    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      if (command == "w") {
        setpointLeft += 0.1;
        setpointRight += 0.1;
      } else if (command == "x") {
        setpointLeft -= 0.1;
        setpointRight -= 0.1;
      } else if (command == "s") {
        setpointLeft = 0.0;
        setpointRight = 0.0;
      } else if (command == "a") {
        setpointLeft -= 0.1;
        setpointRight += 0.1;
      } else if (command == "d") {
        setpointLeft += 0.1;
        setpointRight -= 0.1;
      }
    }

    PIDLeft.Compute();
    PIDRight.Compute();

    leftWheel.move(outputLeft);
    rightWheel.move(outputRight);

    serTargetSpeeds();
    serActualSpeeds();
  }

  void serTargetSpeeds() {
    Serial.print(setpointLeft);
    Serial.print(",");
    Serial.print(setpointRight);
    Serial.print(",");
  }

  void serActualSpeeds() {
    Serial.print(inputLeft);
    Serial.print(",");
    Serial.print(inputRight);
    Serial.println();
  }
};

DifferentialDriveRobot robot(
  leftEncoderPinA, leftEncoderPinB, leftMotorPin1, leftMotorPin2, leftMotorPinPWM,
  rightEncoderPinA, rightEncoderPinB, rightMotorPin1, rightMotorPin2, rightMotorPinPWM,
  wheelbase
);

void setup() {
  Serial.begin(115200);
  robot.init();
}

void loop() {
  robot.update();
  delay(interval);
}

