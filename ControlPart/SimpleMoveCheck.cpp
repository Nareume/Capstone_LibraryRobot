/*
엔코더 모터 작동 정상 여부 확인 코드
*/

#include <Arduino.h>

class MotorEncoderController {
private:
  // Motor and Encoder Pins
  int encoderPinA;
  int encoderPinB;
  int motorPin1;
  int motorPin2;
  int motorPinPWM;

  // Constants
  const double PI = 3.14159265358979323846;
  const double wheelDiameter = 0.13;
  const double wheelbase = 0.30;
  const double distancePerPulse = (wheelDiameter * PI) / 741.0;  // Assuming 741 pulses per revolution

  // Global variables
  volatile long encoderCount;
  unsigned long previousMillis;
  const unsigned long interval;

public:
  // Constructor
  MotorEncoderController(int encPinA, int encPinB, int mPin1, int mPin2, int mPinPWM, unsigned long updateInterval)
      : encoderPinA(encPinA), encoderPinB(encPinB), motorPin1(mPin1), motorPin2(mPin2), motorPinPWM(mPinPWM),
        encoderCount(0), previousMillis(0), interval(updateInterval) {}

  // Initialize motor and encoder pins
  void init() {
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);

    Serial.begin(115200);
  }

  // Update method to be called in loop
  void update() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;

      // Calculate wheel speed
      double wheelSpeed = (encoderCount * distancePerPulse) / (interval / 1000.0);
      Serial.print("Encoder Count: ");
      Serial.print(encoderCount);
      Serial.print(", ");

      // Reset encoder count
      encoderCount = 0;

      // Output the actual speed
      Serial.print("Wheel Speed: ");
      Serial.print(wheelSpeed);
      Serial.println(" m/s");
    }
  }

  // Move the motor with a given speed
  void moveMotor(float speed) {
    int pwmSpeed = map(speed * 1000, 0, MAX_SPEED * 1000, 0, MAX_PWM);
    pwmSpeed = constrain(pwmSpeed, -MAX_PWM, MAX_PWM);

    analogWrite(motorPinPWM, abs(pwmSpeed));

    if (pwmSpeed >= 0) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    } else {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
    }
  }

private:
  // Encoder interrupt service routine
  static void updateEncoder() {
    // This is a static method, but it has access to the MotorEncoderController's member variables
    if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }
};

MotorEncoderController leftMotorEncoder(leftEncoderPinA, leftEncoderPinB, leftMotorPin1, leftMotorPin2, leftMotorPinPWM, interval);
MotorEncoderController rightMotorEncoder(rightEncoderPinA, rightEncoderPinB, rightMotorPin1, rightMotorPin2, rightMotorPinPWM, interval);

void setup() {
  leftMotorEncoder.init();
  rightMotorEncoder.init();
}

void loop() {
  leftMotorEncoder.update();
  rightMotorEncoder.update();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "w") {
      leftMotorEncoder.moveMotor(0.5);
      rightMotorEncoder.moveMotor(0.5);
    } else if (command == "s") {
      leftMotorEncoder.moveMotor(0.0);
      rightMotorEncoder.moveMotor(0.0);
    } else if (command == "a") {
      leftMotorEncoder.moveMotor(0.5);
      rightMotorEncoder.moveMotor(-0.5);
    } else if (command == "d") {
      leftMotorEncoder.moveMotor(-0.5);
      rightMotorEncoder.moveMotor(0.5);
    } else if (command == "x") {
      leftMotorEncoder.moveMotor(-0.5);
      rightMotorEncoder.moveMotor(-0.5);
    }
  }
}
