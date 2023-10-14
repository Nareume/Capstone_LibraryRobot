#include <Arduino.h>

// Motor and Encoder Pins
#define leftEncoderPinA 3
#define leftEncoderPinB 4
#define leftMotorPin1 5
#define leftMotorPin2 6
#define leftMotorPinPWM 7

#define rightEncoderPinA 8
#define rightEncoderPinB 9
#define rightMotorPin1 10
#define rightMotorPin2 11
#define rightMotorPinPWM 12

#define MAX_SPEED 1.0 // Max speed in m/s
#define MAX_PWM 255 // Max PWM value

// Global variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 10;  // Interval in milliseconds

// Robot specifications
const double wheelDiameter = 0.13;
const double wheelbase = 0.30;
const double wheelCircumference = wheelDiameter * PI;
const double encoderPulsesPerRevolution = 741.0; 
const double distancePerPulse = wheelCircumference / encoderPulsesPerRevolution; 

// PID Global variables
float targetSpeedLinear = 0.0;
float targetSpeedAngular = 0.0;
float linearSpeed = 0.0;
float angularSpeed = 0.0;

// PID Constants and Variables
float Kp_linear = 1.5;
float Ki_linear = 0.05;
float Kd_linear = 0.000002;

float Kp_angular = 0.000001;
float Ki_angular = 0.000001;
float Kd_angular = 0.000001;

float errorLinear = 0.0;
float previousErrorLinear = 0.0;
float integralLinear = 0.0;
float derivativeLinear = 0.0;

float errorAngular = 0.0;
float previousErrorAngular = 0.0;
float integralAngular = 0.0;
float derivativeAngular = 0.0;

void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);

  Serial.begin(115200);
}

void updateLeftEncoder() {
  if (digitalRead(leftEncoderPinA) == digitalRead(leftEncoderPinB)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void updateRightEncoder() {
  if (digitalRead(rightEncoderPinA) == digitalRead(rightEncoderPinB)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

float PIDControl(float target, float current, float &previousError, float &integral, float Kp, float Ki, float Kd) {
    float error = target - current;
    integral += error;
    float derivative = error - previousError;
    float output = Kp * error + Ki * integral + Kd * derivative;
    previousError = error;
    return output;
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    double leftWheelSpeed = (leftEncoderCount * distancePerPulse) / (interval / 1000.0);
    double rightWheelSpeed = (rightEncoderCount * distancePerPulse) / (interval / 1000.0);

    leftEncoderCount = 0;
    rightEncoderCount = 0;

    linearSpeed = (leftWheelSpeed + rightWheelSpeed) / 2.0;
    angularSpeed = (rightWheelSpeed - leftWheelSpeed) / wheelbase;
    serTargetSpeeds();
    serActualSpeeds();
  }
  
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "w") {
      targetSpeedLinear += 0.1;
      targetSpeedAngular = 0.0;
      Serial.println("FORWARD");
    } else if (command == "s") {
      targetSpeedLinear = 0.0;
      targetSpeedAngular = 0.0;
      Serial.println("STOP");
    } else if (command == "a") {
      targetSpeedLinear = 0.0;
      targetSpeedAngular += 0.1;
      Serial.println("LEFT");
    } else if (command == "d") {
      targetSpeedLinear = 0.0;
      targetSpeedAngular -= 0.1;
      Serial.println("RIGHT");
    } else if (command == "x") {
      targetSpeedLinear -= 0.1;
      targetSpeedAngular = 0.0;
      Serial.println("BACK");
    }  
  }

  float linearOutput = PIDControl(targetSpeedLinear, linearSpeed, previousErrorLinear, integralLinear, Kp_linear, Ki_linear, Kd_linear);
  float angularOutput = PIDControl(targetSpeedAngular, angularSpeed, previousErrorAngular, integralAngular, Kp_angular, Ki_angular, Kd_angular);

  float rightWheelSpeed = linearOutput + angularOutput;
  float leftWheelSpeed = linearOutput - angularOutput;

  moveMotorPID(leftMotorPinPWM, leftMotorPin1, leftMotorPin2, leftWheelSpeed);
  moveMotorPID(rightMotorPinPWM, rightMotorPin1, rightMotorPin2, rightWheelSpeed);
}

void moveMotorPID(int motorPinPWM, int motorPin1, int motorPin2, float speed) {
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


void serTargetSpeeds() {
  //Serial.print("Target Linear Speed: ");
  Serial.print(targetSpeedLinear);
  Serial.print(",");
  Serial.print(targetSpeedAngular);
}


void serActualSpeeds() {
  Serial.print(",");
  Serial.print(linearSpeed);
  Serial.print(",");
  Serial.println(angularSpeed);
}




void calc_quat(float theta, float &qx, float &qz) {
    float cos_half_theta = cos(theta / 2.0);
    float sin_half_theta = sin(theta / 2.0);

    qx = cos_half_theta;
    qz = sin_half_theta;
}
