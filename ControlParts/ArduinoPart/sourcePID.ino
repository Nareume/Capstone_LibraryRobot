#include <math.h>

#define PI 3.14159265

// Motor Pin Configuration
#define leftMotorPin1 11
#define leftMotorPin2 10
#define rightMotorPin1 6
#define rightMotorPin2 5
#define leftMotorPinPWM 12  // Left Motor PWM Pin
#define rightMotorPinPWM 7  // Right Motor PWM Pin

// Encoder Pin Configuration
#define leftEncoderPinA 8
#define leftEncoderPinB 9
#define rightEncoderPinA 3
#define rightEncoderPinB 4

// PID Constants
#define Kp 1.0 // Proportional control gain
#define Ki 0.5 // Integral control gain
#define Kd 0.1 // Derivative control gain

volatile int leftEncoderPos = 0;
volatile int rightEncoderPos = 0;
int prevLeftEncoderPos = 0;
int prevRightEncoderPos = 0;

// Robot specifications
const double gearboxRatio = 27.0;
const double encoderPulsesPerRevolution = 26.0;
const double wheelDiameter = 13.0; // cm
const double wheelbase = 30.0; // cm
const double wheelCircumference = wheelDiameter * PI;
const double distancePerPulse = wheelCircumference / (encoderPulsesPerRevolution * gearboxRatio); // cm/pulse

// PID Variables
double leftSpeedSetpoint = 0.0; // Desired left wheel speed
double rightSpeedSetpoint = 0.0; // Desired right wheel speed
double leftSpeedError = 0.0; // Error in left wheel speed
double rightSpeedError = 0.0; // Error in right wheel speed
double leftSpeedErrorSum = 0.0; // Sum of left wheel speed errors
double rightSpeedErrorSum = 0.0; // Sum of right wheel speed errors
double prevLeftSpeedError = 0.0; // Previous error in left wheel speed
double prevRightSpeedError = 0.0; // Previous error in right wheel speed

// Robot's position and orientation (initially 0)
double xPosition = 0.0;
double yPosition = 0.0;
double theta = 0.0;

void setup() {
  Serial.begin(9600);

  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPinPWM, OUTPUT);
  pinMode(rightMotorPinPWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinB), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinB), updateRightEncoder, CHANGE);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch(command) {
      case 'F': // Forward
        leftSpeedSetpoint = rightSpeedSetpoint = 200; // Desired speed for forward
        break;
      case 'B': // Backward
        leftSpeedSetpoint = rightSpeedSetpoint = -200; // Desired speed for backward
        break;
      case 'L': // Turn left
        leftSpeedSetpoint = -100; // Desired speed for left turn
        rightSpeedSetpoint = 100; // Desired speed for left turn
        break;
      case 'R': // Turn right
        leftSpeedSetpoint = 100; // Desired speed for right turn
        rightSpeedSetpoint = -100; // Desired speed for right turn
        break;
    }
  }

  // Compute desired speed control values using PID
  double leftControl = computePIDControl(leftSpeedSetpoint, leftSpeedError, leftSpeedErrorSum, prevLeftSpeedError);
  double rightControl = computePIDControl(rightSpeedSetpoint, rightSpeedError, rightSpeedErrorSum, prevRightSpeedError);

  // Use control values to set motor speeds
  moveMotor(leftMotorPin1, leftMotorPin2, leftControl >= 0, leftControl < 0, leftMotorPinPWM, abs(leftControl));
  moveMotor(rightMotorPin1, rightMotorPin2, rightControl >= 0, rightControl < 0, rightMotorPinPWM, abs(rightControl));

  // Compute odometry after performing motor actions
  computeOdometry();

  // Send position data over Serial
  Serial.print("X: ");
  Serial.print(xPosition);
  Serial.print(" Y: ");
  Serial.println(yPosition);
}

void moveMotor(int motorPin1, int motorPin2, int pin1State, int pin2State, int motorPinPWM, int speed) {
  digitalWrite(motorPin1, pin1State);
  digitalWrite(motorPin2, pin2State);
  analogWrite(motorPinPWM, speed);
}

void updateLeftEncoder() {
  updateEncoder(leftEncoderPinA, leftEncoderPinB, leftEncoderPos);
}

void updateRightEncoder() {
  updateEncoder(rightEncoderPinA, rightEncoderPinB, rightEncoderPos);
}

void updateEncoder(int encoderPinA, int encoderPinB, volatile int &encoderPos) {
  int MSB = digitalRead(encoderPinA);
  int LSB = digitalRead(encoderPinB);

  int encoded = (MSB << 1) | LSB;
  int sum = (encoderPos << 2) | encoded;

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos--;
}

void computeOdometry() {
  // Compute the distance each wheel has moved
  double leftDistance = (leftEncoderPos - prevLeftEncoderPos) * distancePerPulse;
  double rightDistance = (rightEncoderPos - prevRightEncoderPos) * distancePerPulse;
  
  // Compute the change in position and angle
  double distanceTravelled = (leftDistance + rightDistance) / 2;
  double deltaTheta = (rightDistance - leftDistance) / wheelbase;
  theta += deltaTheta;
  
  // Compute change in position
  double deltaX = distanceTravelled * cos(theta);
  double deltaY = distanceTravelled * sin(theta);
  
  // Update robot position
  xPosition += deltaX;
  yPosition += deltaY;
  
  // Update previous encoder positions
  prevLeftEncoderPos = leftEncoderPos;
  prevRightEncoderPos = rightEncoderPos;
}

double computePIDControl(double setpoint, double &error, double &errorSum, double &prevError) {
  // Compute error
  double currentError = setpoint - error;

  // Compute integral and derivative terms
  double integralTerm = errorSum;
  double derivativeTerm = currentError - prevError;

  // Compute control value
  double control = Kp * currentError + Ki * integralTerm + Kd * derivativeTerm;

  // Update error values
  errorSum += currentError;
  prevError = currentError;

  return control;
}
