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
#define rightEncoderPinA 4
#define rightEncoderPinB 3

// PID Constants
#define Kp 1.0 // Proportional control gain
#define Ki 0.5 // Integral control gain
#define Kd 0.1 // Derivative control gain

volatile int leftEncoderPos = 0;
volatile int rightEncoderPos = 0;

double speed = 180;

// Robot specifications
const double gearboxRatio = 27.0;
const double encoderPulsesPerRevolution = 26.0 * 4;
const double wheelDiameter = 13.0; // cm
const double wheelbase = 30.0; // cm
const double wheelCircumference = wheelDiameter * PI;
const double distancePerPulse = wheelCircumference / (encoderPulsesPerRevolution * gearboxRatio); // cm/pulse

// Define global position and orientation
double xPosition = 0.0;
double yPosition = 0.0;
double theta = 0.0;

double prevLeftEncoderPos = 0.0;
double prevRightEncoderPos = 0.0;


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
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch(command) {
      case 'F': // Forward
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, speed);
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, speed);
        delay(2000); 
        break;
      case 'B': // Backward
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, -speed);
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, -speed);
        delay(2000);
        break;
      case 'L': // Turn left
        turn('L');
        break;
      case 'R': // Turn right
        turn('R');
        break;
      case 'S':
        // Stop both motors
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, 0);
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, 0);
        delay(250);
    }
  }
}

void turn(char direction) {
  double targetTheta = theta + 90.0; // turn 90 degrees
  double error = 0;
  double errorSum = 0;
  double prevError = 0;
  double control;
  
  while(abs(targetTheta - theta) > 1.0) { // change the threshold as needed
    computeOdometry();
    control = computePIDControl(targetTheta, theta, error, errorSum, prevError);
    if(direction == 'L') {
      moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, control);
    } else { // 'R'
      moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, control);
    }
  }

  // stop the motor after the turn
  if(direction == 'L') {
    moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, 0);
  } else { // 'R'
    moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, 0);
  }
}

void moveMotor(int motorPin1, int motorPin2, int motorPinPWM, double speed) {
  digitalWrite(motorPin1, speed >= 0);
  digitalWrite(motorPin2, speed < 0);
  analogWrite(motorPinPWM, abs(speed));
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

double computePIDControl(double setpoint, double currentValue, double &error, double &errorSum, double &prevError) {
  // Compute error
  double currentError = setpoint - currentValue;

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
