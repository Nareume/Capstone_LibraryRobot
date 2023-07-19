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
const double encoderPulsesPerRevolution = 26.0;
const double wheelDiameter = 13.0; // cm
const double wheelbase = 30.0; // cm
const double wheelCircumference = wheelDiameter * PI;
const double distancePerPulse = wheelCircumference / (encoderPulsesPerRevolution * gearboxRatio); // cm/pulse



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
        speed = updateTurnSpeed();
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, speed);
        delay(2000);
        break;
      case 'R': // Turn right
        speed = updateTurnSpeed();
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, speed);
        delay(2000);
        break;
      case 'S':
        // Stop both motors
        moveMotor(leftMotorPin1, leftMotorPin2, leftMotorPinPWM, 0);
        moveMotor(rightMotorPin1, rightMotorPin2, rightMotorPinPWM, 0);
        delay(250);

    }

  }
}



void moveMotor(int motorPin1, int motorPin2, int motorPinPWM, int speed) {
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
