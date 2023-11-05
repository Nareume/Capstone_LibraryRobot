#include <Arduino.h>
#include <PID_v1.h>

// Motor and Encoder Pins
#define leftEncoderPinA 4
#define leftEncoderPinB 3
#define leftMotorPin1 6
#define leftMotorPin2 5
#define leftMotorPinPWM 7

#define rightEncoderPinA 9
#define rightEncoderPinB 8
#define rightMotorPin1 11
#define rightMotorPin2 10
#define rightMotorPinPWM 12

#define MAX_SPEED 1.5 // Max speed in m/s
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

// PID variables for linear and angular speed
double SetpointLinear = 0.0, InputLinear = 0.0, OutputLinear = 0.0;
double SetpointAngular = 0.0, InputAngular = 0.0, OutputAngular = 0.0;

// PID objects
PID PIDLinear(&InputLinear, &OutputLinear, &SetpointLinear, 650.0, 20.0, 0.000001, DIRECT);
PID PIDAngular(&InputAngular, &OutputAngular, &SetpointAngular, 50.0, 5.0, 0.1, DIRECT);

void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);

  // Initialize Serial
  Serial.begin(115200);
  
  // Start the PID controllers
  PIDLinear.SetMode(AUTOMATIC);
  PIDAngular.SetMode(AUTOMATIC);

  // Set output limits for the PID controllers
  PIDLinear.SetOutputLimits(-MAX_PWM, MAX_PWM);
  PIDAngular.SetOutputLimits(-MAX_PWM, MAX_PWM);
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

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    double leftWheelSpeed = (leftEncoderCount * distancePerPulse) / (interval / 1000.0);
    double rightWheelSpeed = (rightEncoderCount * distancePerPulse) / (interval / 1000.0);

    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // Update PID inputs
    InputLinear = (leftWheelSpeed + rightWheelSpeed) / 2.0;
    InputAngular = (rightWheelSpeed - leftWheelSpeed) / wheelbase;

     // Read serial input to change speeds
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
  
      // Parse command and update setpoints
      if (command == "w") {
        SetpointLinear += 0.1;
        SetpointAngular = 0.0;
        Serial.println("FORWARD");
      } 
      else if (command == "x") {
        SetpointLinear -= 0.1;
        SetpointAngular = 0.0;
        Serial.println("STOP");
      }
      else if (command == "s") {
        SetpointLinear = 0.0;
        SetpointAngular = 0.0;
        Serial.println("STOP");
      } else if (command == "a") {
        SetpointLinear = 0.0;
        SetpointAngular -= 0.1;
        Serial.println("TURN LEFT");
      } else if (command == "d") {
        SetpointLinear = 0.0;
        SetpointAngular += 0.1;
        Serial.println("TURN RIGHT");
      }
    }

    // Compute PID
    PIDLinear.Compute();
    PIDAngular.Compute();

    // Apply PID outputs to motors
    moveMotorPID(leftMotorPinPWM, leftMotorPin1, leftMotorPin2, OutputLinear - OutputAngular);
    moveMotorPID(rightMotorPinPWM, rightMotorPin1, rightMotorPin2, OutputLinear + OutputAngular);

    // Send speeds to serial
    serTargetSpeeds();
    serActualSpeeds();
  }
  
}

void moveMotorPID(int motorPWM, int motorPin1, int motorPin2, double speed) {
  int direction = speed > 0 ? HIGH : LOW;
  speed = abs(speed);
  speed = min(speed, (double)MAX_PWM);

  analogWrite(motorPWM, (int)speed);
  digitalWrite(motorPin1, direction);
  digitalWrite(motorPin2, !direction);
}

void serTargetSpeeds() {
  //Serial.print("Target Linear: ");
  Serial.print(SetpointLinear);
  //Serial.print(" m/s | Target Angular: ");
  Serial.print(",");
  Serial.print(SetpointAngular);
  Serial.print(",");
  //Serial.println(" rad/s");
}

void serActualSpeeds() {
  //Serial.print("Actual Linear: ");
  Serial.print(InputLinear);
  Serial.print(",");
  //Serial.print(" m/s | Actual Angular: ");
  Serial.println(InputAngular);
  //Serial.println(" rad/s");
}
