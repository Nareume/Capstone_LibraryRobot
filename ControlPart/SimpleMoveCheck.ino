'''
엔코더 모터 작동 정상 여부 확인 코드
'''

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
const unsigned long interval = 100;  // Interval in milliseconds


// Robot specifications
const double wheelDiameter = 0.13;
const double wheelbase = 0.30;
const double wheelCircumference = wheelDiameter * PI;
const double encoderPulsesPerRevolution = 741.0; 
const double distancePerPulse = wheelCircumference / encoderPulsesPerRevolution; 

float targetSpeedLinear = 0.0;
float targetSpeedAngular = 0.0;
float linearSpeed = 0.0;
float angularSpeed = 0.0;

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

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Calculate wheel speeds
    double leftWheelSpeed = (leftEncoderCount * distancePerPulse) / (interval / 1000.0);
    double rightWheelSpeed = (rightEncoderCount * distancePerPulse) / (interval / 1000.0);
    Serial.print("leftEncoderCount: ");
    Serial.print(leftEncoderCount);
    Serial.print(", ");
    Serial.print("rightEncoderCount: ");
    Serial.print(rightEncoderCount);
    Serial.print(", ");
    
    // Reset encoder counts
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // Calculate linear and angular speed
    linearSpeed = (leftWheelSpeed + rightWheelSpeed) / 2.0;
    angularSpeed = (rightWheelSpeed - leftWheelSpeed) / wheelbase;

    serActualSpeeds();
  }
  
  if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "w") {
      targetSpeedLinear += 0.5;
      targetSpeedAngular = 0.0;
      Serial.println("FORWARD");
      serTargetSpeeds();
      
    } else if (command == "s") {
      targetSpeedLinear = 0.0;
      targetSpeedAngular = 0.0;
      Serial.println("STOP");
      serTargetSpeeds();
      
    } else if (command == "a") {
      targetSpeedLinear = 0.0;
      targetSpeedAngular += 0.5;
      Serial.println("LEFT");
      serTargetSpeeds();
      
    } else if (command == "d") {
      targetSpeedLinear = 0.0;
      targetSpeedAngular -= 0.5;
      Serial.println("RIGHT");
      serTargetSpeeds();
      
    } else if (command == "x") {
      targetSpeedLinear -= 0.5;
      targetSpeedAngular = 0.0;
      Serial.println("BACK");
      serTargetSpeeds();

    }  
  }


  
  float rightWheelSpeed = targetSpeedLinear + targetSpeedAngular * wheelbase / 2;
  float leftWheelSpeed = targetSpeedLinear - targetSpeedAngular * wheelbase / 2;

  moveMotorSimple(leftMotorPinPWM, leftMotorPin1, leftMotorPin2, leftWheelSpeed);
  moveMotorSimple(rightMotorPinPWM, rightMotorPin1, rightMotorPin2, rightWheelSpeed);
}

void leftEncoderISR() {
  leftEncoderCount++;
}

void rightEncoderISR() {
  rightEncoderCount++;
}

void moveMotorSimple(int motorPinPWM, int motorPin1, int motorPin2, float speed) {
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
 // Output the target speeds
  Serial.print("Target Linear Speed: ");
  Serial.print(targetSpeedLinear);
  Serial.print(" m/s, Target Angular Speed: ");
  Serial.print(targetSpeedAngular);
  Serial.println(" rad/s");
}

void serActualSpeeds() {
     // Output the actual speeds
  Serial.print("Linear Speed: ");
  Serial.print(linearSpeed);
  Serial.print(" m/s, Angular Speed: ");
  Serial.print(angularSpeed);
  Serial.println(" rad/s");
}