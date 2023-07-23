#include <PID_v1.h>

// 모터 핀 설정
const int leftMotorPin1 = 2;
const int leftMotorPin2 = 3;
const int rightMotorPin1 = 4;
const int rightMotorPin2 = 5;

// 엔코더 핀 설정
const int leftEncoderPinA = 18;
const int leftEncoderPinB = 19;
const int rightEncoderPinA = 20;
const int rightEncoderPinB = 21;

// PID 제어 변수
double targetSpeed = 0.0;
double targetPosition = 0.0;
double leftSpeed, rightSpeed;
double leftPosition, rightPosition;
double leftEncoderValue, rightEncoderValue;
double leftOutput, rightOutput;
double leftKp = 1.0, leftKi = 0.0, leftKd = 0.0;
double rightKp = 1.0, rightKi = 0.0, rightKd = 0.0;
PID leftPID(&leftSpeed, &leftOutput, &targetSpeed, leftKp, leftKi, leftKd, DIRECT);
PID rightPID(&rightSpeed, &rightOutput, &targetSpeed, rightKp, rightKi, rightKd, DIRECT);

// 엔코더 모터 사양에 기반한 변수 
double timeInterval = 0.01; // 시간 간격 설정 (측정 주기)
double pulsesPerRevolution = 26.0; // 엔코더의 펄스 수 (해상도)
double wheelRadius = 6.5; // 바퀴 반지름 (단위: cm)
double distancePerPulse = (2 * PI * wheelRadius) / pulsesPerRevolution; // 각 펄스당 이동 거리
double speed = distancePerPulse / timeInterval; // 속도 계산


void setup() {
  // 모터 핀 설정
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  // 엔코더 핀 설정
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);

  // 엔코더 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), rightEncoderISR, CHANGE);

  // PID 제어 설정
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(-255, 255);
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(-255, 255);
}

void loop() {
  // 속도 피드백 제어
  leftSpeed = calculateLeftSpeed();
  rightSpeed = calculateRightSpeed();
  leftPID.Compute();
  rightPID.Compute();
  setMotorSpeeds(leftOutput, rightOutput);

  // 위치 피드백 제어
  leftPosition = calculateLeftPosition();
  rightPosition = calculateRightPosition();
  // 위치 제어 로직 추가

  // 추가 로봇 동작 또는 센서 처리 로직

}


void setMotorSpeeds(double leftSpeed, double rightSpeed) {
  // 왼쪽 모터 속도 설정
  if (leftSpeed >= 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    analogWrite(leftMotorPinPWM, map(leftSpeed, 0, 100, 0, 255));
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
    analogWrite(leftMotorPinPWM, map(abs(leftSpeed), 0, 100, 0, 255));
  }

  // 오른쪽 모터 속도 설정
  if (rightSpeed >= 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
    analogWrite(rightMotorPinPWM, map(rightSpeed, 0, 100, 0, 255));
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
    analogWrite(rightMotorPinPWM, map(abs(rightSpeed), 0, 100, 0, 255));
  }
}


// 왼쪽 모터 속도 계산
double calculateLeftSpeed() { 
  int leftEncoderValue = digitalRead(leftEncoderPinA); // 왼쪽 엔코더 값 읽기
  double leftSpeed = distancePerPulse / timeInterval; // 속도 계산

  return leftSpeed;
}

// 오른쪽 모터 속도 계산
double calculateRightSpeed() {
  int rightEncoderValue = digitalRead(rightEncoderPinA); // 오른쪽 엔코더 값 읽기
  double rightSpeed = distancePerPulse / timeInterval; // 속도 계산

  return rightSpeed;
}

// 왼쪽 모터 위치 계산
double calculateLeftPosition() {
  // 왼쪽 엔코더 값 읽기
  int leftEncoderValue = digitalRead(leftEncoderPinA);
  // 위치 계산
  double degreesPerPulse = 360.0 / pulsesPerRevolution; // 각 펄스당 회전 각도
  double leftPosition = degreesPerPulse * leftEncoderValue; // 위치 계산

  return leftPosition;
}

// 오른쪽 모터 위치 계산
double calculateRightPosition() {
  // 오른쪽 엔코더 값 읽기
  int rightEncoderValue = digitalRead(rightEncoderPinA);
  // 위치 계산
  double degreesPerPulse = 360.0 / pulsesPerRevolution; // 각 펄스당 회전 각도
  double rightPosition = degreesPerPulse * rightEncoderValue; // 위치 계산

  return rightPosition;
}


// 왼쪽 엔코더 인터럽트 처리
void leftEncoderISR() {
  // 인터럽트 처리 로직 추가
}

// 오른쪽 엔코더 인터럽트 처리
void rightEncoderISR() {
  // 인터럽트 처리 로직 추가
}
