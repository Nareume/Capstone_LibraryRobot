#define encoder0PinA 3
#define encoder0PinB 4
#define encoder1PinA 9
#define encoder1PinB 8

#define leftMotorPin1 11
#define leftMotorPin2 10
#define rightMotorPin1 6
#define rightMotorPin2 5
#define leftMotorPinPWM 12
#define rightMotorPinPWM 7

#define P_GAIN 8
#define I_GAIN 0.0
#define D_GAIN 0.0

#define TARGET_SPEED 1.0
#define ConstantPulse 1725 //1m->pulse

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

float currentSpeed0 = 0.00;
float currentSpeed1 = 0.00;

float targetSpeed = 0.00;

float error0 = 0.00;
float accError0 = 0.00;
float errorGap0 = 0.00;

float error1 = 0.00;
float accError1 = 0.00;
float errorGap1 = 0.00;

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

volatile long encoder0PosLast = 0;
volatile long encoder1PosLast = 0;

unsigned long lastUpdateTime = 0;

void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), updateEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), updateEncoder1, CHANGE);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();

  Serial.begin(115200);
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds
  
  if (currentTime - lastUpdateTime >= 100) {
    currentSpeed0 = (encoder0Pos - encoder0PosLast) * 10;
    currentSpeed1 = (encoder1Pos - encoder1PosLast) * 10;

    encoder0PosLast = encoder0Pos;
    encoder1PosLast = encoder1Pos;
    lastUpdateTime = currentTime;
  }

  calculateError(&error0, &accError0, &errorGap0, currentSpeed0, TARGET_SPEED);
  float pidControl0 = pidControlSystem(error0, accError0, errorGap0, dt);
  moveMotor(leftMotorPinPWM, leftMotorPin1, leftMotorPin2, pidControl0);

  calculateError(&error1, &accError1, &errorGap1, currentSpeed1, TARGET_SPEED);
  float pidControl1 = pidControlSystem(error1, accError1, errorGap1, dt);
  moveMotor(rightMotorPinPWM, rightMotorPin1, rightMotorPin2, pidControl1);

  Serial.print(targetSpeed);
  Serial.print("\t");
  Serial.print(currentSpeed0);
  Serial.print("\t");
  Serial.print(currentSpeed1);
  Serial.print("\t");
  Serial.println(pidControl0);
}

void updateEncoder0() {
  if (digitalRead(encoder0PinB) == digitalRead(encoder0PinA)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

void updateEncoder1() {
  if (digitalRead(encoder1PinB) == digitalRead(encoder1PinA)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}

void calculateError(float* error, float* accError, float* errorGap, float currentSpeed, float targetSpeedms) {
  targetSpeed = targetSpeedms * ConstantPulse;
  *errorGap = targetSpeed - currentSpeed - *error;
  *error = targetSpeed - currentSpeed;
  *accError += *error;
}

float pidControlSystem(float error, float accError, float errorGap, float dt) {
  float pControl = P_GAIN * error;
  float iControl = I_GAIN * accError * dt;
  float dControl = D_GAIN * errorGap / dt;
  return pControl + iControl + dControl;
}

void moveMotor(int motorPinPWM, int motorPin1, int motorPin2, float pidControl) {
  int motorSpeed = constrain(abs(pidControl), 0, 255);
  analogWrite(motorPinPWM, motorSpeed);

  if (pidControl >= 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
}

