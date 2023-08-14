// define pin values
#define encoder0PinA 4
#define encoder0PinB 3
#define encoder1PinA 9
#define encoder1PinB 8
#define leftMotorPin1 11
#define leftMotorPin2 10
#define rightMotorPin1 6
#define rightMotorPin2 5
#define leftMotorPinPWM 12
#define rightMotorPinPWM 7

// define absolute values which we use
#define P_GAIN 30.0
#define I_GAIN 20.0
#define D_GAIN 10.0

//values which related by cmd_topic
const double TARGET_SPEED = 0.7; //m/s
const double TARGET_ANGLESPEED =  90/180/3.14; //radian

// define robot specification values
const double gearboxRatio = 27.0;
const double encoderPulsesPerRevolution = 26.0;
const double wheelDiameter = 0.13;  // meter
const double wheelbase = 0.30;      // meter
const double wheelCircumference = wheelDiameter * PI;
const double distancePerPulse = wheelCircumference / (encoderPulsesPerRevolution * gearboxRatio);
float speedDifference = TARGET_ANGLESPEED * wheelbase;  // 바퀴의 속도 차이 계산

// 각 바퀴의 목표 속도 설정
float targetSpeed0 = TARGET_SPEED + speedDifference / 2;
float targetSpeed1 = TARGET_SPEED - speedDifference / 2;

// define related values in code
float currentSpeed0 = 0.00;
float currentSpeed1 = 0.00;
float targetSpeed = 0.00;
float error0 = 0.00;
float accError0 = 0.00;
float errorGap0 = 0.00;
float error1 = 0.00;
float accError1 = 0.00;
float errorGap1 = 0.00;
float motorSpeed = 0.00;
volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;
volatile long encoder0PosLast = 0;
volatile long encoder1PosLast = 0;
unsigned long lastUpdateTime = 0;

// setup function
void setup() {
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder0PinA), updateEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), updateEncoder1, CHANGE);

  Serial.begin(115200);

}

// loop function
void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdateTime) / 1000.0;  // Convert to seconds

  if (currentTime - lastUpdateTime >= 20) {
    currentSpeed0 = (encoder0Pos - encoder0PosLast) * distancePerPulse * 50;  // 50ms 동안의 이동거리를 초당 속도로 변환
    currentSpeed1 = (encoder1Pos - encoder1PosLast) * distancePerPulse * 50;
    
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

  Serial.print(TARGET_SPEED);
  Serial.print(",");
  Serial.print(TARGET_ANGLESPEED);
  Serial.print(",");
  Serial.print(currentSpeed0);
  Serial.print(",");
  Serial.print(currentSpeed1);
  Serial.print(",");
  Serial.print(error0);
  Serial.print(",");
  Serial.print(error1);
  Serial.print(",");
  Serial.print(pidControl0);
//  Serial.print(",");
//  Serial.print(pidControl1);
  Serial.print(",");
  Serial.print(constrain(abs(pidControl0), 0, 255));
  Serial.print(",");
  Serial.println(constrain(abs(pidControl1), 0, 255));
}

// 2체배 (left)
void updateEncoder0() {
  if (digitalRead(encoder0PinB) == digitalRead(encoder0PinA)) {
    encoder0Pos--;
  } else {
    encoder0Pos++;
  }
}

// 2체배 (right)
void updateEncoder1() {
  if (digitalRead(encoder1PinB) == digitalRead(encoder1PinA)) {
    encoder1Pos--;
  } else {
    encoder1Pos++;
  }
}

// Calculate Error in function
void calculateError(float* error, float* accError, float* errorGap, float currentSpeed, float targetSpeed) {
  *errorGap = targetSpeed - currentSpeed - *error;
  *error = targetSpeed - currentSpeed;
  *accError += *error;
}

// PID Control Function
float pidControlSystem(float error, float accError, float errorGap, float dt) {
  float pControl = P_GAIN * error;
  float iControl = I_GAIN * (accError);
  float dControl = D_GAIN * (errorGap / dt);
  return pControl + iControl + dControl;
}

// Moving Motor by PWM constant which is regulated 
void moveMotor(int motorPinPWM, int motorPin1, int motorPin2, float pidControl) {
  motorSpeed = constrain(abs(pidControl), 0, 255);
  analogWrite(motorPinPWM, motorSpeed);

  if (pidControl >= 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
}
