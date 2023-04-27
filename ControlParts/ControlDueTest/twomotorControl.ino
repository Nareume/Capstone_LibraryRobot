// motor control pins
const int motor1DirPin1 = 8; // L298 IN1
const int motor1DirPin2 = 9; // L298 IN2
const int motor1PWMPin = 10; // L298 ENA

const int motor2DirPin1 = 5; // L298 IN3
const int motor2DirPin2 = 6; // L298 IN4
const int motor2PWMPin = 7; // L298 ENB

// encoder pins
const int encoderPinA = 2;
const int encoderPinB = 3;

int encoderPos = 0;
const float ratio = 360.0 / (26.0 * 27.0);

// P control
float Kp = 30;
float targetDeg = 360;

void doEncoderA() {
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;
}

void doEncoderB() {
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? -1 : 1;
}

void doMotor(bool dir, int vel, int dirPin1, int dirPin2, int pwmPin) {
  digitalWrite(dirPin1, dir);
  digitalWrite(dirPin2, !dir);
  analogWrite(pwmPin, vel);
}

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(0, doEncoderA, CHANGE);

  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(1, doEncoderB, CHANGE);

  pinMode(motor1DirPin1, OUTPUT);
  pinMode(motor1DirPin2, OUTPUT);
  pinMode(motor1PWMPin, OUTPUT);
  pinMode(motor2DirPin1, OUTPUT);
  pinMode(motor2DirPin2, OUTPUT);
  pinMode(motor2PWMPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  float motorDeg = float(encoderPos) * ratio;

  float error = targetDeg - motorDeg;
  float control = Kp * error;

  int motorVel = min(abs(control), 255);
  
  doMotor((control >= 0), motorVel, motor1DirPin1, motor1DirPin2, motor1PWMPin);
  doMotor((control >= 0), motorVel, motor2DirPin1, motor2DirPin2, motor2PWMPin);

  Serial.print("encoderPos : ");
  Serial.print(encoderPos);
  Serial.print("   motorDeg : ");
  Serial.print(float(encoderPos) * ratio);
  Serial.print("   error : ");
  Serial.print(error);
  Serial.print("    control : ");
  Serial.print(control);
  Serial.print("    motorVel : ");
  Serial.println(motorVel);
}
