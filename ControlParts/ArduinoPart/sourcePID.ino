#include <PID_v1.h>

#define leftEncoderPinA 3
#define leftEncoderPinB 4
#define rightEncoderPinA 8
#define rightEncoderPinB 9
#define leftMotorPin1 5
#define leftMotorPin2 6
#define rightMotorPin1 10
#define rightMotorPin2 11

double Setpoint, Input, Output;
double Kp=2, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

volatile int leftEncoderPos = 0;
volatile int rightEncoderPos = 0;

void setup() {
  pinMode(leftEncoderPinA, INPUT);
  pinMode(leftEncoderPinB, INPUT);
  pinMode(rightEncoderPinA, INPUT);
  pinMode(rightEncoderPinB, INPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(leftEncoderPinB), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinB), updateRightEncoder, CHANGE);
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10);
  
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    Setpoint = Serial.parseInt(); //get the target speed from serial monitor
  }

  Input = (leftEncoderPos + rightEncoderPos) / 2.0; // take the average encoder value

  myPID.Compute();

  if(Output > 0){
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  
  analogWrite(leftMotorPin1, abs(Output));
  analogWrite(rightMotorPin1, abs(Output));
  
  Serial.print("Position: "); // Print the encoder value on the serial monitor
  Serial.println((leftEncoderPos + rightEncoderPos) / 2.0);

  delay(10);
}

void updateLeftEncoder() {
  int MSB = digitalRead(leftEncoderPinA); //MSB = most significant bit
  int LSB = digitalRead(leftEncoderPinB); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (leftEncoderPos << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) leftEncoderPos ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) leftEncoderPos --;
}

void updateRightEncoder() {
  int MSB = digitalRead(rightEncoderPinA); //MSB = most significant bit
  int LSB = digitalRead(rightEncoderPinB); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (rightEncoderPos << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) rightEncoderPos ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) rightEncoderPos --;
}