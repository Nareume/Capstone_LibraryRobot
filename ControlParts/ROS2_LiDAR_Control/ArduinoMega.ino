#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

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

ros::NodeHandle nh;

std_msgs::Float32 motor_deg_msg;
ros::Publisher motor_deg_pub("motor_deg", &motor_deg_msg);

std_msgs::Int16 motor_vel_msg;
ros::Publisher motor_vel_pub("motor_vel", &motor_vel_msg);

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

  nh.initNode();
  nh.advertise(motor_deg_pub);
  nh.advertise(motor_vel_pub);
}

void loop() {
  float motorDeg = float(encoderPos) * ratio;

  float error = targetDeg - motorDeg;
  float control = Kp * error;

  int motorVel = min(abs(control), 255);
  
  doMotor((control >= 0), motorVel, motor1DirPin1, motor1DirPin2, motor1PWMPin);
  doMotor((control >= 0), motorVel, motor2DirPin1, motor2DirPin2, motor2PWMPin);

  motor_deg_msg.data = motorDeg;
  motor_deg_msg.data = motorDeg;
  motor_deg_pub.publish(&motor_deg_msg);

  motor_vel_msg.data = motorVel;
  motor_vel_pub.publish(&motor_vel_msg);

  nh.spinOnce();
  delay(10);
}