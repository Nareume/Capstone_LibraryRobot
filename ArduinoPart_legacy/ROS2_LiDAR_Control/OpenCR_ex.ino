#include <ros2arduino.h>
#include <std_msgs/Float32.hpp>
#include <std_msgs/Int16.hpp>
#include <PID_v1.h>

#define LED_BUILTIN 13
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

// motor control pins
const int motor1DirPin1 = 8; // L298 IN1
const int motor1DirPin2 = 9; // L298 IN2
const int motor1PWMPin = 10; // L298 ENA

const int motor2DirPin1 = 5; // L298 IN3
const int motor2DirPin2 = 6; // L298 IN4
const int motor2PWMPin = 7; // L298 ENB

// PID parameters
double Setpoint, Input, Output;
double Kp=30, Ki=5, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int encoderPos = 0;
const float ratio = 360.0 / (26.0 * 27.0); 

std_msgs::Float32 motor_deg_msg;
ros2::Publisher motor_deg_pub("motor_deg", motor_deg_msg);

std_msgs::Int16 motor_vel_msg;
ros2::Publisher motor_vel_pub("motor_vel", motor_vel_msg);

void messageCb( const std_msgs::Float32& setpoint_msg){
  Setpoint = setpoint_msg.data;  // Use the incoming message as the setpoint
}

ros2::Subscription<std_msgs::Float32> sub("turtlebot3_input", messageCb);

void doEncoderA() {
  encoderPos += (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) ? 1 : -1;
}

void doEncoderB() {
  encoderPos += (digitalRead(ENCODER_PIN_A) == digitalRead(ENCODER_PIN_B)) ? -1 : 1;
}

void doMotor(bool dir, int vel, int dirPin1, int dirPin2, int pwmPin) {
  digitalWrite(dirPin1, dir);
  digitalWrite(dirPin2, !dir);
  analogWrite(pwmPin, vel);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), doEncoderB, CHANGE);

  pinMode(motor1DirPin1, OUTPUT);
  pinMode(motor1DirPin2, OUTPUT);
  pinMode(motor1PWMPin, OUTPUT);
  pinMode(motor2DirPin1, OUTPUT);
  pinMode(motor2DirPin2, OUTPUT);
  pinMode(motor2PWMPin, OUTPUT);

  if(!nh.initNode()){
    while(1){
      // Add Failure Routine
    }
  }
  if(!nh.createSubscriber(sub)){
    while(1){
      // Add Failure Routine
    }
  }
  
  if(!nh.createPublisher(motor_deg_pub)){
    while(1){
      // Add Failure Routine
    }
  }
  
  if(!nh.createPublisher(motor_vel_pub)){
    while(1){
      // Add Failure Routine
    }
  }

  // Initialize the PID
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  digitalWrite(LED_BUILTIN, nh.connected() ? LOW : HIGH); // LED is on when the connection is successful
  nh.spinOnce();

  Input = float(encoderPos) * ratio;

  myPID.Compute();

  int motorVel = min(abs(Output), 255.0);

  doMotor((Output >= 0), motorVel, motor1DirPin1, motor1DirPin2, motor1PWMPin);
  doMotor((Output >= 0), motorVel, motor2DirPin1, motor2DirPin2, motor2PWMPin);

  motor_deg_msg.data = Input;
  motor_deg_pub.publish(&motor_deg_msg);

  motor_vel_msg.data = motorVel;
  motor_vel_pub.publish(&motor_vel_msg);

  delay(10);
}
