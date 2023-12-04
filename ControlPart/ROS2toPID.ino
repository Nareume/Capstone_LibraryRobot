/*
Mirco-ROS Arduino Node를 활용한 통신 구현 및 PID 제어 코드
*/

#include <Arduino.h>
#include <PID_v1.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>

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

#define LED_PIN 13

#define MAX_SPEED 1.5 // Max speed in m/s
#define MAX_PWM 255 // Max PWM value

// Global variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 100; // Interval in milliseconds

// Robot specifications
const double wheelDiameter = 0.13;
const double wheelbase = 0.30;
const double gearboxRatio = 27.0;
const double encoderResolution = 26.0;
const double wheelCircumference = wheelDiameter * PI;
const double encoderPulsesPerRevolution = 741.0;
const double distancePerPulse = wheelCircumference / encoderPulsesPerRevolution;

// PID variables for left and right motors
double SetpointLeft = 0.0, InputLeft = 0.0, OutputLeft = 0.0;
double SetpointRight = 0.0, InputRight = 0.0, OutputRight = 0.0;
double SetpointLinear = 0.0, SetpointAngular = 0.0, InputLinear = 0.0, InputAngular = 0.0;

float pos_x = 0;
float pos_y = 0;
float angle_z = 0;

float qw = 0;
float qz = 0;

// PID objects for left and right motors
PID PIDLeft(&InputLeft, &OutputLeft, &SetpointLeft, 600.0, 5.0, 1.0,DIRECT);
PID PIDRight(&InputRight, &OutputRight, &SetpointRight, 600.0, 5.0, 1.0, DIRECT);

// Micro-ROS specific definitions
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist t_msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
nav_msgs__msg__Odometry odom;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * t_msg = (const geometry_msgs__msg__Twist *)msgin;
  SetpointLinear = t_msg->linear.x;
  SetpointAngular = t_msg->angular.z;
}

void setup() {
  // Micro-ROS Setup
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  delay(2000);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  
  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "micro_ros_arduino_node_publisher"));

  // Executor and Timer setup
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &t_msg, &subscription_callback, ON_NEW_DATA));// Executor and Timer setup
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(leftMotorPinPWM, OUTPUT);
  pinMode(rightMotorPinPWM, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(leftEncoderPinA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPinA), updateRightEncoder, CHANGE);

  // Initialize Serial
  Serial.begin(115200);

  // Start the PID controllers
  PIDLeft.SetMode(AUTOMATIC);
  PIDRight.SetMode(AUTOMATIC);

  // Set output limits for the PID controllers
  PIDLeft.SetOutputLimits(-MAX_PWM, MAX_PWM);
  PIDRight.SetOutputLimits(-MAX_PWM, MAX_PWM);
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

    // Calculate speed from encoder counts
    double leftWheelSpeed = (leftEncoderCount * distancePerPulse) / (interval / 1000.0);
    double rightWheelSpeed = (rightEncoderCount * distancePerPulse) / (interval / 1000.0);

    // Reset encoder counts
    leftEncoderCount = 0;
    rightEncoderCount = 0;

    // Compute linear and angular setpoints from desired values
    SetpointLeft = SetpointLinear - (SetpointAngular * wheelbase / 2.0);
    SetpointRight = SetpointLinear + (SetpointAngular * wheelbase / 2.0);

    // Update Odometry
    float leftWheelmove = ((wheelCircumference * leftEncoderCount) / encoderResolution) / (interval / 1000.0);
    float rightWheelmove = ((wheelCircumference * leftEncoderCount) / encoderResolution) / (interval / 1000.0);
    
    pos_x += interval * ((leftWheelmove + rightWheelmove) / 2) * cos(angle_z);
    pos_y += interval * ((leftWheelmove + rightWheelmove) / 2) * sin(angle_z);
    angle_z += interval * ((rightWheelmove - leftWheelmove) / wheelbase);

    // Update the PID control inputs
    InputLeft = leftWheelSpeed;
    InputRight = rightWheelSpeed;

    // Compute PID update for each motor
    PIDLeft.Compute();
    PIDRight.Compute();

    // Drive motors using the PID outputs
    moveMotor(leftMotorPinPWM, leftMotorPin1, leftMotorPin2, OutputLeft);
    moveMotor(rightMotorPinPWM, rightMotorPin1, rightMotorPin2, OutputRight);

    // Calculate actual linear and angular speeds based on motor outputs
    InputLinear = (InputLeft + InputRight) / 2.0; // This assumes that Output is proportional to speed
    InputAngular = (InputRight - InputLeft) / wheelbase; // This assumes that wheel separation is tuned to represent the actual turning rate

    // Calculate Quaternion from Current Angle (Yaw)
    calc_quat(-angle_z, qz, qw);

    odom.pose.pose.position.x = -pos_x;
    odom.pose.pose.position.y = -pos_y;
    odom.pose.pose.orientation.w = qw;
    odom.pose.pose.orientation.z = qz;

    // Publish Odometry data
    RCSOFTCHECK(rcl_publish(&publisher, &odom, NULL));
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    
    // For debugging, you might want to print out values to the Serial
    PrintTargetSpeeds();
    PrintActualSpeeds();
  }
}

void moveMotor(int pwmPin, int in1Pin, int in2Pin, double output) {
  if (output > 0) {
    analogWrite(pwmPin, (int)output);
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (output < 0) {
    analogWrite(pwmPin, (int)-output);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void calc_quat(float theta, float &qz, float &qw) {
    float cos_half_theta = cos(theta / 2.0);
    float sin_half_theta = sin(theta / 2.0);

    qw = cos_half_theta;
    qz = sin_half_theta;
}

void PrintTargetSpeeds() {
  Serial.print(SetpointLinear);
  Serial.print(",");
  Serial.print(SetpointAngular);
  Serial.print(",");
}



void PrintActualSpeeds() {
  Serial.print(InputLinear);
  Serial.print(",");
  Serial.println(InputAngular);
}
